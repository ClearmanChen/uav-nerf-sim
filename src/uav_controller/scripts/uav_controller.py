#!/usr/bin/env python3
import rospy
import airsim
import time
import math
# import tf
import os
from airsim.types import CameraInfo, Pose, Quaternionr, Vector3r
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String, Header
from enum import Enum

class Controller_State(Enum):
    STANDBY = 0
    TAKEOFF = 1
    ACTIVE = 2
    LANDING = 3

class UavController:
    def __init__(self, 
    uav_index: int = 1, 
    ros_rate: int = 10,
    airsim_ip: str = '127.0.0.1',
    airsim_port: int = 41451,
    cmd_topic: str = '/planning/pos_cmd_1') -> None:

        self.index = uav_index
        self.state = Controller_State.STANDBY
        self.rate = rospy.Rate(ros_rate)
        rospy.Subscriber(f"/uav_controller_node_uav{uav_index}/trigger", String, self.trigger_callback)
        rospy.Subscriber(cmd_topic, PositionCommand, self.planner_output_callback)
   
        self.airsim_client = airsim.MultirotorClient(ip = airsim_ip, port=airsim_port)
        self.airsim_client.confirmConnection()

        self.vehicle_name = f'uav{uav_index}'
        self.depth_camera_name = f'uav{uav_index}_depth_camera'
        self.scene_camera_name = f'uav{uav_index}_scene_camera'
        self.depth_camera_pose = None
        self.scene_camera_pose = None
        self.depth_pose_pub = rospy.Publisher(f'/airsim_node/uav{uav_index}/uav{uav_index}_depth_camera/pose', PoseStamped, queue_size=10)
        self.scene_pose_pub = rospy.Publisher(f'/airsim_node/uav{uav_index}/uav{uav_index}_scene_camera/pose', PoseStamped, queue_size=10)
        self.depth_pose_seq = 0
        self.scene_pose_seq = 0
        self.airsim_client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.airsim_client.armDisarm(True, vehicle_name=self.vehicle_name)

        while not rospy.has_param(f'/airsim_node/uav{uav_index}/init_x') or \
            not rospy.has_param(f'/airsim_node/uav{uav_index}/init_y') or \
            not rospy.has_param(f'/airsim_node/uav{uav_index}/init_z') :
                self.rate.sleep()
        
        self.init_trans = Vector3r(int(rospy.get_param(f'/airsim_node/uav{uav_index}/init_x')),
            int(rospy.get_param(f'/airsim_node/uav{uav_index}/init_y')),
            int(rospy.get_param(f'/airsim_node/uav{uav_index}/init_z')))
        # tf_listener = tf.TransformListener()
        # try:
        #     (trans, rot) = tf_listener.lookupTransform('world_enu', f'uav{uav_index}', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     trans = [0, 0, 0]
        #     rot = [0, 0, 0, 1]
        # self.init_trans = Vector3r(trans[0], trans[1], trans[2])

    def state_callback(self, data) -> None:
        self.state = data.data

    def planner_output_callback(self, data: PositionCommand):
        # if self.state == Controller_State.ACTIVE:
        self.airsim_client.moveByVelocityAsync(
            data.velocity.x,
            data.velocity.y, 
            data.velocity.z, 
            vehicle_name=self.vehicle_name, 
            duration=0.1
        )
    
    def trigger_callback(self, data: String) -> None:
        print(f'[Uav Controller {self.index}]: Receive command {data.data}.')
        if data.data == 'TAKEOFF':
            if self.state == Controller_State.STANDBY:
                self.state = Controller_State.TAKEOFF
                self.airsim_client.takeoffAsync(vehicle_name=self.vehicle_name).join()
                self.log('Drone took off sucessfully.')
                self.state = Controller_State.ACTIVE
            elif self.state == Controller_State.ACTIVE:
                self.log('Drone has already taken off.')
        elif data.data == 'LAND':
            if self.state == Controller_State.ACTIVE:
                self.state = Controller_State.LANDING
                self.airsim_client.landAsync(vehicle_name=self.vehicle_name).join()
                self.state = Controller_State.STANDBY
                self.log('Drone landed sucessfully.')
            else:
                self.log('Drone landed failed because another task is running.')

    def update_camera_info(self) -> None:
        self.depth_camera_pose = self.airsim_client.simGetCameraInfo(
            camera_name = self.depth_camera_name,
            vehicle_name = self.vehicle_name
        ).pose
        self.depth_camera_pose = self.ned2enu(self.depth_camera_pose)
        self.depth_camera_pose.position += self.init_trans
        
        self.scene_camera_pose = self.airsim_client.simGetCameraInfo(
            camera_name = self.scene_camera_name,
            vehicle_name = self.vehicle_name
        ).pose
        self.scene_camera_pose = self.ned2enu(self.scene_camera_pose)
        self.scene_camera_pose.position += self.init_trans

    def pub_camera_pose(self) -> None:
        self.update_camera_info()
        if self.depth_camera_pose != None:
            self.depth_pose_pub.publish(self.generate_posestamp(self.depth_camera_pose, self.depth_pose_seq))
            self.depth_pose_seq = (self.depth_pose_seq + 1) % 67108864
        if self.scene_camera_pose != None:
            self.scene_pose_pub.publish(self.generate_posestamp(self.scene_camera_pose, self.scene_pose_seq))
            self.scene_pose_seq = (self.scene_pose_seq + 1) % 67108864

    def generate_posestamp(self, pose:Pose, seq:int) -> PoseStamped:
        pose_stamped = PoseStamped()
        if pose != None:
            pose_stamped.pose.position = Point(pose.position.x_val, pose.position.y_val, pose.position.z_val)
            pose_stamped.pose.orientation = Quaternion(
                x=pose.orientation.x_val,
                y=pose.orientation.y_val,
                z=pose.orientation.z_val,
                w=pose.orientation.w_val
            )
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.seq = seq
            pose_stamped.header.frame_id = 'world'
        return pose_stamped

    def ned2enu(self, ned_pose:Pose) -> Pose:
        ned2enu_quaternion = Quaternionr(0, -math.sqrt(2)/2, -math.sqrt(2)/2, 0)
        t = ned_pose.position.y_val 
        ned_pose.position.y_val = ned_pose.position.x_val
        ned_pose.position.x_val = t
        ned_pose.position.z_val *= -1
        ned_pose.orientation.rotate(ned2enu_quaternion)
        return ned_pose

    def log(self, msg: str):
        time_str = time.strftime('%H:%M:%S', time.localtime())
        print(f'[Uav Controller {self.index}] {time_str}: {msg}')


if __name__ == '__main__':
    # 初始化ROS环境
    rospy.init_node('uav_controller', anonymous=True)
    uav_index = int(rospy.get_param('~drone_id', 1))
    cmd_topic = rospy.get_param('~cmd_topic', '/planning/pos_cmd_1')
    airsim_ip = rospy.get_param('~airsim_ip', '127.0.0.1')
    
    if_planner_enabled = rospy.get_param('~if_planner_enabled', False)
    
    controller = UavController(
        uav_index=uav_index,
        airsim_ip=airsim_ip,
        cmd_topic=cmd_topic
    )
    controller.log('Controller created.')
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        controller.pub_camera_pose()
        rate.sleep()
    
    controller.log('Controller shutdown.')