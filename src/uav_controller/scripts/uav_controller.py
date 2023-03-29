#!/usr/bin/env python3
import rospy
import airsim
import time
import math
# import tf
import os
from airsim.types import CameraInfo, Pose, Quaternionr, Vector3r, YawMode, DrivetrainType
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
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
        rospy.Subscriber(f"/airsim_node/uav{uav_index}/odom_local_enu", Odometry, self.vehicle_pose_callback)
        rospy.Subscriber(cmd_topic, PositionCommand, self.planner_output_callback)
   
        self.airsim_client = airsim.MultirotorClient(ip = airsim_ip, port=airsim_port, timeout_value=30)
        self.airsim_client.confirmConnection()

        self.vehicle_name = f'uav{uav_index}'
        self.depth_camera_name = f'uav{uav_index}_depth_camera'
        self.scene_camera_name = f'uav{uav_index}_scene_camera'
        self.depth_camera_pose = None
        self.scene_camera_pose = None
        self.vehicle_pose = None
        self.airsim_client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.airsim_client.armDisarm(True, vehicle_name=self.vehicle_name)

        self.camera_axis_orientation = Quaternionr(0.5, -0.5, 0.5, -0.5)
        self.ned2enu_axis_orientation = Quaternionr(0, -math.sqrt(2)/2, -math.sqrt(2)/2, 0)

        while not rospy.has_param(f'/airsim_node/uav{uav_index}/init_x') or \
            not rospy.has_param(f'/airsim_node/uav{uav_index}/init_y') or \
            not rospy.has_param(f'/airsim_node/uav{uav_index}/init_z') :
                self.rate.sleep()
        
        self.init_trans = Vector3r(int(rospy.get_param(f'/airsim_node/uav{uav_index}/init_x')),
            int(rospy.get_param(f'/airsim_node/uav{uav_index}/init_y')),
            int(rospy.get_param(f'/airsim_node/uav{uav_index}/init_z')))

        self.airsim_client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        self.airsim_client.moveToZAsync(-1.5, 1, vehicle_name=self.vehicle_name).join()

    def state_callback(self, data) -> None:
        self.state = data.data

    def planner_output_callback(self, data: PositionCommand):
        # Airsim api always works on NED coordinate system
        self.airsim_client.enableApiControl(True, vehicle_name=self.vehicle_name)

        # data.position.x -= self.init_trans.x_val
        # data.position.y -= self.init_trans.y_val
        # data.position.z -= self.init_trans.z_val
        # if self.index == 1:
        #     print(f'[{self.vehicle_name}] pos_real: {self.vehicle_pose.position.x_val: .3f}, \
        #         {self.vehicle_pose.position.y_val: .3f}, {self.vehicle_pose.position.z_val: .3f}')
        #     print(f'[{self.vehicle_name}] pos_cmd: {data.position.x: .3f}, {data.position.y: .3f}, {data.position.z: .3f}')
        # self.airsim_client.moveToPositionAsync(
        #     x = data.position.y,
        #     y = data.position.x,
        #     z = -data.position.z,
        #     velocity = 1,
        #     vehicle_name = self.vehicle_name,
        #     drivetrain = DrivetrainType.ForwardOnly,
        #     yaw_mode = YawMode(
        #         is_rate = False,
        #         yaw_or_rate = data.yaw
        #     )
        # ).join
        # self.rate.sleep()
    
    def vehicle_pose_callback(self, data: Odometry):
        self.vehicle_pose = Pose(
            position_val = Vector3r(
                x_val = data.pose.pose.position.x,
                y_val = data.pose.pose.position.y,
                z_val = data.pose.pose.position.z
            ),
            orientation_val = Quaternionr(
                x_val = data.pose.pose.orientation.x,
                y_val = data.pose.pose.orientation.y,
                z_val = data.pose.pose.orientation.z,
                w_val = data.pose.pose.orientation.w
            )
        )

    def trigger_callback(self, data: String) -> None:
        self.log(f'Receive command {data.data}.')
        if data.data == 'TAKEOFF':
            if self.state == Controller_State.STANDBY:
                self.state = Controller_State.TAKEOFF
                self.airsim_client.takeoffAsync(vehicle_name=self.vehicle_name).join()
                self.airsim_client.moveToZAsync(-4, 2).join()
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

    # def update_camera_pose(self) -> bool:
    #     try:
    #         self.depth_camera_pose = self.airsim_client.simGetCameraInfo(
    #             camera_name = self.depth_camera_name,
    #             vehicle_name = self.vehicle_name,
    #             external = False
    #         ).pose
    #         self.scene_camera_pose = self.airsim_client.simGetCameraInfo(
    #             camera_name = self.scene_camera_name,
    #             vehicle_name = self.vehicle_name,
    #             external = False
    #         ).pose
    #     except Exception as e:
    #         self.log('ERROR occured while getting camera info.')
    #         print(e)
    #         self.depth_camera_pose = None
    #         self.scene_camera_pose = None
    #         return False
    #     self.depth_camera_pose = self.ned2enu(self.depth_camera_pose)
    #     self.depth_camera_pose.orientation = self.camera_axis_orientation * self.depth_camera_pose.orientation
    #     self.depth_camera_pose.position += self.init_trans
        
    #     self.scene_camera_pose = self.ned2enu(self.scene_camera_pose)
    #     self.scene_camera_pose = self.ned2enu(self.scene_camera_pose)
    #     self.scene_camera_pose.orientation = self.camera_axis_orientation * self.scene_camera_pose.orientation
    #     self.scene_camera_pose.position += self.init_trans
    #     return True

    # def pub_camera_pose(self) -> None:
    #     self.update_camera_pose()
    #     if self.depth_camera_pose != None:
    #         self.depth_pose_pub.publish(self.generate_posestamp(self.depth_camera_pose, self.depth_pose_seq))
    #         self.depth_pose_seq = (self.depth_pose_seq + 1) % 67108864
    #     if self.scene_camera_pose != None:
    #         self.scene_pose_pub.publish(self.generate_posestamp(self.scene_camera_pose, self.scene_pose_seq))
    #         self.scene_pose_seq = (self.scene_pose_seq + 1) % 67108864

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
            pose_stamped.header.frame_id = 'world_enu'
        return pose_stamped

    def ned2enu(self, ned_pose:Pose) -> Pose:
        
        t = ned_pose.position.y_val 
        ned_pose.position.y_val = ned_pose.position.x_val
        ned_pose.position.x_val = t
        ned_pose.position.z_val *= -1
        # ned_pose.orientation = self.ned2enu_axis_orientation * ned_pose.orientation
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
    
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     controller.pub_camera_pose()
    #     rate.sleep()
    rospy.spin()
    controller.log('Controller shutdown.')