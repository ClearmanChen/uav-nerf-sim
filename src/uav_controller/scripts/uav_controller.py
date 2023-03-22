#!/usr/bin/env python3
import rospy
import airsim
import time
import os
from airsim.types import Vector3r
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from enum import Enum

class Controller_State(Enum):
    STANDBY = 0
    TAKEOFF = 1
    WAITINGTARGET = 2
    ONWAY = 3
    LANDING = 4

class UavController:
    def __init__(self, 
        uav_index: int = 0, 
        ros_rate: int = 10,
        airsim_ip: str = 127.0.0.1
    ) -> None:
        self.index = uav_index
        self.state = Controller_State.STANDBY
        self.rate = rospy.Rate(ros_rate)
        rospy.Subscriber(f"/uav_controller_node_uav{uav_index}/trigger", String, trigger_callback)
        rospy.Subscriber(f"/ego_planner_node_uav{uav_index}/target_input", PoseStamped, target_input_callback)
        rospy.Subscriber(f"/ego_planner_node_uav{uav_index}/pos_cmd", PositionCommand, self.planner_output_callback)
   
        self.airsim_client = airsim.MultirotorClient(ip = airsim_ip)
        self.airsim_client.confirmConnection()

        self.vehicle_name = f'uav{uav_index}'
        self.airsim_client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.airsim_client.armDisarm(True, vehicle_name=self.vehicle_name)

    def state_callback(self, data) -> None:
        self.state = data.data

    def planner_output_callback(self, data):
        if self.state == Controller_State.WAITINGTARGET or self.state == Controller_State.ONWAY:
            self.state = Controller_State.ONWAY
            self.airsim_client.moveByVelocityAsync(
                data.velocity.x, 
                data.velocity.y, 
                data.velocity.z, 
                vehicle_name=self.vehicle_name, 
                duration=0.1
            )

    def target_input_callback(self, data: PoseStamped):
        if if_planner_enabled == False:
            self.airsim_client.moveToPositionAsync(
                data.pose.position.x,
                data.pose.position.y,
                data.pose.position.z,
                5
            )
        self.state = Controller_State.ONWAY
    
    def trigger_callback(self, data):
        print(f'[Uav Controller {self.index}]: Receive command {data.data}.')
        if data.data == 'TAKEOFF':
            if current_state == Controller_State.STANDBY:
                current_state = Controller_State.TAKEOFF
                self.airsim_client.takeoffAsync(vehicle_name=self.vehicle_name).join()
                self.log('Drone took off sucessfully.')
                current_state = Controller_State.WAITINGTARGET
            elif current_state == Controller_State.WAITINGTARGET or current_state == Controller_State.ONWAY:
                self.log('Drone has already taken off.')
        elif data.data == 'LAND':
            if current_state == Controller_State.WAITINGTARGET:
                current_state = Controller_State.LANDING
                self.airsim_client.landAsync(vehicle_name=self.vehicle_name).join()
                current_state = Controller_State.STANDBY
                self.log('Drone landed sucessfully.')
            else:
                self.log('Drone landed failed because another task is running.')

    def target_reach_check(self, target_pos: Vector3r):
        vechicle_state = self.airsim_client.simGetGroundTruthKinematics(vehicle_name=self.vehicle_name)
        if self.state == Controller_State.ONWAY:
            if vechicle_state.position.distance_to(target_pos) < 1 and vechicle_state.linear_velocity.get_length() < 0.1:
                self.log('Reach target. Current position: {vechicle_state.position}')
                self.state = Controller_State.WAITINGTARGET

    def log(self, msg: str):
        time_str = time.strftime('%H:%M:%S', time.localtime())
        print(f'[Uav Controller {self.index}] {time_str}: {msg}')


if __name__ == '__main__':
    # 初始化ROS环境
    rospy.init_node('uav_controller', anonymous=True)
    uav_index = int(rospy.get_param('~drone_id', 1))
    if_planner_enabled = rospy.get_param('~if_planner_enabled', False)
    
    controller = UavController(uav_index=uav_index)
    controller.log('Controller created.')
    
    rospy.spin()