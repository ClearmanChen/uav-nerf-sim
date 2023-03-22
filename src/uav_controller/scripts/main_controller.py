#!/usr/bin/env python3
import airsim
import os
import rospy
from std_msgs.msg import String

class UavController:
    def __init__(self, uav_index: int = 0, ros_rate: int = 10) -> None:
        self.trigger_publisher = rospy.Publisher(f"/uav_controller_node_uav{uav_index}/trigger", String, queue_size=10)
        self.state_subscriber = rospy.Subscriber(f"/uav_controller_node_uav{uav_index}/state", String, self.state_callback)
        self.state = 'STANDBY'
        self.rate = rospy.Rate(ros_rate)

    def state_callback(self, data) -> None:
        self.state = data.data

    def takeoff(self) -> None:
        while self.state == 'STANDBY':
            self.trigger_publisher.publish('TAKEOFF')
            self.rate.sleep()
        
if __name__ == '__main__':

    # ros init and reading param
    rospy.init_node('main_controller', anonymous=True)
    rate = rospy.Rate(10)
    if rospy.has_param('~uav_num'):
        uav_num = int(rospy.get_param('~uav_num'))
    else:
        uav_num = 1
    print(f'[Main Controller] Number of drones is {uav_num}.')

    # Building controller
    uav_controller = []
    for i in range(uav_num):
        uav_controller.append(UavController(i))

    # Initial setting and takeoff
    # TODO: 等待一个控制信号后起飞

    print('[Main Controller] Mission start. Take off commands omitted.')
    for i in range(uav_num):
        uav_controller[i].takeoff()

    rospy.spin()
