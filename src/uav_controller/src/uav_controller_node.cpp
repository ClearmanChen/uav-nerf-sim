/*
Uav Controller节点
开发人员：陈祥艺, chen-xy19@mails.tsinghua.edu.cn
功能简介：将飞控置于offboard状态，并将ego-planner计算的轨迹转为mavros的消息类型，让飞控执行
*/


#include <ros/ros.h>
// #include <boost/format.hpp>

//#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>
#include <std_msgs/Header.h>

#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>


// ego输出
quadrotor_msgs::PositionCommand ego_traj_cmd;
bool get_ego_traj;

// 发布的MAVROS控制指令
mavros_msgs::PositionTarget Command_Now;

ros::Subscriber ego_ouput_sub;
ros::Publisher command_pub;

mavros_msgs::State current_state; // 无人机状态
geometry_msgs::PoseStamped current_pose; // 无人机位姿

// 订阅无人机状态的回调函数将状态信息赋值给全局变量current_state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

bool if_reach_target(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    if ((current.pose.position.x - target.pose.position.x) * (current.pose.position.x - target.pose.position.x)
        + (current.pose.position.y - target.pose.position.y) * (current.pose.position.y - target.pose.position.y)
        + (current.pose.position.z - target.pose.position.z) * (current.pose.position.z - target.pose.position.z)
        < 0.1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ego_ouput_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    ego_traj_cmd = *msg;
    get_ego_traj = true;

    if(ego_traj_cmd.velocity.x == 0)
    {
        get_ego_traj = false;
    }
    else
    {
        Command_Now.header.stamp     = ros::Time::now();
        // 约定坐标系为ENU，详情阅读https://github.com/mavlink/mavros/issues/1500
        Command_Now.coordinate_frame = Command_Now.FRAME_LOCAL_NED;
        Command_Now.type_mask       = Command_Now.IGNORE_YAW_RATE;
        Command_Now.position     = ego_traj_cmd.position;
        Command_Now.velocity    = ego_traj_cmd.velocity;
        Command_Now.acceleration_or_force     = ego_traj_cmd.acceleration;
        Command_Now.yaw = ego_traj_cmd.yaw;
        // Command_Now.yaw_rate         = ego_traj_cmd.yaw_dot;
        
        command_pub.publish(Command_Now);   
    }
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_controller_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);
    // 【订阅】EGO的轨迹输出(traj_server的输出)
    // 该消息类型可查看https://github.com/jchenbr/quadrotor_msgs/blob/master/msg/PositionCommand.msg
    ego_ouput_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
        ("/planning/pos_cmd", 1, ego_ouput_cb);

    // 【订阅】MAVROS的无人机状态输出
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, state_cb);

    // 【订阅】MAVROS的无人机位置输出
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, local_position_cb);
    
    // 【发布】路径指令 （MAVROS发送至px4）
    command_pub = nh.advertise<mavros_msgs::PositionTarget>
        ("/mavros/setpoint_raw/local", 1);
    // 【发布】初始目标点指令 （MAVROS发送至px4）
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 10);

    // 【服务客户端】 装备无人机
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("/mavros/cmd/arming");

    // 【服务客户端】设置无人机模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("/mavros/set_mode");

    // 等待 FCU 的连接
    while(ros::ok() && !current_state.connected) 
    {
        ROS_INFO("[Uav Controller] connected: %d", current_state.connected);
        ros::spinOnce();
        rate.sleep();
    }

    //定义一个初始飞行目标
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // 在切换到offboard模式之前，你必须先发送一些期望点信息到飞控中。不然飞控会拒绝切换到offboard模式。
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        // 如果当前状态已经不是OFFBOARD则切换为OFFBOARD
        // 间隔5s
        if(current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
            last_request = ros::Time::now();
        }
        else
        {
            // 如果当前没有装备无人机 则进行装备
            // 间隔5s
            if(!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if(arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if (if_reach_target(current_pose, pose))
        {
            ROS_INFO("Reached takeoff target");
            break;
        }
        ROS_INFO("Current pos: %f %f %f\n", current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

