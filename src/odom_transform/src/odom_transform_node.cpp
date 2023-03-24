#include "odom_transform.h"
using namespace std;

string odom_input_topic, odom_output_topic;
bool if_recv_init = false;
int drone_id, init_x, init_y, init_z;
ros::Subscriber odom_sub;
ros::Publisher odom_pub;

void odomInputCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (if_recv_init == true)
    {
        nav_msgs::Odometry odom_transformed = (*msg);
        odom_transformed.header.frame_id = "world";
        odom_transformed.pose.pose.position.x += init_x;
        odom_transformed.pose.pose.position.y += init_y;
        odom_transformed.pose.pose.position.z += init_z;
        odom_pub.publish(odom_transformed);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_transform_node");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;

    ros::param::get("~drone_id", drone_id);
    ros::param::get("~odom_input", odom_input_topic);
    ros::param::get("~odom_output", odom_output_topic);

    odom_sub = nh.subscribe(odom_input_topic, 10, odomInputCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_output_topic, 100);
    
    string local_odom_tf = "uav" + to_string(drone_id);
    tf::StampedTransform transform;
    while(ros::ok())
    {
        try
        {
            tf_listener.waitForTransform("world_enu", local_odom_tf, ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("world_enu", local_odom_tf, ros::Time(0), transform);
            init_x = transform.getOrigin().getX();
            init_y = transform.getOrigin().getY();
            init_z = transform.getOrigin().getZ();
            if_recv_init = true;
            break;
        }
        catch(tf::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    ros::param::set("/airsim_node/uav" + to_string(drone_id) + "/init_x", init_x);
    ros::param::set("/airsim_node/uav" + to_string(drone_id) + "/init_y", init_y);
    ros::param::set("/airsim_node/uav" + to_string(drone_id) + "/init_z", init_z);
    ros::spin();
    return 0;
}