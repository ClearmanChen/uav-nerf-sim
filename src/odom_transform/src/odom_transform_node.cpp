#include "odom_transform.h"
using namespace std;

string odom_local_topic, odom_world_topic;
string depth_camera_pose_topic, scene_camera_pose_topic;
bool if_recv_init = false;
int drone_id;
float uav_init_x, uav_init_y, uav_init_z;
float depth_init_x, depth_init_y, depth_init_z;
float scene_init_x, scene_init_y, scene_init_z;
ros::Subscriber odom_sub;
ros::Publisher odom_world_pub, depth_camera_pose_pub, scene_camera_pose_pub;
geometry_msgs::Quaternion enu_to_camera_axis = geometry_msgs::Quaternion();

geometry_msgs::PoseStamped GeneratePoseStampedFromOdom(const nav_msgs::Odometry& odom)
{
    geometry_msgs::PoseStamped p;
    p.header = odom.header;
    p.pose.orientation = odom.pose.pose.orientation;
    p.pose.position = odom.pose.pose.position;
    return p;
}

geometry_msgs::Quaternion quaternion_times(const geometry_msgs::Quaternion& A, const geometry_msgs::Quaternion& B)
{
    geometry_msgs::Quaternion res;
    res.w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z;
    res.x = A.w * B.x + A.x * B.w + A.y * B.z + A.z * B.y;
    res.y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x;
    res.z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w;
    return res;
}

void odomInputCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (if_recv_init == true)
    {
        nav_msgs::Odometry odom_transformed = (*msg);
        odom_transformed.header.frame_id = "world";
        odom_transformed.pose.pose.position.x += uav_init_x;
        odom_transformed.pose.pose.position.y += uav_init_y;
        odom_transformed.pose.pose.position.z += uav_init_z;
        odom_world_pub.publish(odom_transformed);

        nav_msgs::Odometry depth_camera_pose = (*msg);
        depth_camera_pose.header.frame_id = "world";
        depth_camera_pose.pose.pose.position.x += uav_init_x + depth_init_x;
        depth_camera_pose.pose.pose.position.y += uav_init_y + depth_init_y;
        depth_camera_pose.pose.pose.position.z += uav_init_z + depth_init_z;
        depth_camera_pose.pose.pose.orientation = quaternion_times(enu_to_camera_axis, depth_camera_pose.pose.pose.orientation);
        depth_camera_pose_pub.publish(GeneratePoseStampedFromOdom(depth_camera_pose));

        // nav_msgs::Odometry scene_camera_pose = (*msg);
        // scene_camera_pose.header.frame_id = "world_enu";
        // scene_camera_pose.pose.pose.position.x += (uav_init_x + scene_init_x);
        // scene_camera_pose.pose.pose.position.y += (uav_init_y + scene_init_y);
        // scene_camera_pose.pose.pose.position.z += (uav_init_z + scene_init_z);
        // scene_camera_pose_pub.publish(GeneratePoseStampedFromOdom(scene_camera_pose));
    }
}

int main(int argc, char **argv)
{
    enu_to_camera_axis.x = -0.707;
    enu_to_camera_axis.y = 0;
    enu_to_camera_axis.z = 0;
    enu_to_camera_axis.w = 0.707;

    ros::init(argc, argv, "odom_transform_node");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;

    ros::param::get("~drone_id", drone_id);
    odom_local_topic = "/airsim_node/uav" + to_string(drone_id) + "/odom_local_enu";
    odom_world_topic = "/airsim_node/uav" + to_string(drone_id) + "/odom_world";
    depth_camera_pose_topic = "/airsim_node/uav" + to_string(drone_id) + 
        "/uav" + to_string(drone_id) + "_depth_camera/pose";
    scene_camera_pose_topic = "/airsim_node/uav" + to_string(drone_id) + 
        "/uav" + to_string(drone_id) + "_scene_camera/pose";

    odom_sub = nh.subscribe(odom_local_topic, 10, odomInputCallback);
    odom_world_pub = nh.advertise<nav_msgs::Odometry>(odom_world_topic, 100);
    depth_camera_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(depth_camera_pose_topic, 100);
    scene_camera_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(scene_camera_pose_topic, 100);
    
    string odom_tf = "uav" + to_string(drone_id), 
           baselink_tf = "uav" + to_string(drone_id) + "/odom_local_enu",
           depth_camera_tf = "uav" + to_string(drone_id) + "_depth_camera_body",
           scene_camera_tf = "uav" + to_string(drone_id) + "_scene_camera_body";
    tf::StampedTransform world_to_odom_transform, baselink_to_depth_transform, baselink_to_scene_transform;
    while(ros::ok())
    {
        try
        {
            tf_listener.waitForTransform("world_enu", odom_tf, ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("world_enu", odom_tf, ros::Time(0), world_to_odom_transform);
            uav_init_x = world_to_odom_transform.getOrigin().getX();
            uav_init_y = world_to_odom_transform.getOrigin().getY();
            uav_init_z = world_to_odom_transform.getOrigin().getZ();

            tf_listener.waitForTransform(baselink_tf, depth_camera_tf, ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform(baselink_tf, depth_camera_tf, ros::Time(0), baselink_to_depth_transform);

            // transforms in tf tree are defaultly based on NED
            // so we need to manually process NED --> ENU
            depth_init_x = baselink_to_depth_transform.getOrigin().getY();
            depth_init_y = baselink_to_depth_transform.getOrigin().getX();
            depth_init_z = -baselink_to_depth_transform.getOrigin().getZ();

            // tf_listener.waitForTransform(baselink_tf, scene_camera_tf, ros::Time(0), ros::Duration(3.0));
            // tf_listener.lookupTransform(baselink_tf, scene_camera_tf, ros::Time(0), baselink_to_scene_transform);
            // scene_init_x = baselink_to_scene_transform.getOrigin().getY();
            // scene_init_y = baselink_to_scene_transform.getOrigin().getX();
            // scene_init_z = -baselink_to_scene_transform.getOrigin().getZ();
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
    ros::param::set("/airsim_node/uav" + to_string(drone_id) + "/init_x", uav_init_x);
    ros::param::set("/airsim_node/uav" + to_string(drone_id) + "/init_y", uav_init_y);
    ros::param::set("/airsim_node/uav" + to_string(drone_id) + "/init_z", uav_init_z);
    ros::spin();
    return 0;
}