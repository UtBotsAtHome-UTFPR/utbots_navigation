#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

nav_msgs::Odometry msg_odom;
bool newOdom = false;

void callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    msg_odom = *msg;
    newOdom = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_tf_broadcaster");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom = nh.subscribe("/RosAria/pose", 0, callback_odom);

    msg_odom.header.frame_id = "map";
    msg_odom.header.stamp = ros::Time::now();
    msg_odom.child_frame_id = "odom";

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate loopRate(30);

    while (ros::ok())
    {
        loopRate.sleep();
        ros::spinOnce();

        if (newOdom == true) {
            newOdom = false;

            // Tf
            transform.setOrigin( tf::Vector3(
                msg_odom.pose.pose.position.x, 
                msg_odom.pose.pose.position.y, 
                msg_odom.pose.pose.position.z) );

            transform.setRotation(tf::Quaternion(
                msg_odom.pose.pose.orientation.x, 
                msg_odom.pose.pose.orientation.y, 
                msg_odom.pose.pose.orientation.z,
                msg_odom.pose.pose.orientation.w));

            transform.setRotation(tf::Quaternion(
                0, 0, 0, 1.0));

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

            ROS_INFO("Updating odom tf...");
        }
        
    }

    return 0;
}