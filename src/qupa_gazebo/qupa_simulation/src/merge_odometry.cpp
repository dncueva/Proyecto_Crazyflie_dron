#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_odometry");
    ros::NodeHandle nh("~");

    // Leer lista de robots desde YAML
    std::vector<std::string> robots;
    if (!nh.getParam("/robots", robots)) {
        ROS_WARN("No robots found in parameter server, using default qp_1");
        robots.push_back("qp_1");  // Fallback si no hay robots definidos en el YAML
    }

    // Leer la frecuencia desde YAML
    double publish_rate;
    nh.param("/publish_rate", publish_rate, 50.0);
    ros::Rate rate(publish_rate);

    // Publicador de transformaciones
    tf2_ros::TransformBroadcaster br;

    while (ros::ok()) {
        for (const std::string& robot : robots) {
            geometry_msgs::TransformStamped t;
            t.header.stamp = ros::Time::now();
            t.header.frame_id = "odom"; // Marco global
            t.child_frame_id = robot + "/odom"; // Marco del robot
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            t.transform.rotation.w = 1.0;

            br.sendTransform(t);
        }
        rate.sleep();
    }

    return 0;
}
