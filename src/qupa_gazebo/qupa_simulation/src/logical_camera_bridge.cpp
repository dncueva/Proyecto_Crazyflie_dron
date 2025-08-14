#include <ros/ros.h>
#include <qupa_msgs/ModelsList.h>
#include <qupa_msgs/ModelPose.h>
#include <geometry_msgs/Pose.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/logical_camera_image.pb.h>

class GazeboToROSBridge
{
private:
    gazebo::transport::NodePtr gzNode;
    gazebo::transport::SubscriberPtr gzSubscriber;
    ros::NodeHandle nh;
    ros::Publisher pub;
    std::string gazebo_topic;
    std::string ros_topic;

public:
    GazeboToROSBridge(const std::string &robot_namespace)
    {
        gazebo_topic = "~/" + robot_namespace + "/base_link/" + robot_namespace + "_mirror/models";
        ros_topic = "/" + robot_namespace + "/logical_camera";

        pub = nh.advertise<qupa_msgs::ModelsList>(ros_topic, 10);

        // Inicializar el sistema de transporte de Gazebo
        gazebo::client::setup();
        gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
        gzNode->Init();

        gzSubscriber = gzNode->Subscribe(gazebo_topic, &GazeboToROSBridge::OnGazeboMessage, this);
    }

    void OnGazeboMessage(ConstLogicalCameraImagePtr &_msg)
    {
        qupa_msgs::ModelsList ros_msg;
        ros_msg.header.stamp = ros::Time::now();

        for (int i = 0; i < _msg->model_size(); ++i)
        {   
            // Filtrar ground_plane
            if (_msg->model(i).name() == "ground_plane")
                continue;
            if (_msg->model(i).name() == "paredes")
                continue;

            qupa_msgs::ModelPose model;
            model.name = _msg->model(i).name();

            model.pose.position.x = _msg->model(i).pose().position().z();
            model.pose.position.y = _msg->model(i).pose().position().y();
            model.pose.position.z = _msg->model(i).pose().position().x();
            model.pose.orientation.x = _msg->model(i).pose().orientation().x();
            model.pose.orientation.y = _msg->model(i).pose().orientation().y();
            model.pose.orientation.z = _msg->model(i).pose().orientation().z();
            model.pose.orientation.w = _msg->model(i).pose().orientation().w();

            ros_msg.models.push_back(model);
        }

        pub.publish(ros_msg);
    }

    ~GazeboToROSBridge()
    {
        gazebo::client::shutdown();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "logical_camera_bridge");
    ros::NodeHandle private_nh("~");

    std::string robot_namespace;
    private_nh.param<std::string>("robot_namespace", robot_namespace, "qp_1");

    GazeboToROSBridge bridge(robot_namespace);
    ros::spin();
    return 0;
}
