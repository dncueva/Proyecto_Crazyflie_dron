#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ColorRGBA.h>
#include <qupa_msgs/ModelsList.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <random>

class TAMSeeker {
public:
    TAMSeeker(const ros::NodeHandle& nh)
        : nh_(nh),
          state_(WAITING_TAMS)
    {
        // Leer parámetros ROS
        nh_.param<std::string>("robot_name", robot_name_, "qp_1");
        nh_.param<int>("num_tams", num_tams_, 24);
        nh_.param<double>("max_linear", max_linear_, 0.10);
        nh_.param<double>("max_angular", max_angular_, 0.5);
        nh_.param<double>("angle_threshold", angle_threshold_, 1.57); // rad
        nh_.param<double>("scan_min_dist", scan_min_dist_, 0.20);
        nh_.param<int>("search_window", search_window_, 3);
        nh_.param<double>("throttle_print", throttle_print_, 1.0);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/" + robot_name_ + "/cmd_vel", 1);

        // Suscriptores de color para saber si el TAM está activo
        for (int i = 0; i < num_tams_; ++i) {
            std::string tam_name = "TAM_" + std::to_string(i);
            tam_active_[tam_name] = false;
            tam_color_subs_.push_back(
                nh_.subscribe("/" + tam_name + "/set_color", 1, &TAMSeeker::colorCallback, this)
            );
        }
        // Subscripción a la logical_camera
        logical_camera_sub_ = nh_.subscribe("/" + robot_name_ + "/logical_camera", 1, &TAMSeeker::cameraCallback, this);

        // Lidar
        scan_sub_ = nh_.subscribe("/" + robot_name_ + "/scan", 1, &TAMSeeker::scanCallback, this);

        ROS_INFO("[TAMSeeker] Nodo iniciado para %s", robot_name_.c_str());
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();

            if (state_ == WAITING_TAMS) {
                active_tams_.clear();
                for (const auto& kv : tam_active_) {
                    if (kv.second) active_tams_.push_back(kv.first);
                }
                if (active_tams_.empty()) {
                    ROS_INFO_THROTTLE(throttle_print_, "[TAMSeeker] Esperando TAMS activos...");
                    continue;
                } else {
                    ROS_INFO("[TAMSeeker] TAMS activos:");
                    for (const auto& name : active_tams_) ROS_INFO("   %s", name.c_str());
                    state_ = SEEKING;
                }
            }

            // Si no hay TAM en cámara, sigue buscando
            if (!found_tam_) {
                ROS_INFO_THROTTLE(2.0, "[TAMSeeker] Explorando porque no ve TAMS...");
                geometry_msgs::Twist twist;
                twist.linear.x = 0.07;  // Puede ser 0.0 si prefieres solo girar
                twist.angular.z = 0.3;  // Giro suave
                cmd_vel_pub_.publish(twist);
                rate.sleep();
                continue;
            }


            // Control para ir al TAM
            double dist = std::hypot(tam_dx_, tam_dy_);
            double angle = std::atan2(tam_dy_, tam_dx_);

            ROS_INFO_THROTTLE(0.5, "[TAMSeeker] TAM seleccionado: %s | Dist: %.3f | Angle: %.2f deg",
                              selected_tam_.c_str(), dist, angle*180/M_PI);

            if (obstacleAhead()) {
                ROS_WARN_THROTTLE(0.5, "[TAMSeeker] Obstacle detected! Gira...");
                randomTurn();
            } else {
                geometry_msgs::Twist twist;
                if (std::fabs(angle) < angle_threshold_) {
                    twist.linear.x = max_linear_; // Avanza
                    twist.angular.z = max_angular_ * angle / angle_threshold_;
                } else {
                    twist.linear.x = 0.0;
                    twist.angular.z = (angle > 0) ? max_angular_ : -max_angular_;
                }
                cmd_vel_pub_.publish(twist);
            }

            rate.sleep();
        }
    }

private:
    enum State { WAITING_TAMS, SEEKING };
    State state_;
    ros::NodeHandle nh_;
    std::string robot_name_;
    int num_tams_;
    ros::Publisher cmd_vel_pub_;
    std::vector<ros::Subscriber> tam_color_subs_;
    ros::Subscriber logical_camera_sub_;
    ros::Subscriber scan_sub_;

    // Parámetros de control
    double max_linear_, max_angular_, angle_threshold_, scan_min_dist_, throttle_print_;
    int search_window_;

    std::unordered_map<std::string, bool> tam_active_;
    std::vector<std::string> active_tams_;
    std::string selected_tam_;
    double tam_dx_ = 0, tam_dy_ = 0;
    bool found_tam_ = false;
    std::vector<float> laser_;

    void colorCallback(const std_msgs::ColorRGBA::ConstPtr& msg) {
        // Identificar el TAM por el topic name
        std::string topic = ros::names::resolve(ros::this_node::getName());
        std::string tam_name = topic.substr(topic.find_last_of('/') + 1);
        // Verde/azul es activo, blanco y rojo no
        bool activo = !(msg->r > 0.8 && msg->g > 0.8 && msg->b > 0.8) && !(msg->r > 0.8 && msg->g < 0.2 && msg->b < 0.2);
        tam_active_[tam_name] = activo;
    }

    void cameraCallback(const qupa_msgs::ModelsList::ConstPtr& msg) {
        found_tam_ = false;
        double min_dist = 1e9;
        std::string best_tam;
        double best_dx = 0, best_dy = 0;
        for (const auto& model : msg->models) {
            if (tam_active_.count(model.name) && tam_active_[model.name]) {
                double dx = -model.pose.position.x; // Corrige frame si necesario
                double dy = -model.pose.position.y;
                double dist = std::hypot(dx, dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_dx = dx;
                    best_dy = dy;
                    best_tam = model.name;
                }
            }
        }
        if (!best_tam.empty()) {
            found_tam_ = true;
            tam_dx_ = best_dx;
            tam_dy_ = best_dy;
            selected_tam_ = best_tam;
        }
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        laser_ = msg->ranges;
    }

    bool obstacleAhead() {
        if (laser_.empty()) return false;
        int n = laser_.size();
        int center = n / 2;
        for (int i = center - search_window_; i <= center + search_window_; ++i) {
            if (i < 0 || i >= n) continue;
            float r = laser_[i];
            if (!std::isinf(r) && r < scan_min_dist_)
                return true;
        }
        return false;
    }

    void randomTurn() {
        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = ((rand() % 2) == 0) ? max_angular_ : -max_angular_;
        cmd_vel_pub_.publish(twist);
        ros::Duration(0.6).sleep();
    }

    void publishStop() {
        geometry_msgs::Twist twist;
        cmd_vel_pub_.publish(twist);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tam_seeker_cpp");
    ros::NodeHandle nh("~");
    TAMSeeker seeker(nh);
    seeker.spin();
    return 0;
}
