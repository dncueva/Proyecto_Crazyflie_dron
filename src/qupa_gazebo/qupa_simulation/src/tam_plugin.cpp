#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <ignition/math/Color.hh>

namespace gazebo
{ class TAMColorPlugin : public VisualPlugin
    {
        rendering::VisualPtr visual;
        ros::Subscriber color_sub;
        ros::Publisher task_pub;
        std::string model_name;
        

    public:
        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
        {
        visual = _visual;

        if (!ros::isInitialized())
        {
            int argc = 0;
            char** argv = NULL;
            ros::init(argc, argv, "gazebo_tam_color", ros::init_options::NoSigintHandler);
        }

        ros::NodeHandle nh;

        // --- OBTENER EL NOMBRE DEL MODELO (INSTANCIA) ---
        this->model_name = GetModelName(visual);
        //   if (visual && visual->GetScene() && visual->GetScene()->Name() != "__default__") {
        //       rendering::VisualPtr parent = visual->GetParent();
        //       while (parent && parent->GetParent()) {
        //           parent = parent->GetParent();
        //       }
        //       if (parent) {
        //           this->model_name = parent->Name(); // Ej: "TAM_0"
        //       }
        //   }
        //   if (this->model_name.empty()) {
        //       this->model_name = "TAM"; // Valor por defecto, pero debería no pasar
        //   }

        std::string color_topic = "/" + this->model_name + "/set_color";
        std::string task_topic = "/" + this->model_name + "/task";

        ROS_INFO_STREAM("[TAMColorPlugin] Subscribing to: " << color_topic);
        ROS_INFO_STREAM("[TAMColorPlugin] Publishing to: " << task_topic);

        // Suscripción para cambiar color
        color_sub = nh.subscribe(color_topic, 1, &TAMColorPlugin::OnColorMsg, this);

        // Publicador para tarea propuesta
        task_pub = nh.advertise<std_msgs::String>(task_topic, 1,true);
        }
        std::string GetModelName(rendering::VisualPtr visual) {
            rendering::VisualPtr parent = visual;
            while (parent) {
                if (!parent->GetParent() || parent->Name().find("::") == std::string::npos) {
                    return parent->Name();
                }
                parent = parent->GetParent();
            }
            return "TAM";}

        void OnColorMsg(const std_msgs::ColorRGBA::ConstPtr& msg)
        {
        // Cambia el color visual
        ignition::math::Color color(msg->r, msg->g, msg->b, msg->a);
        visual->SetAmbient(color);
        visual->SetDiffuse(color);
        visual->SetEmissive(color);

        // Decide qué tarea propone según el color
        std_msgs::String task_msg;

        if (msg->r > 0.8 && msg->g < 0.2 && msg->b < 0.2)  // Rojo fuerte
        task_msg.data = "OCUPADO";
        if (msg->g > 0.8 && msg->b < 0.2) // Verde
            task_msg.data = "TAREA_A";
        else if (msg->b > 0.8 && msg->g < 0.2) // Azul
            task_msg.data = "TAREA_B";
        else // Blanco u otro
            task_msg.data = "STANDBY";
        
        task_pub.publish(task_msg); // solo publica


        // ROS_INFO_STREAM("[" << model_name << "] Estado: " << task_msg.data);
        }
    };

    GZ_REGISTER_VISUAL_PLUGIN(TAMColorPlugin)
    }
