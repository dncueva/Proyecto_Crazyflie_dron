# qupa_description

In this package we have the urdf files for the robots description and the gazebo simulation controllers
## 1. Content

The content of this package is mainly the description of our robot using Xacro files and Gazebo plugins. We added the conditions inside the xacro files to define the differences between the higher and lower resolution. We we'll be checking them at detail later.

For those who have never worked in ROS what you need to know is this package has the robots transformation between the frames that will be later used on the spawning process and simulation. It also have some dynamic definitions that will be useful in our Gazebo simulation.

## 2. URDF

The URDF or Unified Robot Description Format is as mentioned before the definition of the frames of the elements that represents the robot. Inside the `urdf` directory We have separate the files in different folders so that it could be easier to search the information. 

### `qupa.xacro`

The main urdf file of the robot is called `qupa.xacro` It includes all the other files and has the macro definitions and some conditions on what to show and run with the launch file arguments.

Feel free to check each of the files included and check the arguments that can be edited for simulation purposes. I recommend not to change the camera ones, they could be difficult to set them properly if you are not 

```xml
<?xml version="1.0" ?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="$(arg tf_prefix)">

    <xacro:property name="M_PI" value="3.1415926535897931" />
    <xacro:property name="tf_prefix" default=""/>
    <xacro:property name="use_ir" value="$(arg use_ir)" lazy_eval="false"/>
    <xacro:property name="high_res" value="$(arg high_res)" lazy_eval="false"/>
    


    <!--Include Xacro files -->
    <!--wheel properties and links definitions-->
    <xacro:include filename="$(find qupa_description)/urdf/wheels/wheel.xacro" />
    <!--infrared sensor properties and links definitions-->                   
    <xacro:include filename="$(find qupa_description)/urdf/sensors/infrared.xacro"/> 
    <!--logical camera properties and links definitions, the mirror acts as a 'camera'-->              
    <xacro:include filename="$(find qupa_description)/urdf/sensors/mirror.xacro" />
    <!--base properties and links definitions-->             
    <xacro:include filename="$(find qupa_description)/urdf/bases/base.xacro" />
    <!--Gazebo plugins definitions-->                     
    <xacro:include filename="$(find qupa_description)/urdf/gazebo_plugins.xacro" />     
    <!--color sensor links definitions and plugin-->  
    <xacro:include filename="$(find qupa_description)/urdf/sensors/color.xacro" />
    <!--caster wheel definition-->  
    <xacro:include filename="$(find qupa_description)/urdf/wheels/caster.xacro" />    
        
    <xacro:base/>               <!--Base Macro-->
    <xacro:mirror/>             <!--Mirror Macro-->

    <!-- Wheels Macro-->
    <xacro:wheel 
        wheel_name="R_wheel" 
        wheel_joint_name="R_wheel_joint" 
        wheel_parent_name="base_link" 
        wheel_joint_origin_xyz="-0 0.0415 0.016" 
        wheel_joint_origin_rpy=" 1.5708 0 3.1416" />
    <xacro:wheel 
        wheel_name="L_wheel" 
        wheel_joint_name="L_wheel_joint" 
        wheel_parent_name="base_link" 
        wheel_joint_origin_xyz="0 -0.0415 0.016" 
        wheel_joint_origin_rpy=" 1.5708 0 0" />
    <xacro:caster
        link_name = "caster_R"
        joint_name = "caster_R_joint"
        joint_origin_xyz = "0.0305 -0.0265 0.005"/>
    <xacro:caster
        link_name = "caster_L"
        joint_name = "caster_L_joint"
        joint_origin_xyz = "0.0305 0.0265 0.005"/>
    <xacro:caster
        link_name = "caster_B"
        joint_name = "caster_B_joint"
        joint_origin_xyz = "-0.0327 0 0.005"/>

    <!--Added condition to add the ir sensors, requires CPU-->
    <xacro:if value="${use_ir}">  <!--Launch argument Condition-->           
        <xacro:ir_sensor 
            link_name="ir_0" 
            joint_name="ir_0_joint" 
            joint_origin_xyz="0.004 -0.056 0.034" 
            joint_origin_rpy="${M_PI/2} 0 0" />

        <xacro:ir_sensor 
            link_name="ir_45" 
            joint_name="ir_45_joint" 
            joint_origin_xyz="0.04159 -0.0376 0.034" 
            joint_origin_rpy="${M_PI/2} 0 ${M_PI/4}" />
        
        <xacro:ir_sensor 
            link_name="ir_90" 
            joint_name="ir_90_joint" 
            joint_origin_xyz="0.056 0 0.034" 
            joint_origin_rpy="${M_PI/2} 0 ${M_PI/2}" />
        
        <xacro:ir_sensor 
            link_name="ir_135" 
            joint_name="ir_135_joint" 
            joint_origin_xyz="0.04159 0.0376 0.034" 
            joint_origin_rpy="${M_PI/2} 0 ${3*M_PI/4}" />

        <xacro:ir_sensor 
            link_name="ir_180" 
            joint_name="ir_180_joint" 
            joint_origin_xyz="0.004 0.056 0.034" 
            joint_origin_rpy="${M_PI/2} 0 ${M_PI}" />

        <xacro:ir_sensor 
            link_name="ir_270" 
            joint_name="ir_270_joint" 
            joint_origin_xyz="-0.056 0 0.034" 
            joint_origin_rpy="${M_PI/2} 0 ${-M_PI/2}" />
    </xacro:if>
    <xacro:if value="${high_res}">
        <xacro:color_camera/> <!--Color sensor-->
        <xacro:real_cam/>     <!--Fisheye camera-->
    </xacro:if>
    <xacro:unless value="${high_res}">
        <xacro:log_cam/>            <!--Logical Camera Macro-->
    </xacro:unless>
</robot>

```

## 3.Package Layout
The package is organized in the following structure, feel free to check any of the files and send me any suggestions.
- `qupa_description`   
    - `launch`
        - `view_robot.launch` : Launches qupa robot description and rviz to visualize it
    - `urdf`
        - `qupa.xacro:` Main xacro file, includes all the other files
        - `bases` : Description for the qupa base model
            - `base.xacro` 
        - `sensors` : Sensors directory
            - `color.xacro:` Color sensor xacro definition 
            - `infrared.xacro:` IR sensors xacro definition
            - `mirror.xacro:` Mirror link and camera definition (HD and LD models)
        - `wheels` Wheels directory
            - `caster.xacro:` Caster wheel xacro definition
            - `wheels.xacro:` Regular wheels xacro definition
        - `gazebo_plugins.xacro:` Gazebo added plugins
    - `meshes:`  STL files
    - `rviz:` Rviz configuration 
    