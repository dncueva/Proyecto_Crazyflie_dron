# Qupa simulation package

1. [Content](#1content)
2. [Launch](#2Launch)
3. [Package Layout](#3package-layout)
## 1.Content 

In this package should be all related to the simulation files, in this case the simulator used is Gazebo so we will be able to see the launch file used, the required nodes, the world and the models. We have to use the qupa_description package to add the robots description to our simulation so we need a dependency on our CMake to use the launch files.

The world and models directories have the files that describe the items spawned in the simulation, feel free to check those if you want.


## 2. Launch

The launch file used is `multi_qupa.launch`, this is the main launch file that spawn for the moment two robots, but it has the structure to be able to add different ones. That said let me show you the structure.

### Arguments

The arguments are the variables that define what we want to run our simulation. For example ,as referenced in the qupa_description package, we have "two" versions of our robot, a low resolution and a high resolution. The difference between them are the nodes running and the representation of the robot's real behaviour.

Here you can see the arguments used and what do they change in our simulation.

|`Argument` | Description |
|--------------|----------|
| `world` | Set the desired world file |
| `use_ir` | Add infrared sensors to the simulation |
| `high_res` | Add camera sensors instead of logical cameras |
| `qupa_1` | Robot's namespace for the first robot |
| `qupa_2` | Robot's namespace for the second robot |
| `use_rviz` | Visualize rviz|
| `first_qp_x_pos`| First qupa x axis spawn position|
| `first_qp_y_pos`| First qupa y axis spawn position|
| `first_qp_z_pos`| First qupa z axis spawn position|
| `first_qp_z_pos`| First qupa yaw spawn orientation|

### Structure

The First section on the launch file includes the arguments, robots definitions and world definition. We generally specified the Arguments at the start of the file to be able to check what they do and reference them later in the code.

```xml
<launch>
  <!--Arguments-->
  <arg name="world" value="qupa_sim" />           <!--Gazebo World Argument-->
  <arg name="use_ir" default="false" />           <!--Add ir laser to the robot-->
  <arg name="high_res" default="true" />          <!--Spawn high res version-->
  <arg name="qupa_1" default="qp_1"/>             <!--Robots namespace for first robot-->
  <arg name="qupa_2" default="qp_2"/>             <!--Robots namespace for second robot-->
  <arg name="use_rviz" default="true"/>           <!--Visualize Rviz-->

  <!--Robots poses-->
  <arg name="first_qp_x_pos" default="-1.0"/>     
  <arg name="first_qp_y_pos" default="-1.0"/>
  <arg name="first_qp_z_pos" default="0.0"/>
  <arg name="first_qp_yaw" default="1.57"/>

  <arg name="second_qp_x_pos" default="2.0"/>
  <arg name="second_qp_y_pos" default="-1.0"/>
  <arg name="second_qp_z_pos" default="0.0"/>
  <arg name="second_qp_yaw" default="3.1416"/>

  <!-- Configure GAZEBO_MODEL_PATH  to give access to the packages models-->
  <env name="GAZEBO_MODEL_PATH" value="$(find qupa_simulation)/models"/>
```
After this section we can define the nodes using the arguments previously defined.

### Gazebo World 
```xml
<!--Gazebo World definition-->
<include file="$(find gazebo_ros)/launch/empty_world.launch">
<arg name="world_name" value="$(find qupa_simulation)/worlds/$(arg world).world"/>
<arg name="paused" value="false"/>
<arg name="use_sim_time" value="true"/>
<arg name="gui" value="true"/>
<arg name="headless" value="false"/>
<arg name="debug" value="false"/>
</include>
```
### Rviz Visualization
We add the condition to open rviz by using a conditional and the `use_rviz` argument
```xml
  <group if="$(arg use_rviz)">
    <node   name="rviz" 
            pkg="rviz" 
            type="rviz" 
            args="-d $(find qupa_simulation)/config/multi_qupa.rviz" 
            output="screen" />
  </group>
```
### Robots Spawn

Here is the group of nodes used to spawn a robot. Feel free to check the second robot spawn to check the changes between one and the other. Spoiler alert, you only need to change the robots argument, in this case the `arg qupa_1` to `arg qupa_2`. 

#### Note: You could add a third robot and so on. Do not forget to describe the namespace, the position and the orientation of each of them to later reference this arguments in the robots namespace group. Also remember that adding robots will increase the CPU demand so keep the low definition on bigger simulations

```xml
<group ns="$(arg qupa_1)">    
    <!--Create URDF from Xacro files-->
    <param  name="robot_description" 
            command="$(find xacro)/xacro --inorder $(find qupa_description)/urdf/qupa.xacro tf_prefix:=$(arg qupa_1) use_ir:=$(arg use_ir) high_res:=$(arg high_res)" />
    <!--Create Robot State Publisher-->
    <node   pkg="robot_state_publisher" 
            type="robot_state_publisher" 
            name="robot_state_publisher" 
            output="screen">
      
      <param name="publish_frequency" type="double" value="50.0" />
      <param name="tf_prefix" value="$(arg qupa_1)" />
    </node>
    <!-- Spawn Robot in Gazebo -->
    <node   name="spawn_urdf" 
            pkg="gazebo_ros" 
            type="spawn_model" 
            args="-urdf -model $(arg qupa_1) -x $(arg first_qp_x_pos) -y $(arg first_qp_y_pos) -z $(arg first_qp_z_pos) -Y $(arg first_qp_yaw) -param robot_description" />
    
    <!--Logical Camera Bridge Gazebo-Ros-->
    <group unless="$(arg high_res)">
        <node   pkg="qupa_simulation" 
                type="logical_camera_bridge" 
                name="logical_camera_bridge_$(arg qupa_1)">

        <param  name="robot_namespace" value="$(arg qupa_1)" />
        </node>
    </group>

    <!-- Publish a world tf for each robot -->
    <node   pkg="tf2_ros" 
            type="static_transform_publisher" 
            name="world_to_qp_1" 
            args="$(arg first_qp_x_pos) $(arg first_qp_y_pos) $(arg first_qp_z_pos) $(arg first_qp_yaw) 0 0 world $(arg qupa_1)/base_link" />
</group>
```


## 3. Package Layout

### qupa_simulation

The qupa simulation package has the launch, config, nodes, models and world files to run the simulations. It uses the qupa_description package to bring the robot

- `qupa_simulation` Main package
    - `config:`   Robots configurations  
        - `multi_qupa.rviz`
        - `robots.yaml` 
    - `include:`    
    - `launch:` Simulation launch files
        - `multi_qupa.launch`     
    - `models:` Models used in the world files     
        - `paredes`
        -  `terrain` 
    - `src:`   Ros Nodes     
        - `logical_camera_bridge.cpp`
        
    - `worlds:` World used in the simulation
        - `qupa_sim.world`
