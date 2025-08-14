import math
import random

N_ROBOTS = 6
HEX_RADIUS = 0.75  # Ajusta al radio de tu hexágono en metros
Z = 0.01         # Altura de los robots
HIGH_RES = "false"

def hex_positions(n, radius):
    """Genera n posiciones (x, y) aleatorias dentro de un círculo de radio 'radius'."""
    positions = []
    for _ in range(n):
        r = radius * math.sqrt(random.uniform(0, 1))  # Uniforme en el área
        angle = random.uniform(0, 2 * math.pi)
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        positions.append((x, y))
    return positions

positions = hex_positions(N_ROBOTS, HEX_RADIUS)

with open("src/qupa_gazebo/qupa_simulation/launch/tam_experiment.launch", "w") as f:
    f.write('<?xml version="1.0"?>\n')
    f.write('<launch>\n\n')

    # World argument
    f.write('  <arg name="world" value="TAM_experiment" />\n')
    f.write('  <arg name="high_res" default="{}" />\n'.format(HIGH_RES))
    f.write('  <env name="GAZEBO_MODEL_PATH" value="$(find qupa_simulation)/models" />\n')
    f.write('  <arg name="num_tams" default="{}" />\n\n'.format(N_ROBOTS+3))  # O el número que quieras

    # Gazebo World
    f.write('  <include file="$(find gazebo_ros)/launch/empty_world.launch">\n')
    f.write('    <arg name="world_name" value="$(find qupa_simulation)/worlds/$(arg world).world" />\n')
    f.write('    <arg name="paused" value="false"/>\n')
    f.write('    <arg name="use_sim_time" value="true"/>\n')
    f.write('    <arg name="gui" value="true"/>\n')
    f.write('    <arg name="headless" value="false"/>\n')
    f.write('    <arg name="debug" value="false"/>\n')
    f.write('  </include>\n\n')

    # TAM global checking node (solo uno)
    f.write('  <node pkg="qupa_behavior" type="tam_assign.py" name="tam_assign" output="screen">\n')
    f.write('    <param name="num_tams" value="$(arg num_tams)"/>\n')
    f.write('  </node>\n\n')


    for i, (x, y) in enumerate(positions):
        ns = f"qp_{i+1}"
        # Orientación aleatoria (puedes poner 0 si quieres todos igual)
        yaw = round(random.uniform(0, 2 * math.pi), 3)
        f.write(f'  <group ns="{ns}">\n')
        # Xacro robot description
        f.write(f'    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find qupa_description)/urdf/qupa.xacro tf_prefix:={ns} high_res:=$(arg high_res)" />\n')
        # Robot state publisher
        f.write(f'    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">\n')
        f.write(f'      <param name="publish_frequency" type="double" value="50.0" />\n')
        f.write(f'      <param name="tf_prefix" value="{ns}" />\n')
        f.write(f'    </node>\n')
        # Spawn robot in Gazebo
        f.write(f'    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model {ns} -x {x:.3f} -y {y:.3f} -z {Z} -Y {yaw} -param robot_description" />\n')
        # Logical camera bridge (si no es high_res)
        f.write(f'    <group unless="$(arg high_res)">\n')
        f.write(f'      <node pkg="qupa_simulation" type="logical_camera_bridge" name="logical_camera_bridge_{ns}">\n')
        f.write(f'        <param name="robot_namespace" value="{ns}" />\n')
        f.write(f'      </node>\n')
        f.write(f'    </group>\n')
        # Static transform
        f.write(f'    <node pkg="tf2_ros" type="static_transform_publisher" name="world_to_{ns}" args="{x:.3f} {y:.3f} {Z} {yaw} 0 0 world {ns}/base_link" />\n')
        # QUPA monitoring node
        f.write(f'    <node pkg="qupa_behavior" type="qupa_selective_strategy.py" name="qupa_selective_strategy_{i+1}" output="screen">\n')
        f.write(f'      <param name="robot_name" value="{ns}"/>\n')
        f.write(f'    </node>\n')
        f.write(f'  </group>\n\n')

    f.write('</launch>\n')

print("Archivo de lanzamiento generado con {} robots en posiciones aleatorias dentro del círculo.".format(N_ROBOTS))
