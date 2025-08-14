import numpy as np

def generar_lado_hexagonal(n_por_lado, tam_total, clearance=0.01):
    """Genera un solo lado recto del hexágono desde la punta"""
    poses = []
    step = tam_total + clearance
    for i in range(n_por_lado):
        x = i * step
        y = 0
        poses.append((x, y, 0, 0, 0, 0))
    return poses

def rotar_puntos(poses, angulo_rad):
    """Rota cada punto alrededor del origen por un ángulo"""
    rotated = []
    for i, (x, y, z, r, p, yaw) in enumerate(poses):
        x_rot = np.cos(angulo_rad) * x - np.sin(angulo_rad) * y
        y_rot = np.sin(angulo_rad) * x + np.cos(angulo_rad) * y
        rotated.append((x_rot, y_rot, z, r, p, yaw + angulo_rad))
    return rotated

def generar_hexagono_completo(n_por_lado, tam_total, clearance=0.01):
    """Genera las 24 poses de TAMs en forma de hexágono cerrado"""
    poses_total = []
    lado_base = generar_lado_hexagonal(n_por_lado, tam_total, clearance)
    for i in range(6):
        ang = i * np.pi / 3  # 60°
        lado_rotado = rotar_puntos(lado_base, ang)
        poses_total.extend(lado_rotado)
    return poses_total

def generar_sdf_includes(poses, model_uri="model://TAM"):
    """Genera bloques <include> para cada TAM"""
    lines = []
    for i, pose in enumerate(poses):
        pose_str = " ".join([f"{v:.6f}" for v in pose])
        lines.append(f"""    <include>
      <uri>{model_uri}</uri>
      <name>TAM_{i}</name>
      <pose>{pose_str}</pose>
    </include>""")
    return "\n".join(lines)

def plantilla_mundo_completo(includes_block):
    """Plantilla del mundo SDF completo con plano y luz"""
    return f"""<?xml version='1.7'?>
<sdf version='1.7'>
  <world name='tam_hex_world'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/White</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

{includes_block}

  </world>
</sdf>"""

if __name__ == "__main__":
    tam_interno = 0.12
    borde = 0.01
    escala = 3.0
    clearance = 0.01  # espacio adicional para evitar solapamiento
    tam_total = escala * (tam_interno + 2 * borde)

    poses = generar_hexagono_completo(n_por_lado=4, tam_total=tam_total, clearance=clearance)
    includes_block = generar_sdf_includes(poses)
    sdf_world = plantilla_mundo_completo(includes_block)

    with open("src/qupa_gazebo/qupa_simulation/worlds/tam_hex_modular.world", "w") as f:
        f.write(sdf_world)

    print("Archivo SDF generado: tam_hex_modular.world")
