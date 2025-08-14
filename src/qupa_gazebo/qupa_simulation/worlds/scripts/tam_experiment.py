import numpy as np

def generar_distribucion_hexagonal_tams(
    tam_interno=0.12,
    borde=0.01,
    n_tams_lado=4,
    cantidad_total=24,
    escala=2.60,
    model_uri="model://TAM"
):
    tam_total = escala * (tam_interno + 2 * borde)
    tam_spacing = tam_total

    angle_between_sides = np.pi / 3  # 60 grados
    sdf_includes = []
    index = 0
    for side in range(6):
        angle = side * angle_between_sides
        dx = np.cos(angle) * tam_spacing
        dy = np.sin(angle) * tam_spacing
        start_x = 0
        start_y = 0
        if side > 0:
            prev = sdf_includes[-1]['pose']
            start_x = prev[0] + dx
            start_y = prev[1] + dy

        for i in range(n_tams_lado):
            if side == 0 and i == 0:
                x, y = 0, 0
            else:
                x = start_x + i * dx
                y = start_y + i * dy
            
            yaw = angle +np.pi/2  # orientación hacia afuera
            sdf_includes.append({
                'name': f"TAM_{index}",
                'pose': (x, y, 0, 0, 0, yaw)
            })
            index += 1
            if index == cantidad_total:
                break
        if index == cantidad_total:
            break

    #Centrar la distribución en el origen
    x_coords = [p['pose'][0] for p in sdf_includes]
    y_coords = [p['pose'][1] for p in sdf_includes]
    x_center = sum(x_coords) / len(x_coords)
    y_center = sum(y_coords) / len(y_coords)

    for p in sdf_includes:
        p['pose'] = (
            p['pose'][0] - x_center,
            p['pose'][1] - y_center,
            p['pose'][2],
            p['pose'][3],
            p['pose'][4],
            p['pose'][5]
        )

    tam_apertura_offset = 0.06

    for i in range(len(sdf_includes)):
        lado_actual = i // n_tams_lado
        if lado_actual in [0, 3]:
            continue  # NO mover TAMs horizontales
        x, y, z, r, p, yaw = sdf_includes[i]['pose']
        x -= np.cos(yaw) * tam_apertura_offset
        y -= np.sin(yaw) * tam_apertura_offset
        sdf_includes[i]['pose'] = (x, y, z, r, p, yaw)


    return sdf_includes

def generar_sdf(includes, model_uri="model://TAM"):
    include_lines = []
    for inc in includes:
        pose = inc['pose']
        pose_str = " ".join([f"{v:.6f}" for v in pose])
        include_lines.append(f"""    <include>
      <uri>{model_uri}</uri>
      <name>{inc['name']}</name>
      <pose>{pose_str}</pose>
    </include>""")
    return "\n".join(include_lines)

def plantilla_mundo_completo(includes_block):
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
    includes = generar_distribucion_hexagonal_tams()
    sdf_includes = generar_sdf(includes)
    mundo_sdf = plantilla_mundo_completo(sdf_includes)
    with open("src/qupa_gazebo/qupa_simulation/worlds/tam_hex.world", "w") as f:
        f.write(mundo_sdf)
    print("Archivo completo SDF generado como 'tam_hex.world'")
