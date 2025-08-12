#!/usr/bin/env python3
import math
import rospy
import roslaunch
from roslaunch.rlutil import resolve_launch_arguments, get_or_generate_uuid
from roslaunch.parent import ROSLaunchParent

def compute_pose(layout, idx, n, radius, z0):
    """Devuelve (x,y,z) únicos por dron."""
    if layout == "line":
        # separados 0.8m en Y centrados
        center = (n - 1) / 2.0
        return (0.0, (idx - center) * 0.8, z0)
    elif layout == "grid":
        cols = int(math.ceil(math.sqrt(n)))
        rows = int(math.ceil(n / float(cols)))
        r = idx // cols
        c = idx % cols
        return ((c - (cols-1)/2.0)*0.8, (r - (rows-1)/2.0)*0.8, z0)
    # default: círculo
    ang = 2.0 * math.pi * idx / max(n,1)
    return (radius * math.cos(ang), radius * math.sin(ang), z0)

def main():
    rospy.init_node('swarm_spawner')

    nb = int(rospy.get_param('~nbQuads', 2))
    prefix = rospy.get_param('~cfPrefix', 'cf')
    first = int(rospy.get_param('~firstIndex', 1))
    mav_name = rospy.get_param('~mav_name', 'crazyflie')

    layout = rospy.get_param('~layout', 'circle')
    radius = float(rospy.get_param('~radius', 1.0))
    z0 = float(rospy.get_param('~z0', 0.03))

    # Ruta del launch que ya usas para spawnear un dron
    launch_file = resolve_launch_arguments(['crazyflie_gazebo', 'launch/spawn_mav.launch'])[0]

    uuid = get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Mantener referencias para apagado limpio
    parents = []
    try:
        for k in range(nb):
            i = first + k
            ns = f"{prefix}{i}"
            tf_prefix = ns

            x, y, z = compute_pose(layout, k, nb, radius, z0)

            # Nota: spawn_mav.launch ya define defaults para model con xacro;
            # si tu spawn_mav exige 'model', puedes añadir el arg aquí.
            args = [
                f"mav_name:={mav_name}",
                f"tf_prefix:={tf_prefix}",
                f"x:={x}",
                f"y:={y}",
                f"z:={z}",
                # Colores opcionales: cicla por estética
                f"color_prop_front:={'Blue Red Green White'.split()[k % 4]}",
                f"color_prop_back:={'Blue Red Green White'.split()[k % 4]}",
            ]

            # Ejecutar bajo namespace del dron
            parent = ROSLaunchParent(uuid, [(launch_file, args)], is_core=False, verbose=False)
            parent.start()
            rospy.loginfo(f"[swarm_spawner] Spawned {ns} at x={x:.2f} y={y:.2f} z={z:.2f}")
            parents.append(parent)

        rospy.loginfo(f"[swarm_spawner] Listo: {nb} Crazyflie(s) con prefijo '{prefix}', desde {first}.")
        rospy.spin()

    finally:
        for p in parents:
            try:
                p.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()

