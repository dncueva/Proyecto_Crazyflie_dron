#!/usr/bin/env python

import rospy
import math
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from qupa_msgs.msg import ModelsList

class TAMExplorer:
    def __init__(self):
        # Parámetros
        self.robot_name = rospy.get_param("~robot_name", "qp_1")
        self.num_tams = rospy.get_param("~num_tams", 24)
        self.max_lin = rospy.get_param("~max_linear", 0.06)
        self.max_ang = rospy.get_param("~max_angular", 0.5)
        self.obstacle_dist = rospy.get_param("~scan_min_dist", 0.5)
        self.approach_obstacle_dist = rospy.get_param("~approach_obstacle_dist", 0.1)
        self.tam_reach_dist = rospy.get_param("~tam_reach_dist", 0.12)
        self.retreat_dist = rospy.get_param("~retreat_dist", 0.25)
        self.task_time = rospy.get_param("~task_time", 5.0)
        self.Kp_ang = rospy.get_param("~kp_ang", 1.0)

        # Estado
        self.state = "exploring"
        self.laser_ranges = []
        self.cmd_vel_pub = rospy.Publisher("/{}/cmd_vel".format(self.robot_name), Twist, queue_size=1)
        self.tam_status = {}      # TAM_X -> String ('TAREA_A', 'TAREA_B', etc)
        self.target_tam = None    # nombre del TAM objetivo
        self.target_pose = None   # dx, dy, name
        self.last_task_time = None
        self.task_done = False

        self.turning = False
        self.turn_start = None
        self.turn_dir = 1
        self.retreat_step = 0   # 0: hay que girar, 1: hay que avanzar
        self.retreat_start_time = None

        self.approach_last_check_time = None
        self.approach_last_dist = None
        self.approach_timeout = 4.0  # segundos
        self.approach_delta = 0.02   # metros, umbral de cambio mínimo
        self.sectors = {
            'NORTH': float('inf'),
            'NORTHWEST': float('inf'),
            'WEST': float('inf'),
            'EAST': float('inf'),
            'NORTHEAST': float('inf'),
            'SOUTH': float('inf')
        }

        # Subscripciones a task de TAMs
        for i in range(self.num_tams):
            tam_name = "TAM_{}".format(i)
            rospy.Subscriber("/{}/task".format(tam_name), String, self.make_tam_task_callback(tam_name))
        rospy.Subscriber("/{}/logical_camera".format(self.robot_name), ModelsList, self.camera_callback)
        rospy.Subscriber("/{}/scan".format(self.robot_name), LaserScan, self.scan_callback)

    def make_tam_task_callback(self, tam_name):
        def callback(msg):
            self.tam_status[tam_name] = msg.data
            # print("[DEBUG TASK] {} -> {}".format(tam_name, msg.data))
        return callback

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges

        if len(msg.ranges) >= 9:
            north = [msg.ranges[0], msg.ranges[8]]
            west  = [msg.ranges[6]]
            northwest = [msg.ranges[7]]
            east = [ msg.ranges[2]]
            northeast = [msg.ranges[1]]
            south  = [msg.ranges[4]]

            self.sectors = {
                'NORTH': min([r for r in north if not math.isinf(r)], default=float('inf')),
                'WEST':  min([r for r in west  if not math.isinf(r)], default=float('inf')),
                'NORTHWEST':  min([r for r in northwest  if not math.isinf(r)], default=float('inf')),
                'EAST': min([r for r in east if not math.isinf(r)], default=float('inf')),
                'NORTHEAST': min([r for r in northeast if not math.isinf(r)], default=float('inf')),
                'SOUTH': min([r for r in south  if not math.isinf(r)], default=float('inf')),
            }

    def camera_callback(self, msg):
        if self.state == "exploring":
            min_dist = float('inf')
            best = None

            # Parámetros de visibilidad
            fov = math.radians(144)  # ±72°
            min_visible_dist = 0.3   # No ve si está muy cerca (< 30 cm)
            max_visible_dist = 2.0   # Opcional: no ve más allá de 2 metros

            for model in msg.models:
                if model.name.startswith("TAM_") and self.tam_status.get(model.name, None) in ("TAREA_A", "TAREA_B"):
                    dx = model.pose.position.x
                    dy = -model.pose.position.y
                    dist = math.hypot(dx, dy)
                    angle_to_tam = math.atan2(dy, dx)

                    # Filtro angular y de distancia
                    if abs(angle_to_tam) <= fov / 2 and min_visible_dist <= dist <= max_visible_dist:
                        # print(f"[DEBUG] {model.name} angle={math.degrees(angle_to_tam):.1f}, dist={dist:.2f}")
                        if dist < min_dist:
                            min_dist = dist
                            best = (dx, dy, model.name)

            if best:
                rospy.loginfo("[{}] TAM activo encontrado: {}".format(self.robot_name, best[2]))
                self.state = "approaching"
                self.target_tam = best[2]
                self.target_pose = best


            elif self.state in ("approaching", "retreating"):
                # Buscar solo el TAM objetivo, para actualizar su posición
                for model in msg.models:
                    if model.name == self.target_tam:
                        dx = model.pose.position.x
                        dy = model.pose.position.y
                        self.target_pose = (dx, dy, model.name)
                        break

    def distance_to_tam(self):
        if self.target_pose:
            dx, dy, _ = self.target_pose
            dy = -dy
            return math.hypot(dx, dy)
        return float('inf')

    def set_tam_color(self, color):
        pub = rospy.Publisher("/{}/set_color".format(self.target_tam), ColorRGBA, queue_size=1, latch=True)
        msg = ColorRGBA()
        msg.r, msg.g, msg.b, msg.a = color
        rospy.sleep(0.2)
        for _ in range(5):
            pub.publish(msg)
            rospy.sleep(0.1)

    def get_turn_duration(self):
        # Tiempo para girar 90 grados (pi/2 rad) a velocidad máxima
        return abs((math.pi/2) / self.max_ang)

    def spin(self):
        rate = rospy.Rate(15)
        if not hasattr(self, 'sectors'):
            rate.sleep()  
        

        while not rospy.is_shutdown():
            twist = Twist()

            if self.state == "exploring":
                min_north     = self.sectors['NORTH']
                min_west      = self.sectors['WEST']
                min_east     = self.sectors['EAST']
                min_northwest = self.sectors['NORTHWEST']
                min_northeast = self.sectors['NORTHEAST']
                min_south      = self.sectors['SOUTH']

                if min_north < self.obstacle_dist:
                    # Frente bloqueado → girar al lado más despejado
                    if min_northwest > min_northeast:
                        twist.linear.x = 0
                        twist.angular.z = -self.max_ang
                        #rospy.loginfo("[{}] Explorando: obstáculo al frente, giro a la izquierda".format(self.robot_name))
                    elif min_northeast > min_northwest:
                        twist.linear.x =  0
                        twist.angular.z = self.max_ang
                        #rospy.loginfo("[{}] Explorando: obstáculo al frente, giro a la derecha".format(self.robot_name))
                    elif not math.isinf(min_south):
                        twist.linear.x = -self.max_lin/1.5
                        twist.angular.z = -self.max_ang/2
                        #rospy.loginfo("[{}] Explorando: obstáculo al frente, giro aleatorio".format(self.robot_name))

                elif min_northwest < self.obstacle_dist or min_west < self.obstacle_dist:
                    # Izquierda bloqueada → esquiva a la derecha
                    twist.linear.x = self.max_lin / 2
                    twist.angular.z = -self.max_ang / 1.0
                    #rospy.loginfo("[{}] Explorando: obstáculo a la izquierda, esquivo a la derecha".format(self.robot_name))

                elif min_northeast < self.obstacle_dist or min_east < self.obstacle_dist:
                    # Derecha bloqueada → esquiva a la izquierda
                    twist.linear.x = self.max_lin / 2
                    twist.angular.z = self.max_ang / 1.0
                    #rospy.loginfo("[{}] Explorando: obstáculo a la derecha, esquivo a la izquierda".format(self.robot_name))

                else:
                    # Camino libre, avanza
                    twist.linear.x = self.max_lin / 1
                    twist.angular.z = 0.0

                self.cmd_vel_pub.publish(twist)

            elif self.state == "approaching":
                dist = self.distance_to_tam()
                min_north     = self.sectors['NORTH']
                min_west      = self.sectors['WEST']
                min_east     = self.sectors['EAST']
                min_northwest = self.sectors['NORTHWEST']
                min_northeast = self.sectors['NORTHEAST']
                min_south      = self.sectors['SOUTH']
                # Comprobación de atasco por falta de progreso
                current_time = rospy.Time.now().to_sec()
                if self.approach_last_check_time is None:
                    self.approach_last_check_time = current_time
                    self.approach_last_dist = dist
                else:
                    if abs(dist - self.approach_last_dist) < self.approach_delta:
                        if current_time - self.approach_last_check_time > self.approach_timeout:
                            rospy.logwarn("[{}] Atascado en aproximación a {}. Cambiando a retreating.".format(self.robot_name, self.target_tam))
                            self.state = "retreating"
                            self.task_done = False  # No alcanzó la tarea
                            self.retreat_step = 0
                            self.retreat_start_time = None
                            continue  # Saltarse el resto del bloque "approaching"
                    else:
                        self.approach_last_check_time = current_time
                        self.approach_last_dist = dist
                

                if dist < self.tam_reach_dist:
                    rospy.loginfo("[{}] Alcanzó el {}. Comienza tarea...".format(self.robot_name, self.target_tam))
                    self.set_tam_color((1.0, 0.0, 0.0, 1.0))  # Rojo
                    self.state = "working"
                    self.last_task_time = rospy.Time.now().to_sec()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)

                else:

                    distances = {'front': min_north, 'left': min_northwest, 'right': min_northeast}
                    closest_dir = min(distances, key=distances.get)
                    closest_val = distances[closest_dir]

                    if closest_dir == 'front' and closest_val < self.approach_obstacle_dist:
                        if min_northwest > min_northeast:
                            twist.linear.x = 0.0
                            twist.angular.z = self.max_ang
                            #rospy.loginfo("[{}] Obstáculo más cercano al frente, giro a la izquierda para evitar {}".format(self.robot_name, self.target_tam))
                        elif min_northeast > min_northwest:
                            twist.linear.x = 0.0
                            twist.angular.z = -self.max_ang
                            #rospy.loginfo("[{}] Obstáculo más cercano al frente, giro a la derecha para evitar {}".format(self.robot_name, self.target_tam))
                        else:
                            twist.linear.x = -self.max_lin/2
                            twist.angular.z = 0
                            #rospy.loginfo("[{}] Obstáculo al frente, ambos lados bloqueados, giro aleatorio hacia {}".format(self.robot_name, self.target_tam))
                        self.cmd_vel_pub.publish(twist)

                    elif closest_dir == 'left' and closest_val < self.approach_obstacle_dist+0.03:
                        twist.linear.x = 0
                        twist.angular.z = -self.max_ang / 1
                        #rospy.loginfo("[{}] Obstáculo a la izquierda, esquivo a la derecha hacia {}".format(self.robot_name, self.target_tam))
                        self.cmd_vel_pub.publish(twist)

                    elif closest_dir == 'right' and closest_val < self.approach_obstacle_dist+0.03:
                        twist.linear.x = 0
                        twist.angular.z = self.max_ang / 1
                        #rospy.loginfo("[{}] Obstáculo a la derecha, esquivo a la izquierda hacia {}".format(self.robot_name, self.target_tam))
                        self.cmd_vel_pub.publish(twist)

                    else:
                        dx, dy, _ = self.target_pose
                        dy = -dy
                        ang = math.atan2(dy, dx)
                        twist.linear.x = self.max_lin
                        twist.angular.z = max(-self.max_ang, min(self.max_ang, self.Kp_ang * ang))
                        self.cmd_vel_pub.publish(twist)
                        #print("[DEBUG] Visto {} en estado {} a distancia {:.2f}".format(self.target_tam, self.tam_status.get(self.target_tam, None), dist))

            elif self.state == "working":
                if (rospy.Time.now().to_sec() - self.last_task_time) > self.task_time:
                    rospy.loginfo("[{}] Termina tarea, comienza retirada...".format(self.robot_name))
                    self.state = "retreating"
                    self.task_done = True

            elif self.state == "retreating":
                min_north     = self.sectors['NORTH']
                min_west      = self.sectors['WEST']
                min_east     = self.sectors['EAST']
                min_northwest = self.sectors['NORTHWEST']
                min_northeast = self.sectors['NORTHEAST']
                min_south      = self.sectors['SOUTH']
                dist = self.distance_to_tam()

                # Paso 0: girar hasta encontrar salida
                if self.retreat_step == 0:
                    front_clear = math.isinf(min_north) 
                    back_detected = min_south < float('inf')
                    left_wide = (min_northwest > 0.15) 
                    right_wide = (min_northeast > 0.15)


                    if front_clear and back_detected and left_wide and right_wide:
                        rospy.loginfo("[{}] Se detecta salida, comienza a avanzar.".format(self.robot_name))
                        self.retreat_step = 1
                        self.retreat_start_time = rospy.Time.now().to_sec()
                        self.prev_left_45 = min_northwest
                        self.prev_right_45 = min_northeast
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = self.max_ang /2 # Giro continuo a la izquierda
                        self.cmd_vel_pub.publish(twist)

                # Paso 1: avanzar y corregir rumbo si se acerca a obstáculos laterales
                elif self.retreat_step == 1:
                    if dist > self.retreat_dist:
                        if self.task_done:
                            rospy.loginfo("[{}] Se alejó del TAM. Vuelve a explorar.".format(self.robot_name))
                            self.set_tam_color((0.8, 0.8, 0.8, 1.0))  # Gris claro
                        self.state = "exploring"
                        self.target_tam = None
                        self.target_pose = None
                        self.retreat_step = 0
                        self.retreat_start_time = None
                    else:
                        angular_corr = 0.0
                        left_now = min_northwest
                        right_now = min_northeast

                        # Retroceder si el frente está bloqueado
                        if min_north < 0.2:
                            twist.linear.x = -self.max_lin/2
                        else:
                            twist.linear.x = self.max_lin / 1.50  # Avanza lento si puede

                        if self.prev_left_45 and left_now < self.prev_left_45 - 0.01:
                            angular_corr += self.max_ang/2  # Gira a la derecha
                        if self.prev_right_45 and right_now < self.prev_right_45 - 0.01:
                            angular_corr += -self.max_ang/2   # Gira a la izquierda

                        twist.angular.z = angular_corr
                        self.cmd_vel_pub.publish(twist)

                        # Actualizar valores previos
                        self.prev_left_45 = left_now
                        self.prev_right_45 = right_now


if __name__ == "__main__":
    rospy.init_node("tam_explorer_node")
    explorer = TAMExplorer()
    explorer.spin()
