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
        self.max_lin = rospy.get_param("~max_linear", 0.05)
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

    def camera_callback(self, msg):
        if self.state == "exploring":
            min_dist = float('inf')
            best = None
            for model in msg.models:
                # SOLO TAMS que estén en TAREA_A o TAREA_B
                if model.name.startswith("TAM_") and self.tam_status.get(model.name, None) in ("TAREA_A", "TAREA_B"):
                    dx = model.pose.position.x
                    dy = -model.pose.position.y
                    dist = math.hypot(dx, dy)
                    print("[DEBUG] Visto {} en estado {} a distancia {:.2f}".format(
                        model.name, self.tam_status.get(model.name, None), dist))
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

    def obstacle_ahead(self):
        """Devuelve True si hay un obstáculo cerca en el arco frontal."""
        if not self.laser_ranges or len(self.laser_ranges) < 6:
            return False
        # Tres rayos frontales: 3, 4, 5
        window = [self.laser_ranges[3], self.laser_ranges[4], self.laser_ranges[5]]
        return any(r < self.obstacle_dist for r in window if not math.isinf(r))
    
    def obstacle_left(self):
        """Devuelve True si hay obstáculo en el lado izquierdo (índice 6 y 7 en LIDAR de 9 rayos)."""
        if not self.laser_ranges or len(self.laser_ranges) < 8:
            return False
        window = [self.laser_ranges[6], self.laser_ranges[7]]
        return any(r < self.obstacle_dist for r in window if not math.isinf(r))

    def obstacle_right(self):
        """Devuelve True si hay obstáculo en el lado derecho (índice 1 y 2 en LIDAR de 9 rayos)."""
        if not self.laser_ranges or len(self.laser_ranges) < 3:
            return False
        window = [self.laser_ranges[1], self.laser_ranges[2]]
        return any(r < self.obstacle_dist for r in window if not math.isinf(r))

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

    def get_lidar_sectors(self):
        if len(self.laser_ranges) >= 9:
            front_window = [self.laser_ranges[0], self.laser_ranges[8]]
            left_window  = [self.laser_ranges[1], self.laser_ranges[2]]
            right_window = [self.laser_ranges[6], self.laser_ranges[7]]
            back_window  = [self.laser_ranges[4]]
            return {
                'min_front': min([r for r in front_window if not math.isinf(r)], default=float('inf')),
                'min_left':  min([r for r in left_window  if not math.isinf(r)], default=float('inf')),
                'min_right': min([r for r in right_window if not math.isinf(r)], default=float('inf')),
                'min_back': min([r for r in back_window  if not math.isinf(r)], default=float('inf')),
            }
        else:
            return {'min_front': float('inf'), 'min_left': float('inf'), 'min_right': float('inf'), 'min_back': float('inf')}

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()

            if self.state == "exploring":
                sectors = self.get_lidar_sectors()
                min_front = sectors['min_front']
                min_left  = sectors['min_left']
                min_right = sectors['min_right']

                if min_front < self.obstacle_dist:
                    # Frente bloqueado → girar al lado más despejado
                    if min_left > min_right:
                        twist.linear.x = 0.0
                        twist.angular.z = self.max_ang
                        rospy.loginfo("[{}] Explorando: obstáculo al frente, giro a la izquierda".format(self.robot_name))
                    elif min_right > min_left:
                        twist.linear.x = 0.0
                        twist.angular.z = -self.max_ang
                        rospy.loginfo("[{}] Explorando: obstáculo al frente, giro a la derecha".format(self.robot_name))
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = random.choice([-1, 1]) * self.max_ang
                        rospy.loginfo("[{}] Explorando: obstáculo al frente, giro aleatorio".format(self.robot_name))

                elif min_left < self.obstacle_dist:
                    # Izquierda bloqueada → esquiva a la derecha
                    twist.linear.x = self.max_lin / 1
                    twist.angular.z = -self.max_ang / 1.0
                    rospy.loginfo("[{}] Explorando: obstáculo a la izquierda, esquivo a la derecha".format(self.robot_name))

                elif min_right < self.obstacle_dist:
                    # Derecha bloqueada → esquiva a la izquierda
                    twist.linear.x = self.max_lin / 1
                    twist.angular.z = self.max_ang / 1.0
                    rospy.loginfo("[{}] Explorando: obstáculo a la derecha, esquivo a la izquierda".format(self.robot_name))

                else:
                    # Camino libre, avanza
                    twist.linear.x = self.max_lin / 1
                    twist.angular.z = 0.0

                self.cmd_vel_pub.publish(twist)

            elif self.state == "approaching":
                dist = self.distance_to_tam()
                if dist < self.tam_reach_dist:
                    rospy.loginfo("[{}] Alcanzó el {}. Comienza tarea...".format(self.robot_name, self.target_tam))
                    self.set_tam_color((1.0, 0.0, 0.0, 1.0))  # Rojo
                    self.state = "working"
                    self.last_task_time = rospy.Time.now().to_sec()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)

                else:
                    sectors = self.get_lidar_sectors()
                    min_front = sectors['min_front']
                    min_left  = sectors['min_left']
                    min_right = sectors['min_right']

                    distances = {'front': min_front, 'left': min_left, 'right': min_right}
                    closest_dir = min(distances, key=distances.get)
                    closest_val = distances[closest_dir]

                    if closest_dir == 'front' and closest_val < self.approach_obstacle_dist:
                        if min_left > min_right:
                            twist.linear.x = 0.0
                            twist.angular.z = self.max_ang
                            rospy.loginfo("[{}] Obstáculo más cercano al frente, giro a la izquierda para evitar {}".format(self.robot_name, self.target_tam))
                        elif min_right > min_left:
                            twist.linear.x = 0.0
                            twist.angular.z = -self.max_ang
                            rospy.loginfo("[{}] Obstáculo más cercano al frente, giro a la derecha para evitar {}".format(self.robot_name, self.target_tam))
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = random.choice([-1, 1]) * self.max_ang
                            rospy.loginfo("[{}] Obstáculo al frente, ambos lados bloqueados, giro aleatorio hacia {}".format(self.robot_name, self.target_tam))
                        self.cmd_vel_pub.publish(twist)

                    elif closest_dir == 'left' and closest_val < self.approach_obstacle_dist:
                        twist.linear.x = self.max_lin / 2
                        twist.angular.z = -self.max_ang / 2.0
                        rospy.loginfo("[{}] Obstáculo a la izquierda, esquivo a la derecha hacia {}".format(self.robot_name, self.target_tam))
                        self.cmd_vel_pub.publish(twist)

                    elif closest_dir == 'right' and closest_val < self.approach_obstacle_dist:
                        twist.linear.x = self.max_lin / 2
                        twist.angular.z = self.max_ang / 2.0
                        rospy.loginfo("[{}] Obstáculo a la derecha, esquivo a la izquierda hacia {}".format(self.robot_name, self.target_tam))
                        self.cmd_vel_pub.publish(twist)

                    else:
                        dx, dy, _ = self.target_pose
                        dy = -dy
                        ang = math.atan2(dy, dx)
                        twist.linear.x = self.max_lin
                        twist.angular.z = max(-self.max_ang, min(self.max_ang, self.Kp_ang * ang))
                        self.cmd_vel_pub.publish(twist)
                        print("[DEBUG] Visto {} en estado {} a distancia {:.2f}".format(
                            self.target_tam, self.tam_status.get(self.target_tam, None), dist))

            elif self.state == "working":
                if (rospy.Time.now().to_sec() - self.last_task_time) > self.task_time:
                    rospy.loginfo("[{}] Termina tarea, comienza retirada...".format(self.robot_name))
                    self.state = "retreating"
                    self.task_done = True

            elif self.state == "retreating":
                dist = self.distance_to_tam()
                ranges = self.laser_ranges

                if len(ranges) < 9:
                    rospy.logwarn("[{}] LIDAR incompleto, no puedo retirarme.".format(self.robot_name))
                    return

                # Índices clave
                front_index_1 = 0
                front_index_2 = 8
                left_45_index = 1
                right_45_index = 7
                back_index = 4

                front_clear = math.isinf(ranges[front_index_1]) and math.isinf(ranges[front_index_2])
                back_detected = not math.isinf(ranges[back_index])
                left_wide =  math.isinf(ranges[1]) or ranges[1] > 0.15
                right_wide =  math.isinf(ranges[7]) or ranges[7] > 0.15

                # Paso 0: girar hasta encontrar salida
                if self.retreat_step == 0:
                    # if front_clear and back_detected and (left_clear or right_clear):
                    if front_clear and left_wide and right_wide and back_detected:
                        rospy.loginfo("[{}] Se detecta salida, comienza a avanzar.".format(self.robot_name))
                        self.retreat_step = 1
                        self.retreat_start_time = rospy.Time.now().to_sec()
                        self.prev_left_45 = ranges[1]
                        self.prev_right_45 = ranges[7]
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = self.max_ang  # Giro continuo a la izquierda
                        self.cmd_vel_pub.publish(twist)

                # Paso 1: avanzar y corregir rumbo si se acerca a obstáculos laterales
                elif self.retreat_step == 1:
                    if dist > self.retreat_dist :
                        if self.task_done:
                            rospy.loginfo("[{}] Se alejó del TAM. Vuelve a explorar.".format(self.robot_name))
                            self.set_tam_color((0.8, 0.8, 0.8, 1.0))  # Gris claro (antes blanco)`
                        self.state = "exploring"
                        self.target_tam = None
                        self.target_pose = None
                        self.retreat_step = 0
                        self.retreat_start_time = None
                    else:
                        # Corrección de trayectoria si se está acercando a un obstáculo lateral
                        angular_corr = 0.0
                        left_now = ranges[left_45_index] if not math.isinf(ranges[left_45_index]) else None
                        right_now = ranges[right_45_index] if not math.isinf(ranges[right_45_index]) else None
                        
                        if (ranges[front_index_1] or ranges[front_index_2]) < 0.15:
                            twist.linear.x = - self.max_lin/2

                        if self.prev_left_45 and left_now and left_now < self.prev_left_45 - 0.01:
                            angular_corr += -self.max_ang / 1.0  # Gira a la derecha
                        if self.prev_right_45 and right_now and right_now < self.prev_right_45 - 0.01:
                            angular_corr += self.max_ang / 1.0  # Gira a la izquierda

                        twist.linear.x = self.max_lin / 1.5
                        twist.angular.z = angular_corr
                        self.cmd_vel_pub.publish(twist)

                        # Actualizar los valores previos
                        self.prev_left_45 = left_now
                        self.prev_right_45 = right_now



if __name__ == "__main__":
    rospy.init_node("tam_explorer_node")
    explorer = TAMExplorer()
    explorer.spin()
