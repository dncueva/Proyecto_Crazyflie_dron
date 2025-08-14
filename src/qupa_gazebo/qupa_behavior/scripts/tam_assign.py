#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import ColorRGBA, String
from threading import Timer

class TAMTaskAssigner:
    def __init__(self, num_tams=24):
        self.num_tams = num_tams
        self.tam_pubs = {}
        self.task_map = {}  # Guardar la tarea actual de cada TAM
        self.reassign_timers = {}  # Para temporizadores de reasignación
        self.tams_aleatorios = random.sample(range(24), num_tams)

        # Inicializa publishers y subscribers para todos los TAMs
        for i in self.tams_aleatorios:
            tam_name = "TAM_{}".format(i)
            topic = "/" + tam_name + "/set_color"
            self.tam_pubs[tam_name] = rospy.Publisher(topic, ColorRGBA, queue_size=1, latch=True)
            # Subscriber para el color publicado (loopback)
            rospy.Subscriber(topic, ColorRGBA, self.make_color_callback(tam_name))
        
        # Al iniciar, asigna tarea aleatoria a cada TAM
        rospy.sleep(1.0)  # Espera un poco para que los publishers estén listos
        self.assign_random_tasks()

    def assign_random_tasks(self, tam_list=None):
        # Si tam_list es None, asigna a todos
        if tam_list is None:
            tam_list = self.tam_pubs.keys()
        for tam_name in tam_list:
            pub = self.tam_pubs[tam_name]
            prev_task = self.task_map.get(tam_name, None)
            
            # Elige una tarea distinta a la anterior
            possible_tasks = ["A", "B"]
            if prev_task in possible_tasks:
                possible_tasks.remove(prev_task)
            new_task = random.choice(possible_tasks)

            self.task_map[tam_name] = new_task
            color_msg = self.get_color_msg_for_task(new_task)
            pub.publish(color_msg)

    def get_color_msg_for_task(self, tarea):
        msg = ColorRGBA()
        msg.a = 1.0
        if tarea == "A":   # Verde
            msg.r = 0.0
            msg.g = 1.0
            msg.b = 0.0
        elif tarea == "B": # Azul
            msg.r = 0.0
            msg.g = 0.0
            msg.b = 1.0
        else:  # Standby
            msg.r = 1.0
            msg.g = 1.0
            msg.b = 1.0
        return msg

    def make_color_callback(self, tam_name):
        def callback(msg):
            if msg.r > 0.8 and msg.g > 0.8 and msg.b > 0.8:
                if tam_name in self.reassign_timers:
                    self.reassign_timers[tam_name].cancel()
                
                # Seleccionar otro TAM distinto
                other_tams = [name for name in self.tam_pubs.keys() if name != tam_name]
                if not other_tams:
                    return  # No hay otro TAM disponible
                
                selected_tam = random.choice(other_tams)
                self.reassign_timers[tam_name] = Timer(12.0, self.reassign_task, [selected_tam])
                self.reassign_timers[tam_name].start()
        return callback

    def reassign_task(self, tam_name):
        self.assign_random_tasks([tam_name])
        #rospy.loginfo("[TaskAssigner] Nueva tarea asignada a {}".format(tam_name))

if __name__ == '__main__':
    rospy.init_node("tam_task_assigner")
    num_tams = rospy.get_param("~num_tams", 24)   # Por defecto 24
    assigner = TAMTaskAssigner(num_tams=num_tams)
    rospy.spin()
