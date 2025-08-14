#!/usr/bin/env python

import rospy
from qupa_msgs.msg import ModelsList
from std_msgs.msg import String

class TAMTaskMonitor:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.camera_topic = "/{}/logical_camera".format(robot_name)
        self.visible_tams = set()
        self.tam_tasks = {}
        self.tam_subs = {}
        
        rospy.Subscriber(self.camera_topic, ModelsList, self.camera_callback)
        rospy.loginfo("TAMTaskMonitor iniciado para robot [%s] en topic [%s]", self.robot_name, self.camera_topic)

    def camera_callback(self, msg):
        current_tams = set()
        for model in msg.models:
            if model.name.startswith("TAM_"):
                current_tams.add(model.name)
                if model.name not in self.tam_subs:
                    topic = "/" + model.name + "/task"
                    self.tam_subs[model.name] = rospy.Subscriber(topic, String, self.make_task_callback(model.name))
        self.visible_tams = current_tams
        # self.print_status()

    def make_task_callback(self, tam_name):
        def callback(msg):
            self.tam_tasks[tam_name] = msg.data
        return callback

    def print_status(self):
        print("TAMs visibles y sus tareas para {}:".format(self.robot_name))
        for tam in sorted(self.visible_tams):
            tarea = self.tam_tasks.get(tam, "DESCONOCIDA")
            print("  {}: {}".format(tam, tarea))
        print("----")

if __name__ == '__main__':
    rospy.init_node("tam_task_monitor")
    robot_name = rospy.get_param('~robot_name', 'qp_1')
    monitor = TAMTaskMonitor(robot_name)
    rospy.spin()
