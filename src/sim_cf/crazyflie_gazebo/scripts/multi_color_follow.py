#!/usr/bin/env python3
import sys, time, threading
import rospy, cv2, numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import crazyflie  # tu helper existente

class ColorWatcher:
    def __init__(self, topic="/camera_1/image_raw", color="red"):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, Image, self.cb, queue_size=1)
        self.lock = threading.Lock()
        self.last_err_x = 0.0
        self.sees_color = False
        self.cx = None; self.cy = None; self.area = 0
        # Rangos HSV (ajústalos si hace falta según la iluminación de Gazebo)
        self.color = color
        self.hsv = {
            "red":  [([0,100,100],[10,255,255]), ([160,100,100],[179,255,255])],
            "green":[([35,100,100],[85,255,255])],
            "blue": [([100,100,100],[130,255,255])]
        }

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_total = np.zeros((h,w), dtype=np.uint8)
        for lo, hi in self.hsv.get(self.color, []):
            mask_total |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask_total = cv2.medianBlur(mask_total, 5)

        cnts, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        with self.lock:
            self.sees_color = False
            self.cx = self.cy = None
            self.area = 0
            self.last_err_x = 0.0
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 300:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"]/M["m00"])
                        cy = int(M["m01"]/M["m00"])
                        self.cx, self.cy = cx, cy
                        self.area = int(area)
                        self.sees_color = True
                        self.last_err_x = (w//2) - cx  # px: derecha negativa, izquierda positiva

    def get_state(self):
        with self.lock:
            return self.sees_color, self.last_err_x, self.area

def main(prefixes, track_color="red"):
    rospy.init_node('multi_color_follow', anonymous=True)
    # Cámara del mundo
    cam_topic = rospy.get_param("~image_topic", "/camera_1/image_raw")
    target_color = rospy.get_param("~color", track_color)

    watcher = ColorWatcher(topic=cam_topic, color=target_color)

    # Instancias Crazyflie por dron
    cfs = [crazyflie.Crazyflie(p, f"/{p}") for p in prefixes]

    # Habilita high-level
    for cf in cfs:
        cf.setParam("commander/enHighLevel", 1)

    # Despega
    for cf in cfs:
        cf.takeoff(targetHeight=1.0, duration=2.0)
    time.sleep(3.0)

    # Colócalos en Y separados y z=1.0
    center = (len(cfs) - 1) / 2.0
    for i, cf in enumerate(cfs):
        y = (i - center) * 0.8
        cf.goTo(goal=[0.0, y, 1.0], yaw=0.0, duration=2.0, relative=False)
    time.sleep(2.5)

    # **Bucle de seguimiento por color** durante N segundos
    T = rospy.get_param("~follow_time_sec", 15.0)
    rate_hz = 5.0
    dt = 1.0 / rate_hz
    k_yaw = rospy.get_param("~k_yaw", 0.0025)   # rad/s por pixel de error (aprox)
    step_fwd = rospy.get_param("~step_fwd", 0.10)  # metros por paso
    step_yaw_max = rospy.get_param("~step_yaw_max", 0.35)  # rad máx por paso
    area_stop = rospy.get_param("~area_stop", 15000) # si el blob es muy grande, no avances

    t0 = time.time()
    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown() and (time.time() - t0) < T:
        sees, err_x, area = watcher.get_state()

        # Traducimos “error en imagen” -> pequeños comandos high-level relativos
        if sees:
            yaw = np.clip(k_yaw * err_x, -step_yaw_max, step_yaw_max)
            # gira para centrar el color
            for cf in cfs:
                cf.goTo(goal=[0.0, 0.0, 0.0], yaw=yaw, duration=dt, relative=True)
            # si está centrado (|err_x| pequeño) y el área es chica -> avanza
            if abs(err_x) < 40 and area < area_stop:
                for cf in cfs:
                    cf.goTo(goal=[step_fwd, 0.0, 0.0], yaw=0.0, duration=dt, relative=True)
        else:
            # No ve color -> quédate quieto (no mandar nada también sirve)
            pass

        r.sleep()

    # Aterriza
    for cf in cfs:
        cf.land(targetHeight=0.02, duration=2.0)
    time.sleep(3.0)

if __name__ == "__main__":
    prefixes = sys.argv[1:] or ["cf1","cf2"]  # por defecto dos drones como en tu launch
    # Puedes cambiar color a "red"/"blue"/"green"
    main(prefixes, track_color="red")

