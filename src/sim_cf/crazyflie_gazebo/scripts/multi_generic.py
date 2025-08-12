#!/usr/bin/env python3
import sys, time, rospy
import crazyflie  # mismo helper que usa test_high_level.py

def main(prefixes):
    rospy.init_node('multi_high_level', anonymous=True)

    # Crea una instancia por dron: ("cf1","/cf1"), ("cf2","/cf2"), ...
    cfs = [crazyflie.Crazyflie(p, f"/{p}") for p in prefixes]

    # Habilita high level en todos
    for cf in cfs:
        cf.setParam("commander/enHighLevel", 1)

    # Despegue simultáneo
    for cf in cfs:
        cf.takeoff(targetHeight=1.0, duration=2.0)
    time.sleep(3.0)

    # Sepáralos en Y para que no choquen y mantén z=1.0
    center = (len(cfs) - 1) / 2.0
    for i, cf in enumerate(cfs):
        y = (i - center) * 0.8
        cf.goTo(goal=[0.0, y, 1.0], yaw=0.0, duration=2.0, relative=False)
    time.sleep(3.0)

    # Aterrizaje
    for cf in cfs:
        cf.land(targetHeight=0.02, duration=2.0)
    time.sleep(3.0)

if __name__ == "__main__":
    # Usa los prefijos pasados por CLI, p.ej. cf1 cf2 cf3; si no pasas nada, usa cf1
    prefixes = sys.argv[1:] or ["cf1"]
    main(prefixes)

