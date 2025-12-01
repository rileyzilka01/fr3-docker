import panda_py
import time

ROBOT_IP = "192.168.1.105"
USER = "jagersand"
PASS = "D121wsnf!"

desk = panda_py.Desk(ROBOT_IP, USER, PASS, platform="fr3")
desk.take_control(force=True)
print(desk.has_control())
desk.unlock()
desk.activate_fci()


time.sleep(5)

desk.deactivate_fci()
desk.lock()
desk.release_control()