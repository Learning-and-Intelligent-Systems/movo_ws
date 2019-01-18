from uber_controller import Uber
import rospy

rospy.init_node("testuber")
uc = Uber()
rospy.sleep(1)
uc.move_arm_to_side('l')
uc.move_arm_to_side('r')
uc.move_arm_to_front('l')
uc.move_arm_to_front('r')


uc.command_torso(0.3, 5, True)
uc.look_down_center()
uc.look_forward()

