from open_micro_stage_api import OpenMicroStageInterface
import time

# create interface and connect
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
oms.connect('COM11')
oms.read_device_state_info()
oms.home()
oms.wait_for_stop()


# run this once to calibrate joints
#for i in range(3): oms.calibrate_joint(i, save_result=True)
#oms.calibrate_joint(2, save_result=True)
#oms.read_device_state_info()

# home device
input("press to 0,0,0")
oms.move_to(1.0, 1.0, 1.0, f=10)
oms.wait_for_stop()

#oms.move_to(3.1, 4.1, 5.9, f=26)
#oms.wait_for_stop()

input("press to move")
#oms.move_to(3.1, 4.1, 5.9, f=26)
oms.move_to(0.0, 0.0, 0.0, f=10)
oms.wait_for_stop()

input("press to home")
oms.home()
oms.wait_for_stop()

#oms.move_to(0.0001, 0.0, 0.0, f=10)

# wait for moves to finish
#oms.wait_for_stop()