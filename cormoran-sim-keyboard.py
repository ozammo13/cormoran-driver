from cormoran import Cormoran2WD
import time
robot = Cormoran2WD(['208737A03548', '307F347D3131'],
                    wheelbase=0.0254 * 24, track_width=0.0254 * 24)
# robot.connect_to_hardware()
robot.start()
while True:
    robot.input=[0,0]
    time.sleep(1)
    robot.input = [25,1]
    time.sleep(1)
    robot.input=[0,0]
    time.sleep(1)
    robot.input = [-25,-1]
    time.sleep(1)
