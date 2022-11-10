from cormoran import Cormoran2WD 
import time 
robot = Cormoran2WD(['208737A03548', '307F347D3131'],
                    wheelbase=0.0254 * 24, track_width=0.0254 * 36)  # initialises Robot, dont change this
robot.connect_to_hardware() #connects to the robot. if the is removed then the code will run as a simulation
robot.start()  #starts the robot, remove this line if you remove the connect hardware line

while True: #simple loop

    robot.inputs = [0.5, 0.1]# robot.inputs =[wheel(radians),drive(meters a second)]
    feedback = robot.run_once()
    print(feedback)
    time.sleep(1/50)

