import time

from cormoran import Cormoran2WD

robot = Cormoran2WD(['208737A03548', '307F347D3131'],
                    wheelbase=0.0254 * 24, track_width=0.0254 * 36)  # initialises Robot, dont change this
# connects to the robot. if the is removed then the code will run as a simulation
robot.connect_to_hardware()
robot.start()  # starts the robot, remove this line if you remove the connect hardware line
# robot.inputs =[wheel(radians),drive(meters a second)]
cspeed = 0


def Drive(Rot, Speed, Time):
    if (cspeed != Speed):
        Ramp(cspeed, Speed, Rot, 0.1)
    robot.inputs = [Rot, Speed]
    feedback = robot.run_once()
    time.sleep(Time)


def Ramp(currentspeed, desiredSpeed, Rot, timeStep):
    global cspeed
    while (currentspeed != desiredSpeed):
        if (desiredSpeed >= currentspeed):
            currentspeed = currentspeed + 0.01
        elif (desiredSpeed <= currentspeed):
            currentspeed = currentspeed - 0.01
        robot.inputs = [Rot, currentspeed]
        currentspeed = float("{:.2f}".format(currentspeed))
        feedback = robot.run_once()
        time.sleep(timeStep)
        print(currentspeed)

    print("Ramped")
    cspeed = currentspeed


print("start")
Drive(0.0, 0.3, 10)
Drive(0.0, -0.3, 9)
Drive(0.0, 0.0, 0)
