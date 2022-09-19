from cormoran import Cormoran2WD
import time
robot = Cormoran2WD(['208737A03548', '307F347D3131'],
                    wheelbase=0.0254 * 24, track_width=0.0254 * 24)
# robot.connect_to_hardware()
# robot.start()
while True:
    x=0
    while x < 200:
        x+=1
        robot.inputs=[0.5,1.0]
        feedback = robot.run_once()
        print(feedback)
        time.sleep(1/50)
