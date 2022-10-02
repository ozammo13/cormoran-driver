from cormoran import Cormoran2WD
import pygame
import sys


BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

def mapp(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


if __name__ == '__main__':
    robot = Cormoran2WD(['208737A03548', '307F347D3131'],
                        wheelbase=0.0254 * 12 * 2, track_width=0.0254 * 12 * 2)
    robot.connect_to_hardware()
    # robot.start()

    pygame.init()
    screen = pygame.display.set_mode((500, 700))
    pygame.display.set_caption("Cormoran Controller")
    clock = pygame.time.Clock()
    textPrint = TextPrint()

    done = False
    while not done:
        for event in pygame.event.get():  # User did something.
            if event.type == pygame.QUIT:  # If user clicked close.
                done = True  # Flag that we are done so we exit this loop.
            elif event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            elif event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")
        screen.fill(WHITE)
        textPrint.reset()
        joystick_count = pygame.joystick.get_count()
        textPrint.tprint(
            screen, "Number of controllers connected: {}".format(joystick_count))
        textPrint.indent()
        if joystick_count != 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            try:
                jid = joystick.get_instance_id()
            except AttributeError:
                # get_instance_id() is an SDL2 method
                jid = joystick.get_id()
            textPrint.tprint(screen, "Joystick {}".format(jid))
            name = joystick.get_name()
            textPrint.tprint(
                screen, "Joystick name: {}".format(name))
            axes = joystick.get_numaxes()
            axiss = []
            for i in range(axes):
                axis = joystick.get_axis(i)
                axiss.append(axis)
            trigs = (axiss[5]+1)/2 - (axiss[2]+1)/2
            # print(axiss)
            leftyright = axiss[0]
        else:
            trigs = 0
            leftyright = 0
        
        pygame.display.flip()
        clock.tick(60)

        if joystick_count != 0:
            steering = mapp(axiss[0],-1.0, 1.0, -0.5, 0.5)
            # if sys.platform.startswith("linux"):
            #     throttle = axiss[4] - axiss[5]
            # elif sys.platform == "darwin":
            throttle = mapp(axiss[5] - axiss[2], -2, 2, -0.4, 0.4)
            robot.inputs=[steering, throttle]
            # print(axiss)
            print(robot.inputs)
        else:
            robot.inputs=[0.0,0.0]

        robot.run_once()


        
