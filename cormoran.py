#!/usr/bin/env python3
"""
Root driver code for connecting to Cormoran Robot. Can be run as script or
imported into other code by adding 'from cormoran import Cormoran'
| L = Left  | F = Front | S = Steer | P = Power   | A = Signal 1 |
| R = Right | R = Rear  | D = Drive | E = Encoder | B = Signal 2 |
"""
import math
# import os
import signal
import sys
import time
import numpy as np

# import inputs
import odrive
import serial
# import odrive.enums

import threading

if sys.platform.startswith("linux"):
    print(f'{sys.platform} detected')
elif sys.platform == "darwin":
    print(f'{sys.platform} detected')
elif sys.platform == "win32":
    print(f'{sys.platform} detected')


class Wheel(object):
    """
    sub driver for one wheel. connects to one odrive with motor 0 being the steering motor and motor 1 being the
    drive motor. Directions configured in wiring.
    """

    def __init__(self, odrvser):
        self.odrive_serial = odrvser
        self.simulation = True
        self.drive_gearing = 20 * 39 / 15
        self.drive_setpoint = 0
        self.drive_actual = 0
        self.drive_feedback = 0
        self.steering_gearing = 20 * 50 / 12
        self.steering_setpoint = 0
        self.steering_actual = 0
        self.steering_feedback = 0
        self.last_time = 0
        self.total_dist = 0

    def connect_to_hardware(self):
        """established physical connection to motor driver hardware
        if this function is called simulation mode will be turned off"""
        self.odrive = odrive.find_any(serial_number=self.odrive_serial)
        self.simulation = False
        print(f'WHEEL CONNECTED TO {self.odrive_serial}')
        print(f'voltage detected: {self.odrive.vbus_voltage}')
        time.sleep(0.1)
        self.odrive.clear_errors()
        time.sleep(0.1)
        self.odrive.config.enable_brake_resistor = False
        self.odrive.config.brake_resistance = 0
        self.odrive.config.dc_max_negative_current = -81.0
        self.odrive.config.max_regen_current = 80.0

    # STEERING SPECIFIC METHODS

    def configure_steering(self):
        print('Configuring steering motor')
        self.odrive.axis0.motor.config.current_lim = 80
        self.odrive.axis0.controller.config.vel_limit = 50
        self.odrive.axis0.motor.config.calibration_current = 10
        self.odrive.axis0.controller.config.control_mode = odrive.enums.CONTROL_MODE_POSITION_CONTROL
        self.odrive.axis0.controller.config.input_mode = odrive.enums.INPUT_MODE_TRAP_TRAJ
        self.odrive.axis0.trap_traj.config.vel_limit = 40
        self.odrive.axis0.trap_traj.config.accel_limit = 100
        self.odrive.axis0.trap_traj.config.decel_limit = 100

    def calibrate_steering(self):
        print('Calibrating steering motor')
        self.odrive.axis0.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    def wait_for_calibration_steering(self):
        time.sleep(1)
        # print(self.odrive.axis0.current_state)
        # while self.odrive.axis0.current_state != odrive.enums.AXIS_STATE_IDLE and self.odrive.axis1.current_state != odrive.enums.AXIS_STATE_IDLE:
        while self.odrive.axis0.current_state != odrive.enums.AXIS_STATE_IDLE:
            time.sleep(0.1)
        print('steering idle')

    def engage_steering(self):
        self.odrive.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        print('motors engaged')

    @property
    def wheel_angle(self):
        pos = self.odrive.axis0.encoder.pos_estimate
        return pos*360/self.steering_gearing

    @wheel_angle.setter
    def wheel_angle(self, angle):
        self.steering_setpoint = angle
        self.odrive.axis0.controller.input_pos = self.steering_setpoint * \
            self.steering_gearing / 360

    # DRIVE SPECIFIC METHODS

    def configure_drive(self):
        self.odrive.axis1.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
        self.odrive.axis1.controller.config.input_mode = odrive.enums.INPUT_MODE_VEL_RAMP
        self.odrive.axis1.controller.config.vel_ramp_rate = 100.0

    def calibrate_drive(self):
        print('Calibrating drive motor')
        self.odrive.axis1.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    def wait_for_calibration_drive(self):
        time.sleep(1)
        # print(self.odrive.axis1.current_state)
        # while self.odrive.axis0.current_state != odrive.enums.AXIS_STATE_IDLE and self.odrive.axis1.current_state != odrive.enums.AXIS_STATE_IDLE:
        while self.odrive.axis1.current_state != odrive.enums.AXIS_STATE_IDLE:
            # print(self.odrive.axis1.current_state)
            time.sleep(0.1)
        print('drive idle')
        print(self.odrive.axis1.current_state)

    def engage_drive(self):
        self.odrive.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        print('motors engaged')

    @property
    def wheel_speed(self):
        pos = self.odrive.axis1.encoder.pos_estimate
        return pos*360/self.drive_gearing

    @wheel_speed.setter
    def wheel_speed(self, speed):
        self.drive_setpoint = speed
        self.odrive.axis1.controller.input_vel = self.drive_setpoint * self.drive_gearing

    # set wheel inputs
    def set_inputs(self, inputs):
        # get feedback ready first
        fb = []
        fb.append(float(self.steering_actual))
        deltadist = (time.time()-self.last_time)*self.drive_actual
        self.total_dist += deltadist
        fb.append(float(self.total_dist))
        self.last_time = time.time()
        # set new setpoints
        self.steering_setpoint = inputs[0]
        self.drive_setpoint = inputs[1]

        if self.simulation == True:
            self.steering_actual = self.steering_setpoint
            self.drive_actual = self.drive_setpoint
        else:
            fb = [0.0, 0.0]
            self.odrive.axis0.controller.input_pos = self.steering_setpoint * \
                self.steering_gearing / 6.28
            self.steering_actual = self.odrive.axis0.encoder.pos_estimate
            self.odrive.axis1.controller.input_vel = self.drive_setpoint * self.drive_gearing
            self.drive_actual = self.odrive.axis1.encoder.vel_estimate
        return fb
    # get feedback


class Cormoran2WD(threading.Thread):
    """
    main driver class for cormoran robot
    """

    def __init__(self, odrive_serials, wheelbase=1, track_width=1):
        self.simulation = True
        self.locomotion_mode = 1
        self.wheel_select = 1
        self.wheelbase = wheelbase
        self.track_width = track_width
        # signal.signal(signal.SIGINT, self.handler)
        self.inputs = [0.0, 0.0]
        self.wheels = []
        self.speed = 0.0
        for serial in odrive_serials:
            self.wheels.append(Wheel(serial))
        print(f'{len(self.wheels)} wheels have been instantiated')
        self.exiting = False
        threading.Thread.__init__(self)

    def connect_to_hardware(self):
        self.simulation = False

        print('starting steering calibration...')

        for wheel in self.wheels:
            wheel.connect_to_hardware()

        for wheel in self.wheels:
            wheel.configure_steering()
        for wheel in self.wheels:
            wheel.calibrate_steering()
        for wheel in self.wheels:
            wheel.wait_for_calibration_steering()
        for wheel in self.wheels:
            wheel.engage_steering()

        for wheel in self.wheels:
            wheel.configure_drive()
        for wheel in self.wheels:
            wheel.calibrate_drive()
        for wheel in self.wheels:
            wheel.wait_for_calibration_drive()
        for wheel in self.wheels:
            wheel.engage_drive()

        # print('POWERING ON MOTORS')
        # self.odds.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.odds.axis0.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
        # self.odds.axis0.controller.config.vel_ramp_rate = 20.0
        # self.odds.axis0.controller.config.input_mode = odrive.enums.INPUT_MODE_VEL_RAMP

        # self.odds.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.odds.axis1.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
        # self.odds.axis1.controller.config.vel_ramp_rate = 20.0
        # self.odds.axis1.controller.config.input_mode = odrive.enums.INPUT_MODE_VEL_RAMP

        # self.evens.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.evens.axis0.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
        # self.evens.axis0.controller.config.vel_ramp_rate = 20.0
        # self.evens.axis0.controller.config.input_mode = odrive.enums.INPUT_MODE_VEL_RAMP

        # self.evens.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.evens.axis1.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
        # self.evens.axis1.controller.config.vel_ramp_rate = 20.0
        # self.evens.axis1.controller.config.input_mode = odrive.enums.INPUT_MODE_VEL_RAMP
        return

    def close(self):
        print('YOU ASKED ME TO QUIT SO I WILL QUIT')
        self.exiting = True
        return

    def on_a_pressed(self, button):
        """
        controller 'a' button callback
        """
        if self.locomotion_mode != 1:
            print("Mode changed:      ACKERMAN")
            self.locomotion_mode = 1

    def on_b_pressed(self, button):
        """
        controller 'b' button callback
        """
        if self.locomotion_mode != 2:
            print("Mode changed:         PIVOT")
            self.locomotion_mode = 2

    def on_x_pressed(self, button):
        """
        controller 'x' button callback
        """
        if self.locomotion_mode != 3:
            print("Mode changed:          CRAB")
            self.locomotion_mode = 3

    def on_y_pressed(self, button):
        """
        controller 'y' button callback
        """
        # def on_axis_moved(axis):
        #     print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
        if self.locomotion_mode != 4:
            print("Mode changed:   CALIBRATION")
            self.locomotion_mode = 4
            self.wheel_select = 1
            print(f"    Now calibrating wheel {self.wheel_select}")
        else:
            if self.wheel_select < 4:
                self.wheel_select = self.wheel_select + 1
            elif self.wheel_select == 4:
                self.wheel_select = 1
            print(f"    Now calibrating wheel {self.wheel_select}")

    def pushSetpoints(self, setpoints):
        # message = '{: 06.2f},{: 06.2f},{: 06.2f},{: 06.2f};'.format(
        #     self.motor_1_angle,
        #     self.motor_2_angle,
        #     self.motor_3_angle,
        #     self.motor_4_angle)
        # print(message[:-2], end="\r")
        # self.textPrint.tprint(self.screen, f'{message}')
        # if self.simulation == False:
        feedback = []
        # print(len(self.wheels))
        for x in range(len(self.wheels)):
            fb = self.wheels[x].set_inputs(setpoints[x])
            feedback.append(fb)
            # self.wheels[x].wheel_angle = setpoints[x][0]
            # feedback[x].append(self.wheels[x].wheel_angle)
            # self.wheels[x].wheel_speed = setpoints[x][1]
            # feedback[x].append(self.wheels[x].wheel_speed)
            # self.arduino.write(message.encode())
            # time.sleep(0.001)
            # data = self.arduino.readline().decode()[:-2]
            # # print(data)
            # self.odds.axis0.controller.input_vel = self.motor_1_velocity
            # self.evens.axis0.controller.input_vel = self.motor_2_velocity
            # self.odds.axis1.controller.input_vel = self.motor_3_velocity
            # self.evens.axis1.controller.input_vel = self.motor_4_velocity
        # elif self.simulation == True:
        #     feedback=setpoints
        return feedback

    def handler(self, signum, frame):  # pylint: disable=W0613
        """
        This is the handler for SIGINT events sent by the user pressing ctrl-c.
        """
        # print(" Ctrl-c was pressed. Exiting...")
        # pygame.quit()
        # sys.exit()
        self.exiting = True

    def run_once(self):
        # PYGAME BUSINESS
        # for event in pygame.event.get():  # User did something.
        #     if event.type == pygame.QUIT:  # If user clicked close.
        #         done = True  # Flag that we are done so we exit this loop.
        #     elif event.type == pygame.JOYBUTTONDOWN:
        #         print("Joystick button pressed.")
        #     elif event.type == pygame.JOYBUTTONUP:
        #         print("Joystick button released.")
        # self.screen.fill(WHITE)
        # self.textPrint.reset()
        # joystick_count = pygame.joystick.get_count()
        # self.textPrint.tprint(
        #     self.screen, "Number of controllers connected: {}".format(joystick_count))
        # self.textPrint.indent()
        # if joystick_count != 0:
        #     joystick = pygame.joystick.Joystick(0)
        #     joystick.init()
        #     try:
        #         jid = joystick.get_instance_id()
        #     except AttributeError:
        #         # get_instance_id() is an SDL2 method
        #         jid = joystick.get_id()
        #     self.textPrint.tprint(self.screen, "Joystick {}".format(jid))
        #     name = joystick.get_name()
        #     self.textPrint.tprint(
        #         self.screen, "Joystick name: {}".format(name))
        #     axes = joystick.get_numaxes()
        #     axiss = []
        #     for i in range(axes):
        #         axis = joystick.get_axis(i)
        #         axiss.append(axis)
        #     trigs = (axiss[5]+1)/2 - (axiss[2]+1)/2
        #     # print(axiss)
        #     leftyright = axiss[0]
        # else:
        #     trigs = 0
        #     leftyright = 0

        # print("Number of joysticks: {}".format(joystick_count))
        # trigs = self.controller.trigger_r.value - self.controller.trigger_l.value
        # trigs = 0
        # leftyright = 0
        if self.locomotion_mode == 1:
            # ACKERMAN
            # ackerman_angle = self.controller.axis_l.x * 45.0
            # leftyright = self.inputs[0]
            # throttle = self.inputs[1]
            ackerman_angle = self.inputs[0]
            ackerman_speed = self.inputs[1]
            setpoints = [[], []]

            # -------- Ideal Ackerman calculations
            try:
                ackerman_radius = self.wheelbase/math.sin(ackerman_angle)
            except ZeroDivisionError as zde:
                ackerman_radius = np.inf
            rear_radius = math.cos(ackerman_angle)*ackerman_radius

            # Wheel 0 calculations
            wheel_0_angle = math.atan(
                self.wheelbase/(rear_radius + 0.5*self.track_width))
            try:
                wheel_0_radius = self.wheelbase/math.sin(wheel_0_angle)
            except:
                wheel_0_radius = np.inf
            wheel_0_speed = ackerman_speed * (wheel_0_radius/ackerman_radius)
            if np.isnan(wheel_0_speed):
                wheel_0_speed = ackerman_speed
            setpoints[0].append(wheel_0_angle)
            setpoints[0].append(wheel_0_speed)

            # Wheel 1 calculations
            wheel_1_angle = math.atan(
                self.wheelbase/(rear_radius - 0.5*self.track_width))
            try:
                wheel_1_radius = self.wheelbase/math.sin(wheel_1_angle)
            except:
                wheel_1_radius = np.inf
            wheel_1_speed = ackerman_speed * (wheel_1_radius/ackerman_radius)
            if np.isnan(wheel_1_speed):
                wheel_1_speed = ackerman_speed
            setpoints[1].append(wheel_1_angle)
            setpoints[1].append(wheel_1_speed)

            # Push the setpoints and get feedback
            # ----feedback needs to include total distance travelled not speed
            # print(setpoints)
            feedback = self.pushSetpoints(setpoints)

            # if ackerman_angle != 0:
            #     radius = self.wheelbase / math.tan(math.radians(ackerman_angle))

            #     # self.wheels[0].wheel_angle = math.degrees( math.atan(self.wheelbase / (radius + 0.5 * self.wheelbase)))
            #     setpoints[0].append(math.degrees( math.atan(self.wheelbase / (radius + 0.5 * self.wheelbase))))
            #     # self.wheels[1].wheel_angle = math.degrees( math.atan(self.wheelbase / (radius - 0.5 * self.wheelbase)))
            #     setpoints[1].append(math.degrees( math.atan(self.wheelbase / (radius - 0.5 * self.wheelbase))))
            # else:
            #     # self.wheels[0].wheel_angle = 0.0
            #     setpoints[0].append(0.0)
            #     # self.wheels[1].wheel_angle = 0.0
            #     setpoints[1].append(0.0)

            # # self.motor_3_angle = 0.0
            # # self.motor_4_angle = 0.0
            # # self.wheels[0].wheel_speed = throttle
            # setpoints[0].append(throttle)
            # # self.wheels[1].wheel_speed = throttle
            # setpoints[1].append(throttle)

            # print('--------------------------------')
            # print(f'setpoint = {setpoints}')
            # print(f'feedback = {feedback}')

            # self.motor_3_velocity = trigs * self.gearing
            # self.motor_4_velocity = trigs * self.gearing
        # if self.locomotion_mode == 2:
        #     # PIVOT
        #     self.motor_1_angle = 45.0
        #     self.motor_2_angle = -45.0
        #     self.motor_3_angle = -45.0
        #     self.motor_4_angle = 45.0
        #     self.motor_1_velocity = trigs * self.gearing
        #     self.motor_2_velocity = -trigs * self.gearing
        #     self.motor_3_velocity = trigs * self.gearing
        #     self.motor_4_velocity = -trigs * self.gearing
        # if self.locomotion_mode == 3:
        #     # CRAB
        #     self.motor_1_angle = 90.0
        #     self.motor_2_angle = -90.0
        #     self.motor_3_angle = -90.0
        #     self.motor_4_angle = 90.0
        #     self.motor_1_velocity = trigs * self.gearing
        #     self.motor_2_velocity = -trigs * self.gearing
        #     self.motor_3_velocity = -trigs * self.gearing
        #     self.motor_4_velocity = trigs * self.gearing
        # if self.locomotion_mode == 4:
        #     # CALIBRATION
        #     if self.wheel_select == 1:
        #         if self.controller.button_trigger_l.is_pressed:
        #             self.motor_1_angle = self.motor_1_angle - 0.5
        #         if self.controller.button_trigger_r.is_pressed:
        #             self.motor_1_angle = self.motor_1_angle + 0.5
        #     if self.wheel_select == 2:
        #         if self.controller.button_trigger_l.is_pressed:
        #             self.motor_2_angle = self.motor_2_angle - 0.5
        #         if self.controller.button_trigger_r.is_pressed:
        #             self.motor_2_angle = self.motor_2_angle + 0.5
        #     if self.wheel_select == 3:
        #         if self.controller.button_trigger_l.is_pressed:
        #             self.motor_3_angle = self.motor_3_angle - 0.5
        #         if self.controller.button_trigger_r.is_pressed:
        #             self.motor_3_angle = self.motor_3_angle + 0.5
        #     if self.wheel_select == 4:
        #         if self.controller.button_trigger_l.is_pressed:
        #             self.motor_4_angle = self.motor_4_angle - 0.5
        #         if self.controller.button_trigger_r.is_pressed:
        #             self.motor_4_angle = self.motor_4_angle + 0.5
        # self.pushSetpoints()
        # pygame.display.flip()
        # clock.tick(60)
        return feedback

    def run(self):
        print('STARTING NOW')
        while True:
            self.run_once()
            if self.exiting:
                return
            time.sleep(1/50)


if __name__ == "__main__":
    print('please import this module into another script :)')

    # robot = Cormoran(wheelbase=0.0254 * 12 * 2, track_width=0.0254 * 12 * 2)

    # Uncomment the following line if you wish to run the robot for real

    # robot.connect_to_hardware(
    # '/dev/serial/by-id/usb-Teensyduino_USB_Serial_3946960-if00', '307F347D3131', '208737A03548')

    # robot.main()
    pass
