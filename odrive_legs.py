import odrive
from odrive.enums import *
from odrive.shell import dump_errors
from time import sleep

# odrv0.axis0.motor.config.pole_pairs = 20
# odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
# odrv0.axis0.motor.config.calibration_current = 2
# odrv0.axis0.motor.config.resistance_calib_max_voltage = 12

# odrv0.axis1.motor.config.pole_pairs = 20
# odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
# odrv0.axis1.motor.config.calibration_current = 2
# odrv0.axis1.motor.config.resistance_calib_max_voltage = 12


class Legs:

    def __init__(self):
        self.left_offset = 0
        self.right_offset = 0
        self.left_chain_offset = 0
        self.right_chain_offset = 0
        self.odrv0 = odrive.find_any()
        self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.odrv1 = odrive.find_any()
        self.odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.hold_chain_bar()

    def reset(self):
        self.left_offset = self.odrv0.axis0.encoder.count_in_cpr
        self.right_offset = self.odrv0.axis0.encoder.count_in_cpr

    def cpr_to_deg(self, cpr):
        return cpr / 8. / 4096. * 360.

    def deg_to_cpr(self, deg):
        return deg * 8.0 * 4096. / 360.

    def go_to_deg(self, degrees):
        self.go_to_deg_both(degrees, degrees)

    def go_left(self, degrees):
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

        self.odrv0.axis0.controller.pos_setpoint = degrees * 8.0 * 4096.0 / 360.0
        self.odrv0.axis0.controller.pos_setpoint -= self.left_offset

    def go_right(self, degrees):
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

        self.odrv0.axis1.controller.pos_setpoint = -degrees * 8.0 * 4096.0 / 360.0
        self.odrv0.axis1.controller.pos_setpoint -= self.right_offset

    def go_to_deg_both(self, degrees_r, degrees_l):
        self.go_left(degrees_l)
        self.go_right(degrees_r)

        print(self.odrv0.axis0.controller.pos_setpoint)
        print(self.odrv0.axis1.controller.pos_setpoint)

    def step(self):
        self.odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self.odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.vel_setpoint = -4096*8
        sleep(0.5)
        self.odrv0.axis1.controller.vel_setpoint = 4096*8

    def spring(self):
        self.odrv0.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        for i in range(1000):
            self.odrv0.axis1.controller.current_setpoint = self.odrv0.axis1.encoder.pos / 100.

    def hold_chain_bar(self):
        self.odrv1.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.odrv1.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.left_chain_offset = self.odrv1.axis0.encoder.count_in_cpr
        self.right_chain_offset = self.odrv1.axis1.encoder.count_in_cpr



help_string = '''
    COMMANDS
    go: takes 1 position in degrees, both legs go there
    go_both: takes 2 positions, legs go there individually
    step: moves both at 1 rev/s (EXPERIMENTAL)
    reset: current encoder positions become the new zero positions (EXPERIMENTAL)
    err: dump errors (also clears them)
    reboot: Reboots the ODrive, script needs to be rerun after this
    print: various diagnostics, for testing and not meant for actual use yet
    spring: act like a spring (EXPERIMENTAL)
'''

if __name__ == "__main__":

    legs = Legs()

    while True:
        response = input("what do you want to do?")
        print(response)
        if response == "go_both":
            pos = input("where to?")
            print(float(pos))
            legs.go_to_deg(float(pos))
        elif response == "go":
            pos_R = input("where to right?")
            pos_L = input("where to left?")
            print(pos_R)
            print(pos_L)
            legs.go_to_deg_both(float(pos_R), float(pos_L))
        elif response == "reset":
            legs.reset()
        elif response == "step":
            legs.step()
        elif response == "err":
            print(dump_errors(legs.odrv0, True))
        elif response == "reboot":
            legs.odrv0.reboot()
            # odrv0.reboot()
            # odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            # odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        elif response == "print":
            print("left offset:", legs.left_offset)
            print("right_offset:", legs.right_offset)
        elif response == "spring":
            print('NOT YET IMPLEMENTED')
        elif response == "help":
            print(help_string)
