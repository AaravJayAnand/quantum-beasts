# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       aaravjayanand                                                #
# 	Created:      1/26/2026, 2:59:19 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *


brain=Brain()

# Some Robot Specs for Reference
trackwidth = 11.5
wheelbase = 10.5
wheel_travel = 220
dt_ratio = 4/3

# Wiring
    # Left Drive: 2, 4, 6
    # Right Drive: 1, 3, 5
    # Intake: 11
    # Scorer: 12
    # Inertial: 13
    # Radio: 14
    # Matchloader: A
    # Descorer: B
    # Center Goal: C

# Constants
DRIVETRAIN_MAX_SPEED = 0.8  # Decimal
DRIVETRAIN_TURN_SPEED = 0.7    # Decimal; Stacks on max speed
INTAKE_SPEED = 150
SCORER_SPEED = 100
drive_pid_constants = (0.1, 0.0004, 0.1)  # Kp, Ki, Kd
turn_pid_constants = (3, 0.005, 0.001)  # Kp, Ki, Kd
turn_pid_factor = 1
pid_sensitivity = 2

# Devices and Drive
brain=Brain()

controller = Controller()

matchloader = DigitalOut(brain.three_wire_port.a)
descorer = DigitalOut(brain.three_wire_port.b)
center_goal = DigitalOut(brain.three_wire_port.c)

matchloader.set(False)
descorer.set(True)
center_goal.set(True)

dt_left = MotorGroup(Motor(Ports.PORT2, GearSetting.RATIO_6_1, True), Motor(Ports.PORT4, GearSetting.RATIO_6_1, True), Motor(Ports.PORT6, GearSetting.RATIO_6_1, True))
dt_right = MotorGroup(Motor(Ports.PORT1, GearSetting.RATIO_6_1), Motor(Ports.PORT3, GearSetting.RATIO_6_1), Motor(Ports.PORT5, GearSetting.RATIO_6_1))
# dt_left = Motor(Ports.PORT4, GearSetting.RATIO_6_1)
# dt_right = Motor(Ports.PORT3, GearSetting.RATIO_6_1)
intake = Motor(Ports.PORT11)
scorer = Motor(Ports.PORT12)
intake.set_stopping(BRAKE)
scorer.set_stopping(BRAKE)
dt_left.set_stopping(BRAKE)
dt_right.set_stopping(BRAKE)

inertial = Inertial(Ports.PORT13)
inertial.calibrate()
while inertial.is_calibrating():
    wait(20, MSEC)

drivetrain = DriveTrain(dt_left, dt_right, wheel_travel, trackwidth, wheelbase, DistanceUnits.IN, dt_ratio)

intake.set_velocity(INTAKE_SPEED, PERCENT)
scorer.set_velocity(SCORER_SPEED, PERCENT) 

# PID
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = (error - self.previous_error)
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        self.integral
        return output       
    
def drive(target_distance, speed_factor=1.0):
    # Reset Positions
    dt_left.reset_position()
    dt_right.reset_position()
    left_pos = 0
    right_pos = 0

    # Create PID Objects
    LeftPID = PIDController(*drive_pid_constants)
    RightPID = PIDController(*drive_pid_constants)

    while True:
        # Compute Outputs from PID
        left_output = LeftPID.compute(target_distance, left_pos)
        right_output = RightPID.compute(target_distance, right_pos)

        # Apply Output & Spin
        dt_left.spin(FORWARD, left_output / 1.4 * speed_factor, PERCENT)
        dt_right.spin(FORWARD, right_output / 1.4 * speed_factor, PERCENT)

        # Wait
        controller.screen.clear_screen()
        controller.screen.print(left_output, right_output)
        wait(20)
        # Log New Positions
        left_pos = dt_left.position() / 360 / dt_ratio * wheel_travel
        right_pos = dt_right.position() / 360 / dt_ratio * wheel_travel

        # Check for Exit Condition
        if abs(left_pos - target_distance) < pid_sensitivity * 3 and abs(right_pos - target_distance) < pid_sensitivity * 3:
            dt_left.stop()
            dt_right.stop()
            controller.screen.clear_screen()
            controller.screen.set_cursor(1,1)
            controller.screen.print("DONE")
            break

def turn(target_angle, direction, turn_pid_factor=1.0):
    # Reset Angle
    inertial.reset_heading()
    measured_angle = 0

    if direction == LEFT:
        target_angle = abs(360 - target_angle)

    # Create PID Object
    RotationalPID = PIDController(*turn_pid_constants)

    while True:
        # Log New Angle
        measured_angle = inertial.heading() if direction == RIGHT else abs(360 - inertial.heading())

        # Compute New Output from PID
        output = min(200, max(-200, RotationalPID.compute(target_angle, measured_angle) * turn_pid_factor))
        # output = RotationalPID.compute(target_angle, measured_angle)

        # Apply Output & Turn
        drivetrain.set_turn_velocity(output)
        drivetrain.turn(direction=direction)

        # Wait
        wait(20)

        if abs(target_angle - measured_angle) < pid_sensitivity:
            break
    
    drivetrain.stop()

# Driver Control Functions
def controller_screen():
    while True:
        controller.screen.clear_screen()
        controller.screen.set_cursor(1,1)
        controller.screen.print("Intake: L1")
        controller.screen.new_line()
        controller.screen.print("Outtake: R2")
        controller.screen.new_line()
        controller.screen.print("Score: L2")
        wait(1500, MSEC)
        controller.screen.clear_screen()
        controller.screen.set_cursor(1, 1)
        controller.screen.print("Matchloader [", bool(matchloader.value()), "]: R1")
        controller.screen.set_cursor(2, 1)
        controller.screen.print("Descorer [", not bool(descorer.value()), "]: X")
        controller.screen.set_cursor(3, 1)
        controller.screen.print("Center Goal [", not bool(center_goal.value()), "]: UP")
        wait(1500, MSEC)
        controller.screen.clear_screen()
        controller.screen.set_cursor(1, 1)
        controller.screen.print("Drive Temp: " + str((dt_left.temperature() +\
                                                       dt_right.temperature(TemperatureUnits.FAHRENHEIT)) / 2) + "F")
        controller.screen.set_cursor(2, 1)
        controller.screen.print("Scorer Temp: " + str(scorer.temperature(TemperatureUnits.FAHRENHEIT)) + "F")
        controller.screen.set_cursor(3, 1)
        controller.screen.print("Intake Temp: " + str(intake.temperature(TemperatureUnits.FAHRENHEIT)) + "F")
        wait(1500, MSEC)
        
def toggle_matchloader():
    matchloader.set(not matchloader.value())

def toggle_descorer():
    descorer.set(not descorer.value())

def toggle_center_goal():
    center_goal.set(not center_goal.value())

# Competition Functions
def autonomous():
    intake.spin(FORWARD)
    drive(540, speed_factor=2.4)
    intake.stop()
    turn(110, LEFT)
    drivetrain.drive(FORWARD)
    drive(830)
    matchloader.set(True)
    turn(55, RIGHT, turn_pid_factor=0.7)
    intake.spin(FORWARD)
    drive(280, speed_factor=2.4)
    wait(200, MSEC)
    intake.stop()
    drive(-730, speed_factor=2)
    # matchloader.set(False)
    intake.spin(REVERSE)
    scorer.spin(REVERSE)
    
def driver_control():
    # | Control Scheme | Tank Joysticks | Intake L1 | Outtake R2 | Score L2 | Matchloader R1 | Descorer X | Center Goal UP |
    controller_screen_thread = Thread(controller_screen)

    # Button Toggles
    controller.buttonR1.pressed(toggle_matchloader)
    controller.buttonX.pressed(toggle_descorer)
    controller.buttonUp.pressed(toggle_center_goal)

    while True:
        # Tank Drive
        left_speed = controller.axis3.position()
        right_speed = controller.axis2.position()
        if left_speed * right_speed < 0:
            left_speed *= DRIVETRAIN_TURN_SPEED
            right_speed *= DRIVETRAIN_TURN_SPEED
        dt_left.set_velocity(left_speed * DRIVETRAIN_MAX_SPEED, PERCENT)
        dt_right.set_velocity(right_speed * DRIVETRAIN_MAX_SPEED, PERCENT)
        dt_left.spin(FORWARD)
        dt_right.spin(FORWARD)

        # Intake Control
        if controller.buttonL1.pressing():
            # Spin intake in
            intake.spin(FORWARD)
            
        elif controller.buttonR2.pressing():
            # Spin intake out
            intake.spin(REVERSE)
        else:
            # Stop intake
            intake.stop()

        # Scorer Control
        if controller.buttonL2.pressing():
            # Spin scorer forward
            intake.spin(FORWARD)
            scorer.spin(FORWARD)
        else:
            # Stop scorer
            scorer.stop()
        wait(20, MSEC)

autonomous()
wait(5)