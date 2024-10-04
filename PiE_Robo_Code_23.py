# NUMBER OF SEQUENCE
SEQUENCE_NUM = 1

# Motors
MOTOR_ID = "6_7841804632356801521"
LEFT_MTR = "b"
RIGHT_MTR = "a"

# Arm motor
ARM_MOTOR_ID = "6_7409335424966160129"
ARM_MTR = "b"
ARM_SPEED = 0.45

# Servo claw
SERVO_MTR_CLAW = "servo1"
SERVO_ID_CLAW = "4_14356227426326476275"

# Servo pusher
SERVO_PUSHER_MTR = "servo0"
SERVO_PUSHER_ID = "4_14356227426326476275"

# Wheel's const for autonomous in mm
WHEEL_DIAMETER = 101.6
ENC_CONST = 747 # Encoder counts for 1 wheel revolution

# Autonomous constants
TOLERANCE = 20
PI_VALUE = 3.14159265358979323846264338327950288419716939937510

def autonomous_setup():
    print("Autonomous mode has started!")
    Robot.set_value(MOTOR_ID, "invert_a", False)
    Robot.set_value(MOTOR_ID, "invert_b", True)
    Robot.set_value(MOTOR_ID, "pid_enabled_a", True)
    Robot.set_value(MOTOR_ID, "pid_enabled_b", True)
    Robot.set_value(SERVO_ID_CLAW, SERVO_PUSHER_MTR, -1)
    Robot.set_value(SERVO_ID_CLAW, SERVO_MTR_CLAW, -1)
    #Robot.set_value(ARM_MTR_ID, "enc_b", 0)

def teleop_setup():
    print("Tele-operated mode has started!")
    Robot.set_value(MOTOR_ID, "invert_a", True)
    Robot.set_value(MOTOR_ID, "invert_b", False)
    Robot.set_value(MOTOR_ID, "pid_enabled_a", False)
    Robot.set_value(MOTOR_ID, "pid_enabled_b", False)
    Robot.set_value(ARM_MOTOR_ID, "pid_enabled_a", False)
    Robot.set_value(ARM_MOTOR_ID, "pid_enabled_b", False)

    Robot.set_value(SERVO_ID_CLAW, SERVO_MTR_CLAW, 0)
    Robot.set_value(SERVO_PUSHER_ID, SERVO_PUSHER_MTR, 0)

    Robot.set_value(MOTOR_ID, "enc_a", 0)
    Robot.set_value(MOTOR_ID, "enc_b", 0)
    print(Robot.get_value(MOTOR_ID, "enc_b"))




def getEncoderA():
    encoder_a = Robot.get_value(MOTOR_ID, "enc_a")
    if encoder_a < 0:
        return abs(Robot.get_value(MOTOR_ID, "enc_a"))
    elif encoder_a == 0:
        return 0
    else:
        return -abs(Robot.get_value(MOTOR_ID, "enc_a"))

def getEncoderB():
    encoder_b = Robot.get_value(MOTOR_ID, "enc_b")
    if encoder_b < 0:
        return abs(Robot.get_value(MOTOR_ID, "enc_b"))
    elif encoder_b == 0:
        return 0
    else:
        return -abs(Robot.get_value(MOTOR_ID, "enc_b"))

def findTargetValue(distance):
  revolution = (distance)/(WHEEL_DIAMETER * PI_VALUE) #OR math.pi
  target_value = revolution * ENC_CONST
  return target_value


def getMotorSpeedValue(distance):
  if distance > 0:
    return 1
  elif distance < 0:
    return -1
  else:
    return 0


def autoGo(distance):
    current_pos_b = getEncoderB()
    current_pos_a = Robot.get_value(MOTOR_ID, "enc_a")
    target_pos = int(findTargetValue(distance))

    motors_speeed = getMotorSpeedValue(distance)

    enc_value_a_min = target_pos - TOLERANCE + current_pos_a
    enc_value_a_max = target_pos + TOLERANCE + current_pos_a
    enc_value_b_min = target_pos - TOLERANCE + current_pos_b
    enc_value_b_max = target_pos + TOLERANCE + current_pos_b

    while True:
        current_pos_b = getEncoderB()
        current_pos_a = Robot.get_value(MOTOR_ID, "enc_a")
        if (enc_value_a_min <= current_pos_a <= enc_value_a_max) and (enc_value_b_min <= current_pos_b <= enc_value_b_max):
            Robot.set_value(MOTOR_ID, "velocity_a", 0)
            Robot.set_value(MOTOR_ID, "velocity_b", 0)
            return False
        else:
            Robot.set_value(MOTOR_ID, "velocity_a", motors_speeed)
            Robot.set_value(MOTOR_ID, "velocity_b", motors_speeed)


def getTargetLenght(angle):
  arc_lenght = (angle/360)*2*PI_VALUE*370
  return arc_lenght


def getMotorSpeed(motor_id, direction):
    speed_map = {
        ("a", "r"): 1,
        ("a", "l"): -1,
        ("b", "r"): -1,
        ("b", "l"): 1
    }
    return speed_map[(motor_id, direction)]


# side getting only "l" or "r" and angle between 0 to 180 (where 180 full turn back)
def autoGoTurn(angle, direction, pos_adj):
    pos_adj = 1 + pos_adj/100 #transforms pos_adj parameter into digestable value for target_len
    current_pos_b = getEncoderB()
    current_pos_a = Robot.get_value(MOTOR_ID, "enc_a")
    target_len = getTargetLenght(angle)
    target_len = target_len * pos_adj #changed from set value to parameter, new parameter value will be continous variable such as 1.35
    target_pos = findTargetValue(target_len)

    motor_speed_a = getMotorSpeed("a", direction)
    motor_speed_b = getMotorSpeed("b", direction)

    enc_value_a_min = (target_pos * motor_speed_a) - TOLERANCE + current_pos_a
    enc_value_a_max = (target_pos * motor_speed_a) + TOLERANCE + current_pos_a
    enc_value_b_min = (target_pos * motor_speed_b) - TOLERANCE + current_pos_b
    enc_value_b_max = (target_pos * motor_speed_b) + TOLERANCE + current_pos_b

    acceleration_rate = 0.02  # Adjust as needed
    deceleration_zone = 0.5   # Value between 0 and 1, adjust as needed
    position_adjustment = pos_adj   # Adjust based on your observation

    max_motor_speed_a = motor_speed_a*0.45
    max_motor_speed_b = motor_speed_b*0.45
    adjusted_enc_value_a_max = enc_value_a_max - position_adjustment
    adjusted_enc_value_b_max = enc_value_b_max + position_adjustment

    while True:
        current_pos_b = getEncoderB()
        current_pos_a = Robot.get_value(MOTOR_ID, "enc_a")

        distance_a = abs(adjusted_enc_value_a_max - current_pos_a)
        distance_b = abs(adjusted_enc_value_b_max - current_pos_b)

        # Ramp-up motor speed
        if distance_a > (enc_value_a_max - enc_value_a_min) * deceleration_zone:
            motor_speed_a = min(max_motor_speed_a, motor_speed_a + acceleration_rate)
        # Ramp-down motor speed
        else:
            motor_speed_a = max(0.1 * max_motor_speed_a, motor_speed_a - acceleration_rate)

        # Repeat for motor B
        if distance_b > (enc_value_b_max - enc_value_b_min) * deceleration_zone:
            motor_speed_b = min(max_motor_speed_b, motor_speed_b + acceleration_rate)
        else:
            motor_speed_b = max(0.1 * max_motor_speed_b, motor_speed_b - acceleration_rate)

        if (enc_value_a_min <= current_pos_a <= adjusted_enc_value_a_max) and (enc_value_b_min <= current_pos_b <= adjusted_enc_value_b_max):
            Robot.set_value(MOTOR_ID, "velocity_a", 0)
            Robot.set_value(MOTOR_ID, "velocity_b", 0)
            return False
        else:
            Robot.set_value(MOTOR_ID, "velocity_a", motor_speed_a)
            Robot.set_value(MOTOR_ID, "velocity_b", motor_speed_b)

def dropBall():
    #TODO uncomment servo code
    Robot.set_value(SERVO_ID_CLAW, SERVO_MTR_CLAW, increase_servo_auto(SERVO_ID_CLAW, SERVO_MTR_CLAW))

def armCode(position):
    arm_target_pos = Robot.get_value(ARM_MOTOR_ID, "enc_" + ARM_MTR) + position
    while True:
        current_pos = Robot.get_value(ARM_MOTOR_ID, "enc_" + ARM_MTR) # Retrieves current position of the arm motor
        if current_pos < arm_target_pos:
            Robot.set_value(ARM_MOTOR_ID, "velocity_" + ARM_MTR, ARM_SPEED)
        elif current_pos > arm_target_pos:
            Robot.set_value(ARM_MOTOR_ID, "velocity_" + ARM_MTR, ARM_SPEED * -1.0)
        else:
            Robot.set_value(ARM_MOTOR_ID, "velocity_" + ARM_MTR, 0.0)
            return False

function_sequences = [
    [
        (autoGoTurn, [120, "l", 45]),
        (autoGo, [230]),
        (dropBall,()),
        (dropBall,()),
        (autoGoTurn, [180, "r", 15]),
        (autoGo, [180]),
        (autoGoTurn, [25, "l", 25]),

        (autoGo, [500]),
        #(autoGoTurn, [45, "l", 31]),
        #(autoGo, [400]),
        #(autoGoTurn, [45, "r", 50]),
        (autoGo, [620]),

    ],
    [
        (autoGoTurn, [120, "r", 36]),
        (autoGo, [225]),
        (armCode, [-100]),
        (dropBall,()),
        (autoGoTurn, [180, "l", 31]),
        (autoGo, [80]),
        (autoGoTurn, [60, "r", 15]),

        #(autoGo, [500]),
        #(autoGoTurn, [45, "r", 20]),
        (autoGo, [2100]),
        #(autoGoTurn, [45, "l", 25]),
        #(autoGo, [580]),
    ],
    [
        (autoGo, [150]),
        (autoGoTurn, [30, "l", 60]),
        (autoGo, [100]),
    ],
    [
        (autoGo, [300]),
        (autoGoTurn, [90, "l", 40]),
        (autoGo, [100]),
        (autoGoTurn, [90, "l", 40]),
        (autoGo, [200]),
    ],
]

function_sequence = function_sequences[SEQUENCE_NUM]
sequence_counter = 0


def autonomous_main():
    global function_sequence
    global sequence_counter
    if sequence_counter < len(function_sequence):
        func, func_args = function_sequence[sequence_counter]
        func(*func_args)
        Robot.sleep(1)
        sequence_counter += 1


def primaryForward():
    Robot.set_value(MOTOR_ID, "velocity_" + LEFT_MTR, 1.0)
    Robot.set_value(MOTOR_ID, "velocity_" + RIGHT_MTR, 1.0)

def primaryBackward():
    Robot.set_value(MOTOR_ID, "velocity_" + LEFT_MTR, -1.0)
    Robot.set_value(MOTOR_ID, "velocity_" + RIGHT_MTR, -1.0)

def setDefaultMotors():
    Robot.set_value(MOTOR_ID, "velocity_" + LEFT_MTR, 0.0)
    Robot.set_value(MOTOR_ID, "velocity_" + RIGHT_MTR, 0.0)

def arm_outside_limit():
    lower_limit = 600
    upper_limit = -500
    while True:
        current_pos = Robot.get_value(ARM_MOTOR_ID, "enc_" + ARM_MTR)
        if current_pos < upper_limit:
            Robot.set_value(ARM_MOTOR_ID, "velocity_" + ARM_MTR, 0.25)
        elif current_pos > lower_limit:
            Robot.set_value(ARM_MOTOR_ID, "velocity_" + ARM_MTR, -0.25)
        elif Gamepad.get_value("dpad_up"):
            arm_code(-10)
        else:
            return False

def arm_code(position):
  # position getting + or - values
    arm_target_pos = Robot.get_value(ARM_MOTOR_ID, "enc_" + ARM_MTR) + position
    while True:
        current_pos = Robot.get_value(ARM_MOTOR_ID, "enc_" + ARM_MTR) # Retrieves current position of the arm motor
        if current_pos < arm_target_pos:
            Robot.set_value(ARM_MOTOR_ID, "velocity_" + ARM_MTR, ARM_SPEED)
        elif current_pos > arm_target_pos:
            Robot.set_value(ARM_MOTOR_ID, "velocity_" + ARM_MTR, ARM_SPEED * -1.0)
        else:
            Robot.set_value(ARM_MOTOR_ID, "velocity_" + ARM_MTR, 0.0)
            return False

def increase_servo(SERVO_ID, SERVO_MTR):
    servo_val = Robot.get_value(SERVO_ID, SERVO_MTR)
    if servo_val <= 1:
        return servo_val + 0.25
    else:
        return servo_val
def decrease_servo(SERVO_ID, SERVO_MTR):
    servo_val = Robot.get_value(SERVO_ID, SERVO_MTR)
    if servo_val >= -1:
        return servo_val - 0.125
    else:
        return servo_val

def increase_servo_auto(SERVO_ID, SERVO_MTR):
    servo_val = Robot.get_value(SERVO_ID, SERVO_MTR)
    if servo_val <= 1:
        return servo_val + 0.5


pusher_speed = 0
claw_speed = 0

def teleop_main():
    joystick_y = Gamepad.get_value("joystick_left_y")
    joystick_x = Gamepad.get_value("joystick_left_x")

    left_motor_speed = joystick_y + joystick_x
    right_motor_speed = joystick_y - joystick_x

    global pusher_speed
    global claw_speed


    threshold = 0.2
    if abs(left_motor_speed) < threshold:
        left_motor_speed = 0
    if abs(right_motor_speed) < threshold:
        right_motor_speed = 0
    if Gamepad.get_value("r_trigger"):
        primaryBackward()
    elif Gamepad.get_value("l_trigger"):
        primaryForward()
    elif Gamepad.get_value("button_b"):
        claw_speed = increase_servo(SERVO_ID_CLAW, SERVO_MTR_CLAW)
        # autoGoTurn(90, "r", 24)
    elif Gamepad.get_value("button_y"):
        pusher_speed = increase_servo(SERVO_PUSHER_ID, SERVO_PUSHER_MTR)
        # autoGoTurn(180, "r", 29)
    elif Gamepad.get_value("button_x"):
        claw_speed = decrease_servo(SERVO_ID_CLAW, SERVO_MTR_CLAW)
        # autoGoTurn(90, "l",26)
    elif Gamepad.get_value("button_a"):
        pusher_speed = decrease_servo(SERVO_PUSHER_ID, SERVO_PUSHER_MTR)
        # autoGoTurn(180, "l", 29)
    elif Gamepad.get_value("dpad_up"):
        arm_outside_limit()
        arm_code(-10)
    elif Gamepad.get_value("dpad_down"):
        arm_code(10)
    elif Gamepad.get_value("r_bumper"):
        pass
    elif Gamepad.get_value("l_bumper"):
        pass
    elif Gamepad.get_value("button_start"):
        pass
    elif Gamepad.get_value("button_back"):
        pass
    else:
        Robot.set_value(ARM_MOTOR_ID, "velocity_" + ARM_MTR, 0)
        Robot.set_value(MOTOR_ID, "velocity_" + LEFT_MTR, left_motor_speed)
        Robot.set_value(MOTOR_ID, "velocity_" + RIGHT_MTR, right_motor_speed)
        Robot.set_value(SERVO_PUSHER_ID, SERVO_PUSHER_MTR, pusher_speed)
        Robot.set_value(SERVO_ID_CLAW, SERVO_MTR_CLAW, claw_speed)
    Robot.sleep(0.01)