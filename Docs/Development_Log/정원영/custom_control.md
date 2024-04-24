# 커스텀 컨트롤
- 기존의 터틀봇 조작 코드는 실시간 반응을 통한 이동이 어렵다
- 파이썬의 pynput 라이브러리를 통해 콜백 함수를 등록하고 이를 통해 즉각적인 움직임을 가능하게 했다.

```(python)
#!/usr/bin/env python

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

from pynput import keyboard

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.35
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.35
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

os.environ['TURTLEBOT3_MODEL'] = 'burger'
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

keys_on = 0
is_input_done = False

def on_press(key):
    global keys_on
    if key == keyboard.KeyCode.from_char('w') or key == keyboard.KeyCode.from_char('W'):
        keys_on = keys_on | 0b1000
    elif key == keyboard.KeyCode.from_char('s') or key == keyboard.KeyCode.from_char('S'):
        keys_on = keys_on | 0b100
    elif key == keyboard.KeyCode.from_char('a') or key == keyboard.KeyCode.from_char('A'):
        keys_on = keys_on | 0b10
    elif key == keyboard.KeyCode.from_char('d') or key == keyboard.KeyCode.from_char('d'):
        keys_on = keys_on | 0b1

def on_release(key):
    global keys_on
    global is_input_done
    if key == keyboard.KeyCode.from_char('w') or key == keyboard.KeyCode.from_char('W'):
        keys_on = keys_on & ~0b1000
    elif key == keyboard.KeyCode.from_char('s') or key == keyboard.KeyCode.from_char('S'):
        keys_on = keys_on & ~0b100
    elif key == keyboard.KeyCode.from_char('a') or key == keyboard.KeyCode.from_char('A'):
        keys_on = keys_on & ~0b10
    elif key == keyboard.KeyCode.from_char('d') or key == keyboard.KeyCode.from_char('d'):
        keys_on = keys_on & ~0b1
    elif key == keyboard.Key.esc:
        keys_on = 0
        is_input_done = True
        return False


def print_vels(target_linear_velocity, target_angular_velocity, keys):
    print('currently:\tlinear velocity {0}\t angular velocity {1}\t key {2} '.format(
        target_linear_velocity,
        target_angular_velocity, keys))


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


def main():
    global keys_on
    global is_input_done
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    key_listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release
    )

    key_listener.start()

    try:
        while(1):
            #print(keys_on)
            if keys_on & 0b1000 and keys_on & 0b0100:
                # no linear movement
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
            elif keys_on & 0b1000:
                # advance
                if TURTLEBOT3_MODEL == 'burger':
                    target_linear_velocity = BURGER_MAX_LIN_VEL
                else:
                    target_linear_velocity = WAFFLE_MAX_LIN_VEL
            elif keys_on & 0b0100:
                # retreat
                if TURTLEBOT3_MODEL == 'burger':
                    target_linear_velocity = -BURGER_MAX_LIN_VEL
                else:
                    target_linear_velocity = -WAFFLE_MAX_LIN_VEL
            else:
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0

            if keys_on & 0b0010 and keys_on & 0b0001:
                # no linear movement
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
            elif keys_on & 0b0010:
                # advance
                if TURTLEBOT3_MODEL == 'burger':
                    target_angular_velocity = BURGER_MAX_ANG_VEL
                else:
                    target_angular_velocity = WAFFLE_MAX_ANG_VEL
            elif keys_on & 0b0001:
                # retreat
                if TURTLEBOT3_MODEL == 'burger':
                    target_angular_velocity = -BURGER_MAX_ANG_VEL
                else:
                    target_angular_velocity = -WAFFLE_MAX_ANG_VEL
            else:
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0

            print_vels(target_linear_velocity, target_angular_velocity,keys_on)

            twist = Twist()

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))
            
            control_linear_velocity = target_linear_velocity

            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))
            
            control_angular_velocity = target_angular_velocity

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)

            #print(twist)

            if is_input_done:
                key_listener.join()
                break

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()

```