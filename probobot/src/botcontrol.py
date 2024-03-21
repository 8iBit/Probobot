#!/usr/bin/env python3
import sys

import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray
import rclpy

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------

Moving around:          Moving crane:
                        1   : crane rotate right
                        2   : crane rotate left
        w               Control grippers:
                        3   : shaft slide out
                        4   : shaft slide in
   a    s    d          5   : shaft up
                        6   : shaft down
        x               7   : gripper open 
                        8   : gripper close
                        0   : stop   

q : speed up
e : speed down
s : stop

CTRL-C to quit
"""

moveBindings = {
    "w": (1, 0, 0, 0),
    "d": (1, 0, 0, -1),
    "s": (0, 0, 0, 0),
    "a": (1, 0, 0, 1),
    "x": (-1, 0, 0, 0),
}

speedBindings = {
    "q": (1.25, 1.25),
    "e": (0.25, 0.25),
}

gripperBindings = {
    "1": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "2": (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "3": (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
    "4": (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
    "5": (0.0, 0.0, 1.0, 1.0, 0.0, 0.0),
    "6": (0.0, 0.0, -1.0, -1.0, 0.0, 0.0),
    "7": (0.0, 0.0, 0.0, 0.0, -1.0, -1.0),
    "8": (0.0, 0.0, 0.0, 0.0, 1.0, 1.0),
    "0": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
}

def getKey(settings):
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node("teleop_twist_keyboard")
    pub = node.create_publisher(geometry_msgs.msg.Twist, "cmd_vel", 10)
    pub1 = node.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)

    speed   = 0.5
    turn    = 1.0
    x       = 0.0
    y       = 0.0
    z       = 0.0
    th      = 0.0
    status  = 0.0
    crane   = 0.0
    shafty1 = 0.0
    shafty2 = 0.0
    shafty3 = 0.0
    grip1   = 0.0
    grip2   = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x   = moveBindings[key][0]
                y   = moveBindings[key][1]
                z   = moveBindings[key][2]
                th  = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed   = speed * speedBindings[key][0]
                turn    = turn * speedBindings[key][1]
            elif key in gripperBindings.keys():
                crane   = gripperBindings[key][0]
                shafty1 = gripperBindings[key][1]
                shafty2 = gripperBindings[key][2]
                shafty3 = gripperBindings[key][3]
                grip1   = gripperBindings[key][4]
                grip2   = gripperBindings[key][5]
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if key == "\x03":
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

            Float1 = Float64MultiArray()
            Float1.data = [crane, shafty1, shafty2, shafty3, grip1, grip2]
            pub1.publish(Float1)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)

if __name__ == "__main__":
    main()