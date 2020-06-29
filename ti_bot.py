#!/usr/bin/env python3
import rospy
# serial is dependent on pyserial: $ python3 -m pip install pyserial
import serial.tools.list_ports as port_list
import serial
from geometry_msgs.msg import Twist

PORT='/dev/ttyACM1'

## TiBot
#
#  Connects to the MSP 432 over serial and writes linear/angular velocity
#   to enable the TI_Bot to drive.
#
#  Subscriber
#   Topic: cmd_vel
#     Msg type: Twist
#     Freq: 100 Hz
class TiBot:
    def __init__(self):
        
        # Initialize a serial w/ the MSP432 microcontroller
        self.ser = serial.Serial(port = PORT,
                        baudrate = 115200,
                        parity = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        bytesize = serial.EIGHTBITS,
                        timeout = 1)
        
        rospy.Subscriber('cmd_vel', Twist, self.callback_write)

    # Subscriber function that gets linear/angular velocity
    # values from controller and writes to MSP_432
    # Topic: Cmd_vel
    # Msg type: Twist
    def callback_write(self, data):
        input = str(round(data.linear.x,0)) + ',' + str(round(data.angular.z,0)) + '\r\n'
        self.ser.write(input.encode())
        print(input)
    
if __name__ == '__main__':
    rospy.init_node('ti_bot', anonymous = True)
    
    # list available ports
    ports = list(port_list.comports())
    for p in ports:
        print(p)
    
    # TODO: automate connection similar to crazyflie-lib-python serial connect
    TiBot()
    rospy.spin()
