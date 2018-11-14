#!/usr/bin/python
import logging
import time
import rospy
import numpy as np
from argparse import ArgumentParser

from sensor_msgs.msg import Imu
from am_post_processing_cloud import BNO055

msg = Imu()
rospy.init_node('BNO055_imu', anonymous=True)
pub_imu = rospy.Publisher('BNO055_imu', Imu, queue_size=100)

parser = ArgumentParser()
parser.add_argument('-p','--port', action='store',help='Ip address of target', default='/dev/ttyUSB4')
parser.add_argument('-f','--frequency', action='store',type=str,help='Frequency', default=60)
parser.add_argument('-A','--accelerometer', action='store_true', default=False)
parser.add_argument('-G','--gyroscope', action='store_true', default=False)
parser.add_argument('-E','--euler', action='store_true', default=True)
arg = parser.parse_args(rospy.myargv()[1:]) #handle roslaunch args

rate = rospy.Rate(arg.frequency)
time.sleep(5)

while not rospy.is_shutdown():
    try:
        bno = BNO055.BNO055(serial_port=arg.port)
        # Initialize the BNO055 and stop if something went wrong.2
        if not bno.begin():
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
        status, self_test, error = bno.get_system_status()
        # Print system status and self test result.
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        # Print out an error if system status is in error mode.
        if status == 0x01:
            print('System error: {0}'.format(error))
            print('See datasheet section 4.3.59 for the meaning.')

        # Print BNO055 software revision and other diagnostic data.
        sw, bl, accel, mag, gyro = bno.get_revision()
        print('Software version:   {0}'.format(sw))
        print('Bootloader version: {0}'.format(bl))
        print('Accelerometer ID:   0x{0:02X}'.format(accel))
        print('Magnetometer ID:    0x{0:02X}'.format(mag))
        print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

        print('Reading BNO055 data, press Ctrl-C to quit...')

        break
    except:
        print("Problem intiialize BNO055 retryting")
    time.sleep(3)



def toQuaternion(pitch, roll, yaw):
    cy = np.cos(yaw * 0.5);
    sy = np.sin(yaw * 0.5);
    cr = np.cos(roll * 0.5);
    sr = np.sin(roll * 0.5);
    cp = np.cos(pitch * 0.5);
    sp = np.sin(pitch * 0.5);

    w = cy*cr*cp+sy*sr*sp
    x = cy*sr*cp-sy*cr*sp
    y = cy*cr*sp+sy*sr*cp
    z = sy*cr*cp-cy*sr*sp
    return (w,x,y,z)


while not rospy.is_shutdown():
    #heading, roll, pitch = bno.read_euler()
    #w,x,y,z = toQuaternion(pitch, roll, heading)
    if(arg.euler):
        x,y,z,w = bno.read_quaternion()
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z

    if(arg.accelerometer):
        ax, ay, az = bno.read_accelerometer()
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

    if(arg.gyroscope):
        gx, gy, gz = bno.read_gyroscope()
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

    rate.sleep()
    msg.header.stamp = rospy.Time.now()
    pub_imu.publish(msg)
