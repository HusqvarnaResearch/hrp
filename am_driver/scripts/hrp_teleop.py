#! /usr/bin/env python
import rospy, math
import numpy as np
import sys, termios, tty, select, os
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from am_driver.msg import SensorStatus
from am_driver.msg import BatteryStatus

class KeyTeleop(object):
  cmd_bindings = {'q':np.array([1,1]),
                  'w':np.array([1,0]),
                  'e':np.array([1,-1]),
                  'a':np.array([0,1]),
                  'd':np.array([0,-1]),
                  'z':np.array([-1,-1]),
                  'x':np.array([-1,0]),
                  'c':np.array([-1,1]),
                  's':np.array([0,0])
                  }

  set_bindings = { 't':np.array([1,1]),
                  'b':np.array([-1,-1]),
                  'y':np.array([1,0]),
                  'n':np.array([-1,0]),
                  'u':np.array([0,1]),
                  'm':np.array([0,-1])
                  } 

  def init(self):
    # Save terminal settings
    self.settings = termios.tcgetattr(sys.stdin)
    # Initial values
    self.inc_ratio = 0.1
    self.speed = np.array([0.3, 1.0])
    self.command = np.array([0, 0])
    self.update_rate = 10   # Hz
    self.alive = True
    self.battAVolt = 0.0
    self.battBVolt = 0.0
    # Setup publishers
    self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
    rospy.Subscriber('/sensor_status',SensorStatus,self.callback_sensor_status)
    rospy.Subscriber('/battery_status',BatteryStatus,self.callback_battery_status)
    
    self.searching = False
    
    self.shapeNum = 0x20
    self.operationalMode = 0;
    self.sensorStatus = 0;
 
  def fini(self):
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
 

  def callback_sensor_status(self,data):
    if (self.operationalMode != data.operationalMode) or (self.sensorStatus != data.sensorStatus):
      self.operationalMode = data.operationalMode
      self.sensorStatus = data.sensorStatus
      self.showstatuslines()
     
  def callback_battery_status(self,data):
    self.battAVolt = data.batteryAVoltage/1000.0
    self.battBVolt = data.batteryBVoltage/1000.0
    self.showstatuslines()
	
    
  def showstatuslines(self):
    mode =''
    if self.operationalMode==0:
      mode = 'Offline'
    elif self.operationalMode == 1:
      mode = 'Manual '
    elif self.operationalMode == 2:
      mode = 'Random '
		 
	#define HVA_SS_HMB_CTRL 0x0001
    #define HVA_SS_OUTSIDE 0x0002
    #define HVA_SS_COLLISION 0x0004
    #define HVA_SS_LIFTED 0x0008
    #define HVA_SS_TOO_STEEP 0x0010
    #define HVA_SS_PARKED 0x0020
    #define HVA_SS_IN_CS 0x0040
    #define HVA_SS_USER_STOP 0x0080
    #define HVA_SS_CFG_NEEDED 0x0100
    #define HVA_SS_DISC_ON 0x0200
    #define HVA_SS_LOOP_ON 0x0400
	
    status = ''
    if self.sensorStatus & 0x0001:
      status = status + 'HMB_CTRL '
    if self.sensorStatus & 0x0002:
      status = status + 'Outside '
    if self.sensorStatus & 0x0004:
      status = status + 'Collision '
    if self.sensorStatus & 0x0008:
      status = status + 'Lifted '
    if self.sensorStatus & 0x0010:
      status = status + 'TooSteep '
    if self.sensorStatus & 0x0020:
      status = status + 'PARKED '
    if self.sensorStatus & 0x0040:
      status = status + 'IN_CS '
    if self.sensorStatus & 0x0080:
      status = status + 'USER_STOP '
    if self.sensorStatus & 0x0100:
      status = status + 'CFG NEEDED'
    if self.sensorStatus & 0x0200:
      status = status + 'DISC_ON '
    if self.sensorStatus & 0x0400:
      status = status + 'LOOP_ON '
    else:
      status = status + 'LOOP_OFF '
      
    status = status + '                                                                           '
    
    msg = u"\u008D"
    self.loginfoline(msg)
    msg = u"\u008D"
    self.loginfoline(msg)

    msg = 'Linear %.2f\tangular %.2f     batA %.1f V  batB %.1f V\n' % (self.speed[0],self.speed[1],self.battAVolt,self.battBVolt)
    self.loginfoline(msg)
    msg = ' OpMode: %s\t  Status: %s \n' % (mode,status)
    self.loginfoline(msg)

  def run(self):
    try:
      self.init()
      self.print_usage()
      r = rospy.Rate(self.update_rate) # Hz
      while not rospy.is_shutdown():
        ch = self.get_key()
        self.process_key(ch)
        self.update()
        self.showstatuslines()
        r.sleep()
    except rospy.exceptions.ROSInterruptException:
      pass
    finally:
      self.fini()
 
  def print_usage(self):
    msg = """
    Keyboard Teleop that Publish to /cmd_vel (geometry_msgs/Twist)
    -------------------------------------------------------
    H:       Print this menu

    Moving around:     Adjust Speed:    Cut high/low: I O
      Q   W   E          T  Y  U        Cut on/off:   J K
      A   S   D	                        Loop on/off:  8 9
      Z   X   C          B  N  M
 
    1:   Manual mode    2: Random Mode
    P:   Park           
    
    G :   Quit
    --------------------------------------------------------
    
    """
    self.loginfo(msg)
    self.showstatuslines()
 
  # Used to print items to screen, while terminal is in funky mode
  def loginfo(self, str):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    print(str)
    tty.setraw(sys.stdin.fileno())

  # Used to print items to screen, while terminal is in funky mode
  def loginfoline(self, str):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    print(str),
    tty.setraw(sys.stdin.fileno())
 
  # Used to print teleop status
  def show_status(self):
    msg = 'Status:\tlinear %.2f\tangular %.2f  batA %.1f V  batB %.1f V' % (self.speed[0],self.speed[1],self.batAVolt,self.batBVolt)
    self.loginfo(msg)
  
  # For everything that can't be a binding, use if/elif instead
  def process_key(self, ch):
	#
	# AM_DRIVER COMMANDS
	#
    
    if ch == 'h':
      self.print_usage()
    elif ch in self.cmd_bindings.keys():
      self.command = self.cmd_bindings[ch]
    elif ch in self.set_bindings.keys():
      self.speed = self.speed * (1 + self.set_bindings[ch]*self.inc_ratio)
    elif ch == 'g':
      self.loginfo('Quitting')
      # Stop the robot
      twist = Twist()
      self.pub_twist.publish(twist)

      # Stop following loop!
      mode = UInt16()
      mode.data = 0x17 
      self.pub_mode.publish(mode)

      rospy.signal_shutdown('Shutdown')
    elif ch == '1':
      # Manual mode
      mode = UInt16()
      mode.data = 0x90
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == '2':
      # Random mode
      mode = UInt16()
      mode.data = 0x91
      self.pub_mode.publish(mode)
      self.command = np.array([0, 0])
    elif ch == 'j':
      # Cutting disc ON
      mode = UInt16()
      mode.data = 0x93
      self.pub_mode.publish(mode)
    elif ch == 'k':
      # Cutting disc OFF
      mode = UInt16()
      mode.data = 0x92
      self.pub_mode.publish(mode)
    elif ch == 'i':
      # Cutting height HIGH
      mode = UInt16()
      mode.data = 0x95
      self.pub_mode.publish(mode)
    elif ch == 'o':
      # Cutting height LOW
      mode = UInt16()
      mode.data = 0x94
      self.pub_mode.publish(mode)
    elif ch == 'p':
      # Request SEARCHING (charge)
      mode = UInt16()
      mode.data = 0x100
        #  self.searching = True
      #else:
       #   mode.data = 0x101
        #  self.searching = False
      self.pub_mode.publish(mode)

    elif ch == '8':
      # Disable Loop
      mode = UInt16()
      mode.data = 0x110
      self.pub_mode.publish(mode)
    elif ch == '9':
      # Enable Loop
      mode = UInt16()
      mode.data = 0x111
      self.pub_mode.publish(mode)
    else:
      self.command = np.array([0, 0])
      

 
  def update(self):
    if rospy.is_shutdown():
      return
    twist = Twist()
    cmd  = self.speed*self.command
    twist.linear.x = cmd[0]
    twist.angular.z = cmd[1]

    self.pub_twist.publish(twist)
 
  # Get input from the terminal
  def get_key(self):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    return key.lower()
#    return key
 
if __name__ == '__main__':
  rospy.init_node('keyboard_teleop')
  teleop = KeyTeleop()
  teleop.run()
