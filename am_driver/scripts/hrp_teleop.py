#! /usr/bin/env python
import rospy, math
import numpy as np
import sys, termios, tty, select, os, threading
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from am_driver.msg import SensorStatus
from am_driver.msg import BatteryStatus



def mowerStateToString(x):
  return {
    0:'OFF',
    1:'WAIT_SAFETY_PIN',
    2:'STOPPED',
    3:'FATAL_ERROR',
    4:'PENDING_START',
    5:'PAUSED',
    6:'IN_OPERATION',
    7:'RESTRICTED',
    8:'ERROR'
  }.get(x,'UNKNONW_VALUE')


def controlStateToString(x):
  return {
    0:'UNDEFINED',
    1:'IDLE',
    2:'INIT',
    3:'MANUAL',
    4:'RANDOM',
    5:'PARK',
  }.get(x,'UNKNONW_VALUE')



#define	AM_STATE_UNDEFINED     0x0
#define	AM_STATE_IDLE          0x1
#define	AM_STATE_INIT          0x2
#define	AM_STATE_MANUAL        0x3
#define	AM_STATE_RANDOM        0x4
#define	AM_STATE_PARK          0x5



my_mutex = threading.Lock() 

class HRP_Teleop(object):
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
    self.mowerInternalState = 0;
    self.controlState = 0;

    self.last_terminalWidth = 0
   
    
    
 
  def fini(self):
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
 

  def callback_sensor_status(self,data):
    if (self.operationalMode != data.operationalMode) or (self.sensorStatus != data.sensorStatus) or (self.mowerInternalState != data.mowerInternalState) or (self.controlState != data.controlState):
      self.operationalMode = data.operationalMode
      self.sensorStatus = data.sensorStatus
      self.mowerInternalState = data.mowerInternalState
      self.controlState = data.controlState
      self.showstatuslines()
     
  def callback_battery_status(self,data):
    self.battAVolt = data.batteryAVoltage/1000.0
    self.battBVolt = data.batteryBVoltage/1000.0
    self.showstatuslines()
	


    
  def showstatuslines(self):
    my_mutex.acquire()

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
    if self.sensorStatus & 0x0800:
      status = status + 'Charging '
    if self.sensorStatus & 0x0080:
      status = status + 'USER_STOP '
    if self.sensorStatus & 0x0100:
      status = status + 'CFG NEEDED'
    if self.sensorStatus & 0x0200:
      status = status + 'DISC_ON '
    else:
      status = status + 'DISC_OFF '
    if self.sensorStatus & 0x0400:
      status = status + 'LOOP_ON '
    else:
      status = status + 'LOOP_OFF '
      

    
    rows, term_width = os.popen('stty size', 'r').read().split()

    if term_width < self.last_terminalWidth:
    # Terminal shrink destroys status message, show usage again
      self.loginfo('\n\n\n\n\n\n\n\n\n\n\n')
      self.print_usage()
    self.last_terminalWidth = term_width

    self.move_cursor_one_line_up() 
    self.move_cursor_one_line_up()
    self.move_cursor_one_line_up()

    msg = 'Linear %.2f  angular %.2f     batA %.1f V  batB %.1f V' % (self.speed[0],self.speed[1],self.battAVolt,self.battBVolt)
    self.logstatusline(msg)

    if self.mowerInternalState != 0:
      # We are controlling via am_driver_safe
      mowerState = mowerStateToString(self.mowerInternalState)
      controlState = controlStateToString(self.controlState)
      msg = 'ControlMode: %s            MowerState: %s' % (controlState,mowerState)
    else:
      # We are controlling via am_driver_legacy
      msg = 'ControlMode: %s            ' % (mode)
		
    
    self.logstatusline(msg)

    msg = 'Status: %s' % (status)
    self.logstatusline(msg)

    my_mutex.release()

  def run(self):
    try:
      self.init()
      self.print_usage()
      self.showstatuslines()

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
    HRP Teleop that Publish to /cmd_vel /cmd_mode
    -------------------------------------------------------
    Moving around:     Adjust Speed:    Cut high/low: I O
      Q   W   E          T  Y  U        Cut on/off:   J K
      A   S   D	                        Loop on/off:  8 9
      Z   X   C          B  N  M
 
    1:   MANUAL mode       
    2:   RANDOM Mode       5: Inject Collision (random mode)    
    P:   PARK   Mode       6: Beep	                
    
    G :   Quit
    --------------------------------------------------------
    
    """
    self.loginfo(msg)
 

  # Used to move cursor one line up, to enable printing status message repeatidly on the same line
  def move_cursor_one_line_up(self):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    print(u"\u008D\r"),
    tty.setraw(sys.stdin.fileno())

  # Used to print satus line, fills terminal width with spaces
  def logstatusline(self, str):

    # Add extra spaces and truncate to terminal width
    rows, termWidth = os.popen('stty size', 'r').read().split()
    str = str + '                                              '
    str = str[0:int(termWidth)]
    str = str + '\n'

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    print(str),
    tty.setraw(sys.stdin.fileno())
  
  # Used to print items to screen, while terminal is in funky mode
  def loginfo(self, str):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    print(str)
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
    
    if ch in self.cmd_bindings.keys():
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
    elif ch == '5':
      # Collsion inject
      mode = UInt16()
      mode.data = 0x112
      self.pub_mode.publish(mode)
    elif ch == '6':
      # Beep
      mode = UInt16()
      mode.data = 0x401
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
  rospy.init_node('HRP_keyboard_teleop')
  teleop = HRP_Teleop()
  teleop.run()
