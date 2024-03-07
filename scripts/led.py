import serial
import rospy
from std_msgs.msg import UInt8MultiArray

LED_STATES = [False, False, False]

arduino = serial.Serial("/dev/ttyUSB0", 9600)

def led_callback(msg):
  global LED_STATE
  global LED_MESSAGE
  
  color = msg.data[0]
  set = msg.data[1]
  
  if set == 0:
    if LED_STATES[color]:
      arduino.write(color.encode())
      LED_STATES[color] = False
      return
  else:
    if not LED_STATES[color]:
      arduino.write(color.encode())
      LED_STATES[color] = True
      return
    
rospy.init_node("led_control", anonymous=True)
rospy.Subscriber("led", UInt8MultiArray, led_callback)
rospy.spin()