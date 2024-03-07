#!/usr/bin/python3
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Joy
from ozurover_messages.msg import Steer
from TMotorCANControl.servo_can import TMotorManager_servo_can
import threading
import rospy
import time

# Global Variables
c_kin = {
  "b_width" : 0.5,
  "w_width" : 0.1,
  "w_radii" : 0.1
} # kinematic constants
k_c = (c_kin["b_width"] + c_kin["w_width"]) * c_kin["w_radii"]
wheel_speeds = [.0, .0, .0, .0]
teleoperation = False
teleoperation_counter = 0


# Main
if __name__=="__main__":
  rospy.init_node("ozurover_locomotion", anonymous=True)
  with TMotorManager_servo_can(motor_type="AK70-10", motor_ID=1) as motor1:
    with TMotorManager_servo_can(motor_type="AK70-10", motor_ID=2) as motor2:
      with TMotorManager_servo_can(motor_type="AK70-10", motor_ID=3) as motor3:
        with TMotorManager_servo_can(motor_type="AK70-10", motor_ID=4) as motor4:
          
          pub = rospy.Publisher("led", UInt8MultiArray, queue_size=10)
          
          # Control Threads    
          def update_loop():
            global wheel_speeds
            speed_coeff = 10.0
            while not rospy.is_shutdown():
              motor1.velocity = wheel_speeds[0]*speed_coeff
              motor2.velocity = wheel_speeds[1]*speed_coeff
              motor3.velocity = wheel_speeds[2]*speed_coeff
              motor4.velocity = wheel_speeds[3]*speed_coeff
              motor1.update()
              motor2.update()
              motor3.update()
              motor4.update()
              time.sleep(0.05)
          
          def decay_loop():
            global wheel_speeds
            global teleoperation
            global teleoperation_counter
            while not rospy.is_shutdown():
              for i in range(4):
                wheel_speeds[i] *= 0.9
              if teleoperation:
                teleoperation_counter += 1
                if teleoperation_counter > 3:
                  teleoperation = False
                  teleoperation_counter = 0
              time.sleep(1.0)
          
          # ROS Callbacks
          def cmd_callback(data):
            global wheel_speeds
            global k_c
            x = data.speed
            w = data.angle
            wheel_speeds[0] = x + (w)*k_c/2.0
            wheel_speeds[1] = x - (w)*k_c/2.0
            wheel_speeds[2] = x - (w)*k_c/2.0
            wheel_speeds[3] = x + (w)*k_c/2.0

          def joy_callback(data):
            global wheel_speeds
            global teleoperation
            global teleoperation_counter
            pub.publish([0, 0])
            pub.publish([1, 0])
            pub.publish([2, 1])
            teleoperation = True
            teleoperation_counter = 0
            wheel_speeds[0] = -data.axes[4]
            wheel_speeds[1] = data.axes[1]
            wheel_speeds[2] = data.axes[1]
            wheel_speeds[3] = -data.axes[4]
            
          # start motors
          motor1.enter_velocity_control()
          motor2.enter_velocity_control()
          motor3.enter_velocity_control()
          motor4.enter_velocity_control()
          time.sleep(1)
          
          # start threads
          update_thread = threading.Thread(target=update_loop)
          update_thread.start()
          
          decay_thread = threading.Thread(target=decay_loop)
          decay_thread.start()
          
          # start subscriptions
          rospy.Subscriber("/joy", Joy, joy_callback)
          rospy.Subscriber("/ares/cmd_vel", Steer, cmd_callback)
          rospy.spin()