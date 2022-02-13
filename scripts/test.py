#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from Robot_State import Bot_State
from OdomSubscriber import OdomSubscriber
from bug_algorithm import Object_Avoider
import numpy as np


                                     #-------------CLASS--------------#
class CurveTracer:

    def __init__(self):

        rospy.init_node("test", anonymous=True)
        self.odom = OdomSubscriber()
        self.obj_avoid = Object_Avoider()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity_msg = Twist()
        self.theta_precision = 0.1
        self.dist_precision = 0.1
        self.P = 3
        self.D = 0
        self.I = 0
        self.state = Bot_State.IdleState.value
        
                                  
                        
         
                                 #--------------GOTO FUNCTION--------------#
    def goto(self,dest_x,dest_y):
     self.state = Bot_State.IdleState.value
    
     while self.state != Bot_State.Goal_Reached.value:
     
         print(" ")
         print("************************")
         print("****** BOT INFO ********")
         print("************************")
         print(" ")
         info_call = self.info(dest_x,dest_y)
         bot_theta_error =  info_call[0]
         bot_position_error = info_call[1] 
         
         if self.obj_avoid.state_value() != 0:
          rospy.loginfo("object detected")
          value  =  self._state_()
                    
        
         while((np.abs(bot_theta_error) > self.theta_precision) or (np.abs(bot_position_error) > self.dist_precision)):
              
              while (np.abs(bot_theta_error) > self.theta_precision) : 
                print(" ") 
                print("************************")
                print("***** FIXING YAW *******")
                print("************************")
                print(" ")
                info_call = self.info(dest_x,dest_y)
                bot_theta_error =  info_call[0]
                bot_position_error = info_call[1]
                self.fix_error(0, bot_theta_error) 
                if self.obj_avoid.state_value() != 0:
                      rospy.loginfo("object detected")
                      value  =  self._state_()
                
                            
              rospy.loginfo (" YAW FIXED ! Moving straight .. " )                    
              rospy.loginfo("going to:         ---> " + "x: " + str(dest_x) + " " + "y: " + str(dest_y))
              
              while (np.abs(bot_position_error) > self.dist_precision):
                       print(" ")
                       print("************************")
                       print("*** Moving Straight ****")
                       print("************************") 
                       print(" ")                                                              
                       info_call = self.info(dest_x,dest_y)                       
                       bot_theta_error =  info_call[0]
                       bot_position_error = info_call[1] 
                       self.fix_error(bot_position_error ,0) 
                       if self.obj_avoid.state_value() != 0:
                              rospy.loginfo("object detected")
                              value  =  self._state_()  
                       
                       
                       while (np.abs(bot_theta_error) > self.theta_precision) : 
                          print(" ")
                          print("*********************************")
                          print("* FIXING YAW && MOVING STRAIGHT *")
                          print("*********************************")    
                          print(" ")                      
                          info_call = self.info(dest_x,dest_y)
                          bot_theta_error =  info_call[0]
                          bot_position_error = info_call[1]                     
                          self.fix_error(0,bot_theta_error) 
                          if self.obj_avoid.state_value() != 0:
                                 rospy.loginfo("object detected")
                                 value  =  self._state_()

                  
                    
         if bot_position_error <= self.dist_precision and bot_theta_error <= self.theta_precision :
                  print(" ")
                  print("*****************************")
                  print("** HURRAY !! GOAL REACHED ***")
                  print("*****************************")  
                  print(" ")                    
                  self.state = Bot_State.Goal_Reached.value 
                     
                              
              
              
              
                                    #--------------MOVE--------------#
    def move(self, linear, angular):

        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular
        self.pub.publish(self.velocity_msg)
        
        
                                    #--------------FIX ERROR--------------#

    def fix_error(self, linear_error, orien_error):
        
        if linear_error != 0:
            
            # moving in straight line
            self.move(self.P*linear_error, 0)
            
        if orien_error != 0:           
            # fixing the yaw     
             self.move(0,self.P*-1*orien_error)
           
            
    
                                    #--------------UTILS FUNCTION--------------#
              
    def vector(self,dest_x,dest_y):
    
         bot_theta = self.odom.get_orientation("euler")["yaw"]      
         bot_x_coordinate = round(self.odom.get_position()["x"], 2)          
         bot_y_coordinate = round(self.odom.get_position()["y"], 2)                    
         bot_vector = np.array([np.cos(bot_theta), np.sin(bot_theta)])                         
         reach_point_vector = np.array([dest_x - bot_x_coordinate, dest_y - bot_y_coordinate ])         
         dot_product = reach_point_vector.dot(bot_vector)         
         bot_position_error = np.sqrt((dest_x - bot_x_coordinate)**2+(dest_y - bot_y_coordinate)**2)         
         bot_theta_error = np.arccos(dot_product/bot_position_error)         
         
         if np.cos(bot_theta)*(dest_y - bot_y_coordinate) -  np.sin(bot_theta)*(dest_x - bot_x_coordinate) > 0 :
            bot_theta_error = - bot_theta_error 
            
         return [bot_vector,reach_point_vector,bot_theta_error, bot_position_error,bot_x_coordinate,bot_y_coordinate,bot_theta]
     
    def info(self, dest_x, dest_y):
       
         ls  = self.vector(dest_x,dest_y)
         bot_theta_error    = ls[2]
         bot_position_error = ls[3]
         rospy.loginfo("bot vector:       ---> " + "{:.2f}".format(ls[0][0])+" i " + "+ " + "{:.2f}".format(ls[0][1]) + " j")
         rospy.loginfo("position vector:  ---> " + "{:.2f}".format(ls[1][0])+" i " + "+ " + "{:.2f}".format(ls[1][1]) + " j")
         rospy.loginfo("theta error:      ---> " + "{:.2f}".format(ls[2]))
         rospy.loginfo("position error:   ---> " + "{:.2f}".format(ls[3]))
         rospy.loginfo("current position: ---> " + "x: {:.2f}".format(ls[4]) + " "   + "y: {:.2f}".format(ls[5])) 
         rospy.loginfo("going to:         ---> " + "x: " + str(dest_x) + " " + "y: " + str(dest_y)) 
         
         return [bot_theta_error,bot_position_error]
         
                                     #-------------------- state ----------------------------#
    def _state_(self):
        
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
    
            if self.obj_avoid.state_value()== 0:
                return 
            elif self.obj_avoid.state_value() == 1:
                self.obj_avoid.turn_left()
            elif self.obj_avoid.state_value() == 2:
                self.obj_avoid.follow_the_wall()
                pass
            else:
                rospy.logerr('Unknown state!')
                        
            rate.sleep()
   
         
                   
                  
              
              
              
              
              
                                    #--------------CLASS CALLING--------------#  
curve = CurveTracer()
curve.goto(0,0)
curve.move(0,0)






