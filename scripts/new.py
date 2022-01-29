#-------------------------------------------------- IMPORTS ---------------------------------------------------------------------------#
import rospy
from geometry_msgs.msg import Twist
from Robot_State import Bot_State
from WaypointManager import WaypointManager
from OdomSubscriber import OdomSubscriber
import numpy as np
#-------------------------------------------------------------------------------------------------------------------------------------#
#----------------------------------------------------CLASS----------------------------------------------------------------------------#
class CurveTracer:

    def __init__(self):

        rospy.init_node("CurveTracer", anonymous=True)
        self.waypoint = WaypointManager(100)
        self.odom = OdomSubscriber()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.velocity_msg = Twist()
        self.theta_precision = rospy.get_param("curve_tracer_controller/theta_precision")
        self.dist_precision = rospy.get_param("curve_tracer_controller/distance_precision")
        self.P = rospy.get_param("curve_tracer_controller/pid/p")
        self.D = rospy.get_param("curve_tracer_controller/pid/d")
        self.I = rospy.get_param("curve_tracer_controller/pid/i")
        self.state = Bot_State.IdleState.value
        
#-------------------------------------------------------------------------------------------------------------------------------------#
#------------------------------------------------------- CONTROL LOOP --------------------------------------------------------------- #
    def control_loop(self):
        rate = rospy.Rate(10)
        self.move(0, 0) 
        rospy.loginfo("Waypoints : " + str(self.waypoint.way_point()))
         
        while not rospy.is_shutdown() :
             if self.waypoint.get_next_waypoint == None:
              self.move(0,0)
              rospy.loginfo("GOAL REACHED")
             else:
              (x, y) = self.waypoint.get_next_waypoint()
              rospy.loginfo("Moving to point: ({},{}) ".format(x, y))
              self.goto(round(x, 2), round(y, 2))
              rate.sleep()
              
         #-----------------------------------------------------------------------------------------------------------------------------------#     
#--------------------------------------------------- GOTO FUNCTION ------------------------------------------------------------------ #
    def goto(self, dest_x, dest_y):
        self.state = Bot_State.Fixing_Yaw.value                   
        
        
        while self.state != Bot_State.Goal_Reached.value:        
            theta_error = self.utils(dest_x,dest_y,"theta_error")
            print("theta error: "+ str(theta_error))
             
           
            if self.state == Bot_State.Fixing_Yaw.value:          

                
                
              while (np.abs(theta_error) > self.theta_precision):  
                rospy.loginfo("Bot Current THETA: "+ str(bot_theta))
                rospy.loginfo("goal Current THETA: "+ str(theta_goal))
                rospy.loginfo("STATE:" + str(self.state))
                rospy.loginfo("FIXING YAW")
                theta_error = self.utils(dest_x,dest_y,"theta_error")
                self.fix_error(self.P,0,theta_error)  
                rospy.loginfo("THETA ERROR:" + str(theta_error))
                
                            
              rospy.loginfo("YAW FIXED ! Moving toward Goal")
              self.state = Bot_State.Moving_Straight.value
              position_error = self.utils(dest_x,dest_y,"position_error")
              rospy.loginfo("POSITION ERROR: " + str(position_error))
              
            if self.state == Bot_State.Moving_Straight.value:

                while (np.abs(position_error) > self.dist_precision )

         
                   if position_error > self.dist_precision and np.abs(theta_error) < self.theta_precision:
                       rospy.loginfo("Moving Straight - positional error but no orientation error ")
                       self.fix_error(self.P,position_error,0)

                   elif np.abs(theta_error) > self.theta_precision:
                       rospy.loginfo("Going out of line! orientaion error ")
                       while (np.abs(theta_error) > self.theta_precision):  
                         rospy.loginfo("Bot Current THETA: "+ str(bot_theta))
                         rospy.loginfo("goal Current THETA: "+ str(theta_goal))
                         rospy.loginfo("STATE:" + str(self.state))
                         rospy.loginfo("FIXING YAW")
                         theta_error = self.utils(dest_x,dest_y,"theta_error")
                         self.fix_error(self.P,0,theta_error)  
                         rospy.loginfo("THETA ERROR:" + str(theta_error))
                         #self.state = Bot_State.Fixing_Yaw.value

                    
                   elif position_error < self.dist_precision:
                       rospy.loginfo("point GOAL REACHED")
                       self.state = Bot_State.Goal_Reached.value
                   
                   theta_error = self.utils(dest_x,dest_y,"theta_error")
                              
              
              
              
#-------- ------------------------------------- MOVE ---------------------------------------------------------------------------------- #
    def move(self, linear, angular):

        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular
        self.pub.publish(self.velocity_msg)
        
        
#---------------------------------------------- FIX ERROR ----------------------------------------------------------------------------- #

    def fix_error(self, P, linear_error, orien_error):
        
        if linear_error != 0:
            
            # moving in straight line
            self.move(linear_error*P, 0)
            
        if orien_error != 0:
            p = 1*orien_error
            self.i = self.i + 2*orien_error*(0.01)
            d = 3*(orien_error-self.past_orien_error)/0.01
            # fixing the yaw
            self.move(0,p+self.i+d)
            self.past_orien_error = orien_error
            
    
#------------------------------------------- UTILS FUNCTION ---------------------------------------------------------------------------#
              
    def utils(self,dest_x,dest_y,error_type):
       # mathematical formula to calculate angle to be rotated
       theta_goal = np.arctan((dest_y - self.odom.get_position()["y"]) / dest_x - self.odom.get_position()["x"])
       bot_theta = self.odom.get_orientation("euler")["yaw"]
       theta_error = round(bot_theta - theta_goal, 2)
       position_error = np.sqrt(pow(dest_y - self.odom.get_position()["y"], 2) + pow(dest_x - self.odom.get_position()["x"], 2))
       if error_type.lower() == "theta_error":
          return theta_error
       elif error_type.lower() == "position_error":
          return position_error
                   
                  
              
              
              
              
              
# -------------------------------------------------------CLASS CALLING -------------------------------------------------------------- #
curve = CurveTracer()
curve.control_loop()
rospy.spin()

