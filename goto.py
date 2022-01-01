import numpy as np

def goto(dest_x,dest_y):
  
  while state != Bot_State.Goal_Reached.value:
      theta_goal = np.arctan((dest_y-pose[1])/dest_x-pose[0])
      bot_theta = pose[2]
      theta_error = round(bot_theta-theta_goal,2)
      rospy.loginfo("STATE:" + str(state))
      rospy.loginfo("THETA ERROR:" + str(theta_error))
      if state == Bot_State.Fixing_Yaw.value:
         if np.abs(theta_error)> self.theta_precision:
           rospy.loginfo("FIXING YAW ")
           fix_yaw(theta_error,1.7)
         else:
           rospy.loginfo("YAW FIXED ! Moving toward Goal")
           state = Bot_State.Moving_Straight.value
     
      elif state==Bot_State.Moving_Straight.value:
         position_error = np.sqrt(pow(dest_y - pose[1],2 ) + pow(dest_x - pose[0],2 ))
         rospy.loginfo("POSITION ERROR: " + str(position_error))
         if position_error > dist_precision and np.abs(theta_error) < theta_precision:
            rospy.loginfo("Moving Straight")
            move_straight(position_error, )
         elif np.abs(theta_error) > theta_precision: 
            rospy.loginfo("Going out of line!")
            state = 0
         elif position_error < dist_precision:
            rospy.loginfo("GOAL REACHED")
            state = 2
     
   
   
