import numpy as np

def vector(self,dest_x,dest_y):
         bot_theta = self.odom.get_orientation("euler")["yaw"]      # bot making angle with x axis 
         bot_x_coordinate = round(self.odom.get_position()["x"], 2) # current x coordinate of bot
         bot_y_coordinate = round(self.odom.get_position()["y"],2)  # current y cordinate of bot
         bot_array = [np.cos(bot_theta),np.sin(bot_bot)]
         bot_vector = np.array(bot_array)
         reach_point_array = [dest_x - bot_x_coordinate , dest_y - bot_y_coordinate ]
         reach_point_vector = np.array(reach_point_array)
         dot_product = reach_point_vector.dot(bot_vector)
         bot_theta_error = dot_product/(np.sqrt((dest_x - bot_x_coordinate)**2+(dest_y - bot_y_coordinate)**2))
         bot_position_error = np.sqrt((dest_x - bot_x_coordinate)**2+(dest_y - bot_y_coordinate)**2))
         if np.cos(bot_theta)*(dest_y - bot_y_coordinate) -  np.sin(bot_theta)*(dest_x - bot_x_coordinate)
            bot_theta = -bot_theta 
         return [bot_vector,reach_point_vector,bot_theta_error, bot_position_error,bot_x_coordinate,bot_y_coordinate]
            #         0           1               2                   3              4                 5                
             
             
         
         
         
        
