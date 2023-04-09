import math

class APF:

  # constants
  K_ATTRACT = 0.01
  K_REPEL = 100000.0
  NUM_SENSORS = 7
  RADIUS_MAX = 500.0
  
  # function to calculate net force
  @staticmethod
  def obs_avoid_APF(distances, dest_dist): # distance from destination
      obs_angle = [-0.610865, -1.5708, -2.53072742, 2.53072742, 1.5708, 0.610865, -3.14159]  # angle of each sensor in radians
      #convert to dec cos: 0.81915, 0,   -0.81915,  -0.81915,  0,    0.81915, -1
      #convert to dec sin: -0.57286, -1,   -0.573576,  0.573576,  1,    0.573576, 
      # convert sensor readings to x,y coordinates
      obs_x = [APF.get_x(obs_angle[i], distances[i]) for i in range(APF.NUM_SENSORS)]
      obs_y = [APF.get_y(obs_angle[i], distances[i]) for i in range(APF.NUM_SENSORS)]

  
      # calculate attractive force
      fx_attract = APF.K_ATTRACT * (dest_dist - 500)
      fy_attract = 0 # this will always be 0, because we are always facing the subject
  
      # calculate repulsive forces
      fx_repels = [APF.repel_x(0, obs_x[i], 0, obs_y[i], distances[i]) for i in range(APF.NUM_SENSORS)]
      fy_repels = [APF.repel_y(0, obs_x[i], 0, obs_y[i], distances[i]) for i in range(APF.NUM_SENSORS)]
  
      # calculate net force
      #fx_net = sum(fx_repels) + fx_attract
      #fy_net = sum(fy_repels) + fy_attract
      fx_net = sum(fx_repels)
      fy_net = sum(fy_repels)
      return (fx_net, fy_net)
  

  # helper function to calculate x-coordinate
  def get_x(angle, distance):
      x_val = distance * math.cos(angle)
      return x_val
    
  
  # helper function to calculate y-coordinate
  def get_y(angle, distance):
      y_val = distance * math.sin(angle)
      return y_val
  
  # helper function to calculate repulsive force in x-direction
  def repel_x(x, xob, y, yob, distance):
      fx_repel = 0.0
      if distance <= APF.RADIUS_MAX:
        #  fx_repel = 0.5 * APF.K_REPEL * ((1 / distance) - (1 / APF.RADIUS_MAX)) * (0.5 * pow(((xob ) * (xob ) + (yob ) * (yob )), -0.5) * (-2 * (xob )) / distance)
          fx_repel = 0.5 * APF.K_REPEL * ((1.0 / distance) - (1.0 / APF.RADIUS_MAX)) * 0.5 *(1.0/distance) * (-2.0 * (xob )) / distance
      else:
          fx_repel = 0.0
      return fx_repel
  
  # helper function to calculate repulsive force in y-direction
  def repel_y(x, xob, y, yob, distance):
      fy_repel = 0.0
      if distance <= APF.RADIUS_MAX:
        #  fy_repel = 0.5 * APF.K_REPEL * ((1 / distance) - (1 / APF.RADIUS_MAX)) * (0.5 * pow(((xob ) * (xob ) + (yob ) * (yob )), -0.5) * (-2 * (yob )) / distance)
          fy_repel = 0.5 * APF.K_REPEL * ((1.0 / distance) - (1.0 / APF.RADIUS_MAX)) * 0.5 * (1.0/distance) * (-2.0 * (yob )) / distance
      else:
          fy_repel = 0.0
      return fy_repel
