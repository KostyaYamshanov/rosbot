#!/usr/bin/env python
# license removed for brevity

import math
import numpy as np
import rospy

class RobotState():

    def __init__(self, x = 0.0, y = 0.0, yaw = 0.0, vx = 0.0, vy = 0.0, w = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.w = w


class Goal():

    def __init__(self, x = 0.0, y = 0.0):
        self.x = x
        self.y = y

class Params():

    def __init__(self):
        self.v_max = 5.0
        self.w_max = 1.5
        self.xy_margin_squared = 0.05



class Rosbot():


    def __init__(self):
        self.odom_state = RobotState()
        self.model_state = RobotState()
        self.params = Params()


    def set_odom_state(self, odom_state):
        self.odom_state = odom_state


    def dist_to_goal_L2(self, goal):             
        """
        param goal - Goal Class object
        """
        return  (goal.x - self.odom_state.x)**2 + (goal.y - self.odom_state.y)**2
            

    def goal_reached(self, goal):
        return self.dist_to_goal_L2(goal) <= self.params.xy_margin_squared



    def calculate_contol(self, state, goal):
        """
        Given state calculate control towards goal
        """
        rho = self.dist_to_goal_L2(goal)

        azim_goal = math.atan2((goal.y - state.y),(goal.x - state.x))
        alpha = azim_goal - state.yaw                                        

        if (abs(alpha) > math.pi): 
            alpha -= np.sign(alpha) * 2 * math.pi

        v = self.params.v_max * math.tanh(rho) * math.cos(alpha)
        w = self.params.w_max * alpha + math.tanh(rho)*math.sin(alpha)*math.cos(alpha)/rho

        return v, w


    def update_model_state(self, v, w, freq):

        eps_w = 0.0001
        dt = 1.0 / freq

        if abs(w) > eps_w:
            rho = v / w
            
            # step 1. Calc new robot position relative to its previous pose

            x_r = rho * math.sin(w*dt)
            y_r = rho * (1 - math.cos(w*dt))

            # step 2. Transfrom this point to map fixed coordinate system taking into account current robot pose

            self.model_state.x += x_r * math.cos(self.model_state.yaw) - y_r * math.sin(self.model_state.yaw)
            self.model_state.y += x_r * math.sin(self.model_state.yaw) + y_r * math.cos(self.model_state.yaw)
            self.model_state.yaw +=  w * dt
        
        else:
            self.model_state.x += v * dt * math.cos(self.model_state.yaw)
            self.model_state.y += v * dt * math.sin(self.model_state.yaw)

        return self.model_state


