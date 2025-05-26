# Type of planner
from math import sin, cos, pi
import numpy as np
import math

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_):

        self.type=type_
        self.num_points=20
    
    def plan(self, goalPoint=(-1.0, -1.0)):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return (x, y)

    # TODO Part 6: Implement the trajectories here 
    def trajectory_planner(self, trajectory_type="parabola"):
        trajectory = []

        if trajectory_type == "parabola":
            x_values = np.linspace(0.0, 1.5, self.num_points)
            for x in x_values:
                y = x ** 2
                trajectory.append([x, y])
        elif trajectory_type == "sigmoid":
            x_values = np.linspace(0.0, 2.5, self.num_points)
            for x in x_values:
                y = 2 / (1 + math.exp(-2 * x)) - 1
                trajectory.append([x, y])
        else:
            raise ValueError("Unknown trajectory type")
   
        return trajectory
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 

