from scipy.spatial import KDTree
import numpy as np
import random
import math
import time


class Node(object):
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent


class PRM(object):
    def __init__(self, N_SAMPLE=100, KNN=10, MAX_EDGE_LEN=5000):
        self.N_SAMPLE = N_SAMPLE
        self.KNN = KNN
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 200
        self.avoid_dist = 200

    def plan(self, vision, start_x, start_y, goal_x, goal_y):
        
        return path_x, path_y, road_map, sample_x, sample_y

    def sampling(self, start_x, start_y, goal_x, goal_y, obstree):

        return sample_x, sample_y

    def generate_roadmap(self, sample_x, sample_y, obstree):
       
        return road_map

    def check_obs(self, ix, iy, nx, ny, obstree):
    
        return False

    def dijkstra_search(self, start_x, start_y, goal_x, goal_y, road_map,

        return path_x, path_y
