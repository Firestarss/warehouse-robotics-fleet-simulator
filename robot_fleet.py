from utils import *
from warehouse_map import WarehouseMap
from itertools import chain


class Robot:
    speed = 5.0 # ft per second
    z_fixed = None
    robot_type = None
    carrying_cap = None

    def __init__(self, robot_id, pos):
        self.robot_id = robot_id
        self.pos = pos
        self.task_list = TaskList()
        self.path = [[pos]]
        self.curr_path_idx: [0,0]
        
    def __repr__(self):
        return f"Robot('{self.robot_id}', {self.pos})"
    
    def lookup_pos(self, time_step_num):
        """
        Get the position of this robot at a given time step (int).
        """
        unnested_path = list(chain(*self.path))
        return unnested_path[time_step_num]
    
    def add_task(self, task):
        task.assigned_robot = self
        self.task_list.add_task
    
    def add_paths(self, additional_paths_list):
        self.path.extend(additional_paths_list)
    
    def update(self):
        curr_path_idx[1] += 1
        if curr_path_idx[1] == len(path[curr_path_idx[0]]):
            curr_path_idx = [curr_path_idx[0]+1, 0]
        self.pos = path[curr_path_idx[0],curr_path_idx[1]]
        
        
class Drone(Robot):
    robot_type = "Drone"
    carrying_cap = 1

class AMR(Robot):
    robot_type = "AMR"
    speed = 3.0
    z_fixed = 5
    carrying_cap = 6
    
    
class Fleet:
    def __init__(self, robots={}):
        """
        robots is a dictionary of lists of robots of the key's robot_type
        """
        self.robots = robots
            
    def __repr__(self):
        return_str = "Fleet({\n"
        for key in self.robots:
            return_str += "    '" + str(key) + "': {\n"
            for bot_id in self.robots[key]:
                return_str += ("        '" + bot_id + "': " + 
                               str(self.robots[key][bot_id]) + ",\n")
            return_str = return_str[:-1]
            return_str += " }\n"
        return_str += "})"
        return return_str
    
    def populate_by_composition(self, fleet_composition, start_points):
        """
        fleet composition is a 2D list of the form:
        [ [<robot_type1>, <num_of_robot_type1>],
          [<robot_type2>, <num_of_robot_type2>] ]
          
        """
        self.robots = {}
        robot_num = 0
        for line in fleet_composition:
            bots = {}
            if line[0] == "Drone":
                for i in range(line[1]):
                    robot_id = "D" + str(i)
                    bots[robot_id] = Robot(robot_id, start_points[robot_num])
                    robot_num += 1
                self.robots["Drone"] = bots
            if line[0] == "AMR":
                for i in range(line[1]):
                    robot_id = "A" + str(i)
                    bots[robot_id] = Robot(robot_id, start_points[robot_num])
                    robot_num += 1
                self.robots["AMR"] = bots
