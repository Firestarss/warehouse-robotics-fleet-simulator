import random
import math
import numpy as np

class Location:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"Location({self.x},{self.y},{self.z})"
    
    def __iter__(self):
        return iter((self.x, self.y, self.z))
    
    def as_str(self):
        return f"({self.x},{self.y},{self.z})"
    
    def dist_manhattan(self, point2):
        dx = abs(self.x - point2.x)
        dy = abs(self.y - point2.y)
        dz = abs(self.z - point2.z)
        return dx + dy + dz

    def dist_diag(self, point2):
        dx = abs(self.x - point2.x)
        dy = abs(self.y - point2.y)
        dz = abs(self.z - point2.z)
        return math.sqrt(dx + dy + dz)

class Point(Location):
    def __repr__(self):
        return f"Point({self.x},{self.y},{self.z})"

class Cell(Location):
    def __repr__(self):
        return f"Cell({self.x},{self.y},{self.z})"

class Zone:
    def __init__(self, x_lims, y_lims, z_lims, zone_id=None):
        self.x_lims = x_lims
        self.y_lims = y_lims
        self.z_lims = z_lims
        self.zone_id = zone_id

        self.corners = [Location(x_lims[0], y_lims[0], z_lims[0]),
                        Location(x_lims[1], y_lims[1], z_lims[1])]

    def __repr__(self):
        return f"Zone({self.x_lims}, {self.y_lims}, {self.z_lims})"

    def is_inside(self, x, y, z):
        if self.x_lims[0] <= x <= self.x_lims[1]:
            if self.y_lims[0] <= y <= self.y_lims[1]:
                if self.z_lims[0] <= z <= self.z_lims[1]:
                    return True
        return False
    
    def x_range(self):
        return self.x_lims[1] - self.x_lims[0]
    def y_range(self):
        return self.y_lims[1] - self.y_lims[0]
    def z_range(self):
        return self.z_lims[1] - self.z_lims[0]
    
    def mesh_params(self):
        """
        Returns a tuple of lists with values x, y, z, i, j, and k values for the
        plotly.graph_objects.mesh3d function. 
        Return format is (x_list, y_list, z_list, i_list, j_list, k_list).
        """
        
        x_list, y_list, z_list, i_list, j_list, k_list = [], [], [], [], [], []
        
        for z in self.z_lims:
            for y in self.y_lims:
                for x in self.x_lims:
                    x_list.append(x)
                    y_list.append(y)
                    z_list.append(z)
        
        i_list = [0, 0, 0, 0, 0, 0, 7, 7, 7, 7, 7, 7]
        j_list = [3, 3, 5, 5, 6, 6, 1, 1, 2, 2, 4, 4]
        k_list = [1, 2, 1, 4, 2, 4, 3, 5, 3, 6, 5, 6]
        
        return (x_list, y_list, z_list, i_list, j_list, k_list)
        

class PointZone(Zone):
    def __repr__(self):
        return f"PointZone({self.x_lims}, {self.y_lims}, {self.z_lims})"

class CellZone(Zone):
    def __repr__(self):
        return f"CellZone({self.x_lims}, {self.y_lims}, {self.z_lims})"

class Task: 
    def __init__(self, task_id, pick_point, drop_points, 
                 robot=None, started=False, picked=False, done=False):
        self.task_id = task_id
        self.pick_point = pick_point
        self.drop_points = [drop_points] if type(drop_points) is not list else drop_points
        self.assigned_robot = robot
        self.started = started
        self.picked = picked
        self.done = done
        self.start_time = None
        self.end_time = None
    
    def __repr__(self):
        if self.assigned_robot == None:
            return f"Task('{self.task_id}', {self.pick_point}, {self.drop_points})"
        else:
            if self.assigned_robot == None:
                robot = "None"
            else: robot = self.assigned_robot.robot_id
            return f"Task('{self.task_id}', {self.pick_point}, {self.drop_points}, robot={robot}, started={self.started}, picked={self.picked}, done={self.done})"
    
    def drop_point(self):
        return self.drop_points[0]

class TaskList:
    def __init__(self, tasks=None):
        if tasks == None:
            self.tasks = []
        else: self.tasks = tasks
        
    
    def __repr__(self):
        if len(self.tasks) == 0:
            return "TaskList([])"
        tasklist_rep = "TaskList([ "
        indent = " "*len(tasklist_rep)

        if self.tasks == []:
            return "No Tasks"

        tasklist_rep += self.tasks[0].__repr__()
        for i in range(1,len(self.tasks)):
            tasklist_rep += ",\n" + indent + self.tasks[i].__repr__()
        tasklist_rep += " ])"
        return tasklist_rep
        
    def add_task(self, task):
        self.tasks.append(task)

    def remove_task(self, task):
        i = self.tasks.index(task)
        self.tasks.pop(i)

    def get_current_task(self):
        for task in self.tasks:
            if task.started and not task.done:
                return task
        return None
            
    def get_next_task(self):
        for task in self.tasks:
            if not task.started:
                return task
        return None
    
    def get_last_task(self):
        if len(self.tasks) > 0:
            return self.tasks[-1]
        return None
    
    def populate_randomly(self, pick_points, drop_points, num_tasks, task_id_prefix="T"):
        self.tasks = []
        for i in range(num_tasks):
            task_id = task_id_prefix + str(i)
            pick_point = random.choice(pick_points)
            drop_point = random.choice(drop_points)
            self.tasks.append(Task(task_id, pick_point, drop_point))

def manhattan_dist(point1, point2):
    dx = abs(point1.x - point2.x)
    dy = abs(point1.y - point2.y)
    dz = abs(point1.z - point2.z)
    return dx + dy + dz

def diag_dist(point1, point2):
    dx = abs(point1.x - point2.x)
    dy = abs(point1.y - point2.y)
    dz = abs(point1.z - point2.z)
    return math.sqrt(dx + dy + dz)

def ccw(A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
    
def intersect(A,B,C,D):
    """
    Return true if line segments AB and CD intersect
    Return false if segments are equal

    path1: start Point, end Point
    path2: start Point, end Point
    """
    if not ((A==C) & (B==D)):
        return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
    return False

terminal_colors = {
    "HEADER" : '\033[95m',
    "OKBLUE" : '\033[94m',
    "OKCYAN" : '\033[96m',
    "OKGREEN" : '\033[92m',
    "WARNING" : '\033[93m',
    "FAIL" : '\033[91m',
    "ENDC" : '\033[0m',
    "BOLD" : '\033[1m',
    "UNDERLINE" : '\033[4m'
    }