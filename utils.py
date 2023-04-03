import random

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        
    def __repr__(self):
        return f"Point({self.x},{self.y},{self.z})"

class Zone:
    def __init__(self, x_lims, y_lims, z_lims, zone_id=None):
        self.x_lims = x_lims
        self.y_lims = y_lims
        self.z_lims = z_lims
        self.zone_id = zone_id

        self.corners = [Point(x_lims[0], y_lims[0], z_lims[0]),
                        Point(x_lims[1], y_lims[1], z_lims[1])]

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

class Task: 
    def __init__(self, task_id, pick_point, drop_point, 
                 robot=None, started=False, done=False):
        self.task_id = task_id
        self.pick_point = pick_point
        self.drop_point = drop_point
        self.assigned_robot = robot
        self.started = started
        self.done = done
    
    def __repr__(self):
        if self.assigned_robot == None:
            return f"Task('{self.task_id}', {self.pick_point}, {self.drop_point})"
        else:
            return f"Task('{self.task_id}', {self.pick_point}, {self.drop_point}, robot= {self.robot}, started={self.started}, done={self.done})"

class TaskList:
    def __init__(self, tasks=[]):
        self.tasks = tasks
    
    def __repr__(self):
        tasklist_rep = "TaskList([ "
        indent = " "*len(tasklist_rep)
        tasklist_rep += self.tasks[0].__repr__() + ",\n"
        for i in range(1, len(self.tasks)-1):
            tasklist_rep += indent + self.tasks[i].__repr__() + ",\n"
        tasklist_rep += indent + self.tasks[i].__repr__() + " ])"
        return tasklist_rep
    
    def populate_randomly(self, pick_points, drop_points, num_tasks, task_id_prefix=""):
        for i in range(num_tasks):
            task_id = task_id_prefix + str(i)
            pick_point = random.choice(pick_points)
            drop_point = random.choice(drop_points)
            self.tasks.append(Task(task_id, pick_point, drop_point))

# pick_points = [Point(0,1,1), Point(0,2,2), Point(0,3,3), Point(0,4,4)]
# drop_points = [Point(1,1,1), Point(1,2,2), Point(1,3,3), Point(1,4,4)]
# tl = TaskList()
# tl.populate_randomly(pick_points, drop_points, 5, "task_")
# print(tl)
