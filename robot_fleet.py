from utils import *
from warehouse_map import WarehouseMap
from itertools import chain


class Robot:
    speed = 5.0 # ft per second
    z_fixed = None
    robot_type = None
    carrying_cap = None
    x_buffer_lims = None
    y_buffer_lims = None
    z_buffer_lims = None

    def __init__(self, robot_id, pos):
        self.robot_id = robot_id
        self.pos = pos
        self.task_list = TaskList(tasks=[])
        self.path = []
        self.curr_path_idx = [0,0]

    def __repr__(self):
        # return f"Robot('{self.robot_id}', {self.pos})"
        return f"Robot('{self.robot_id}', {self.pos}, {self.task_list})"

    def __lt__(self, other):
        # So that robots can be sorted by path length - this is untested, comment this out if behavior is weird. Inequality might be flipped.
        return self.path_len() < other.path_len()

    def path_len(self, r_end=None):
        if r_end == None:
            if self.path:
                return len(list(chain(*self.path)))
            else:
                return 0
        else: return len(list(chain(*self.path[:r_end])))

    def path_time(self):
        return max(self.path_len() - 1,0)
    
    def get_last_path_pos(self):
        if self.path_len() > 0:
            return self.lookup_pos(self.path_len())
        return self.pos

    def lookup_pos(self, time_step_num):
        """
        Get the position of this robot at a given time step (int).
        """
        unnested_path = list(chain(*self.path))
        if len(unnested_path) == 0:
            return self.pos
        if time_step_num >= len(unnested_path):
            return unnested_path[-1]
        return unnested_path[time_step_num]

    def lookup_pos_and_remaining_path_len(self, time_step_num):
        """
        Get the position of the robot and the given time step and the number of steps remaining on it's path. If the path ended before the given timestep, give the number of steps overshot as a negative number.
        """
        unnested_path = list(chain(*self.path))
        path_len = len(unnested_path)
        remaining_path = path_len - time_step_num - 1
        if path_len == 0:
            pos = self.pos
        elif time_step_num >= path_len:
            pos = unnested_path[-1]
        else:
            pos = unnested_path[time_step_num]
        return (pos, remaining_path)
    
    def lookup_last_assigned_pos(self):
        """
        Get the last assigned location of the robot (time invariant). If no tasks have been assigned, return current pos.
        """
        last_task = self.task_list.get_last_task()
        if last_task:
            return last_task.drop_points[-1]
        return self.pos

    def add_task(self, task):
        task.assigned_robot = self
        self.task_list.add_task(task)

    def remove_task(self, task):
        task.assigned_robot = None
        self.task_list.add_task(task)

    def add_path_segment(self, path):
        self.path.append(path)

    def get_current_task(self): return self.task_list.get_current_task()

    def get_next_task(self): return self.task_list.get_next_task()

    def update(self):
        curr_path_idx[1] += 1
        if curr_path_idx[1] == len(self.path[curr_path_idx[0]]):
            curr_path_idx = [curr_path_idx[0]+1, 0]
        self.pos = self.path[curr_path_idx[0],curr_path_idx[1]]

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
            return_str = return_str[:-2]
            return_str += " }\n"
        return_str += "})"
        return return_str
    
    def add(self, agent):
        # print(f"Adding {agent} to {self.robots}")
        # print(f"Robot Type: {agent.robot_type}, Robot ID: {agent.robot_id}")
        self.robots[agent.robot_type][agent.robot_id] = agent
        # print(self.robots)
    
    def remove(self, agent):
        # print(f"Removing {agent} from {self.robots}")
        # print(f"Robot Type: {agent.robot_type}, Robot ID: {agent.robot_id}")
        self.robots[agent.robot_type].pop(agent.robot_id)
        # print(self.robots)

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
                    bots[robot_id] = Drone(robot_id, start_points[robot_num])
                    robot_num += 1
                self.robots["Drone"] = bots
            if line[0] == "AMR":
                for i in range(line[1]):
                    robot_id = "A" + str(i)
                    bots[robot_id] = AMR(robot_id, start_points[robot_num])
                    robot_num += 1
                self.robots["AMR"] = bots

    def get_robots_as_list(self, robot_type="All"):
        if robot_type == "All":
            robot_list = []
            for robot_type in self.robots:
                robot_list.extend(list(self.robots[robot_type].values()))
            return robot_list
        else:
            return list(self.robots[robot_type].values())
        
    def get_fleet_tasks(self, time=None, status=None, robot_type="All"):
        robots = self.get_robots_as_list(robot_type)
        if time == None:
            nested_tasks = [robot.task_list.tasks for robot in robots]
            return list(chain(*nested_tasks))

    def longest_path_len(self, robot_type="All"):
        robot_list = self.get_robots_as_list(robot_type)
        path_lens = [bot.path_len() for bot in robot_list]
        return max(path_lens)

    def closest_robots(self, point, timestep, robot_type="Drone"):
        """
        Returns a list of the robots closest to a given point at a given time.
        """
        robot_list = self.get_robots_as_list(robot_type)
        dists = []
        bots_dists = []
        for bot in robot_list:
            bot_pos = bot.lookup_pos(timestep)
            dist = point.dist_manhattan(bot_pos)
            dists.append(dist)
            bots_dists.append([bot, dist])
        min_dist = min(dists)
        closest_bots = [bot_dist[0] for bot_dist in bots_dists 
                        if bot_dist[1]==min_dist]
        return closest_bots

    def closest_robots_at_end_path(self, point, robot_type="Drone"):
        """
        Returns a list of the robots closest to the given point when at the end
        of their paths. Robots that finish with their currently planned paths
        sooner are considered to be that much "closer."
        """
        robot_list = self.get_robots_as_list(robot_type=robot_type)
        # dists = []
        bots_dists = []
        time = self.longest_path_len(robot_type)
        for bot in robot_list:
            (bot_pos, r_steps) = bot.lookup_pos_and_remaining_path_len(time)
            dist = point.dist_manhattan(bot_pos)+r_steps
            # dists.append(dist)
            bots_dists.append([bot, dist])

        # closest_bot = min(bots_dists, key=lambda x: x[1])
        closest_bots = [bot_dist[0] for bot_dist in sorted(bots_dists, key=lambda x: x[1])]
        # min_dist = min(dists)
        # closest_bots = [bot_dist[0] for bot_dist in bots_dists 
        #                 if bot_dist[1]==min_dist]
        return closest_bots
    
    def get_robots_with_unplanned_tasks(self):
        for robot_type, robots in self.robots.items():
            for robot_id, robot in robots.items():
                if robot.get_next_task() is not None or robot.get_current_task() is not None:
                    yield robot

    def is_fleet_done(self):
        for _ in self.get_robots_with_unplanned_tasks():
            return False
        return True