from utils import *
from warehouse_map import *
from robot_fleet import *
import numpy as np
import heapq
import itertools
import math

class Node:
    """
    A node class for CA* Pathfinding
    """

    def __init__(self, position=None, parent=None):
        self.position = position
        self.parent = parent

        self.g = 0
        self.h = 0
        self.f = 0

    def get_position(self): return self.position

    def set_position(self, value): self.position = value

    def get_parent(self): return self.parent

    def set_parent(self, value): self.parent = value

    def get_g(self): return self.g

    def set_g(self, value):
        self.g = value
        self.update_f()

    def get_h(self): return self.h

    def set_h(self, value):
        self.h = value
        self.update_f()

    def get_f(self): return self.f

    def update_f(self): self.f = self.g + self.h

    def is_valid_end(self, end):
        valid_pos = self.position[:-1] == end.position[:-1]
        valid_time = self.position[-1] >= end.position[-1]
        return valid_pos and valid_time

    def __eq__(self, other) -> bool:
        return self.position == other.position
    
    def __ne__(self, other) -> bool:
        return not self.__eq__(other)
    
    def __hash__(self) -> int:
        return hash(self.position)
    
    def __repr__(self) -> str:
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
      return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
      return self.f > other.f

class PathPlanner:
    def __init__(self, map, fleet):
        self.map = map
        self.fleet = fleet
        self.occupied_nodes = set()
        
    def __repr__(self):
        pass

    def backtrack_path(self, node):
        path = []
        cur_node = node
        while cur_node is not None:
            path.append(cur_node.get_position())
            self.occupied_nodes.add(cur_node.get_position())
            cur_node = cur_node.parent

        return [self.map.cell_to_point_center(Cell(x, y, z)) for x,y,z,_ in path[::-1]]
    
    def calc_ca_star_path(self, start: Point, end: Point, start_time: int, end_time: int, path_type: str, debug=False):
        start_cell = self.map.point_to_cell(start)
        end_cell = self.map.point_to_cell(end)

        start_tuple = tuple(list(start_cell) + [start_time])
        end_tuple = tuple(list(end_cell) + [end_time])
        
        start_node = Node(start_tuple)
        end_node = Node(end_tuple)

        cityblock_estimate = sum(abs(x2-x1) for x1, x2 in zip(start_cell, end_cell))

        if debug: print(f"Planning {path_type}: {start_tuple} --> {end_tuple}")

        open_list = [start_node]
        closed_set = set()

        heapq.heapify(open_list)

        if path_type.lower().startswith("w"):
            adjacent_deltas = set(itertools.product([0], [0], [-1,0,1], [1]))
        elif path_type.lower().startswith("d"):
            adjacent_deltas = set(itertools.product([-1,0,1], [-1,0,1], [-1,0,1], [1]))
        else:
            adjacent = [(0,0), (1,0), (-1,0), (0,1), (0,-1)]
            adjacent_deltas = set((x,y,0,1) for x,y in adjacent)

        x_lim = self.map.wh_zone.x_lims[1] * self.map.resolution
        y_lim = self.map.wh_zone.y_lims[1] * self.map.resolution
        z_lim = self.map.wh_zone.z_lims[1] * self.map.resolution
        

        while len(open_list) > 0:
            if len(closed_set) > x_lim * y_lim * z_lim * (cityblock_estimate + abs(end_time - start_time)):
                raise RuntimeError(f"{terminal_colors['FAIL']}" +
                                   f"No path found:\n\tPath Type: {path_type}"+
                                   f"\n\tCells: {start_cell} --> {end_cell}" +
                                   f"\n\tPoints: {start} --> {end}" +
                                   f"{terminal_colors['ENDC']}")
            
            # Get the current node
            cur_node = heapq.heappop(open_list)
            closed_set.add(cur_node)

            # If the goal is found
            if cur_node.is_valid_end(end_node):
                return self.backtrack_path(cur_node)
            
            # Generate children
            children = []

            for delta in adjacent_deltas:
                neighbor = tuple(cur_node.get_position()[i] + delta[i] for i in range(len(cur_node.get_position())))

                # Check if in range
                lt_zero = [neighbor[i] < 0 for i in range(3)]
                gt_lim = [neighbor[0] >= x_lim, neighbor[1] >= y_lim, neighbor[2] >= z_lim]
                if True in lt_zero or True in gt_lim:
                    continue

                # Check if position collides with shelves or planned paths
                cur_cell = Cell(neighbor[0], neighbor[1], neighbor[2])
                lookahead = tuple(neighbor[i]+int(i==3) for i in range(len(neighbor)))
                robot_to_robot_collision = [node in self.occupied_nodes for node in [neighbor, lookahead]]

                if self.map.cell_blocked(cur_cell) or True in robot_to_robot_collision:
                    continue

                # Create and add new node to children
                new_node = Node(neighbor, cur_node)
                children.append(new_node)


            for child in children:
                if child in closed_set:
                    continue

                if child in open_list:
                    continue
                
                dist_with_time = math.dist(cur_node.get_position(), child.get_position())
                child.set_g(cur_node.get_g() + dist_with_time)
                child.set_h(math.dist(child.get_position()[:-1], end_tuple[:-1]))

                heapq.heappush(open_list, child)

        print(f"{terminal_colors['FAIL']}{f'No path found: {start} --> {end}'.ljust(55)}{f'|| {start_cell} --> {end_cell}'.ljust(35)}")

        return None

    
    def plan_next_region(self):
        """
        1. Plan AMR path to drop location
        2. Execute all drone tasks
            - Plan drone to pick point
            - Wait till a bit before AMR gets there
            - Plan drone to drop point
        3. Plan AMR to all drop points
        """

        # Get robots involved in region and separate into AMR and Drone
        to_plan = list(self.fleet.get_robots_with_unplanned_tasks())
        current_amr = [robot for robot in to_plan if robot.robot_id.startswith("A")]
        current_drones = [robot for robot in to_plan if robot.robot_id.startswith("D")]
        use_amr = False

        if len(current_amr):
            current_amr = current_amr[0]
            use_amr = True

        # Plan AMR path to pick location
        if use_amr:
            amr_to_pick_path = self.calc_ca_star_path(
                current_amr.get_last_path_pos(),
                current_amr.get_next_task().pick_point,
                current_amr.path_time(),
                current_amr.path_time(),
                current_amr.robot_id)
        
            current_amr.add_path_segment(amr_to_pick_path)
            current_amr.get_next_task().picked = current_amr.path_time()

        for drone in current_drones:
            while drone.get_next_task():
                # Plan Drone path to pick location
                drone_to_pick_path = self.calc_ca_star_path(
                    drone.get_last_path_pos(),
                    drone.get_next_task().pick_point,
                    drone.path_time(),
                    drone.path_time(),
                    drone.robot_id)
                
                drone.add_path_segment(drone_to_pick_path)
                drone.get_next_task().picked = drone.path_time()

                # Plan Drone path to wait at pick location
                if use_amr:
                    estimated_dist = diag_dist(drone.get_last_path_pos(),
                                                    current_amr.get_last_path_pos())
                    estimated_resume_time = current_amr.path_time() - int(estimated_dist)

                    drone_wait_path = self.calc_ca_star_path(
                        drone.get_last_path_pos(),
                        drone.get_last_path_pos(),
                        drone.path_time(),
                        estimated_resume_time,
                        "W")
                    
                    drone.add_path_segment(drone_wait_path)

                if use_amr:
                    drop_time = current_amr.path_time()
                else:
                    drop_time = drone.path_time()

                # Plan Drone path to drop/handoff location
                drone_to_drop_path = self.calc_ca_star_path(
                    drone.get_last_path_pos(),
                    drone.get_current_task().drop_point,
                    drone.path_time(),
                    drop_time,
                    drone.robot_id)
                
                drone.add_path_segment(drone_to_drop_path)
                drone.get_current_task().done = drone.path_time()

        
        # Plan how long AMR waits for deliveries
        if use_amr:
            leave_time = max([drone.path_time() for drone in current_drones])
            amr_wait_path = amr_to_pick_path = self.calc_ca_star_path(
                current_amr.get_last_path_pos(),
                current_amr.get_last_path_pos(),
                current_amr.path_time(),
                leave_time,
                "W")
            
            current_amr.add_path_segment(amr_wait_path)
        
        # Plan AMR path to all drop points
        if use_amr:
            for drop in current_amr.get_current_task().drop_points:
                amr_next_drop_path = self.calc_ca_star_path(
                    current_amr.get_last_path_pos(),
                    drop,
                    current_amr.path_time(),
                    current_amr.path_time(),
                    current_amr.robot_id)
                
                current_amr.add_path_segment(amr_next_drop_path)
            current_amr.get_current_task().done = current_amr.path_time()

        # for path in current_amr.path:
        #     print(path)