from utils import *
from warehouse_map import *
from robot_fleet import *
import numpy as np
from sklearn.cluster import KMeans

"""
1. Group tasks such that each local group gets one AMR (kmeans?). Every AMR must be used.
2. Assign AMR to go to kmeans centroid
3. Based on groups cost, assign each drone a group
4. For each task, split into nearest drone and AMR
5. Assign Each drone a task and each AMR a task
"""     

class Region:
    def __init__(self, id, labels, center):
        self.id = id
        self.labels = labels
        self.center = center
        self.task_list = TaskList(tasks=[])
        self.fleet = None

    def __repr__(self):
        return (f"Region(id={self.id}\n" + 
                f"    labels={self.labels},\n" +
                f"    center={self.center},\n" +
                f"    task_list={self.task_list},\n" +
                f"    fleet={self.fleet})")

class TaskAllocator:
    def __init__(self, task_list, fleet):
        self.task_list = task_list
        
        self.fleet = fleet

        self.ground_agents = self.fleet.get_robots_as_list(robot_type="AMR")

        self.drone_agents = self.fleet.get_robots_as_list(robot_type="Drone")

        self.regions = []

        self.create_Regions()

        self.assign_Tasks2Regions()

        self.assign_AMRs2Regions()

        self.assign_Drones2Regions()

        self.assign_Tasks2Agents()


        # [print(r) for r in self.regions]


    def __repr__(self):
        return (f"WarehouseMap(task_list={self.task_list}\n" + 
                f"    fleet={self.fleet},\n" +
                f"    pick_points={self.pick_points})")
    
    def create_Regions(self):
        # kmeans group using xy coordinates -> do not care about z height for grouping
        flattened_pick_points = np.asarray([[task.pick_point.x, task.pick_point.y] for task in self.task_list.tasks])

        # Use if weighting is desired between
        # kmeans = KMeans(n_clusters=n_ground_agents, random_state=0).fit(w_vector_array[:,0:2], sample_weight = w_vector_array[:,2])
        kmeans = KMeans(
                    n_clusters=len(self.ground_agents),
                    random_state=0,
                    n_init='auto'
                ).fit(flattened_pick_points)
        
        for i in range(len(self.ground_agents)):
            labels = (kmeans.labels_ == i)

            center = Point(int(kmeans.cluster_centers_[i][0]), int(kmeans.cluster_centers_[i][1]), 5)

            self.regions.append(Region(i, labels, center))
    
    def assign_Tasks2Regions(self):
        """
        Clusters tasks into a regions
        """
        for r in self.regions:
            for i, task in enumerate(self.task_list.tasks):
                if r.labels[i]:
                    r.task_list.add_task(task)

    def assign_AMRs2Regions(self):
        """
        Assign AMR's to Regions by checking if paths to their destinations overlap
        Switch any overlapping paths to get an optimized allocation
        """
        # Initially assign ground agents (Not Optimized)
        for i, region in enumerate(self.regions):
            region.fleet = Fleet(robots={"AMR":self.ground_agents[i], "Drone":{}})

        print(f"AMR TOTAL ASSIGNMENT COST PRIOR = {sum([manhattan_dist(r.center, r.fleet.robots['AMR'].pos) for r in self.regions])}")

        for i in range(len(self.regions)):
            k = 0
            while k < len(self.regions):

                line1 = [self.regions[i].center, self.regions[i].fleet.robots["AMR"].pos]
                line2 = [self.regions[k].center, self.regions[k].fleet.robots["AMR"].pos]

                if self.paths_intersect(line1, line2):
                    # If region-to-agent paths intersect, swap assigned agents
                    agent1 = self.regions[i].fleet.robots["AMR"]
                    agent2 = self.regions[k].fleet.robots["AMR"]
                    self.regions[i].fleet.robots["AMR"] = agent2
                    self.regions[k].fleet.robots["AMR"] = agent1
                    
                    k = 0
                else:
                    k += 1

        print(f"AMR TOTAL ASSIGNMENT COST POST = {sum([manhattan_dist(r.center, r.fleet.robots['AMR'].pos) for r in self.regions])}")

    def assign_Drones2Regions(self):

        # Assign each drone to closest region
        # Based on target drone capacity, re-shuffle

        for agent in self.drone_agents:
            start_region = min(self.regions, key=lambda x: manhattan_dist(agent.pos, x.center))
            start_region.fleet.robots["Drone"][agent.robot_id] = agent
            print(start_region.fleet.robots["Drone"])
            


        # m = sum([len(r.task_list.tasks) for r in self.regions])
        # for region in self.regions:
        #     target = len(region.task_list.tasks)/m * len(self.fleet.robots["Drone"])

            # region

    def assign_Tasks2Agents(self):
        # Repeat AMR line solving allocation

        # for task in self.regions[i].task_list.tasks:
        #     self.regions[i].fleet.robots["AMR"].add_task(task)
        pass

    def paths_intersect(self, path1, path2):
        """
        Function to check whether 2 paths will intersect
        path1: start Point, end Point
        path2: start Point, end Point
        """
        if path1 != path2:
            return intersect(path1[0], path1[1], path2[0], path2[1])

        return False


        




            # TODO: populate AMR based on closest distance to kmeans centroid

            # TODO: populate drones based on closest distance to kmeans centroid? -> this is what the paper was for

            # TODO: Assign Tasks to AMR

            # TODO: Assign Tasks to Drone

