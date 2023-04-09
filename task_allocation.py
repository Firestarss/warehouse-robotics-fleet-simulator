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
    def __init__(self, id, center, task_list, fleet):
        self.id = id
        self.center = center
        self.task_list = task_list
        self.fleet = fleet

    def __repr__(self):
        return (f"Region(id={self.id}\n" + 
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

        self.assign_AMRs2Regions()

        self.assign_Drones2Regions()

        # self.assign_Tasks2Agents()

        # [print(r) for r in self.regions]


    def __repr__(self):
        return (f"WarehouseMap(task_list={self.task_list}\n" + 
                f"    fleet={self.fleet},\n" +
                f"    pick_points={self.pick_points})")
    
    def create_Regions(self):
        """
        Create regions equal to the number ground agents
        Each region is initialized with an id, 
        """
        # group using xy coordinates -> do not care about z height for grouping
        flattened_pick_points = np.asarray([[task.pick_point.x, task.pick_point.y] for task in self.task_list.tasks])
        
        # Use example if weighting is desired between clusters and tasks
        # kmeans = KMeans(n_clusters=n_ground_agents, random_state=0).fit(w_vector_array[:,0:2], sample_weight = w_vector_array[:,2])
        KM_tasks = KMeans(
                    n_clusters=len(self.ground_agents),
                    random_state=0,
                    n_init='auto'
                ).fit(flattened_pick_points)
        
        for i, agent in enumerate(self.ground_agents):
            # Cast the cluster centers as 3D Points with height 5
            center = Point(int(KM_tasks.cluster_centers_[i][0]), int(KM_tasks.cluster_centers_[i][1]), 5)

            # Split the total task list into regional task lists using kmeans labels
            region_tasks =  [task for k, task in enumerate(self.task_list.tasks) if (KM_tasks.labels_[k] == i)]

            # Initialize each regional fleet with a ground agent, agents are re-allocated in assign_AMRs2Regions()
            region_robots = {"AMR":{agent.robot_id:agent}, "Drone":{}}

            # Create new region
            r = Region(
                    id=f"R{i}",
                    center=center,
                    task_list=TaskList(region_tasks),
                    fleet=Fleet(region_robots)
                )
            
            # Add to region list
            self.regions.append(r)            

    def assign_AMRs2Regions(self):
        """
        Assign AMR's to Regions by checking if paths to their destinations overlap
        Switch any overlapping paths to get an approximated optimal allocation
        """

        print(f"AMR TOTAL ASSIGNMENT COST PRIOR = {sum([manhattan_dist(r.center, list(r.fleet.robots['AMR'].values())[0].pos) for r in self.regions])}")

        for i in range(len(self.regions)):
            k = 0            
            while k < len(self.regions):
                # Must be converted to list to index the first value
                amr1 = list(self.regions[i].fleet.robots["AMR"].values())[0]
                amr2 = list(self.regions[k].fleet.robots["AMR"].values())[0]

                # AMR -> region centers: start and end points
                path1 = [amr1.pos, self.regions[i].center]
                path2 = [amr2.pos, self.regions[k].center]

                # If region-to-agent paths intersect, swap assigned agents
                if intersect(path1[0], path1[1], path2[0], path2[1]):
                    self.regions[i].fleet.robots["AMR"] = {amr2.robot_id:amr2}
                    self.regions[k].fleet.robots["AMR"] = {amr1.robot_id:amr1}
                    
                    # Restart nested loop for every intersection to check back over previous paths
                    k = 0
                else:
                    k += 1

        print(f"AMR TOTAL ASSIGNMENT COST POST = {sum([manhattan_dist(r.center, list(r.fleet.robots['AMR'].values())[0].pos) for r in self.regions])}")
        
    def assign_Drones2Regions(self):
        """
        Assign Drones to regions by balancing the drone_per_task metric of each region
        DPT = Drones per task
        """
        # Idle drones -> not currently assigned to a region
        d_idle = self.drone_agents

        # Assign every drone to a region
        while len(d_idle) > 0:
            for r in self.regions:
                if d_idle:
                    # Closest drone by manhattan distance to region centroid
                    closest_drone_index = min(range(len(d_idle)), key=lambda x: manhattan_dist(r.center, d_idle[x].pos))

                    # Add closest drone to fleet and remove from the drone list
                    r.fleet.add(d_idle.pop(closest_drone_index))

        # # Tasks Per Drone of each region if solution is optimal
        TPD_avg = len(self.task_list.tasks) / len(self.fleet.robots["Drone"])
        print("Tasks per drone avg = ", TPD_avg)

        # If a group is oversized, shift one robot to the nearest undersized group
        TPD_std = None
        last_std = 0
        while TPD_std != last_std:
            last_std = TPD_std

            TPD_std = sum([(len(t.task_list.tasks) / len(t.fleet.robots['Drone']) - TPD_avg)**2 for t in self.regions])/len(self.regions)
            
            print(f"Tasks per drone std: {TPD_std}")

            for r in self.regions:                
                # Tasks per drone for this region
                TPD_rgn = len(r.task_list.tasks) / len(r.fleet.robots["Drone"])

                # Donate a drone if region has too many. CANNOT DONATE LAST DRONE
                if TPD_rgn < TPD_avg and len(r.fleet.robots['Drone']) > 1:
                    # Recalculate all regions that still need more drones
                    undersized_regions = [region for region in self.regions if len(region.task_list.tasks) / len(region.fleet.robots["Drone"]) > TPD_avg]

                    # closest undersized region
                    r2 = min(undersized_regions, key=lambda x: manhattan_dist(r.center, x.center))

                    # closest drone in this fleet to centroid of next region
                    closest_drone = min(r.fleet.robots["Drone"].values(), key=lambda x: manhattan_dist(x.pos, r2.center))

                    # Move drone from one region's fleet to the other
                    r.fleet.remove(closest_drone)
                    r2.fleet.add(closest_drone)
                        

        print(self.regions)


    def assign_Tasks2Agents(self):
        # Repeat AMR line solving allocation

        # for task in self.regions[i].task_list.tasks:
        #     self.regions[i].fleet.robots["AMR"].add_task(task)
        pass



            # TODO: populate AMR based on closest distance to kmeans centroid

            # TODO: populate drones based on closest distance to kmeans centroid? -> this is what the paper was for

            # TODO: Assign Tasks to AMR

            # TODO: Assign Tasks to Drone