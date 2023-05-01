from utils import *
from warehouse_map import *
from robot_fleet import *
from sklearn.cluster import KMeans
from k_means_constrained import KMeansConstrained
import statistics

# ============================================================================================================
# Region Class
# ============================================================================================================

class Region:
    def __init__(self, id, center, task_list, fleet):
        self.id = id
        self.center = center
        self.task_list = task_list
        self.fleet = fleet

        # Handoff point between Drones and AMR's for this region (used to be region centroid but is now based on closest task)
        self.pick_point = None

        # List of AMR tasks held so that they can be merged at the end of task allocation
        self.amr_tasks = []

    def __repr__(self):
        return (f"Region(id={self.id}\n" + 
                f"    center={self.center},\n" +
                f"    task_list={self.task_list},\n" +
                f"    fleet={self.fleet})")
    
    def split_task_to_AMR(self, task):
        """
        Given a task, split it up by this region's pick point (defined outside)
        Return the drone task for assignment,
        save AMR tasks for easy combination in merge_amr_tasks()
        """
        if "AMR" in self.fleet.robots.keys():
            # Drone task is to pick at the task pick point and drop at the region pick point   
            drone_task = Task(task.task_id, task.pick_point, self.pick_point)
            
            # Save second half of task for merging later
            self.amr_tasks.append(Task(task.task_id, self.pick_point, task.drop_point()))

            return drone_task
        
        # If there is no AMR in the fleet, do not split the task
        return task
    
    def merge_amr_tasks(self):
        """
        Given that the list 'self.amr_tasks' has been populated,
        order drop points ccw based on this region's center,
        add all drop points to a single task,
        and assign to AMR
        """
        if "AMR" in self.fleet.robots.keys():
            # Order drop points to be visited ccw
            sorted_tasks = sorted(self.amr_tasks, key=lambda task: math.atan2(task.drop_point().y - self.center.y, task.drop_point().x - self.center.x))

            # Give task regional id and drop points list
            task = Task(f"{self.id}", self.pick_point, [task.drop_point() for task in sorted_tasks])

            # Assign extended task to regional AMR
            self.fleet.get_robots_as_list(robot_type="AMR")[0].add_task(task)

# ============================================================================================================
# TASK ALLOCATOR
# ============================================================================================================

class TaskAllocator:
    def __init__(self, task_list, fleet, resolution, region_type="homogeneous", allocation_type="no_handoff"):
        self.task_list = task_list
        
        self.fleet = fleet

        self.resolution = resolution

        self.regions = []

        if self.check_for_input_error():
            return
        
        region_options = {
            "homogeneous" :                 self.homogeneous_region,
            "AMR_regions" :                 self.AMR_regions,
            "AMR_regions_hypercluster" :    self.AMR_regions_hypercluster,
            "sized_regions" :               self.sized_regions,
            "sized_regions_hypercluster" :  self.sized_regions_hypercluster
        }

        allocation_options = {
            "no_handoff" :                  self.no_handoff,
            "center_handoff" :              self.center_handoff,
            "closest2AMR_handoff" :         self.closest2AMR_handoff,
            "closest2drop_handoff" :        self.closest2drop_handoff
        }

        self.cluster_regions = region_options[region_type]

        self.allocate_tasks = allocation_options[allocation_type]

    def __repr__(self):
        return (f"TaskAllocator(task_list={self.task_list}\n" + 
                f"    fleet={self.fleet},\n" +
                f"    pick_points={self.pick_points})")
    
    def check_for_input_error(self):
        n_tasks = len(self.task_list.tasks)
        n_ground = len(self.fleet.get_robots_as_list(robot_type="AMR"))
        n_drone = len(self.fleet.get_robots_as_list(robot_type="Drone"))
        
        if n_tasks < n_ground:
            print(f"TASK ALLOCATION WARNING: # of ground agents '{n_ground}' cannot exceed # of tasks '{n_tasks}'")
            # return True
        
        if n_tasks < n_drone:
            print(f"TASK ALLOCATION WARNING: # of drones '{n_drone}' is larger than # of tasks '{n_tasks}', drones will be unused")        

        return False

# ============================================================================================================
# HIGH-LEVEL Region Clustering functions

    def homogeneous_region(self):
        self.create_homogeneous_region()

    def AMR_regions(self):
        self.create_AMR_regions(hyperclustering=False)

    def AMR_regions_hypercluster(self):
        self.create_AMR_regions(hyperclustering=True)

    def sized_regions(self):
        self.create_sized_regions(hyperclustering=False)

    def sized_regions_hypercluster(self):
        self.create_sized_regions(hyperclustering=True)

# HIGH-LEVEL Region Clustering functions
# ============================================================================================================

# ============================================================================================================
# HIGH-LEVEL Task Allocation functions

    def no_handoff(self, r):
        self.allocate_Drones2Region(r)
        self.assign_tasks(r)
    
    def center_handoff(self, r):
        self.allocate_AMR2Region(r, pick_point="center")
        self.allocate_Drones2Region(r)
        self.assign_tasks(r)

    def closest2AMR_handoff(self, r):
        self.allocate_AMR2Region(r, pick_point="closest2AMR")
        self.allocate_Drones2Region(r)
        self.assign_tasks(r)

    def closest2drop_handoff(self, r):
        self.allocate_AMR2Region(r, pick_point="closest2drop")
        self.allocate_Drones2Region(r)
        self.assign_tasks(r)

# HIGH-LEVEL Task Allocation functions
# ============================================================================================================

# ============================================================================================================
# Region Clustering functions

    def create_homogeneous_region(self):
        """
        Create one region containing all tasks
        Each region is initialized with an id, 
        """

        # Find mean of points x and y to determine cluster center
        u = np.mean([[task.pick_point.x, task.pick_point.y] for task in self.task_list.tasks], axis=0)

        # Define cluster center z at the center of the second cell to the bottom (to avoid collisions with potential ground units)
        center = Point(u[0], u[1], 1.5 * (1/self.resolution))

        # Ignore all AMRs for this region's fleet
        fleet = Fleet({"Drone":self.fleet.robots["Drone"]})

        # Create new region
        r = Region(
                id=f"R0",
                center=center,
                task_list=self.task_list,
                fleet=fleet
            )

        # Add to region list
        self.regions.append(r)
    
    def create_AMR_regions(self, hyperclustering=False):
        """
        Create regions equal to the number ground agents
        Each region is initialized with an id, 
        """
        # group using xy coordinates -> do not care about z height for grouping
        if hyperclustering:
            flattened_pick_points = [[task.pick_point.x, task.pick_point.y, task.drop_points[0].x, task.drop_points[0].y] for task in self.task_list.tasks]
        else:
            flattened_pick_points = [[task.pick_point.x, task.pick_point.y] for task in self.task_list.tasks]
        
        n_ground_agents = len(self.fleet.get_robots_as_list(robot_type="AMR"))

        KM_tasks = KMeans(
                    n_clusters=n_ground_agents,
                    random_state=0,
                    n_init='auto'
                ).fit(flattened_pick_points)

        for i in range(n_ground_agents):
            # Cast the cluster centers as 3D Points with height 5
            center = Point(int(KM_tasks.cluster_centers_[i][0]), int(KM_tasks.cluster_centers_[i][1]), 5)

            # Split the total task list into regional task lists using kmeans labels
            region_tasks =  [task for k, task in enumerate(self.task_list.tasks) if (KM_tasks.labels_[k] == i)]

            # Create new region
            r = Region(
                    id=f"R{i}",
                    center=center,
                    task_list=TaskList(region_tasks),
                    fleet=Fleet({"AMR":{}, "Drone":{}})
                )
            
            # Add to region list
            self.regions.append(r)

    def create_sized_regions(self, hyperclustering=False):
        """
        Create regions equal to the number ground agents
        Each region is initialized with an id, 
        """
        # group using xy coordinates -> do not care about z height for grouping
        if hyperclustering:
            flattened_pick_points = [[task.pick_point.x, task.pick_point.y, task.drop_points[0].x, task.drop_points[0].y] for task in self.task_list.tasks]
        else:
            flattened_pick_points = [[task.pick_point.x, task.pick_point.y] for task in self.task_list.tasks]

        carrying_capacity = 10

        n_regions = math.ceil(len(flattened_pick_points) / carrying_capacity)

        print("n_regions: ", n_regions)
        
        KM_tasks = KMeansConstrained(
                    n_clusters=n_regions,
                    size_min=2,
                    size_max = 10,
                    random_state=0
                ).fit(flattened_pick_points)

        for i in range(n_regions):
            # Cast the cluster centers as 3D Points with height 5
            center = Point(int(KM_tasks.cluster_centers_[i][0]), int(KM_tasks.cluster_centers_[i][1]), 5)

            # Split the total task list into regional task lists using kmeans labels
            region_tasks =  [task for k, task in enumerate(self.task_list.tasks) if (KM_tasks.labels_[k] == i)]

            # Create new region
            r = Region(
                    id=f"R{i}",
                    center=center,
                    task_list=TaskList(region_tasks),
                    fleet=Fleet({"AMR":{}, "Drone":{}})
                )
            
            # Add to region list
            self.regions.append(r)

# Region Clustering functions
# ============================================================================================================

# ============================================================================================================
# Task Allocation functions

    def allocate_AMR2Region(self, r, pick_point="center"):
        """
        Assign AMR's to Regions by checking if paths to their destinations overlap
        Switch any overlapping paths to get an approximated optimal allocation
        """
        # AMR allocated to this region
        closest_AMR = self.fleet.closest_robots_at_end_path(r.center, robot_type="AMR")[0]

        # Assign pick points for each region after AMR has been assigned
        if pick_point == "center":
            r.pick_point = Point(r.center.x, r.center.y, 1.5 * 1/self.resolution)

        elif pick_point == "closest2AMR":
            closest_task_to_amr = min(r.task_list.tasks, key=lambda x: manhattan_dist(x.pick_point, closest_AMR.lookup_last_assigned_pos()))
            
            r.pick_point = Point(closest_task_to_amr.pick_point.x, closest_task_to_amr.pick_point.y, 1.5 * 1/self.resolution)
        
        elif pick_point == "closest2drop":
            closest_task_to_drop = min(r.task_list.tasks, key=lambda x: manhattan_dist(x.pick_point, x.drop_point()))

            r.pick_point = Point(closest_task_to_drop.pick_point.x, closest_task_to_drop.pick_point.y, 1.5 * 1/self.resolution)

        # Move drone to this region's fleet
        r.fleet.add(closest_AMR)
        
    def allocate_Drones2Region(self, r):
        """
        Assign Drones to regions by balancing the drone_per_task metric of each region
        DPT = Drones per task
        """
        # Drones Per Task of each region if solution is optimal
        DPT_avg = len(self.fleet.robots["Drone"]) / len(self.task_list.tasks)

        # value for how many extra drones this region has based on the average estimate
        n_drone_target = round(DPT_avg * len(r.task_list.tasks))

        # closest drones in the fleet to centroid of this region
        closest_drones = self.fleet.closest_robots_at_end_path(r.center, robot_type="Drone")

        # Move drone to this region's fleet
        [r.fleet.add(drone) for drone in closest_drones[0:n_drone_target]]
        
    def assign_tasks(self, r):
        """
        2. Assign each drone to 1 random task while unclaimed tasks remain
        3. Swap any overlapping lines
        4. Assign tasks to agents
        5. Repeat
        6. If AMR exists in this region, assign one lumped task for all dropoffs
        """
        dist_prior = 0
        dist_post = 0

        # Drones for this region
        agent_fleet = r.fleet.get_robots_as_list(robot_type="Drone")
        
        # tasks for this region
        unclaimed_tasks = r.task_list.tasks

        # A sorted list of TaskLists corresponding to the sorted droned fleet
        while unclaimed_tasks:
            sweep_task_list = []

            # Semi-arbitrarily assign next sweep of tasks
            for i in range(len(agent_fleet)):
                if unclaimed_tasks:
                    task = unclaimed_tasks.pop(0)
                    sweep_task_list.append(task)

            # Logging variable keeps track of drone travel distance prior to optimization
            dist_prior += sum([manhattan_dist(agent_fleet[i].lookup_last_assigned_pos(), task.pick_point) for i,task in enumerate(sweep_task_list)])

            # Optimize sweep
            for i in range(len(sweep_task_list)):
                k = 0
                while k < len(sweep_task_list):
                    path1 = [agent_fleet[i].lookup_last_assigned_pos(), sweep_task_list[i].pick_point]
                    path2 = [agent_fleet[k].lookup_last_assigned_pos(), sweep_task_list[k].pick_point]

                    # If paths intersect, swap assigned endpoints
                    if intersect(path1[0], path1[1], path2[0], path2[1]):
                        t1 = sweep_task_list[i]
                        t2 = sweep_task_list[k]

                        sweep_task_list[i] = t2
                        sweep_task_list[k] = t1
                    
                        # Restart nested loop for every intersection to check back over previous paths
                        k = 0
                    else:
                        k += 1

            # Logging variable keeps track of drone travel distance post optimization
            dist_post += sum([manhattan_dist(agent_fleet[i].lookup_last_assigned_pos(), task.pick_point) for i,task in enumerate(sweep_task_list)])

            # Assign tasks to drones and AMRs if AMRs exist
            for i, task in enumerate(sweep_task_list):
                # Split up task into two parts at handoff point. Will return full task if no AMR in this region
                drone_task = r.split_task_to_AMR(task)

                # Add task to drone
                agent_fleet[i].add_task(drone_task)

        # If AMR in this region, merge tasks and assign
        r.merge_amr_tasks()

        print(f"{r.id} task assignment distance optimized: {dist_prior} -> {dist_post} (ft)")        

# Task Allocation functions
# ============================================================================================================

# ============================================================================================================
# Depricated

    # def allocate_AMR2Region(self, r, pick_point="center"):
    #     """
    #     Assign AMR's to Regions by checking if paths to their destinations overlap
    #     Switch any overlapping paths to get an approximated optimal allocation
    #     """

    #     # Logging variable keeps track of drone travel distance prior to optimization
    #     dist_prior = sum([manhattan_dist(r.center, list(r.fleet.robots['AMR'].values())[0].pos) for r in self.regions])

    #     for i in range(len(self.regions)):
    #         k = 0            
    #         while k < len(self.regions):
    #             # Must be converted to list to index the first value
    #             amr1 = list(self.regions[i].fleet.robots["AMR"].values())[0]
    #             amr2 = list(self.regions[k].fleet.robots["AMR"].values())[0]

    #             # AMR -> region centers: start and end points
    #             path1 = [amr1.pos, self.regions[i].center]
    #             path2 = [amr2.pos, self.regions[k].center]

    #             # If region-to-agent paths intersect, swap assigned agents
    #             if intersect(path1[0], path1[1], path2[0], path2[1]):
    #                 self.regions[i].fleet.robots["AMR"] = {amr2.robot_id:amr2}
    #                 self.regions[k].fleet.robots["AMR"] = {amr1.robot_id:amr1}
                    
    #                 # Restart nested loop for every intersection to check back over previous paths
    #                 k = 0
    #             else:
    #                 k += 1

    #     # Assign pick points for each region after all AMR's have been assigned
    #     if pick_point == "center":
    #         r.pick_point = r.center

    #     elif pick_point == "closest2AMR":
    #         # AMR for this region
    #         amr = r.fleet.get_robots_as_list(robot_type="AMR")[0]

    #         # Define pick point for this regiclosest_drone = self.fleet.closest_robots_at_end_path(r.center, robot_type="Drone")

    #         # Move drone to this region's fleet
    #         r.fleet.add(closest_drone)on as the floor under the pick point closest to the AMR's initial position
    #         closest_task_to_amr = min(r.task_list.tasks, key=lambda x: manhattan_dist(x.pick_point, amr.lookup_last_assigned_pos()))
    #         r.pick_point = Point(closest_task_to_amr.pick_point.x, closest_task_to_amr.pick_point.y, 5)
        
    #     elif pick_point == "closest2drop":
    #         pass
                

    #     # Logging variable keeps track of drone travel distance post optimization
    #     dist_post = sum([manhattan_dist(r.center, list(r.fleet.robots['AMR'].values())[0].pos) for r in self.regions])

    #     print(f"AMR assignment distance optimized: {dist_prior} -> {dist_post} (ft)")

    # def create_Regions(self, hyperclustering=False):
    #     """
    #     Create regions equal to the number ground agents
    #     Each region is initialized with an id, 
    #     """
    #     # group using xy coordinates -> do not care about z height for grouping
    #     if hyperclustering:
    #         flattened_pick_points = [[task.pick_point.x, task.pick_point.y, task.drop_points[0].x, task.drop_points[0].y] for task in self.task_list.tasks]
    #     else:
    #         flattened_pick_points = [[task.pick_point.x, task.pick_point.y] for task in self.task_list.tasks]
        
    #     KM_tasks = KMeans(
    #                 n_clusters=len(self.ground_agents),
    #                 random_state=0,
    #                 n_init='auto'
    #             ).fit(flattened_pick_points)

    #     # Kmeans for maximum cluster sized

    #     # carrying_capacity = 10

    #     # n_regions = math.ceil(len(flattened_pick_points) / carrying_capacity)
        
    #     # KM_tasks = KMeansConstrained(
    #     #             n_clusters=n_regions,
    #     #             size_min=2,
    #     #             size_max = 10,
    #     #             random_state=0
    #     #         )
    #     # print("POINT LENGTH = ", len(flattened_pick_points))
    #     # print("KM CHECK = ", n_regions)
    #     # KM_tasks.fit_predict(flattened_pick_points)

    #     for i, agent in enumerate(self.ground_agents):
    #         # Cast the cluster centers as 3D Points with height 5
    #         center = Point(int(KM_tasks.cluster_centers_[i][0]), int(KM_tasks.cluster_centers_[i][1]), 5)

    #         # Split the total task list into regional task lists using kmeans labels
    #         region_tasks =  [task for k, task in enumerate(self.task_list.tasks) if (KM_tasks.labels_[k] == i)]

    #         # Initialize each regional fleet with a ground agent, agents are re-allocated in assign_AMRs2Regions()
    #         region_robots = {"AMR":{agent.robot_id:agent}, "Drone":{}}

    #         # Create new region
    #         r = Region(
    #                 id=f"R{i}",
    #                 center=center,
    #                 task_list=TaskList(region_tasks),
    #                 fleet=Fleet(region_robots)
    #             )
            
    #         # Add to region list
    #         self.regions.append(r)



# def assign_Drones2Regions(self):
    #     """
    #     Assign Drones to regions by balancing the drone_per_task metric of each region
    #     DPT = Drones per task
    #     """
    #     # Idle drones -> not currently assigned to a region
    #     d_idle = self.drone_agents

    #     # Assign every drone to a region
    #     while d_idle:
    #         for r in self.regions:
    #             if d_idle:
    #                 # Closest drone by manhattan distance to region centroid
    #                 closest_drone_index = min(range(len(d_idle)), key=lambda x: manhattan_dist(r.center, d_idle[x].pos))

    #                 # Add closest drone to fleet and remove from the drone list
    #                 r.fleet.add(d_idle.pop(closest_drone_index))

    #     # # Drones Per Task of each region if solution is optimal
    #     DPT_avg = len(self.fleet.robots["Drone"]) / len(self.task_list.tasks)

    #     # If a group is oversized, shift one robot to the nearest undersized group
    #     DPT_std = np.inf
    #     last_std = None
    #     while last_std is None or DPT_std < last_std:
    #         last_std = DPT_std

    #         # Drones Per Task standard deviation used to approximate solution achievement
    #         DPT_std = sum([(len(t.fleet.robots['Drone']) / len(t.task_list.tasks) - DPT_avg)**2 for t in self.regions])/len(self.regions)

    #         for r in self.regions:                
    #             # float value for how many extra drones this region has based on the average estimate
    #             extra_drones = len(r.fleet.robots['Drone']) -  DPT_avg * len(r.task_list.tasks)

    #             # If more than 1/2 of a drone is unused, donate closest drone. CANNOT DONATE LAST DRONE
    #             if extra_drones > 0.5 and len(r.fleet.robots['Drone']) > 1:
    #                 # Recalculate all regions that still need more drones
    #                 undersized_regions = [region for region in self.regions if len(region.fleet.robots["Drone"]) / len(region.task_list.tasks) < DPT_avg]

    #                 # closest undersized region by centroid distance
    #                 r2 = min(undersized_regions, key=lambda x: manhattan_dist(r.center, x.center))

    #                 # closest drone in this fleet to centroid of next region
    #                 closest_drone = min(r.fleet.robots["Drone"].values(), key=lambda x: manhattan_dist(x.pos, r2.center))

    #                 # Move drone from one region's fleet to the other
    #                 r.fleet.remove(closest_drone)
    #                 r2.fleet.add(closest_drone)  



    # def assign_Tasks2Agents(self):
    #     """
    #     1. Sort the drones in order of availability (combination of distance to region and task distance)
    #     2. Assign each drone to 1 random task while unclaimed tasks remain
    #     3. Swap any overlapping lines
    #     4. Split up task to drone and AMR
    #     5. Repeat
    #     """
    #     dist_prior = 0
    #     dist_post = 0

    #     for r in self.regions:
    #         r.amr_tasks = []

    #         # Sort drones by distance to center of region
    #         sorted_drone_fleet = sorted(r.fleet.get_robots_as_list(robot_type="Drone"), key=lambda x: manhattan_dist(x.pos, r.center))
    #         # sorted_drone_fleet = r.fleet.get_robots_as_list(robot_type="Drone")
            
    #         # Sort tasks by inverse distance to the center of region
    #         unclaimed_tasks = sorted(r.task_list.tasks, key=lambda x: manhattan_dist(x.pick_point, r.center), reverse=True)
    #         # unclaimed_tasks = r.task_list.tasks

    #         # A sorted list of TaskLists corresponding to the sorted droned fleet
    #         while unclaimed_tasks:
    #             sweep_task_list = []

    #             # Semi-arbitrarily assign next sweep of tasks
    #             for i in range(len(sorted_drone_fleet)):
    #                 if unclaimed_tasks:
    #                     task = unclaimed_tasks.pop(0)
    #                     sweep_task_list.append(task)

    #             # Logging variable keeps track of drone travel distance prior to optimization
    #             dist_prior += sum([manhattan_dist(sorted_drone_fleet[i].pos, task.pick_point) for i,task in enumerate(sweep_task_list)])

    #             # Optimize sweep
    #             for i in range(len(sweep_task_list)):
    #                 k = 0
    #                 while k < len(sweep_task_list):
    #                     path1 = [sorted_drone_fleet[i].lookup_last_assigned_pos(), sweep_task_list[i].pick_point]
    #                     path2 = [sorted_drone_fleet[k].lookup_last_assigned_pos(), sweep_task_list[k].pick_point]
                            
    #                     # If paths intersect, swap assigned endpoints
    #                     if intersect(path1[0], path1[1], path2[0], path2[1]):
    #                         t1 = sweep_task_list[i]
    #                         t2 = sweep_task_list[k]

    #                         sweep_task_list[i] = t2
    #                         sweep_task_list[k] = t1
                        
    #                         # Restart nested loop for every intersection to check back over previous paths
    #                         k = 0
    #                     else:
    #                         k += 1

    #             # Logging variable keeps track of drone travel distance post optimization
    #             dist_post += sum([manhattan_dist(sorted_drone_fleet[i].pos, task.pick_point) for i,task in enumerate(sweep_task_list)])

    #             # Assign tasks to drones and AMR's
    #             for i, task in enumerate(sweep_task_list):
    #                 # drone for this task id
    #                 drone = sorted_drone_fleet[i]

    #                 # Split up task into two parts at handoff point
    #                 drone_task = r.crop_drone_task(task)

    #                 # Add task to drone
    #                 drone.add_task(drone_task)

    #                 # Add task to this region's AMR
    #                 # amr.add_task(amr_task)

    #         r.merge_amr_tasks()

    #     print(f"DRONE assignment distance optimized: {dist_prior} -> {dist_post} (ft)")   