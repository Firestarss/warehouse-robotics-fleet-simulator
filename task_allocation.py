from utils import *
from warehouse_map import *
from robot_fleet import *
from sklearn.cluster import KMeans

"""
1. Group tasks such that each local group gets one AMR. Every AMR must be used.
2. Assign AMR to go to kmeans centroid
3. Assign a drone to each group and balance allocation based on task list length
4. Assign tasks to agents splitting each into nearest drone and AMR
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

        if self.check_for_input_error():
            return

    def __repr__(self):
        return (f"TaskAllocator(task_list={self.task_list}\n" + 
                f"    fleet={self.fleet},\n" +
                f"    pick_points={self.pick_points})")
    
    def populate_fleet(self, allocation_type="regional"):
        if allocation_type == "regional":
            self.create_Regions()

            self.assign_AMRs2Regions()

            self.assign_Drones2Regions()

            self.assign_Tasks2Agents()

        elif allocation_type == "drones_only":
            self.assign_Tasks2Fleet_homogeneous(robot_type="Drone")

        elif allocation_type == "amrs_only":
            self.assign_Tasks2Fleet_homogeneous(robot_type="AMR")

        elif allocation_type == "treat_as_homogeneous":
            self.assign_Tasks2Fleet_homogeneous(robot_type="All")

        else:
            print(f"UNRECOGNIZED TASK ALLOCATION KEY: '{allocation_type}'" +
                  f"\nTry:" +
                  f"\n    'regional'" +
                  f"\n    'drones_only'" +
                  f"\n    'amrs_only'" +
                  f"\n    'treat_as_homogeneous'")


        return self.fleet
    
    def check_for_input_error(self):
        n_tasks = len(self.task_list.tasks)
        n_ground = len(self.ground_agents)
        n_drone = len(self.drone_agents)
        
        if n_tasks < n_ground:
            print(f"TASK ALLOCATION WARNING: # of ground agents '{n_ground}' cannot exceed # of tasks '{n_tasks}'")
            # return True
        
        if n_tasks < n_drone:
            print(f"TASK ALLOCATION WARNING: # of drones '{n_drone}' is larger than # of tasks '{n_tasks}', drones will be unused")        

        return False
    
    def split_task_by_point(self, task, point):        
        drone_task = Task(task.task_id, task.pick_point, point)
        
        amr_task = Task(task.task_id, point, task.drop_point)

        return drone_task, amr_task
    
    def create_Regions(self):
        """
        Create regions equal to the number ground agents
        Each region is initialized with an id, 
        """
        # group using xy coordinates -> do not care about z height for grouping
        flattened_pick_points = [[task.pick_point.x, task.pick_point.y] for task in self.task_list.tasks]
        
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

        # Logging variable keeps track of drone travel distance prior to optimization
        dist_prior = sum([manhattan_dist(r.center, list(r.fleet.robots['AMR'].values())[0].pos) for r in self.regions])

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

        # Logging variable keeps track of drone travel distance post optimization
        dist_post = sum([manhattan_dist(r.center, list(r.fleet.robots['AMR'].values())[0].pos) for r in self.regions])

        print(f"AMR assignment distance optimized: {dist_prior} -> {dist_post} (ft)")
        
    def assign_Drones2Regions(self):
        """
        Assign Drones to regions by balancing the drone_per_task metric of each region
        DPT = Drones per task
        """
        # Idle drones -> not currently assigned to a region
        d_idle = self.drone_agents

        # Assign every drone to a region
        while d_idle:
            for r in self.regions:
                if d_idle:
                    # Closest drone by manhattan distance to region centroid
                    closest_drone_index = min(range(len(d_idle)), key=lambda x: manhattan_dist(r.center, d_idle[x].pos))

                    # Add closest drone to fleet and remove from the drone list
                    r.fleet.add(d_idle.pop(closest_drone_index))

        # # Drones Per Task of each region if solution is optimal
        DPT_avg = len(self.fleet.robots["Drone"]) / len(self.task_list.tasks)

        # If a group is oversized, shift one robot to the nearest undersized group
        DPT_std = np.inf
        last_std = None
        while last_std is None or DPT_std < last_std:
            last_std = DPT_std

            # Drones Per Task standard deviation used to approximate solution achievement
            DPT_std = sum([(len(t.fleet.robots['Drone']) / len(t.task_list.tasks) - DPT_avg)**2 for t in self.regions])/len(self.regions)

            for r in self.regions:                
                # float value for how many extra drones this region has based on the average estimate
                extra_drones = len(r.fleet.robots['Drone']) -  DPT_avg * len(r.task_list.tasks)

                # If more than 1/2 of a drone is unused, donate closest drone. CANNOT DONATE LAST DRONE
                if extra_drones > 0.5 and len(r.fleet.robots['Drone']) > 1:
                    # Recalculate all regions that still need more drones
                    undersized_regions = [region for region in self.regions if len(region.fleet.robots["Drone"]) / len(region.task_list.tasks) < DPT_avg]

                    # closest undersized region by centroid distance
                    r2 = min(undersized_regions, key=lambda x: manhattan_dist(r.center, x.center))

                    # closest drone in this fleet to centroid of next region
                    closest_drone = min(r.fleet.robots["Drone"].values(), key=lambda x: manhattan_dist(x.pos, r2.center))

                    # Move drone from one region's fleet to the other
                    r.fleet.remove(closest_drone)
                    r2.fleet.add(closest_drone)

    def assign_Tasks2Agents(self):
        """
        1. Sort the drones in order of availability (combination of distance to region and task distance)
        2. Assign each drone to 1 random task while unclaimed tasks remain
        3. Swap any overlapping lines
        4. Split up task to drone and AMR
        5. Repeat
        """
        dist_prior = 0
        dist_post = 0

        for r in self.regions:
            # Sort drones by distance to center of region
            sorted_drone_fleet = sorted(r.fleet.get_robots_as_list(robot_type="Drone"), key=lambda x: manhattan_dist(x.pos, r.center))
            # sorted_drone_fleet = r.fleet.get_robots_as_list(robot_type="Drone")
            
            # Sort tasks by inverse distance to the center of region
            unclaimed_tasks = sorted(r.task_list.tasks, key=lambda x: manhattan_dist(x.pick_point, r.center), reverse=True)
            # unclaimed_tasks = r.task_list.tasks

            # Sweep depth keeps track of how many times we sweep through the fleet to assign tasks
            sweep_depth = 0

            # A sorted list of TaskLists corresponding to the sorted droned fleet
            while unclaimed_tasks:
                sweep_task_list = []

                # Semi-arbitrarily assign next sweep of tasks
                for i in range(len(sorted_drone_fleet)):
                    if unclaimed_tasks:
                        task = unclaimed_tasks.pop(0)
                        sweep_task_list.append(task)

                # Logging variable keeps track of drone travel distance prior to optimization
                dist_prior += sum([manhattan_dist(sorted_drone_fleet[i].pos, task.pick_point) for i,task in enumerate(sweep_task_list)])

                # Optimize sweep
                for i in range(len(sweep_task_list)):
                    k = 0
                    while k < len(sweep_task_list):
                        # if this is the drone's first task, start from its current position
                        if sweep_depth == 0:
                            path1 = [sorted_drone_fleet[i].pos, sweep_task_list[i].pick_point]
                            path2 = [sorted_drone_fleet[k].pos, sweep_task_list[k].pick_point]

                        # If previous tasks, assume that the drone is navigating from the center of the region (last drop off)
                        else:
                            path1 = [r.center, sweep_task_list[i].pick_point]
                            path2 = [r.center, sweep_task_list[k].pick_point]
                            
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
                dist_post += sum([manhattan_dist(sorted_drone_fleet[i].pos, task.pick_point) for i,task in enumerate(sweep_task_list)])

                # Assign tasks to drones and AMR's
                for i, task in enumerate(sweep_task_list):
                    # Split up task into two parts at handoff point (region center)
                    drone_task, amr_task = self.split_task_by_point(task, r.center)

                    # Add task to drone
                    sorted_drone_fleet[i].add_task(drone_task)

                    # Add task to this region's AMR
                    r.fleet.get_robots_as_list(robot_type="AMR")[0].add_task(amr_task)

                sweep_depth += 1

        print(f"DRONE assignment distance optimized: {dist_prior} -> {dist_post} (ft)")


    def assign_Tasks2Fleet_homogeneous(self, robot_type="All"):
        """
        2. Assign each drone to 1 random task while unclaimed tasks remain
        3. Swap any overlapping lines
        4. Assign tasks to agents
        5. Repeat
        """
        dist_prior = 0
        dist_post = 0

        # Sort drones by distance to center of region
        agent_fleet = self.fleet.get_robots_as_list(robot_type=robot_type)
        
        # Sort tasks by inverse distance to the center of region
        unclaimed_tasks = self.task_list.tasks

        # Sweep depth keeps track of how many times we sweep through the fleet to assign tasks
        sweep_depth = 0

        # A sorted list of TaskLists corresponding to the sorted droned fleet
        while unclaimed_tasks:
            sweep_task_list = []

            # Semi-arbitrarily assign next sweep of tasks
            for i in range(len(agent_fleet)):
                if unclaimed_tasks:
                    task = unclaimed_tasks.pop(0)
                    sweep_task_list.append(task)

            # Logging variable keeps track of drone travel distance prior to optimization
            dist_prior += sum([manhattan_dist(agent_fleet[i].pos, task.pick_point) for i,task in enumerate(sweep_task_list)])

            # Optimize sweep
            for i in range(len(sweep_task_list)):
                k = 0
                while k < len(sweep_task_list):
                    # if this is the drone's first task, start from its current position
                    if sweep_depth == 0:
                        path1 = [agent_fleet[i].pos, sweep_task_list[i].pick_point]
                        path2 = [agent_fleet[k].pos, sweep_task_list[k].pick_point]

                    # If previous tasks, assume that the drone is navigating from the last drop off
                    else:
                        path1 = [agent_fleet[i].task_list.tasks[sweep_depth-1].drop_point, sweep_task_list[i].pick_point]
                        path2 = [agent_fleet[i].task_list.tasks[sweep_depth-1].drop_point, sweep_task_list[k].pick_point]
                        
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
            dist_post += sum([manhattan_dist(agent_fleet[i].pos, task.pick_point) for i,task in enumerate(sweep_task_list)])

            # Assign tasks to fleet
            [agent_fleet[i].add_task(task) for i, task in enumerate(sweep_task_list)]

            sweep_depth += 1

        print(f"Homogeneous fleet assignment distance optimized: {dist_prior} -> {dist_post} (ft)")