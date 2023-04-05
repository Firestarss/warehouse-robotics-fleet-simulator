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

class TaskAllocator:
    def __init__(self, task_list, fleet):
        self.task_list = task_list
        self.fleet = fleet

        self.group_tasks()

    def __repr__(self):
        return (f"WarehouseMap(task_list={self.task_list}\n" + 
                f"    fleet={self.fleet},\n" +
                f"    pick_points={self.pick_points})")
    
    def group_tasks(self):
        flattened_pick_points = np.asarray([[task.pick_point.x, task.pick_point.y] for task in self.task_list.tasks])

        n_ground_agents = len(self.fleet.robots["AMR"])
        # kmeans = KMeans(n_clusters=n_ground_agents, random_state=0).fit(w_vector_array[:,0:2], sample_weight = w_vector_array[:,2])
        kmeans = KMeans(n_clusters=n_ground_agents, random_state=0, n_init='auto').fit(flattened_pick_points)
        centers = np.array(kmeans.cluster_centers_)
        labels = kmeans.labels_

        for i, group in enumerate(labels):
            # TODO: populate AMR based on closest distance to kmeans centroid
            taskGroup = Fleet(some AMR Here)

            # TODO: populate drones based on closest distance to kmeans centroid -> this is what the paper was for

            # TODO: Assign Tasks to AMR

            # TODO: Assign Tasks to Drone

        print(centers)
        print(labels)
        print(centers[kmeans.labels_])