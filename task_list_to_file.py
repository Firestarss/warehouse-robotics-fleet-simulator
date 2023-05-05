from utils import *
from warehouse_map import *
from robot_fleet import *
from path_planning import *
from task_allocation import *
from evaluation import *
from visualization import *

evaluator = Evaluator()

# evaluator.set_static("warehouse_x",  4)
# evaluator.set_static("warehouse_y",  8)

# evaluator.set_variable("warehouse_x",  min=2, max=10, step=2)
# evaluator.set_variable("warehouse_y",  min=2, max=10, step=2)
# evaluator.set_variable("warehouse_z",  min=50, max=100, step=10)

# evaluator.set_static("n_drone", 20)
# evaluator.set_static("n_amr", 5)
# evaluator.set_variable("n_drone",  min=10, max=100, step=10)
# evaluator.set_variable("n_amr", min=1,  max=15,  step=5)

file1 = open('task_list_txt.txt', 'w')

# Writing a string to file
file1.write("task_lists = [\n\n")

for i, wh_info in enumerate(evaluator.generate_wh_info()):

    wh_map = WarehouseMap(wh_info, resolution=0.1, units="ft")
    wh_pick_points, wh1_drop_points = wh_map.generate_points()
    # wh1_map.show_occ_matrix(0)

    # Initialize task list for this warehouse
    rand_task_list = TaskList()
    rand_task_list.populate_randomly(wh_pick_points, wh1_drop_points, 100)
    print(rand_task_list)

    
    file1.write(rand_task_list.__repr__()+",\n\n")

    
file1.write("]")
 
# Closing file
file1.close()