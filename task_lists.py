from utils import *
from warehouse_map import *
from robot_fleet import *
from path_planning import *
from task_allocation import *
from evaluation import *
from visualization import *

task_lists = [
TaskList([ Task('T0', Point(95,45.0,15.0), [Point(65,5,5)]),
           Task('T1', Point(185,45.0,35.0), [Point(205,5,5)]),
           Task('T2', Point(125,85.0,25.0), [Point(25,5,5)]),
           Task('T3', Point(125,155.0,15.0), [Point(45,5,5)]),
           Task('T4', Point(5,55.0,35.0), [Point(105,5,5)]),
           Task('T5', Point(95,155.0,15.0), [Point(105,5,5)]),
           Task('T6', Point(35,25.0,15.0), [Point(85,5,5)]),
           Task('T7', Point(215,65.0,35.0), [Point(85,5,5)]),
           Task('T8', Point(35,55.0,35.0), [Point(165,5,5)]),
           Task('T9', Point(155,205.0,35.0), [Point(65,5,5)]) ])
]