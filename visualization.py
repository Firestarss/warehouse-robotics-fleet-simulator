from utils import *
from warehouse_map import *
from robot_fleet import *
import plotly.graph_objects as go

class Visualizer:
    colors = ['rgba( 31,  119, 180, 1.0 )', 
              'rgba(255, 127,   14, 1.0 )',
              'rgba( 44, 160,   44, 1.0 )', 
              'rgba(214,  39,   40, 1.0 )',
              'rgba(148, 103,  189, 1.0 )', 
              'rgba(140,  86,   75, 1.0 )',
              'rgba(227, 119,  194, 1.0 )', 
              'rgba(127, 127,  127, 1.0 )',
              'rgba(188, 189,   34, 1.0 )', 
              'rgba( 23, 190,  207, 1.0 )']
    def __init__(self, wh_map, fleet, task_list):
        self.wh_map = wh_map
        self.fleet = fleet
        self.task_list = task_list
        
        # type_options = {
        #     "simple_trace": self.simple_trace
        # }
        # self.vis_func = type_options[vis_type]
        
        self.fig = go.Figure()
        
        self.plot_pick_drop_points()
        self.fig.show()
   
    # def __repr__(self):
    #     pass
    
    def color(self, idx):
        return self.colors[idx % len(self.colors)]
    
    def plot_blocked_areas(self):
        pass
    
    def simple_trace(self):
        pass
    
    def plot_pick_drop_points(self, color=None):
        """creates 3D plot showing agent start locations and task locations"""
        tasks = self.task_list.tasks
        if color == None:
            for i in range(len(tasks)):
                marker_size = np.linspace(10,5, len(tasks))
                pick_str = tasks[i].pick_point.as_str()
                drop_str = tasks[i].drop_point.as_str()
                self.fig.add_scatter3d(
                    x=[tasks[i].pick_point.x,tasks[i].drop_point.x], 
                    y=[tasks[i].pick_point.y,tasks[i].drop_point.y], 
                    z=[tasks[i].pick_point.z,tasks[i].drop_point.z], 
                    mode='lines',
                    marker=dict(color=self.color(i)), 
                    name=f"Task {tasks[i].task_id}")
                self.fig.add_scatter3d(
                    x=[tasks[i].pick_point.x], 
                    y=[tasks[i].pick_point.y], 
                    z=[tasks[i].pick_point.z], 
                    mode='markers+text',
                    marker=dict(color=self.color(i), size=marker_size[i]),
                    name=f"Pick {tasks[i].task_id}: {pick_str}",
                    text=[tasks[i].task_id])
                self.fig.add_scatter3d(
                    x=[tasks[i].drop_point.x], 
                    y=[tasks[i].drop_point.y], 
                    z=[tasks[i].drop_point.z], 
                    mode='markers',
                    marker=dict(color=self.color(i), size=marker_size[i],
                                symbol='square'), 
                    name=f"Drop {tasks[i].task_id}: {drop_str}")
        else:
            marker_size = 10
            for i in range(len(tasks)):
                pick_str = tasks[i].pick_point.as_str()
                self.fig.add_scatter3d(
                    x=[tasks[i].pick_point.x,tasks[i].drop_point.x], 
                    y=[tasks[i].pick_point.y,tasks[i].drop_point.y], 
                    z=[tasks[i].pick_point.z,tasks[i].drop_point.z], 
                    mode='lines+markers',
                    marker=dict(color=color, size=marker_size, symbol='square'),
                    name=f"Task {tasks[i].task_id}")
                self.fig.add_scatter3d(
                    x=[tasks[i].pick_point.x], 
                    y=[tasks[i].pick_point.y], 
                    z=[tasks[i].pick_point.z], 
                    mode='markers+text',
                    marker=dict(color=color, size=marker_size, symbol='square'),
                    name=f"Pick {tasks[i].task_id}: {pick_str}",
                    text=[tasks[i].task_id])

        self.fig.update_layout(
            scene = dict(
                xaxis = dict(nticks=self.wh_map.wh_zone.x_range()//5, range=[self.wh_map.wh_zone.x_lims[1], self.wh_map.wh_zone.x_lims[0]]),
                yaxis = dict(nticks=self.wh_map.wh_zone.y_range()//5, range=self.wh_map.wh_zone.y_lims,),
                zaxis = dict(nticks=self.wh_map.wh_zone.z_range()//5, range=self.wh_map.wh_zone.z_lims,),
                aspectmode="data"),
            width=1200,
            margin=dict(r=20, l=20, b=20, t=20))