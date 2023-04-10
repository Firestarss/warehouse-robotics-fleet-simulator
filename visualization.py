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
    def __init__(self, wh_map, task_list, fleet, vis_type="default_static"):
        self.wh_map = wh_map
        self.fleet = fleet
        self.task_list = task_list

        type_options = {
            "default_static": self.color_tasks_traces_off,
            "color_tasks_traces_off": self.color_tasks_traces_off,
            "black_tasks_traces_on": self.black_tasks_traces_on
        }
        self.vis_func = type_options[vis_type]

        self.fig = go.Figure()
        self.set_fig_layout()

    # def __repr__(self):
    #     pass

    def show(self):
        self.vis_func()
        self.fig.show()

    def color_tasks_traces_off(self):
        self.plot_blocked_areas()
        self.plot_pick_drop_points()

    def black_tasks_traces_on(self):
        self.plot_blocked_areas()
        self.plot_pick_drop_points(color="rgba(10,10,10,0.4)")
        test_path = [Point(155,45,15), 
                     Point(155,55,15), 
                     Point(155,65,15),
                     Point(155,65,25),
                     Point(155,65,25),
                     Point(145,65,25)]
        self.trace_path(test_path, "Test Path")

    def set_fig_layout(self):
        self.fig.update_layout(
            scene = dict(
                xaxis = dict(nticks=self.wh_map.wh_zone.x_range()//5, range=[self.wh_map.wh_zone.x_lims[1], self.wh_map.wh_zone.x_lims[0]]),
                yaxis = dict(nticks=self.wh_map.wh_zone.y_range()//5, range=self.wh_map.wh_zone.y_lims,),
                zaxis = dict(nticks=self.wh_map.wh_zone.z_range()//5, range=self.wh_map.wh_zone.z_lims,),
                aspectmode="data"),
            width=1200,
            margin=dict(r=20, l=20, b=20, t=20))
    
    def color(self, idx):
        return self.colors[idx % len(self.colors)]
    
    def plot_blocked_areas(self):
        lightness = 150
        for blocked_area in self.wh_map.blocked_areas:
            x, y, z, i, j, k = blocked_area.mesh_params()
            self.fig.add_mesh3d(
                # 8 vertices of a cube
                x=x,
                y=y,
                z=z,
                i = i,
                j = j,
                k = k,
                opacity=0.4,
                color=f'rgb({lightness},{lightness},{lightness})',
                flatshading = True
            )

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

    def trace_path(self, point_path, path_name="", color='rgb(31,119,180)'):
        xs = [point.x for point in point_path]
        ys = [point.y for point in point_path]
        zs = [point.z for point in point_path]
        labels = [f"Seg. Idx. {i}" for i in list(range(len(point_path)))]
        self.fig.add_scatter3d(
            x=xs,
            y=ys,
            z=zs,
            mode='lines+markers',
            marker=dict(color=color, size=3),
            name=path_name,
            text=labels)
        self.fig.add_scatter3d(
            x=[xs[0]],
            y=[ys[0]],
            z=[zs[0]],
            mode='markers+text',
            marker=dict(color=color, size=5, symbol='square'),
            name=path_name,
            showlegend=False,
            text=path_name)