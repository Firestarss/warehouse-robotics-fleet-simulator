from utils import *
from warehouse_map import *
from robot_fleet import *
import plotly.graph_objects as go

class Visualizer:
    """
    Uses plotly to make a visualization of the warehouse, tasks, and the fleet's
    behavior.
    Parameters:
        wh_map (WarehouseMap): The warehouse map to be displayed
        task_list (TaskList): The TaskList to plot. Can be None if fleet tasks
            are used.
        fleet (Fleet): The robot fleet to plot.
        vis_type (str, optional): Defaults to "static_with_robots"
            "static_no_robots" - plots tasks only
            "static_with_robots" - plots tasks and traces robot paths as a line
                (default)
            "animated_with_static_tasks" - animates robots but just shows tasks
                as original picks and drops
            "animated_with_dynamic_tasks" - animates robots and shows the live
                location of the tasks with the robots
        task_plot_mode (str, optional): Defaults to "no_lines".
            "detailed" - 3 traces per task, pick and drop sparately in color
            "simple" - 1 trace per task, grey tasks with line connecting
            "no_lines" - 1 trace total, grey tasks with no line connecting
            "off" - skips plotting tasks
        split_tasks (bool, optional): Whether to plot the fleet's tasks (Drone
            pick to drop at AMR, then AMR drop at goal location) or the given
            task list (original task pick to task drop). Defaults to False.
        show_task_labels (bool, optional): Whether to print the name of the task
            above the task pick/drop or only show that info on hover. Defaults
            to True.
        show_t (bool, optional): Whether to show the timestep number that the
            robot is at that position or show only on hover in the
            "static_with_robots" visualization mode. Defaults to False.
        """
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
    
    def __init__(self, wh_map, task_list, fleet, vis_type="static_with_robots",
                 task_plot_mode="no_lines", split_tasks=False, show_task_labels=True, show_t=False):
        """Make a new visualizer.
        Args:
            wh_map (WarehouseMap): The warehouse map to be displayed
            task_list (TaskList): The TaskList to plot. Can be None if fleet
                tasks are used.
            fleet (Fleet): The robot fleet to plot.
            vis_type (str, optional): Defaults to "static_with_robots"
                "static_no_robots" - plots tasks only
                "static_with_robots" - plots tasks and traces robot paths as a
                    line (default)
                "animated_with_static_tasks" - animates robots but just shows as
                    original picks and drops
                "animated_with_dynamic_tasks" - animates robots and shows the 
                    live location of the tasks with the robots
            task_plot_mode (str, optional): Defaults to "no_lines".
                "detailed" - 3 traces per task, pick and drop separately in
                    color
                "simple" - 1 trace per task, grey tasks with line connecting
                "no_lines" - 1 trace total, grey tasks with no line connecting
                "off" - skips plotting tasks
            split_tasks (bool, optional): Whether to plot the fleet's tasks
                (Drone pick to drop at AMR, then AMR drop at goal location) or
                the given task list (original task pick to task drop). Defaults
                to False.
            show_task_labels (bool, optional): Whether to print the name of the
                task above the task pick/drop or only show that info on hover. 
                Defaults to True.
            show_t (bool, optional): Whether to show the timestep number that
            the robot is at that position or show only on hover in the
                "static_with_robots" visualization mode. Defaults to False.
        """
        
        self.wh_map = wh_map
        self.fleet = fleet
        self.task_list = task_list
        
        self.vis_type = vis_type
        self.task_plot_mode = task_plot_mode
        self.split_tasks = split_tasks
        self.show_task_labels = show_task_labels
        self.show_t = show_t
        
        self.t_max = None
        
        self.vis_types = {
            "static_no_robots": self.static_no_robots,
            "static_with_robots": self.static_with_robots,
            "animated_with_static_tasks": self.animated_with_static_tasks,
            "animated_with_dynamic_tasks": self.animated_with_dynamic_tasks,
        }
        self.vis_func = self.vis_types[vis_type]
        
        if self.vis_type == "animated_with_pick_updates":
            self.task_plot_mode = "no_lines"
            print("self.task_plot_mode for vis_type" + "'animated_with_pick_updates' is forced to be 'no_lines'.")

        self.fig = go.Figure()
        self.set_fig_layout()

    def __repr__(self):
        return (f"Visualizer({self.wh_map}, {self.fleet}, {self.task_list}, "
                f"{self.vis_type}, {self.task_plot_mode}, {self.split_tasks}, "
                f"{self.show_task_labels}, {self.show_t})")
    
    def color(self, idx):
        return self.colors[idx % len(self.colors)]

    def show(self):
        if "animate" in self.vis_type:
            self.animation_setup()
            self.animation_fig_dict["data"].append(go.Scatter3d())
            self.animation_fig_dict["data"].extend(self.make_blocked_areas_meshes())
            self.vis_func()
            self.fig = go.Figure(self.animation_fig_dict)
        else:
            self.set_fig_layout()
            self.add_traces(self.make_blocked_areas_meshes())
            self.vis_func()
        self.fig.show()
        
    def static_no_robots(self):
        self.add_traces(self.make_task_scatters())
    
    def static_with_robots(self):
        self.add_traces(self.make_task_scatters())
        self.add_traces(self.make_robot_static_scatters())

    def animated_with_static_tasks(self):
        # make initial data
        self.animation_fig_dict["data"].extend(self.make_task_scatters())
        # make frames
        for t in range(self.t_max):
            frame = {"data": [], "name": str(t)}
            frame["data"].extend(self.make_robot_frame_scatters(t))
            self.animation_fig_dict["frames"].append(frame)

    def animated_with_dynamic_tasks(self):
        pass
    
    def make_layout_dict(self):
        tic_div = 20
        fig_dict = {}
        fig_dict["layout"] = {}
        fig_dict["layout"]["scene"] = dict(
                xaxis = dict(nticks=self.wh_map.wh_zone.x_range()//tic_div, range=[self.wh_map.wh_zone.x_lims[1], self.wh_map.wh_zone.x_lims[0]]),
                yaxis = dict(nticks=self.wh_map.wh_zone.y_range()//tic_div, range=self.wh_map.wh_zone.y_lims,),
                zaxis = dict(nticks=self.wh_map.wh_zone.z_range()//tic_div, range=self.wh_map.wh_zone.z_lims,),
                aspectmode="manual", 
                aspectratio=dict(x=1, y=self.wh_map.wh_zone.y_range()/self.wh_map.wh_zone.x_range(), z=self.wh_map.wh_zone.z_range()/self.wh_map.wh_zone.x_range()))
        fig_dict["layout"]["width"] = 1500
        fig_dict["layout"]["margin"] = dict(r=20, l=20, b=20, t=20)
        fig_dict["layout"]["xaxis"] = {"title":"x-label" , "visible":False, 'showticklabels':False}
        fig_dict["layout"]["yaxis"] = {"title":"y-label" , "visible":False, 'showticklabels':False}
        return fig_dict
            
    def set_fig_layout(self):
        self.fig.update_layout(self.make_layout_dict()["layout"])
        
    def add_traces(self, traces):
        for trace in traces:
            self.fig.add_trace(trace)

    def make_zone_mesh(self, zone, color="red", name="", hoverinfo=None):
        x, y, z, i, j, k = zone.mesh_params()
        if hoverinfo == None:
            return go.Mesh3d(x=x, y=y, z=z, i = i, j = j, k = k, opacity=0.4, color=color, flatshading = True, name=name)
        else:
            return go.Mesh3d(x=x, y=y, z=z, i = i, j = j, k = k, opacity=0.4, color=color, flatshading = True, name=name, hoverinfo=hoverinfo)
    
    def make_blocked_areas_meshes(self, hoverinfo="skip"):
        meshes = []
        meshes.append(go.Mesh3d())
        lightness = 150
        color = f'rgb({lightness},{lightness},{lightness})'
        for blocked_area in self.wh_map.blocked_areas:
            meshes.append(self.make_zone_mesh(blocked_area, hoverinfo="skip", color=color))
        return meshes
    
    def make_task_scatters(self, time=None):
        color = "rgba(10,10,10,0.4)"
        if self.split_tasks:
            tasks = self.fleet.get_fleet_tasks(time=time)
        else:
            tasks = self.task_list.tasks
        if self.task_plot_mode == "no_lines":
            if self.show_task_labels:
                mode = "markers+text"
            else:
                mode = "markers"
            xs =  [task.location(time).x for task in tasks]
            xs.extend(list(chain(*[task.drop_points_xs() for task in tasks])))
            ys =  [task.location(time).y for task in tasks]
            ys.extend(list(chain(*[task.drop_points_ys() for task in tasks])))
            zs =  [task.location(time).z for task in tasks]
            zs.extend(list(chain(*[task.drop_points_zs() for task in tasks])))
            labels =  [task.task_id+"-P" for task in tasks]
            # labels.extend([task.task_id+"-D" for task in tasks])
            return [go.Scatter3d(
                x=xs, 
                y=ys, 
                z=zs, 
                mode=mode,
                marker=dict(color=color, size=5,
                            symbol='square'),
                name="Tasks",
                text=labels)]
        # Otherwise
        task_traces = []
        if self.task_plot_mode == "simple":
            if self.show_task_labels:
                mode = "lines+markers+text"
            else:
                mode = "lines+markers"
            for i in range(len(tasks)):
                task_traces.append(go.Scatter3d(
                    x=[tasks[i].pick_point.x,*tasks[i].drop_points_xs()], 
                    y=[tasks[i].pick_point.y,*tasks[i].drop_points_ys()], 
                    z=[tasks[i].pick_point.z,*tasks[i].drop_points_zs()], 
                    mode=mode,
                    marker=dict(color=color, size=5,
                                symbol='square'),
                    name=f"Task {tasks[i].task_id}",
                    text=[tasks[i].task_id]))
        if self.task_plot_mode == "detailed":
            if self.show_task_labels:
                mode = "markers+text"
            else:
                mode = "markers"
            for i in range(len(tasks)):
                    task_num = int(tasks[i].task_id[1:])
                    marker_size = np.linspace(10,5, len(tasks))
                    pick_str = tasks[i].pick_point.as_str()
                    drop_str = str([point.as_str() for point in tasks[i].drop_points])
                    task_traces.append(go.Scatter3d(
                        x=[tasks[i].pick_point.x,*tasks[i].drop_points_xs()], 
                        y=[tasks[i].pick_point.y,*tasks[i].drop_points_ys()], 
                        z=[tasks[i].pick_point.z,*tasks[i].drop_points_zs()], 
                        mode='lines',
                        marker=dict(color=self.color(task_num)), 
                        name=f"Task {tasks[i].task_id}"))
                    task_traces.append(go.Scatter3d(
                        x=[tasks[i].pick_point.x], 
                        y=[tasks[i].pick_point.y], 
                        z=[tasks[i].pick_point.z], 
                        mode=mode,
                        marker=dict(color=self.color(task_num), size=marker_size[i]),
                        name=f"Pick {tasks[i].task_id}: {pick_str}",
                        text=[tasks[i].task_id]))
                    task_traces.append(go.Scatter3d(
                        x=tasks[i].drop_points_xs(), 
                        y=tasks[i].drop_points_ys(), 
                        z=tasks[i].drop_points_zs(), 
                        mode='markers',
                        marker=dict(color=self.color(task_num), size=marker_size[i],
                                    symbol='square'), 
                        name=f"Drop {tasks[i].task_id}: {drop_str}"))
        return task_traces
    
    def make_path_trace(self, point_path, path_name="", color='rgb(31,119,180)', t_start=None, show_t=False):
        if len(point_path) > 0:
            xs = [point.x for point in point_path]
            ys = [point.y for point in point_path]
            zs = [point.z for point in point_path]
            if t_start == None:
                labels = [f"si={i}" for i in list(range(len(point_path)))]
            else:
                idx = list(range(len(point_path)))
                ts = list(range(t_start,t_start+len(point_path)))
                labels = [f"si={i}, t={t}" for i, t in zip(idx, ts)]
            if show_t:
                ts[0] = ""
                ts[-1] = ""
                return go.Scatter3d(
                    x=xs,
                    y=ys,
                    z=zs,
                    mode='lines+markers+text',
                    marker=dict(color=color, size=3),
                    name=path_name,
                    text=ts)
            else:
                return go.Scatter3d(
                    x=xs,
                    y=ys,
                    z=zs,
                    mode='lines+markers',
                    marker=dict(color=color, size=3),
                    name=path_name,
                    text=labels)
        if path_name[-2:] != "-1":
            return go.Scatter3d(
                x=[point_path[0].x],
                y=[point_path[0].y],
                z=[point_path[0].z],
                mode='markers+text',
                marker=dict(color=color, size=5, symbol='square'),
                name=path_name,
                showlegend=False,
                text=path_name)

    def make_robot_static_scatters(self):
        path_traces = []
        robot_list = self.fleet.get_robots_as_list()
        bot_num = 0
        for bot in robot_list:
            for i in range(len(bot.path)):
                t_start = bot.path_len(r_end=i)
                path_traces.append(self.make_path_trace(bot.path[i],f"{bot.robot_id}-{i}", 
                                self.color(bot_num), t_start, show_t=self.show_t))
            bot_num += 1
        return path_traces
    
    def make_robot_frame_scatters(self, frame_num):
        scatters = []
        amr_marker = dict(size=10, symbol="square", color="red")
        drone_marker = dict(size=7, color="blue")
        amr_list = self.fleet.get_robots_as_list(robot_type="AMR")
        drone_list = self.fleet.get_robots_as_list(robot_type="Drone")
        scatters.append(go.Scatter3d(
                x=[bot.lookup_pos(frame_num).x for bot in amr_list],
                y=[bot.lookup_pos(frame_num).y for bot in amr_list],
                z=[bot.lookup_pos(frame_num).z for bot in amr_list],
                mode='markers+text',
                marker=amr_marker,
                name=f"AMRs at t={frame_num}",
                showlegend=True,
                text=[bot.robot_id for bot in amr_list]))
        scatters.append(go.Scatter3d(
                x=[bot.lookup_pos(frame_num).x for bot in drone_list],
                y=[bot.lookup_pos(frame_num).y for bot in drone_list],
                z=[bot.lookup_pos(frame_num).z for bot in drone_list],
                mode='markers+text',
                marker=drone_marker,
                name=f"Drones at t={frame_num}",
                showlegend=True,
                text=[bot.robot_id for bot in drone_list]))
        return scatters
        
    def animation_setup(self, step_duration=200):
        self.t_max = self.fleet.longest_path_len()
        
        # make figure dict
        self.animation_fig_dict = self.make_layout_dict()
        self.animation_fig_dict["data"] = []
        self.animation_fig_dict["frames"] = []
        self.animation_fig_dict["layout"]["hovermode"] = "closest"
        self.animation_fig_dict["layout"]["updatemenus"] = [
            {
                "buttons": [
                    {
                        "args": [None, {"frame": {"duration": step_duration, "redraw": True},
                                        "fromcurrent": True, "transition": {"duration": step_duration,
                                                                            "easing": "linear"}}],
                        "label": "Play",
                        "method": "animate"
                    },
                    {
                        "args": [[None], {"frame": {"duration": 0, "redraw": True},
                                        "mode": "immediate",
                                        "transition": {"duration": 0}}],
                        "label": "Pause",
                        "method": "animate"
                    }
                ],
                "direction": "left",
                "pad": {"r": 10, "t": 87},
                "showactive": False,
                "type": "buttons",
                "x": 0.1,
                "xanchor": "right",
                "y": 0,
                "yanchor": "top"
            }
        ]

        sliders_dict = {
            "active": 0,
            "yanchor": "top",
            "xanchor": "left",
            "currentvalue": {
                "font": {"size": 20},
                "prefix": "Timestep: ",
                "visible": True,
                "xanchor": "right"
            },
            "transition": {"duration": step_duration, "easing": "cubic-in-out"},
            "pad": {"b": 10, "t": 50},
            "len": 0.9,
            "x": 0.1,
            "y": 0,
            "steps": []
        }
    
        for t in range(self.t_max):
            slider_step = {"args": [
                [t],
                {"frame": {"duration": step_duration, "redraw": True},
                "mode": "immediate",
                "transition": {"duration": step_duration}}
            ],
                "label": t,
                "method": "animate"}
            sliders_dict["steps"].append(slider_step)
        self.animation_fig_dict["layout"]["sliders"] = [sliders_dict]