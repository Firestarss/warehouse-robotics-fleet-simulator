from utils import *
from warehouse_map import *
from robot_fleet import *

class SimParam:
    def __init__(self, name, value=None, min=None, max=None, step=None):
        self.name = name
        self.value=value
        self.min=min
        self.max=max
        self.step=step

        if self.value == None:
            self.sweep = np.arange(self.min, self.max, self.step)
            self.is_dynamic = True
        else:
            self.sweep = None
            self.is_dynamic = False

    def get_as_list(self):
        if self.value:
            return [self.value]
        return self.sweep
    
    def __repr__(self):
        if self.is_dynamic:
            return (f"SimParam('{self.name}',\n" +
                    f"    min={self.min},\n" + 
                    f"    max={self.max},\n" + 
                    f"    step={self.step})")
        else:
            return (f"SimParam('{self.name}',\n" +
                    f"    value={self.value})")

class Evaluation:
    def __init__(self):
        pass
    def __repr__(self):
        pass

class Evaluator:
    def __init__(self):
        self.simulation_parameters = {}

        self.simulation_data = {}

        self.set_static("pick_dist",  5)

        self.set_static("bin_y",  10)
        self.set_static("bin_x",  10)
        self.set_static("bin_z",  10)

        self.set_static("shelf_x",  2)
        self.set_static("shelf_y",  10)
        self.set_static("shelf_z",  4)

        self.set_static("warehouse_x",  4)
        self.set_static("warehouse_y",  2)
        self.set_static("warehouse_z",  60)

        self.set_static("aisle_x",  40)
        self.set_static("aisle_y",  30)

        self.set_static("border",   [20,20,10,10])

        self.set_static("drop_points",  [True,False,False,False])

    def __repr__(self):
        pass

    def log(self, name, data):
        if name in self.simulation_data:
            self.simulation_data[name].append(data)
        else:
            self.simulation_data[name] = [data]
    
    def set_static(self, name, value):
        self.simulation_parameters[name] = SimParam(name, value=value)

    def set_variable(self, name, min, max, step):
        self.simulation_parameters[name] = SimParam(name, min=min, max=max, step=step)
    
    def generate_wh_info(self):
        sim_param_config_list = self.recursive_split_wh_info([self.simulation_parameters])

        # Convert list from SimParam objects to a dictionary readable by warehouse generator
        wh_info_list = []
        for sim_param_config in sim_param_config_list:
            wh_info = {}
            for name, param in sim_param_config.items():
                wh_info[name] = param.value

            wh_info_list.append(wh_info)

        return wh_info_list
    
    def recursive_split_wh_info(self, sim_param_config_list):
        expanded_sim_param_config_list = []
        for config in sim_param_config_list:
            for name, param in config.items():
                if (param.is_dynamic) and (param.name != "n_drone") and (param.name != "n_amr"):
                    for val in param.get_as_list():
                        config[name] = SimParam(name, value=val)
                        expanded_sim_param_config_list.append(copy.copy(config))

        # if nothing has been appended, return the original list
        if len(expanded_sim_param_config_list) == 0:
            return sim_param_config_list

        return self.recursive_split_wh_info(expanded_sim_param_config_list)
        
    def generate_fleet_comp(self):
        test_params = []
        for n_drones in self.simulation_parameters["n_drone"].get_as_list():
            for n_amrs in self.simulation_parameters["n_amr"].get_as_list():
                if n_drones >= n_amrs:
                    test_params.append([
                        ["Drone", int(n_drones)],
                        ["AMR", int(n_amrs)]
                    ])
        
        return test_params