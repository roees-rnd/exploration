
import numpy as np

class ChoseExplorationPointClass():
    def __init__(self):
        self.list_of_missions = []

    
    def do_step(self, G, pos, list_of_missions):
        closest_mission = None
        dist_to_closest_mission = np.Inf
        path_to_best_mission = []
        for m in list_of_missions:
            path, dist_pos_mission = G.get_path_pos2target(pos, m)
            if dist_pos_mission<dist_to_closest_mission:
                closest_mission = m
                dist_to_closest_mission = dist_pos_mission
                path_to_best_mission = path
        return closest_mission, path_to_best_mission
        


