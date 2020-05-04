
import numpy as np

class ChoseExplorationPointClass():
    def __init__(self):
        self.list_of_missions = []

    
    def do_step(self, G, pos, list_of_missions):
        closest_mission = None
        dist_to_closest_mission = np.Inf
        for m in list_of_missions:
            dist_pos_mission = G.getDistInGraph(pos, m)
            if dist_pos_mission<dist_to_closest_mission:
                closest_mission = m
                dist_to_closest_mission = dist_pos_mission
        return closest_mission
        


