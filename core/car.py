import random
from attributes import *

class car():
    def __init__(self,id,NUM_NODES,pos):
        self.id = id
        self.startPt = random.randint(0,NUM_NODES-1)
        self.endPt = self.startPt
        while self.startPt == self.endPt:
            self.endPt = random.randint(0,NUM_NODES-1)
        #self.startPt = pos[self.startPt]
        #self.endPt = pos[self.endPt]
        self.route = []
        self.distance = 0          #fraction of the current road, kept for the drawing code
        self.progress = 0          #index of the node the car is leaving
        self.time_cost = 0
        self.destination = False
        self.tot_distance = 0      #km
        self.logs = []
        #car-following state
        self.route_edges = []      #compiled edge ids of route, filled on the first simulation step
        self.s = 0.0               #metres travelled along the current road
        self.v = 0.0               #speed in m/s
        self.lane = 0              #lane index inside the current road
        self.car_len_m = 5.0
        self.moved_frame = -1      #guards against advancing twice in one frame
        self.needs_place = True    #waiting for a free slot at the road entrance

    def asdict(self):
        return {'id':self.id,'startPt':self.startPt,'endPt':self.endPt,'route':self.route,'time_cost':self.time_cost,'total_distance':self.tot_distance,'logs':self.logs}
