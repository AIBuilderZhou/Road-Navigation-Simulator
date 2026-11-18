"""Vehicle state used by the simulator, the report writer and the plots."""
import random


class Car:
    def __init__(self, car_id, num_nodes, start=None, goal=None):
        self.id = car_id
        self.startPt = random.randrange(num_nodes) if start is None else int(start)
        self.endPt = random.randrange(num_nodes) if goal is None else int(goal)
        if self.startPt == self.endPt:
            while self.endPt == self.startPt:
                self.endPt = random.randrange(num_nodes)

        self.route = []
        self.distance = 0.0        # fraction of the current road, used by the plots
        self.progress = 0          # index of the node the car is leaving
        self.time_cost = 0.0
        self.destination = False
        self.arrive_frame = -1
        self.spawn_frame = -1
        self.tot_distance = 0.0    # km
        self.logs = []

        # car-following state, filled by the simulator
        self.route_edges = []      # compiled directed edge ids
        self.s = 0.0               # metres travelled along the current road
        self.v = 0.0               # speed in m/s
        self.lane = 0              # lane index inside the current road
        self.car_len_m = 5.0
        self.moved_frame = -1      # guards against advancing twice in one frame
        self.needs_place = True    # waiting for a free slot at the road entrance

    def asdict(self):
        """Legacy dictionary form, kept for the old report reader."""
        return {
            "id": self.id,
            "startPt": self.startPt,
            "endPt": self.endPt,
            "spawn_frame": self.spawn_frame,
            "route": self.route,
            "time_cost": self.time_cost,
            "total_distance": self.tot_distance,
            "arrived": self.destination,
            "arrive_frame": self.arrive_frame,
            "logs": self.logs,
        }
