"""Traffic simulation with two independent directions per road.

Three layers:
  1. static: every directed arc is prepared once (length in metres, lanes,
     free-flow speed, jam capacity, level-of-service thresholds);
  2. longitudinal: each arc keeps one queue per lane, so the A->B traffic and
     the B->A traffic never share a queue. Cars follow their own lane's leader
     with the IDM; entrance admission keeps queues physical; a full downstream
     lane makes cars queue at the junction (spillback);
  3. evaluation: an arc is judged by density (veh/km/lane), mean speed, speed
     ratio, queue length and level of service - not by one speed.

One frame lasts 3600/FPS seconds (1/12 s with the default FPS=43200), which the
IDM needs.
"""
import math

import attributes
from attributes import Config_simulator

# Level of service by density, vehicles per km per lane (urban road, HCM-like).
LOS_LIMITS = (7.0, 11.0, 16.0, 22.0, 28.0)


def _los(density):
    for grade, limit in zip("ABCDE", LOS_LIMITS):
        if density < limit:
            return grade
    return "F"


def _prepare(G, cfg: Config_simulator, fps=None):
    """Build and cache the static per-arc data in G.graph["_sim"]."""
    sim = G.graph.get("_sim")
    if sim is not None:
        return sim
    if not G.is_directed():
        raise ValueError(
            "car_simulation requires a directed graph; "
            "build it with road_generation.generate_graph"
        )

    fps = attributes.FPS if fps is None else fps
    car_len_m = cfg.car_length * 1000.0
    min_gap_m = cfg.minimum_distance * 1000.0
    max_lanes = max((d.get("lanes", 1) for _, _, d in G.edges(data=True)), default=1)

    edges = []
    index = {}
    for u, v, d in G.edges(data=True):
        lanes = max(1, int(d.get("lanes", 1)))
        length_m = d["length"] * cfg.meters_per_unit
        free = max(
            cfg.minimum_speed,
            cfg.standard_speed - cfg.lane_speed_step * (max_lanes - lanes),
        )
        edges.append(
            {
                "u": u,
                "v": v,
                "road_id": d.get("road_id"),
                "direction": d.get("direction"),
                "lanes": lanes,
                "length_m": length_m,
                "length_km": length_m / 1000.0,
                "free_speed": free,
                "v0": free / 3.6,
                "jam_cars": max(1.0, lanes * length_m / (car_len_m + min_gap_m)),
                "lists": [[] for _ in range(lanes)],
                "cars": 0,
                "density": 0.0,
                "v_mean": free,
                "speed_ratio": 1.0,
                "queue": 0,
                "congestion": 0.0,
                "los": "A",
            }
        )
        if (u, v) in index:
            raise ValueError(f"duplicate directed edge {u}->{v}")
        index[(u, v)] = len(edges) - 1

    sim = {
        "edges": edges,
        "index": index,
        "dt": 3600.0 / fps,
        "fps": fps,
        "car_len_m": car_len_m,
        "min_gap_m": min_gap_m,
        "max_lanes": max_lanes,
    }
    G.graph["_sim"] = sim
    if sim["dt"] > 2.0:
        print(
            f"[warn] one frame lasts {sim['dt']:.1f}s - too coarse for "
            f"car-following, set FPS >= 1800"
        )
    return sim


def _init_car(c, sim, frame):
    """Compile the node route into directed edge ids; _admit places the car."""
    index = sim["index"]
    compiled = []
    for k in range(len(c.route) - 1):
        edge_id = index.get((c.route[k], c.route[k + 1]))
        if edge_id is None:
            break
        compiled.append(edge_id)

    c.route_edges = compiled
    c.s = 0.0
    c.v = 0.0
    c.lane = 0
    c.car_len_m = sim["car_len_m"]
    c.distance = 0.0
    c.moved_frame = -1
    c.needs_place = True
    if not compiled:
        c.destination = True
        c.arrive_frame = frame
        c.logs.append("end")


def _free_lane(e, car_len, min_gap):
    """Return a lane with enough room at the entrance, or -1 if all are full."""
    best_lane = -1
    best_gap = -1.0
    for lane, cars in enumerate(e["lists"]):
        gap = (cars[-1].s - car_len - min_gap) if cars else float("inf")
        if gap >= 0 and gap > best_gap:
            best_lane, best_gap = lane, gap
    return best_lane


def _admit(c, sim, edges):
    """Put a newly spawned car on the first arc, waiting if the entrance is full."""
    e = edges[c.route_edges[c.progress]]
    if __debug__:
        assert c.route[c.progress] == e["u"] and c.route[c.progress + 1] == e["v"]
    lane = _free_lane(e, sim["car_len_m"], sim["min_gap_m"])
    if lane < 0:
        return False
    c.lane = lane
    c.s = 0.0
    c.needs_place = False
    e["lists"][lane].append(c)
    return True


def _idm(v, v0, gap, dv, cfg: Config_simulator):
    """IDM acceleration: v = speed, v0 = desired speed, gap = free gap, dv = dv."""
    if gap <= 0.05:
        return -3.0 * cfg.idm_decel
    v = max(0.0, v)
    s_star = cfg.idm_min_gap + max(
        0.0,
        v * cfg.idm_time_gap
        + v * dv / (2.0 * math.sqrt(cfg.idm_accel * cfg.idm_decel)),
    )
    return cfg.idm_accel * (
        1.0 - (v / v0) ** cfg.idm_delta - (s_star / gap) ** 2
    )


def _move(G, sim, cars, cfg: Config_simulator, frame):
    edges = sim["edges"]
    dt = sim["dt"]
    car_len = sim["car_len_m"]
    min_gap = sim["min_gap_m"]

    # 1) Clear the lane queues, then enqueue every moving car on its own arc.
    for e in edges:
        for lane in e["lists"]:
            lane.clear()
        e["cars"] = 0

    to_admit = []
    for c in cars:
        if c.destination:
            continue
        if not c.route_edges:
            _init_car(c, sim, frame)
            if c.destination:
                continue
        if c.needs_place:
            to_admit.append(c)  # wait until the queues are sorted
            continue
        e = edges[c.route_edges[c.progress]]
        if __debug__:
            assert c.route[c.progress] == e["u"] and c.route[c.progress + 1] == e["v"]
        e["lists"][c.lane].append(c)
        e["cars"] += 1

    # 2) Sort each lane by position (the entrance check needs the real tail).
    for e in edges:
        for lane in e["lists"]:
            if len(lane) > 1:
                lane.sort(key=lambda car: car.s, reverse=True)

    # 2b) Admit new cars after sorting; cars that do not fit wait one frame.
    for c in to_admit:
        if _admit(c, sim, edges):
            edges[c.route_edges[c.progress]]["cars"] += 1

    # 3) Advance every car with the IDM. The head car treats a blocked
    # downstream arc as a standing obstacle at the end of this arc.
    for e in edges:
        for lane in e["lists"]:
            if not lane:
                continue
            order = tuple(lane)  # snapshot: cars entering this frame are not advanced
            edge_length = e["length_m"]
            v0 = e["v0"]
            for i, c in enumerate(order):
                if c.moved_frame == frame:
                    continue
                # A car that crosses an arc must compare against the new arc's
                # length; keep this per car so the next car in the lane still
                # uses edge_length.
                current_length = edge_length
                if i > 0:
                    leader = order[i - 1]
                    accel = _idm(
                        c.v,
                        v0,
                        leader.s - car_len - c.s,
                        c.v - leader.v,
                        cfg,
                    )
                else:
                    next_edge = (
                        c.route_edges[c.progress + 1]
                        if c.progress + 1 < len(c.route_edges)
                        else -1
                    )
                    if next_edge >= 0 and _free_lane(
                        edges[next_edge], car_len, min_gap
                    ) < 0:
                        accel = _idm(c.v, v0, current_length - c.s, c.v, cfg)
                    else:
                        accel = cfg.idm_accel * (
                            1.0 - (max(0.0, c.v) / v0) ** cfg.idm_delta
                        )

                s_previous = c.s
                c.v = max(0.0, c.v + accel * dt)
                c.s += c.v * dt

                if i > 0:
                    # Hard constraint: a car must not pass the leader's rear
                    # bumper inside one frame.
                    limit = order[i - 1].s - car_len
                    if c.s > limit:
                        c.s = max(s_previous, limit)
                        c.v = min(c.v, max(0.0, order[i - 1].v))

                c.tot_distance += c.v * dt / 1000.0
                c.time_cost += dt / 3600.0
                c.moved_frame = frame

                # 4) Leave the arc only when the next arc has a free entrance.
                while c.s >= current_length:
                    if c.progress + 1 >= len(c.route_edges):
                        c.destination = True
                        c.arrive_frame = frame
                        c.logs.append("end")
                        break
                    next_id = c.route_edges[c.progress + 1]
                    next_edge = edges[next_id]
                    lane_id = _free_lane(next_edge, car_len, min_gap)
                    if lane_id < 0:
                        c.s = current_length
                        c.v = 0.0
                        break
                    current_edge = edges[c.route_edges[c.progress]]
                    current_lane = c.lane
                    c.s = 0.0  # drop the overflow, so cars never overlap
                    c.progress += 1
                    c.lane = lane_id
                    c.logs.append(frame)
                    next_edge["lists"][lane_id].append(c)
                    next_edge["cars"] += 1
                    if current_edge is not next_edge:
                        # The car has left current_edge; remove it so the two
                        # arcs do not both count it in the same frame.
                        try:
                            current_edge["lists"][current_lane].remove(c)
                        except ValueError:
                            pass
                    current_length = next_edge["length_m"]
                    if c.route[c.progress] == c.endPt:
                        c.destination = True
                        c.arrive_frame = frame
                        c.logs.append("end")
                        break

                c.distance = (
                    min(1.0, c.s / current_length) if current_length else 1.0
                )


def _update_edges(G, sim, cfg: Config_simulator, frame):
    """Evaluate every directed arc and append its congestion sample."""
    stop_speed = cfg.minimum_speed / 3.6
    log_every = max(1, int(cfg.log_every))

    for e in sim["edges"]:
        n = sum(len(lane) for lane in e["lists"])
        e["cars"] = n
        length_km = e["length_km"]
        lanes = e["lanes"]
        density = n / (lanes * length_km) if length_km > 0 and lanes else 0.0

        queue = 0
        if n:
            speed_sum = 0.0
            for lane in e["lists"]:
                for c in lane:
                    speed_sum += c.v
                    if c.v <= stop_speed:
                        queue += 1
            v_mean = speed_sum / n * 3.6
        else:
            v_mean = e["free_speed"]

        speed_ratio = min(1.0, v_mean / e["free_speed"]) if e["free_speed"] else 1.0
        # Congestion is the worse of "speed loss" and "density level", so an
        # almost stopped road and a packed road both score high.
        congestion = min(
            1.0, max(0.0, max(1.0 - speed_ratio, density / cfg.jam_density))
        )

        e["density"] = density
        e["v_mean"] = v_mean
        e["speed_ratio"] = speed_ratio
        e["queue"] = queue
        e["congestion"] = congestion
        e["los"] = _los(density)

        d = G[e["u"]][e["v"]]
        d["cars"] = n
        d["density"] = density
        d["v_mean"] = v_mean
        d["speed_ratio"] = speed_ratio
        d["queue"] = queue
        d["los"] = e["los"]
        d["congestion"] = congestion
        if frame % log_every == 0:
            d["congestion_logs"].append(congestion)


def whole_process(
    G, cars, cfg_car, cfg_simulation: Config_simulator, frame, fps=None
):
    sim = _prepare(G, cfg_simulation, fps)
    _move(G, sim, cars, cfg_simulation, frame)
    _update_edges(G, sim, cfg_simulation, frame)


def cars_run(G, cars, cfg_car, cfg_simulation: Config_simulator, frame, fps=None):
    """Legacy compatibility wrapper: advance the cars only."""
    sim = _prepare(G, cfg_simulation, fps)
    _move(G, sim, cars, cfg_simulation, frame)


def congestion_calculation(G, cars, cfg_simulation: Config_simulator, fps=None):
    """Legacy compatibility wrapper: refresh the arc metrics only."""
    sim = _prepare(G, cfg_simulation, fps)
    _update_edges(G, sim, cfg_simulation, 0)


def speed_calculation(road: dict, cfg_simulation: Config_simulator):
    """Legacy density-only speed estimate (km/h), kept for external callers."""
    length_km = road["length"] * cfg_simulation.meters_per_unit / 1000.0
    lanes = road.get("lanes", 1)
    density = road.get("cars", 0) / max(1e-9, lanes * length_km)
    v = cfg_simulation.standard_speed * max(
        0.0, 1.0 - density / cfg_simulation.jam_density
    )
    return max(cfg_simulation.minimum_speed, v)
