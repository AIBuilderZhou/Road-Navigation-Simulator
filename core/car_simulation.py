"""Traffic simulation.

Three layers:
  1. static: every road is prepared once (length in metres, lanes, free-flow speed,
     jam capacity, level-of-service thresholds);
  2. longitudinal: each road keeps one queue per lane; cars follow their own lane's
     leader with the IDM, entrance admission keeps queues physical (no overlap),
     a full downstream lane makes cars queue at the junction (spillback);
  3. evaluation: a road is judged by several measures - density (veh/km/lane),
     mean speed, speed ratio, queue length and level of service - not by one speed.

One frame lasts 3600/attributes.FPS seconds (1 s with FPS=3600), which the IDM needs.
"""
import math

import attributes
from attributes import Config_simulator, Config_car_generator

#level of service by density, veh per km per lane (urban road, HCM-like)
LOS_LIMITS = (7.0, 11.0, 16.0, 22.0, 28.0)


def _los(density):
    for grade, limit in zip("ABCDE", LOS_LIMITS):
        if density < limit:
            return grade
    return "F"


def _prepare(G, cfg: Config_simulator):
    """计算每条边的静态参数并缓存在 G.graph["_sim"]，每张图只做一次"""
    sim = G.graph.get("_sim")
    if sim is not None:
        return sim
    car_len_m = cfg.car_length * 1000.0
    min_gap_m = cfg.minimum_distance * 1000.0
    max_lanes = max((d.get("lanes", 1) for _, _, d in G.edges(data=True)), default=1)
    edges = []
    index = {}
    for u, v, d in G.edges(data=True):
        lanes = d.get("lanes", 1)
        length_m = d["length"] * cfg.meters_per_unit
        free = max(cfg.minimum_speed, cfg.standard_speed - cfg.lane_speed_step * (max_lanes - lanes))
        edges.append({
            "u": u, "v": v, "lanes": lanes,
            "length_m": length_m, "length_km": length_m / 1000.0,
            "free_speed": free, "v0": free / 3.6,
            "jam_cars": max(1.0, lanes * length_m / (car_len_m + min_gap_m)),
            "lists": [[] for _ in range(lanes)],
            "cars": 0, "density": 0.0, "v_mean": free,
            "speed_ratio": 1.0, "queue": 0, "congestion": 0.0, "los": "A",
        })
        index[(u, v)] = len(edges) - 1
        index[(v, u)] = len(edges) - 1
    sim = {"edges": edges, "index": index, "dt": 3600.0 / attributes.FPS,
           "car_len_m": car_len_m, "min_gap_m": min_gap_m, "max_lanes": max_lanes}
    G.graph["_sim"] = sim
    if sim["dt"] > 2.0:
        print(f"[warn] one frame lasts {sim['dt']:.1f}s - too coarse for car-following, "
              f"set FPS >= 1800")
    return sim


def _init_car(c, sim):
    """把路线编译成边 id 列表；真正的进场位置由 _admit 决定"""
    index = sim["index"]
    compiled = []
    for k in range(len(c.route) - 1):
        e = index.get((c.route[k], c.route[k + 1]))
        if e is None:
            break
        compiled.append(e)
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
        c.logs.append("end")


def _free_lane(e, car_len, min_gap):
    """找一条入口有足够空间的车道，返回车道号（-1 表示全满）"""
    best_lane = -1
    best_gap = -1.0
    for lane, lst in enumerate(e["lists"]):
        gap = (lst[-1].s - car_len - min_gap) if lst else float("inf")
        if gap >= 0 and gap > best_gap:
            best_lane, best_gap = lane, gap
    return best_lane


def _admit(c, sim, edges):
    """让刚生成的车进入它在的第一条边；入口满则等下一帧"""
    e = edges[c.route_edges[c.progress]]
    lane = _free_lane(e, sim["car_len_m"], sim["min_gap_m"])
    if lane < 0:
        return False
    c.lane = lane
    c.s = 0.0
    c.needs_place = False
    e["lists"][lane].append(c)
    return True


def _idm(v, v0, gap, dv, cfg: Config_simulator):
    """IDM 加速度：v 本车速度, v0 期望速度, gap 净车距, dv 速度差"""
    if gap <= 0.05:
        return -3.0 * cfg.idm_decel
    v = max(0.0, v)
    s_star = cfg.idm_min_gap + max(0.0, v * cfg.idm_time_gap + v * dv / (2.0 * math.sqrt(cfg.idm_accel * cfg.idm_decel)))
    return cfg.idm_accel * (1.0 - (v / v0) ** cfg.idm_delta - (s_star / gap) ** 2)


def _move(G, sim, cars, cfg: Config_simulator, frame):
    edges = sim["edges"]
    dt = sim["dt"]
    car_len = sim["car_len_m"]
    min_gap = sim["min_gap_m"]

    #1) 清空车道队列，把在途车辆按「边 + 车道」入队（已到达 / 入口没位置的车跳过）
    for e in edges:
        for lst in e["lists"]:
            lst.clear()
        e["cars"] = 0
    to_admit = []
    for c in cars:
        if c.destination:
            continue
        if not c.route_edges:
            _init_car(c, sim)
            if c.destination:
                continue
        if c.needs_place:
            to_admit.append(c)                      #进场要等排序之后，才能看到真正的队尾车
            continue
        e = edges[c.route_edges[c.progress]]
        e["lists"][c.lane].append(c)
        e["cars"] += 1

    #2) 先按位置排好每条车道（入口/过街判断依赖「队尾车」，必须有序）
    for e in edges:
        for lst in e["lists"]:
            if len(lst) > 1:
                lst.sort(key=lambda c: c.s, reverse=True)

    #2b) 新生成的车按排序后的队尾位置准入（放不下的留到下一帧）
    for c in to_admit:
        if _admit(c, sim, edges):
            edges[c.route_edges[c.progress]]["cars"] += 1

    #3) 每条车道内逐车 IDM；车头若无下游空间，就把本边尾部当成静止障碍
    for e in edges:
        for lst in e["lists"]:
            if not lst:
                continue
            order = tuple(lst)                      #快照，避免本帧新进车辆重复推进
            L = e["length_m"]
            v0 = e["v0"]
            for i, c in enumerate(order):
                if c.moved_frame == frame:
                    continue
                if i > 0:
                    lead = order[i - 1]
                    a = _idm(c.v, v0, lead.s - car_len - c.s, c.v - lead.v, cfg)
                else:
                    nxt = c.route_edges[c.progress + 1] if c.progress + 1 < len(c.route_edges) else -1
                    if nxt >= 0 and _free_lane(edges[nxt], car_len, min_gap) < 0:
                        a = _idm(c.v, v0, L - c.s, c.v, cfg)      #下游进不去 -> 排队
                    else:
                        a = cfg.idm_accel * (1.0 - (max(0.0, c.v) / v0) ** cfg.idm_delta)
                s_prev = c.s
                c.v = max(0.0, c.v + a * dt)
                c.s += c.v * dt
                if i > 0:                                   #硬性跟车约束：一帧内不允许越过前车车尾
                    limit = order[i - 1].s - car_len
                    if c.s > limit:
                        c.s = max(s_prev, limit)
                        c.v = min(c.v, max(0.0, order[i - 1].v))
                c.tot_distance += c.v * dt / 1000.0
                c.time_cost += 1.0 / attributes.FPS
                c.moved_frame = frame

                #4) 驶离当前边：下游某车道入口有空位才进，否则停在路口等待（可向上游溢出）
                while c.s >= L:
                    if c.progress + 1 >= len(c.route_edges):
                        c.destination = True
                        c.logs.append("end")
                        break
                    nxt = c.route_edges[c.progress + 1]
                    ne = edges[nxt]
                    lane = _free_lane(ne, car_len, min_gap)
                    if lane < 0:
                        c.s = L
                        c.v = 0.0
                        break
                    c.s = 0.0                            #从新边起点进入，丢弃溢出距离，避免与前车重叠
                    c.progress += 1
                    c.lane = lane
                    c.logs.append(frame)
                    ne["lists"][lane].append(c)
                    ne["cars"] += 1
                    L = ne["length_m"]
                    if c.route[c.progress] == c.endPt:
                        c.destination = True
                        c.logs.append("end")
                        break
                c.distance = min(1.0, c.s / L) if L else 1.0


def _update_edges(G, sim, cfg: Config_simulator, frame):
    """多指标评判每条边，并写回图属性 + 记录时间序列"""
    stop_speed = cfg.minimum_speed / 3.6
    for e in sim["edges"]:
        lanes_list = e["lists"]
        n = sum(len(lst) for lst in lanes_list)
        e["cars"] = n
        length_km = e["length_km"]
        lanes = e["lanes"]
        density = n / (lanes * length_km) if length_km > 0 and lanes else 0.0
        queue = 0
        if n:
            v_sum = 0.0
            for lst in lanes_list:
                for c in lst:
                    v_sum += c.v
                    if c.v <= stop_speed:
                        queue += 1
            v_mean = v_sum / n * 3.6
        else:
            v_mean = e["free_speed"]
        speed_ratio = min(1.0, v_mean / e["free_speed"]) if e["free_speed"] else 1.0
        #拥堵 = 「速度损失」与「密度水平」里更严重的那个（多指标，而不是只看速度）
        congestion = min(1.0, max(0.0, max(1.0 - speed_ratio, density / cfg.jam_density)))
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
        if frame % cfg.log_every == 0:
            d["congestion_logs"].append(congestion)


def whole_process(G, cars, cfg_car: Config_car_generator, cfg_simulation: Config_simulator, frame):
    sim = _prepare(G, cfg_simulation)
    _move(G, sim, cars, cfg_simulation, frame)
    _update_edges(G, sim, cfg_simulation, frame)


def cars_run(G, cars, cfg_car: Config_car_generator, cfg_simulation: Config_simulator, frame):
    """兼容旧接口：只推进车辆"""
    sim = _prepare(G, cfg_simulation)
    _move(G, sim, cars, cfg_simulation, frame)


def congestion_calculation(G, cars, cfg_simulation: Config_simulator):
    """兼容旧接口：只更新边状态"""
    sim = _prepare(G, cfg_simulation)
    _update_edges(G, sim, cfg_simulation, 0)


def speed_calculation(road: dict, cfg_simulation: Config_simulator):
    """兼容旧接口：由密度估一个速度（km/h），Greenshields 形式，仅供外部调用"""
    length_km = road["length"] * cfg_simulation.meters_per_unit / 1000.0
    lanes = road.get("lanes", 1)
    density = road.get("cars", 0) / max(1e-9, lanes * length_km)
    v = cfg_simulation.standard_speed * max(0.0, 1.0 - density / cfg_simulation.jam_density)
    return max(cfg_simulation.minimum_speed, v)
