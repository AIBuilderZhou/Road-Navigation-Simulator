from road_generation import generate
from road_generation.Default import default_road
import core.car as car
import algorithm.astar as astar
import visualize.road_draw as visualize
import visualize.result_congestion as result_congestion
import core.car_simulation as processing
from attributes import *
import core.logs_recording as recording
from road_generation.generate import generate_graph

import networkx as nx
from time import time,localtime,strftime
import random
from tqdm import tqdm
import imageio


''' CONFIG '''
cfg_road = Config_road_generator(
    num_nodes=100,
    max_lanes=3,
    max_degree=4,
    edge_prob=0.2,
    force_single_road=[],
    connection_limit=10,
    ring=False,
    exp_road_length=10,
    var_road_length=0.2,
    angle_variation=0.1)
cfg_car = Config_car_generator(
    #真实单位下这张 50 节点路网满负载约 1300 辆，200 辆是正常车流
    num_cars_initially=200,
    #一帧现在是 1 秒，所以每 30 秒尝试生成一次、每次 1~3 辆
    new_car_prob=1/60,
    new_car_range=[1,3]
)
cfg_simulation = Config_simulator(
    standard_speed=80
)

recording.reset()

''' CREATE GRAPH '''
if REPLAY_REPORT:
    #replay a past report: road network, cars and running parameters all come from it
    report = recording.load_report(REPLAY_REPORT)
    params = report.get("params",{})
    cfg_road,cfg_car,cfg_simulation = recording.apply_params(params,cfg_road,cfg_car,cfg_simulation)
    applied = recording.apply_run_params(params.get("run",{}))
    FPS = applied.get("FPS",FPS)
    FRAMES = applied.get("FRAMES",FRAMES)
    PIC_SCALE = applied.get("PIC_SCALE",PIC_SCALE)
    SHOW_MID_PROCESS = applied.get("SHOW_MID_PROCESS",SHOW_MID_PROCESS)
    FRAMES_PER_OUTPUT = applied.get("FRAMES_PER_OUTPUT",FRAMES_PER_OUTPUT)
    GENERATE_GIF = applied.get("GENERATE_GIF",GENERATE_GIF)
    nodeList,roadList = recording.restore_map(report["nodes"],report["roads"])
    spawn_plan = recording.replay_spawn_plan(report)
    print(f"[replay] {REPLAY_REPORT}: {len(nodeList)} nodes, {len(roadList)} roads, "
          f"{sum(len(v) for v in spawn_plan.values())} cars, FPS={FPS}, FRAMES={FRAMES}")
else:
    nodeList,roadList = default_road(cfg_road)
    spawn_plan = None

G = generate_graph(nodeList,roadList)
#the coordinates come from the generator itself, no spring_layout needed
pos = nx.get_node_attributes(G, "pos")
edges = G.edges()
nodes = G.nodes()
recording.record_map(nodeList,roadList)
recording.record_params(cfg_road,cfg_car,cfg_simulation,
                        {"FPS":FPS,"FRAMES":FRAMES,"PIC_SCALE":PIC_SCALE,
                         "SHOW_MID_PROCESS":SHOW_MID_PROCESS,
                         "FRAMES_PER_OUTPUT":FRAMES_PER_OUTPUT,
                         "GENERATE_GIF":GENERATE_GIF},
                        REPLAY_REPORT)

''' GENERATE CARS '''
cars = []
num_cars = 0
num_nodes_available = len(nodeList)

def spawn_car(cid,start,end,frame):
    """start/end 为 None 时随机取起终点（正常生成）；否则按报告回放这批车"""
    global num_cars
    c = car.car(cid,num_nodes_available,pos)
    if start is not None and end is not None:
        c.startPt,c.endPt = start,end
    c.route = astar.astar(G,c.startPt,c.endPt,pos)
    if not c.route or len(c.route) < 2:
        print(f"[warn] car {cid}: no route {c.startPt} -> {c.endPt}, skipped")
        c.destination = True
    c.logs.append(("Spawned",frame))
    cars.append(c)
    num_cars += 1

if spawn_plan is None:
    for i in range(cfg_car.num_cars_initially):
        spawn_car(num_cars+1,None,None,0)
else:
    #在报告里，id <= num_cars_initially 且生成帧为 0 的车属于「开局车队」，原运行是在循环之前建好、
    #因此从第 0 帧就开始移动；其余车都是逐帧在 whole_process 之后才生成的。这里按同样的时机拆分。
    initial = []
    rest = []
    for entry in spawn_plan.get(0,[]):
        (initial if entry[0] is not None and entry[0] <= cfg_car.num_cars_initially else rest).append(entry)
    spawn_plan[0] = rest
    for cid,start,end in initial:
        spawn_car(cid,start,end,0)

''' PROCESSING '''
frames = []
for t in tqdm(range(FRAMES+1)):
    processing.whole_process(G,cars,cfg_car,cfg_simulation,t)
    if SHOW_MID_PROCESS and t%FRAMES_PER_OUTPUT==0:
        visualize.draw(G,cars,t,frames,cfg_simulation.standard_speed)
    if spawn_plan is None:
        if random.random() < cfg_car.new_car_prob:
            for i in range(random.randint(cfg_car.new_car_range[0],cfg_car.new_car_range[1])):
                spawn_car(num_cars+1,None,None,t)
    else:
        for cid,start,end in spawn_plan.get(t,[]):
            spawn_car(cid,start,end,t)

''' CALCULATE RESULTS'''
recording.record_congestion(G)
recording.record_car_logs(cars)
passed = 0
tot_time = 0
tot_avg_speed = 0
for i in cars:
    if i.destination:
        passed += 1
        tot_time += i.time_cost
        tot_avg_speed += i.tot_distance / i.time_cost if i.time_cost else 0.0
avg_time = tot_time/passed if passed else 0.0
avg_speed = tot_avg_speed/passed if passed else 0.0

#网络层面的多指标：车辆加权平均车速 + 时序平均拥堵 + 服务水平分布
tot_cars_now = sum(d["cars"] for _,_,d in G.edges(data=True))
if tot_cars_now:
    net_speed = sum(d["v_mean"]*d["cars"] for _,_,d in G.edges(data=True))/tot_cars_now
else:   #网络此刻没车时退化为按道路长度加权的空间平均车速（=自由流）
    tot_length = sum(d["length"] for _,_,d in G.edges(data=True))
    net_speed = (sum(d["v_mean"]*d["length"] for _,_,d in G.edges(data=True))/tot_length
                 if tot_length else 0.0)
cong_series = [sum(d["congestion_logs"])/len(d["congestion_logs"])
               for _,_,d in G.edges(data=True) if d["congestion_logs"]]
mean_congestion = sum(cong_series)/len(cong_series) if cong_series else 0.0
los_count = {g:0 for g in "ABCDEF"}
for _,_,d in G.edges(data=True):
    los_count[d.get("los","A")] = los_count.get(d.get("los","A"),0)+1

''' REPORTS, RESULT IMAGES AND GIF'''
recording.save_logs()
result_congestion.draw(G,passed,avg_time,avg_speed,cfg_simulation.standard_speed,
                       net_speed,mean_congestion,los_count)
if GENERATE_GIF:
    imageio.mimsave(f"logs/gif/traffic{strftime('%Y%m%d-%H%M%S',localtime(time()))}.gif", frames, fps=FPS)
