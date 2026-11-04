""" Record the logs of the simulation and store it into logs/reports directory """
import json
import os
from dataclasses import asdict
from time import time,localtime,strftime

import networkx as nx

from road_generation.classes import Road,Node

Nodes = []
Roads = []
Cars = []
Congestion = []
Params = {}

REPORT_DIR = os.path.join("logs","reports")


def reset():
    """清空上一次运行的累积（同一进程里连续跑多次时必需）"""
    Nodes.clear()
    Roads.clear()
    Cars.clear()
    Congestion.clear()
    Params.clear()


def record_map(node:list[Node],road:list[Road]):
    for i in node:
        Nodes.append(i.asdict())
    for j in road:
        Roads.append(j.asdict())
    print("nodes:",Nodes)
    print("roads:",Roads)


def restore_map(node:list[dict],road:list[dict]):
    """把报告里的 nodes/roads 还原成生成器使用的对象，用于回放"""
    nodes = []
    roads = []
    for i in node:
        nodes.append(Node(i['id'],i['pos'],i['tiled_pos'],i['connection'],i['degrees']))
    for j in road:
        roads.append(Road(j['ptA'],j['ptB'],j['length'],j.get('lanes',1)))
    return nodes,roads


def record_congestion(G:nx.Graph):
    for i in G.edges():
        c = {"endpoints":i,"congestion_logs":G[i[0]][i[1]]["congestion_logs"]}
        Congestion.append(c)


def record_car_logs(cars):
    for i in cars:
        Cars.append(i.asdict())


def record_params(road,cars,simulator,run,replay=""):
    """把这次运行用到的全部参数一起写进报告，供下次回放"""
    Params.clear()
    Params.update({"road":asdict(road),
                   "cars":asdict(cars),
                   "simulator":asdict(simulator),
                   "run":dict(run),
                   "replay":replay})


def save_logs():
    report = {"params":Params,"nodes":Nodes,"roads":Roads,
              "congestion":Congestion,"car_logs":Cars}
    os.makedirs(REPORT_DIR,exist_ok=True)
    path = os.path.join(REPORT_DIR,f"report-{strftime('%Y%m%d-%H%M%S',localtime(time()))}.json")
    with open(path,'w') as f:
        json.dump(report,f)
    return path


def find_report(name:str):
    """报告名可以是路径，也可以是 logs/reports 下的文件名（.json 可省）"""
    candidates = [name,os.path.join(REPORT_DIR,name),os.path.join(REPORT_DIR,name+".json")]
    for p in candidates:
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(f"report not found: {name} (tried {candidates})")


def load_report(name:str):
    with open(find_report(name),encoding="utf-8") as f:
        return json.load(f)


def apply_params(params:dict,cfg_road,cfg_car,cfg_simulation):
    """用报告里的参数覆盖当前 Config 对象；报告里缺的字段保持当前值"""
    def merge(cfg,stored):
        base = asdict(cfg)
        for k,v in (stored or {}).items():
            if k in base:
                base[k] = v
        return type(cfg)(**base)
    return (merge(cfg_road,params.get("road")),
            merge(cfg_car,params.get("cars")),
            merge(cfg_simulation,params.get("simulator")))


def apply_run_params(run:dict):
    """把 run 段写回 attributes，并同步那些之前用 from attributes import * 复制走该名字的模块"""
    import sys
    import attributes
    applied = {}
    prefixes = ("__main__","main","attributes","visualize","core","road_generation","algorithm")
    for name,value in (run or {}).items():
        if not hasattr(attributes,name):
            continue
        setattr(attributes,name,value)
        applied[name] = value
        for mod in list(sys.modules.values()):
            modname = getattr(mod,"__name__","") or ""
            if mod is attributes or modname.split(".")[0] not in prefixes:
                continue
            if hasattr(mod,name):
                setattr(mod,name,value)
    return applied


def replay_spawn_plan(report:dict):
    """还原车辆生成计划：{帧: [(id, 起点, 终点), ...]}，起点终点为空表示随机"""
    plan = {}
    for c in report.get("car_logs",[]):
        logs = c.get("logs") or []
        frame = logs[0][1] if logs and isinstance(logs[0],(list,tuple)) else 0
        plan.setdefault(int(frame),[]).append((c.get("id"),c.get("startPt"),c.get("endPt")))
    for v in plan.values():
        v.sort()
    return plan
