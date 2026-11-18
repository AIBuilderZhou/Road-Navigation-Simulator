"""Record a simulation run as JSON and replay a previous report."""
import json
import os
from dataclasses import asdict
from time import localtime, strftime, time

from road_generation.classes import Node, Road

REPORT_SCHEMA = 2
REPORT_DIR = os.path.join("logs", "reports")

Nodes = []
Roads = []
Cars = []
Congestion = []
Params = {}


def reset():
    """Clear the accumulators (needed when one process runs several scenes)."""
    Nodes.clear()
    Roads.clear()
    Cars.clear()
    Congestion.clear()
    Params.clear()


def record_map(nodes, roads):
    for n in nodes:
        Nodes.append(n.asdict())
    for i, r in enumerate(roads):
        road = r.asdict()
        road["id"] = i
        Roads.append(road)


def restore_map(node_list, road_list):
    """Rebuild the generator objects from a report, used by replay."""
    nodes = []
    roads = []
    for n in node_list:
        nodes.append(
            Node(
                n["id"],
                n["pos"],
                n["tiled_pos"],
                n["connection"],
                n["degrees"],
            )
        )
    for r in road_list:
        roads.append(Road(r["ptA"], r["ptB"], r["length"], r.get("lanes", 1)))
    return nodes, roads


def _car_events(car):
    """Convert the internal log encoding into explicit events for the viewer."""
    events = [
        {"frame": int(car.spawn_frame), "type": "spawn", "node": car.startPt}
    ]
    route = car.route or []
    numeric_index = 0
    for entry in car.logs:
        if isinstance(entry, bool) or not isinstance(entry, (int, float)):
            continue
        edge_index = numeric_index + 1
        if edge_index + 1 < len(route):
            events.append(
                {
                    "frame": int(entry),
                    "type": "enter",
                    "from": route[edge_index],
                    "to": route[edge_index + 1],
                }
            )
        numeric_index += 1
    if car.destination:
        arrive = car.arrive_frame if car.arrive_frame >= 0 else car.spawn_frame
        events.append(
            {"frame": int(arrive), "type": "arrive", "node": car.endPt}
        )
    return events


def record_congestion(G):
    """Write one row per directed arc, with its own congestion time series."""
    for u, v, d in G.edges(data=True):
        entry = {
            "road": d.get("road_id"),
            "from": u,
            "to": v,
            "lanes": d.get("lanes", 1),
            "congestion_logs": d.get("congestion_logs", []),
        }
        for key in (
            "cars",
            "density",
            "v_mean",
            "speed_ratio",
            "queue",
            "los",
            "congestion",
        ):
            if key in d:
                entry[key] = d[key]
        Congestion.append(entry)


def record_car_logs(cars):
    for c in cars:
        Cars.append(
            {
                "id": c.id,
                "start": c.startPt,
                "goal": c.endPt,
                "spawn_frame": c.spawn_frame,
                "route": c.route,
                "events": _car_events(c),
                "time_cost": c.time_cost,
                "total_distance": c.tot_distance,
                "arrived": c.destination,
                "arrive_frame": c.arrive_frame if c.destination else -1,
            }
        )


def record_params(road, cars, simulator, run, replay=""):
    """Store every parameter used by the run, so the report can be replayed."""
    Params.clear()
    Params.update(
        {
            "road": asdict(road),
            "cars": asdict(cars),
            "simulator": asdict(simulator),
            "run": dict(run),
            "replay": replay,
        }
    )


def _unique_path(path):
    """Avoid overwriting a report/image when two runs end in the same second."""
    if not os.path.exists(path):
        return path
    base, ext = os.path.splitext(path)
    counter = 1
    while os.path.exists(f"{base}-{counter}{ext}"):
        counter += 1
    return f"{base}-{counter}{ext}"


def save_logs(summary=None):
    report = {
        "schema": REPORT_SCHEMA,
        "params": Params,
        "nodes": Nodes,
        "roads": Roads,
        "congestion": Congestion,
        "cars": Cars,
    }
    if summary is not None:
        report["summary"] = summary
    os.makedirs(REPORT_DIR, exist_ok=True)
    path = os.path.join(
        REPORT_DIR,
        f"report-{strftime('%Y%m%d-%H%M%S', localtime(time()))}.json",
    )
    path = _unique_path(path)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, separators=(",", ":"))
    return path


def find_report(name):
    """A report name can be a path, or a filename under logs/reports."""
    candidates = [
        name,
        os.path.join(REPORT_DIR, name),
        os.path.join(REPORT_DIR, name + ".json"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(f"report not found: {name} (tried {candidates})")


def load_report(name):
    with open(find_report(name), encoding="utf-8") as f:
        return json.load(f)


def apply_params(params, cfg_road, cfg_car, cfg_simulation):
    """Overlay the report's parameter blocks on the current config objects."""

    def merge(cfg, stored):
        base = asdict(cfg)
        for key, value in (stored or {}).items():
            if key in base:
                base[key] = value
        return type(cfg)(**base)

    return (
        merge(cfg_road, params.get("road")),
        merge(cfg_car, params.get("cars")),
        merge(cfg_simulation, params.get("simulator")),
    )


def apply_run_params(run, base=None):
    """Return a RunConfig with the contents of a report's ``run`` block."""
    import attributes

    if base is None:
        base = attributes.RunConfig(
            fps=attributes.FPS,
            frames=attributes.FRAMES,
            pic_scale=attributes.PIC_SCALE,
            show_mid_process=attributes.SHOW_MID_PROCESS,
            frames_per_output=attributes.FRAMES_PER_OUTPUT,
            generate_gif=attributes.GENERATE_GIF,
            replay_report=attributes.REPLAY_REPORT,
            seed=attributes.SEED,
            gif_fps=attributes.GIF_FPS,
        )
    data = asdict(base)
    mapping = {
        "FPS": "fps",
        "FRAMES": "frames",
        "PIC_SCALE": "pic_scale",
        "SHOW_MID_PROCESS": "show_mid_process",
        "FRAMES_PER_OUTPUT": "frames_per_output",
        "GENERATE_GIF": "generate_gif",
        "GIF_FPS": "gif_fps",
        "SEED": "seed",
    }
    for key, field in mapping.items():
        if key in (run or {}):
            data[field] = run[key]
        elif field in (run or {}):
            data[field] = run[field]
    return attributes.RunConfig(**data)


def replay_spawn_plan(report):
    """Return {frame: [(id, start, goal), ...]} for replay.

    Supports both the current ``cars`` list with explicit spawn frames and the
    legacy ``car_logs`` list where the first log entry is ("Spawned", frame).
    """
    plan = {}
    cars = report.get("cars") or report.get("car_logs") or []
    for c in cars:
        if c.get("spawn_frame") is not None:
            frame = int(c["spawn_frame"])
        else:
            logs = c.get("logs") or []
            first = logs[0] if logs else None
            frame = int(first[1]) if isinstance(first, (list, tuple)) else 0
        start = c.get("start", c.get("startPt"))
        goal = c.get("goal", c.get("endPt"))
        plan.setdefault(frame, []).append((c.get("id"), start, goal))
    for entries in plan.values():
        entries.sort()
    return plan
