"""Run one traffic simulation and write the report, images and optional GIF."""
import os
import random
from time import localtime, perf_counter, strftime, time as now

import imageio.v2 as imageio
import networkx as nx
from tqdm import tqdm

import algorithm.astar as astar
import attributes
import core.car as car_module
import core.car_simulation as processing
import core.logs_recording as recording
import visualize.result_congestion as result_congestion
import visualize.road_draw as visualize
from attributes import (
    Config_car_generator,
    Config_road_generator,
    Config_simulator,
    RunConfig,
)
from road_generation.Default import default_road
from road_generation.generate import generate_graph


def _run_dict(run):
    """Report form of RunConfig (uppercase keys match the legacy reports)."""
    return {
        "FPS": run.fps,
        "FRAMES": run.frames,
        "PIC_SCALE": run.pic_scale,
        "SHOW_MID_PROCESS": run.show_mid_process,
        "FRAMES_PER_OUTPUT": run.frames_per_output,
        "GENERATE_GIF": run.generate_gif,
        "GIF_FPS": run.gif_fps,
        "SEED": run.seed,
    }


def _p95(values):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(0.95 * (len(ordered) - 1)))))
    return ordered[index]


def run_simulation(cfg_road, cfg_car, cfg_simulation, run):
    recording.reset()

    # ------------------------------------------------------------------ #
    # CREATE GRAPH
    # ------------------------------------------------------------------ #
    if run.replay_report:
        report = recording.load_report(run.replay_report)
        params = report.get("params", {})
        cfg_road, cfg_car, cfg_simulation = recording.apply_params(
            params, cfg_road, cfg_car, cfg_simulation
        )
        run = recording.apply_run_params(params.get("run", {}), run)
        if run.seed is not None:
            random.seed(run.seed)
        node_list, road_list = recording.restore_map(
            report["nodes"], report["roads"]
        )
        spawn_plan = recording.replay_spawn_plan(report)
        print(
            f"[replay] {run.replay_report}: {len(node_list)} nodes, "
            f"{len(road_list)} roads, "
            f"{sum(len(v) for v in spawn_plan.values())} cars, "
            f"FPS={run.fps}, FRAMES={run.frames}"
        )
    else:
        # Seed before road generation so the whole scene is reproducible.
        if run.seed is not None:
            random.seed(run.seed)
        node_list, road_list = default_road(cfg_road)
        spawn_plan = None

    G = generate_graph(node_list, road_list)
    pos = nx.get_node_attributes(G, "pos")
    recording.record_map(node_list, road_list)
    recording.record_params(
        cfg_road, cfg_car, cfg_simulation, _run_dict(run), run.replay_report
    )

    # ------------------------------------------------------------------ #
    # GENERATE CARS
    # ------------------------------------------------------------------ #
    cars = []
    route_times_ms = []
    num_nodes_available = len(node_list)

    def spawn_car(car_id, start, end, frame):
        """Create one car; None endpoints mean a random scene."""
        c = car_module.Car(car_id, num_nodes_available, start, end)
        c.spawn_frame = frame
        route_started = perf_counter()
        c.route = astar.astar(G, c.startPt, c.endPt, pos)
        route_times_ms.append((perf_counter() - route_started) * 1000.0)
        if not c.route or len(c.route) < 2:
            print(
                f"[warn] car {car_id}: no route {c.startPt} -> {c.endPt}, skipped"
            )
            c.destination = True
            c.arrive_frame = frame
        c.logs.append(("Spawned", frame))
        cars.append(c)

    if spawn_plan is None:
        for _ in range(cfg_car.num_cars_initially):
            spawn_car(len(cars) + 1, None, None, 0)
    else:
        # Cars with id <= num_cars_initially and spawn frame 0 belong to the
        # opening fleet; the rest are spawned after the first frame.
        initial = []
        rest = []
        for entry in spawn_plan.get(0, []):
            if entry[0] is not None and entry[0] <= cfg_car.num_cars_initially:
                initial.append(entry)
            else:
                rest.append(entry)
        spawn_plan[0] = rest
        for car_id, start, end in initial:
            spawn_car(car_id, start, end, 0)

    # ------------------------------------------------------------------ #
    # PROCESSING
    # ------------------------------------------------------------------ #
    frames = []
    for t in tqdm(range(run.frames + 1)):
        processing.whole_process(
            G, cars, cfg_car, cfg_simulation, t, fps=run.fps
        )
        if (
            run.show_mid_process
            and run.frames_per_output > 0
            and t % run.frames_per_output == 0
        ):
            visualize.draw(
                G, cars, t, frames, cfg_simulation.standard_speed, run
            )
        if spawn_plan is None:
            if random.random() < cfg_car.new_car_prob:
                low, high = cfg_car.new_car_range
                for _ in range(random.randint(low, high)):
                    spawn_car(len(cars) + 1, None, None, t)
        else:
            for car_id, start, end in spawn_plan.get(t, []):
                spawn_car(car_id, start, end, t)

    # ------------------------------------------------------------------ #
    # CALCULATE RESULTS
    # ------------------------------------------------------------------ #
    recording.record_congestion(G)
    recording.record_car_logs(cars)

    passed = 0
    total_time = 0.0
    total_avg_speed = 0.0
    for c in cars:
        if c.destination:
            passed += 1
            total_time += c.time_cost
            total_avg_speed += (
                c.tot_distance / c.time_cost if c.time_cost else 0.0
            )
    avg_time = total_time / passed if passed else 0.0
    avg_speed = total_avg_speed / passed if passed else 0.0

    edges = list(G.edges(data=True))
    total_cars_now = sum(d["cars"] for _, _, d in edges)
    if total_cars_now:
        net_speed = (
            sum(d["v_mean"] * d["cars"] for _, _, d in edges) / total_cars_now
        )
    else:
        total_length = sum(d["length"] for _, _, d in edges)
        net_speed = (
            sum(d["v_mean"] * d["length"] for _, _, d in edges) / total_length
            if total_length
            else 0.0
        )

    congestion_series = [
        sum(d["congestion_logs"]) / len(d["congestion_logs"])
        for _, _, d in edges
        if d["congestion_logs"]
    ]
    mean_congestion = (
        sum(congestion_series) / len(congestion_series)
        if congestion_series
        else 0.0
    )

    los_count = {grade: 0 for grade in "ABCDEF"}
    los_by_direction = {
        "AB": {grade: 0 for grade in "ABCDEF"},
        "BA": {grade: 0 for grade in "ABCDEF"},
    }
    congestion_by_direction = {"AB": [], "BA": []}
    for _, _, d in edges:
        grade = d.get("los", "A")
        los_count[grade] = los_count.get(grade, 0) + 1
        direction = d.get("direction", "AB")
        los_by_direction[direction][grade] = (
            los_by_direction[direction].get(grade, 0) + 1
        )
        if d["congestion_logs"]:
            congestion_by_direction[direction].append(
                sum(d["congestion_logs"]) / len(d["congestion_logs"])
            )

    mean_congestion_by_direction = {
        direction: (sum(values) / len(values) if values else 0.0)
        for direction, values in congestion_by_direction.items()
    }
    route_summary = {
        "calls": len(route_times_ms),
        "total_ms": sum(route_times_ms),
        "mean_ms": (
            sum(route_times_ms) / len(route_times_ms) if route_times_ms else 0.0
        ),
        "p95_ms": _p95(route_times_ms),
        "max_ms": max(route_times_ms) if route_times_ms else 0.0,
    }
    summary = {
        "cars_total": len(cars),
        "cars_arrived": passed,
        "cars_incomplete": len(cars) - passed,
        "avg_time_h": avg_time,
        "avg_speed_kmh": avg_speed,
        "network_speed_kmh": net_speed,
        "mean_congestion": mean_congestion,
        "mean_congestion_by_direction": mean_congestion_by_direction,
        "los_count": los_count,
        "los_count_by_direction": los_by_direction,
        "route_ms": route_summary,
    }

    # ------------------------------------------------------------------ #
    # REPORTS, RESULT IMAGES AND GIF
    # ------------------------------------------------------------------ #
    report_path = recording.save_logs(summary)
    result_congestion.draw(G, summary, run, cfg_simulation.standard_speed)
    if run.generate_gif:
        if frames:
            os.makedirs("logs/gif", exist_ok=True)
            stamp = strftime("%Y%m%d-%H%M%S", localtime(now()))
            gif_path = f"logs/gif/traffic-{stamp}.gif"
            if os.path.exists(gif_path):
                base, ext = os.path.splitext(gif_path)
                counter = 1
                while os.path.exists(f"{base}-{counter}{ext}"):
                    counter += 1
                gif_path = f"{base}-{counter}{ext}"
            imageio.mimsave(gif_path, frames, fps=run.gif_fps)
        else:
            print(
                "[warn] GENERATE_GIF is on but no frames were drawn; "
                "set SHOW_MID_PROCESS=True"
            )

    print(
        f"[done] cars {passed}/{len(cars)} arrived, "
        f"avg time {avg_time * 3600:.1f}s, avg speed {avg_speed:.1f} km/h, "
        f"network speed {net_speed:.1f} km/h, "
        f"mean congestion {mean_congestion:.3f}"
    )
    print(f"[done] report: {report_path}")
    return summary, report_path


def main():
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
        angle_variation=0.1,
    )
    cfg_car = Config_car_generator(
        # 200 cars is a normal load for this 100-node network.
        num_cars_initially=200,
        # Probability of a spawn attempt per frame (frame-rate dependent).
        new_car_prob=1 / 60,
        new_car_range=[1, 3],
    )
    cfg_simulation = Config_simulator(
        standard_speed=80,
        # One congestion sample per simulated second (12 frames at FPS=43200).
        log_every=12,
    )
    run = RunConfig(
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
    run_simulation(cfg_road, cfg_car, cfg_simulation, run)


if __name__ == "__main__":
    main()
