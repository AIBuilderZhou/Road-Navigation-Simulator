"""Run the road network with the UXsim mesoscopic simulator.

The project still owns the network generation, the A* route of every car and
the report format.  This module is only the execution engine: it builds a
UXsim World from the directed NetworkX graph, forces every car onto its
pre-computed route and translates UXsim's vehicle/link state back into the
existing ``Car`` objects and schema-2 congestion data.

Design notes
------------
* UXsim advances in discrete steps.  A one-second step is used because the
  default report samples once per simulated second (``log_every=12`` at
  ``FPS=43200``); it is also the coarsest step that keeps the animation close
  to the report frames.  Finer frame indices are obtained by sampling the
  last completed one-second state.
* UXsim is a link-based model: it has no per-lane IDM, no explicit lane
  changing and no traffic signals unless they are configured.  The legacy
  IDM fields are kept in the configuration/report for compatibility but are
  not used by this engine.
* All vehicles must be known before ``exec_simulation``.  ``main`` therefore
  materialises the random spawn plan first, exactly like a replay plan.
"""
import math
import os
from time import localtime, perf_counter, strftime, time as now

import uxsim

UXSIM_STEP_SECONDS = 1  # keep this an int: UXsim's native animator uses range(step)
PROGRESS_INTERVAL_SECONDS = 30.0

# Level of service by density, vehicles per km per lane (urban road, HCM-like).
LOS_LIMITS = (7.0, 11.0, 16.0, 22.0, 28.0)


def _los(density):
    for grade, limit in zip("ABCDE", LOS_LIMITS):
        if density < limit:
            return grade
    return "F"


def _free_speed_kmh(cfg, max_lanes, lanes):
    return max(
        cfg.minimum_speed,
        cfg.standard_speed - cfg.lane_speed_step * (max_lanes - lanes),
    )


def _edge_frame(frame, frames_per_step):
    """Convert a frame index to the nearest completed simulation step."""
    if frames_per_step <= 0:
        return 0
    return int(round(frame / frames_per_step))


def _step_frame(step, frames_per_step):
    return int(round(step * frames_per_step))


def _quantize_departure(frame, frames_per_step):
    """Return (step index, actual frame) without departing before the plan."""
    step = int(math.ceil(max(0.0, float(frame)) / frames_per_step - 1e-9))
    return step, _step_frame(step, frames_per_step)


def _initialise_edges(G, cfg, max_lanes):
    for u, v, d in G.edges(data=True):
        lanes = max(1, int(d.get("lanes", 1)))
        length_m = max(1e-6, d["length"] * cfg.meters_per_unit)
        free = _free_speed_kmh(cfg, max_lanes, lanes)
        d.setdefault("cars", 0)
        d["density"] = 0.0
        d["v_mean"] = free
        d["speed_ratio"] = 1.0
        d["queue"] = 0
        d["congestion"] = 0.0
        d["los"] = "A"
        d["congestion_logs"] = []
        d["_uxsim_length_m"] = length_m
        d["_uxsim_free_speed"] = free


def _build_world(G, cfg, run, step_seconds):
    frames_per_step = run.fps * step_seconds / 3600.0
    last_step = max(0, int(math.ceil(run.frames / frames_per_step - 1e-9)))
    tmax = max(1, int(round((last_step + 1) * step_seconds)))
    world = uxsim.World(
        name="ipq-road-navigation",
        deltan=1,
        reaction_time=step_seconds,
        tmax=tmax,
        random_seed=run.seed,
        print_mode=0,
        save_mode=0,
        show_progress=0,
        vehicle_logging_timestep_interval=-1,
        #cpp=True,
        threads=5,
    )

    for n, data in G.nodes(data=True):
        world.addNode(
            str(n),
            float(data.get("x", data.get("pos", (0.0, 0.0))[0])),
            float(data.get("y", data.get("pos", (0.0, 0.0))[1])),
        )

    max_lanes = max(
        (max(1, int(d.get("lanes", 1))) for _, _, d in G.edges(data=True)),
        default=1,
    )
    link_to_edge = {}
    edge_to_link = {}
    for u, v, d in G.edges(data=True):
        lanes = max(1, int(d.get("lanes", 1)))
        length_m = max(1e-6, d["length"] * cfg.meters_per_unit)
        free = _free_speed_kmh(cfg, max_lanes, lanes)
        name = f"{u}->{v}"
        link = world.addLink(
            name,
            str(u),
            str(v),
            length_m,
            free_flow_speed=free / 3.6,
            jam_density_per_lane=max(1e-9, cfg.jam_density / 1000.0),
            number_of_lanes=lanes,
        )
        link_to_edge[name] = (u, v)
        edge_to_link[(u, v)] = link
        d["_uxsim_link"] = link
    return world, link_to_edge, edge_to_link, frames_per_step


def _stats_for_link(link, cfg):
    """Translate one UXsim link snapshot into the legacy metric vocabulary."""
    lanes = max(1, int(link.number_of_lanes))
    length_km = max(1e-9, link.length / 1000.0)
    cars = int(round(link.num_vehicles))
    density = cars / (lanes * length_km) if length_km > 0 else 0.0
    stop_speed = cfg.minimum_speed / 3.6
    queue = sum(1 for veh in link.vehicles if veh.v <= stop_speed)
    v_mean = link.speed * 3.6
    free = link.u * 3.6
    speed_ratio = min(1.0, v_mean / free) if free else 1.0
    # A single car that just entered a link can have v=0 (or a very low
    # average speed) without any queue, and one car on a short link also has
    # a misleadingly high density. Do not call either case congestion. A
    # stopped single vehicle still counts through the queue/speed term.
    if queue == 0 and cars <= 1:
        congestion = 0.0
    else:
        speed_term = max(0.0, 1.0 - speed_ratio) if queue > 0 else 0.0
        density_term = density / cfg.jam_density if cars >= 2 else 0.0
        congestion = min(1.0, max(0.0, max(speed_term, density_term)))
    return {
        "cars": cars,
        "density": density,
        "v_mean": v_mean,
        "speed_ratio": speed_ratio,
        "queue": queue,
        "congestion": congestion,
        "los": _los(density),
    }


def _write_stats(G, edge, stats):
    u, v = edge
    d = G[u][v]
    for key, value in stats.items():
        d[key] = value


def _resample_logs(series, frames, frames_per_step, log_every):
    if not series:
        return []
    result = []
    for frame in range(0, frames + 1, max(1, int(log_every))):
        step = min(len(series) - 1, max(0, _edge_frame(frame, frames_per_step)))
        result.append(series[step])
    return result


def _write_uxsim_gif(world, run):
    """Render UXsim's native network animation after the run.

    The native animation uses UXsim's link state directly, so it works even
    when the project's own frame collector is disabled.  It is intentionally
    opt-in: a full 24 h template with a small skip can create thousands of
    frames and a large GIF.
    """
    skip = max(1, int(run.gif_timestep_skip))
    # network_anim samples t = 0, skip, 2*skip, ... < TMAX.
    total = int((max(1, int(world.TMAX)) - 1) // max(1, int(world.DELTAT) * skip)) + 1
    if total <= 0:
        print("[warn] UXsim native GIF skipped: no simulation frames")
        return None
    os.makedirs("logs/gif", exist_ok=True)
    stamp = strftime("%Y%m%d-%H%M%S", localtime(now()))
    path = os.path.join("logs", "gif", f"uxsim-{stamp}.gif")
    if os.path.exists(path):
        base, ext = os.path.splitext(path)
        counter = 1
        while os.path.exists(f"{base}-{counter}{ext}"):
            counter += 1
        path = f"{base}-{counter}{ext}"
    # network_anim's duration is animation_speed_inverse * timestep_skip ms.
    duration = max(1, int(round(1000.0 / (max(1, int(run.gif_fps)) * skip))))
    print(
        f"[uxsim] native GIF: {total} frames, skip={skip}s, "
        f"figsize=6x6, writing {path}",
        flush=True,
    )
    try:
        uxsim.Analyzer(world).network_anim(
            animation_speed_inverse=duration,
            detailed=0,
            state_variables="density_speed",
            minwidth=0.5,
            maxwidth=12,
            left_handed=0,  # right-hand traffic, matching the project drawing
            figsize=(6, 6),
            node_size=2,
            network_font_size=4,
            timestep_skip=skip,
            file_name=path,
            legend=True,
        )
    except Exception as exc:  # keep the simulation report even if GIF fails
        print(f"[warn] UXsim native GIF failed: {exc}", flush=True)
        return None
    if not os.path.exists(path):
        print("[warn] UXsim native GIF was not written", flush=True)
        return None
    print(f"[uxsim] native GIF written: {path}", flush=True)
    return path


def _events_for_car(car, vehicle, frames_per_step, fps, link_to_edge, max_frame):
    events = [
        {"frame": int(car.spawn_frame), "type": "spawn", "node": car.startPt}
    ]
    link_entries = [
        (seconds, link)
        for seconds, link in vehicle.log_t_link
        if hasattr(link, "name")
    ]
    # Keep the original report convention: the first arc is implied by
    # route[0] -> route[1], so "enter" starts with the second arc.
    for seconds, link in link_entries[1:]:
        edge = link_to_edge.get(link.name)
        if edge is None:
            continue
        frame = min(max_frame, max(0, int(round(seconds * fps / 3600.0))))
        events.append(
            {
                "frame": frame,
                "type": "enter",
                "from": edge[0],
                "to": edge[1],
            }
        )
    if car.destination:
        events.append(
            {
                "frame": int(car.arrive_frame),
                "type": "arrive",
                "node": car.endPt,
            }
        )
    return events


def run_simulation(G, cars, cfg_simulator, run):
    """Execute every car through UXsim and return drawing frames.

    The function mutates ``G`` (link metrics and congestion logs) and every
    routed ``Car`` (arrival, travel time, distance and explicit events).
    """
    _initialise_edges(G, cfg_simulator, max(
        (max(1, int(d.get("lanes", 1))) for _, _, d in G.edges(data=True)),
        default=1,
    ))
    step_seconds = UXSIM_STEP_SECONDS
    world, link_to_edge, edge_to_link, frames_per_step = _build_world(
        G, cfg_simulator, run, step_seconds
    )
    series = {edge: [] for edge in edge_to_link}
    frames = []
    next_draw = 0
    draw_custom_frames = not (
        run.generate_gif
        and str(getattr(run, "gif_engine", "auto")).lower() == "uxsim"
    )
    total_steps = max(
        1, int(math.ceil(run.frames / frames_per_step - 1e-9)) + 1
    )
    started = perf_counter()
    exec_started = 0.0
    next_progress = 0.0

    def capture(current_world):
        nonlocal next_draw, next_progress
        now = perf_counter()
        if exec_started and now >= next_progress:
            done = min(total_steps, max(0, int(current_world.T)))
            elapsed = max(1e-9, now - exec_started)
            rate = done / elapsed if done else 0.0
            eta = (total_steps - done) / rate if rate > 0 else 0.0
            print(
                f"[uxsim] {done}/{total_steps} steps "
                f"({100.0 * done / total_steps:.1f}%), "
                f"{elapsed:5.0f}s elapsed, ETA ~{eta / 60.0:5.1f} min",
                flush=True,
            )
            next_progress = now + PROGRESS_INTERVAL_SECONDS
        frame = min(run.frames, max(0, _step_frame(current_world.T, frames_per_step)))
        for edge, link in edge_to_link.items():
            stats = _stats_for_link(link, cfg_simulator)
            _write_stats(G, edge, stats)
            series[edge].append(stats["congestion"])
        if (
            draw_custom_frames
            and run.show_mid_process
            and run.frames_per_output > 0
            and frame >= next_draw
        ):
            from visualize.road_draw import draw_uxsim

            draw_uxsim(
                G,
                current_world,
                frame,
                frames,
                cfg_simulator.standard_speed,
                run,
                link_to_edge,
            )
            next_draw = frame + max(1, run.frames_per_output)

    world.user_function = capture

    routed = 0
    for car in cars:
        route = car.route or []
        if len(route) < 2:
            continue
        links = []
        valid = True
        for u, v in zip(route, route[1:]):
            link = edge_to_link.get((u, v))
            if link is None:
                valid = False
                break
            links.append(link)
        if not valid:
            continue
        step, actual_frame = _quantize_departure(
            car.spawn_frame, frames_per_step
        )
        car.spawn_frame = actual_frame
        vehicle = world.addVehicle(
            str(car.startPt),
            str(car.endPt),
            step,
            name=str(car.id),
            departure_time_is_time_step=1,
        )
        vehicle.enforce_route(links)
        routed += 1

    if routed:
        print(
            f"[uxsim] starting: {routed} routed cars, {total_steps} steps "
            f"at {step_seconds:g}s; progress every "
            f"{PROGRESS_INTERVAL_SECONDS:g}s",
            flush=True,
        )
        exec_started = perf_counter()
        next_progress = exec_started + PROGRESS_INTERVAL_SECONDS
        world.exec_simulation()
    else:
        print("[uxsim] no routed cars; simulation skipped")

    for edge in edge_to_link:
        d = G[edge[0]][edge[1]]
        d["congestion_logs"] = _resample_logs(
            series[edge], run.frames, frames_per_step, cfg_simulator.log_every
        )

    for car in cars:
        vehicle = world.VEHICLES.get(str(car.id))
        if vehicle is None:
            continue
        car.destination = vehicle.state == "end"
        if car.destination:
            end_event = next(
                (entry for entry in reversed(vehicle.log_t_link)
                 if len(entry) > 1 and entry[1] == "end"),
                None,
            )
            if end_event is not None:
                car.arrive_frame = min(
                    run.frames,
                    max(0, int(round(end_event[0] * run.fps / 3600.0))),
                )
            elif vehicle.arrival_time >= 0:
                car.arrive_frame = min(
                    run.frames,
                    _step_frame(vehicle.arrival_time, frames_per_step),
                )
            else:
                car.arrive_frame = run.frames
        else:
            car.arrive_frame = -1

        if vehicle.travel_time >= 0:
            car.time_cost = vehicle.travel_time / 3600.0
        else:
            # Keep the same units as the report; incomplete trips are not
            # included in the average travel time by main.
            car.time_cost = max(
                0.0, (run.frames - car.spawn_frame) / run.fps
            )
        car.tot_distance = max(0.0, vehicle.distance_traveled) / 1000.0
        car.events = _events_for_car(
            car,
            vehicle,
            frames_per_step,
            run.fps,
            link_to_edge,
            run.frames,
        )

    elapsed = perf_counter() - started
    print(
        f"[uxsim] {routed} routed cars, step={step_seconds}s, "
        f"{world.T} steps, {elapsed:.2f}s"
    )

    gif_path = None
    engine = str(getattr(run, "gif_engine", "auto")).lower()
    if run.generate_gif and routed:
        if engine == "uxsim" or (engine == "auto" and not frames):
            gif_path = _write_uxsim_gif(world, run)

    return {
        "frames": frames,
        "step_seconds": step_seconds,
        "routed_cars": routed,
        "elapsed_s": elapsed,
        "gif_path": gif_path,
    }
