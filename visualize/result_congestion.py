"""Draw the final congestion map and the run summary."""
import os
from time import localtime, strftime, time

import matplotlib.pyplot as plt

from visualize.road_draw import draw_network


def _mean_logs(d):
    logs = d.get("congestion_logs") or []
    if not logs:
        return d.get("congestion", 0.0)
    return sum(logs) / len(logs)


def _los_line(label, counts):
    if not counts:
        return f"{label}: no data"
    return f"{label}: " + " ".join(
        f"{grade}{counts.get(grade, 0)}" for grade in "ABCDEF"
    )


def draw(G, summary, run, standard_speed):
    values = {(u, v): _mean_logs(d) for u, v, d in G.edges(data=True)}
    fig, ax = plt.subplots(figsize=(15 * run.pic_scale, 11.25 * run.pic_scale))
    draw_network(fig, ax, G, values, run)

    title_size = 16 * run.pic_scale
    text_size = 12 * run.pic_scale

    mean_by_direction = summary.get("mean_congestion_by_direction", {})
    los_by_direction = summary.get("los_count_by_direction", {})
    route = summary.get("route_ms", {})
    stats = (
        f"Free-flow speed: {standard_speed} km/h\n"
        f"Cars passed: {summary.get('cars_arrived', 0)} / "
        f"{summary.get('cars_total', 0)}\n"
        f"Avg time cost: {summary.get('avg_time_h', 0.0):.4f} h "
        f"({summary.get('avg_time_h', 0.0) * 3600:.1f} s)\n"
        f"Avg speed: {summary.get('avg_speed_kmh', 0.0):.2f} km/h\n"
        f"Network speed: {summary.get('network_speed_kmh', 0.0):.2f} km/h\n"
        f"Mean congestion: {summary.get('mean_congestion', 0.0):.4f} "
        f"(A->B {mean_by_direction.get('AB', 0.0):.4f}, "
        f"B->A {mean_by_direction.get('BA', 0.0):.4f})\n"
        f"{_los_line('LOS A->B', los_by_direction.get('AB'))}\n"
        f"{_los_line('LOS B->A', los_by_direction.get('BA'))}\n"
        f"Route A*: {route.get('calls', 0)} calls, "
        f"{route.get('mean_ms', 0.0):.3f} ms avg, "
        f"{route.get('p95_ms', 0.0):.3f} ms p95"
    )
    ax.set_title("Traffic Simulation Result - Congestion", size=title_size)
    fig.text(0.02, 0.04, stats, fontsize=text_size, ha="left", va="bottom",
             bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "none",
                   "pad": 4})

    os.makedirs("logs/results", exist_ok=True)
    stamp = strftime("%Y%m%d-%H%M%S", localtime(time()))
    path = f"logs/results/congestion-{stamp}.png"
    if os.path.exists(path):
        base, ext = os.path.splitext(path)
        counter = 1
        while os.path.exists(f"{base}-{counter}{ext}"):
            counter += 1
        path = f"{base}-{counter}{ext}"
    fig.savefig(path)
    plt.close(fig)
