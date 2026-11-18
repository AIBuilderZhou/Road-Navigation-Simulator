"""Draw the road network, both directions and the cars of one frame."""
import math
import os
from io import BytesIO

import imageio.v2 as imageio
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import networkx as nx
from matplotlib.transforms import Affine2D


def _right_normal_display(ax, x1, y1, x2, y2):
    """Unit vector pointing to the right of travel, in display pixels."""
    p1 = ax.transData.transform((x1, y1))
    p2 = ax.transData.transform((x2, y2))
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.hypot(dx, dy) or 1.0
    return dy / length, -dx / length


def _offset_px(lanes, dpi):
    """Half the visual line width plus a small gap, in pixels."""
    line_width_px = max(1, int(lanes)) * 2.0 * dpi / 72.0
    return line_width_px / 2.0 + 1.5


def _clamp(value):
    return max(0.0, min(1.0, float(value)))


def draw_network(fig, ax, G, values, run):
    """Draw every directed arc on the right-hand side of its travel direction.

    ``values`` maps (u, v) to the congestion colour in [0, 1]. The physical
    road is drawn twice: one offset line for A->B and one for B->A. No arrows
    are used; the right-hand convention is the direction cue.
    """
    pos = nx.get_node_attributes(G, "pos")
    cmap = cm.RdYlGn_r
    norm = mcolors.Normalize(vmin=0, vmax=1)

    line_items = []
    for u, v, d in G.edges(data=True):
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        line, = ax.plot(
            [x1, x2],
            [y1, y2],
            color=cmap(norm(_clamp(values.get((u, v), 0.0)))),
            linewidth=max(1, int(d.get("lanes", 1))) * 2.0,
            alpha=0.75,
            solid_capstyle="round",
            zorder=2,
        )
        line_items.append((line, u, v, d))

    # Node markers and labels are shared by both directions.
    xs = [pos[n][0] for n in G.nodes]
    ys = [pos[n][1] for n in G.nodes]
    ax.scatter(xs, ys, s=300, c="lightblue", edgecolors="#3d5a73",
               linewidths=0.8, zorder=5)
    for n in G.nodes:
        x, y = pos[n]
        ax.text(x, y, str(n), fontsize=9, ha="center", va="center",
                color="#10202e", zorder=6)

    # One length label per physical road, not once per direction.
    seen_roads = set()
    for u, v, d in G.edges(data=True):
        road_key = d.get("road_id")
        if road_key is None:
            road_key = frozenset((u, v))
        if road_key in seen_roads:
            continue
        seen_roads.add(road_key)
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        ax.text(
            (x1 + x2) / 2.0,
            (y1 + y2) / 2.0,
            f"{d.get('length', 0.0):.1f}",
            fontsize=12,
            ha="center",
            va="center",
            color="#203040",
            bbox={"facecolor": "white", "alpha": 0.7, "edgecolor": "none",
                  "pad": 0.5},
            zorder=4,
        )

    ax.set_axis_off()
    ax.autoscale_view()
    fig.canvas.draw()

    # Offset each arc to the right of its own travel direction. The translation
    # is in display pixels, so the two lines of a road stay parallel at every
    # PIC_SCALE and the gap does not shrink when the figure gets larger.
    for line, u, v, d in line_items:
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        nx_, ny_ = _right_normal_display(ax, x1, y1, x2, y2)
        offset = _offset_px(d.get("lanes", 1), fig.dpi)
        line.set_transform(
            ax.transData + Affine2D().translate(nx_ * offset, ny_ * offset)
        )

    _add_colorbar(fig, cmap, norm)
    return pos


def _add_colorbar(fig, cmap, norm):
    cax = fig.add_axes([0.70, 0.045, 0.24, 0.018])
    sm = cm.ScalarMappable(norm=norm, cmap=cmap)
    colorbar = fig.colorbar(sm, cax=cax, orientation="horizontal")
    colorbar.set_ticks([0.0, 0.5, 1.0])
    colorbar.ax.tick_params(labelsize=9)
    colorbar.set_label("Congestion (0 = free flow, 1 = jam)", fontsize=9)


def _car_xy(fig, ax, G, pos, car):
    """Position of one car, offset to the right of its current arc."""
    if car.destination or len(car.route) < 2:
        return None
    if car.progress < 0 or car.progress + 1 >= len(car.route):
        return None
    u = car.route[car.progress]
    v = car.route[car.progress + 1]
    if u not in pos or v not in pos or not G.has_edge(u, v):
        return None

    t = _clamp(car.distance)
    x = pos[u][0] + t * (pos[v][0] - pos[u][0])
    y = pos[u][1] + t * (pos[v][1] - pos[u][1])

    nx_, ny_ = _right_normal_display(
        ax, pos[u][0], pos[u][1], pos[v][0], pos[v][1]
    )
    offset = _offset_px(G[u][v].get("lanes", 1), fig.dpi)
    px, py = ax.transData.transform((x, y))
    return ax.transData.inverted().transform((px + nx_ * offset,
                                              py + ny_ * offset))


def draw(G, cars, frame, frames, speed, run):
    """Write one mid-process frame (PNG) or append it to the GIF buffer."""
    values = {
        (u, v): d.get("congestion", 0.0)
        for u, v, d in G.edges(data=True)
    }
    fig, ax = plt.subplots(figsize=(15 * run.pic_scale, 11.25 * run.pic_scale))
    pos = draw_network(fig, ax, G, values, run)

    cars_on_road = 0
    cars_passed = 0
    cars_by_direction = {"AB": 0, "BA": 0}
    for c in cars:
        if c.destination:
            cars_passed += 1
            continue
        xy = _car_xy(fig, ax, G, pos, c)
        if xy is None:
            continue
        ax.scatter(xy[0], xy[1], color="red", s=55, zorder=10,
                   edgecolors="white", linewidths=0.4)
        cars_on_road += 1
        u = c.route[c.progress]
        v = c.route[c.progress + 1]
        direction = G[u][v].get("direction", "AB")
        cars_by_direction[direction] += 1

    title_size = 16 * run.pic_scale
    text_size = 13 * run.pic_scale
    ax.set_title("Traffic Simulation", size=title_size)
    stats = (
        f"Free-flow speed: {speed} km/h\n"
        f"Frame: {frame} / {run.frames}\n"
        f"Cars on road: {cars_on_road} "
        f"(A->B {cars_by_direction['AB']}, B->A {cars_by_direction['BA']})\n"
        f"Cars passed: {cars_passed}"
    )
    fig.text(0.02, 0.04, stats, fontsize=text_size, ha="left", va="bottom",
             bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "none",
                   "pad": 4})

    if run.generate_gif:
        buffer = BytesIO()
        fig.savefig(buffer, format="png")
        buffer.seek(0)
        frames.append(imageio.imread(buffer))
    else:
        os.makedirs("logs/frames", exist_ok=True)
        fig.savefig(f"logs/frames/frame{frame}.png")

    plt.close(fig)
