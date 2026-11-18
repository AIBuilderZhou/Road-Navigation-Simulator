"""A* shortest path on the directed road graph (cost = road length)."""
import heapq
import math


def heuristic(pos, a, b):
    """Euclidean distance between two nodes."""
    x1, y1 = pos[a]
    x2, y2 = pos[b]
    return math.hypot(x1 - x2, y1 - y2)


def astar(G, start, goal, pos):
    """Return the node list of a shortest path from start to goal, or None.

    Every directed arc carries its geometric length, so the Euclidean
    heuristic is consistent. Ties are broken by node id, which keeps the
    route (and a replayed run) reproducible.
    """
    open_heap = [(heuristic(pos, start, goal), start)]
    came_from = {}
    g_score = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for neighbor in G.neighbors(current):
            cost = G[current][neighbor].get("length", 1.0)
            tentative_g = g_score[current] + cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                heapq.heappush(
                    open_heap,
                    (tentative_g + heuristic(pos, neighbor, goal), neighbor),
                )

    return None
