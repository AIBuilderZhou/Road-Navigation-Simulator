"""Turn the generator's Node/Road lists into a directed NetworkX graph.

A physical road (ptA, ptB) becomes two independent arcs:
    ptA -> ptB   and   ptB -> ptA
Both arcs share the same length and lane count, but every statistic (cars,
density, speed, queue, level of service, congestion log) belongs to one arc
only. This is what makes the two directions independent.
"""
import networkx as nx


def generate_graph(nodes, roads):
    G = nx.DiGraph()

    for n in nodes:
        # x/y are the real coordinates, tiled_pos is the grid cell of the node.
        G.add_node(
            n.id,
            pos=(n.pos[0], n.pos[1]),
            x=n.pos[0],
            y=n.pos[1],
            tiled_pos=(n.tiled_pos[0], n.tiled_pos[1]),
            degrees=n.degrees,
            connection=tuple(n.connection),
        )

    for road_id, r in enumerate(roads):
        for u, v in ((r.ptA, r.ptB), (r.ptB, r.ptA)):
            # length is the real geometric distance, so the A* heuristic stays
            # consistent. "direction" only labels which half of the road this is.
            G.add_edge(
                u,
                v,
                road_id=road_id,
                direction="AB" if u == r.ptA else "BA",
                weight=1 / r.length,
                length=r.length,
                lanes=r.lanes,
                capacity=r.length * r.lanes,
                congestion=0.0,
                cars=0,
                queue=0,
                congestion_logs=[],
            )

    return G
