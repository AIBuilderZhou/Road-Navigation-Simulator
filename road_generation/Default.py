"""Tile-based road generation that roughly follows an urban street pattern.

The generator starts at node 0 and grows a spanning tree on a square grid:
every new node is attached to an existing node through one of the four free
grid directions. Extra roads ("upgrades") are then added between nearby nodes
to create three- and four-way junctions.

The module comment below documents the design target distribution. The actual
distribution depends on the random growth and is usually not exactly the
target; the loop that tries to reach it can stop early when a degree pool is
empty. See docs/02-road-generation.md for the measured behaviour.

Fields of Config_road_generator that this generator reads:
    num_nodes, max_lanes, exp_road_length, var_road_length, angle_variation
Fields that only the legacy core/road.py reads:
    max_degree, edge_prob, force_single_road, connection_limit, ring
"""
import math
import random

from road_generation.classes import Node, Road

# Grid directions as (x, y) vectors, in connection-slot order:
# slot 0 = right, slot 1 = up, slot 2 = left, slot 3 = down.
RIGHT = (1, 0)
UP = (0, 1)
LEFT = (-1, 0)
DOWN = (0, -1)
INDEX_TO_DIR = [RIGHT, UP, LEFT, DOWN]
# The old code used x + 2*y as a key; keep the same mapping for compatibility.
DIRECTIONS = {v[0] + v[1] * 2: i for i, v in enumerate(INDEX_TO_DIR)}


def default_road(road_cfg):
    """Return (nodes, roads) for one randomly generated map.

    The design target is about 18% one-way, 62% three-way and 20% four-way
    junctions, but the realised distribution can differ (see module docstring).
    """
    # Step 1: expected number of nodes for each degree class.
    exp = [
        int(road_cfg.num_nodes * 0.18),
        int(road_cfg.num_nodes * 0.62),
        int(road_cfg.num_nodes * 0.2),
    ]
    origin = Node(0, [0, 0], [0, 0], [None, None, None, None])
    num = 1
    occupied_pos = [[0, 0]]
    nodes = [origin]
    roads = []
    # Kept as a diagnostic bucket from the original generator. It is filled
    # once below, but the upgrade loop below uses live degree counts instead.
    nodes_degrees = [[], [], [], [], []]

    def new_road(a: Node, b: Node):
        lanes = random.randint(1, road_cfg.max_lanes)
        roads.append(Road(a.id, b.id, math.dist(a.pos, b.pos), lanes))
        a.degrees += 1
        b.degrees += 1

    def has_edge(a: Node, b: Node):
        """Whether a road between a and b already exists."""
        return any({r.ptA, r.ptB} == {a.id, b.id} for r in roads)

    def new_node(node_id: int, father: Node, direction):
        nonlocal num

        # Mark the father's connection slot as connected.
        direction_index = DIRECTIONS[direction[0] + direction[1] * 2]
        father.connection[direction_index] = node_id

        # Position = father + grid direction rotated by a small random angle.
        angle = math.pi * (direction_index / 2) + (
            random.random() * road_cfg.angle_variation * 2
            - road_cfg.angle_variation
        )
        distance = road_cfg.exp_road_length * (
            1 + random.random() * road_cfg.var_road_length * 2
            - road_cfg.var_road_length
        )
        x = father.pos[0] + math.cos(angle) * distance
        y = father.pos[1] + math.sin(angle) * distance
        tx = father.tiled_pos[0] + direction[0]
        ty = father.tiled_pos[1] + direction[1]

        # The new node's opposite slot points back to its father.
        conn = [None, None, None, None]
        conn[(direction_index + 2) % 4] = father.id

        node = Node(node_id, [x, y], [tx, ty], conn, 0)
        nodes.append(node)
        new_road(father, node)

        occupied_pos.append([tx, ty])
        num += 1

    # Step 2: grow the spanning tree.
    for _ in range(1, road_cfg.num_nodes):
        while True:
            father = random.choice(nodes)
            found = False
            direction = random.randint(0, 3)
            for _ in range(4):
                if father.connection[direction] is None:
                    next_cell = [
                        father.tiled_pos[0] + INDEX_TO_DIR[direction][0],
                        father.tiled_pos[1] + INDEX_TO_DIR[direction][1],
                    ]
                    if next_cell not in occupied_pos:
                        found = True
                        break
                direction = (direction + 1) % 4
            if found:
                break
        new_node(num, father, INDEX_TO_DIR[direction])

    # Step 3: count the initial degree distribution.
    for node in nodes:
        nodes_degrees[node.degrees].append(node)

    max_d = road_cfg.exp_road_length * (1 + road_cfg.var_road_length)
    targets = {1: exp[0], 3: exp[1], 4: exp[2]}

    # Step 4: add roads until each target count is reached, or no candidate
    # is left. This loop cannot create degree-1 nodes, so the first target is
    # often already satisfied (or unreachable) and is skipped.
    for deg, target in targets.items():
        while sum(1 for n in nodes if n.degrees == deg) < target:
            pool = [n for n in nodes if n.degrees == deg - 1]
            if not pool:
                break
            a = random.choice(pool)
            candidates = [
                b
                for b in nodes
                if b is not a
                and b.degrees < 4
                and math.dist(a.pos, b.pos) <= max_d
                and not has_edge(a, b)
            ]
            if not candidates:
                break
            # These extra roads are not aligned to the tile grid, so they are
            # not written into node.connection; they only live in the roads list.
            new_road(a, random.choice(candidates))

    return nodes, roads
