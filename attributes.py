"""All tunable parameters of the project live here.

Edit the config block in main.py for one run; edit this file when you want a
different default for every run. Every saved report stores a copy of these
values, so a past run can be replayed exactly.

Some fields are kept only for old reports/configs. They are marked ``legacy``
below and are still written to reports/templates, but the current UXsim path
does not read them. The template editor shows active fields first and puts
those legacy fields into a collapsed block at the bottom of the parameter page.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class Config_road_generator:
    # ---- Active fields -------------------------------------------------
    # Number of nodes in the map.
    num_nodes: int
    # Maximum lanes of a road. Every direction gets this many lanes, so a
    # 3-lane road carries 3 lanes from A to B and 3 lanes from B to A.
    max_lanes: int
    # Expected average road length, in generator length units.
    exp_road_length: float
    # Relative variation of the road length, in [0, 1).
    var_road_length: float
    # Angular jitter of a new road in radians; the base angle is a grid direction.
    angle_variation: float

    # ---- Legacy road fields (old undirected generator) -----------------
    # Legacy field: only core/road.py reads it. The Default generator uses the
    # four grid directions and therefore caps the degree at 4.
    max_degree: int
    # Legacy field: only core/road.py reads it. The Default generator ignores it.
    edge_prob: float
    # Legacy field: only core/road.py reads it. The Default generator ignores it.
    force_single_road: list
    # Legacy field: only core/road.py reads it. The Default generator limits
    # extra roads by the distance derived from exp_road_length/var_road_length.
    connection_limit: int
    # Legacy field: only core/road.py reads it. The Default generator never
    # closes a ring.
    ring: bool


@dataclass
class Config_car_generator:
    # Number of cars created before the first simulation frame.
    num_cars_initially: int
    # Probability of one spawn attempt per frame. This is frame-rate dependent:
    # the expected number of attempts per simulated hour is new_car_prob * FPS.
    new_car_prob: float
    # Number of cars created by a successful spawn attempt.
    new_car_range: list[int]


@dataclass
class Config_simulator:
    # ---- Active fields -------------------------------------------------
    # Free-flow speed of the widest road, in km/h. Narrower roads lose
    # lane_speed_step km/h for each missing lane.
    standard_speed: int
    # Lowest speed used by the queue metric, in km/h.
    minimum_speed: float = 5
    # Metres represented by one generator length unit.
    meters_per_unit: float = 10.0
    # Vehicles per km per lane at standstill. The adapter converts it to
    # UXsim's jam_density_per_lane (veh/m/lane).
    jam_density: float = 130.0
    # Free-flow speed lost per missing lane, in km/h.
    lane_speed_step: float = 10.0
    # Write one congestion sample every N frames. At the default FPS=43200,
    # 12 frames = one simulated second. UXsim runs at 1-second steps, so
    # values below 12 repeat the same 1-second snapshot.
    log_every: int = 12

    # ---- Legacy simulator fields (old IDM/density model) ---------------
    # Legacy vehicle length in km. UXsim's link model does not read it.
    car_length: float = 0.005
    # Legacy standstill gap in km. UXsim's link model does not read it.
    minimum_distance: float = 0.002
    # Legacy reaction time in hours (old IDM/density code). The UXsim adapter
    # uses its own fixed one-second step.
    reaction_time: float = 1.5 / 3600
    # Legacy IDM parameters, kept so old reports/configs still load. UXsim's
    # link model does not use them.
    idm_accel: float = 1.5       # maximum acceleration [m/s^2]
    idm_decel: float = 2.0       # comfortable deceleration [m/s^2]
    idm_time_gap: float = 1.2    # desired time headway [s]
    idm_min_gap: float = 2.0     # standstill gap [m]
    idm_delta: float = 4.0       # free-road exponent


@dataclass
class RunConfig:
    """Settings that control one run, separate from the physical model."""
    fps: float
    frames: int
    pic_scale: float
    show_mid_process: bool
    frames_per_output: int
    generate_gif: bool
    replay_report: str
    seed: Optional[int]
    gif_fps: int = 24
    # "auto": custom frames if SHOW_MID_PROCESS produced any, otherwise
    # UXsim's native animation; "custom": only the project renderer;
    # "uxsim": always UXsim's native animation.
    gif_engine: str = "auto"
    # UXsim samples one native GIF frame every N simulation steps (1 s each).
    gif_timestep_skip: int = 24


# Metadata for the editor/documentation. These fields remain in the report
# schema, but the current Default generator / UXsim adapter does not read them.
LEGACY_ROAD_FIELDS = (
    "max_degree",
    "edge_prob",
    "force_single_road",
    "connection_limit",
    "ring",
)
LEGACY_SIMULATOR_FIELDS = (
    "car_length",
    "minimum_distance",
    "reaction_time",
    "idm_accel",
    "idm_decel",
    "idm_time_gap",
    "idm_min_gap",
    "idm_delta",
)


# ---- Run defaults ------------------------------------------------------
# Edit the config block in main.py for a single run; edit these values only
# when you want a different default for every run.
FPS = 24 * 1800            # frames per simulated hour -> 1/12 s per frame
FRAMES = FPS * 2           # simulated frames (2 hours with the default FPS)
PIC_SCALE = 2              # output image scale
SHOW_MID_PROCESS = False   # write frames while the simulation is running
FRAMES_PER_OUTPUT = 30     # write one frame every N simulation frames
GENERATE_GIF = False       # write a GIF at the end
GIF_FPS = 24               # playback rate of the GIF
GIF_ENGINE = "auto"        # auto / custom / uxsim (see RunConfig)
GIF_TIMESTEP_SKIP = 24     # UXsim native GIF: one frame every N 1-second steps
SEED = None                # None = random scene; int = reproducible scene

# Replay a past report instead of generating a new scene.
# "" = generate a new road network and cars from the parameters above.
# "report-YYYYMMDD-HHMMSS.json" (or any readable path) = replay that report:
# the road network, the car spawn plan and every running parameter are read from it.
REPLAY_REPORT = "template-20261119-135812"
