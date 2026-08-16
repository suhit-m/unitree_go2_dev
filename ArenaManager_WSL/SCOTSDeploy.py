"""
SCOTSDeploy.py

Replaces the pFaces half of AutoDeploy.py with a local SCOTS pipeline.

Pipeline
--------
  1. read the GUI-placed geometry out of ObjectManager (the GUI is the
     authoritative source for what the user drew)
  2. optionally merge in physically tracked obstacles from OptiTrack
  3. convert Targets and Obstacles into a SCOTS arena_config.txt
  4. run `make && ./go2_controller <config>` in WSL -> go2_controller.bdd
  5. launch the Go2 closed-loop script inside WSL

Where the geometry comes from
-----------------------------
Synthesis geometry and runtime state are two different concerns and they
come from two different places:

  ObjectManager      what the user drew: virtual Targets and Obstacles.
                     Synchronous, in-process, correct the instant a drag
                     ends. This is what synthesis reads.

  OptiTrack server   live poses of physically tracked rigid bodies. Needed
                     at RUNTIME so the deployment loop can query the BDD
                     with the robot's actual state, and needed for
                     rendering. It also happens to echo back the GUI-placed
                     objects, but reading them from there means a network
                     round trip to learn something the GUI already knows,
                     with a race against the POST landing.


Everything after step 2 runs in WSL because CUDD, SCOTS, the pybind11
binding (SymbolicSetPython.so) and unitree_sdk2py all live there.

The GUI process itself stays on Windows and talks to WSL two ways:
  - file IO through the \\\\wsl.localhost\\<distro>\\... UNC path
  - process launch through wsl.exe

Usage from AutoDeploy.py
------------------------
    import SCOTSDeploy

    self.thread_scots = SCOTSDeploy.SynthesisThread(
        geometry_provider=lambda: SCOTSDeploy.collect_arena(
            self.object_manager, get_objects()),
        state_lb=x_lb, state_ub=x_ub)
    self.thread_scots.log.connect(lambda s: self.status.showMessage(s))
    self.thread_scots.finished_ok.connect(self.on_synthesis_done)
    self.thread_scots.start()
"""

import os
import json
import math
import subprocess

try:
    from PyQt5.QtCore import QThread, pyqtSignal
    _HAVE_QT = True
except ImportError:                                   # allow headless testing
    _HAVE_QT = False
    class QThread(object):
        def start(self): self.run()
    def pyqtSignal(*a, **k): return None


# ===========================================================================
# CONFIGURATION -- edit these to match your machine
# ===========================================================================

WSL_DISTRO = "Ubuntu-20.04"

# where go2_controller.cc / Makefile / the .bdd outputs live, as seen from WSL
SCOTS_WSL_DIR = "/home/cublab/Workspace-Linux/unitree_go2_dev/go2_dev/scots_dev"
CONFIG_DIR    = SCOTS_WSL_DIR

# the same directory as seen from Windows
SCOTS_WIN_DIR = (r"\\wsl.localhost\Ubuntu-20.04\home\cublab"
                 r"\Workspace-Linux\unitree_go2_dev\go2_dev\scots_dev")

# the synthesis binary produced by `make` in SCOTS_WSL_DIR.
# The Makefile must have  TARGET = go2_controller  and a  %.o:%.cc  rule.
SCOTS_BINARY = "go2_controller"

# where the closed-loop deployment script lives, as seen from WSL
GO2_WSL_DIR    = "/home/cublab/Workspace-Linux/unitree_go2_dev/go2_dev"
GO2_RUN_SCRIPT = "SCOTSControlTest.py"

# the rigid body name in Motive for the quadruped
ROBOT_OBJECT_NAME = "GO2-001"

CONFIG_FILENAME = "arena_config.txt"

# ---------------------------------------------------------------------------
# synthesis parameters
# ---------------------------------------------------------------------------

TAU  = 0.3          # sampling period [s], must match the deployment loop
NINT = 5            # RK4 sub-steps per period


# Give yaw some extra padding; it tends to bug out
YAW_PAD = 0.4

# For x, y, z (in meters and radians)
STATE_ETA = [0.06, 0.06, 0.1256]

# Go2 hardware limits are vx [-2.5, 3.8], vy [-1.0, 1.0], vyaw [-4, 4].
# Those are far too fast for a 3 m arena at tau = 0.3 s: one step at 3.8 m/s
# crosses 1.14 m. Scale them down for indoor operation.
INPUT_LB  = [-2.5/5,-1.0/5,-4.0/5]
INPUT_UB  = [3.8/5, 1.0/5 ,4.0/5]
INPUT_ETA = [0.63, 0.2, 0.8]

# Grow every obstacle by this many metres on each side. The abstraction treats
# the robot as a point, so without inflation it will clip corners. Roughly
# half the Go2 footprint diagonal.
OBSTACLE_INFLATION = 0.15

# Yaw entry of the target matrix. Half-axis is 1/L, so 0.001 -> 1000 rad,
# i.e. the target accepts any heading.
TARGET_L_YAW = 0.001

# Fallback target half-extents [m] when the GUI object carries no size.
DEFAULT_TARGET_SIZE = (0.5, 0.5)

# ===========================================================================
# object harvesting
# ===========================================================================

def normalize_objects(raw):
    """Coerce ObjectManager's output into {name: csv_string}.

    ObjectManager.getObjectsString() is:

        def getObjectsString(self):
            return json.dumps(self.objects)

    and self.objects is a dict written only by addObject/updateObjectPosition,
    each value the 7-field CSV that AutoDeploy builds:

        "0, x, y, angle, 0, l, w"

    A dict is also accepted so callers can pass object_manager.objects
    directly. Anything else raises: an earlier version returned {} on a parse
    failure, which surfaced downstream as "No Target placed on the arena"
    and sent you looking at the canvas instead of the data.
    """
    if raw is None:
        return {}


    # check if its a json
    if isinstance(raw, str):
        try:
            # redefine raw
            raw = json.loads(raw)
        except ValueError as e:
            raise RuntimeError(
                "ObjectManager.getObjectsString() did not return valid JSON: %s"
                % e) from None
    # check if the loaded json is a dict (which it should be)
    if not isinstance(raw, dict):
        raise RuntimeError(
            "Expected a dict of {name: csv} from ObjectManager, got %s"
            % type(raw).__name__)

    out = {}
    for name, value in raw.items():
        if not isinstance(value, str):
            raise RuntimeError(
                "Object '%s' has a non-string value (%s). Expected the "
                "7-field CSV that addObject stores." % (name, type(value).__name__))
        out[str(name)] = value
    return out


def objects_from_manager(object_manager):
    """GUI-placed geometry, straight out of ObjectManager. No network.

    Raises if any entry is not the 7-field CSV, rather than silently dropping
    it -- a Target that fails to parse is indistinguishable from no Target at
    all by the time it reaches the config writer.
    """
    if object_manager is None:
        return {}

    objs = normalize_objects(object_manager.getObjectsString())

    # check there are 7 fields in each object entry
    for name, value in objs.items():
        n = len(value.split(','))
        if n != 7:
            raise RuntimeError(
                "Object '%s' has %d CSV fields, expected 7 "
                "(\"0,x,y,angle,0,l,w\"). Value was: %r" % (name, n, value))
    return objs


def collect_arena(object_manager=None,
                  tracked_objects=None,
                  include_tracked_obstacles=True,
                  robot_name=ROBOT_OBJECT_NAME):
    """Assemble the geometry to synthesize against.

    object_manager            the GUI's ObjectManager (authoritative)
    tracked_objects           the OptiTrack dict, or None to skip entirely
    include_tracked_obstacles pull in marker-tracked obstacles that were
                              never drawn in the GUI

    GUI objects win on a name collision. The robot itself is never an
    obstacle. Returns (targets, obstacles) as lists of object dicts.
    """
    gui = objects_from_manager(object_manager)
    _, targets, obstacles = parse_objects(gui, robot_name)

    # getValidObjectName only ever produces "Target<n>" or "Obstacle<n>", so
    # every GUI object must land in one of the two lists. Anything left over
    # would otherwise vanish without a trace.
    if gui and not targets and not obstacles:
        raise RuntimeError(
            "ObjectManager holds %d object(s) but none classified as a Target "
            "or Obstacle: %s" % (len(gui), sorted(gui.keys())))

    # physical objects on the field 
    if include_tracked_obstacles and tracked_objects:
        seen = set(o["name"] for o in obstacles)
        _, _, tracked_obs = parse_objects(tracked_objects, robot_name)
        for o in tracked_obs:
            if o["name"] not in seen:
                obstacles.append(o)

    return targets, obstacles


def parse_objects(objects, robot_name=ROBOT_OBJECT_NAME):
    # Categorizes a bunch of objects into 3 categories: dict robot, List<dict> targets, List<dict> obstacles
    robot, targets, obstacles = None, [], []
    # if empty, return nothing
    if not objects:
        return robot, targets, obstacles

    for name, raw in objects.items():
        if raw == "untracked":
            continue
        values = raw.split(',')
        if len(values) != 7:
            continue
        try:
            obj = {
                "name":  name,
                "x":     float(values[1]),
                "y":     float(values[2]),
                "angle": float(values[3]),
                "l":     float(values[5]),
                "w":     float(values[6]),
            }
        except ValueError:
            continue

        if name == robot_name:
            robot = obj
        elif "Target" in name:
            targets.append(obj)
        elif "Obstacle" in name:
            obstacles.append(obj)

    return robot, targets, obstacles


def obstacle_to_box(obj, inflation=OBSTACLE_INFLATION):
    """Axis-aligned [xmin, xmax, ymin, ymax] for one obstacle, inflated.

    Rotated obstacles are replaced by their axis-aligned bounding box, which
    over-approximates them. That is the safe direction to err.
    """
    hl, hw = obj["l"] / 2.0, obj["w"] / 2.0
    if abs(obj["angle"]) > 1e-6:
        c, s = abs(math.cos(obj["angle"])), abs(math.sin(obj["angle"]))
        hl, hw = hl * c + hw * s, hl * s + hw * c
    hl += inflation
    hw += inflation
    return [obj["x"] - hl, obj["x"] + hl, obj["y"] - hw, obj["y"] + hw]


def target_to_ellipsoid(obj, state_eta=STATE_ETA):
    """(cx, cy, cyaw, Lxx, Lyy, Lyaw) for one target.

    addEllipsoid uses  (x-c)' L'L (x-c) <= 1, so for diagonal L the half-axis
    is 1/L_ii. A rectangle of size l x w therefore maps to L = (2/l, 2/w).

    The INNER approximation only keeps cells lying entirely inside the
    ellipsoid, so a target smaller than about one cell yields an empty set.
    Half-axes are floored at 1.5 cells to keep that from happening silently.
    """
    a = max(obj["l"] / 2.0, 1.5 * state_eta[0])
    b = max(obj["w"] / 2.0, 1.5 * state_eta[1])
    return (obj["x"], obj["y"], 0.0, 1.0 / a, 1.0 / b, TARGET_L_YAW)


# ===========================================================================
# config writing
# ===========================================================================

def build_config_text(targets, obstacles,
                      state_lb, state_ub,
                      state_eta=STATE_ETA,
                      input_lb=INPUT_LB, input_ub=INPUT_UB, input_eta=INPUT_ETA,
                      tau=TAU, nint=NINT,
                      yaw_pad=YAW_PAD):
    """Render arena_config.txt. state_lb/ub are [x, y] in world metres."""
    yaw_lb = -math.pi - yaw_pad
    yaw_ub = math.pi + yaw_pad

    lines = [
        "# generated by SCOTSDeploy.py -- do not edit by hand",
        "",
        "tau            %.6g" % tau,
        "nint           %d"   % nint,
        # "growth_bound   %d"   % growth_bound,
        "",
        "state_lb       %.6g %.6g %.6g" % (state_lb[0], state_lb[1], yaw_lb),
        "state_ub       %.6g %.6g %.6g" % (state_ub[0], state_ub[1], yaw_ub),
        "state_eta      %.6g %.6g %.6g" % tuple(state_eta),
        "",
        "input_lb       %.6g %.6g %.6g" % tuple(input_lb),
        "input_ub       %.6g %.6g %.6g" % tuple(input_ub),
        "input_eta      %.6g %.6g %.6g" % tuple(input_eta),
        "",
    ]

    lines.append("# targets: cx cy cyaw Lxx Lyy Lyaw   (half-axis = 1/L)")
    for t in targets:
        lines.append("target         %.6g %.6g %.6g %.6g %.6g %.6g"
                     % target_to_ellipsoid(t, state_eta))
    lines.append("")

    lines.append("# obstacles: xmin xmax ymin ymax   (inflated by %.3g m)"
                 % OBSTACLE_INFLATION)
    for o in obstacles:
        lines.append("obstacle       %.6g %.6g %.6g %.6g" % tuple(obstacle_to_box(o)))
    lines.append("")

    return "\n".join(lines)


def write_config(text, config_dir=CONFIG_DIR, filename=CONFIG_FILENAME):
    path = config_dir.rstrip("/") + "/" + filename
    with open(path, "w", newline="\n") as f:
        f.write(text)
    return path


# ===========================================================================
# WSL execution
# ===========================================================================

def _wsl_command(shell_cmd, distro=WSL_DISTRO):
    return ["bash", "-lc", shell_cmd]


def run_in_wsl(shell_cmd, line_callback=None, distro=WSL_DISTRO):
    """Run a shell command in WSL, streaming stdout. Returns the exit code."""
    proc = subprocess.Popen(
        _wsl_command(shell_cmd, distro),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        bufsize=1,
    )
    for line in iter(proc.stdout.readline, ""):
        line = line.rstrip()
        if line_callback:
            line_callback(line)
        else:
            print(line)
    proc.stdout.close()
    return proc.wait()


def synthesize(targets, obstacles, state_lb, state_ub,
               line_callback=None, **kwargs):
    """Full synthesis: config -> make -> ./go2_controller.

    targets/obstacles come from collect_arena(), so this function does no
    harvesting of its own and never touches the network.

    Returns (exit_code, config_path). Exit code 0 means go2_controller.bdd
    was written with a non-empty winning set.
    """
    if not targets:
        raise RuntimeError("No Target placed on the arena. "
                           "Add one in the GUI before synthesising.")

    text = build_config_text(targets, obstacles, state_lb, state_ub, **kwargs)
    path = write_config(text)

    if line_callback:
        line_callback("Wrote %s (%d target(s), %d obstacle(s))"
                      % (CONFIG_FILENAME, len(targets), len(obstacles)))

    cmd = "cd '%s' && make && ./%s %s" % (SCOTS_WSL_DIR, SCOTS_BINARY, CONFIG_FILENAME)
    code = run_in_wsl(cmd, line_callback)
    return code, path


def launch_closed_loop(distro=WSL_DISTRO):
    """Start the Go2 deployment loop in WSL. Non-blocking."""
    cmd = "cd '%s' && python3 %s" % (GO2_WSL_DIR, GO2_RUN_SCRIPT)
    return subprocess.Popen(_wsl_command(cmd, distro))


def kill_closed_loop(distro=WSL_DISTRO):
    subprocess.Popen(_wsl_command("pkill -f %s" % GO2_RUN_SCRIPT, distro))


# ===========================================================================
# Qt wrapper so the GUI does not freeze during the transition relation
# ===========================================================================

if _HAVE_QT:

    class SynthesisThread(QThread):
        """Runs synthesize() off the GUI thread.

        Signals:
            log(str)            one line of build/synthesis output
            finished_ok(bool)   True if the controller was written
        """
        log = pyqtSignal(str)
        finished_ok = pyqtSignal(bool)

        def __init__(self, geometry_provider, state_lb, state_ub,
                     parent=None, **kwargs):
            """geometry_provider() -> (targets, obstacles)

            Typically  lambda: SCOTSDeploy.collect_arena(self.object_manager,
                                                         get_objects())
            Called on the GUI thread before start() would be safer still, but
            ObjectManager reads are cheap and non-blocking so calling it here
            is fine.
            """
            super(SynthesisThread, self).__init__(parent)
            self.geometry_provider = geometry_provider
            self.state_lb = state_lb
            self.state_ub = state_ub
            self.kwargs = kwargs

        def run(self):
            try:
                targets, obstacles = self.geometry_provider()
                code, _ = synthesize(targets, obstacles,
                                     self.state_lb, self.state_ub,
                                     line_callback=self.log.emit,
                                     **self.kwargs)
                if code == 0:
                    self.log.emit("Synthesis complete: go2_controller.bdd ready")
                    self.finished_ok.emit(True)
                else:
                    self.log.emit("Synthesis FAILED (exit %d) -- see output above"
                                  % code)
                    self.finished_ok.emit(False)
            except Exception as e:
                self.log.emit("Synthesis error: %s" % e)
                self.finished_ok.emit(False)


# ===========================================================================
# standalone smoke test:  python SCOTSDeploy.py
# ===========================================================================

if __name__ == "__main__":

    class FakeObjectManager(object):
        """Stands in for the GUI's ObjectManager, matching addObject's CSV."""
        def __init__(self, objs):
            self._objs = objs
        def getObjectsString(self):
            return json.dumps(self._objs)

    # what the user drew in the GUI
    gui = FakeObjectManager({
        "Target1":   "0,-1.25,0.0,0,0,1.0,1.0",
        "Obstacle1": "0,0.275,-0.55,0,0,0.05,0.5",
        "Obstacle2": "0,-0.275,0.55,0,0,0.05,0.5",
    })

    # what OptiTrack sees: the robot, an echo of the GUI objects, and one
    # physically tracked obstacle that was never drawn
    tracked = {
        "GO2-001":     "0,1.0,0.5,3.05,0,0.35,0.25",
        "Target1":     "0,-1.25,0.0,0,0,1.0,1.0",
        "Obstacle1":   "0,0.275,-0.55,0,0,0.05,0.5",
        "ObstacleBox": "0,0.8,-0.8,0,0,0.4,0.4",
    }

    print("--- GUI only ---")
    t, o = collect_arena(gui, tracked_objects=None)
    print("targets  :", [x["name"] for x in t])
    print("obstacles:", [x["name"] for x in o])

    print("\n--- GUI + tracked physical obstacles ---")
    t, o = collect_arena(gui, tracked, include_tracked_obstacles=True)
    print("targets  :", [x["name"] for x in t])
    print("obstacles:", [x["name"] for x in o])
    print("(GO2-001 correctly excluded, Obstacle1 not duplicated)")

    print()
    print(build_config_text(t, o,
                            state_lb=[-1.5, -1.5], state_ub=[1.5, 1.5]))