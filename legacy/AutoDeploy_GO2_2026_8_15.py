import sys, os, math, requests, json, keyboard, cv2, subprocess
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtTest
from PIL import Image, ImageDraw
import numpy as np

import ConfigReader
import NDIImageSender
import ObjectManager
import ArenaManager_WSL.SCOTSDeploy as SCOTSDeploy

ENABLE_PROJECTION = False   # Ventuz + NDI
ENABLE_TRACKING   = True   # Motive + localization server
ENABLE_CAMERA     = False    # DirectShow webcam

config_file = "robot.cfg"
config_reader = ConfigReader.ConfigReader(config_file)

localization_server_url = "http://192.168.1.194:12345/OptiTrackRestServer"

manual_control_port = ":1234"

camera_resolution = (1280, 720)  # 16:9

# rigid body name in Motive for the quadruped
ROBOT_NAME = SCOTSDeploy.ROBOT_OBJECT_NAME  # "GO2-001"

ROBOT_RENDER_ANGLE_OFFSET = 0

global render_robot
# Hidden by default ONLY because the canvas is projected onto the arena floor
# where the real robot already is. With projection off there is no reason to
# hide it -- and doing so was a regression: originally "GO2-001" never matched
# the "DeepRacer" test, so it skipped this gate and always drew.
render_robot = not ENABLE_PROJECTION

# Synthesis geometry comes from ObjectManager. Set this True to additionally
# pull in marker-tracked obstacles that physically exist in the arena but were
# never drawn in the GUI (an ObstacleCar, a box with a rigid body). Leave it
# False if every obstacle is drawn, which avoids a network read entirely.
MERGE_TRACKED_OBSTACLES = True

# NOTE: the Obstacle default was ("3.0", "0.4") for the DeepRacer arena, which
# was much larger. In a 3 m arena a 3.0 m wide obstacle spans the full width
# and, once inflated, seals it off completely -> empty winning set.
default_object_sizes = {"GO2": ("0.25", "0.35"), "Target": ("0.8", "0.8"), "Obstacle": ("0.6", "0.2")}


def str2list(strList):
    return [float(i.replace(" ", "")) for i in strList.split(",")]


x_lb = str2list(config_reader.get_value_string("system.states.first_symbol"))
x_ub = str2list(config_reader.get_value_string("system.states.last_symbol"))
x_eta = str2list(config_reader.get_value_string("system.states.quantizers"))

print("bounds:", x_lb, x_ub, x_eta)


if __name__ == "__main__":
    # ndiImgSender = NDIImageSender.NDIImageSender(b'My_PNG', 10)  # initialize Ventuz projection
    if ENABLE_CAMERA:
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if cap.isOpened() == False:
            print("connection to camera device failed")
        else:
            cap.set(3, camera_resolution[0])
            cap.set(4, camera_resolution[1])
    else:
        # Do not even open the device when the camera is disabled -- under WSL
        # this always fails and the failed-but-not-None capture object then
        # confuses CameraThread.
        cap = None
    img_background = Image.open("background.png")

SIM_WIDTH = int(config_reader.get_value_string("simulation.window_width"))
SIM_HEIGHT = int(config_reader.get_value_string("simulation.window_height"))
path_tail_length = int(config_reader.get_value_string("simulation.path_tail_length"))
draw_tail = False
path_tail = []


# run motive program
def start_motive():
    # motive = subprocess.Popen("C:/Users/CUBLab/Desktop/Motive_Best_Calibration.lnk",stdout=subprocess.PIPE,shell=True)
    motive = subprocess.Popen("C:/Users/CUBLab/Desktop/Motive_Updated_Calibration_Dimmed_1_8_2026.lnk", stdout=subprocess.PIPE, shell=True)
    return motive


# run localization server
def start_localization_server():
    localization_server = subprocess.Popen("D:/Workspace/OptiTrackRESTServer/start_admin.bat")
    return localization_server


# run ventuz
def start_ventuz():
    ventuz = subprocess.Popen("D:/Workspace/NDIRestServer/ventuz/NDIRestServerRecveiver/Presentations/NDIRestServerReceiver.vpr", stdout=subprocess.PIPE, shell=True)
    return ventuz


def kill_go2_controller():
    """Stop the closed-loop deployment script running inside WSL."""
    SCOTSDeploy.kill_closed_loop()


def controller_bdd_path():
    """Where go2_controller.bdd lands after a successful synthesis."""
    return os.path.join(SCOTSDeploy.SCOTS_WSL_DIR, "go2_controller.bdd")


def controller_exists():
    """True if there is a controller on disk to deploy.

    Lets the Run button work against a controller synthesised in an earlier
    session, without forcing a re-run of the transition relation.
    """
    try:
        return os.path.isfile(controller_bdd_path())
    except Exception:
        return False


def get_objects():
    try:
        return json.loads(requests.get(localization_server_url).text)
    except Exception as e:
        print("unable to retrieve from localization server: " + str(e))


# function to convert a rectangle defined by (x,y) positon, (w,l) size and angle in rad to polygon coordinates
def get_polygon(x, y, w, l, angle):
    c, s = math.cos(angle), math.sin(angle)
    coords = [(l / 2.0, w / 2.0), (l / 2.0, -w / 2.0), (-l / 2.0, -w / 2.0), (-l / 2.0, w / 2.0)]
    return [(c * x_val - s * y_val + x, s * x_val + c * y_val + y) for (x_val, y_val) in coords]


_last_draw_report = None

def _report_draw(objects):
    """Print the object set once whenever it changes.

    Nothing rendering is almost always one of: the localization server is
    unreachable, it is reachable but empty, or every entry reads "untracked".
    Those look identical on a blank canvas, so name them explicitly.
    """
    global _last_draw_report
    if objects is None:
        report = "unreachable"
    elif not objects:
        report = "reachable, 0 objects"
    else:
        tracked = [n for n, v in objects.items() if v != "untracked"]
        untracked = [n for n, v in objects.items() if v == "untracked"]
        report = "drawable=%s" % (sorted(tracked) or "NONE")
        if untracked:
            report += "  untracked=%s" % sorted(untracked)
    if report != _last_draw_report:
        print("[canvas] localization server: %s" % report)
        _last_draw_report = report


def draw_simulation():
    global path_tail
    objects = get_objects()
    _report_draw(objects)
    if objects is None:
        # img is not assigned yet at this point -- referencing it here was the
        # "local variable 'img' referenced before assignment" error.
        return img_background.copy().transpose(Image.FLIP_TOP_BOTTOM)
    if ROBOT_NAME not in objects:
        path_tail = []
    img = img_background.copy()
    draw = ImageDraw.Draw(img)
    for i in range(len(objects.keys()) - 1, -1, -1):
        name = list(objects.keys())[i]
        if objects[name] != "untracked":
            values = objects[name].split(',')
            if len(values) != 7: continue
            x, y = fields_to_world(values)
            x, y = world_to_canvas(x, y)
            w = float(values[6]); l = float(values[5])
            angle = 0
            color = "#00FFFF"  # default color yellow
            is_robot = (name == ROBOT_NAME) or ("GO2" in name) or ("ObstacleCar" in name)
            if is_robot:
                if not render_robot: continue
                angle = (float(values[3]) + ROBOT_RENDER_ANGLE_OFFSET)
                color = "#0000EE"
            if "Obstacle" in name:
                color = "#EE0000"
            elif "Target" in name:
                color = "#00EE00"
            [x, y] = state_world_to_arena([x, y])
            # l is the x-extent and w the y-extent, so they must be scaled by
            # the x and y scales respectively. The original passed [w, l],
            # which applied the x scale to the y-extent and vice versa.
            [l, w] = norm_world_to_arena([l, w])
            if name == ROBOT_NAME:
                path_tail.insert(0, (x, y))
                if len(path_tail) > path_tail_length:
                    path_tail = path_tail[:path_tail_length]
                draw.polygon(get_polygon(x, y, w, l, angle), fill=color)
            elif is_robot:
                draw.polygon(get_polygon(x, y, w, l, angle), fill=color)
            else:
                draw.rounded_rectangle((x - l / 2, y - w / 2, x + l / 2, y + w / 2), fill=color, radius=10)
            draw.polygon(get_polygon(x, y, 5, 5, angle), fill="black")  # draw black dot at center
    if path_tail != [] and draw_tail:
        draw.line(path_tail, width=6, fill="#0000EE", joint="curve")
    return img.transpose(Image.FLIP_TOP_BOTTOM)


def pil2pixmap(im):  # function to convert pil image to pyqt pixmap
    r, g, b, a = im.split()
    im = Image.merge("RGBA", (b, g, r, a))
    data = im.tobytes("raw", "RGBA")
    qim = QImage(data, im.size[0], im.size[1], QImage.Format_ARGB32)
    pixmap = QPixmap.fromImage(qim)
    return pixmap


def norm_world_to_arena(norm):  # translate the size tuple in world units to pixels
    arena_x = (norm[0]) * (SIM_WIDTH / (x_ub[0] - x_lb[0]))
    arena_y = (norm[1]) * (SIM_HEIGHT / (x_ub[1] - x_lb[1]))
    return [arena_x, arena_y]


def norm_arena_to_world(norm):  # translate the size tuple in pixels to world units
    world_x = (norm[0]) / (SIM_WIDTH / (x_ub[0] - x_lb[0]))
    world_y = (norm[1]) / (SIM_HEIGHT / (x_ub[1] - x_lb[1]))
    return [world_x, world_y]


def state_world_to_arena(state):  # translate the position tuple in world units to pixels
    arena_x = (state[0] - x_lb[0] + x_eta[0] / 2) * (SIM_WIDTH / (x_ub[0] - x_lb[0] + x_eta[0]))
    arena_y = (state[1] - x_lb[1] + x_eta[1] / 2) * (SIM_HEIGHT / (x_ub[1] - x_lb[1] + x_eta[1]))
    return [arena_x, arena_y]


def state_arena_to_world(state):  # translate the position tuple in pixels to world units
    world_x = state[0] / (SIM_WIDTH / (x_ub[0] - x_lb[0] + x_eta[0])) + x_lb[0] - x_eta[0] / 2
    world_y = state[1] / (SIM_HEIGHT / (x_ub[1] - x_lb[1] + x_eta[1])) + x_lb[1] - x_eta[1] / 2
    return [world_x, world_y]

def fields_to_world(values):
    """CSV fields -> canvas (x, y). Inverse of SCOTSControlTest.parse_objects."""
    return float(values[2]), -float(values[1])


def world_to_fields(x, y):
    """Canvas (x, y) -> the [1] and [2] CSV fields."""
    return -y, x

def world_to_canvas(x, y):
    """World -> canvas axes. World +x is up-screen, world +y is left."""
    return -y, x


def canvas_to_world_axes(cx, cy):
    """Inverse of the above."""
    return cy, -cx


def canvas_to_world(px, py):
    cx, cy = state_arena_to_world([px, SIM_HEIGHT - py])
    return canvas_to_world_axes(cx, cy)


class CameraThread(QThread):
    changePixmap = pyqtSignal(QPixmap)

    def __init__(self, parent=None):
        super(CameraThread, self).__init__(parent)
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        if cap is None or not cap.isOpened():
            return
        while self._running:
            status, image = cap.read()
            if not status:
                self.msleep(50)
                continue
            image = cv2.rotate(cv2.cvtColor(cv2.resize(image, (640, 360)), cv2.COLOR_BGR2RGB), cv2.ROTATE_180)
            self.changePixmap.emit(pil2pixmap(Image.fromarray(image).convert("RGBA")))


class CanvasThread(QThread):
    changePixmap = pyqtSignal(QPixmap)

    def __init__(self, parent=None):
        super(CanvasThread, self).__init__(parent)
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        while self._running:
            try:
                img = draw_simulation()
                img_bytes = bytes(np.array(img))
                self.changePixmap.emit(pil2pixmap(img))
                # ndiImgSender.send_image(img_bytes, SIM_WIDTH, SIM_HEIGHT)
            except Exception as e:
                print("[canvas] draw failed: %s" % e)
                self.msleep(1000)
                continue
            self.msleep(33)


class AutoDeploy(QMainWindow):  # class for main gui window
    def __init__(self):
        super().__init__()
        self.object_manager = None
        self.thread_scots = None
        self.controller_ready = False
        self.controller_proc = None       # the running closed loop, if any

        self.view_camera = True
        self.view_edit = True
        self.checked_object_type = "None"
        self.drag_object_name = "None"

        hbox_btns = QHBoxLayout()

        self.font = QFont("Calibri", 11)
        self.font_bold = QFont("Calibri", 11, QFont.Bold)

        # setup menu bar
        self.menu = self.menuBar()
        # create file menu
        self.menu_file = self.menu.addMenu("File")

        # create 2 actions
        self.action_load_config = QAction("Load Objects Configuration")
        self.action_load_config.setShortcut("Ctrl+O")
        self.action_load_config.triggered.connect(self.load_config)
        self.action_load_config.setEnabled(False)
        self.action_load_config.setFont(self.font)

        self.action_save_config = QAction("Save Current Objects Configuration")
        self.action_save_config.setShortcut("Ctrl+S")
        self.action_save_config.triggered.connect(self.save_config)
        self.action_save_config.setEnabled(False)
        self.action_save_config.setFont(self.font)
        # add actions to file menu
        self.menu_file.addAction(self.action_load_config)
        self.menu_file.addAction(self.action_save_config)

        self.menu_edit = self.menu.addMenu("Edit")
        self.action_undo = QAction("Undo")
        self.action_undo.setShortcut("Ctrl+Z")
        self.action_undo.triggered.connect(self.undo)
        self.action_undo.setEnabled(False)
        self.action_undo.setFont(self.font)
        self.action_delete_target = QAction("Delete Added Targets")
        self.action_delete_target.triggered.connect(self.delete_target)
        self.action_delete_target.setEnabled(False)
        self.action_delete_target.setFont(self.font)
        self.action_delete_obstacle = QAction("Delete Added Obstacles")
        self.action_delete_obstacle.triggered.connect(self.delete_obstacle)
        self.action_delete_obstacle.setEnabled(False)
        self.action_delete_obstacle.setFont(self.font)
        self.action_delete_config = QAction("Delete All Added Objects")
        self.action_delete_config.triggered.connect(self.delete_config)
        self.action_delete_config.setEnabled(False)
        self.action_delete_config.setFont(self.font)
        self.menu_edit.addAction(self.action_undo)
        self.menu_edit.addSeparator()
        self.menu_edit.addAction(self.action_delete_target)
        self.menu_edit.addAction(self.action_delete_obstacle)
        self.menu_edit.addSeparator()
        self.menu_edit.addAction(self.action_delete_config)

        self.menu_view = self.menu.addMenu("View")
        self.action_toggle_camera = QAction("Toggle Camera View", self, checkable=True)
        self.action_toggle_camera.setFont(self.font)
        self.action_toggle_camera.triggered.connect(self.toggle_camera)
        self.action_toggle_edit = QAction("Toggle Edit View", self, checkable=True)
        self.action_toggle_edit.setChecked(True)
        self.action_toggle_edit.setFont(self.font)
        self.action_toggle_edit.triggered.connect(self.toggle_edit)
        self.action_toggle_tail = QAction("Toggle Tail Visibility", self, checkable=True)
        self.action_toggle_tail.setChecked(False)
        self.action_toggle_tail.setFont(self.font)
        self.action_toggle_tail.triggered.connect(self.toggle_tail)
        self.action_toggle_robot_render = QAction("Toggle Go2 Rendering", self, checkable=True)
        # match the module default rather than contradicting it
        self.action_toggle_robot_render.setChecked(render_robot)
        self.action_toggle_robot_render.setFont(self.font)
        self.action_toggle_robot_render.triggered.connect(self.toggle_robot_render)
        self.action_toggle_camera.setChecked(ENABLE_CAMERA)
        self.menu_view.addAction(self.action_toggle_camera)
        self.menu_view.addAction(self.action_toggle_edit)
        self.menu_view.addSeparator()
        self.menu_view.addAction(self.action_toggle_tail)
        self.menu_view.addAction(self.action_toggle_robot_render)

        self.menu_scots = self.menu.addMenu("SCOTS")
        self.action_write_config_only = QAction("Write arena_config.txt Only")
        self.action_write_config_only.triggered.connect(self.write_config_only)
        self.action_write_config_only.setEnabled(False)
        self.action_write_config_only.setFont(self.font)
        self.action_show_config = QAction("Preview arena_config.txt")
        self.action_show_config.triggered.connect(self.show_config)
        self.action_show_config.setEnabled(False)
        self.action_show_config.setFont(self.font)
        self.menu_scots.addAction(self.action_write_config_only)
        self.menu_scots.addAction(self.action_show_config)

        self.menu_help = self.menu.addMenu("Help")
        self.action_kill_go2_controller = QAction("Kill Go2 Controller")
        self.action_kill_go2_controller.triggered.connect(self.stop_controller)
        self.action_kill_go2_controller.setFont(self.font)
        self.menu_help.addAction(self.action_kill_go2_controller)

        # setup buttons at the top
        self.setup_complete = False
        self.btn_env = QPushButton("Initialize Environment", self)
        self.btn_env.clicked.connect(self.start_environment)
        self.btn_env.setFont(self.font)

        # --- synthesis and deployment are separate actions -----------------
        # Synthesis is a multi-minute offline computation that produces
        # go2_controller.bdd. Running is a live loop that reads that file.
        # Keeping them apart means you can re-run a controller without
        # re-synthesising, and synthesise without immediately driving a robot.
        self.btn_synth = QPushButton("Synthesize Controller", self)
        self.btn_synth.clicked.connect(self.start_symbolic)
        self.btn_synth.setFont(self.font)
        self.btn_synth.setEnabled(False)

        self.btn_run = QPushButton("Run Controller", self)
        self.btn_run.clicked.connect(self.toggle_controller)
        self.btn_run.setFont(self.font)
        self.btn_run.setEnabled(False)

        self.btn_manual = QPushButton("Manual Drive", self)
        self.btn_manual.setFont(self.font)
        self.btn_manual.clicked.connect(self.start_manual_drive)

        self.btns = [self.btn_env, self.btn_synth, self.btn_run, self.btn_manual]

        hbox_btns.addWidget(self.btn_env)
        hbox_btns.addWidget(self.btn_synth)
        hbox_btns.addWidget(self.btn_run)
        hbox_btns.addWidget(self.btn_manual)

        # poll the closed loop so the button resets if it exits on its own
        self.controller_timer = QTimer(self)
        self.controller_timer.setInterval(1000)
        self.controller_timer.timeout.connect(self._poll_controller)

        # setup camera view
        self.camera = QLabel()
        self.camera.setAlignment(Qt.AlignCenter)
        self.thread_camera = CameraThread(self)
        self.thread_camera.changePixmap.connect(self.set_camera_image)
        if ENABLE_CAMERA:
            self.thread_camera.start()
        else:
            self.camera.hide()
            self.view_camera = False

        # setup canvas / simulation view
        self.canvas = QLabel()
        self.canvas.setAlignment(Qt.AlignCenter)
        self.canvas.setPixmap(QPixmap("background.png"))
        self.canvas.mousePressEvent = self.canvas_press_event
        self.canvas.mouseReleaseEvent = self.canvas_release_event
        self.canvas.mouseMoveEvent = self.canvas_move_event
        self.thread_canvas = CanvasThread(self)
        self.thread_canvas.changePixmap.connect(self.set_canvas_image)

        # setup edit view
        self.hbox_edit = QHBoxLayout()

        vbox_shape = QVBoxLayout()
        self.label_object_type = QLabel("Select Object Type:")
        self.label_object_type.setFont(self.font_bold)
        self.checkbox_target = QCheckBox("Target")
        self.checkbox_target.setFont(self.font)
        self.checkbox_target.stateChanged.connect(self.edit_checked)
        self.checkbox_obstacle = QCheckBox("Obstacle")
        self.checkbox_obstacle.setFont(self.font)
        self.checkbox_obstacle.stateChanged.connect(self.edit_checked)
        vbox_shape.addWidget(self.label_object_type)
        vbox_shape.addWidget(self.checkbox_target)
        vbox_shape.addWidget(self.checkbox_obstacle)
        vbox_shape.setAlignment(Qt.AlignCenter)

        vbox_dimensions = QVBoxLayout()
        self.label_dimensions = QLabel("Dimensions: (one block is size " + str(norm_arena_to_world([600 / 4, 0])[0])[:3] + ")")
        self.label_dimensions.setFont(self.font_bold)
        vbox_dimensions_info2 = QHBoxLayout()
        self.label_dimensions_info2 = QLabel("Leave Blank for Default Sizes")
        self.label_dimensions_info2.setFont(self.font)
        self.btn_edit_clear = QPushButton("Clear")
        self.btn_edit_clear.setFont(self.font)
        self.btn_edit_clear.clicked.connect(self.clear_edit_textbox)
        vbox_dimensions_info2.addWidget(self.label_dimensions_info2)
        vbox_dimensions_info2.addStretch(1)
        vbox_dimensions_info2.addWidget(self.btn_edit_clear)
        hbox_edit_line1 = QHBoxLayout()
        self.label_width = QLabel("Horizontal Width: ")
        self.label_width.setFont(self.font)
        self.textbox_width = QLineEdit()
        self.textbox_width.setFont(self.font)
        self.textbox_width.setPlaceholderText("Enter Width")
        self.textbox_width.setValidator(QRegExpValidator(QRegExp(r'[0-9].+')))
        hbox_edit_line1.addWidget(self.label_width)
        hbox_edit_line1.addStretch(1)
        hbox_edit_line1.addWidget(self.textbox_width)
        hbox_edit_line2 = QHBoxLayout()
        self.label_height = QLabel("Vertical Height: ")
        self.label_height.setFont(self.font)
        self.textbox_height = QLineEdit()
        self.textbox_height.setFont(self.font)
        self.textbox_height.setPlaceholderText("Enter Height")
        self.textbox_height.setValidator(QRegExpValidator(QRegExp(r'[0-9].+')))
        hbox_edit_line2.addWidget(self.label_height)
        hbox_edit_line2.addStretch(1)
        hbox_edit_line2.addWidget(self.textbox_height)
        vbox_dimensions.addWidget(self.label_dimensions)
        vbox_dimensions.addLayout(vbox_dimensions_info2)
        vbox_dimensions.addLayout(hbox_edit_line1)
        vbox_dimensions.addLayout(hbox_edit_line2)
        vbox_dimensions.setAlignment(Qt.AlignCenter)

        self.hbox_edit.addStretch(1)
        self.hbox_edit.addLayout(vbox_shape, 2)
        self.hbox_edit.addStretch(1)
        self.hbox_edit.addLayout(vbox_dimensions, 3)
        self.hbox_edit.addStretch(1)
        self.frame_edit = QFrame()
        self.frame_edit.setLayout(self.hbox_edit)
        self.frame_edit.setEnabled(False)

        vbox_left = QVBoxLayout()
        vbox_left.addWidget(self.camera)
        vbox_left.addWidget(self.frame_edit)

        hbox_main = QHBoxLayout()
        hbox_main.addLayout(vbox_left)
        hbox_main.addWidget(self.canvas)

        self.status = self.statusBar()
        self.status.setFont(self.font)
        self.status.showMessage("Initialize Environment to Run Simulation")

        # synthesis log, hidden until the first synthesis run
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Consolas", 9))
        self.log_view.setMaximumHeight(150)
        self.log_view.hide()

        # setup window layout
        vbox = QVBoxLayout()
        vbox.addLayout(hbox_btns)
        vbox.addLayout(hbox_main)
        vbox.addWidget(self.log_view)
        vbox.addWidget(self.status)

        self.setWindowTitle("Unitree Go2 SCOTS Automatic Deployment")
        self.setWindowIcon(QIcon('background.png'))
        main_widget = QWidget()
        main_widget.setLayout(vbox)
        self.setCentralWidget(main_widget)
        self.setWindowFlag(Qt.WindowMaximizeButtonHint, False)  # disable maximize button

    @pyqtSlot(QPixmap)
    def set_camera_image(self, pixmap):
        self.camera.setPixmap(pixmap)

    @pyqtSlot(QPixmap)
    def set_canvas_image(self, pixmap):
        self.canvas.setPixmap(pixmap)

    def toggle_camera(self, state):
        if state:
            self.view_camera = True
            self.camera.show()
        else:
            self.view_camera = False
            self.camera.hide()

    def toggle_edit(self, state):
        if state:
            self.view_edit = True
            self.frame_edit.show()
        else:
            self.view_edit = False
            self.frame_edit.hide()

    def toggle_tail(self, state):
        global draw_tail
        draw_tail = bool(state)

    def toggle_robot_render(self, state):
        global render_robot
        render_robot = bool(state)

    def clear_edit_textbox(self):
        self.textbox_width.setText("")
        self.textbox_height.setText("")

    def edit_checked(self, state):
        checkboxes = [self.checkbox_obstacle, self.checkbox_target]
        if state == Qt.Checked:
            for checkbox in checkboxes:
                if self.sender() != checkbox:
                    checkbox.setChecked(False)
                else:
                    self.checked_object_type = checkbox.text()
                    self.status.showMessage(checkbox.text() + " selected, click on the simulation on the right to place object")
        is_all_unchecked = True
        for checkbox in checkboxes:
            if checkbox.checkState() == 2:
                is_all_unchecked = False
        if is_all_unchecked:
            self.checked_object_type = "None"

    def start_environment(self):  # function to start necessay software environments in sequence ~30s
        self.btn_env.setEnabled(False)
        self.motive = None
        self.ventuz = None
        self.localization_server = None
        skipped = []

        if ENABLE_TRACKING:
            self.status.showMessage("Starting Motive...")
            self.motive = subprocess.Popen(["cmd.exe", "/c", "start", "", "Motive_Best_Calibration.lnk"], cwd="/mnt/c/Users/CUBLab/Desktop", stdout=subprocess.PIPE)

            self.activateWindow()
            QtTest.QTest.qWait(16000)
        else:
            skipped.append("Motive")

        if ENABLE_PROJECTION:
            self.status.showMessage("Starting Ventuz...")
            self.ventuz = start_ventuz()
            self.activateWindow()
            QtTest.QTest.qWait(10000)
        else:
            skipped.append("Ventuz/NDI")

        if ENABLE_TRACKING:
            self.status.showMessage("Starting Localization Server...")
            self.localization_server = subprocess.Popen(["cmd.exe", "/c", "start_admin.bat"], cwd="/mnt/d/Workspace/OptiTrackRESTServer")
            QtTest.QTest.qWait(2000)
            self.activateWindow()
        else:
            skipped.append("localization server")

        self.setup_complete = True
        self.action_load_config.setEnabled(True)
        self.action_save_config.setEnabled(True)
        self.action_undo.setEnabled(True)
        self.action_delete_target.setEnabled(True)
        self.action_delete_obstacle.setEnabled(True)
        self.action_delete_config.setEnabled(True)
        self.action_write_config_only.setEnabled(True)
        self.action_show_config.setEnabled(True)
        self.btn_synth.setEnabled(True)
        self.frame_edit.setEnabled(True)

        # Run is available immediately if a controller from an earlier session
        # is still on disk -- no need to re-synthesise to redeploy.
        self.controller_ready = controller_exists()
        self.btn_run.setEnabled(self.controller_ready)

        self.object_manager = ObjectManager.ObjectManager()
        stale = get_objects() or {}
        orphans = [n for n in stale
                   if ("Target" in n or "Obstacle" in n) and n not in self.object_manager.objects]
        if orphans:
            self.object_manager.objects.update({n: stale[n] for n in orphans})
            print("adopted %d orphaned object(s) from a previous session: %s"
                  % (len(orphans), orphans))
        self.thread_canvas.start()

        reachable = get_objects() is not None
        msg = "Environment initialized. Tracking %s." % ("OK" if reachable else "NOT reachable")
        if skipped:
            msg += "  Not started: %s." % ", ".join(skipped)
        if self.controller_ready:
            msg += "  Existing controller found."
        self.status.showMessage(msg)

    def load_config(self):
        self.delete_config()
        path = os.path.expanduser("~/Desktop")
        config_file = QFileDialog.getOpenFileName(self, "Open Configuration File", path, "JSON Files (*.json)")[0]
        if config_file:
            self.object_manager.loadConfig(config_file)
            self.status.showMessage("Config File Successfully Loaded")

    def save_config(self):
        path = os.path.expanduser("~/Desktop")
        save_filename = QFileDialog.getSaveFileName(self, "Save Configuration File", path + "/objects_config.json", "JSON Files (*.json)")[0]
        if save_filename:
            objects_string = self.object_manager.getObjectsString()
            f = open(save_filename, "w")
            f.write(objects_string)
            f.close()
            self.status.showMessage("Config File Saved")

    def undo(self):
        self.object_manager.undo()

    def delete_target(self):
        self.object_manager.deleteByType("Target")

    def delete_obstacle(self):
        self.object_manager.deleteByType("Obstacle")

    def delete_config(self):
        self.object_manager.deleteAll()

    # -----------------------------------------------------------------------
    # SCOTS -- synthesis
    # -----------------------------------------------------------------------
    def collect_geometry(self):
        """Geometry to synthesise against.

        ObjectManager is authoritative for whatever the user drew -- it is
        in-process and correct the moment a drag ends, with no network round
        trip and no race against the REST POST landing.

        OptiTrack is consulted only for physically tracked obstacles that
        carry markers and were therefore never drawn in the GUI. That read
        is best-effort: if the server is down, synthesis still proceeds on
        the drawn geometry.
        """
        tracked = None
        if MERGE_TRACKED_OBSTACLES:
            try:
                tracked = get_objects()
            except Exception as e:
                print("tracked-obstacle merge skipped: " + str(e))
        return SCOTSDeploy.collect_arena(
            self.object_manager,
            tracked_objects=tracked,
            include_tracked_obstacles=MERGE_TRACKED_OBSTACLES)

    def _build_config_text(self):
        """Render arena_config.txt from the GUI geometry. Raises on no target."""
        targets, obstacles = self.collect_geometry()
        if not targets:
            raise RuntimeError("No Target placed on the arena.")
        return SCOTSDeploy.build_config_text(targets, obstacles,
                                             state_lb=x_lb, state_ub=x_ub), targets, obstacles

    def write_config_only(self):
        """Write arena_config.txt without running synthesis."""
        try:
            text, targets, obstacles = self._build_config_text()
            path = SCOTSDeploy.write_config(text)
            self.status.showMessage("Wrote %s  (%d target(s), %d obstacle(s))"
                                    % (path, len(targets), len(obstacles)))
        except Exception as e:
            self.status.showMessage("Could not write config: " + str(e))

    def show_config(self):
        """Preview what would be written, without touching disk."""
        try:
            text, _, _ = self._build_config_text()
        except Exception as e:
            text = "Could not build config: " + str(e)
        dlg = QDialog(self)
        dlg.setWindowTitle("arena_config.txt preview")
        dlg.resize(600, 500)
        view = QPlainTextEdit(dlg)
        view.setReadOnly(True)
        view.setFont(QFont("Consolas", 10))
        view.setPlainText(text)
        lay = QVBoxLayout(dlg)
        lay.addWidget(view)
        dlg.exec_()

    def start_symbolic(self):
        """Harvest arena -> write config -> make && ./go2_controller.

        Synthesis only. Deployment is the separate Run button, so a long
        synthesis never surprises you by putting a robot in motion when it
        finishes.
        """
        if self.thread_scots is not None and self.thread_scots.isRunning():
            self.status.showMessage("Synthesis already running...")
            return

        self.btn_synth.setEnabled(False)
        self.btn_synth.setText("Synthesizing...")
        self.log_view.clear()
        self.log_view.show()
        global path_tail
        path_tail = []

        self.thread_scots = SCOTSDeploy.SynthesisThread(
            geometry_provider=self.collect_geometry,
            state_lb=x_lb,
            state_ub=x_ub,
            parent=self)
        self.thread_scots.log.connect(self.append_log)
        self.thread_scots.finished_ok.connect(self.on_synthesis_done)
        self.thread_scots.start()
        self.status.showMessage("Synthesising controller -- the transition "
                                "relation can take several minutes...")

    @pyqtSlot(str)
    def append_log(self, line):
        self.log_view.appendPlainText(line)
        self.log_view.verticalScrollBar().setValue(
            self.log_view.verticalScrollBar().maximum())
        self.status.showMessage(line[:160])

    @pyqtSlot(bool)
    def on_synthesis_done(self, ok):
        self.btn_synth.setEnabled(True)
        self.btn_synth.setText("Re-synthesize Controller" if ok else "Synthesize Controller")
        self.controller_ready = ok or controller_exists()
        self.btn_run.setEnabled(self.controller_ready)
        if ok:
            self.status.showMessage("Controller ready. Press Run Controller to deploy.")
        else:
            self.status.showMessage("Synthesis failed -- see log above. "
                                    "Empty winning set usually means the input grid "
                                    "is too coarse or the target is unreachable.")

    # -----------------------------------------------------------------------
    # SCOTS -- deployment
    # -----------------------------------------------------------------------
    def _controller_running(self):
        return (self.controller_proc is not None
                and self.controller_proc.poll() is None)

    def toggle_controller(self):
        if self._controller_running():
            self.stop_controller()
        else:
            self.run_controller()

    def run_controller(self):
        """Launch the closed loop against whatever go2_controller.bdd exists."""
        if not controller_exists():
            self.status.showMessage(
                "No go2_controller.bdd at %s -- synthesize first."
                % controller_bdd_path())
            self.btn_run.setEnabled(False)
            return
        try:
            self.controller_proc = SCOTSDeploy.launch_closed_loop()
        except Exception as e:
            self.status.showMessage("Could not launch closed loop: %s" % e)
            return
        self.btn_run.setText("Stop Controller")
        self.btn_synth.setEnabled(False)      # do not re-synthesise mid-run
        self.controller_timer.start()
        self.status.showMessage("Closed loop running. Press Stop Controller to halt.")

    def stop_controller(self):
        try:
            kill_go2_controller()
        except Exception as e:
            print("kill failed: %s" % e)
        if self.controller_proc is not None:
            try:
                self.controller_proc.terminate()
            except Exception:
                pass
        self.controller_proc = None
        self.controller_timer.stop()
        self.btn_run.setText("Run Controller")
        self.btn_synth.setEnabled(True)
        self.status.showMessage("Closed loop stopped.")

    def _poll_controller(self):
        """Reset the button if the loop exited on its own."""
        if self.controller_proc is not None and self.controller_proc.poll() is not None:
            code = self.controller_proc.returncode
            self.controller_proc = None
            self.controller_timer.stop()
            self.btn_run.setText("Run Controller")
            self.btn_synth.setEnabled(True)
            self.status.showMessage("Closed loop exited (code %s)." % code)

    # -----------------------------------------------------------------------

    def start_manual_drive(self):
        global path_tail
        path_tail = []
        self.btn_manual.setEnabled(False)
        self.status.showMessage("Manual Drive Started, use arrow keys to navigate, press esc to quit")
        subprocess.Popen([sys.executable, "KeyboardControl.py"])
        keyboard.add_hotkey("esc", self.exit_manual_drive)

    def exit_manual_drive(self):
        keyboard.unhook_all_hotkeys()
        self.btn_manual.setEnabled(True)
        self.status.showMessage("Manual Drive Exited")

    def canvas_press_event(self, event):  # get object ready for mouse drag event
        if self.setup_complete:
            x = event.pos().x()
            y = event.pos().y()
            [x, y] = canvas_to_world(x, y)
            f1, f2 = world_to_fields(x, y)
            bounding_objects = self.object_manager.getObjectsBounding(f1, f2)
            if len(bounding_objects) == 1 and self.checked_object_type == "None":
                self.drag_object_name = bounding_objects[0]

    def canvas_release_event(self, event):  # mouse released event on canvas
        if self.setup_complete:
            self.drag_object_name = "None"
            if self.view_edit:
                x = event.pos().x()
                y = event.pos().y()
                [x, y] = canvas_to_world(x, y)
                if self.checked_object_type != "None" and self.drag_object_name == "None":
                    if self.textbox_width.text() == "" or self.textbox_height.text() == "":
                        width, height = default_object_sizes[self.checked_object_type]
                    else:
                        width = self.textbox_width.text()
                        height = self.textbox_height.text()
                    f1, f2 = world_to_fields(x, y)
                    values = ["0", str(f1), str(f2), "0", "0", width, height]
                    self.object_manager.addObject(self.object_manager.getValidObjectName(self.checked_object_type), ",".join(values))
                    self.status.showMessage(self.checked_object_type + " placed at x=" + str(x)[:5] + " y=" + str(y)[:5])
                    self.checked_object_type = "None"
                    self.checkbox_target.setChecked(False)
                    self.checkbox_obstacle.setChecked(False)
                    self.textbox_height.setText("")
                    self.textbox_width.setText("")
                    self.canvas.setFocus()
        self.drag_object_name = "None"

    def canvas_move_event(self, event):  # mouse move event on canvas for moving object positions
        if self.setup_complete:
            if self.drag_object_name != "None" and self.checked_object_type == "None":
                x = event.pos().x()
                y = event.pos().y()
                [x, y] = canvas_to_world(x, y)
                f1, f2 = world_to_fields(x, y)
                self.object_manager.updateObjectPosition(self.drag_object_name, f1, f2)
                self.status.showMessage(self.drag_object_name + " moved to x=" + str(x)[:5] + " y=" + str(y)[:5])

    def _shutdown_thread(self, thread, timeout_ms=3000):
        """Ask a worker to stop, then block until it actually has.

        terminate() only requests termination and returns immediately, so
        without the wait() the interpreter tears down the QThread while run()
        is still executing -- that is the "QThread: Destroyed while thread is
        still running" warning.
        """
        if thread is None or not thread.isRunning():
            return
        if hasattr(thread, "stop"):
            thread.stop()
        if not thread.wait(timeout_ms):
            thread.terminate()
            thread.wait(1000)

    def closeEvent(self, event):
        print("Quitting " + __file__)

        self._shutdown_thread(self.thread_camera)
        self._shutdown_thread(self.thread_canvas)
        self._shutdown_thread(self.thread_scots)

        try:
            self.stop_controller()
        except Exception:
            pass

        if self.setup_complete and ENABLE_TRACKING:
            # These are Windows executables reached through cmd.exe, matching
            # how start_environment launches them. A bare Popen of a .bat path
            # raises FileNotFoundError on Linux.
            try:
                subprocess.Popen(["cmd.exe", "/c", "kill_admin.bat"],
                                 cwd="/mnt/d/Workspace/OptiTrackRESTServer")
            except Exception as e:
                print("kill_admin failed: %s" % e)
            for proc in (self.motive, self.ventuz):
                if proc is not None:
                    try:
                        proc.terminate()
                    except Exception:
                        pass


if __name__ == "__main__":
    import signal
    App = QApplication(sys.argv)
    window = AutoDeploy()

    signal.signal(signal.SIGINT, lambda *a: window.close())

    # give the interpreter a slice of time so the signal handler can run
    timer = QTimer()
    timer.start(200)
    timer.timeout.connect(lambda: None)

    window.show()
    window.setMaximumSize(window.size())
    sys.exit(App.exec())