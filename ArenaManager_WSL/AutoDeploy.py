import sys, os, math, requests, paramiko, json, keyboard, cv2, subprocess
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtTest
from PIL import Image, ImageDraw
import numpy as np

import util.ConfigReader as ConfigReader
import ArenaManager_WSL.NDIImageSender as NDIImageSender
import ArenaManager_WSL.ObjectManager as ObjectManager

CONFIG_FILE = "robot.cfg"
config_reader = ConfigReader.ConfigReader(CONFIG_FILE)

CAMERA_RESOLUTION = (1280, 720) # 16:9
ENABLE_CAMERA = False
ENABLE_PROJECTION = False

default_object_sizes = {
    # In Meters
    "GO2":("0.31", "0.70"), 
    "Target":("1", "1"), 
    "Obstacle":("3.0", "0.4")
}

# Start here!
if __name__=="__main__":
    #TODO does not work!
    ndiImgSender = NDIImageSender.NDIImageSender(b'My_PNG', 10) # initialize Ventuz projection
    #TODO does not work!
    cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
    if cap.isOpened()==False: print("connection to camera device failed")
    cap.set(3,CAMERA_RESOLUTION[0])
    cap.set(4,CAMERA_RESOLUTION[1])
    # Load background image
    img_background = Image.open("background.png")


# Read config file (robot.cfg)

def string_to_float_list(strList):
    # Takes a string like "12, 3, 5, 6" and returns a list [12,3,5,6] 
    return [float(i.replace(" ","")) for i in strList.split(",")]

# 2D State space lower bound
X_LB = string_to_float_list(config_reader.get_value_string("system.states.first_symbol"))
# 2D State space upper bound
X_UB = string_to_float_list(config_reader.get_value_string("system.states.last_symbol"))
# Grid step size
X_ETA = string_to_float_list(config_reader.get_value_string("system.states.quantizers"))


ROBOT_NAME = str(config_reader.get_value_string("optitrack.robot_name"))
LOCALIZATION_SERVER_URL = str(config_reader.get_value_string("optitrack.objects_server_url"))
SIM_WIDTH = int(config_reader.get_value_string("simulation.window_width"))
SIM_HEIGHT = int(config_reader.get_value_string("simulation.window_height"))
PATH_TAIL_LENGTH = int(config_reader.get_value_string("simulation.path_tail_length"))
DRAW_TAIL = bool(config_reader.get_value_string("simulation.DRAW_TAIL"))
DRAW_ROBOT = bool(config_reader.get_value_string("simumlation.DRAW_ROBOT"))



# run motive program
def start_motive():
    motive = subprocess.Popen("C:/Users/CUBLab/Desktop/Motive_Updated_Calibration_Dimmed_1_8_2026.lnk",stdout=subprocess.PIPE,shell=True)
    return motive

# run localization server
def start_localization_server():
    localization_server = subprocess.Popen("D:/Workspace/OptiTrackRESTServer/start_admin.bat")
    return localization_server

# run ventuz
def start_ventuz():
    ventuz = subprocess.Popen("D:/Workspace/NDIRestServer/ventuz/NDIRestServerRecveiver/Presentations/NDIRestServerReceiver.vpr",stdout=subprocess.PIPE,shell=True)
    return ventuz

# pull from the REST server where the Optitrack data is held
def get_objects():
    try:
        return json.loads(requests.get(LOCALIZATION_SERVER_URL).text)
    except Exception as e:
        print("unable to retrieve from localization server: "+str(e))

# Intended to be used with parse_objects()
def parse_dict_values(string):
    values = string.split(",")
    if len(values) != 7:
        print("Warning! There should be 7 values within the following: %s" % string )
    return values

# Intended to be used with get_objects()
# returns 3 dicts with elements formatted as such:
# "name": [id, x, y, 0, 0, w, l] 
# Only x, y, w, l really matter
def parse_objects(dict):
    robots, obstacles, targets = dict()
    for key in dict:
        if "GO2" in key:
            robots[key] = parse_dict_values(dict[key])
        if "Obstacle" in key:
            obstacles[key] = parse_dict_values(dict[key])
        if "Target" in key:
            targets[key] = parse_dict_values(dict[key])
    return robots, obstacles, targets




# function to convert a rectangle defined by (x,y) positon, (w,l) size and angle in rad to polygon coordinates
def get_polygon(x, y, w, l, angle): 
    c,s = math.cos(angle),math.sin(angle)
    coords = [(l/2.0, w/2.0), (l/2.0, -w/2.0), (-l/2.0, -w/2.0), (-l/2.0, w/2.0)]
    return [(c*x_val-s*y_val+x, s*x_val+c*y_val+y) for (x_val,y_val) in coords]

# function that returns a PIL image for projection based on location and sizes from localization server
def draw_simulation():
    global path_tail
    objects = get_objects()

    # Set path_tail to none if robot cannot be located.
    if ROBOT_NAME not in objects:
        path_tail = []

    img = img_background.copy()
    draw = ImageDraw.Draw(img)

    if objects == None: return img

    # 0 <= i <= len(object.keys())-1
    # Final value is exclusive, and i counts down to -1.
    for i in range(len(objects.keys())-1, -1, -1):

        # Define object name as key value
        name = list(objects.keys())[i]

        # Only proceed if the object is actually being tracked
        if objects[name] != "untracked":

            # The object state is written as a key value pair with the value being a list of values
            values = objects[name].split(',')
            # only proceed if there are 7 values
            if len(values) != 7: continue
            # Parse it
            x = float(values[1]); y = float(values[2]); w = float(values[6]); l = float(values[5])
            angle = (float(values[3])-math.pi/2)

            # Choose color based on what type of object it is.

            color = "#00FFFF" # default color

            if ROBOT_NAME in name or "ObstacleCar" in name:
                if not DRAW_ROBOT: continue
                # Checks the last number of its name
                if name[-1]=="1":
                    color = "#0000EE"
                elif name[-1]=="2":
                    color = "#007CE8"

            if "Obstacle" in name:
                color = "#EE0000"

            elif "Target" in name:
                color = "#00EE00"

            # Optitrack to Arena Kinematics; remember, positive X is towards the 
            # computing server and positive Y is towards the other room.    
            [x, y] = state_world_to_arena([x, y])
            [w, l] = norm_world_to_arena([w, l])

            if name == ROBOT_NAME:
                # The start of the tail (index 0) should be the robot's position (x,y)
                path_tail.insert(0,(x,y))
                # If the length of the tail is greater than the stated length, 
                # then cut the list to 0 (implicit) to PATH_TAIL_LENGTH
                if len(path_tail) > PATH_TAIL_LENGTH:
                    path_tail = path_tail[:PATH_TAIL_LENGTH]
                draw.polygon(get_polygon(x,y,w,l,angle), fill=color)
            elif name == "ObstacleCar":
                draw.polygon(get_polygon(x,y,w,l,angle), fill=color)
            else:
                draw.rounded_rectangle((x-l/2,y-w/2,x+l/2,y+w/2), fill=color, radius=10)
            # draw black dot at center
            draw.polygon(get_polygon(x,y,5,5,angle), fill="black") 

    # If path tail is not empty and DRAW_TAIL is true
    if path_tail!=[] and DRAW_TAIL:
        draw.line(path_tail, width=6, fill="#0000EE", joint="curve")

    # Return the compiled image with everything drawn on it
    return img.transpose(Image.Transpose.FLIP_TOP_BOTTOM)

def pil2pixmap(im): # function to convert pil image to pyqt pixmap
    r, g, b, a = im.split()
    im = Image.merge("RGBA", (b, g, r, a))
    data = im.tobytes("raw", "RGBA")
    qim = QImage(data, im.size[0], im.size[1], QImage.Format_ARGB32)
    pixmap = QPixmap.fromImage(qim)
    return pixmap

def norm_world_to_arena(norm): # translate the size tuple in world units to pixels
    arena_x = (norm[0])*(SIM_WIDTH/(X_UB[0] - X_LB[0]))
    arena_y = (norm[1])*(SIM_HEIGHT/(X_UB[1] - X_LB[1]))
    return [arena_x, arena_y]

def norm_arena_to_world(norm): # translate the size tuple in pixels to world units
    world_x = (norm[0])/(SIM_WIDTH/(X_UB[0] - X_LB[0]))
    world_y = (norm[1])/(SIM_HEIGHT/(X_UB[1] - X_LB[1]))
    return [world_x, world_y]

def state_world_to_arena(state): # translate the position tuple in world units to pixels
    arena_x = (state[0] - X_LB[0] + X_ETA[0]/2)*(SIM_WIDTH/(X_UB[0] - X_LB[0] + X_ETA[0]))    
    arena_y = (state[1] - X_LB[1] + X_ETA[1]/2)*(SIM_HEIGHT/(X_UB[1] - X_LB[1] + X_ETA[1]))
    return [arena_x, arena_y]

def state_arena_to_world(state): # translate the position tuple in pixels to world units
    world_x = state[0]/(SIM_WIDTH/(X_UB[0] - X_LB[0] + X_ETA[0])) + X_LB[0] - X_ETA[0]/2
    world_y = state[1]/(SIM_HEIGHT/(X_UB[1] - X_LB[1] + X_ETA[1])) + X_LB[1] - X_ETA[1]/2
    return [world_x, world_y]

class CameraThread(QThread):
    changePixmap = pyqtSignal(QPixmap)
    def run(self):
        while True:
            status, image = cap.read()
            if status:
                image = cv2.rotate(cv2.cvtColor(cv2.resize(image,(640,360)),cv2.COLOR_BGR2RGB),cv2.cv2.ROTATE_180)
                self.changePixmap.emit(pil2pixmap(Image.fromarray(image).convert("RGBA")))

class CanvasThread(QThread):
    changePixmap = pyqtSignal(QPixmap)
    def run(self):
        while True:
            img = draw_simulation()
            img_bytes = bytes(np.array(img))
            self.changePixmap.emit(pil2pixmap(img))
            ndiImgSender.send_image(img_bytes, SIM_WIDTH, SIM_HEIGHT)

# Main GUI Window; massive class, generated with PyQt5
class AutoDeploy(QMainWindow): 
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

        # Create File menu
        self.menu_file = self.menu.addMenu("File")
        # self.load_config
        self.action_load_config = QAction("Load Objects Configuration")
        self.action_load_config.setShortcut("Ctrl+O")
        self.action_load_config.triggered.connect(self.load_config)
        self.action_load_config.setEnabled(False)
        self.action_load_config.setFont(self.font)
        #self.save_config
        self.action_save_config = QAction("Save Current Objects Configuration")
        self.action_save_config.setShortcut("Ctrl+S")
        self.action_save_config.triggered.connect(self.save_config)
        self.action_save_config.setEnabled(False)
        self.action_save_config.setFont(self.font)
        #Add actions to File menu
        self.menu_file.addAction(self.action_load_config)
        self.menu_file.addAction(self.action_save_config)


        #Create Edit Menu
        self.menu_edit = self.menu.addMenu("Edit")
        #self.undo
        self.action_undo = QAction("Undo")
        self.action_undo.setShortcut("Ctrl+Z")
        self.action_undo.triggered.connect(self.undo)
        self.action_undo.setEnabled(False)
        self.action_undo.setFont(self.font)
        #self.delete_target
        self.action_delete_target = QAction("Delete Added Targets")
        self.action_delete_target.triggered.connect(self.delete_target)
        self.action_delete_target.setEnabled(False)
        self.action_delete_target.setFont(self.font)
        #self.delete_obstacle
        self.action_delete_obstacle = QAction("Delete Added Obstacles")
        self.action_delete_obstacle.triggered.connect(self.delete_obstacle)
        self.action_delete_obstacle.setEnabled(False)
        self.action_delete_obstacle.setFont(self.font)
        #self.delete_config
        self.action_delete_config = QAction("Delete All Added Objects")
        self.action_delete_config.triggered.connect(self.delete_config)
        self.action_delete_config.setEnabled(False)
        self.action_delete_config.setFont(self.font)
        # Add actions to edit menu
        self.menu_edit.addAction(self.action_undo)
        self.menu_edit.addSeparator()
        self.menu_edit.addAction(self.action_delete_target)
        self.menu_edit.addAction(self.action_delete_obstacle)
        self.menu_edit.addSeparator()
        self.menu_edit.addAction(self.action_delete_config)


        #Create View Menu
        self.menu_view = self.menu.addMenu("View")
        #self.toggle_camera
        self.action_toggle_camera = QAction("Toggle Camera View", self, checkable=True)
        self.action_toggle_camera.setFont(self.font)
        self.action_toggle_camera.triggered.connect(self.toggle_camera)
        #Set default as defined
        self.action_toggle_camera.setChecked(ENABLE_CAMERA)
        #self.toggle_edit
        self.action_toggle_edit = QAction("Toggle Edit View", self, checkable=True)
        self.action_toggle_edit.setChecked(True)
        self.action_toggle_edit.setFont(self.font)
        self.action_toggle_edit.triggered.connect(self.toggle_edit)
        #self.toggle_tail
        self.action_toggle_tail = QAction("Toggle Tail Visibility", self, checkable=True)
        self.action_toggle_tail.setChecked(False)
        self.action_toggle_tail.setFont(self.font)
        self.action_toggle_tail.triggered.connect(self.toggle_tail)
        #self.toggle_robot_render
        self.action_toggle_robot_render = QAction("Toggle Go2 Rendering", self, checkable=True)
        self.action_toggle_robot_render.setChecked(render_robot)
        self.action_toggle_robot_render.setFont(self.font)
        self.action_toggle_robot_render.triggered.connect(self.toggle_robot_render)
        #Add actions to view menu
        self.menu_view.addAction(self.action_toggle_camera)
        self.menu_view.addAction(self.action_toggle_edit)
        self.menu_view.addSeparator()
        self.menu_view.addAction(self.action_toggle_tail)
        self.menu_view.addAction(self.action_toggle_robot_render)


        #Create SCOTS Menu
        self.menu_scots = self.menu.addMenu("SCOTS")
        #self.write_config_only
        self.action_write_config_only = QAction("Write arena_config.txt Only")
        self.action_write_config_only.triggered.connect(self.write_config_only)
        self.action_write_config_only.setEnabled(False)
        self.action_write_config_only.setFont(self.font)
        #self.show_config
        self.action_show_config = QAction("Preview arena_config.txt")
        self.action_show_config.triggered.connect(self.show_config)
        self.action_show_config.setEnabled(False)
        self.action_show_config.setFont(self.font)
        #Add actions to SCOTS menu
        self.menu_scots.addAction(self.action_write_config_only)
        self.menu_scots.addAction(self.action_show_config)

        #Create Help Menu
        self.menu_help = self.menu.addMenu("Help")
        #self.stop_controler
        self.action_kill_go2_controller = QAction("Kill Go2 Controller")
        self.action_kill_go2_controller.triggered.connect(self.stop_controller)
        self.action_kill_go2_controller.setFont(self.font)
        #Add action to Help Menu
        self.menu_help.addAction(self.action_kill_go2_controller)



        #Setup buttons at the top
        self.setup_complete = False
        #self.start_environment
        self.btn_env = QPushButton("Initialize Environment", self)
        self.btn_env.clicked.connect(self.start_environment)
        self.btn_env.setFont(self.font)
        # Synthesize and Run used to be combined; it is separate in this program to allow 
        # for the running of a program twice without resynthesis.
        #self.start_symbolic
        self.btn_synth = QPushButton("Synthesize Controller", self)
        self.btn_synth.clicked.connect(self.start_symbolic)
        self.btn_synth.setFont(self.font)
        self.btn_synth.setEnabled(False)
        #self.toggle_controller
        self.btn_run = QPushButton("Run Controller", self)
        self.btn_run.clicked.connect(self.toggle_controller)
        self.btn_run.setFont(self.font)
        self.btn_run.setEnabled(False)
        #Add Buttons to top
        self.btns = [self.btn_env, self.btn_synth, self.btn_run]
        hbox_btns.addWidget(self.btn_env)
        hbox_btns.addWidget(self.btn_synth)
        hbox_btns.addWidget(self.btn_run)
        hbox_btns.addWidget(self.btn_manual)

        # poll the closed loop so the button resets if it exits on its own
        # Every 1000ms, self._poll_controller will be called
        self.controller_timer = QTimer(self)
        self.controller_timer.setInterval(1000)
        self.controller_timer.timeout.connect(self._poll_controller)

        # setup camera view
        # Only enable camera if defined as so
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
        # Methods called when canvas is interacted with
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

        # What kind of obstacle to place on the canvas?
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

        # Advanced Options
        vbox_dimensions = QVBoxLayout()
        self.label_dimensions = QLabel("Dimensions: (one block is size " + str(norm_arena_to_world([600 / 4, 0])[0])[:3] + ")")
        self.label_dimensions.setFont(self.font_bold)
        vbox_dimensions_info2 = QHBoxLayout()
        self.label_dimensions_info2 = QLabel("Leave Blank for Default Sizes")
        self.label_dimensions_info2.setFont(self.font)
        #self.clear_edit_textbox
        self.btn_edit_clear = QPushButton("Clear")
        self.btn_edit_clear.setFont(self.font)
        self.btn_edit_clear.clicked.connect(self.clear_edit_textbox)
        vbox_dimensions_info2.addWidget(self.label_dimensions_info2)
        vbox_dimensions_info2.addStretch(1)
        vbox_dimensions_info2.addWidget(self.btn_edit_clear)
        hbox_edit_line1 = QHBoxLayout()
        # Input goes to self.label_width
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
        # Input goes to self.label_height
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

        # Align Everything
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

        # Status Bar
        self.status = self.statusBar()
        self.status.setFont(self.font)
        self.status.showMessage("Initialize Environment to Run Simulation")

        # Synthesis log, hidden until the first synthesis run
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

        self.setWindowTitle("Unitree GO2 SCOTS Auto Deploy")
        self.setWindowIcon(QIcon('background.png'))
        main_widget = QWidget()
        main_widget.setLayout(vbox)
        self.setCentralWidget(main_widget)
        self.setWindowFlag(Qt.WindowMaximizeButtonHint, False)  # disable maximize button


    # Setup over; the following are method definitions

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
        global DRAW_TAIL
        DRAW_TAIL = bool(state)

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


    # Important! Launches everything else
    def start_environment(self):  # function to start necessay software environments in sequence ~30s
        self.btn_env.setEnabled(False)
        self.motive = None
        self.ventuz = None
        self.localization_server = None
        skipped = []

        # Start Motive!
        self.status.showMessage("Starting Motive...")
        self.motive = subprocess.Popen(["cmd.exe", "/c", "start", "", "Motive_Best_Calibration.lnk"], cwd="/mnt/c/Users/CUBLab/Desktop", stdout=subprocess.PIPE)
        self.activateWindow()
        QtTest.QTest.qWait(16000)

        # Start Localization Server!
        self.status.showMessage("Starting Localization Server...")
        self.localization_server = subprocess.Popen(["cmd.exe", "/c", "start_admin.bat"], cwd="/mnt/d/Workspace/OptiTrackRESTServer")
        QtTest.QTest.qWait(2000)
        self.activateWindow()

        #Start Ventuz!
        if ENABLE_PROJECTION:
            self.status.showMessage("Starting Ventuz...")
            self.ventuz = start_ventuz()
            self.activateWindow()
            QtTest.QTest.qWait(10000)
        else:
            skipped.append("Ventuz/NDI")

        # Make options available
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

        #TODO Could cause problems later... should check if file exists before running
        self.btn_run.setEnabled(self.controller_ready)

        # Referring to ObjectManager.py
        # Instantiate an ObjectManager
        self.object_manager = ObjectManager.ObjectManager()


        # This adoption block runs once, when the environment is initialized
        # If left side is true, return get_objects()
        stale = get_objects() or {}
        # Add n to the list if it satisfies the predicate, if its a Target/Obstacle 
        # and not managed by the object_manager.
        orphans = [n for n in stale
                   if ("Target" in n or "Obstacle" in n) and n not in self.object_manager.objects]
        # If non-empty, update object_manager with 'orphan' key:value pair 
        if orphans:
            self.object_manager.objects.update({n:stale[n] for n in orphans})
            print("adopted %d orphaned object(s) from a previous session: %s"
                  % (len(orphans), orphans))
            
        self.thread_canvas.start()

        # True if you can reach the objects
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

    

    def _build_config_text(self):
        """Render arena_config.txt from the GUI geometry. Raises on no target."""
        robots, targets, obstacles = parse_objects(get_objects())
        if not targets:
            raise RuntimeError("No Target placed on the arena.")
        return SCOTSDeploy.build_config_text(targets, obstacles,
                                             state_lb=X_LB, state_ub=X_UB), targets, obstacles

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
            state_lb=X_LB,
            state_ub=X_UB,
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

        if self.setup_complete:
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
    App = QApplication(sys.argv)
    window = AutoDeploy()
    window.show()
    window.setMaximumSize(window.size())
    sys.exit(App.exec())