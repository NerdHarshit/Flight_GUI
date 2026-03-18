import pyqtgraph.opengl as gl
from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtCore import QTimer
import numpy as np
import cv2

def create_fin():
    import numpy as np
    import pyqtgraph.opengl as gl

    vertices = np.array([
        [-0.1,0,0],
        [0.7,0,0],
        [0.1,0,1]
    ])

    faces = np.array([
        [0,1,2]
    ])

    return gl.MeshData(vertexes=vertices, faces=faces)

def create_cone(radius=0.3, height=0.8, segments=20):
    vertices = []
    faces = []

    # tip of cone
    vertices.append([0, 0, height])

    # base circle
    for i in range(segments):
        angle = 2 * np.pi * i / segments
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        vertices.append([x, y, 0])

    # center of base
    vertices.append([0, 0, 0])

    tip_index = 0
    base_center_index = len(vertices) - 1

    # side faces
    for i in range(1, segments + 1):
        next_i = 1 if i == segments else i + 1
        faces.append([tip_index, i, next_i])

    # base faces
    for i in range(1, segments + 1):
        next_i = 1 if i == segments else i + 1
        faces.append([base_center_index, next_i, i])

    vertices = np.array(vertices)
    faces = np.array(faces)

    return gl.MeshData(vertexes=vertices, faces=faces)

class AnimationWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Rocket Animation")
        self.setGeometry(200, 200, 900, 700)

        self.current_height = 0
        self.current_pitch = 0
        self.current_roll = 0
        self.current_yaw = 0
        self.trail_points = []

        #this part for mp4 recording
        self.recording = False
        self.frames = []
        self.latest_packet = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.render_frame)
        self.timer.start(33) #30fps

        # 3D view
        self.view = gl.GLViewWidget()
        self.setCentralWidget(self.view)

        self.view.setCameraPosition(distance=30, elevation=20, azimuth=45)
        self.view.setBackgroundColor((94, 148, 189))  # sky blue
        self.create_scene()

    # ---------------------------------------------------

    def create_scene(self):

        # Rocket body (cylinder)
        md = gl.MeshData.cylinder(rows=10, cols=20, radius=[0.3, 0.3], length=3)
        cone = create_cone()

        self.trail = gl.GLLinePlotItem(color=(0.7,0.2,1,1), width=2)
        
        #nose mesh
        self.nose = gl.GLMeshItem(
            meshdata = cone,
            smooth = True,
            color=(0.8,0.8,0.8,1),
            shader = "shaded"
        )

        self.nose.translate(0,0,3)

        # engine nozzle
        nozzle_mesh = create_cone(radius=0.18, height=0.4)

        self.nozzle = gl.GLMeshItem(
            meshdata=nozzle_mesh,
            smooth=True,
            color=(0.4,0.4,0.4,1),
            shader="shaded"
        )

        # place at bottom of rocket
        self.nozzle.translate(0,0,-0.1)

        self.view.addItem(self.nozzle)

        fin_mesh = create_fin()

        self.fins = []

        #fins mesh and creation init
        for i in range(4):

            fin = gl.GLMeshItem(
                meshdata=fin_mesh,
                smooth=False,
                color=(0.6,0.6,0.6,1),
                shader="shaded"
            )

            # rotate fins around rocket
            fin.rotate(i*90,0,0,1)

            # move to rocket base
            fin.translate(0,0,0)

            self.view.addItem(fin)

            self.fins.append(fin)

        #loop over
        
        #rocket body mesh
        self.rocket = gl.GLMeshItem(
            meshdata=md,
            smooth=True,
            color=(0.8, 0.8, 0.8, 1),
            shader="shaded",
            drawEdges=False
        )

        #ground plane mesh
        plane = gl.GLMeshItem(
            vertexes=np.array([
                [-20,-20,0],
                [20,-20,0],
                [20,20,0],
                [-20,20,0]
            ]),
            faces=np.array([[0,1,2],[0,2,3]]),
            color=(0.1,0.8,0.1,1),#green
            shader="shaded",
            smooth=False
        )

        #parachute mesh
        self.parachute = gl.GLMeshItem(
            meshdata = create_cone(radius=2, height=1.5, segments=30),
            smooth = True,
            color=(1,0.2,0.2,0.8),#red
            shader = "shaded"
        )

        self.parachute.translate(0,0,5)
        self.parachute.setVisible(False)

        #launch rail mesh
        self.rail = gl.GLLinePlotItem(
            antialias = True,
            pos = np.array([[0,0,0], [0,0,7]]),
            color=(0.0, 0.0, 0.0, 1),
            width=5,
        )

        self.rail.translate(0.3,0.1,-0.1)

        self.view.addItem(plane)
        self.view.addItem(self.rocket)
        self.view.addItem(self.nose)
        self.view.addItem(self.trail)
        self.view.addItem(self.rail)
        self.view.addItem(self.parachute)


    # ---------------------------------------------------

    # ✅ CHANGE: update_state now ONLY stores latest packet
    def update_state(self, packet):
        self.latest_packet = packet

    # ✅ NEW FUNCTION: runs at 30 FPS (called by QTimer)
    def render_frame(self):

        if self.latest_packet is None:
            return

        packet = self.latest_packet

        target_height = packet["H_baro"]
        target_pitch = packet["Gy"]
        target_roll = packet["Gx"]
        target_yaw = packet["Gz"]

        # smoothing
        alpha = 0.2

        self.current_height = (1-alpha)*self.current_height + alpha*target_height
        self.current_pitch = (1-alpha)*self.current_pitch + alpha*target_pitch
        self.current_roll = (1-alpha)*self.current_roll + alpha*target_roll
        self.current_yaw = (1-alpha)*self.current_yaw + alpha*target_yaw

        z = self.current_height * 0.02

        pitch = self.current_pitch * 0.3
        roll  = self.current_roll * 0.3
        yaw   = self.current_yaw * 0.3

        # ---------- COMMON TRANSFORM ----------
        def apply_transform(item, offset):
            item.resetTransform()
            item.translate(offset[0], offset[1], offset[2])
            item.rotate(pitch, 1, 0, 0)
            item.rotate(roll, 0, 1, 0)
            item.rotate(yaw, 0, 0, 1)
            item.translate(0, 0, z)

        # body + nose
        apply_transform(self.rocket, [0, 0, 0])
        apply_transform(self.nose, [0, 0, 3])

        # nozzle
        self.nozzle.resetTransform()
        self.nozzle.translate(0, 0, -0.4)
        self.nozzle.rotate(180, 1, 0, 0)
        self.nozzle.rotate(pitch, 1, 0, 0)
        self.nozzle.rotate(roll, 0, 1, 0)
        self.nozzle.rotate(yaw, 0, 0, 1)
        self.nozzle.translate(0, 0, z)

        # fins
        for i, fin in enumerate(self.fins):

            angle = i * 90
            fin.resetTransform()

            fin.rotate(angle, 0, 0, 1)
            fin.rotate(pitch, 1, 0, 0)
            fin.rotate(roll, 0, 1, 0)
            fin.rotate(yaw, 0, 0, 1)

            rad = np.radians(angle)
            x = 0.3 * np.cos(rad)
            y = 0.3 * np.sin(rad)

            fin.translate(x, y, z)

        # trail
        self.trail_points.append([0, 0, z])
        self.trail.setData(pos=np.array(self.trail_points))

        # parachute
        state = packet["FSM"]

        if state in [4,5,6,7]:
            self.parachute.setVisible(True)
            self.parachute.resetTransform()
            self.parachute.translate(0, 0, z + 4)
        else:
            self.parachute.setVisible(False)

        # ✅ IMPORTANT: RECORDING HAPPENS HERE
        if self.recording:
            img = self.view.grabFramebuffer()
            self.frames.append(img)

    def save_video(self,filename = "flight.mp4"):
        if not self.frames:
            return
        
        w = self.frames[0].width()
        h = self.frames[0].height()
        print(len(self.frames))

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(filename,fourcc,30,(w,h))

        for img in self.frames:
            ptr = img.bits()
            ptr.setsize(img.sizeInBytes())
            frame = np.array(ptr).reshape(h,w,4)
            frame = cv2.cvtColor(frame,cv2.COLOR_RGBA2BGR)
            out.write(frame)
        
        out.release()