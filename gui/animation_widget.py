import pyqtgraph.opengl as gl
from PyQt6.QtWidgets import QMainWindow
import numpy as np

def create_fin():
    import numpy as np
    import pyqtgraph.opengl as gl

    vertices = np.array([
        [0,0,0],
        [0.8,0,0],
        [0.2,0,1]
    ])

    faces = np.array([
        [0,1,2]
    ])

    return gl.MeshData(vertexes=vertices, faces=faces)

def create_cone(radius=0.3, height=0.8, segments=20):
    import numpy as np
    import pyqtgraph.opengl as gl

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

        # 3D view
        self.view = gl.GLViewWidget()
        self.setCentralWidget(self.view)

        self.view.setCameraPosition(distance=30, elevation=20, azimuth=45)
        self.view.setBackgroundColor((94, 148, 189))  # sky blue
        self.create_scene()

    # ---------------------------------------------------

    def create_scene(self):

        # Ground grid
        #grid = gl.GLGridItem()
        #grid.scale(2, 2, 1)
        #sself.view.addItem(grid)

        # Rocket body (cylinder)
        md = gl.MeshData.cylinder(rows=10, cols=20, radius=[0.3, 0.3], length=3)
        cone = create_cone()

        self.trail = gl.GLLinePlotItem(color=(0.7,0.2,1,1), width=2)
        
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

        # flip it downward
        #self.nozzle.rotate(180,1,0,0)

        self.view.addItem(self.nozzle)

        fin_mesh = create_fin()

        self.fins = []

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
        
        self.rocket = gl.GLMeshItem(
            meshdata=md,
            smooth=True,
            color=(0.8, 0.8, 0.8, 1),
            shader="shaded",
            drawEdges=False
        )

        import numpy as np

        #ground plane
        plane = gl.GLMeshItem(
            vertexes=np.array([
                [-20,-20,0],
                [20,-20,0],
                [20,20,0],
                [-20,20,0]
            ]),
            faces=np.array([[0,1,2],[0,2,3]]),
            color=(0.1,0.8,0.1,1),
            shader="shaded",
            smooth=False
        )

        self.view.addItem(plane)

        self.view.addItem(self.rocket)
        self.view.addItem(self.nose)
        self.view.addItem(self.trail)


    # ---------------------------------------------------

    def update_state(self, packet):

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

        # ---------------- BODY ----------------

        self.rocket.resetTransform()

        self.rocket.rotate(self.current_pitch,1,0,0)
        self.rocket.rotate(self.current_roll,0,1,0)
        self.rocket.rotate(self.current_yaw,0,0,1)

        self.rocket.translate(0,0,z)

        # ---------------- NOSE ----------------

        self.nose.resetTransform()

        self.nose.rotate(self.current_pitch,1,0,0)
        self.nose.rotate(self.current_roll,0,1,0)
        self.nose.rotate(self.current_yaw,0,0,1)

        self.nose.translate(0,0,z+3)

        # ---------------- NOZZLE ----------------

        self.nozzle.resetTransform()

        # keep it inverted
        self.nozzle.rotate(180,1,0,0)

        self.nozzle.rotate(self.current_pitch,1,0,0)
        self.nozzle.rotate(self.current_roll,0,1,0)
        self.nozzle.rotate(self.current_yaw,0,0,1)

        self.nozzle.translate(0,0,z-0.4)

        # ---------------- FINS ----------------

        for i , fin in enumerate(self.fins):

            fin.resetTransform()

            # rotate fin around rocket axis
            fin.rotate(i*90,0,0,1)

            fin.rotate(self.current_pitch,1,0,0)
            fin.rotate(self.current_roll,0,1,0)
            fin.rotate(self.current_yaw,0,0,1)

            fin.translate(0.3,0,z)

        # ---------------- TRAIL ----------------

        self.trail_points.append([0,0,z])

        pts = np.array(self.trail_points)

        self.trail.setData(pos=pts)