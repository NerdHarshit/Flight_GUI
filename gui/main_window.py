from PyQt6.QtWidgets import QMainWindow , QWidget , QGridLayout , QVBoxLayout

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("DJS Impulse Ground Station")
        self.setGeometry(100,100,500,400)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        self.main_layout = QVBoxLayout()
        central_widget.setLayout(self.main_layout)

        self.build_top_dashboard()

    def build_top_dashboard(self):
        self.top_grid = QGridLayout()
        self.main_layout.addLayout(self.top_grid)

        