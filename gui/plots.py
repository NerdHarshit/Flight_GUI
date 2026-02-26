import pyqtgraph as pg
from PyQt6.QtWidgets import QFrame, QVBoxLayout


class LivePlot(QFrame):
    def __init__(self, title="Live Plot", curve_names=None, y_label="Value"):
        super().__init__()

        if curve_names is None:
            curve_names = ["Data"]

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setTitle(title)
        self.plot_widget.setBackground("#1E1E1E")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend()

        # Axis labels
        self.plot_widget.setLabel("left", y_label)
        self.plot_widget.setLabel("bottom", "Time (s)")

        self.layout.addWidget(self.plot_widget)

        self.curves = {}
        self.data = {}
        self.max_points = 300

        colors = [
            (255, 80, 80),      # Red
            (80, 255, 80),      # Green
            (187, 134, 252),    # Purple
            (80, 180, 255),     # Blue
        ]

        for i, name in enumerate(curve_names):
            pen = pg.mkPen(color=colors[i % len(colors)], width=2)
            self.curves[name] = self.plot_widget.plot(name=name, pen=pen)
            self.data[name] = {"x": [], "y": []}

    def add_point(self, name, x, y):
        if name not in self.data:
            return

        self.data[name]["x"].append(x)
        self.data[name]["y"].append(y)

        if len(self.data[name]["x"]) > self.max_points:
            self.data[name]["x"].pop(0)
            self.data[name]["y"].pop(0)

        self.curves[name].setData(
            self.data[name]["x"],
            self.data[name]["y"]
        )

    def clear(self):
        for name in self.data:
            self.data[name]["x"].clear()
            self.data[name]["y"].clear()
            self.curves[name].setData([], [])