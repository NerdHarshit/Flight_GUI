import pyqtgraph as pg
from PyQt6.QtWidgets import QFrame, QVBoxLayout
import pyqtgraph.exporters

class LivePlot(QFrame):
    def __init__(self, title="Live Plot", curve_names=None, y_label="Value", colors=None):
        super().__init__()

        if curve_names is None:
            curve_names = ["Data"]

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setTitle(title)
        self.plot_widget.setBackground("#0b0c10")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.addLegend()

        # Axis labels
        self.plot_widget.setLabel("left", y_label)
        self.plot_widget.setLabel("bottom", "Time (s)")

        self.layout.addWidget(self.plot_widget)

        self.curves = {}
        self.data = {}
        self.max_points = 300

        _default_colors = [
            (255, 80, 80),      # Red
            (80, 255, 80),      # Green
            (187, 134, 252),    # Purple
            (80, 180, 255),     # Blue
        ]
        palette = colors if colors else _default_colors

        for i, name in enumerate(curve_names):
            pen = pg.mkPen(color=palette[i % len(palette)], width=2)
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

    def save_plot(self,filename):
        exporter = pg.exporters.ImageExporter(self.plot_widget.plotItem)
        exporter.parameters()['width'] = 800
        exporter.export(filename)