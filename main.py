import sys
from PyQt6.QtWidgets import QApplication
from gui.main_window import MainWindow

def load_stylesheet(app):
    with open("styles//dark.qss","r")as f:
        app.setStyleSheet(f.read())


if __name__ == "__main__":
    app = QApplication(sys.argv)

    load_stylesheet(app)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())

