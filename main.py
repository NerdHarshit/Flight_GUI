import sys
from PyQt6.QtWidgets import QApplication , QMainWindow , QWidget , QGridLayout , QFrame

app = QApplication(sys.argv)

app.setStyleSheet("""
QMainWindow {
    background-color: #121212;
}

QFrame {
    background-color: #1E1E1E;
    border-radius: 10px;
    padding: 10px;
}
""")

win = QMainWindow()
win.setWindowTitle("PyQt6 window")
win.setGeometry(100,100,400,400)

central_widget = QWidget()
win.setCentralWidget(central_widget)

layout = QGridLayout()
central_widget.setLayout(layout)

box1 = QFrame()
box2 = QFrame()
box3 = QFrame()
box4 = QFrame()

box1.setStyleSheet("background-color: red")
box2.setStyleSheet("background-color: green")
box3.setStyleSheet("background-color: blue")
box4.setStyleSheet("background-color: yellow")

layout.addWidget(box1,0,0)
layout.addWidget(box2,0,1)
layout.addWidget(box3,1,0)
layout.addWidget(box4,1,1)

win.show()

sys.exit(app.exec())
