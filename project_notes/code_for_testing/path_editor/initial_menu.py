import sys
import pandas as pd
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsView, QGraphicsEllipseItem, QVBoxLayout, QWidget
from PyQt5.QtGui import QBrush, QColor

class PathEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):

        #self.scene.setSceneRect(-50, -30, 140, 100)  # x, y, width, height
        
        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene, self)

        # Set the layout
        layout = QVBoxLayout()
        layout.addWidget(self.view)
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        file_path_a = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveA.csv'
        file_path_b = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveB.csv'

        path_a = self.loadCSV(file_path_a)
        path_b = self.loadCSV(file_path_b)

        self.loadPathData(path_a, 'red')
        self.loadPathData(path_b, 'blue')

        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('Path Editor')
        self.show()



    def loadCSV(self, file_path):
        data = pd.read_csv(file_path)
        return data

    def loadPathData(self, path, color):
        min_x, min_y = path['X'].min(), path['Y'].min()
        max_y = path['Y'].max()
        
        for index, row in path.iterrows():
            x, y = row['X'] - min_x, max_y - (row['Y'] - min_y)
            #print(f"Adding point: {x}, {y}")  # Debug print
            circle = QGraphicsEllipseItem(x, y, 10, 10)
            if color == 'red':
                circle.setBrush(QBrush(QColor('red')))
            elif color == 'blue':
                circle.setBrush(QBrush(QColor('blue')))
            self.scene.addItem(circle)
        #self.scene.setSceneRect(-50, -30, 140, 100)  # Adjust based on data range



app = QApplication(sys.argv)
window = PathEditor()
sys.exit(app.exec_())

