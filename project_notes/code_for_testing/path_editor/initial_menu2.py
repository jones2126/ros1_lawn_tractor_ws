import sys
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPainter, QBrush, QColor
from PyQt5.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsScene, QGraphicsView, QMainWindow, QVBoxLayout, QWidget
import pandas as pd

class PathEditor(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene, self)
        
        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.view)
        self.setCentralWidget(central_widget)

        # Load path data
        path_a = pd.read_csv('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveA.csv')
        path_b = pd.read_csv('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveB.csv')

        # file_path_a = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveA.csv'
        # file_path_b = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveB.csv'        
        
        self.loadPathData(path_a, 'red')
        self.loadPathData(path_b, 'blue')

        # Apply scaling
        self.view.scale(10, 10)  # Adjust the scaling factor as needed

        # Center the view
        self.view.centerOn(self.scene.itemsBoundingRect().center())

        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('Path Editor')
        self.show()

    def loadPathData(self, path, color):
        min_x, min_y = path['X'].min(), path['Y'].min()
        max_y = path['Y'].max()
        
        for index, row in path.iterrows():
            x, y = row['X'] - min_x, max_y - (row['Y'] - min_y)
            print(f"Adding point: {x}, {y}")  # Debug print
            
            # Adjust point size here
            circle = QGraphicsEllipseItem(x, y, 20, 20)  # Increased size
            if color == 'red':
                circle.setBrush(QBrush(QColor('red')))
            elif color == 'blue':
                circle.setBrush(QBrush(QColor('blue')))
            self.scene.addItem(circle)

        # Test circle at (0, 0)
        test_circle = QGraphicsEllipseItem(0, 0, 100, 100)
        test_circle.setBrush(QBrush(QColor('green')))
        self.scene.addItem(test_circle)

        # Adjust scene rectangle
        self.scene.setSceneRect(self.scene.itemsBoundingRect())

def main():
    app = QApplication(sys.argv)
    window = PathEditor()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
