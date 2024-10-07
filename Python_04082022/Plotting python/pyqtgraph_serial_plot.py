from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
from random import randint
import numpy as np
import serial


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.ser = serial.Serial("COM5", 38400)
        self.ser.readline()  #  Dummy read 

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        # Data to plot
        self.x = []
        self.y = []
        self.cnt = 0

        self.graphWidget.setBackground('w')
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setTitle("PPG Signal", color='b', size='15px')
        styles = {'color':'r', 'font-size':'20px'}
        self.graphWidget.setLabel('left', 'ADC Value', **styles)
        self.graphWidget.setLabel('bottom', 'Time', **styles)
        self.graphWidget.addLegend()

        pen = pg.mkPen(color=(255,0,0), width=3)
        self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    # def update_plot_data(self):

    #     self.cnt = self.cnt + 1
    #     if self.cnt > 500:
    #         self.x = self.x[1:]   # Remove first element
    #         self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last

    #         self.y = self.y[1:]   # Remove first element
    #         self.IR = self.ser.readline()
    #         self.y.append(int(self.IR))
    #         self.ydata = np.array(self.y, dtype='float64')
    #         self.data_line.setData(self.x, self.ydata)  # Update the data 
    #     else:
    #         self.x.append(self.cnt)
    #         self.IR = self.ser.readline()
    #         self.y.append(self.IR)
    #         self.ydata = np.array(self.y, dtype='float64')
    #         self.data_line.setData(self.x, self.ydata)  # Update the data

    def update_plot_data(self):

        self.IR = self.ser.readline()
        if(int(self.IR) > 5000 and int(self.IR) < 2097155):
            self.cnt = self.cnt + 1
            if self.cnt > 500:
                self.x = self.x[1:]   # Remove first element
                self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last

                self.y = self.y[1:]   # Remove first element
                self.y.append(int(self.IR))
                self.ydata = np.array(self.y, dtype='float64')
                self.data_line.setData(self.x, self.ydata)  # Update the data 
            else:
                self.x.append(self.cnt)
                self.y.append(int(self.IR))
                self.ydata = np.array(self.y, dtype='float64')
                self.data_line.setData(self.x, self.ydata)  # Update the data



def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()