from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph.ptime import time
import numpy as np
import serial

app = QtGui.QApplication([])

N = 0

p = pg.plot()
p.setWindowTitle('live plot from serial')
curve = p.plot()

data = [0]
raw=serial.Serial("COM4",38400)

def update():
	global curve, data, N
	N = N + 1
	if N > 500:
		data = data[1:]
		line = raw.readline()
		data.append(int(line))
		xdata = np.array(data, dtype='float64')
		curve.setData(xdata)
		app.processEvents()
	else:
		line = raw.readline()
		data.append(int(line))
		xdata = np.array(data, dtype='float64')
		curve.setData(xdata)
		app.processEvents()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)

if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()