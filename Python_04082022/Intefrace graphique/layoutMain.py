# -- coding: utf-8 --

# Form implementation generated from reading ui file 'layoutMain.ui'
#
# Created by: PyQt5 UI code generator 5.15.10
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(868, 930)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(0, 0, 861, 831))
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.frame_2 = QtWidgets.QFrame(self.frame)
        self.frame_2.setGeometry(QtCore.QRect(10, 10, 140, 140))
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.frame_2)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label = QtWidgets.QLabel(self.frame_2)
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap(":/icons/images/EMSE_LOGO_1.png"))
        self.label.setScaledContents(True)
        self.label.setObjectName("label")
        self.gridLayout_2.addWidget(self.label, 0, 0, 1, 1)
        self.frame_3 = QtWidgets.QFrame(self.frame)
        self.frame_3.setGeometry(QtCore.QRect(160, 10, 401, 76))
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.label_2 = QtWidgets.QLabel(self.frame_3)
        self.label_2.setGeometry(QtCore.QRect(10, 10, 441, 25))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.frame_3)
        self.label_3.setGeometry(QtCore.QRect(10, 41, 451, 25))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.frame_4 = QtWidgets.QFrame(self.frame)
        self.frame_4.setGeometry(QtCore.QRect(20, 140, 831, 211))
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.label_4 = QtWidgets.QLabel(self.frame_4)
        self.label_4.setGeometry(QtCore.QRect(10, 10, 151, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.widget2 = PlotWidget(self.frame_4)
        self.widget2.setGeometry(QtCore.QRect(0, 40, 831, 171))
        self.widget2.setObjectName("widget2")
        self.frame_5 = QtWidgets.QFrame(self.frame)
        self.frame_5.setGeometry(QtCore.QRect(10, 570, 821, 271))
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.label_8 = QtWidgets.QLabel(self.frame_5)
        self.label_8.setGeometry(QtCore.QRect(0, 180, 191, 23))
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.layoutWidget_2 = QtWidgets.QWidget(self.frame_5)
        self.layoutWidget_2.setGeometry(QtCore.QRect(210, 180, 591, 30))
        self.layoutWidget_2.setObjectName("layoutWidget_2")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.layoutWidget_2)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.pushButton_5 = QtWidgets.QPushButton(self.layoutWidget_2)
        self.pushButton_5.setObjectName("pushButton_5")
        self.horizontalLayout_4.addWidget(self.pushButton_5)
        self.pushButton_7 = QtWidgets.QPushButton(self.layoutWidget_2)
        self.pushButton_7.setObjectName("pushButton_7")
        self.horizontalLayout_4.addWidget(self.pushButton_7)
        self.pushButton_6 = QtWidgets.QPushButton(self.layoutWidget_2)
        self.pushButton_6.setObjectName("pushButton_6")
        self.horizontalLayout_4.addWidget(self.pushButton_6)
        self.pushButton_8 = QtWidgets.QPushButton(self.layoutWidget_2)
        self.pushButton_8.setObjectName("pushButton_8")
        self.horizontalLayout_4.addWidget(self.pushButton_8)
        self.pushButton_10 = QtWidgets.QPushButton(self.layoutWidget_2)
        self.pushButton_10.setObjectName("pushButton_10")
        self.horizontalLayout_4.addWidget(self.pushButton_10)
        self.pushButton_9 = QtWidgets.QPushButton(self.layoutWidget_2)
        self.pushButton_9.setObjectName("pushButton_9")
        self.horizontalLayout_4.addWidget(self.pushButton_9)
        self.label_10 = QtWidgets.QLabel(self.frame_5)
        self.label_10.setGeometry(QtCore.QRect(0, 140, 729, 28))
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(True)
        font.setWeight(75)
        self.label_10.setFont(font)
        self.label_10.setObjectName("label_10")
        self.layoutWidget_3 = QtWidgets.QWidget(self.frame_5)
        self.layoutWidget_3.setGeometry(QtCore.QRect(210, 140, 591, 30))
        self.layoutWidget_3.setObjectName("layoutWidget_3")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.layoutWidget_3)
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.pushButton_11 = QtWidgets.QPushButton(self.layoutWidget_3)
        self.pushButton_11.setObjectName("pushButton_11")
        self.horizontalLayout_5.addWidget(self.pushButton_11)
        self.pushButton_12 = QtWidgets.QPushButton(self.layoutWidget_3)
        self.pushButton_12.setObjectName("pushButton_12")
        self.horizontalLayout_5.addWidget(self.pushButton_12)
        self.pushButton_13 = QtWidgets.QPushButton(self.layoutWidget_3)
        self.pushButton_13.setObjectName("pushButton_13")
        self.horizontalLayout_5.addWidget(self.pushButton_13)
        self.pushButton_14 = QtWidgets.QPushButton(self.layoutWidget_3)
        self.pushButton_14.setObjectName("pushButton_14")
        self.horizontalLayout_5.addWidget(self.pushButton_14)
        self.pushButton_15 = QtWidgets.QPushButton(self.layoutWidget_3)
        self.pushButton_15.setObjectName("pushButton_15")
        self.horizontalLayout_5.addWidget(self.pushButton_15)
        self.pushButton_16 = QtWidgets.QPushButton(self.layoutWidget_3)
        self.pushButton_16.setObjectName("pushButton_16")
        self.horizontalLayout_5.addWidget(self.pushButton_16)
        self.label_7 = QtWidgets.QLabel(self.frame_5)
        self.label_7.setGeometry(QtCore.QRect(0, 60, 729, 28))
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.layoutWidget_4 = QtWidgets.QWidget(self.frame_5)
        self.layoutWidget_4.setGeometry(QtCore.QRect(210, 100, 591, 30))
        self.layoutWidget_4.setObjectName("layoutWidget_4")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.layoutWidget_4)
        self.horizontalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.pushButton_23 = QtWidgets.QPushButton(self.layoutWidget_4)
        self.pushButton_23.setObjectName("pushButton_23")
        self.horizontalLayout_7.addWidget(self.pushButton_23)
        self.pushButton_24 = QtWidgets.QPushButton(self.layoutWidget_4)
        self.pushButton_24.setObjectName("pushButton_24")
        self.horizontalLayout_7.addWidget(self.pushButton_24)
        self.pushButton_25 = QtWidgets.QPushButton(self.layoutWidget_4)
        self.pushButton_25.setObjectName("pushButton_25")
        self.horizontalLayout_7.addWidget(self.pushButton_25)
        self.pushButton_26 = QtWidgets.QPushButton(self.layoutWidget_4)
        self.pushButton_26.setObjectName("pushButton_26")
        self.horizontalLayout_7.addWidget(self.pushButton_26)
        self.pushButton_27 = QtWidgets.QPushButton(self.layoutWidget_4)
        self.pushButton_27.setObjectName("pushButton_27")
        self.horizontalLayout_7.addWidget(self.pushButton_27)
        self.label_9 = QtWidgets.QLabel(self.frame_5)
        self.label_9.setGeometry(QtCore.QRect(0, 100, 39, 28))
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(True)
        font.setWeight(75)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.layoutWidget_5 = QtWidgets.QWidget(self.frame_5)
        self.layoutWidget_5.setGeometry(QtCore.QRect(210, 60, 321, 30))
        self.layoutWidget_5.setObjectName("layoutWidget_5")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.layoutWidget_5)
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.layoutWidget_5)
        self.lineEdit_2.setText("")
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.horizontalLayout_8.addWidget(self.lineEdit_2)
        self.pushButton_33 = QtWidgets.QPushButton(self.layoutWidget_5)
        self.pushButton_33.setObjectName("pushButton_33")
        self.horizontalLayout_8.addWidget(self.pushButton_33)
        self.label_6 = QtWidgets.QLabel(self.frame_5)
        self.label_6.setGeometry(QtCore.QRect(0, 20, 489, 28))
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.layoutWidget_6 = QtWidgets.QWidget(self.frame_5)
        self.layoutWidget_6.setGeometry(QtCore.QRect(210, 20, 321, 41))
        self.layoutWidget_6.setObjectName("layoutWidget_6")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.layoutWidget_6)
        self.horizontalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.lineEdit = QtWidgets.QLineEdit(self.layoutWidget_6)
        self.lineEdit.setToolTip("")
        self.lineEdit.setAccessibleDescription("")
        self.lineEdit.setAutoFillBackground(False)
        self.lineEdit.setInputMask("")
        self.lineEdit.setText("")
        self.lineEdit.setClearButtonEnabled(True)
        self.lineEdit.setObjectName("lineEdit")
        self.horizontalLayout_9.addWidget(self.lineEdit)
        self.pushButton_38 = QtWidgets.QPushButton(self.layoutWidget_6)
        self.pushButton_38.setObjectName("pushButton_38")
        self.horizontalLayout_9.addWidget(self.pushButton_38)
        self.label_12 = QtWidgets.QLabel(self.frame_5)
        self.label_12.setGeometry(QtCore.QRect(0, 220, 191, 23))
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(True)
        font.setWeight(75)
        self.label_12.setFont(font)
        self.label_12.setObjectName("label_12")
        self.layoutWidget_7 = QtWidgets.QWidget(self.frame_5)
        self.layoutWidget_7.setGeometry(QtCore.QRect(210, 220, 591, 31))
        self.layoutWidget_7.setObjectName("layoutWidget_7")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.layoutWidget_7)
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.pushButton_17 = QtWidgets.QPushButton(self.layoutWidget_7)
        self.pushButton_17.setObjectName("pushButton_17")
        self.horizontalLayout_6.addWidget(self.pushButton_17)
        self.pushButton_18 = QtWidgets.QPushButton(self.layoutWidget_7)
        self.pushButton_18.setObjectName("pushButton_18")
        self.horizontalLayout_6.addWidget(self.pushButton_18)
        self.pushButton_19 = QtWidgets.QPushButton(self.layoutWidget_7)
        self.pushButton_19.setObjectName("pushButton_19")
        self.horizontalLayout_6.addWidget(self.pushButton_19)
        self.pushButton_20 = QtWidgets.QPushButton(self.layoutWidget_7)
        self.pushButton_20.setObjectName("pushButton_20")
        self.horizontalLayout_6.addWidget(self.pushButton_20)
        self.pushButton_21 = QtWidgets.QPushButton(self.layoutWidget_7)
        self.pushButton_21.setObjectName("pushButton_21")
        self.horizontalLayout_6.addWidget(self.pushButton_21)
        self.pushButton_28 = QtWidgets.QPushButton(self.layoutWidget_7)
        self.pushButton_28.setObjectName("pushButton_28")
        self.horizontalLayout_6.addWidget(self.pushButton_28)
        self.pushButton_22 = QtWidgets.QPushButton(self.layoutWidget_7)
        self.pushButton_22.setObjectName("pushButton_22")
        self.horizontalLayout_6.addWidget(self.pushButton_22)
        self.label_5 = QtWidgets.QLabel(self.frame_5)
        self.label_5.setGeometry(QtCore.QRect(560, 30, 211, 21))
        self.label_5.setObjectName("label_5")
        self.label_13 = QtWidgets.QLabel(self.frame_5)
        self.label_13.setGeometry(QtCore.QRect(560, 60, 191, 16))
        self.label_13.setObjectName("label_13")
        self.widget2_3 = PlotWidget(self.frame)
        self.widget2_3.setGeometry(QtCore.QRect(20, 400, 831, 171))
        self.widget2_3.setObjectName("widget2_3")
        self.frame_6 = QtWidgets.QFrame(self.frame)
        self.frame_6.setGeometry(QtCore.QRect(20, 360, 831, 221))
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.label_11 = QtWidgets.QLabel(self.frame_6)
        self.label_11.setGeometry(QtCore.QRect(10, 10, 151, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_11.setFont(font)
        self.label_11.setObjectName("label_11")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(330, 840, 391, 41))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icons/images/check.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton.setIcon(icon)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_2.setFont(font)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/icons/images/close.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_2.setIcon(icon1)
        self.pushButton_2.setObjectName("pushButton_2")
        self.horizontalLayout.addWidget(self.pushButton_2)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 868, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_2.setText(_translate("MainWindow", "STM32 - AFE4490 PCB GUI"))
        self.label_3.setText(_translate("MainWindow", "Oxymetry Project 2024"))
        self.label_4.setText(_translate("MainWindow", "IR LED Graph :"))
        self.label_8.setText(_translate("MainWindow", "Frequency (Hz) :"))
        self.pushButton_5.setText(_translate("MainWindow", "62.5"))
        self.pushButton_7.setText(_translate("MainWindow", "250"))
        self.pushButton_6.setText(_translate("MainWindow", "100"))
        self.pushButton_8.setText(_translate("MainWindow", "500"))
        self.pushButton_10.setText(_translate("MainWindow", "5k"))
        self.pushButton_9.setText(_translate("MainWindow", "1k"))
        self.label_10.setText(_translate("MainWindow", "Capacitor Cf (pF):"))
        self.pushButton_11.setText(_translate("MainWindow", "5"))
        self.pushButton_12.setText(_translate("MainWindow", "10"))
        self.pushButton_13.setText(_translate("MainWindow", "25"))
        self.pushButton_14.setText(_translate("MainWindow", "50"))
        self.pushButton_15.setText(_translate("MainWindow", "100"))
        self.pushButton_16.setText(_translate("MainWindow", "250"))
        self.label_7.setText(_translate("MainWindow", "Set IR Led Current (mA): "))
        self.pushButton_23.setText(_translate("MainWindow", "0"))
        self.pushButton_24.setText(_translate("MainWindow", "3.5"))
        self.pushButton_25.setText(_translate("MainWindow", "6"))
        self.pushButton_26.setText(_translate("MainWindow", "9.5"))
        self.pushButton_27.setText(_translate("MainWindow", "12"))
        self.label_9.setText(_translate("MainWindow", "Gain:"))
        self.pushButton_33.setText(_translate("MainWindow", "SET"))
        self.label_6.setText(_translate("MainWindow", "Set RED Led Current (mA): "))
        self.pushButton_38.setText(_translate("MainWindow", "SET"))
        self.label_12.setText(_translate("MainWindow", "Rf (k ohm):"))
        self.pushButton_17.setText(_translate("MainWindow", "10"))
        self.pushButton_18.setText(_translate("MainWindow", "25"))
        self.pushButton_19.setText(_translate("MainWindow", "50"))
        self.pushButton_20.setText(_translate("MainWindow", "100"))
        self.pushButton_21.setText(_translate("MainWindow", "250"))
        self.pushButton_28.setText(_translate("MainWindow", "500"))
        self.pushButton_22.setText(_translate("MainWindow", "1000"))
        self.label_5.setText(_translate("MainWindow", "Values under 75mA "))
        self.label_13.setText(_translate("MainWindow", "(Enter only the value)"))
        self.label_11.setText(_translate("MainWindow", "RED LED Graph:"))
        self.pushButton.setText(_translate("MainWindow", "START Acqusition"))
        self.pushButton_2.setText(_translate("MainWindow", "STOP Acquisition"))
from pyqtgraph import PlotWidget
import icons_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())