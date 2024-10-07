from PyQt5 import QtWidgets, uic, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QMessageBox
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import sys
import layoutMain
import serial
import numpy as np
import re
import threading
import time
import pandas as pd
from datetime import datetime, timedelta

# Class to handle data processing and communication with the serial device
class DataProcessor(QtCore.QObject):
    # Define signals to communicate with the main GUI thread
    data_updated = QtCore.pyqtSignal(float, float)
    configuration_completed = QtCore.pyqtSignal()
    prompt_ired = QtCore.pyqtSignal()
    prompt_iir = QtCore.pyqtSignal()
    prompt_freq = QtCore.pyqtSignal()
    prompt_gain = QtCore.pyqtSignal()
    prompt_cap = QtCore.pyqtSignal()
    prompt_res = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(DataProcessor, self).__init__(parent)
        # Setup the serial connection
        self.ser = serial.Serial("COM3", 115200)
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.parity = serial.PARITY_NONE
        self.ser.timeout = 0.1
        self.stop_signal = threading.Event()
        self.buffer = ""

    # Function to check device configuration messages
    def check_configuration(self):
        while True:
            uart_data = self.ser.readline().decode("utf-8").strip()
            if "Configuration terminée" in uart_data:
                self.configuration_completed.emit()
                break
            elif "Entrer IRED" in uart_data:
                self.prompt_ired.emit()
            elif "Entrer IIR" in uart_data:
                self.prompt_iir.emit()
            elif "Entrer F" in uart_data:
                self.prompt_freq.emit()
            elif "Entrer G" in uart_data:
                self.prompt_gain.emit()
            elif "Entrer Rf" in uart_data:
                self.prompt_res.emit()
            elif "Entrer Cf" in uart_data:
                self.prompt_cap.emit()

    # Function to process incoming data from the serial device
    def process_data(self):
        while not self.stop_signal.is_set():
            uart_data = self.ser.read(256).decode("utf-8")
            if uart_data:
                self.buffer += uart_data
                lines = self.buffer.split('\n')
                self.buffer = lines.pop()  # The last line might be incomplete, keep it in the buffer

                for line in lines:
                    line = line.strip()
                    match = re.match(r"IRdata: (\d+\.\d+), REDdata: (\d+\.\d+)", line)
                    if match:
                        IR_data = float(match.group(1))
                        Red_data = float(match.group(2))
                        self.data_updated.emit(IR_data, Red_data)

    # Function to stop processing data and close the serial connection
    def stop_processing(self):
        self.stop_signal.set()
        self.ser.close()

    # Function to send commands to the serial device
    def send_command(self, command):
        if self.ser.is_open:
            self.ser.write(command.encode('utf-8'))
            print(f"Sent: {command.encode('utf-8')}")
        else:
            print("Serial port is not open")

# Main GUI class
class MainWindow(layoutMain.Ui_MainWindow, QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self.setWindowTitle("STM32 - AFE4490 GUI")

        # Connect buttons to their respective functions
        self.pushButton.clicked.connect(self.start_plot)
        self.pushButton_2.clicked.connect(self.stop_plot)
        self.pushButton_38.clicked.connect(self.set_red_current)
        self.pushButton_33.clicked.connect(self.set_ir_current)
        self.pushButton_5.clicked.connect(lambda: self.set_frequency(62.5))
        self.pushButton_6.clicked.connect(lambda: self.set_frequency(100))
        self.pushButton_7.clicked.connect(lambda: self.set_frequency(250))
        self.pushButton_8.clicked.connect(lambda: self.set_frequency(500))
        self.pushButton_9.clicked.connect(lambda: self.set_frequency(1000))
        self.pushButton_10.clicked.connect(lambda: self.set_frequency(5000))
        self.pushButton_11.clicked.connect(lambda: self.set_capacity(5))
        self.pushButton_12.clicked.connect(lambda: self.set_capacity(10))
        self.pushButton_13.clicked.connect(lambda: self.set_capacity(25))
        self.pushButton_14.clicked.connect(lambda: self.set_capacity(50))
        self.pushButton_15.clicked.connect(lambda: self.set_capacity(100))
        self.pushButton_16.clicked.connect(lambda: self.set_capacity(250))
        self.pushButton_17.clicked.connect(lambda: self.set_resistance(10))
        self.pushButton_18.clicked.connect(lambda: self.set_resistance(25))
        self.pushButton_19.clicked.connect(lambda: self.set_resistance(50))
        self.pushButton_20.clicked.connect(lambda: self.set_resistance(100))
        self.pushButton_21.clicked.connect(lambda: self.set_resistance(250))
        self.pushButton_22.clicked.connect(lambda: self.set_resistance(1000))
        self.pushButton_28.clicked.connect(lambda: self.set_resistance(500))
        self.pushButton_23.clicked.connect(lambda: self.set_gain(0))
        self.pushButton_24.clicked.connect(lambda: self.set_gain(3.5))
        self.pushButton_25.clicked.connect(lambda: self.set_gain(6))
        self.pushButton_26.clicked.connect(lambda: self.set_gain(9.5))
        self.pushButton_27.clicked.connect(lambda: self.set_gain(12))

        # Initialize variables for plotting
        self.max_samples = 2000
        self.sample_count = 0
        self.start_time = time.time()
        self.x = np.zeros(self.max_samples)
        self.y = np.zeros(self.max_samples)
        self.y2 = np.zeros(self.max_samples)

        self.setup_plots()

        # Setup file
        now = datetime.now()
        data_formatada = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.file = f'{data_formatada}.csv'

        # Initialize data processor and thread
        self.data_processor = DataProcessor()
        self.data_processor_thread = QtCore.QThread()
        self.data_processor.moveToThread(self.data_processor_thread)
        self.data_processor_thread.started.connect(self.data_processor.process_data)
        self.data_processor.data_updated.connect(self.update_plot_data)
        self.data_processor.configuration_completed.connect(self.show_configuration_message)
        self.data_processor.prompt_ired.connect(self.prompt_for_ired)
        self.data_processor.prompt_iir.connect(self.prompt_for_iir)
        self.data_processor.prompt_freq.connect(self.prompt_for_freq)
        self.data_processor.prompt_gain.connect(self.prompt_for_gain)
        self.data_processor.prompt_cap.connect(self.prompt_for_cap)
        self.data_processor.prompt_res.connect(self.prompt_for_res)

        # Check for configuration message on startup
        threading.Thread(target=self.data_processor.check_configuration, daemon=True).start()

        # Send initial command
        #self.command = f"pret\n"
        #self.data_processor.send_command(self.command)

    # Setup the plots
    def setup_plots(self):
        self.widget2.setBackground('w')
        self.widget2.showGrid(x=True, y=True)
        self.widget2.setTitle("PPG Signal (IR)", color='b', size='15px')
        styles = {'color': 'r', 'font-size': '12px'}
        self.widget2.setLabel('left', 'Volts', **styles)
        self.widget2.setLabel('bottom', 'Time (s)', **styles)
        self.widget2.addLegend()
        pen = pg.mkPen(color=(255, 0, 0), width=1)
        self.data_line = self.widget2.plot(self.x, self.y, pen=pen)

        self.widget2_3.setBackground('w')
        self.widget2_3.showGrid(x=True, y=True)
        self.widget2_3.setTitle("PPG Signal (RED)", color='b', size='15px')
        styles = {'color': 'r', 'font-size': '12px'}
        self.widget2_3.setLabel('left', 'Volts', **styles)
        self.widget2_3.setLabel('bottom', 'Time (s)', **styles)
        self.widget2_3.addLegend()
        pen2 = pg.mkPen(color=(255, 0, 0), width=1)
        self.data_line2 = self.widget2_3.plot(self.x, self.y2, pen=pen2)

    # Update plot data and save to CSV
    def update_plot_data(self, IR_data, Red_data):
        self.sample_count += 1
        tempo = self.sample_count / self.selected_frequency
        self.y[:-1] = self.y[1:]
        self.y2[:-1] = self.y2[1:]
        self.x[:-1] = self.x[1:]
        self.y[-1] = IR_data
        self.y2[-1] = Red_data
        self.x[-1] = tempo
        self.data_line.setData(self.x, self.y)
        self.data_line2.setData(self.x, self.y2)

        data = {'Time (s)': [tempo], 'IR_data': [IR_data], 'Red_data': [Red_data]}        
        df = pd.DataFrame(data)
        df.to_csv(self.file, sep=',', mode='a', index=False, header=False)

    # Display configuration completed message
    def show_configuration_message(self):
        mbox = QMessageBox()
        mbox.setText("Configuration terminée. Vous pouvez commencer l'acquisition")
        mbox.exec_()
    
    # Display prompt messages for various parameters
    def prompt_for_ired(self):
        mbox = QMessageBox()
        mbox.setText("Veuillez entrer la valeur de IRED")
        mbox.exec_()
    
    def prompt_for_iir(self):
        mbox = QMessageBox()
        mbox.setText("Veuillez entrer la valeur de IIR")
        mbox.exec_()
    
    def prompt_for_freq(self):
        mbox = QMessageBox()
        mbox.setText("Veuillez entrer la valeur de F")
        mbox.exec_()
    
    def prompt_for_cap(self):
        mbox = QMessageBox()
        mbox.setText("Veuillez entrer la valeur de Cf")
        mbox.exec_()
    
    def prompt_for_res(self):
        mbox = QMessageBox()
        mbox.setText("Veuillez entrer la valeur de Rf")
        mbox.exec_()
    
    def prompt_for_gain(self):
        mbox = QMessageBox()
        mbox.setText("Veuillez entrer la valeur de G")
        mbox.exec_()

    # Functions to set various parameters and send corresponding commands
    def set_red_current(self):
        red_current = self.lineEdit.text()
        self.command = f"IRED: {red_current}\n"
        self.data_processor.send_command(self.command)

    def set_ir_current(self):
        ir_current = self.lineEdit_2.text()
        self.command = f"IIR: {ir_current}\n"
        self.data_processor.send_command(self.command)

    def set_frequency(self, frequency):
        self.selected_frequency = frequency
        self.command = f"F: {self.selected_frequency}\n"
        self.data_processor.send_command(self.command)

    def set_capacity(self, capacity):
        self.selected_capacity = capacity
        self.command = f"Cf: {self.selected_capacity}\n"
        self.data_processor.send_command(self.command)
    
    def set_resistance(self, resistance):
        self.selected_resistance = resistance
        self.command = f"Rf: {self.selected_resistance}\n"
        self.data_processor.send_command(self.command)
    
    def set_gain(self, gain):
        self.selected_gain = gain
        self.command = f"G: {self.selected_gain}\n"
        self.data_processor.send_command(self.command)

    # Start and stop plotting
    def start_plot(self):
        self.data_processor_thread.start()

    def stop_plot(self):
        self.command = f"stop\n"
        self.data_processor.send_command(self.command)
    
        # Add a delay to ensure the message is sent
        time.sleep(0.1)

        self.data_processor.stop_processing()

# Main function to run the application
def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()