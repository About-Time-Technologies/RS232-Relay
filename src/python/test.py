# Python code transmits a byte to Arduino /Microcontroller
import serial
import time
SerialObj = serial.Serial('COM6') # COMxx  format on Windows
                  # ttyUSBx format on Linux
SerialObj.baudrate = 115200  # set Baud rate to 9600
SerialObj.bytesize = 8   # Number of data bits = 8
SerialObj.parity  ='N'   # No parity
SerialObj.stopbits = 1   # Number of Stop bits = 1
time.sleep(1)


SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:+\r')    #transmit 'A' (8bit) to micro/Arduino


SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino
SerialObj.write(b'03:-\r')    #transmit 'A' (8bit) to micro/Arduino


SerialObj.write(b'03:?\r')    #transmit 'A' (8bit) to micro/Arduino
time.sleep(0.2)
print(SerialObj.read_all())
SerialObj.close()      # Close the port