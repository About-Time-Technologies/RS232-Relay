import serial

ser = serial.Serial('COM4', 115200)
ser.write(b"\r")
ser.close()


