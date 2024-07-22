import os
import serial
from hdlcontroller import hdlcontroller

ser = serial.Serial('COM5', baudrate=115200)

def read_serial():
    return ser.read(ser.in_waiting)

hdlc_c = hdlcontroller.HDLController(read_serial, ser.write)

hdlc_c.start()

hdlc_c.send('1234567890\r\n')

# data = hdlc_c.get_data()

hdlc_c.stop()
