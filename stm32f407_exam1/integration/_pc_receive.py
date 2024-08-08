#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This script needs some external modules. To install them:

::

    pip install python4yahdlc[examples]

To create a virtual serial bus, you can use socat as follows:

::

    socat -d -d pty,raw,echo=0 pty,raw,echo=0

Then, edit `SERIAL_PORT` accordingly.
"""

from sys import exit as sys_exit
from sys import stderr
from time import sleep
import threading

import serial

from yahdlc import (
    FRAME_ACK,
    FRAME_DATA,
    FRAME_NACK,
    FCSError,
    MessageError,
    frame_data,
    get_data,
)
# -------------------------------------------------- #
# Global variable
# -------------------------------------------------- #
timeout = False
# -------------------------------------------------- #
# Set up timeout handler
# -------------------------------------------------- #
def timeout_handler():
    global timeout
    print("Timeout signal received")
    timeout = True

# Set a timer for 1 second
timer = threading.Timer(10, timeout_handler)
timer.start()
# -------------------------------------------------- #
# Serial port configuration
# -------------------------------------------------- #
SERIAL_PORT = "COM5"
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0

def test_receive():
    global timeout
    # -------------------------------------------------- #
    # Open serial port
    # -------------------------------------------------- #
    print("[*] Connection...")

    try:
        with serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
            # -------------------------------------------------- #
            # Wait for HDLC frame
            # -------------------------------------------------- #
            print("[*] Waiting for data...")

            while timeout == False:
                try:
                    # 200 µs
                    sleep(200 / 1000000.0)
                    data, ftype, seq_no = get_data(ser.read(ser.in_waiting))   
                    print(f'data: {data} - ftype: {ftype} - seq_no: {seq_no}')
                    break
                except MessageError:
                    # No HDLC frame detected.
                    pass
                except FCSError:
                    stderr.write("[x] Bad FCS\n")

                    print("[*] Sending NACK...")
                    ser.write(frame_data("", FRAME_NACK, 0))
                    sys_exit(0)
                except KeyboardInterrupt:
                    print("[*] Bye!")
                    sys_exit(0)

            # -------------------------------------------------- #
            # Handle HDLC frame received
            # -------------------------------------------------- #
            if timeout != True:
                FRAME_ERROR = False

                assert ftype == FRAME_DATA
                if ftype != FRAME_DATA:
                    stderr.write(f"[x] Bad frame type: {ftype}\n")
                    FRAME_ERROR = True
                else:
                    print("[*] Data frame received")

                assert seq_no == 0
                if seq_no != 0:
                    stderr.write(f"[x] Bad sequence number: {seq_no}\n")
                    FRAME_ERROR = True
                else:
                    print("[*] Sequence number OK")

                if FRAME_ERROR is False:
                    print("[*] Sending ACK ...")
                    ser.write(frame_data("", FRAME_ACK, 1))
                else:
                    print("[*] Sending NACK ...")
                    ser.write(frame_data("", FRAME_NACK, 0))
    except serial.SerialException as err:
        sys_exit(f"[x] Serial connection problem: {err}")