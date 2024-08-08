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
import os
import signal
from sys import exit as sys_exit
from time import sleep
import threading
import pytest

import serial
import re

from yahdlc import (
    FRAME_ACK,          #1
    FRAME_DATA,         #0
    FRAME_NACK,         #2
    FCSError,
    MessageError,
    frame_data,
    get_data,
)

FTYPE = ('FRAME_DATA', 'FRAME_ACK', 'FRAME_NACK')
# -------------------------------------------------- #
# Global variable
# -------------------------------------------------- #
timeout = False
# -------------------------------------------------- #
# Serial port configuration
# -------------------------------------------------- #
SERIAL_PORT = "COM5"
SERIAL_BAUDRATE = 115200



# -------------------------------------------------- #
# Set up timeout handler
# -------------------------------------------------- #
def timeout_handler():
    global timeout
    print("Timeout !!!")
    timeout = True
    assert timeout == False

def open_serial_port(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Opened serial port {port} successfully.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        return None

def close_serial_port(ser):
    try:
        if ser and ser.is_open:
            ser.close()
            print("Closed serial port successfully.")
    except serial.SerialException as e:
        print(f"Error closing serial port: {e}")
        
def calculate_test(test_case: str):
    sleep(0.1) 
    pattern = r"([0-9]*)\s*Tests\s*([0-9]*)\s*Failures\s*([0-9]*)\s*Ignored"
    regex = re.compile(pattern)
    global timeout
    # Set a timer for 1 second
    timer = threading.Timer(1, timeout_handler)
    timer.start()


    # -------------------------------------------------- #
    # Open serial port
    # -------------------------------------------------- #
    print("[*] Connection...")
    ser = open_serial_port(SERIAL_PORT, SERIAL_BAUDRATE)
    if ser is None:
        pytest.skip("Skipping this test because can not open serial port")

    try:
        # -------------------------------------------------- #
        # Send HDLC frame
        # -------------------------------------------------- #
        print("[*] Sending data frame...")
        ser.write(frame_data(test_case))

        # -------------------------------------------------- #
        # Wait for (N)ACK
        # -------------------------------------------------- #
        print("[*] Waiting for (N)ACK...")
        while timeout == False:
            try:
                # 200 µs.
                sleep(200 / 1000000.0)
                data, ftype, seq_no = get_data(ser.read(ser.in_waiting))
                break
            except MessageError:
                # No HDLC frame detected.
                pass
            except FCSError:
                sys_exit("[x] Bad FCS")
            except KeyboardInterrupt:
                print("[*] Bye!")
                sys_exit(0)
    except serial.SerialException as err:
        sys_exit(f"[x] Serial connection problem: {err}")

    # -------------------------------------------------- #
    # Handle response
    # -------------------------------------------------- #
    if timeout != True:
        assert ftype in (FRAME_ACK, FRAME_NACK)
        if ftype not in (FRAME_ACK, FRAME_NACK):
            sys_exit(f"[x] Bad frame type: {ftype}")
        elif ftype == FRAME_ACK:
            print("[*] ACK received")
            assert seq_no == 1
            if seq_no != 1:
                sys_exit(f"[x] Bad sequence number: {seq_no}")
            else:
                print("[*] Sequence number OK")
        else:
            print("[*] NACK received")
            assert seq_no == 1
            if seq_no != 0:
                sys_exit(f"[x] Bad sequence number: {seq_no}")
            else:
                print("[*] Sequence number OK")
                
    # # -------------------------------------------------- #
    # # Handle test result
    # # -------------------------------------------------- #
    timeout = False;
    timer.cancel()
    timer = threading.Timer(5, timeout_handler)
    timer.start()
    try:
        # -------------------------------------------------- #
        # Wait for HDLC frame
        # -------------------------------------------------- #
        print("[*] Waiting for test result...")
        buffer = bytearray()
        start_record = False

        while timeout == False:
            try:
                # 200 µs
                sleep(200 / 1000000.0)
                buf = ser.read(ser.in_waiting)
                if start_record == True:
                    buffer.extend(buf)
                if 0x7E in buf:
                    if start_record == True:
                        buffer.extend(buf)
                        # print('\r\n=============================================\r\n')
                        # print(buffer)
                        # print('\r\n=============================================\r\n')
                        data, ftype, seq_no = get_data(bytes(buffer))  
                        result = data.decode("utf-8") 
                        print(f'\r\n- data: \r\n{result}\r\n- ftype: {FTYPE[ftype]}\r\n- seq_no: {seq_no}')
                        result = result.rsplit("\n")
                        for result_ in result:
                            match = regex.search(result_)
                            if match:
                                tests_passed = match.group(1)
                                tests_failed = match.group(2)
                                tests_ignored = match.group(3)
                                # print(f'\r\nresult: {match.group(0)} \r\nPASS: {match.group(1)}\r\nFAIL: {match.group(2)}\r\nIGNORED: {match.group(3)}')
                                assert int(tests_failed) == 0
                        break
                    if start_record == False:
                        start_record = True
                        buffer.extend(buf)
            except MessageError:
                # No HDLC frame detected.
                pass
            except FCSError:
                stderr.write("[x] Bad FCS\n")

                print("[*] Sending NACK...")
                ser.write(frame_data("", FRAME_NACK, seq_no))
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

            assert seq_no == 2
            if seq_no != 2:
                stderr.write(f"[x] Bad sequence number: {seq_no}\n")
                FRAME_ERROR = True
            else:
                print("[*] Sequence number OK")

            if FRAME_ERROR is False:
                print("[*] Sending ACK ...")
                timer.cancel()
                ser.write(frame_data("", FRAME_ACK, seq_no+1))
            else:
                print("[*] Sending NACK ...")
                ser.write(frame_data("", FRAME_NACK, seq_no))
    except serial.SerialException as err:
        sys_exit(f"[x] Serial connection problem: {err}")
    close_serial_port(ser)
    sleep(1)

def test_add():
    calculate_test('tc_add') 
    
    
def test_subtract():
    calculate_test('tc_subtract')  
    
  
def test_multiply():
    calculate_test('tc_multiply')  
    

def test_divide():
    calculate_test('tc_divide')
    
# test_add()