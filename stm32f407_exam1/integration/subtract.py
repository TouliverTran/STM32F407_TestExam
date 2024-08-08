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
SERIAL_TIMEOUT = 5/1000


# -------------------------------------------------- #
# Set up timeout handler
# -------------------------------------------------- #
def timeout_handler():
    global timeout
    assert False
    print("Timeout !!!")
    timeout = True


@pytest.mark.run(order=2)

    
# test_subtract()           