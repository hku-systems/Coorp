#!/usr/bin/python
from ctypes import *
import os
from fcntl import ioctl

import sys, getopt
import time


ETH_ALEN = 6

path = "/dev/flow_valve"

IOCTL_SYNC = 2148038145
IOCTL_SETVALVE = 1074296322
VALVE_STARTPROTECT = 1074296323
VALVE_RENEWPROTECT = 1074296324
VALVE_ENDPROTECT = 30213

class timespec(Structure):
    _fields_ = [
        ('tv_sec', c_longlong),
        ('tv_ns', c_long)
    ]

class flow_tsf(Structure):
    _fields_ = [
        ('ts_beacon', c_uint64),
        ('ts_kernel', c_int64),
        ('ts_system', timespec)
    ]

    def send(self):
        return create_string_buffer(self)[:]

    def receive(self, bytes):
        fit = min(len(bytes), sizeof(self))
        memmove(addressof(self), bytes, fit)

class struct_plan(Structure):
    _fields_ = [
        ('q_minus_delta_b', c_longlong),
        ('q_plus_delta_b', c_longlong)
    ]

    def send(self):
        return create_string_buffer(self)[:]

    def receive(self, bytes):
        fit = min(len(bytes), sizeof(self))
        memmove(addressof(self), bytes, fit)

def time_sync():
    tsf = flow_tsf()
    b = create_string_buffer(sizeof(tsf))

    fd = os.open(path, os.O_RDWR)
    ioctl(fd, IOCTL_SYNC, b)
    os.close(fd)

    tsf.receive(b)

    return tsf.ts_beacon * 1e-6, tsf.ts_kernel * 1e-9

def valve_control(control):
    if (type(control)!= list or len(control) != 4):
        print("control type wrong")
        return 0
    control = (c_int * 4)(*control)
    fd = os.open(path, os.O_RDWR)
    ret = ioctl(fd, IOCTL_SETVALVE, control)
    os.close(fd)
    if ret == 0:
        return 0
    else:
        return -1

def valve_startprotect(t):
    if (type(t) != int):
        raise RuntimeError('parameter must be integer time')
    c_t = c_int64(t)
    fd = os.open(path, os.O_RDWR)
    ret = ioctl(fd, VALVE_STARTPROTECT, c_t)
    os.close(fd)

    if ret != 0:
        raise RuntimeError('failed to inform')

def valve_renewprotect(t):
    if (type(t) != int):
        print("parameter is not time")
        return 0
    c_t = c_int64(t)
    fd = os.open(path, os.O_RDWR)
    ret = ioctl(fd, VALVE_RENEWPROTECT, c_t)
    os.close(fd)

    if ret == 0:
        return 0
    else:
        return -1

def valve_endprotect():
    fd = os.open(path, os.O_RDWR)
    ret = ioctl(fd, VALVE_ENDPROTECT)
    os.close(fd)

    if ret == 0:
        return 0
    else:
        return -1

if __name__ == "__main__":
    valve_endprotect()

