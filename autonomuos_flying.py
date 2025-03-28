#!/usr/bin/env python

import serial
import argparse
import cv2 as cv
import numpy as np
from picamera2 import Picamera2
from dronekit import connect, VehicleMode
from time import time, sleep
from threading import Thread

# Video resolution
dispW=720
dispH=480

# ROI size
roi_size = 150

# Center of video window
x = dispW // 2
y = dispH // 2

# FPS value for test
fps=0

# Bounding Box
SW = None

CRSF_SYNC = 0xC8
RC_CHANNELS_PACKED = 0x16
chans = []

# Channels values
yaw = 1500
roll = 1500
pitch = 1800
thr = 1650

# Wait 60 seconds
print("Wait 60 seconds")
sleep(60)

# Create the connection to drone
print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.145:5762', rate=40)
vehicle = connect("/dev/ttyAMA1", baud=57600, wait_ready=True,  timeout=100, rate=40)
print('Connected to FC')

# Create picamera instance
picam2 = Picamera2()

# Video Settings
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=40
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Object tracker
tracker = cv.TrackerCSRT_create() # Initialize tracker with CSRT algorithm

def crc8_dvb_s2(crc, a) -> int:
  crc = crc ^ a
  for ii in range(8):
    if crc & 0x80:
      crc = (crc << 1) ^ 0xD5
    else:
      crc = crc << 1
  return crc & 0xFF

def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]

def packCrsfToBytes(channels) -> bytes:
    # channels is in CRSF format! (0-1984)
    # Values are packed little-endianish such that bits BA987654321 -> 87654321, 00000BA9
    # 11 bits per channel x 16 channels = 22 bytes
    if len(channels) != 16:
        raise ValueError('CRSF must have 16 channels')
    result = bytearray()
    destShift = 0
    newVal = 0
    for ch in channels:
        # Put the low bits in any remaining dest capacity
        newVal |= (ch << destShift) & 0xff
        result.append(newVal)

        # Shift the high bits down and place them into the next dest byte
        srcBitsLeft = 11 - 8 + destShift
        newVal = ch >> (11 - srcBitsLeft)
        # When there's at least a full byte remaining, consume that as well
        if srcBitsLeft >= 8:
            result.append(newVal & 0xff)
            newVal >>= 8
            srcBitsLeft -= 8

        # Next dest should be shifted up by the bits consumed
        destShift = srcBitsLeft

    return result

#def channelsCrsfToChannelsPacket(channels) -> bytes:
#    result = bytearray([CRSF_SYNC, 24, RC_CHANNELS_PACKED]) # 24 is packet length
#    result += packCrsfToBytes(channels)
#    result.append(crc8_data(result[2:]))
#    return result

def unpackChannels(payload, dest, data):
    num_of_channels = 16
    src_bits = 11
    input_channel_mask = (1 << src_bits) - 1
    bits_merged = 0
    read_value = 0
    read_byte_index = 0
    for n in range(num_of_channels):
        while bits_merged < src_bits:
            read_byte = payload[read_byte_index]
            read_byte_index += 1
            read_value |= (read_byte << bits_merged)
            bits_merged += 8
        try:
            dest[n] = read_value & input_channel_mask
        except ValueError:
            pass
        data.append(dest[n])
        read_value >>= src_bits
        bits_merged -= src_bits
    return data

def rcOverrides(roll, pitch, thr, yaw):
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}

def armAndtakeoff():
    while not vehicle.armed:
        vehicle.armed = True

def merge_roi(frame, roi, x, y):
    frame[y:y+roi_size, x:x+roi_size, :] = roi
    return frame

def pwmCalc(crsf_value):
    pwm = 1500 + (0.625 * (crsf_value - 992))
    return int(pwm)

def openSerial():
    print("Open serial port")
    global chans

    with serial.Serial(args.port0, args.baud, timeout=2) as ser:
            input = bytearray()
            while True:
                if ser.in_waiting > 0:
                    input.extend(ser.read(ser.in_waiting))
                while len(input) > 2:
                # This simple parser works with malformed CRSF streams
                # it does not check the first byte for SYNC_BYTE, but
                # instead just looks for anything where the packet length
                # is 4-64 bytes, and the CRC validates
                    expected_len = input[1] + 2
                    if expected_len > 64 or expected_len < 4:
                        input = bytearray()
                    elif len(input) >= expected_len:
                        single = input[:expected_len] # copy out this whole packet
                        input = input[expected_len:] # and remove it from the buffer

                        if not crsf_validate_frame(single): # single[-1] != crc:
                            packet = ' '.join(map(hex, single))
                            print(f"crc error: {packet}")
                        else:
                            if single[2] == RC_CHANNELS_PACKED:
                                dst = np.zeros(16, dtype=np.uint32)
                                chans = unpackChannels(single[3:], dst, data=[])
                                if ser.in_waiting > 0:
                                    input.extend(ser.read(ser.in_waiting))
                                else:
                                    if SW is None:
                                        rcOverrides(pwmCalc(chans[0]), pwmCalc(chans[1]), pwmCalc(chans[2]), pwmCalc(chans[3]))
                    else:
                        break
    return chans

parser = argparse.ArgumentParser()
parser.add_argument('-p0', '--port0', default='/dev/ttyAMA0', required=False)
parser.add_argument('-b', '--baud', default=420000, required=False)
args = parser.parse_args()


if __name__ == "__main__":

    # Start serial connection with RX
    Thread(target=openSerial).start()

    while True:
    #    tStart = time()
        frame = picam2.capture_array()
        frame = cv.flip(frame, -1)

        key = cv.waitKey(1) & 0xFF

        if SW is not None:
            cv.putText(frame, "Autonomous", (5,30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
#            cv.putText(frame, str(int(fps))+' FPS', (5,50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

            if vehicle.armed == False:
                armAndtakeoff()

            # Enable althold mode
            if vehicle.mode != "ALT_HOLD":
                vehicle.mode = VehicleMode("ALT_HOLD")
            rcOverrides(roll, pitch, thr, yaw)

        if SW is None:
            cv.putText(frame, "Manual", (5,30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
#            cv.putText(frame, str(int(fps))+' FPS', (5,50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
            if vehicle.mode != "STABILIZE":
                vehicle.mode = VehicleMode("STABILIZE")

        try:
            if chans[4] > 1600 and SW is None:
                SW = 1

            if chans[4] < 1600 and SW is not None:
                SW = None

            if chans[7] > 1600:
                frame[0:int(dispH/2), int(dispW/1.5):] = [60,100,100]
                frame[:int(dispH/2), :int(dispW/3)] = [60,100,100]
                frame[int(dispH/2):, :int(dispW/2)] = [60,100,100]
                frame[int(dispH/2):, int(dispW/2):] = [60,100,100]

        except IndexError:
            cv.putText(frame, "NO RC Control", (5,55), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
    #        cv.putText(frame, str(int(fps))+' FPS', (5,80), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
            pass

        cv.namedWindow("Frame", cv.WND_PROP_FULLSCREEN)
        cv.setWindowProperty("Frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
        cv.imshow("Frame", frame)

        # Close video window
        if key == ord("q"):
                break

        # FPS count
    #    tEnd=time()
    #    loopTime=tEnd-tStart
    #    print(loopTime)
    #    fps=.9*fps + .1*(1/loopTime)

    # Stop tracking
    cv.destroyAllWindows()