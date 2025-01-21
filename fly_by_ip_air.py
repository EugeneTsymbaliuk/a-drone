#!/usr/bin/env python

import socket
import pickle
import serial
import argparse
from time import time, sleep

fps = 0
# Bounding Box
CRSF_SYNC = 0xC8
RC_CHANNELS_PACKED = 0x16
chans = []

# UDP IP address and port to listen on
UDP_IP = "0.0.0.0" 
UDP_PORT = 5005

sskey = '0000000006f12a37'

parser = argparse.ArgumentParser()
parser.add_argument('-p0', '--port0', default='/dev/ttyAMA0', required=False)
parser.add_argument('-b', '--baud', default=420000, required=False)
args = parser.parse_args()

def sinfo():
    global skey
    f = open('/proc/cpuinfo','r')
    for l in f:
        if l.startswith('Serial'):
            skey = l[-18:].strip()

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

    return bytes(result)

def channelsCrsfToChannelsPacket(channels) -> bytes:
    result = bytearray([CRSF_SYNC, 24, RC_CHANNELS_PACKED]) # 24 is packet length
    result += packCrsfToBytes(channels)
    result.append(crc8_data(result[2:]))
    return bytes(result)

def openSerial():
    global chans
    sinfo()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print("Open serial port")
    with serial.Serial(args.port0, args.baud, timeout=2) as ser:
        if skey == sskey:
            while True:
                data, addr = sock.recvfrom(1024)
                chans = pickle.loads(data)
                ser.write(channelsCrsfToChannelsPacket(chans))

    return bytes(chans)

if __name__ == "__main__":
    openSerial()
