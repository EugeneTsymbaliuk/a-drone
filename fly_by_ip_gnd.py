#!/usr/bin/env python

"""
Check CPU Serial Number. 
cat /proc/cpuinfo
"""

import socket
import pickle
import serial
import argparse
from time import time, sleep
import numpy as np
from threading import Thread

fps = 0
# Bounding Box
CRSF_SYNC = 0xC8
RC_CHANNELS_PACKED = 0x16
chans = []

# Receiver IP and UDP port
UDP_IP = "192.168.10.1"  # Replace with the target IP address
UDP_PORT = 5005       # Replace with the target port

sskey = '100000001daa5c9b'

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

    return bytes(result)

def channelsCrsfToChannelsPacket(channels) -> bytes:
    result = bytearray([CRSF_SYNC, 24, RC_CHANNELS_PACKED]) # 24 is packet length
    result += packCrsfToBytes(channels)
    result.append(crc8_data(result[2:]))
    return bytes(result)

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

def openSerial():
    global chans
    sinfo()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Open serial port")
    with serial.Serial(args.port0, args.baud, timeout=2) as ser:
            input = bytearray()
            if skey == sskey:
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
    #                        if not crsf_validate_frame(single): # single[-1] != crc:
    #                            packet = ' '.join(map(hex, single))
    #                            print(f"crc error: {packet}")
    #                        else:
                            if single[2] == RC_CHANNELS_PACKED:
                                dst = np.zeros(16, dtype=np.uint32)
                                chans = unpackChannels(single[3:], dst, data=[])
                                if ser.in_waiting > 0:
                                    input.extend(ser.read(ser.in_waiting))
                                else:
                                    print(chans)
                                    message = pickle.dumps(chans)
                                    sock.sendto(message, (UDP_IP, UDP_PORT))
                        else:
                            break
    return bytes(chans)

if __name__ == "__main__":
    openSerial()
