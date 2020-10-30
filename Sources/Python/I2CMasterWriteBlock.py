# -*- coding: utf-8 -*-
import smbus
import time
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeBlock(opcode, dataBlock):
    bus.write_i2c_block_data(address, opcode, dataBlock)
# bus.write_byte_data(address, 0, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
# number = bus.read_byte_data(address, 1)
    return number

def splitToByteArray(number):
    #return number.to_bytes((number.bit_length() + 7) // 8, 'big') or b'\0'
    return number.to_bytes(2, byteorder='big')

while True:
    userMotion = int(input("Enter a motion 0(stop)-1(reverse)-2(fwd)-3(left)-4(right): "))

    if userMotion != 0:
        
        userSpeed = int(input("Enter a speed between 1-10: "))
        userDistance = int(input("Enter a distance 0(no distance) or number(1-65535cm): "))

        if userDistance != 0:
            byteList = list(splitToByteArray(userDistance))
            print ("  Distance(unformated): {0}, Distance(byte array): {1}"
                   .format(str(userDistance), byteList))        
            print
    else:
        userSpeed = 0

    # print("The Direction you have entered is: " + str(userDirection))
    # print("The Speed you have entered is: " + str(userSpeed))

    if userDistance != 0:
        opcode = 1
        dataBlock = [userMotion, userSpeed, byteList[1], byteList[0]]        
    else:
        opcode = 0
        dataBlock = [userMotion, userSpeed]

    #Argument 0 is the opcode: 0 (represents a block of 2 bytes: Dir + Speed), 
    #                          1 (represents a block of 4 byte2: Dir + Speed + Distance(lowByte) + Distance(highByte))
    #Argument 1 is the dataBlock
    writeBlock(opcode, dataBlock)
    print ("RPI: Hi Arduino, I sent you Direction, Speed, Distance(lowByte), Distance(highByte): {0}"
           # with Speed: {1} and Distance: 0'{2}',1'{3}'
           # .format(str(userMotion), str(userSpeed), byteList[1], byteList[0]))
           .format(dataBlock))
    # sleep one second
    time.sleep(1)

    number = readNumber()
    print ("Arduino: Hey RPI, I received a digit "), number
    print

