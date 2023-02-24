#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import serial
import crcmod.predefined
import serial.tools.list_ports
import threading
import time
import statistics

count = 0
UP = 0
Down = 0
Left = 0
Right = 0
ALL_avg = 0.0
cnt = 0
cntArr = ["", ""]
cntArr2 = ["", ""]
person_count = -1
person_up_count = 0
person_down_count = 0
person_left_count = 0
person_right_count = 0
detection_person_count=[0,0,0,0]
detection_flag = True
up_flag = False
down_flag = False
left_flag = False
right_flag = False


class Evo_64px(object):

    def __init__(self, portname=None):

        if portname is None:
            ports = list(serial.tools.list_ports.comports())
            for p in ports:
                if ":5740" in p[2]:
                    print("Evo 64px found on port {}".format(p[0]))
                    portname = p[0]
            if portname is None:
                print("Sensor not found. Please Check connections.")
                exit()
        self.portname = portname  # To be adapted if using UART backboard
        self.baudrate = 115200  # 3000000 for UART backboard

        # Configure the serial connections (the parameters differs on the device you are connecting to)
        self.port = serial.Serial(
            port=self.portname,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.port.isOpen()
        self.crc32 = crcmod.predefined.mkPredefinedCrcFun('crc-32-mpeg')
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        self.serial_lock = threading.Lock()

    def get_depth_array(self):
        '''
        This function reads the data from the serial port and returns it as
        an array of 12 bit values with the shape 8x8
        '''
        got_frame = False
        while not got_frame:
            with self.serial_lock:
                # print(self.port.in_waiting)
                frame = self.port.readline()
            if len(frame) == 269:
                if frame[0] == 0x11 and self.crc_check(frame):  # Check for range frame header and crc
                    dec_out = []
                    for i in range(1, 65):
                        rng = frame[2 * i - 1] << 7
                        rng = rng | (frame[2 * i] & 0x7F)
                        dec_out.append(rng & 0x3FFF)
                    depth_array = [dec_out[i:i + 8] for i in range(0, len(dec_out), 8)]
                    depth_array = np.array(depth_array)
                    got_frame = True
            else:
                print("Invalid frame length: {}".format(len(frame)))
        depth_array.astype(np.uint16)

        global count
        count = count + 1
        global UP, Down, Left, Right, ALL_avg, cnt, cntArr, cntArr2, person_count, detection_flag
        global person_up_count, person_down_count, person_left_count, person_right_count,detection_person_count
        global up_flag, down_flag, left_flag, right_flag

        if count % 10 == 0:  # 기준점 20개씩 들어올때 마다 찍어내는 방식
            cnt = 0
            cntArr = ["default", "default"]
            UP = sum(depth_array[0], 0.0) / 8
            Down = sum(depth_array[7], 0.0) / 8
            Left = (depth_array[0][0] + depth_array[1][0] + depth_array[2][0] + depth_array[3][0] + depth_array[4][0] +
                    depth_array[5][0] + depth_array[6][0] + depth_array[7][0]) / 8
            Right = (depth_array[0][7] + depth_array[1][7] + depth_array[2][7] + depth_array[3][7] + depth_array[4][7] +
                     depth_array[5][7] + depth_array[6][7] + depth_array[7][7]) / 8
            for i in range(8):
                ALL_avg = ALL_avg + depth_array[i].mean()
            ALL_avg = ALL_avg / 8
        error_value = 150
        ############### 수시로 들어오는 애들 ###############
        test_UP = sum(depth_array[0], 0.0) / 8
        test_Down = sum(depth_array[7], 0.0) / 8
        test_Left = (depth_array[0][0] + depth_array[1][0] + depth_array[2][0] + depth_array[3][0] + depth_array[4][
            0] +
                     depth_array[5][0] + depth_array[6][0] + depth_array[7][0]) / 8
        test_Right = (depth_array[0][7] + depth_array[1][7] + depth_array[2][7] + depth_array[3][7] +
                      depth_array[4][7] +
                      depth_array[5][7] + depth_array[6][7] + depth_array[7][7]) / 8
        test_avg = 0.0
        for i in range(8):
            test_avg = test_avg + depth_array[i].mean()
        test_avg = test_avg / 8
        checkMin = min(test_UP, test_Down, test_Left, test_Right)

        if (UP - error_value) > test_UP and checkMin == test_UP:  #### 처음 움직이는 방향 확인
            if cntArr[0] != "Up" and cntArr[1] != "Up":
                cntArr[cnt] = "Up"
                cnt = 1
                if cntArr[0] == "Down" and cntArr[1] == "Up":
                    up_flag = True
            print("In: Up Movement Detection")

            # if 나머지 세개가 그 해당 사이드의 평균값이 기준보다 작으면 아웃은그쪽방향
        elif (Down - error_value) > test_Down and checkMin == test_Down:
            if cntArr[0] != "Down" and cntArr[1] != "Down":
                cntArr[cnt] = "Down"
                cnt = 1
                if cntArr[0] == "Down" and cntArr[1] == "Up":
                    down_flag = True
            print("In: Down Movement Detection")

        elif (Left - error_value) > test_Left and checkMin == test_Left:
            if cntArr[0] != "Left" and cntArr[1] != "Left":
                cntArr[cnt] = "Left"
                cnt = 1
                if cntArr[0] == "Right" and cntArr[1] == "Left":
                    left_flag = True
            print("In: Left Movement Detection")

        elif (Right - error_value) > test_Right and checkMin == test_Right:
            if cntArr[0] != "Right" and cntArr[1] != "Right":
                cntArr[cnt] = "Right"
                cnt = 1
                if cntArr[0] == "Left" and cntArr[1] == "Right":
                    right_flag = True
            print("In: Right Movement Detection")

        elif cntArr[0] == "default" and cntArr[1] == "default":
            if detection_flag==False and cntArr2[0] == "Up" and cntArr2[1] == "Down":
                person_count = person_count + 1
                detection_flag = True
            elif detection_flag==False and cntArr2[0] == "Down" and cntArr2[1] == "Up":
                person_count = person_count - 1
                detection_flag = True
            elif detection_flag and cntArr2[0] != "default" and cntArr2[1] != "default":
                # if up_flag:
                #     person_count = person_count - 1
                #     up_flag = False
                # if down_flag:
                #     person_count = person_count - 1
                #     down_flag = False
                # if left_flag:
                #     person_count = person_count - 1
                #     left_flag = False
                # elif right_flag:
                #     person_count = person_count - 1
                #     right_flag = False
                cntArr2[0] = "default"
                cntArr2[1] = "default"
            print("Waiting")
            print("Person Count: %d" % person_count)



        # if cntArr[0] != "default" and cntArr[1] == "default" :
        #     person_count = person_count + 1
        #     print("Person_Count: %d" % person_count)
        if cntArr[0] != "default" and cntArr[1] != "default":
            # cnt = 2
            print("***********************", cntArr[0], " -> ", cntArr[1],
                  "***********************")
            cntArr2 = cntArr.copy()
            detection_flag = False

        # print("UP: ", (UP))
        # print("Down: ", (Down))
        # print("Left: ", (Left))
        # print("Right: ", (Right))
        # print("All AVG:, ", avg)
        # print("COunt: ", count)
        # print(depth_array[0].mean())
        # print(depth_array[1].mean())
        # print(depth_array[2].mean())
        # print(depth_array[3].mean())
        # print(depth_array[4].mean())
        # print(depth_array[5].mean())
        # print(depth_array[6].mean())
        # print(depth_array[7].mean())
        print(count)
        print(ALL_avg, test_avg)
        print(ALL_avg - test_avg)
        return count

    # min(map(min,depth_array)), max(map(max,depth_array))
    def crc_check(self, frame):
        index = len(frame) - 9  # Start of CRC
        crc_value = (frame[index] & 0x0F) << 28
        crc_value |= (frame[index + 1] & 0x0F) << 24
        crc_value |= (frame[index + 2] & 0x0F) << 20
        crc_value |= (frame[index + 3] & 0x0F) << 16
        crc_value |= (frame[index + 4] & 0x0F) << 12
        crc_value |= (frame[index + 5] & 0x0F) << 8
        crc_value |= (frame[index + 6] & 0x0F) << 4
        crc_value |= (frame[index + 7] & 0x0F)
        crc_value = crc_value & 0xFFFFFFFF
        crc32 = self.crc32(frame[:index])

        if crc32 == crc_value:
            return True
        else:
            print("Discarding current buffer because of bad checksum")
            return False

    def send_command(self, command):
        with self.serial_lock:  # This avoid concurrent writes/reads of serial
            self.port.write(command)
            ack = self.port.read(1)
            # This loop discards buffered frames until an ACK header is reached
            while ack != b"\x14":
                self.port.readline()
                ack = self.port.read(1)
            else:
                ack += self.port.read(3)

            # Check ACK crc8
            crc8 = self.crc8(ack[:3])
            if crc8 == ack[3]:
                # Check if ACK or NACK
                if ack[2] == 0:
                    return True
                else:
                    print("Command not acknowledged")
                    return False
            else:
                print("Error in ACK checksum")
                return False

    def start_sensor(self):
        if self.send_command(b"\x00\x52\x02\x01\xDF"):
            print("Sensor started successfully")

    def stop_sensor(self):
        if self.send_command(b"\x00\x52\x02\x00\xD8"):
            print("Sensor stopped successfully")

    def run(self):
        self.port.flushInput()
        if self.baudrate == 115200:  # Sending VCP start when connected via USB
            self.start_sensor()
        start_t = time.time()
        depth_array = []
        while depth_array is not None:
            depth_array = self.get_depth_array()
            # print(depth_array)
        else:
            if self.baudrate == 115200:
                self.stop_sensor()  # Sending VCP stop when connected via USB


if __name__ == '__main__':
    evo_64px = Evo_64px()
    evo_64px.run()
