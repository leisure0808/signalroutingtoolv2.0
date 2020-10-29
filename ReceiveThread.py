import time
from PyQt5.QtCore import QThread, pyqtSignal
from zlgcan import *

MAX_RCV_NUM = 10


class ReceiveThread(QThread):
    signal = pyqtSignal(object)

    def __init__(self, channelhandle, idlist, zcan):
        super().__init__()
        self.chn_handle = channelhandle
        self.working = True
        self.zcanLib = zcan
        self.id_list = idlist

    def stop(self):
        self.working = False
        self.wait()

    def run(self):
        while self.working:
            # print("进入到接收线程", self.chn_handle)
            canfd_num = self.zcanLib.GetReceiveNum(self.chn_handle, ZCAN_TYPE_CANFD)
            can_num = self.zcanLib.GetReceiveNum(self.chn_handle, ZCAN_TYPE_CAN)
            if not canfd_num and not can_num:
                time.sleep(0.005)  # wait 5ms
                continue
            while canfd_num and self.working:
                read0_cnt = MAX_RCV_NUM if canfd_num >= MAX_RCV_NUM else canfd_num
                canfd0_msgs_temp, act0_num = self.zcanLib.ReceiveFD(self.chn_handle, read0_cnt)
                if act0_num:
                    # get resp data
                    for i0 in range(act0_num):
                        id = canfd0_msgs_temp[i0].frame.can_id
                        if hex(id) in self.id_list:
                            self.signal.emit(canfd0_msgs_temp[i0])
                        else:
                            continue
                else:
                    break
                canfd_num -= act0_num
            while can_num and self.working:
                read1_cnt = MAX_RCV_NUM if can_num >= MAX_RCV_NUM else can_num
                can_msgs_temp, act1_num = self.zcanLib.Receive(self.chn_handle, read1_cnt)
                if act1_num:
                    # get resp data
                    for i0 in range(act1_num):
                        id = can_msgs_temp[i0].frame.can_id
                        if hex(id) in self.id_list:
                            self.signal.emit(can_msgs_temp[i0])
                        else:
                            continue
                else:
                    break
                can_num -= act1_num