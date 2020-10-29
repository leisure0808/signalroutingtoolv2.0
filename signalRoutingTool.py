import configparser
import logging
import os
import sys
import time
from logging.handlers import RotatingFileHandler

from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtWidgets import QMessageBox, QTableWidgetItem, QAction

from zlgcan import *
import threading
from signalRoutingWindow import Ui_MainWindow
from PyQt5 import QtCore, QtGui, QtWidgets
import xml.etree.ElementTree as ET
from signalRoutingDll import *
import json

MAX_RCV_NUM = 10
###############################################################################


class ReceiveThread(QThread):
    signal = pyqtSignal(int, list)

    def __init__(self, chn0, idlist, zcan):
        super().__init__()
        self.chn_handle = chn0
        self.working = True
        self.zcanLib = zcan
        self.id_list = idlist

    def stop(self):
        self.working = False
        self.wait()

    def run(self):
        data64 = [0 for x in range(64)]
        data = [0 for x in range(8)]
        while self.working:
            # print("进入到接收线程", self.chn_handle)
            canfd_num = self.zcanLib.GetReceiveNum(self.chn_handle, ZCAN_TYPE_CANFD)
            can_num = self.zcanLib.GetReceiveNum(self.chn_handle, ZCAN_TYPE_CAN)
            if not canfd_num and not can_num:
                time.sleep(0.005)  # wait 5ms
                continue
            # print("+_+_+_+_+_")
            while canfd_num and self.working:
                read0_cnt = MAX_RCV_NUM if canfd_num >= MAX_RCV_NUM else canfd_num
                canfd0_msgs_temp, act0_num = self.zcanLib.ReceiveFD(self.chn_handle, read0_cnt)
                if act0_num:
                    # get resp data
                    for i0 in range(act0_num):
                        id = canfd0_msgs_temp[i0].frame.can_id
                        if hex(id) in self.id_list:
                            for j in range(64):
                                data64[j] = canfd0_msgs_temp[i0].frame.data[j]
                            self.signal.emit(id, data64)
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
                        # print("frame id:",hex(id), type(id))
                        if hex(id) in self.id_list:
                            for j in range(8):
                                data[j] = can_msgs_temp[i0].frame.data[j]
                            self.signal.emit(id, data)
                        else:
                            continue
                else:
                    break
                can_num -= act1_num


class DeviceInit:
    def __init__(self, n):
        # device info
        self.deviceIsChecked = False
        self.deviceIndex = n
        self.deviceHandle = INVALID_DEVICE_HANDLE
        self.deviceIsOpen = False
        # channel info
        self.channelIndex = [0, 1]
        self.chnIsChecked = [False, False]
        self.channelHandle = [INVALID_CHANNEL_HANDLE, INVALID_CHANNEL_HANDLE]
        self.chnIsOpen = [False, False]
        self.chnIsCANfd = [False, False]
        self.chnResSupport = [False, False]
        self.chnCfg = [ZCAN_CHANNEL_INIT_CONFIG(), ZCAN_CHANNEL_INIT_CONFIG()]
        self.chnArbBt = [None, None]
        self.chnDataBt = [None, None]
        self.chnType = [None, None]

        #  read can/canfd message thread
        self.checkReadValue = [None, None]
        self.isCheckReadThread = [False, False]
        self.chnReadThread = [None, None]
        self.terminated = [False, False]

        # period send var
        self.checkSendValue = [None, None]
        self.isCheckSendThread = [False, False]
        self.chnIsSending = [False, False]
        self.isCANfdMsg = [False, False]
        self.chnSendMsg = [None, None]
        self.chnSendThread = [None, None]


class Signal(object):
    def __init__(self):
        # 信号信息
        self.signal = {'name': None, 'startBit': None, 'len': None, 'factor': None, 'offset': None, 'funFlag': None, 'unsigned':None}


class Message(object):  # XML文件中报文信号信息
    def __init__(self, n):
        # 报文信息
        self.message = {'id': None, 'dlc': None, 'signal': [Signal() for x in range(n)]}


class SignalRoutingTool(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        # 实例化UI界面
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        # 初始化周立功函数
        self.zcanlib = ZCAN()
        # 雷达报文初始化
        self.commondmessage = ZCAN_TransmitFD_Data()
        self.commondmessage.frame.can_id = 0x111
        self.commondmessage.frame.len = 64
        self.ecutimemessage = ZCAN_TransmitFD_Data()
        self.ecutimemessage.frame.can_id = 0x101
        self.ecutimemessage.frame.len = 64
        self.vehiclepart1message = ZCAN_TransmitFD_Data()
        self.vehiclepart1message.frame.can_id = 0x121
        self.vehiclepart1message.frame.len = 64
        self.vehiclepart2message = ZCAN_TransmitFD_Data()
        self.vehiclepart2message.frame.can_id = 0x122
        self.vehiclepart2message.frame.len = 64
        # JSON文件中读取设备信息
        self._dev_info = None
        with open("./dev_info.json", "r") as fd:
            self._dev_info = json.load(fd)
            # print(self._dev_info)
        if self._dev_info is None:
            print("device info no exist!")
            return
        else:
            self.cur_dev_info = self._dev_info["USBCANFD-200U"]
        # 车身参数映射泪飙初始化
        listcfg = "parameterMapping.ini"
        self.listvaluecfg = configparser.ConfigParser()
        self.listvaluecfg.read(listcfg)
        # 初始化挡位映射关系
        self.gearsourcedata = list(map(int, (self.listvaluecfg.get("gearstatus", "sourcedata")).split(' ')))
        self.geartargetdata = list(map(int, (self.listvaluecfg.get("gearstatus", "targetdata")).split(' ')))
        # 初始化车门状态映射关系
        self.doorsourcedata = list(map(int, (self.listvaluecfg.get("doorstatus", "sourcedata")).split(' ')))
        self.doortargetdata = list(map(int, (self.listvaluecfg.get("doorstatus", "targetdata")).split(' ')))
        # 初始化转向灯状态映射关系
        self.rightturnswsourcedata = list(map(int, (self.listvaluecfg.get("rightturn", "sourcedata")).split(' ')))
        self.rightturnswtargetdata = list(map(int, (self.listvaluecfg.get("rightturn", "targetdata")).split(' ')))

        self.leftturnswsourcedata = list(map(int, (self.listvaluecfg.get("leftturn", "sourcedata")).split(' ')))
        self.leftturnswtargetdata = list(map(int, (self.listvaluecfg.get("leftturn", "targetdata")).split(' ')))
        # 其他待增加的初始化映射关系-增加后请在接收进程中更新相关代码

        # 初始化signalRoutingDll
        self.AVP_dll = AVPDll()
        self.Body_dll = BodyDll()
        # xml文档读取初始化
        self.msg = []
        # print(self.msg.message['signal'][0].signal['name'])
        self.idlist = []
        self.load_xml()
        # ui界面默认值初始化
        self.ui_init()
        # 设备参数实例化
        self.initDevice = [DeviceInit(i) for i in range(2)]
        self.device_init()

        # 控件事件绑定
        self.ui.device1_check.clicked.connect(self.device_check)
        self.ui.device2_check.clicked.connect(self.device_check)
        self.ui.device1_index.currentIndexChanged.connect(self.deviceIndexSelect)
        self.ui.device2_index.currentIndexChanged.connect(self.deviceIndexSelect)
        for i in range(len(self.initDevice)):
            for j in range(2):
                self.initDevice[i].chnType[j].currentIndexChanged.connect(self.CmbMsgCANFDUpdate)
                self.initDevice[i].checkReadValue[j].clicked.connect(self.TRupdate)
                self.initDevice[i].checkSendValue[j].clicked.connect(self.TRupdate)
        self.ui.RCTA.clicked.connect(self.adasfunction)
        self.ui.FCTA.clicked.connect(self.adasfunction)
        self.ui.DOW.clicked.connect(self.adasfunction)
        self.ui.LCA.clicked.connect(self.adasfunction)
        self.ui.BSD.clicked.connect(self.adasfunction)
        self.ui.sensor_mode.currentTextChanged.connect(self.modeselect)
        self.ui.device_open_close.clicked.connect(self.deciceOpenClose)
        # self.ui.CycleSend.clicked.connect(self.cycle_send)
        self.ui.routing_start_stop.clicked.connect(self.vehiclepart_cyclesend)
        self.ui.log_check.clicked.connect(self.log)

        help_file = QAction('操作指导手册', self)
        self.ui.menuHelp.addAction(help_file)
        self.ui.menuHelp.triggered.connect(self.help)

        # 车辆参数初始化
        self.wheel_base = 1.56
        self.RLwheelspeed = 0
        self.RRwheelspeed = 0
        self.currentGear = 0

        # 周期事件
        self.commandmsg_cycle = QTimer()
        self.commandmsg_cycle.timeout.connect(self.commandmsg_send)

        self.ecutimemsg_cycle = QTimer()
        self.ecutimemsg_cycle.timeout.connect(self.ecutimemsg_seng)

        self.vehiclepart1_cycle = QTimer()
        self.vehiclepart1_cycle.timeout.connect(self.vehiclepart1_send)

        self.vehiclepart2_cycle = QTimer()
        self.vehiclepart2_cycle.timeout.connect(self.vehiclepart2_send)

        # 界面默认值设置
        self.ui.device1_check.click()
        self.ui.device2_check.click()
        self.ui.device1_chn0_Rx.click()
        self.ui.device2_chn0_Tx.click()
        self.ui.device2_chn1_Tx.click()
        self.ui.sensor_mode.setCurrentIndex(1)  # m默认standby
        self.ui.CycleSend.click()
        self.ui.device1_index.setCurrentIndex(1)
        self.ui.device2_index.setCurrentIndex(2)

        self.ui.routing_start_stop.setEnabled(False)

        self.tablemodel = QStandardItemModel()

        self.signallist = [float(0) for i in range(6)]

        self._logging = False

    def load_xml(self):
        tree = ET.ElementTree(file='SourceEcuDatabaseCongfig.xml')
        root = tree.getroot()
        for message in root:
            message_temp = Message(int(message.attrib['signalNum']))
            # print(message.attrib)
            message_temp.message['id'] = message.attrib['id']
            message_temp.message['dlc'] = message.attrib['dlc']
            i = 0
            for signal in message:
                # print(signal.attrib)
                message_temp.message['signal'][i].signal['name'] = signal.attrib['name']
                message_temp.message['signal'][i].signal['startBit'] = signal.attrib['startBit']
                message_temp.message['signal'][i].signal['len'] = signal.attrib['len']
                message_temp.message['signal'][i].signal['factor'] = signal.attrib['factor']
                message_temp.message['signal'][i].signal['startBit'] = signal.attrib['startBit']
                message_temp.message['signal'][i].signal['offset'] = signal.attrib['offset']
                message_temp.message['signal'][i].signal['funFlag'] = signal.attrib['funFlag']
                message_temp.message['signal'][i].signal['unsigned'] = signal.attrib['unsigned']
                i = i + 1
            self.msg.append(message_temp)
        for m in self.msg:
            # print(m.message['id'], type(m.message['id']))
            self.idlist.append(m.message['id'])

    def ui_init(self):
        # 设备1参数初始化
        self.ui.device1_chn0_type.setCurrentIndex(1)
        self.ui.device1_chn0_Arb.setCurrentIndex(4)
        self.ui.device1_chn0_Data.setCurrentIndex(1)
        self.ui.device1_chn1_type.setCurrentIndex(1)
        self.ui.device1_chn1_Arb.setCurrentIndex(4)
        self.ui.device1_chn1_Data.setCurrentIndex(1)
        # 设备2参数初始化
        self.ui.device2_chn0_type.setCurrentIndex(1)
        self.ui.device2_chn0_Arb.setCurrentIndex(4)
        self.ui.device2_chn0_Data.setCurrentIndex(1)
        self.ui.device2_chn1_type.setCurrentIndex(1)
        self.ui.device2_chn1_Arb.setCurrentIndex(4)
        self.ui.device2_chn1_Data.setCurrentIndex(1)

    def device_init(self):
        self.initDevice[0].deviceIsChecked = self.ui.device1_check.isChecked()
        self.initDevice[0].deviceIndex = self.ui.device1_index.currentIndex()
        self.initDevice[0].chnType[0] = self.ui.device1_chn0_type
        self.initDevice[0].chnType[1] = self.ui.device1_chn1_type
        self.initDevice[0].chnArbBt[0] = self.ui.device1_chn0_Arb
        self.initDevice[0].chnArbBt[1] = self.ui.device1_chn1_Arb
        self.initDevice[0].chnDataBt[0] = self.ui.device1_chn0_Data
        self.initDevice[0].chnDataBt[1] = self.ui.device1_chn1_Data
        self.initDevice[0].checkReadValue[0] = self.ui.device1_chn0_Rx
        self.initDevice[0].checkReadValue[1] = self.ui.device1_chn1_Rx
        self.initDevice[0].checkSendValue[0] = self.ui.device1_chn0_Tx
        self.initDevice[0].checkSendValue[1] = self.ui.device1_chn1_Tx

        self.initDevice[1].deviceIsChecked = self.ui.device2_check.isChecked()
        self.initDevice[1].deviceIndex = self.ui.device2_index.currentIndex()
        self.initDevice[1].chnType[0] = self.ui.device2_chn0_type
        self.initDevice[1].chnType[1] = self.ui.device2_chn1_type
        self.initDevice[1].chnArbBt[0] = self.ui.device2_chn0_Arb
        self.initDevice[1].chnArbBt[1] = self.ui.device2_chn1_Arb
        self.initDevice[1].chnDataBt[0] = self.ui.device2_chn0_Data
        self.initDevice[1].chnDataBt[1] = self.ui.device2_chn1_Data
        self.initDevice[1].checkReadValue[0] = self.ui.device2_chn0_Rx
        self.initDevice[1].checkReadValue[1] = self.ui.device2_chn1_Rx
        self.initDevice[1].checkSendValue[0] = self.ui.device2_chn0_Tx
        self.initDevice[1].checkSendValue[1] = self.ui.device2_chn1_Tx
    # UI响应函数--------------------------------------------------------------------------------------------------------

    def device_check(self):
        self.initDevice[0].deviceIsChecked = self.ui.device1_check.isChecked()
        self.initDevice[1].deviceIsChecked = self.ui.device2_check.isChecked()
        # print(self.initDevice[0].deviceIsChecked, self.initDevice[1].deviceIsChecked)

    def deviceIndexSelect(self):
        self.initDevice[0].deviceIndex = self.ui.device1_index.currentIndex()
        self.initDevice[1].deviceIndex = self.ui.device2_index.currentIndex()
        # print(self.initDevice[0].deviceIndex, self.initDevice[1].deviceIndex)

    def deciceOpenClose(self):
        if self.ui.device_open_close.text() == "打开设备":
            for index in range(2):
                if self.initDevice[index].deviceIsChecked and (not self.initDevice[index].deviceIsOpen):
                    # Open Device
                    self.initDevice[index].deviceHandle = self.zcanlib.OpenDevice(self.cur_dev_info["dev_type"], self.initDevice[index].deviceIndex, 0)
                    print(self.initDevice[index].deviceIndex)
                    if self.initDevice[index].deviceHandle == INVALID_DEVICE_HANDLE:
                        # Open failed
                        QMessageBox.warning(self, "打开设备", "打开设备" + str(index + 1) + "失败！")
                        return
                    self.initDevice[index].deviceIsOpen = True
                    for j in range(self.cur_dev_info["chn_num"]):
                        ip = self.zcanlib.GetIProperty(self.initDevice[index].deviceHandle)
                        self.zcanlib.SetValue(ip, str(j) + "/clock", "60000000")
                        self.zcanlib.ReleaseIProperty(ip)

                        self.initDevice[index].chnIsCANfd[j] = self.cur_dev_info["chn_info"]["is_canfd"]
                        self.initDevice[index].chnResSupport[j] = self.cur_dev_info["chn_info"]["sf_res"]
                        # open channel
                        self.initDevice[index].chnCfg[j].can_type = ZCAN_TYPE_CANFD if self.initDevice[index].chnIsCANfd[
                            j] else ZCAN_TYPE_CAN
                        self.initDevice[index].chnCfg[j].config.canfd.mode = 0
                        self.initDevice[index].chnCfg[j].config.canfd.abit_timing = \
                        self.cur_dev_info["chn_info"]["baudrate"][self.initDevice[index].chnArbBt[j].currentText()]
                        print(self.initDevice[index].chnCfg[j].config.canfd.abit_timing)
                        self.initDevice[index].chnCfg[j].config.canfd.dbit_timing = \
                        self.cur_dev_info["chn_info"]["data_baudrate"][self.initDevice[index].chnDataBt[j].currentText()]
                        print(self.initDevice[index].chnCfg[j].config.canfd.dbit_timing)
                        self.initDevice[index].channelHandle[j] = self.zcanlib.InitCAN(self.initDevice[index].deviceHandle,
                                                                                       self.initDevice[index].channelIndex[j],
                                                                                       self.initDevice[index].chnCfg[j])
                        if self.initDevice[index].channelHandle[j] == INVALID_CHANNEL_HANDLE:
                            QMessageBox.warning("打开通道", "初始化通道失败!")
                            return
                        ret = self.zcanlib.StartCAN(self.initDevice[index].channelHandle[j])
                        if ret != ZCAN_STATUS_OK:
                            QMessageBox.warning("打开通道", "打开通道失败!")
                            return
                        # start receive thread
                        print(self.initDevice[index].terminated[j])
                        self._terminated = self.initDevice[index].terminated[j]
                        self._can_handle = self.initDevice[index].channelHandle[j]
                        if self.initDevice[index].checkReadValue[j].isChecked():
                            self.initDevice[index].chnReadThread[j] = ReceiveThread(self.initDevice[index].channelHandle[j], self.idlist, self.zcanlib)
                            self.initDevice[index].chnReadThread[j].signal.connect(self.dataUpdate)
                            self.initDevice[index].chnReadThread[j].start()
                            print("接收线程开启")
                elif (not self.initDevice[index].deviceIsChecked) and (self.initDevice[index].deviceIsOpen):
                    print("budakai", index)
                    self.zcanlib.CloseDevice(self.initDevice[index].deviceHandle)
                    self.initDevice[index].deviceHandle = INVALID_DEVICE_HANDLE
                    self.initDevice[index].deviceIsOpen = False
                elif (not self.initDevice[index].deviceIsChecked) and (not self.initDevice[index].deviceIsOpen):
                    self.zcanlib.ResetCAN(self.initDevice[index].channelHandle[0])
                    self.zcanlib.ResetCAN(self.initDevice[index].channelHandle[1])
                    self.zcanlib.CloseDevice(self.initDevice[index].deviceHandle)
                    self.initDevice[index].deviceHandle = INVALID_DEVICE_HANDLE
                    self.initDevice[index].deviceIsOpen = False

            self.ui.device_open_close.setText("关闭设备")
            self.time = 0
            self.ui.routing_start_stop.setEnabled(True)
            
        else:
            for index in range(len(self.initDevice)):
                for j in range(2):
                    if self.initDevice[index].checkReadValue[j].isChecked():
                        self.initDevice[index].chnReadThread[j].stop()
                if self.initDevice[index].deviceIsOpen is True:
                    self.initDevice[index].deviceIsOpen = False
                    self.zcanlib.CloseDevice(self.initDevice[index].deviceHandle)
                else:
                    continue
            self.ui.device_open_close.setText("打开设备")
            self.ui.routing_start_stop.setEnabled(False)

    def CmbMsgCANFDUpdate(self):
        for i in range(len(self.initDevice)):
            for j in range(2):
                if self.initDevice[i].chnType[j].currentIndex() == 0:
                    self.initDevice[i].chnDataBt[j].setEnabled(False)
                else:
                    self.initDevice[i].chnDataBt[j].setEnabled(True)

    def TRupdate(self):
        for i in range(len(self.initDevice)):
            for j in range(2):
                if self.initDevice[i].checkReadValue[j].isChecked():
                    self.initDevice[i].checkSendValue[j].setEnabled(False)
                else:
                    self.initDevice[i].checkSendValue[j].setEnabled(True)
                if self.initDevice[i].checkSendValue[j].isChecked():
                    self.initDevice[i].checkReadValue[j].setEnabled(False)
                else:
                    self.initDevice[i].checkReadValue[j].setEnabled(True)

    def modeselect(self):
        mode = self.ui.sensor_mode.currentIndex()
        print('mode', hex(mode))
        self.AVP_dll.setSensorMode(mode)
        # print(self.AVP_dll.getByteData(self.AVP_dll.EcuCommandMessage64))

    def adasfunction(self):
        bsd = self.ui.BSD.isChecked()
        # print(bsd)
        lca = self.ui.LCA.isChecked()
        dow = self.ui.DOW.isChecked()
        fcta = self.ui.FCTA.isChecked()
        rcta = self.ui.RCTA.isChecked()
        adasSwitch = dow << 4 | rcta << 3 | fcta << 2 | lca << 1 | bsd
        # print('adasSwitch', hex(adasSwitch))
        self.AVP_dll.setAdasSwitch(adasSwitch)

    def dataUpdate(self, id, data):
        # print("数据跟新")
        for m in self.msg:
            if m.message['id'] == hex(id):
                for s in m.message['signal']:
                    # print(s.signal['name'], s.signal['startBit'])
                    # 待完善报文信号更新
                    signalName, signalPhyValue, funFlag = self.Body_dll.getSignalPhysValue(data, s.signal)
                    # if signalName == "SteerWheelAngle":
                        # print("原始物理值", signalName, signalPhyValue)
                    if signalName == "RLwheelspeed":
                        self.RLwheelspeed = signalPhyValue
                    elif signalName == "RRwheelSpeed":
                        self.RRwheelSpeed = signalPhyValue
                    # 车身信号转换为雷达所需信号-------------------------------------------------------------------------
                    if funFlag == "21":
                        signalValue = [self.RLwheelspeed, self.RRwheelSpeed, self.wheel_base]
                        signalValueOut = self.Body_dll.signalConvert(signalValue, funFlag)
                        signalName = "yawRate"
                    elif funFlag == "02":
                        if signalName == "GearStatus":
                            signalValue = [signalPhyValue, self.gearsourcedata, self.geartargetdata]
                            signalValueOut = self.Body_dll.signalConvert(signalValue, funFlag)
                            self.currentGear = signalValueOut
                        elif signalName == "FrontLeftDoor" or signalName == "FrontRightDoor" or signalName == "RearLeftDoor" or signalName == "RearRightDoor":
                            signalValue = [signalPhyValue, self.doorsourcedata, self.doortargetdata]
                            signalValueOut = self.Body_dll.signalConvert(signalValue, funFlag)
                        elif signalName == "RightTurn":
                            signalValue = [signalPhyValue, self.rightturnswsourcedata, self.rightturnswtargetdata]
                            signalValueOut = self.Body_dll.signalConvert(signalValue, funFlag)
                        elif signalName == "LeftTurn":
                            signalValue = [signalPhyValue, self.leftturnswsourcedata, self.leftturnswtargetdata]
                            signalValueOut = self.Body_dll.signalConvert(signalValue, funFlag)
                        else:
                            print("该信号不在当前映射表中，请更新参数映射表")
                            pass
                    else:
                        signalValueOut = self.Body_dll.signalConvert(signalPhyValue, funFlag)
                        # if signalName == "TemperatureOutside":
                        #     print("转换后的物理值", signalName, signalPhyValue)
                    # 转换后的车身信号赋值给相应的报文-------------------------------------------------------------------
                    if signalName == "yawRate":
                        # print("转换后的物理值", "yawRate", signalValueOut)
                        self.AVP_dll.setHostVehicleYawrate(signalValueOut)
                        self.signallist[1] = signalValueOut
                    elif signalName == "Vspeed":
                        if self.currentGear == 1:
                            self.AVP_dll.setHostVehicleVelocity(-signalValueOut)
                            self.signallist[0] = -signalValueOut
                        else:
                            self.AVP_dll.setHostVehicleVelocity(signalValueOut)
                            self.signallist[0] = signalValueOut
                    elif signalName == "RearLeftDoor":
                        self.AVP_dll.setRearLeftDoorStatus(int(signalValueOut))
                    elif signalName == "RearRightDoor":
                        self.AVP_dll.setRearRightDoorStatus(int(signalValueOut))
                    elif signalName == "FrontLeftDoor":
                        self.AVP_dll.setFrontLeftDoorStatus(int(signalValueOut))
                    elif signalName == "FrontRightDoor":
                        self.AVP_dll.setFrontRightDoorStatus(int(signalValueOut))
                    elif signalName == "RightTurn":
                        # print("RightTurn = ", signalValueOut)
                        self.AVP_dll.setRightTurnSignal(int(signalValueOut))
                    elif signalName == "LeftTurn":
                        self.AVP_dll.setLeftTurnSignal(int(signalValueOut))
                    elif signalName == "GearStatus":
                        self.currentGear = int(signalValueOut)
                        self.AVP_dll.setGearStatus(int(signalValueOut))
                        self.signallist[5] = int(signalValueOut)
                        # print("currentgear = %d" % self.currentGear)
                    elif signalName == "TemperatureOutside":
                        # print("TemperatureOutside = %f" % signalValueOut)
                        self.AVP_dll.setTemperatureOutside(signalValueOut)
                    elif signalName == "SteerWheelAngle":
                        # print("Receive_SteerWheelAngle = %f" % signalValueOut)
                        self.AVP_dll.setStreeringAngle(signalValueOut)
                        self.signallist[4] = signalValueOut
                    elif signalName == "LongitudinalACC":
                        # print("Receive_LongitudinalACC = ", signalValueOut)
                        self.AVP_dll.setLongitudeACC(signalValueOut)
                        self.signallist[2] = signalValueOut
                    elif signalName == "LateralACC":
                        # print("Receive_LateralACC = ", signalValueOut)
                        self.AVP_dll.setLateralACC(signalValueOut)
                        self.signallist[3] = signalValueOut
        # index = self.ui.tableWidget.rowCount()
        # self.ui.tableWidget.insertRow(0)
        for i in range(6):
            self.ui.tableWidget.setItem(0, i, QTableWidgetItem(str(round(self.signallist[i], 3))))
        self.ui.tableWidget.viewport().update()

    # ------------------------------------------------------------------------------------------------------------------
    def commandmsg_send(self):
        global app_log
        data = self.AVP_dll.getByteData(self.AVP_dll.EcuCommandMessage64)
        for k in range(64):
            self.commondmessage.frame.data[k] = data[k]
        for i in range(len(self.initDevice)):
            for j in range(2):
                if self.initDevice[i].checkSendValue[j].isChecked():
                    self.zcanlib.TransmitFD(self.initDevice[i].channelHandle[j], self.commondmessage, 1)
                    if self._logging:
                        message = str(i*2+j)+" "+hex(self.commondmessage.frame.can_id)+" "+str(len(data))+" "+" ".join([hex(x)[2:] for x in data])
                        app_log.info(message)

    def ecutimemsg_seng(self):
        global app_log
        data = self.AVP_dll.getByteData(self.AVP_dll.EcuTimeMessage)
        for k in range(64):
            self.ecutimemessage.frame.data[k] = data[k]
        for i in range(len(self.initDevice)):
            for j in range(2):
                if self.initDevice[i].checkSendValue[j].isChecked():
                    self.zcanlib.TransmitFD(self.initDevice[i].channelHandle[j], self.ecutimemessage, 1)
                    if self._logging:
                        message = str(i*2+j)+" "+hex(self.ecutimemessage.frame.can_id)+" "+str(len(data))+" "+" ".join([hex(x)[2:] for x in data])
                        app_log.info(message)

    def vehiclepart1_send(self):
        global app_log
        data = self.AVP_dll.getByteData(self.AVP_dll.ECUVehicleInformationPart1)
        for k in range(64):
            self.vehiclepart1message.frame.data[k] = data[k]
        for i in range(len(self.initDevice)):
            for j in range(2):
                if self.initDevice[i].checkSendValue[j].isChecked():
                    self.zcanlib.TransmitFD(self.initDevice[i].channelHandle[j], self.vehiclepart1message, 1)
                    if self._logging:
                        message = str(i*2+j)+" "+hex(self.vehiclepart1message.frame.can_id)+" "+str(len(data))+" "+" ".join([hex(x)[2:] for x in data])
                        app_log.info(message)

    def vehiclepart2_send(self):
        global app_log
        data = self.AVP_dll.getByteData(self.AVP_dll.ECUVehicleInformationPart2)
        for k in range(64):
            self.vehiclepart2message.frame.data[k] = data[k]
        for i in range(len(self.initDevice)):
            for j in range(2):
                if self.initDevice[i].checkSendValue[j].isChecked():
                    self.zcanlib.TransmitFD(self.initDevice[i].channelHandle[j], self.vehiclepart2message, 1)
                    if self._logging:
                        message = str(i*2+j)+" "+hex(self.vehiclepart2message.frame.can_id)+" "+str(len(data))+" "+" ".join([hex(x)[2:] for x in data])
                        app_log.info(message)

    def cycle_send(self):
        if self.ui.CycleSend.isChecked():
            self.ecutimemsg_cycle.start(50)
            self.commandmsg_cycle.start(50)
        else:
            self.ecutimemsg_cycle.stop()
            self.commandmsg_cycle.stop()

    def vehiclepart_cyclesend(self):
        if self.ui.routing_start_stop.text() == "转发开启":
            self.cycle_send()
            self.vehiclepart1_cycle.start(25)
            self.vehiclepart2_cycle.start(75)
            self.ui.routing_start_stop.setText("转发关闭")
            self.ui.device_open_close.setEnabled(False)
        else:
            self.vehiclepart1_cycle.stop()
            self.vehiclepart2_cycle.stop()
            self.ecutimemsg_cycle.stop()
            self.commandmsg_cycle.stop()
            self.ui.routing_start_stop.setText("转发开启")
            self.ui.device_open_close.setEnabled(True)

    def log(self):
        global app_log
        if self.ui.log_check.isChecked():
            localtime = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
            log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
            logfile = '.\\log\\' + localtime + '.log'
            my_hadler = RotatingFileHandler(logfile, mode='a', maxBytes=500 * 1024 * 1024, backupCount=1000, encoding=None, delay=0)
            my_hadler.setFormatter(log_formatter)
            my_hadler.setLevel(logging.INFO)
            app_log = logging.getLogger('root')
            app_log.setLevel(logging.INFO)
            app_log.addHandler(my_hadler)
            self._logging = True
        else:
            self._logging = False

    def help(self):
        os.startfile('help.chm')


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = SignalRoutingTool()
    # window.setWindowFlags(QtCore.Qt.WindowMinimizeButtonHint | QtCore.Qt.WindowCloseButtonHint)
    window.show()
    sys.exit(app.exec_())