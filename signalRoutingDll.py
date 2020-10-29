from ctypes import *


class bytedata(Structure):
    _fields_ = [("byte_data", c_ubyte*64)]


class ECUTimestamp(LittleEndianStructure):
    _fields_ = [("Reserved1", c_ubyte, 8),
                ("ECUTime_0", c_ubyte, 8),
                ("ECUTime_1", c_ubyte, 8),
                ("ECUTime_2", c_ubyte, 8),
                ("ECUTime_3", c_ubyte, 8),
                ("ECUTime_4", c_ubyte, 8),
                ("ECUTime_5", c_ubyte, 8),
                ("ECUTime_6", c_ubyte, 8),
                ("Resvered", c_ubyte*56)]


class ECUTimestampInfo(Union):
    _fields_ = [("data", bytedata),
                ("ECUTimestamp", ECUTimestamp)]


class ECUCommandMsg(LittleEndianStructure):
    _fields_ = [("Reserved1", c_ubyte, 4),
                ("SensorMode", c_ubyte, 4),
                ("ADASFunctionSwitch", c_ubyte, 8),
                ("Reserved2", c_ubyte*62)]


class ECUCommandMsgInfo(Union):
    _fields_ = [("data", bytedata),
                ("ECUCommandMsg", ECUCommandMsg)]


class ECUCommandMsg64(LittleEndianStructure):
    _fields_ = [("Reserved1", c_ubyte, 3),
                ("SensorMode", c_ubyte, 5),
                ("ADASFunctionSwitch", c_ubyte, 8),
                ("Reserved2", c_ubyte*62)]


class ECUCommandMsgInfo64(Union):
    _fields_ = [("data", bytedata),
                ("ECUCommandMsg64", ECUCommandMsg64)]


class ECUVehicleInformationPart1Msg(LittleEndianStructure):
    _fields_ = [("HostVehicleYawrate_H", c_ubyte, 8),
                ("HostVehicleVelocity_H", c_ubyte, 4),
                ("HostVehicleYawrate_L", c_ubyte, 4),
                ("HostVehicleVelocity_L", c_ubyte, 8),
                ("WheelPulseFrontLeft", c_ubyte, 8),
                ("WheelPulseFrontRight", c_ubyte, 8),
                ("WheelPulseRearLeft", c_ubyte, 8),
                ("WheelPulseRearRight", c_ubyte, 8),
                ("RearRightDoorStatus", c_ubyte, 1),
                ("RearLeftDoorStatus", c_ubyte, 1),
                ("FrontRightDoorStatus", c_ubyte, 1),
                ("FrontLeftDoorStatus", c_ubyte, 1),
                ("RightTurnSignal", c_ubyte, 2),
                ("LeftTurnSignal", c_ubyte, 2),
                ("Reserved", c_ubyte*56)]


class ECUVehicleInformationPart1MsgInfo(Union):
    _fields_ = [("data", bytedata),
                ("ECUVehicleInformationPart1Msg", ECUVehicleInformationPart1Msg)]


class ECUVehicleInformationPart2Msg(LittleEndianStructure):
    _fields_ = [("Reserved1", c_ubyte, 4),
                ("WindshieldWiper", c_ubyte, 2),
                ("GearStatus", c_ubyte, 2),
                ("TemperatureOutside", c_ubyte, 8),
                ("StreeringAngle_H", c_ubyte, 8),
                ("LongitudeACC_H", c_ubyte, 2),
                ("StreeringAngle_L", c_ubyte, 6),
                ("LongitudeACC_L", c_ubyte, 8),
                ("LateralACC", c_ubyte, 8),
                ("Reserved6", c_ubyte, 8),
                ("Reserved7", c_ubyte, 8),
                ("Reserved", c_ubyte*56)]


class ECUVehicleInformationPart2MsgInfo(Union):
    _fields_ = [("data", bytedata),
                ("ECUVehicleInformationPart2Msg", ECUVehicleInformationPart2Msg)]


class AVPDll(object):
    def __init__(self):
        # 报文初始化(开辟报文内存空间)
        self.EcuTimeMessage = ECUTimestampInfo()
        self.EcuCommandMessage64 = ECUCommandMsgInfo64()
        self.ECUVehicleInformationPart1 = ECUVehicleInformationPart1MsgInfo()
        self.ECUVehicleInformationPart2 = ECUVehicleInformationPart2MsgInfo()

    # ECUTime Frame
    def setEcuTime(self, time):
        self.EcuTimeMessage.ECUTimestamp.ECUTime_0 = time >> 48 & 0xFF
        self.EcuTimeMessage.ECUTimestamp.ECUTime_1 = time >> 40 & 0xFF
        self.EcuTimeMessage.ECUTimestamp.ECUTime_2 = time >> 32 & 0xFF
        self.EcuTimeMessage.ECUTimestamp.ECUTime_3 = time >> 24 & 0xFF
        self.EcuTimeMessage.ECUTimestamp.ECUTime_4 = time >> 16 & 0xFF
        self.EcuTimeMessage.ECUTimestamp.ECUTime_5 = time >> 8 & 0xFF
        self.EcuTimeMessage.ECUTimestamp.ECUTime_6 = time & 0xFF

    # Command Frame
    def setSensorMode(self, mode):
        self.EcuCommandMessage64.ECUCommandMsg64.SensorMode = mode

    def setAdasSwitch(self, adas):
        self.EcuCommandMessage64.ECUCommandMsg64.ADASFunctionSwitch = adas

    # Vehicle part1 Frame
    def setHostVehicleYawrate(self, hostyawrate):
        hostyawrate += 102.35
        hostyawrate *= 20
        yawrate = int(hostyawrate + 0.5)
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.HostVehicleYawrate_H = yawrate >> 4 & 0xFF
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.HostVehicleYawrate_L = yawrate & 0xF

    def setHostVehicleVelocity(self, hostvelcoity):
        hostvelcoity += 71.645 + 0.0175
        hostvelcoity /= 0.035
        velcoity = int(hostvelcoity + 0.5)
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.HostVehicleVelocity_H = velcoity >> 8 & 0xF
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.HostVehicleVelocity_L = velcoity & 0xFF

    def setWheelPulseFrontLeft(self, WheelPulseFrontLeft):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.WheelPulseFrontLeft = WheelPulseFrontLeft

    def setWheelPulseFrontRight(self, WheelPulseFrontRight):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.WheelPulseFrontRight = WheelPulseFrontRight

    def setWheelPulseRearLeft(self, WheelPulseRearLeft):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.WheelPulseRearLeft = WheelPulseRearLeft

    def setWheelPulseRearRight(self, WheelPulseRearRight):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.WheelPulseRearRight = WheelPulseRearRight

    def setRearRightDoorStatus(self, RearRightDoorStatus):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.RearRightDoorStatus = RearRightDoorStatus

    def setRearLeftDoorStatus(self, RearLeftDoorStatus):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.RearLeftDoorStatus = RearLeftDoorStatus

    def setFrontRightDoorStatus(self, FrontRightDoorStatus):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.FrontRightDoorStatus = FrontRightDoorStatus

    def setFrontLeftDoorStatus(self, FrontLeftDoorStatus):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.FrontLeftDoorStatus = FrontLeftDoorStatus

    def setRightTurnSignal(self, RightTurnSignal):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.RightTurnSignal = RightTurnSignal

    def setLeftTurnSignal(self, LeftTurnSignal):
        self.ECUVehicleInformationPart1.ECUVehicleInformationPart1Msg.LeftTurnSignal = LeftTurnSignal

    # Vehicle part1 Frame
    def setWindshieldWiper(self, WindshieldWiper):
        self.ECUVehicleInformationPart2.ECUVehicleInformationPart2Msg.WindshieldWiper = WindshieldWiper

    def setGearStatus(self, GearStatus):
        self.ECUVehicleInformationPart2.ECUVehicleInformationPart2Msg.GearStatus = GearStatus

    def setTemperatureOutside(self, TemperatureOutside):
        TemperatureOutside += 40
        TemperatureOutside /= 0.5
        temperature = int(TemperatureOutside + 0.5)
        # print(hex(temperature))
        self.ECUVehicleInformationPart2.ECUVehicleInformationPart2Msg.TemperatureOutside = temperature

    def setStreeringAngle(self, StreeringAngle):
        StreeringAngle += 780
        StreeringAngle /= 0.1
        angle = int(StreeringAngle+0.5)
        self.ECUVehicleInformationPart2.ECUVehicleInformationPart2Msg.StreeringAngle_H = angle >> 6 & 0xFF
        self.ECUVehicleInformationPart2.ECUVehicleInformationPart2Msg.StreeringAngle_L = angle & 0x3F

    def setLongitudeACC(self, LongitudeACC):
        LongitudeACC += 16
        LongitudeACC /= 0.03125
        ACC = int(LongitudeACC + 0.5)
        self.ECUVehicleInformationPart2.ECUVehicleInformationPart2Msg.LongitudeACC_H = ACC >> 8 & 0x3
        self.ECUVehicleInformationPart2.ECUVehicleInformationPart2Msg.LongitudeACC_L = ACC & 0xFF

    def setLateralACC(self, LateralACC):
        LateralACC += 1.27
        LateralACC /= 0.01
        ACC = int(LateralACC + 0.5)
        self.ECUVehicleInformationPart2.ECUVehicleInformationPart2Msg.LateralACC = ACC & 0xFF

    def getByteData(self, message):
        data = [0 for m in range(64)]
        for i in range(64):
            data[i] = message.data.byte_data[i]
        return data


class BodyDll(object):
    def signed_type(self, data, length):
        if length == 8:
            tempdata = cast(pointer(c_uint8(data)), POINTER(c_int8)).contents.value
        elif length == 16:
            tempdata = cast(pointer(c_uint16(data)), POINTER(c_int16)).contents.value
        else:
            tempdata = cast(pointer(c_uint32(data)), POINTER(c_int32)).contents.value
        return tempdata

    def getSignalPhysValue(self, data, signal):
        try:
            name = signal['name']
            startBit = int(signal['startBit'])
            len = int(signal['len'])
            factor = float(signal['factor'])
            offset = float(signal['offset'])
            # print(offset)
            funFlag = signal['funFlag']
            unsigned = signal['unsigned']
            # print(unsigned, type(unsigned))
            startByte = startBit // 8
            startBytePosition = startBit % 8
            tempByte = 0x00
            tempByte1 = 0
            for i in range(startByte+1):
                tempByte = tempByte << 8
                tempByte |= data[i]
                tempByte1 |= ((tempByte1 << 8) | 0xFF)
            signalValue = (tempByte >> startBytePosition) & (tempByte1 >> ((startByte + 1) * 8 - len))
            if unsigned == "False":
                if len == 8:
                    signalValue = self.signed_type(signalValue, 8)
                elif len == 16:
                    signalValue = self.signed_type(signalValue, 16)
                elif len == 32:
                    signalValue = self.signed_type(signalValue, 32)
            signalValuePhy = signalValue * factor + offset
            return name, signalValuePhy, funFlag
        except:
            print("signal format is not correct")
            return name, 0, funFlag

    def signalConvert(self, signalValue, funFlag):
        if funFlag == "00":
            signalValueOut = signalValue
        elif funFlag == "01":
            signalValueOut = signalValue * -1
        elif funFlag == "10":  # km/h -> m/s
            signalValueOut = signalValue / 3.6
        elif funFlag == "11":
            signalValueOut = signalValue /3.14159 * 180
        elif funFlag == "21":
            # 通过轮速计算车辆横摆角速度
            # signalValue[0]:wheelSpeedRL  signalValue[1]:wheelSpeedRR  signalValue[2]:wheelbase
            signalValueOut = (signalValue[0] - signalValue[1]) / 3.6 / signalValue[2]
        elif funFlag == "02":
            # signalValue格式[0, [0,1,2], [2,1,0]] 信号映射
            # signalValue[0]:原始信号值，signalValue[1]:车身信号的取值范围，signalValue[1]:与车身信号对应的取值范围
            pos = signalValue[1].index(signalValue[0])
            signalValueOut = signalValue[2][pos]
        else:
            signalValueOut = 0
        return signalValueOut

