# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'signalRoutingWindow.ui'
#
# Created by: PyQt5 UI code generator 5.13.0
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(544, 389)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("signalrouting.ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.device1_check = QtWidgets.QCheckBox(self.groupBox)
        self.device1_check.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device1_check.setObjectName("device1_check")
        self.gridLayout_2.addWidget(self.device1_check, 0, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.groupBox)
        self.label.setObjectName("label")
        self.gridLayout_2.addWidget(self.label, 0, 1, 1, 1)
        self.device1_chn0_type = QtWidgets.QComboBox(self.groupBox)
        self.device1_chn0_type.setObjectName("device1_chn0_type")
        self.device1_chn0_type.addItem("")
        self.device1_chn0_type.addItem("")
        self.gridLayout_2.addWidget(self.device1_chn0_type, 0, 2, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.groupBox)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 0, 3, 1, 1)
        self.device1_chn0_Arb = QtWidgets.QComboBox(self.groupBox)
        self.device1_chn0_Arb.setObjectName("device1_chn0_Arb")
        self.device1_chn0_Arb.addItem("")
        self.device1_chn0_Arb.addItem("")
        self.device1_chn0_Arb.addItem("")
        self.device1_chn0_Arb.addItem("")
        self.device1_chn0_Arb.addItem("")
        self.device1_chn0_Arb.addItem("")
        self.device1_chn0_Arb.addItem("")
        self.gridLayout_2.addWidget(self.device1_chn0_Arb, 0, 4, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.groupBox)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 0, 5, 1, 1)
        self.device1_chn0_Data = QtWidgets.QComboBox(self.groupBox)
        self.device1_chn0_Data.setObjectName("device1_chn0_Data")
        self.device1_chn0_Data.addItem("")
        self.device1_chn0_Data.addItem("")
        self.device1_chn0_Data.addItem("")
        self.device1_chn0_Data.addItem("")
        self.gridLayout_2.addWidget(self.device1_chn0_Data, 0, 6, 1, 1)
        self.device1_chn0_Rx = QtWidgets.QCheckBox(self.groupBox)
        self.device1_chn0_Rx.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device1_chn0_Rx.setObjectName("device1_chn0_Rx")
        self.gridLayout_2.addWidget(self.device1_chn0_Rx, 0, 7, 1, 1)
        self.device1_chn0_Tx = QtWidgets.QCheckBox(self.groupBox)
        self.device1_chn0_Tx.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device1_chn0_Tx.setObjectName("device1_chn0_Tx")
        self.gridLayout_2.addWidget(self.device1_chn0_Tx, 0, 8, 1, 1)
        self.device1_index = QtWidgets.QComboBox(self.groupBox)
        self.device1_index.setMinimumSize(QtCore.QSize(30, 19))
        self.device1_index.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.device1_index.setObjectName("device1_index")
        self.device1_index.addItem("")
        self.device1_index.addItem("")
        self.device1_index.addItem("")
        self.device1_index.addItem("")
        self.gridLayout_2.addWidget(self.device1_index, 1, 0, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.groupBox)
        self.label_7.setObjectName("label_7")
        self.gridLayout_2.addWidget(self.label_7, 1, 1, 1, 1)
        self.device1_chn1_type = QtWidgets.QComboBox(self.groupBox)
        self.device1_chn1_type.setObjectName("device1_chn1_type")
        self.device1_chn1_type.addItem("")
        self.device1_chn1_type.addItem("")
        self.gridLayout_2.addWidget(self.device1_chn1_type, 1, 2, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.groupBox)
        self.label_8.setObjectName("label_8")
        self.gridLayout_2.addWidget(self.label_8, 1, 3, 1, 1)
        self.device1_chn1_Arb = QtWidgets.QComboBox(self.groupBox)
        self.device1_chn1_Arb.setObjectName("device1_chn1_Arb")
        self.device1_chn1_Arb.addItem("")
        self.device1_chn1_Arb.addItem("")
        self.device1_chn1_Arb.addItem("")
        self.device1_chn1_Arb.addItem("")
        self.device1_chn1_Arb.addItem("")
        self.device1_chn1_Arb.addItem("")
        self.device1_chn1_Arb.addItem("")
        self.gridLayout_2.addWidget(self.device1_chn1_Arb, 1, 4, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.groupBox)
        self.label_9.setObjectName("label_9")
        self.gridLayout_2.addWidget(self.label_9, 1, 5, 1, 1)
        self.device1_chn1_Data = QtWidgets.QComboBox(self.groupBox)
        self.device1_chn1_Data.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.device1_chn1_Data.setObjectName("device1_chn1_Data")
        self.device1_chn1_Data.addItem("")
        self.device1_chn1_Data.addItem("")
        self.device1_chn1_Data.addItem("")
        self.device1_chn1_Data.addItem("")
        self.gridLayout_2.addWidget(self.device1_chn1_Data, 1, 6, 1, 1)
        self.device1_chn1_Rx = QtWidgets.QCheckBox(self.groupBox)
        self.device1_chn1_Rx.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device1_chn1_Rx.setObjectName("device1_chn1_Rx")
        self.gridLayout_2.addWidget(self.device1_chn1_Rx, 1, 7, 1, 1)
        self.device1_chn1_Tx = QtWidgets.QCheckBox(self.groupBox)
        self.device1_chn1_Tx.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device1_chn1_Tx.setObjectName("device1_chn1_Tx")
        self.gridLayout_2.addWidget(self.device1_chn1_Tx, 1, 8, 1, 1)
        self.device2_check = QtWidgets.QCheckBox(self.groupBox)
        self.device2_check.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device2_check.setObjectName("device2_check")
        self.gridLayout_2.addWidget(self.device2_check, 2, 0, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.groupBox)
        self.label_10.setObjectName("label_10")
        self.gridLayout_2.addWidget(self.label_10, 2, 1, 1, 1)
        self.device2_chn0_type = QtWidgets.QComboBox(self.groupBox)
        self.device2_chn0_type.setObjectName("device2_chn0_type")
        self.device2_chn0_type.addItem("")
        self.device2_chn0_type.addItem("")
        self.gridLayout_2.addWidget(self.device2_chn0_type, 2, 2, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.groupBox)
        self.label_11.setObjectName("label_11")
        self.gridLayout_2.addWidget(self.label_11, 2, 3, 1, 1)
        self.device2_chn0_Arb = QtWidgets.QComboBox(self.groupBox)
        self.device2_chn0_Arb.setObjectName("device2_chn0_Arb")
        self.device2_chn0_Arb.addItem("")
        self.device2_chn0_Arb.addItem("")
        self.device2_chn0_Arb.addItem("")
        self.device2_chn0_Arb.addItem("")
        self.device2_chn0_Arb.addItem("")
        self.device2_chn0_Arb.addItem("")
        self.device2_chn0_Arb.addItem("")
        self.gridLayout_2.addWidget(self.device2_chn0_Arb, 2, 4, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.groupBox)
        self.label_12.setObjectName("label_12")
        self.gridLayout_2.addWidget(self.label_12, 2, 5, 1, 1)
        self.device2_chn0_Data = QtWidgets.QComboBox(self.groupBox)
        self.device2_chn0_Data.setObjectName("device2_chn0_Data")
        self.device2_chn0_Data.addItem("")
        self.device2_chn0_Data.addItem("")
        self.device2_chn0_Data.addItem("")
        self.device2_chn0_Data.addItem("")
        self.gridLayout_2.addWidget(self.device2_chn0_Data, 2, 6, 1, 1)
        self.device2_chn0_Rx = QtWidgets.QCheckBox(self.groupBox)
        self.device2_chn0_Rx.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device2_chn0_Rx.setObjectName("device2_chn0_Rx")
        self.gridLayout_2.addWidget(self.device2_chn0_Rx, 2, 7, 1, 1)
        self.device2_chn0_Tx = QtWidgets.QCheckBox(self.groupBox)
        self.device2_chn0_Tx.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device2_chn0_Tx.setObjectName("device2_chn0_Tx")
        self.gridLayout_2.addWidget(self.device2_chn0_Tx, 2, 8, 1, 1)
        self.device2_index = QtWidgets.QComboBox(self.groupBox)
        self.device2_index.setMinimumSize(QtCore.QSize(30, 19))
        self.device2_index.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.device2_index.setObjectName("device2_index")
        self.device2_index.addItem("")
        self.device2_index.addItem("")
        self.device2_index.addItem("")
        self.device2_index.addItem("")
        self.gridLayout_2.addWidget(self.device2_index, 3, 0, 1, 1)
        self.label_22 = QtWidgets.QLabel(self.groupBox)
        self.label_22.setObjectName("label_22")
        self.gridLayout_2.addWidget(self.label_22, 3, 1, 1, 1)
        self.device2_chn1_type = QtWidgets.QComboBox(self.groupBox)
        self.device2_chn1_type.setObjectName("device2_chn1_type")
        self.device2_chn1_type.addItem("")
        self.device2_chn1_type.addItem("")
        self.gridLayout_2.addWidget(self.device2_chn1_type, 3, 2, 1, 1)
        self.label_23 = QtWidgets.QLabel(self.groupBox)
        self.label_23.setObjectName("label_23")
        self.gridLayout_2.addWidget(self.label_23, 3, 3, 1, 1)
        self.device2_chn1_Arb = QtWidgets.QComboBox(self.groupBox)
        self.device2_chn1_Arb.setObjectName("device2_chn1_Arb")
        self.device2_chn1_Arb.addItem("")
        self.device2_chn1_Arb.addItem("")
        self.device2_chn1_Arb.addItem("")
        self.device2_chn1_Arb.addItem("")
        self.device2_chn1_Arb.addItem("")
        self.device2_chn1_Arb.addItem("")
        self.device2_chn1_Arb.addItem("")
        self.gridLayout_2.addWidget(self.device2_chn1_Arb, 3, 4, 1, 1)
        self.label_24 = QtWidgets.QLabel(self.groupBox)
        self.label_24.setObjectName("label_24")
        self.gridLayout_2.addWidget(self.label_24, 3, 5, 1, 1)
        self.device2_chn1_Data = QtWidgets.QComboBox(self.groupBox)
        self.device2_chn1_Data.setObjectName("device2_chn1_Data")
        self.device2_chn1_Data.addItem("")
        self.device2_chn1_Data.addItem("")
        self.device2_chn1_Data.addItem("")
        self.device2_chn1_Data.addItem("")
        self.gridLayout_2.addWidget(self.device2_chn1_Data, 3, 6, 1, 1)
        self.device2_chn1_Rx = QtWidgets.QCheckBox(self.groupBox)
        self.device2_chn1_Rx.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device2_chn1_Rx.setObjectName("device2_chn1_Rx")
        self.gridLayout_2.addWidget(self.device2_chn1_Rx, 3, 7, 1, 1)
        self.device2_chn1_Tx = QtWidgets.QCheckBox(self.groupBox)
        self.device2_chn1_Tx.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.device2_chn1_Tx.setObjectName("device2_chn1_Tx")
        self.gridLayout_2.addWidget(self.device2_chn1_Tx, 3, 8, 1, 1)
        self.verticalLayout.addWidget(self.groupBox)
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setObjectName("groupBox_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.groupBox_2)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.CycleSend = QtWidgets.QCheckBox(self.groupBox_2)
        self.CycleSend.setObjectName("CycleSend")
        self.horizontalLayout_2.addWidget(self.CycleSend)
        self.sensor_mode = QtWidgets.QComboBox(self.groupBox_2)
        self.sensor_mode.setObjectName("sensor_mode")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.sensor_mode.addItem("")
        self.horizontalLayout_2.addWidget(self.sensor_mode)
        self.BSD = QtWidgets.QCheckBox(self.groupBox_2)
        self.BSD.setObjectName("BSD")
        self.horizontalLayout_2.addWidget(self.BSD)
        self.LCA = QtWidgets.QCheckBox(self.groupBox_2)
        self.LCA.setObjectName("LCA")
        self.horizontalLayout_2.addWidget(self.LCA)
        self.DOW = QtWidgets.QCheckBox(self.groupBox_2)
        self.DOW.setObjectName("DOW")
        self.horizontalLayout_2.addWidget(self.DOW)
        self.FCTA = QtWidgets.QCheckBox(self.groupBox_2)
        self.FCTA.setObjectName("FCTA")
        self.horizontalLayout_2.addWidget(self.FCTA)
        self.RCTA = QtWidgets.QCheckBox(self.groupBox_2)
        self.RCTA.setObjectName("RCTA")
        self.horizontalLayout_2.addWidget(self.RCTA)
        self.verticalLayout.addWidget(self.groupBox_2)
        self.groupBox_4 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_4.setObjectName("groupBox_4")
        self.gridLayout = QtWidgets.QGridLayout(self.groupBox_4)
        self.gridLayout.setObjectName("gridLayout")
        self.device_open_close = QtWidgets.QPushButton(self.groupBox_4)
        self.device_open_close.setObjectName("device_open_close")
        self.gridLayout.addWidget(self.device_open_close, 0, 1, 1, 1)
        self.routing_start_stop = QtWidgets.QPushButton(self.groupBox_4)
        self.routing_start_stop.setObjectName("routing_start_stop")
        self.gridLayout.addWidget(self.routing_start_stop, 0, 2, 1, 1)
        self.log_check = QtWidgets.QCheckBox(self.groupBox_4)
        self.log_check.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.log_check.setObjectName("log_check")
        self.gridLayout.addWidget(self.log_check, 0, 3, 1, 1)
        self.verticalLayout.addWidget(self.groupBox_4)
        self.tableWidget = QtWidgets.QTableWidget(self.centralwidget)
        self.tableWidget.setObjectName("tableWidget")
        self.tableWidget.setColumnCount(6)
        self.tableWidget.setRowCount(1)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setVerticalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(2, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(3, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(4, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget.setHorizontalHeaderItem(5, item)
        self.verticalLayout.addWidget(self.tableWidget)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 544, 18))
        self.menubar.setObjectName("menubar")
        self.menuHelp = QtWidgets.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionhelp = QtWidgets.QAction(MainWindow)
        self.actionhelp.setObjectName("actionhelp")
        self.menubar.addAction(self.menuHelp.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "SignalRoutingTool_V2.0"))
        self.groupBox.setTitle(_translate("MainWindow", "设备选择"))
        self.device1_check.setText(_translate("MainWindow", "设备1"))
        self.label.setText(_translate("MainWindow", "CAN0："))
        self.device1_chn0_type.setItemText(0, _translate("MainWindow", "CAN"))
        self.device1_chn0_type.setItemText(1, _translate("MainWindow", "CANFD"))
        self.label_2.setText(_translate("MainWindow", "Arb："))
        self.device1_chn0_Arb.setItemText(0, _translate("MainWindow", "50K"))
        self.device1_chn0_Arb.setItemText(1, _translate("MainWindow", "100K"))
        self.device1_chn0_Arb.setItemText(2, _translate("MainWindow", "125K"))
        self.device1_chn0_Arb.setItemText(3, _translate("MainWindow", "250K"))
        self.device1_chn0_Arb.setItemText(4, _translate("MainWindow", "500K"))
        self.device1_chn0_Arb.setItemText(5, _translate("MainWindow", "800K"))
        self.device1_chn0_Arb.setItemText(6, _translate("MainWindow", "1M"))
        self.label_3.setText(_translate("MainWindow", "Data："))
        self.device1_chn0_Data.setItemText(0, _translate("MainWindow", "1M"))
        self.device1_chn0_Data.setItemText(1, _translate("MainWindow", "2M"))
        self.device1_chn0_Data.setItemText(2, _translate("MainWindow", "4M"))
        self.device1_chn0_Data.setItemText(3, _translate("MainWindow", "5M"))
        self.device1_chn0_Rx.setText(_translate("MainWindow", "Rx："))
        self.device1_chn0_Tx.setText(_translate("MainWindow", "Tx："))
        self.device1_index.setItemText(0, _translate("MainWindow", "0"))
        self.device1_index.setItemText(1, _translate("MainWindow", "1"))
        self.device1_index.setItemText(2, _translate("MainWindow", "2"))
        self.device1_index.setItemText(3, _translate("MainWindow", "3"))
        self.label_7.setText(_translate("MainWindow", "CAN1："))
        self.device1_chn1_type.setItemText(0, _translate("MainWindow", "CAN"))
        self.device1_chn1_type.setItemText(1, _translate("MainWindow", "CANFD"))
        self.label_8.setText(_translate("MainWindow", "Arb："))
        self.device1_chn1_Arb.setItemText(0, _translate("MainWindow", "50K"))
        self.device1_chn1_Arb.setItemText(1, _translate("MainWindow", "100K"))
        self.device1_chn1_Arb.setItemText(2, _translate("MainWindow", "125K"))
        self.device1_chn1_Arb.setItemText(3, _translate("MainWindow", "250K"))
        self.device1_chn1_Arb.setItemText(4, _translate("MainWindow", "500K"))
        self.device1_chn1_Arb.setItemText(5, _translate("MainWindow", "800K"))
        self.device1_chn1_Arb.setItemText(6, _translate("MainWindow", "1M"))
        self.label_9.setText(_translate("MainWindow", "Data："))
        self.device1_chn1_Data.setItemText(0, _translate("MainWindow", "1M"))
        self.device1_chn1_Data.setItemText(1, _translate("MainWindow", "2M"))
        self.device1_chn1_Data.setItemText(2, _translate("MainWindow", "4M"))
        self.device1_chn1_Data.setItemText(3, _translate("MainWindow", "5M"))
        self.device1_chn1_Rx.setText(_translate("MainWindow", "Rx："))
        self.device1_chn1_Tx.setText(_translate("MainWindow", "Tx："))
        self.device2_check.setText(_translate("MainWindow", "设备2"))
        self.label_10.setText(_translate("MainWindow", "CAN0："))
        self.device2_chn0_type.setItemText(0, _translate("MainWindow", "CAN"))
        self.device2_chn0_type.setItemText(1, _translate("MainWindow", "CANFD"))
        self.label_11.setText(_translate("MainWindow", "Arb："))
        self.device2_chn0_Arb.setItemText(0, _translate("MainWindow", "50K"))
        self.device2_chn0_Arb.setItemText(1, _translate("MainWindow", "100K"))
        self.device2_chn0_Arb.setItemText(2, _translate("MainWindow", "125K"))
        self.device2_chn0_Arb.setItemText(3, _translate("MainWindow", "250K"))
        self.device2_chn0_Arb.setItemText(4, _translate("MainWindow", "500K"))
        self.device2_chn0_Arb.setItemText(5, _translate("MainWindow", "800K"))
        self.device2_chn0_Arb.setItemText(6, _translate("MainWindow", "1M"))
        self.label_12.setText(_translate("MainWindow", "Data："))
        self.device2_chn0_Data.setItemText(0, _translate("MainWindow", "1M"))
        self.device2_chn0_Data.setItemText(1, _translate("MainWindow", "2M"))
        self.device2_chn0_Data.setItemText(2, _translate("MainWindow", "4M"))
        self.device2_chn0_Data.setItemText(3, _translate("MainWindow", "5M"))
        self.device2_chn0_Rx.setText(_translate("MainWindow", "Rx："))
        self.device2_chn0_Tx.setText(_translate("MainWindow", "Tx："))
        self.device2_index.setItemText(0, _translate("MainWindow", "0"))
        self.device2_index.setItemText(1, _translate("MainWindow", "1"))
        self.device2_index.setItemText(2, _translate("MainWindow", "2"))
        self.device2_index.setItemText(3, _translate("MainWindow", "3"))
        self.label_22.setText(_translate("MainWindow", "CAN1："))
        self.device2_chn1_type.setItemText(0, _translate("MainWindow", "CAN"))
        self.device2_chn1_type.setItemText(1, _translate("MainWindow", "CANFD"))
        self.label_23.setText(_translate("MainWindow", "Arb："))
        self.device2_chn1_Arb.setItemText(0, _translate("MainWindow", "50K"))
        self.device2_chn1_Arb.setItemText(1, _translate("MainWindow", "100K"))
        self.device2_chn1_Arb.setItemText(2, _translate("MainWindow", "125K"))
        self.device2_chn1_Arb.setItemText(3, _translate("MainWindow", "250K"))
        self.device2_chn1_Arb.setItemText(4, _translate("MainWindow", "500K"))
        self.device2_chn1_Arb.setItemText(5, _translate("MainWindow", "800K"))
        self.device2_chn1_Arb.setItemText(6, _translate("MainWindow", "1M"))
        self.label_24.setText(_translate("MainWindow", "Data："))
        self.device2_chn1_Data.setItemText(0, _translate("MainWindow", "1M"))
        self.device2_chn1_Data.setItemText(1, _translate("MainWindow", "2M"))
        self.device2_chn1_Data.setItemText(2, _translate("MainWindow", "4M"))
        self.device2_chn1_Data.setItemText(3, _translate("MainWindow", "5M"))
        self.device2_chn1_Rx.setText(_translate("MainWindow", "Rx："))
        self.device2_chn1_Tx.setText(_translate("MainWindow", "Tx："))
        self.groupBox_2.setTitle(_translate("MainWindow", "模式选择/ADAS"))
        self.CycleSend.setText(_translate("MainWindow", "CycleSend"))
        self.sensor_mode.setItemText(0, _translate("MainWindow", "Sleep"))
        self.sensor_mode.setItemText(1, _translate("MainWindow", "Standby"))
        self.sensor_mode.setItemText(2, _translate("MainWindow", "ADAS"))
        self.sensor_mode.setItemText(3, _translate("MainWindow", "LongRangeObject&Freespace(Fusion)"))
        self.sensor_mode.setItemText(4, _translate("MainWindow", "Reserved"))
        self.sensor_mode.setItemText(5, _translate("MainWindow", "ShortRangeObject&Freespace(Fusion)"))
        self.sensor_mode.setItemText(6, _translate("MainWindow", "Reserved"))
        self.sensor_mode.setItemText(7, _translate("MainWindow", "LongRangePointcloud"))
        self.sensor_mode.setItemText(8, _translate("MainWindow", "ShortRangePointcloud"))
        self.sensor_mode.setItemText(9, _translate("MainWindow", "Reserved"))
        self.sensor_mode.setItemText(10, _translate("MainWindow", "LongRangeObject&Freespace(5°,noFusion)"))
        self.sensor_mode.setItemText(11, _translate("MainWindow", "Reserved"))
        self.sensor_mode.setItemText(12, _translate("MainWindow", "ShortRangeObject&Freespace(5°,noFusion)"))
        self.sensor_mode.setItemText(13, _translate("MainWindow", "Reserved"))
        self.sensor_mode.setItemText(14, _translate("MainWindow", "LongRangeObject&Freespace(Fusion)&ADAS&Guardrail"))
        self.sensor_mode.setItemText(15, _translate("MainWindow", "LongRangeObject&Freespace(5°,noFusion)&ADAS&Guardrail"))
        self.BSD.setText(_translate("MainWindow", "BSD"))
        self.LCA.setText(_translate("MainWindow", "LCA"))
        self.DOW.setText(_translate("MainWindow", "DOW"))
        self.FCTA.setText(_translate("MainWindow", "FCTA"))
        self.RCTA.setText(_translate("MainWindow", "RCTA"))
        self.groupBox_4.setTitle(_translate("MainWindow", "启动/关闭/log"))
        self.device_open_close.setText(_translate("MainWindow", "打开设备"))
        self.routing_start_stop.setText(_translate("MainWindow", "转发开启"))
        self.log_check.setText(_translate("MainWindow", "log"))
        item = self.tableWidget.verticalHeaderItem(0)
        item.setText(_translate("MainWindow", "值"))
        item = self.tableWidget.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "车速(m/s)"))
        item = self.tableWidget.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "横摆角速度(deg/s)"))
        item = self.tableWidget.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "纵向加速度(m/s²)"))
        item = self.tableWidget.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "横向加速度(m/s²)"))
        item = self.tableWidget.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "方向盘转角(°)"))
        item = self.tableWidget.horizontalHeaderItem(5)
        item.setText(_translate("MainWindow", "挡位"))
        self.menuHelp.setTitle(_translate("MainWindow", "Help"))
        self.actionhelp.setText(_translate("MainWindow", "help"))