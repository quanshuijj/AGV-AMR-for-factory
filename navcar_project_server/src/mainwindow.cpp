#include "navcar_project_server/mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QMenu>
#include <cmath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //核心节点管理类
    //navCarManagerNode.moveToThread(&navCarManagerNodeThread);
    //LIN操作
    connect(this,SIGNAL(ConnectToLIN1PortSignal(QString,qint32)),&navCarManagerNode,
            SIGNAL(ConnectToLIN1PortSignal(QString,qint32)));
    connect(this,SIGNAL(ConnectToLIN2PortSignal(QString,qint32)),&navCarManagerNode,
            SIGNAL(ConnectToLIN1PortSignal(QString,qint32)));
    connect(this,SIGNAL(DisConnectToLIN1Signal()),&navCarManagerNode,
            SIGNAL(DisConnectToLIN1Signal()));
    connect(this,SIGNAL(DisConnectToLIN2Signal()),&navCarManagerNode,
            SIGNAL(DisConnectToLIN2Signal()));
    connect(this,SIGNAL(RefreshAvailablePortsSignal()),&navCarManagerNode,
            SIGNAL(RefreshAvailablePortsSignal()));
    connect(this,SIGNAL(SendLIN1MsgSignal(quint8,QByteArray)),&navCarManagerNode,
            SIGNAL(SendLIN1MsgSignal(quint8,QByteArray)));
    // connect(this,SIGNAL(SendLIN2MsgSignal(quint8,QByteArray)),&navCarManagerNode,
    //         SIGNAL(SendLIN2MsgSignal(quint8,QByteArray)));


    connect(&navCarManagerNode,SIGNAL(RefreshPortDataSignal(QStringList)),this,
            SLOT(refreshPortData(QStringList)));
    connect(&navCarManagerNode,SIGNAL(LogDataSignal(QString)),this,
            SLOT(logDataUpdate(QString)));

    //CAN操作
    connect(this,SIGNAL(RefreshAvailableCANPortsSignal()),&navCarManagerNode,
            SIGNAL(RefreshAvailableCANPortsSignal()));
    connect(&navCarManagerNode,SIGNAL(RefreshCANPortDataSignal(QStringList)),this,
            SLOT(refreshCANPortData(QStringList)));

    //CAN读取
    connect(&navCarManagerNode, SIGNAL(canMotorDataUpdate(int,int,bool,float,float,float)),
        this, SLOT(updateMotorDisplay(int,int,bool,float,float,float)));
    connect(&navCarManagerNode, SIGNAL(canOilPressureUpdate(float,float,float,float)),
        this, SLOT(updateOilPressureDisplay(float,float,float,float)));
    connect(&navCarManagerNode, SIGNAL(canOilTemperatureUpdate(float)),
        this, SLOT(updateOilTemperatureDisplay(float)));
    connect(&navCarManagerNode, SIGNAL(canModeUpdate(int)),
        this, SLOT(updateModeDisplay(int)));
    connect(&navCarManagerNode, SIGNAL(canHydraulicLockUpdate(bool,bool,bool,bool)),
        this, SLOT(updateHydraulicLockDisplay(bool,bool,bool,bool)));
    connect(&navCarManagerNode, SIGNAL(canLightUpdate(bool,bool,bool,bool)),
        this, SLOT(updateLightDisplay(bool,bool,bool,bool)));
    connect(&navCarManagerNode, SIGNAL(canEmergencyUpdate(bool,bool,bool,bool)),
        this, SLOT(updateEmergencyDisplay(bool,bool,bool,bool)));
    connect(&navCarManagerNode, SIGNAL(canbumpmotorspeedUpdate(float)),
        this, SLOT(updateBumpMotorSpeedDisplay(float)));
    connect(&navCarManagerNode, SIGNAL(canEncoderUpdate(int,quint8,float)),
        this, SLOT(updateEncoderDisplay(int,quint8,float)));
    connect(&navCarManagerNode, SIGNAL(canOilPumpUpdate(float,float,float,float)),
        this, SLOT(updateOilPumpDisplay(float,float,float,float)));
    connect(&navCarManagerNode, SIGNAL(canBatteryStatusUpdate(int,int,int,int)),
        this, SLOT(updateBatteryStatusDisplay(int,int,int,int)));
    // 速度更新
    connect(&navCarManagerNode, SIGNAL(calculatedVelocityUpdate(double,double)),
            this, SLOT(updateVelocityDisplay(double,double)));
    
        // odom数据更新（通过信号转发）
        connect(&navCarManagerNode, SIGNAL(odomUpdate(nav_msgs::Odometry)),
            this, SLOT(updateOdomDisplay(nav_msgs::Odometry)));
    //navCarManagerNodeThread.start();

    // 初始化仪表盘范围 (假设最大速度2.5m/s -> 250, 最大角速度 1.0rad/s -> 100)
    ui->linearVelocityDial->setRange(-250, 250);
    ui->linearVelocityDial->setNotchesVisible(true);
    ui->angularVelocityDial->setRange(-100, 100);
    ui->angularVelocityDial->setNotchesVisible(true);

    // 初始化地图和点管理
    mapWidget = new MapDisplayWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(ui->graphicsView);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(mapWidget);
    pointManager = new PointManager(this);
    // 初始化目标点Publisher
    goalPub = nh.advertise<geometry_msgs::PoseStamped>("/waypoint_route_goal", 1);

    // 连接ROS话题
    mapSub = nh.subscribe("/map", 1, &MainWindow::mapCallback, this);
    poseSub = nh.subscribe("/amcl_pose", 1, &MainWindow::poseCallback, this);
    pathSub = nh.subscribe("/move_base/NavfnROS/plan", 1, &MainWindow::pathCallback, this);

    // 轨迹相关订阅
    globalPathSub = nh.subscribe("/move_base_node/SBPLLatticePlanner/plan", 1, &MainWindow::globalPathCallback, this);
    tebGlobalPathSub = nh.subscribe("/move_base_node/TebLocalPlannerROS/global_plan", 1, &MainWindow::tebGlobalPathCallback, this);
    tebLocalPathSub = nh.subscribe("/move_base_node/TebLocalPlannerROS/local_plan", 1, &MainWindow::tebLocalPathCallback, this);
    laserOdomSub = nh.subscribe("/laser_odometry", 100, &MainWindow::laserOdomCallback, this);
    originalLaserOdomSub = nh.subscribe("/original_laser_odometry", 100, &MainWindow::originalLaserOdomCallback, this);
    // moveBaseFeedbackSub = nh.subscribe("/move_base/feedback", 10, &MainWindow::moveBaseFeedbackCallback, this);
    // moveBaseResultSub = nh.subscribe("/move_base/result", 10, &MainWindow::moveBaseResultCallback, this);
    // moveBaseStatusSub = nh.subscribe("/move_base/status", 10, &MainWindow::moveBaseStatusCallback, this);

    ///地图连接信号
    connect(ui->loadMapButton, &QPushButton::clicked,
        this, &MainWindow::on_loadMapButton_clicked);
    connect(ui->saveMapButton, &QPushButton::clicked,
        this, &MainWindow::on_saveMapButton_clicked);
    connect(ui->clearMapButton, &QPushButton::clicked,
        this, &MainWindow::on_clearMapButton_clicked);

    // 连接点管理信号
    connect(pointManager, &PointManager::pointsChanged, 
        this, &MainWindow::updatePointList);
    connect(pointManager, &PointManager::pointAdded,
        this, &MainWindow::onPointAdded);
    connect(pointManager, &PointManager::pointRemoved,
        this, &MainWindow::onPointRemoved);

    // 连接地图点击信号
    connect(mapWidget, &MapDisplayWidget::pointClicked, 
        this, &MainWindow::onMapPointClicked);

    // 点列表右键菜单
    ui->pointListWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->pointListWidget, &QWidget::customContextMenuRequested,
            this, &MainWindow::showPointListContextMenu);

    connect(ui->startNavigationButton, &QPushButton::clicked,
            this, &MainWindow::on_startNavigationButton_clicked);
}

MainWindow::~MainWindow()
{
    navCarManagerNodeThread.quit();
    navCarManagerNodeThread.wait();
    delete ui;
}

// ROS回调实现
void MainWindow::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    MapDisplayWidget* mapWidget = qobject_cast<MapDisplayWidget*>(ui->graphicsView);
    if(mapWidget) {
        mapWidget->loadMap(*msg);
    }
}

void MainWindow::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    MapDisplayWidget* mapWidget = qobject_cast<MapDisplayWidget*>(ui->graphicsView);
    if(mapWidget) {
        mapWidget->updateRobotPosition(*msg);
    }
}

void MainWindow::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    MapDisplayWidget* mapWidget = qobject_cast<MapDisplayWidget*>(ui->graphicsView);
    if(mapWidget) {
        mapWidget->updatePath(*msg);
        
        // 更新状态栏显示
        QString message = QString("收到路径规划结果，共 %1 个路径点").arg(msg->poses.size());
        ui->statusbar->showMessage(message, 3000);
        
        qDebug() << "路径回调: 收到" << msg->poses.size() << "个路径点";
    }
}

void MainWindow::on_refreshLin1Button_clicked()
{
    emit RefreshAvailablePortsSignal();
}


void MainWindow::on_connectLin1Button_clicked()
{
    QString portName=ui->portInput->currentText();
    qint32 baudRate=ui->baudrateInput->text().toInt();
    qDebug()<<"连接信息"<<portName<<baudRate;
    emit ConnectToLIN1PortSignal(portName,baudRate);

}


void MainWindow::on_disconnectLin1Button_clicked()
{
    emit DisConnectToLIN1Signal();
}


void MainWindow::on_sendMsgLin1Button_clicked()
{
    bool ok;
    quint8 frameId = ui->LIN1IDInput->text().toUInt(&ok, 16);
    if (!ok) {
        QMessageBox::warning(this, "错误", "无效的帧ID（应为十六进制）");
        return;
    }

    QByteArray data = QByteArray::fromHex(ui->LIN1DataINput->text().toUtf8());
    if (data.size() > 8) {
        QMessageBox::warning(this, "错误", "数据长度不能超过8字节");
        return;
    }
    qDebug()<<frameId<<data;
    qDebug()<<data.toHex();
    qDebug()<<frameId<<data;
    emit SendLIN1MsgSignal(frameId,data);
}

void MainWindow::logDataUpdate(QString newLog)
{
    qDebug()<<"日志新增";
    ui->LIN1Log->append(newLog);

}

void MainWindow::refreshPortData(QStringList portData)
{
    ui->portInput->clear();
    ui->portInput->addItems(portData);
}


void MainWindow::on_paramDatapushButton_clicked()
{
    navCarManagerNode.initLinDevice();
    //navCarManagerNode.setLINSensorParam();
    /*
    //id配置初始化
    QString str;
    QByteArray data;
    str="7f06b5ff7f010200";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x3c,data);
    QThread::msleep(5);
    str="7f06b5ff7f010201";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x3c,data);
    QThread::msleep(5);
    str="7f06b5ff7f010202";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x3c,data);
    QThread::msleep(5);
    str="7f06b5ff7f010203";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x3c,data);
    QThread::msleep(5);
    str="7f06b5ff7f010204";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x3c,data);
    QThread::msleep(5);
    str="7f06b5ff7f010205";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x3c,data);
    QThread::msleep(5);
    str="7f06b5ff7f010206";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x3c,data);
    QThread::msleep(5);
    str="7f06b5ff7f030200";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x3c,data);
    QThread::msleep(5);
    str="7f06b5ff7f040200";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x3c,data);
    QThread::msleep(5);
*/
}


void MainWindow::on_readTestpushButton_clicked()
{
    navCarManagerNode.startMesuareLIN1();
    //navCarManagerNode.setLINSensorEnable();
    /*QString str="aaaac0";
    QByteArray data;
    data=QByteArray::fromHex(str.toUtf8());
    emit navCarManagerNode.SendLIN1MsgSignal(0x00,data);
    QThread::msleep(50);
    emit navCarManagerNode.RequestFrameSignal(0x02,8);*/

//f6f800e0
    /*
    QString str;
    QByteArray data;
    str="0100";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x38,data);
    QThread::msleep(5);
    str="0200";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x38,data);
    QThread::msleep(5);
    str="0300";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x38,data);
    QThread::msleep(5);
    str="0400";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x38,data);
    QThread::msleep(5);
    str="0500";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x38,data);
    QThread::msleep(5);
    str="0600";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x38,data);
    QThread::msleep(5);

    str="f1f800e0";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x1a,data);
    QThread::msleep(5);
    str="f2f800e0";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x1a,data);
    QThread::msleep(5);
    str="f3f800e0";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x1a,data);
    QThread::msleep(5);
    str="f4f800e0";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x1a,data);
    QThread::msleep(5);
    str="f5f800e0";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x1a,data);
    QThread::msleep(5);
    str="f6f800e0";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x1a,data);
    QThread::msleep(5);

    str="6506c0";
    data=QByteArray::fromHex(str.toUtf8());
    emit SendLIN1MsgSignal(0x00,data);
    QThread::msleep(5);

    emit navCarManagerNode.RequestFrameSignal(0x01,8);*/

}


void MainWindow::on_refreshCANButton_clicked()
{
    emit RefreshAvailableCANPortsSignal();
}

void MainWindow::refreshCANPortData(QStringList portData)
{
    ui->portCANInput->clear();
    ui->portCANInput->addItems(portData);

}

void MainWindow::on_connectCANButton_clicked()
{
    QString portName=ui->portCANInput->currentText();
    emit navCarManagerNode.OpenCANInterfaceSignal(portName);
}


void MainWindow::on_disconnectCANButton_clicked()
{
    emit navCarManagerNode.CloseCANInterfaceSignal();
}


void MainWindow::on_sendMsgCANButton_clicked()
{
    bool ok;
    quint32 id = ui->CANIDInput->text().toUInt(&ok, 16);
    if (!ok) {
        QMessageBox::warning(this, "错误", "无效的帧ID（应为十六进制）");
        return;
    }

    QByteArray data = QByteArray::fromHex(ui->CANDataINput->text().toUtf8());
    if (data.size() > 8) {
        QMessageBox::warning(this, "错误", "数据长度不能超过8字节");
        return;
    }
    emit navCarManagerNode.SendCANMsgSignal(id,data,sizeof(data));
}


void MainWindow::on_frontBtn_clicked()
{
    navCarManagerNode.velDataHandle(1,0);
}


void MainWindow::on_rearBtn_clicked()
{
    navCarManagerNode.velDataHandle(-1,0);
}


void MainWindow::on_leftBtn_clicked()
{
    navCarManagerNode.velDataHandle(1,0.2);
}


void MainWindow::on_rightBtn_clicked()
{
    navCarManagerNode.velDataHandle(1,-0.2);
}


void MainWindow::on_stopBtn_clicked()
{
    navCarManagerNode.velDataHandle(0,0);
}


void MainWindow::on_showLatestSensorDataBtn_clicked()
{
    QVector<float> distances=navCarManagerNode.getLatestUltraData();
    ui->sensorDataLabel->setText(QString::number(distances[0]));
    ui->sensorDataLabel_2->setText(QString::number(distances[1]));
    ui->sensorDataLabel_3->setText(QString::number(distances[2]));
    ui->sensorDataLabel_4->setText(QString::number(distances[3]));
    ui->sensorDataLabel_5->setText(QString::number(distances[4]));
    ui->sensorDataLabel_6->setText(QString::number(distances[5]));
}


void MainWindow::on_startListenBtn_clicked()
{
    navCarManagerNode.startDataBroadcastServer(ui->networkPortInput->text().toUInt());
}


void MainWindow::on_stopListenBtn_clicked()
{
    navCarManagerNode.stopDataBroadcastServer();
}

//更新电机数据
void MainWindow::updateMotorDisplay(int motorGroup, int motorNumber, bool enabled, 
                                   float rpm, float current1, float current2)
{
    // 计算树形控件的索引
    int treeIndex = (motorGroup - 1) * 2 + (motorNumber - 1);
    
    if (treeIndex >= 0 && treeIndex < ui->motorTree_2->topLevelItemCount()) {
        QTreeWidgetItem* item = ui->motorTree_2->topLevelItem(treeIndex);
        if (item) {
            // 更新使能状态
            item->setText(1, enabled ? "使能" : "失能");
            
            // 更新转速
            item->setText(2, QString("%1 rpm").arg(rpm, 0, 'f', 1));
            // 更新电流
            if (motorNumber == 1){
                item->setText(3, QString("%1 A").arg(current1, 0, 'f', 1));
            } else if (motorNumber == 2){
                item->setText(3, QString("%1 A").arg(current2, 0, 'f', 1));
            };
            
        }
    }
}

//更新油压数据
void MainWindow::updateOilPressureDisplay(float pressure1, float pressure2, float pressure3, float pressure4)
{
    // 更新油压传感器的树形显示 (pumpTree_4)
    if (ui->pumpTree_4->topLevelItemCount() >= 5) {
        QTreeWidgetItem* item = ui->pumpTree_4->topLevelItem(1); // 油压1
        if (item) item->setText(1, QString("%1 bar").arg(pressure1, 0, 'f', 2));
        
        item = ui->pumpTree_4->topLevelItem(2); // 油压2
        if (item) item->setText(1, QString("%1 bar").arg(pressure2, 0, 'f', 2));
        
        item = ui->pumpTree_4->topLevelItem(3); // 油压3
        if (item) item->setText(1, QString("%1 bar").arg(pressure3, 0, 'f', 2));
        
        item = ui->pumpTree_4->topLevelItem(4); // 油压4
        if (item) item->setText(1, QString("%1 bar").arg(pressure4, 0, 'f', 2));
    }
}

//油温数据
void MainWindow::updateOilTemperatureDisplay(float temperature)
{
    // 更新油温显示 (pumpTree_4的第一个item)
    if (ui->pumpTree_4 && ui->pumpTree_4->topLevelItemCount() > 0) {
        QTreeWidgetItem* item = ui->pumpTree_4->topLevelItem(0);
        if (item) {
            item->setText(1, QString("%1°C").arg(temperature, 0, 'f', 2));
        }
    }
}

void MainWindow::updateModeDisplay(int mode)
{
    // 更新模式显示 (currentmode控件)
    if (!ui->currentmode) return;
    
    QString modeText;
    switch (mode) {
    case 1: modeText = "复位"; break;
    case 2: modeText = "手调"; break;
    case 3: modeText = "自调"; break;
    case 4: modeText = "原地转向"; break;
    case 5: modeText = "斜行"; break;
    case 6: modeText = "直行"; break;
    default: modeText = "未知"; break;
    }
    
    ui->currentmode->setText(modeText);
    // 同时更新模式选择框
    if (mode >= 1 && mode <= 6) {
        ui->modeComboBox_2->setCurrentIndex(mode - 1);
    }
}

void MainWindow::updateHydraulicLockDisplay(bool lock1, bool lock2, bool lock3, bool lock4)
{
    // 更新液压锁显示 (legTree_2)
    if (!ui->legTree_2) return;
    
    QList<bool> locks = {lock1, lock2, lock3, lock4};
    for (int i = 0; i < 4 && i < ui->legTree_2->topLevelItemCount(); i++) {
        QTreeWidgetItem* item = ui->legTree_2->topLevelItem(i);
        if (item) {
            item->setText(1, locks[i] ? "打开" : "关闭");
        }
    }
}

void MainWindow::updateLightDisplay(bool light1, bool light2, bool light3, bool light4)
{
    // 更新灯光显示 (checkbox控件)
    if (ui->light1Check_2) ui->light1Check_2->setChecked(light1);  // 左转前灯1
    if (ui->light2Check_2) ui->light2Check_2->setChecked(light2);  // 左转后灯2
    if (ui->light3Check_2) ui->light3Check_2->setChecked(light3);  // 右转前灯3
    if (ui->light4Check_2) ui->light4Check_2->setChecked(light4);  // 右转后灯4
}

void MainWindow::updateEmergencyDisplay(bool remote, bool passive, bool safety, bool blockage)
{
    // 更新急停显示
    if (ui->remoteEmergencyCheck_2) ui->remoteEmergencyCheck_2->setChecked(remote);      // 遥控器
    if (ui->passiveEmergencyCheck_2) ui->passiveEmergencyCheck_2->setChecked(passive);  // 被动
    if (ui->safetyEmergencyCheck_2) ui->safetyEmergencyCheck_2->setChecked(safety);     // 安全
    if (ui->blockageCheck_2) ui->blockageCheck_2->setChecked(blockage);                  // 堵塞信号
}

void MainWindow::updateEncoderDisplay(int encoderId, quint8 encoderValue, float angle)
{
    // 更新编码器显示 (encoderTree)
    if (!ui->encoderTree) return;
    
    // 确保QTreeWidget有足够的列来显示原始值和角度
    if (ui->encoderTree->columnCount() < 3) {
        ui->encoderTree->setColumnCount(3);
        ui->encoderTree->setHeaderLabels(QStringList() << "编码器ID" << "原始值" << "角度");
    }
    
    // 根据编码器ID找到对应的树形控件项（索引0对应编码器1，以此类推）
    if (encoderId >= 1 && encoderId <= ui->encoderTree->topLevelItemCount()) {
        QTreeWidgetItem* item = ui->encoderTree->topLevelItem(encoderId - 1);
        if (item) {
            // 更新原始值显示
            item->setText(1, QString::number(encoderValue));
            // 更新角度显示，格式化为1位小数
            item->setText(2, QString("%1°").arg(angle, 0, 'f', 1));
        }
    } else {
        // 如果项目不存在，创建新的项目
        QTreeWidgetItem* item = new QTreeWidgetItem(ui->encoderTree);
        item->setText(0, QString("编码器%1").arg(encoderId));
        item->setText(1, QString::number(encoderValue));
        item->setText(2, QString("%1°").arg(angle, 0, 'f', 1));
    }
}

void MainWindow::updateOilPumpDisplay(float motorVoltage, float motorCurrent, 
                                     float motorTemperature, float controllerTemperature)
{
    // 更新油泵电机信息显示 (groupBox_23相关控件)
    // 假设pumpTree是显示油泵电机信息的树形控件
    if (!ui->pumpTree) return;
    
    // 更新电机电压
    if (ui->pumpTree->topLevelItemCount() > 0) {
        QTreeWidgetItem* item = ui->pumpTree->topLevelItem(0); // 电机电压
        if (item) item->setText(1, QString("%1 V").arg(motorVoltage, 0, 'f', 1));
    }
    
    // 更新电机电流
    if (ui->pumpTree->topLevelItemCount() > 1) {
        QTreeWidgetItem* item = ui->pumpTree->topLevelItem(1); // 电机电流
        if (item) item->setText(1, QString("%1 A").arg(motorCurrent, 0, 'f', 2));
    }
    
    // 更新电机温度
    if (ui->pumpTree->topLevelItemCount() > 2) {
        QTreeWidgetItem* item = ui->pumpTree->topLevelItem(2); // 电机温度
        if (item) item->setText(1, QString("%1°C").arg(motorTemperature, 0, 'f', 1));
    }
    
    // 更新控制器温度
    if (ui->pumpTree->topLevelItemCount() > 3) {
        QTreeWidgetItem* item = ui->pumpTree->topLevelItem(3); // 控制器温度
        if (item) item->setText(1, QString("%1°C").arg(controllerTemperature, 0, 'f', 1));
    }
}

void MainWindow::updateBatteryStatusDisplay(int soc, int soe, int soh, int life)
{
    // 更新电池状态显示 (batteryGroup中的四个progressBar)
    if (ui->progressBar_1) ui->progressBar_1->setValue(soc);  // SOC进度条
    if (ui->progressBar_2) ui->progressBar_2->setValue(soe);  // SOE进度条
    if (ui->progressBar_3) ui->progressBar_3->setValue(soh);  // SOH进度条
    if (ui->progressBar_4) ui->progressBar_4->setValue(life); // LIFE进度条

    qDebug() << "更新电池状态显示: SOC=" << soc << "%, SOE=" << soe << "%, SOH=" << soh << "%, LIFE=" << life << "%";
}

void MainWindow::updateBumpMotorSpeedDisplay(float pumpmotorspeed)
{
    ///更新油泵电机转速显示
    // 如果pumpTree有5个以上的项目，可以在第5个位置显示转速
    if (!ui->pumpTree) return;
    
    // 如果树形控件有足够的项，更新第4项
    if (ui->pumpTree->topLevelItemCount() > 4) {
        QTreeWidgetItem* item = ui->pumpTree->topLevelItem(4);
        if (item) {
            item->setText(1, QString("%1 rpm").arg(pumpmotorspeed, 0, 'f', 0));
        }
    } else {
        // 如果没有足够的项，可以创建一个新项
        QTreeWidgetItem* item = new QTreeWidgetItem();
        item->setText(0, "油泵电机转速");
        item->setText(1, QString("%1 rpm").arg(pumpmotorspeed, 0, 'f', 0));
        ui->pumpTree->addTopLevelItem(item);
    }
}

///地图处理
void MainWindow::on_loadMapButton_clicked() {
    QString mapFile = QFileDialog::getOpenFileName(this, 
        tr("选择地图文件"), 
        "/home/zxh/navCarProject_ws/src/my_navcar_navigation/maps",
        tr("地图文件 (*.yaml *.yml)")
    );
    
    if(!mapFile.isEmpty()) {
        if(mapWidget && mapWidget->loadMapFromFile(mapFile)) {
            qDebug() << "地图加载成功:" << mapFile;
            updateMapStatus();  // 更新状态栏
            // 自动加载点文件
            QString pointsFile = mapFile;
            pointsFile.replace(QRegExp("\\.ya?ml$"), "_points.dat");
            pointManager->loadFromFile(pointsFile);
            updatePointList();
            mapWidget->drawPoints(pointManager->getAllPoints());
        } else {
            QMessageBox::warning(this, tr("错误"), tr("地图加载失败"));
        }
    }
}

void MainWindow::on_clearMapButton_clicked() {
    MapDisplayWidget* mapWidget = qobject_cast<MapDisplayWidget*>(ui->graphicsView);
    if (mapWidget) {
        mapWidget->clear();
        // 清除后刷新点显示（如果有点数据）
        mapWidget->drawPoints(pointManager->getAllPoints());
    }
}

void MainWindow::on_saveMapButton_clicked() {
    // 保存当前地图状态（如果需要）
    QFileDialog dialog(this);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setWindowTitle(tr("保存地图状态"));
    dialog.setDirectory(QDir::homePath());
    dialog.setNameFilter(tr("地图状态文件 (*.mapstate)"));
    dialog.setDefaultSuffix("mapstate");
    if (dialog.exec() == QDialog::Accepted) {
        QString saveFile = dialog.selectedFiles().first();
        if (!saveFile.endsWith(".mapstate", Qt::CaseInsensitive)) {
            saveFile += ".mapstate";
        }
        pointManager->saveToFile(saveFile);
    }
}

void MainWindow::updateMapStatus() {
    MapDisplayWidget* mapWidget = qobject_cast<MapDisplayWidget*>(ui->graphicsView);
    if (!mapWidget) {
        return;
    }
    QString status = QString("地图已加载 | 分辨率: %1 m/像素")
        .arg(mapWidget->getResolution(), 0, 'f', 3);
    
    ui->statusbar->showMessage(status);
}

// 点列表更新
void MainWindow::updatePointList() {
    ui->pointListWidget->clear();
    
    // 添加固定点
    for (const auto& point : pointManager->getFixedPoints()) {
        addPointToList(point);
    }
    
    // 添加自定义点
    for (const auto& point : pointManager->getCustomPoints()) {
        addPointToList(point);
    }
}

void MainWindow::onPointAdded(const NavigationPoint& point) {
    // 更新点列表
    updatePointList();
    
    // 处理选点模式
    if (m_selectionMode == ModeStartPoint) {
        startPointId = point.id;
        mapWidget->setStartPointId(startPointId);
        
        ui->startPointInput->setText(QString("%1, %2").arg(point.position.x(), 0, 'f', 2).arg(point.position.y(), 0, 'f', 2));
        m_selectionMode = ModeNone;
        mapWidget->unsetCursor(); // Restore cursor
        ui->statusbar->showMessage(QString("已设置起点: %1").arg(point.name), 3000);
    } else if (m_selectionMode == ModeTargetPoint) {
        goalPointId = point.id;
        mapWidget->setTargetPointId(goalPointId);

        ui->targetPointInput->setText(QString("%1, %2").arg(point.position.x(), 0, 'f', 2).arg(point.position.y(), 0, 'f', 2));
        m_selectionMode = ModeNone;
        mapWidget->unsetCursor(); // Restore cursor
        ui->statusbar->showMessage(QString("已设置终点: %1").arg(point.name), 3000);
    } else {
        // 普通添加显示的提示
        ui->statusbar->showMessage(
            QString("已添加%1: %2 (%3, %4)")
                .arg(point.isFixed ? "固定点" : "自定义点")
                .arg(point.name)
                .arg(point.position.x(), 0, 'f', 2)
                .arg(point.position.y(), 0, 'f', 2),
            3000
        );
    }
    
    // 在地图上绘制新点
    if (mapWidget) {
        mapWidget->drawPoints(pointManager->getAllPoints());
    }
    // 自动保存点文件
    QString mapFile = ui->startPointInput->text(); // 或保存当前地图路径变量
    if (!mapFile.isEmpty()) {
        QString pointsFile = mapFile;
        pointsFile.replace(QRegExp("\\.ya?ml$"), "_points.dat");
        pointManager->saveToFile(pointsFile);
    }

    updatePathPlanningStatus();
}

void MainWindow::onPointRemoved(int pointId) {
    // 检查是否删除了起点或终点
    if (pointId == startPointId) {
        startPointId = -1;
        ui->startPointInput->clear();
        if (mapWidget) mapWidget->setStartPointId(-1);
    }
    if (pointId == goalPointId) {
        goalPointId = -1;
        ui->targetPointInput->clear();
        if (mapWidget) mapWidget->setTargetPointId(-1);
    }
    updatePathPlanningStatus();

    // 更新点列表
    updatePointList();
    
    // 更新地图显示
    MapDisplayWidget* mapWidget = qobject_cast<MapDisplayWidget*>(ui->graphicsView);
    if (mapWidget) {
        mapWidget->drawPoints(pointManager->getAllPoints());
    }
    // 自动保存点文件
    QString mapFile = ui->startPointInput->text(); // 或保存当前地图路径变量
    if (!mapFile.isEmpty()) {
        QString pointsFile = mapFile;
        pointsFile.replace(QRegExp("\\.ya?ml$"), "_points.dat");
        pointManager->saveToFile(pointsFile);
    }
    
    // 显示状态信息
    ui->statusbar->showMessage(QString("已删除点ID: %1").arg(pointId), 3000);
}

void MainWindow::addPointToList(const NavigationPoint& point) {
    QString itemText = QString("%1 (%2, %3)")
        .arg(point.name)
        .arg(point.position.x(), 0, 'f', 2)
        .arg(point.position.y(), 0, 'f', 2);
    
    QListWidgetItem* item = new QListWidgetItem(itemText, ui->pointListWidget);
    item->setData(Qt::UserRole, point.id);
    item->setForeground(point.color);
}

// 添加固定点按钮
void MainWindow::on_addFixedPointButton_clicked() {
    m_selectionMode = ModeAddFixedPoint;
    ui->statusbar->showMessage("请在地图上点击添加固定点...");
    if (mapWidget) mapWidget->setCursor(Qt::CrossCursor);
}

// 添加自定义点按钮
void MainWindow::on_addCustomPointButton_clicked() {
    m_selectionMode = ModeAddCustomPoint;
    ui->statusbar->showMessage("请在地图上点击添加自定义点...");
    if (mapWidget) mapWidget->setCursor(Qt::CrossCursor);
}

// 删除选中点
void MainWindow::on_removePointButton_clicked() {
    QListWidgetItem* item = ui->pointListWidget->currentItem();
    if (item) {
        int pointId = item->data(Qt::UserRole).toInt();
        
        // 如果删除的是起点或终点，重置相应变量
        if (pointId == startPointId) startPointId = -1;
        if (pointId == goalPointId) goalPointId = -1;
        
        pointManager->removePoint(pointId);
        
        // 更新状态显示
        updatePathPlanningStatus();
    }
}

// 更新路径规划状态显示
void MainWindow::updatePathPlanningStatus() {
    QString status;
    NavigationPoint startPoint = pointManager->getPoint(startPointId);
    NavigationPoint goalPoint = pointManager->getPoint(goalPointId);
    
    if (startPointId != -1 && goalPointId != -1) {
        status = QString("路径规划就绪: %1 → %2")
            .arg(startPoint.name)
            .arg(goalPoint.name);
    } else if (startPointId != -1) {
        status = QString("已选择起点: %1").arg(startPoint.name);
    } else if (goalPointId != -1) {
        status = QString("已选择终点: %1").arg(goalPoint.name);
    } else {
        status = "请选择起点和终点进行路径规划";
    }
    
    ui->statusbar->showMessage(status, 10000);
}

QPointF MainWindow::getSelectedMapPosition() const {
    // 从地图控件获取当前鼠标位置
    MapDisplayWidget* mapWidget = qobject_cast<MapDisplayWidget*>(ui->graphicsView);
    if (!mapWidget) {
        return QPointF();
    }
    QPointF scenePos = mapWidget->getLastClickedPosition();
    return mapWidget->sceneToMap(scenePos);
}

void MainWindow::on_planPathButton_clicked() {
    // 检查是否已选择起点和终点
    if (startPointId == -1 || goalPointId == -1) {
        QMessageBox::warning(this, tr("错误"), tr("请先选择起点和终点"));
        return;
    }
    
    // 获取起点和终点坐标
    NavigationPoint startPoint = pointManager->getPoint(startPointId);
    NavigationPoint goalPoint = pointManager->getPoint(goalPointId);
    
    if (startPoint.name.isEmpty() || goalPoint.name.isEmpty()) {
        QMessageBox::warning(this, tr("错误"), tr("选择的点无效"));
        return;
    }
    
    // 发送路径规划请求到ROS导航栈
    QString message = QString("正在规划从 %1 到 %2 的路径...")
        .arg(startPoint.name)
        .arg(goalPoint.name);
    
    ui->statusbar->showMessage(message, 5000);
    
    // TODO: 实现ROS路径规划请求
    // 这里可以调用ROS的导航服务或发布目标点
    qDebug() << "路径规划请求:"
             << "起点:" << startPoint.name << startPoint.position
             << "终点:" << goalPoint.name << goalPoint.position;
}

void MainWindow::on_selectStartPointButton_clicked()
{
    m_selectionMode = ModeStartPoint;
    ui->statusbar->showMessage("请在地图上点击选择起点...");
    if (mapWidget) mapWidget->setCursor(Qt::CrossCursor);
}

void MainWindow::on_selectTargetPointButton_clicked()
{
    m_selectionMode = ModeTargetPoint;
    ui->statusbar->showMessage("请在地图上点击选择终点...");
    if (mapWidget) mapWidget->setCursor(Qt::CrossCursor);
}

void MainWindow::onMapPointClicked(const QPointF &pos)
{
    // 发布目标点到/waypoint_route_goal
    geometry_msgs::PoseStamped goalMsg;
    goalMsg.header.stamp = ros::Time::now();
    goalMsg.header.frame_id = "map";
    goalMsg.pose.position.x = pos.x();
    goalMsg.pose.position.y = pos.y();
    goalMsg.pose.position.z = 0.0;
    goalMsg.pose.orientation.x = 0.0;
    goalMsg.pose.orientation.y = 0.0;
    goalMsg.pose.orientation.z = 0.0;
    goalMsg.pose.orientation.w = 1.0;
    goalPub.publish(goalMsg);

    ui->statusbar->showMessage(QString("已发送目标点: (%1, %2)").arg(pos.x(), 0, 'f', 2).arg(pos.y(), 0, 'f', 2), 3000);
}

void MainWindow::showPointListContextMenu(const QPoint& pos) {
    QListWidgetItem* item = ui->pointListWidget->itemAt(pos);
    if (!item) return;

    int pointId = item->data(Qt::UserRole).toInt();
    NavigationPoint point = pointManager->getPoint(pointId); 

    QMenu menu(this);
    QAction* setStartAction = menu.addAction("设为起点");
    QAction* setTargetAction = menu.addAction("设为终点");
    QAction* selectedAction = menu.exec(ui->pointListWidget->mapToGlobal(pos));

    if (selectedAction == setStartAction) {
        startPointId = pointId;
        ui->startPointInput->setText(QString("%1, %2").arg(point.position.x(), 0, 'f', 2).arg(point.position.y(), 0, 'f', 2));
        ui->statusbar->showMessage(QString("已设置起点: %1").arg(point.name), 3000);
        
        mapWidget->setStartPointId(startPointId);
        mapWidget->drawPoints(pointManager->getAllPoints());

        updatePathPlanningStatus();
    } else if (selectedAction == setTargetAction) {
        goalPointId = pointId;
        ui->targetPointInput->setText(QString("%1, %2").arg(point.position.x(), 0, 'f', 2).arg(point.position.y(), 0, 'f', 2));
        ui->statusbar->showMessage(QString("已设置终点: %1").arg(point.name), 3000);
        
        mapWidget->setTargetPointId(goalPointId);
        mapWidget->drawPoints(pointManager->getAllPoints());
        
        updatePathPlanningStatus();
    }
}

void MainWindow::updateVelocityDisplay(double linear, double angular) {
    ui->linearVelocity->setText(QString::number(linear, 'f', 2));
    ui->angularVelocity->setText(QString::number(angular, 'f', 2));
    
    // Update dials (scaled by 100 for display resolution)
    ui->linearVelocityDial->setValue(static_cast<int>(linear * 100));
    ui->angularVelocityDial->setValue(static_cast<int>(angular * 100));
}

void MainWindow::updateOdomDisplay(const nav_msgs::Odometry& odom)
{
    // 更新位置信息
    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    ui->positionLabel->setText(QString("当前位置：(%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
    
    // 从四元数计算航向角
    double qx = odom.pose.pose.orientation.x;
    double qy = odom.pose.pose.orientation.y;
    double qz = odom.pose.pose.orientation.z;
    double qw = odom.pose.pose.orientation.w;
    
    // 使用atan2计算yaw角（绕z轴旋转）
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    // 转换为角度
    double yaw_deg = yaw * 180.0 / M_PI;
    
    ui->orientationLabel->setText(QString("朝向: %1°").arg(yaw_deg, 0, 'f', 1));
}

void MainWindow::on_startNavigationButton_clicked()
{
    ui->statusbar->showMessage("导航已启动，机器人开始前往目标点...", 3000);
    // 可扩展为发布action goal等
}

// 轨迹相关回调
void MainWindow::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    QList<QPointF> points;
    for (const auto& pose : msg->poses)
        points.append(QPointF(pose.pose.position.x, pose.pose.position.y));
    if (mapWidget) mapWidget->drawGlobalPath(points);
}
void MainWindow::tebGlobalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    QList<QPointF> points;
    for (const auto& pose : msg->poses)
        points.append(QPointF(pose.pose.position.x, pose.pose.position.y));
    if (mapWidget) mapWidget->drawTebGlobalPath(points);
}
void MainWindow::tebLocalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    QList<QPointF> points;
    for (const auto& pose : msg->poses)
        points.append(QPointF(pose.pose.position.x, pose.pose.position.y));
    if (mapWidget) mapWidget->drawTebLocalPath(points);
}
QList<QPointF> actualOdomPathPoints;
void MainWindow::laserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    actualOdomPathPoints.append(QPointF(msg->pose.pose.position.x, msg->pose.pose.position.y));
    if (actualOdomPathPoints.size() > 1000) actualOdomPathPoints.removeFirst();
    if (mapWidget) mapWidget->drawActualOdomPath(actualOdomPathPoints);
}
void MainWindow::originalLaserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 可选：如需显示原始里程计轨迹，可仿照laserOdomCallback实现
}
