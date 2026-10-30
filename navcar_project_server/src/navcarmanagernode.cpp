// navcarmanagernode.cpp - 车辆管理节点实现（完整修改版）
#include "navcar_project_server/navcarmanagernode.h"
#include "navcar_project_server/qt_version_compat.h"

NavCarManagerNode::NavCarManagerNode(QObject *parent)
    : QObject{parent}
    , m_dataBroadcastWorker(nullptr)  // 修改：使用Worker
{
    // ========== 原有CANLIN通讯线程及信号绑定 (完全保持不变) ==========
    CANLINWork.moveToThread(&CANLINWorkThread);

    // LIN操作 (完全保持不变)
    connect(this,SIGNAL(ConnectToLIN1PortSignal(QString,qint32)),&CANLINWork,
            SLOT(connectToLIN1Port(QString,qint32)));
    connect(this,SIGNAL(ConnectToLIN2PortSignal(QString,qint32)),&CANLINWork,
            SLOT(connectToLIN2Port(QString,qint32)));
    connect(this,SIGNAL(DisConnectToLIN1Signal()),&CANLINWork,
            SLOT(disconnectToLIN1Port()));
    connect(this,SIGNAL(DisConnectToLIN2Signal()),&CANLINWork,
            SLOT(disconnectToLIN1Port()));
    connect(this,SIGNAL(RefreshAvailablePortsSignal()),&CANLINWork,
            SLOT(refreshAvailablePorts()));
    connect(this,SIGNAL(RefreshAvailablePortsSignal()),&CANLINWork,
            SLOT(refreshAvailablePorts()));
    connect(this,SIGNAL(SendLIN1MsgSignal(quint8,QByteArray)),&CANLINWork,
            SLOT(sendLIN1Msg(quint8,QByteArray)));
    connect(this,SIGNAL(SendLIN2MsgSignal(quint8,QByteArray)),&CANLINWork,
            SLOT(sendLIN2Msg(quint8,QByteArray)));
    connect(this,SIGNAL(RequestFrameSignal(quint8,int)),&CANLINWork,
            SLOT(onRequestFrame(quint8,int)));

    connect(&CANLINWork,SIGNAL(ResponseAvailablePortsSignal(QStringList)),this,
            SLOT(responseAvailablePorts(QStringList)));
    connect(&CANLINWork,SIGNAL(LogDataSignal(QString)),this,
            SIGNAL(LogDataSignal(QString)));

    // 信号时间表管理类 (完全保持不变)
    connect(&lin1SchedulerManager,SIGNAL(SendLIN1MsgSignal(quint8,QByteArray)),&CANLINWork,
            SLOT(sendLIN1Msg(quint8,QByteArray)));
    connect(&lin1SchedulerManager,SIGNAL(SendRequestLIN1MsgSignal(quint8,int)),&CANLINWork,
            SLOT(onRequestFrame(quint8,int)));

    // CAN操作 (完全保持不变)
    connect(this,SIGNAL(RefreshAvailableCANPortsSignal()),&CANLINWork,
            SLOT(refreshAvailableCANPorts()));
    connect(this,SIGNAL(OpenCANInterfaceSignal(QString)),&CANLINWork,
            SLOT(openCANInterface(QString)));
    connect(this,SIGNAL(CloseCANInterfaceSignal()),&CANLINWork,
            SLOT(closeCANInterface()));
    connect(&CANLINWork,SIGNAL(ResponseAvailableCANPortsSignal(QStringList)),this,
            SLOT(responseAvailableCANPorts(QStringList)));
    connect(this,SIGNAL(SendCANMsgSignal(quint32,QByteArray,quint8)),&CANLINWork,
            SLOT(sendCANMsg(quint32,QByteArray,quint8)));
    connect(&CANLINWork,&CANLINHandle::MessageReceivedSignal,this,&NavCarManagerNode::handleCANMsg);

    connect(&CANLINWorkThread,&QThread::finished,&CANLINWork,&CANLINHandle::cleanup);

    CANLINWorkThread.start();

    // ========== ROS操作 (保持原有车辆控制逻辑) ==========
    myROSWorkManager.moveToThread(&myROSWorkThread);
    // ROSWorkManager 在构造时位于主线程，定时器必须等对象搬到工作线程后再启动。
    // 否则会触发 “QObject::startTimer: Timers cannot be started from another thread”。
    connect(&myROSWorkThread, &QThread::started,
            &myROSWorkManager, &ROSWorkManager::startROSWork);
    connect(&myROSWorkManager,SIGNAL(SetVelCMDSignal(double,double)),this,
            SLOT(velDataHandle(double,double)));
    connect(&myROSWorkManager,SIGNAL(SetApproachCMDSignal(double,double,double,double,double)),this,
            SLOT(velApproachCMDHandle(double,double,double,double,double)));
    connect(&myROSWorkManager,SIGNAL(SetOrientationCMDSignal(double,double)),this,
            SLOT(velOrientationCMDHandle(double,double)));
        // 转发odom信号
        connect(&myROSWorkManager, SIGNAL(OdomReceivedSignal(nav_msgs::Odometry)),
            this, SIGNAL(odomUpdate(nav_msgs::Odometry)));
    connect(&myROSWorkThread,&QThread::finished,&myROSWorkManager,&ROSWorkManager::stopROSWork);
    
    // 连接内部canModeUpdate信号到CAN模式切换处理
    connect(this, &NavCarManagerNode::canModeUpdate, 
            this, &NavCarManagerNode::handleCANModeChange);

    myROSWorkThread.start();

    // ====LOG消息连接部分=====
    myDataLogger.moveToThread(&myDataLoggerThread);
    myDataLogger.setAutoSaveInterval(3000);
    connect(&myDataLoggerThread,&QThread::started,&myDataLogger,&DataLogger::startLogging);
    connect(&myDataLoggerThread,&QThread::finished,&myDataLogger,&DataLogger::stopLogging);
    connect(this,SIGNAL(CANLogMsgSignal(CanMessage)),&myDataLogger,SLOT(onCanMessage(CanMessage)));

    myDataLoggerThread.start();

    //Lin传感器读取信号
    myLIN1Timer=new QTimer(this);
    myLIN2Timer=new QTimer(this);
    //定时循环LIN操作
    connect(myLIN1Timer,&QTimer::timeout,this,&NavCarManagerNode::mesuareLIN1Once);
    connect(myLIN2Timer,&QTimer::timeout,this,&NavCarManagerNode::mesuareLIN2Once);
    //LIN数据处理
    connect(&CANLINWork,&CANLINHandle::SendLIN1SensorData,this,&NavCarManagerNode::handleLIN1SensorData);
    connect(&CANLINWork,&CANLINHandle::SendLIN2SensorData,this,&NavCarManagerNode::handleLIN2SensorData);
    startSaveUltrasonicData();
}

NavCarManagerNode::~NavCarManagerNode()
{
    // ========== 修改：清理数据广播Worker ==========
    stopDataBroadcastServer();

    if (m_dataBroadcastWorker) {
        // 通知Worker清理
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "cleanup", Qt::QueuedConnection);
    }

    // 停止线程
    dataBroadcastWorkThread.quit();
    dataBroadcastWorkThread.wait(3000); // 等待最多3秒

    stopMesuareLIN1();
    stopMesuareLIN2();
    finishSaveUltrasonicData();

    CANLINWorkThread.quit();
    myROSWorkThread.quit();
    myDataLoggerThread.quit();
    CANLINWorkThread.wait();
    myROSWorkThread.wait();
    myDataLoggerThread.wait();

}

// ========== 修改：数据广播初始化 ==========

bool NavCarManagerNode::initializeDataBroadcast()
{
    if (m_dataBroadcastInitialized) {
        return true;
    }

    try {
        qDebug() << "初始化数据广播Worker线程...";

        // 创建Worker
        m_dataBroadcastWorker = new DataBroadcastWorker();

        // 移动Worker到工作线程
        m_dataBroadcastWorker->moveToThread(&dataBroadcastWorkThread);

        // 连接线程生命周期信号
        connect(&dataBroadcastWorkThread, &QThread::started,
                this, &NavCarManagerNode::onBroadcastThreadStarted);
        connect(&dataBroadcastWorkThread, &QThread::finished,
                m_dataBroadcastWorker, &DataBroadcastWorker::deleteLater);

        // 连接Worker信号
        connectDataBroadcastSignals();

        // 启动工作线程
        dataBroadcastWorkThread.start();

        qDebug() << "数据广播Worker线程启动成功";
        return true;

    } catch (const std::exception& e) {
        qCritical() << "数据广播初始化异常:" << e.what();
        return false;
    }
}

void NavCarManagerNode::connectROSSignalsToWorker()
{
    if (!m_dataBroadcastWorker || !m_dataBroadcastInitialized) {
        qWarning() << "无法连接ROS信号：Worker未就绪";
        return;
    }

    qDebug() << "连接ROS信号到Worker...";
    // ========== 直接连接ROS信号到Worker槽函数==========
    connect(&myROSWorkManager, SIGNAL(NewPointCloudRecievd(QSharedPointer<sensor_msgs::PointCloud2>)),
            m_dataBroadcastWorker, SLOT(broadcastPointCloud(QSharedPointer<sensor_msgs::PointCloud2>)));
    connect(&myROSWorkManager, SIGNAL(NewCamFrontRecievd(QSharedPointer<sensor_msgs::Image>)),
            m_dataBroadcastWorker, SLOT(broadcastFrontCameraImage(QSharedPointer<sensor_msgs::Image>)));
    connect(&myROSWorkManager, SIGNAL(NewCamLeftRecievd(QSharedPointer<sensor_msgs::Image>)),
            m_dataBroadcastWorker, SLOT(broadcastLeftCameraImage(QSharedPointer<sensor_msgs::Image>)));
    connect(&myROSWorkManager, SIGNAL(NewCamRightRecievd(QSharedPointer<sensor_msgs::Image>)),
            m_dataBroadcastWorker, SLOT(broadcastRightCameraImage(QSharedPointer<sensor_msgs::Image>)));
    connect(&myROSWorkManager, SIGNAL(NewCamRearRecievd(QSharedPointer<sensor_msgs::Image>)),
            m_dataBroadcastWorker, SLOT(broadcastRearCameraImage(QSharedPointer<sensor_msgs::Image>)));
    connect(&myROSWorkManager,SIGNAL(NewGPSReciev(NetDataGPSData)),m_dataBroadcastWorker,SLOT(broadcastGPSData(NetDataGPSData)));
    qDebug() << "ROS信号到Worker连接完成";
}

void NavCarManagerNode::connectDataBroadcastSignals()
{
    if (!m_dataBroadcastWorker) return;

    qDebug() << "连接数据广播Worker信号...";

    // ========== Worker初始化信号 ==========
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::initializeResult,
            this, &NavCarManagerNode::onWorkerInitializeResult);
/*
    // ========== 直接连接ROS信号到Worker槽函数==========
    connect(&myROSWorkManager, SIGNAL(NewPointCloudRecievd(QSharedPointer<sensor_msgs::PointCloud2>)),
            m_dataBroadcastWorker, SLOT(broadcastPointCloud(QSharedPointer<sensor_msgs::PointCloud2>)));
    connect(&myROSWorkManager, SIGNAL(NewCamFrontRecievd(QSharedPointer<sensor_msgs::Image>)),
            m_dataBroadcastWorker, SLOT(broadcastFrontCameraImage(QSharedPointer<sensor_msgs::Image>)));
    connect(&myROSWorkManager, SIGNAL(NewCamLeftRecievd(QSharedPointer<sensor_msgs::Image>)),
            m_dataBroadcastWorker, SLOT(broadcastLeftCameraImage(QSharedPointer<sensor_msgs::Image>)));
    connect(&myROSWorkManager, SIGNAL(NewCamRightRecievd(QSharedPointer<sensor_msgs::Image>)),
            m_dataBroadcastWorker, SLOT(broadcastRightCameraImage(QSharedPointer<sensor_msgs::Image>)));
    connect(&myROSWorkManager, SIGNAL(NewCamRearRecievd(QSharedPointer<sensor_msgs::Image>)),
            m_dataBroadcastWorker, SLOT(broadcastRearCameraImage(QSharedPointer<sensor_msgs::Image>)));
    connect(&myROSWorkManager,SIGNAL(NewGPSReciev(NetDataGPSData)),m_dataBroadcastWorker,SLOT(broadcastGPSData(NetDataGPSData)));
*/
    // ========== Worker事件信号连接 ==========
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::serverStarted,
            this, &NavCarManagerNode::onDataBroadcastServerStarted);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::serverStopped,
            this, &NavCarManagerNode::onDataBroadcastServerStopped);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::serverError,
            this, &NavCarManagerNode::onDataBroadcastServerError);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::clientConnected,
            this, &NavCarManagerNode::onBroadcastClientConnected);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::clientDisconnected,
            this, &NavCarManagerNode::onBroadcastClientDisconnected);

    // ========== 直接转发查询结果信号 ==========
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::serverStatusResult,
            this, &NavCarManagerNode::serverStatusReceived);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::clientCountResult,
            this, &NavCarManagerNode::connectedClientCountReceived);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::clientInfoResult,
            this, &NavCarManagerNode::connectedClientInfoReceived);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::clientIdsResult,
            this, &NavCarManagerNode::connectedClientIdsReceived);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::configResult,
            this, &NavCarManagerNode::dataBroadcastConfigReceived);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::statisticsResult,
            this, &NavCarManagerNode::dataBroadcastStatisticsReceived);
    connect(m_dataBroadcastWorker, &DataBroadcastWorker::networkStatisticsResult,
            this, &NavCarManagerNode::networkStatisticsReceived);

    qDebug() << "数据广播Worker信号连接完成";
}

// ========== Worker线程事件处理 ==========

void NavCarManagerNode::onBroadcastThreadStarted()
{
    qDebug() << "数据广播工作线程已启动，开始初始化Worker...";

    // 设置默认配置
    BroadcastConfig config;
    config.enablePointCloudBroadcast = true;
    config.pointCloudCompressionLevel = MEDIUM_QUALITY;
    config.imageCodec = CODEC_H264;
    config.imageCompressionLevel = MEDIUM_QUALITY;
    config.enableHeartbeat = true;
    config.enableStatusBroadcast = true;
    config.imageMaxSize = QSize(1920, 1080);

    // 在工作线程中初始化Worker
    QMetaObject::invokeMethod(m_dataBroadcastWorker, "initialize",
                              Qt::QueuedConnection,
                              Q_ARG(BroadcastConfig, config));
}

void NavCarManagerNode::onWorkerInitializeResult(bool success)
{
    if (success) {
        m_dataBroadcastInitialized = true;
        qDebug() << "数据广播Worker初始化成功";

        connectROSSignalsToWorker();

        // 如果有待启动的服务器请求，现在启动
        if (m_pendingServerPort > 0) {
            qDebug() << "启动延迟的数据广播服务器，端口:" << m_pendingServerPort;
            QMetaObject::invokeMethod(m_dataBroadcastWorker, "startServer",
                                      Qt::QueuedConnection,
                                      Q_ARG(quint16, m_pendingServerPort));
            m_pendingServerPort = 0;
        }
    } else {
        qCritical() << "数据广播Worker初始化失败";
        m_pendingServerPort = 0; // 清除待启动的端口
    }
    /*
    if (success) {
        m_dataBroadcastInitialized = true;
        qDebug() << "数据广播Worker初始化成功";
    } else {
        qCritical() << "数据广播Worker初始化失败";
    }*/
}

// ========== 修改：数据广播管理接口（改为纯异步模式）==========

bool NavCarManagerNode::startDataBroadcastServer(quint16 port)
{
    // 延迟初始化数据广播系统
    if (!m_dataBroadcastInitialized) {
        if (!initializeDataBroadcast()) {
            return false;
        }
        // 保存端口，等待初始化完成后启动
        m_pendingServerPort = port;
        qDebug() << "数据广播系统正在初始化，服务器将在初始化完成后启动，端口:" << port;
        return true;
    }

    if (!m_dataBroadcastWorker) {
        qWarning() << "数据广播Worker未初始化";
        return false;
    }

    qDebug() << "启动数据广播服务器，端口:" << port;
    QMetaObject::invokeMethod(m_dataBroadcastWorker, "startServer",
                              Qt::QueuedConnection,
                              Q_ARG(quint16, port));
    return true;
    /*// 延迟初始化数据广播系统
    if (!m_dataBroadcastInitialized) {
        if (!initializeDataBroadcast()) {
            return false;
        }
    }

    if (!m_dataBroadcastWorker) {
        qWarning() << "数据广播Worker未初始化";
        return false;
    }

    qDebug() << "启动数据广播服务器，端口:" << port;
    QMetaObject::invokeMethod(m_dataBroadcastWorker, "startServer",
                              Qt::QueuedConnection,
                              Q_ARG(quint16, port));
    return true; // 异步操作，先返回true*/
}

void NavCarManagerNode::stopDataBroadcastServer()
{
    if (m_dataBroadcastWorker) {
        qDebug() << "停止数据广播服务器";
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "stopServer",
                                  Qt::QueuedConnection);
    }
}

// ========== 异步查询接口（结果通过信号返回）==========

void NavCarManagerNode::requestServerStatus()
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "requestServerStatus",
                                  Qt::QueuedConnection);
    }
}

void NavCarManagerNode::requestConnectedClientCount()
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "requestClientCount",
                                  Qt::QueuedConnection);
    }
}

void NavCarManagerNode::requestConnectedClientInfo()
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "requestClientInfo",
                                  Qt::QueuedConnection);
    }
}

void NavCarManagerNode::requestConnectedClientIds()
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "requestClientIds",
                                  Qt::QueuedConnection);
    }
}

void NavCarManagerNode::requestDataBroadcastConfig()
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "requestConfig",
                                  Qt::QueuedConnection);
    }
}

void NavCarManagerNode::requestDataBroadcastStatistics()
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "requestStatistics",
                                  Qt::QueuedConnection);
    }
}

void NavCarManagerNode::requestNetworkStatistics()
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "requestNetworkStatistics",
                                  Qt::QueuedConnection);
    }
}

void NavCarManagerNode::kickClient(quint32 clientId, const QString& reason)
{
    if (!m_dataBroadcastWorker) return;

    qDebug() << "踢出客户端:" << clientId << "原因:" << reason;
    QMetaObject::invokeMethod(m_dataBroadcastWorker, "kickClient",
                              Qt::QueuedConnection,
                              Q_ARG(quint32, clientId),
                              Q_ARG(QString, reason));
}

void NavCarManagerNode::kickAllClients(const QString& reason)
{
    if (!m_dataBroadcastWorker) return;

    qDebug() << "踢出所有客户端，原因:" << reason;
    QMetaObject::invokeMethod(m_dataBroadcastWorker, "kickAllClients",
                              Qt::QueuedConnection,
                              Q_ARG(QString, reason));
}

void NavCarManagerNode::setClientStreamConfig(quint32 clientId, StreamType streamType, bool enabled)
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "setClientStreamConfig",
                                  Qt::QueuedConnection,
                                  Q_ARG(quint32, clientId),
                                  Q_ARG(StreamType, streamType),
                                  Q_ARG(bool, enabled));
    }
}

void NavCarManagerNode::setClientCameraSubscription(quint32 clientId, int cameraId, bool enabled)
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "setClientCameraSubscription",
                                  Qt::QueuedConnection,
                                  Q_ARG(quint32, clientId),
                                  Q_ARG(int, cameraId),
                                  Q_ARG(bool, enabled));
    }
}

void NavCarManagerNode::setClientCompressionLevel(quint32 clientId, CompressionLevel level)
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "setClientCompressionLevel",
                                  Qt::QueuedConnection,
                                  Q_ARG(quint32, clientId),
                                  Q_ARG(CompressionLevel, level));
    }
}

void NavCarManagerNode::setDataBroadcastConfig(const BroadcastConfig& config)
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "updateConfig",
                                  Qt::QueuedConnection,
                                  Q_ARG(BroadcastConfig, config));
    }
}

void NavCarManagerNode::pauseDataBroadcast(bool pause)
{
    if (m_dataBroadcastWorker) {
        QMetaObject::invokeMethod(m_dataBroadcastWorker, "pauseBroadcast",
                                  Qt::QueuedConnection,
                                  Q_ARG(bool, pause));
    }
}

// ========== 数据广播事件处理 (保持不变) ==========

void NavCarManagerNode::onDataBroadcastServerStarted(quint16 port)
{
    qDebug() << "数据广播服务器已启动，端口:" << port;
    emit dataBroadcastServerStarted(port);
}

void NavCarManagerNode::onDataBroadcastServerStopped()
{
    qDebug() << "数据广播服务器已停止";
    emit dataBroadcastServerStopped();
}

void NavCarManagerNode::onDataBroadcastServerError(const QString& error)
{
    qWarning() << "数据广播服务器错误:" << error;
    emit dataBroadcastServerError(error);
}

void NavCarManagerNode::onBroadcastClientConnected(quint32 clientId, const QString& info)
{
    qDebug() << "广播客户端连接:" << clientId << info;
    emit broadcastClientConnected(clientId, info);
}

void NavCarManagerNode::onBroadcastClientDisconnected(quint32 clientId, const QString& reason)
{
    qDebug() << "广播客户端断开:" << clientId << reason;
    emit broadcastClientDisconnected(clientId, reason);
}

// ========== 以下代码保持完全不变 ==========
// 这些是原有的车辆控制功能，不做任何修改

void NavCarManagerNode::testClick()
{
    // 原有实现保持不变
}

//===新增：LIN操作======
//LIN操作（暴露接口测试）
void NavCarManagerNode::initLinDevice()
{
    initLIN1SensorParams();
    initLIN2SensorParams();
}

void NavCarManagerNode::startMesuareLIN1()
{
    if(!myLIN1Timer->isActive())
    {
        myLIN1Timer->start(LIN_TIMER_MS);
    }
}

void NavCarManagerNode::startMesuareLIN2()
{
    if(!myLIN2Timer->isActive())
    {
        myLIN2Timer->start(LIN_TIMER_MS);
    }
}

void NavCarManagerNode::stopMesuareLIN1()
{
    if(myLIN1Timer->isActive())
    {
        myLIN1Timer->stop();
    }
}

void NavCarManagerNode::stopMesuareLIN2()
{
    if(myLIN1Timer->isActive())
    {
        myLIN1Timer->stop();
    }
}

void NavCarManagerNode::initLIN1SensorParams()
{
    QString str;
    QByteArray data;
    str="ff00";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(10,0x38,data,0,2);
    str="fff800e0";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(10,0x38,data,0,4);
}

void NavCarManagerNode::initLIN2SensorParams()
{
    //预留LIN2空间
}

void NavCarManagerNode::mesuareLIN1Once()
{
    QString str;
    QByteArray data;
    str="aaaac0";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(50,0x00,data,0,3);
    lin1SchedulerManager.scheduleSignal(15,0x01,data,1,8);
    lin1SchedulerManager.scheduleSignal(15,0x02,data,1,8);
    lin1SchedulerManager.scheduleSignal(15,0x03,data,1,8);
    lin1SchedulerManager.scheduleSignal(15,0x04,data,1,8);
    lin1SchedulerManager.scheduleSignal(15,0x05,data,1,8);
    lin1SchedulerManager.scheduleSignal(15,0x06,data,1,8);
}

void NavCarManagerNode::mesuareLIN2Once()
{
    //预留LIN2空间
}

//===CAN LOG Msg====
void NavCarManagerNode::handleCANMsg(CanMessage rcvMsg)
{
    switch (rcvMsg.id) {
    case 0x4F110D0:
    case 0x4F210D0:
    case 0x4F310D0:
    case 0x4F410D0:
        parseMotorMessage(rcvMsg.id, rcvMsg.data, rcvMsg.dataLen);
        emit CANLogMsgSignal(rcvMsg);
        break;
    case 0x4F510D0:
        parseOilPressureMessage(rcvMsg.id, rcvMsg.data, rcvMsg.dataLen);
        emit CANLogMsgSignal(rcvMsg);
        break;
    case 0x4F610D0:
        parseHydraulicSystemMessage(rcvMsg.id, rcvMsg.data, rcvMsg.dataLen);
        emit CANLogMsgSignal(rcvMsg);
        break;
    case 0x4F710D0:
        parseOilPumpMessage(rcvMsg.id, rcvMsg.data, rcvMsg.dataLen);
        emit CANLogMsgSignal(rcvMsg);
        break;
    case 0x191:
    case 0x192:
    case 0x193:
    case 0x194:
        parseEncoderMessage(rcvMsg.id, rcvMsg.data, rcvMsg.dataLen);
        emit CANLogMsgSignal(rcvMsg);
        break;
    case 0x181B9CF3:
        qDebug() << "收到电池状态数据，ID:" << rcvMsg.id;
        parseBatteryStatusMessage(rcvMsg.id, rcvMsg.data, rcvMsg.dataLen);
        emit CANLogMsgSignal(rcvMsg);
        break;
    default:
        break;   
    }
}

//====超声波数据处理=====
void NavCarManagerNode::handleLIN1SensorData(EchoData newSensorData)
{
    ultraFrontManager.setData(newSensorData);
    qDebug()<<"主线程收取数据完成";
    if(ultraFrontManager.areAllUpdated())
    {
        qDebug()<<"收取全部数据完成";
        //TODO:加入安全模块
        ultraSensorDetectHandle_front();
    }
}

void NavCarManagerNode::handleLIN2SensorData(EchoData newSensorData)
{
    ultraRearManager.setData(newSensorData);
    qDebug()<<"主线程收取数据完成";
    if(ultraRearManager.areAllUpdated())
    {
        qDebug()<<"收取全部数据完成";
        //TODO:加入安全模块
        ultraSensorDetectHandle_rear();
    }
}

void NavCarManagerNode::ultraSensorDetectHandle_front()
{
    //TODO:safeControl
    ultraFrontManager.resetUpdateFlags();
    // 计算距离并存入容器
    QVector<float> distances;
    for (int id = 1; id <= 6; ++id) {
        auto data = ultraFrontManager.getData(id);
        float distance = data.first.flightTime1 * 340 * 0.064f / 2000.0f;
        distances.append(distance);
        qDebug() << "设备" << id << "距离：" << distance;
    }
    //saveUltrasonicData(distances);
}

QVector<float> NavCarManagerNode::getLatestUltraData()
{
    QVector<float> distances;
    for (int id = 1; id <= 6; ++id) {
        auto data = ultraFrontManager.getData(id);
        float distance = data.first.flightTime1 * 340 * 0.064f / 2000.0f;
        distances.append(distance);
        qDebug() << "设备" << id << "距离：" << distance;
    }
    return distances;
}

void NavCarManagerNode::ultraSensorDetectHandle_rear()
{
    //TODO:safeControl
    ultraRearManager.resetUpdateFlags();
}

//LIN
void NavCarManagerNode::responseAvailablePorts(QStringList availablePorts)
{
    emit RefreshPortDataSignal(availablePorts);
}

void NavCarManagerNode::setLINSensorParam()
{
    // 原有实现完全保持不变
    QString str;
    QByteArray data;
    str="7f06b5ff7e010200";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(5,0x3c,data,0,0);
    str="7f06b5ff7e010201";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(5,0x3c,data,0,0);
    str="7f06b5ff7e010202";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(5,0x3c,data,0,0);
    str="7f06b5ff7e010203";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(5,0x3c,data,0,0);
    str="7f06b5ff7e010204";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(5,0x3c,data,0,0);
    str="7f06b5ff7e010205";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(5,0x3c,data,0,0);
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(5,0x3c,data,0,0);
    str="7f06b5ff7e030200";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(5,0x3c,data,0,0);
    str="7f06b5ff7e040200";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(5,0x3c,data,0,0);
}

void NavCarManagerNode::setLINSensorEnable()
{
    // 原有实现完全保持不变
    QString str;
    QByteArray data;
    str="ff00";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(10,0x38,data,0,0);
    str="fff800e0";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(10,0x1a,data,0,0);
    str="aaaac0";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(10,0x00,data,0,0);
    QThread::msleep(500);
    emit RequestFrameSignal(0x01,8);
}

void NavCarManagerNode::measureLINSensor()
{
    // 原有实现完全保持不变
    QString str;
    QByteArray data;
    str="2000c0";
    data=QByteArray::fromHex(str.toUtf8());
    lin1SchedulerManager.scheduleSignal(40,0x01,data,1,8);
}

void NavCarManagerNode::sendLIN1Msg(quint8 id,QByteArray data)
{
    // TODO:已经在下层实现
}

void NavCarManagerNode::sendLIN2Msg(quint8 id,QByteArray data)
{
    // TODO:已经在下层实现
}

//CAN
void NavCarManagerNode::responseAvailableCANPorts(QStringList availablePorts)
{
    emit RefreshCANPortDataSignal(availablePorts);
}

void NavCarManagerNode::sendCANMsg(quint32 id,QByteArray data,quint8 datalen)
{
    emit SendCANMsgSignal(id,data,datalen);
}

void NavCarManagerNode::sendCANMsg(CanMessage sendMsg)
{
    // TODO:已经在下层实现
}

//CAN 底层转向电机控制接口
//速度模型转换底层控制信号(自动）
void NavCarManagerNode::velDataHandle(double vx,double rz)
{
    // 发送UI更新信号
    emit calculatedVelocityUpdate(vx, rz);

    VelControlParams myVelControlParams=myVelControlModel.calAKMControlParams_new(vx,rz);
    sendVelControlMsg(myVelControlParams);
}

//速度模型转换手动控制信号
void NavCarManagerNode::velManualDataHandle(double vx,double theta)
{
    VelControlParams myVelControlParams=myVelControlModel.calAKMThetaControlParams_new(vx,theta);
    sendVelControlMsg(myVelControlParams);
}

void NavCarManagerNode::sendVelControlMsg(VelControlParams myVelControlParams)
{
    QByteArray sendData;
    sendData.resize(0);
    sendData.append(myVelControlParams.controlMode);
    sendData.append(myVelControlParams.velParams_unsigned);
    sendData.append(myVelControlParams.velParams_direction);
    sendData.append(myVelControlParams.theta_unsigned);
    sendData.append(myVelControlParams.theta_direction);
    sendCANMsg(0x8F910D8,sendData,5);
}

void NavCarManagerNode::sendSteeringControlMsg(SteeringParams mySteeringsParams,int controlMode)
{
    // 原有实现完全保持不变
    QByteArray sendData;
    sendData.resize(0);
    sendData.append(mySteeringsParams.right_front_steering);
    sendData.append(mySteeringsParams.left_front_steering);
    sendData.append(mySteeringsParams.right_rear_steering);
    sendData.append(mySteeringsParams.left_rear_steering);
    sendData.append(controlMode);
    sendCANMsg(0x8F110D8,sendData,5); //转向角度
}

void NavCarManagerNode::sendMotorControlMsg(MotorParams myMotorParams)
{
    // 原有实现完全保持不变
    QByteArray sendData;
    sendData.resize(0);
    sendData.append(myMotorParams.left_front_motor1);
    sendData.append(myMotorParams.left_front_motor2);
    sendData.append(myMotorParams.right_front_motor1);
    sendData.append(myMotorParams.right_front_motor2);
    sendData.append(myMotorParams.left_rear_motor1);
    sendData.append(myMotorParams.left_rear_motor2);
    sendData.append(myMotorParams.right_rear_motor1);
    sendData.append(myMotorParams.right_rear_motor2);
    sendCANMsg(0x8F210D8,sendData,8);
}

void NavCarManagerNode::velDataHandle(double vx,double theta,int controlMode)
{
    VelControlParams myVelControlParams=myVelControlModel.calAKMThetaControlParams_new(vx,theta);
    sendVelControlMsg(myVelControlParams);
}

//接近模式自动响应
void NavCarManagerNode::velApproachCMDHandle(double targetX, double targetY, double currentX,
                                             double currentY, double currentYaw)
{
    VelControlParams myVelControlParams=myVelControlModel.calApproachParams_new(targetX,targetY,currentX,currentY,currentYaw);
    qDebug()<<(QString("斜行控制: δa=%1° δb=%2° 速度=%3m/s")
                     .arg(QString::number(myVelControlParams.theta_direction, 'f', 1))
                     .arg(QString::number(myVelControlParams.theta_unsigned, 'f', 1))
                     .arg(QString::number(myVelControlParams.velParams_unsigned, 'f', 3)));
    sendVelControlMsg(myVelControlParams);
}

//原地旋转模式
void NavCarManagerNode::velOrientationCMDHandle(double targetYaw, double currentYaw)
{
    VelControlParams myVelControlParams=myVelControlModel.calOrientationParams_new(targetYaw,currentYaw);
    qDebug()<<(QString("姿态调整: 转向=%1° 误差=%2° 旋转速度=%3m/s")
                     .arg(QString::number(myVelControlParams.theta_direction, 'f', 1))
                     .arg(QString::number(myVelControlParams.theta_unsigned, 'f', 1))
                     .arg(QString::number(myVelControlParams.velParams_unsigned, 'f', 3)));
    sendVelControlMsg(myVelControlParams);
}

//====新增数据记录类，验证超声波数据有效性======
void NavCarManagerNode::startSaveUltrasonicData()
{
    // 生成带时间戳的文件名（避免覆盖历史数据）
    fileName = QString("ultrasonic_data_%1.csv")
                   .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));

    file.setFileName(fileName);

    // 首次写入时创建文件并添加表头
    if (!file.exists()) {
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream header(&file);
            header << "Timestamp,Device1,Device2,Device3,Device4,Device5,Device6\n";
            file.close();
        }
    }
}

void NavCarManagerNode::finishSaveUltrasonicData()
{
    file.close();
}

void NavCarManagerNode::saveUltrasonicData(const QVector<float>& distances) {
    // 追加新数据行
    if (file.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&file);
        QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
        out << timestamp; // 写入时间戳

        // 写入6个传感器的距离值
        for (const auto& dist : distances) {
            out << "," << dist;
        }
        out << "\n";
        file.close(); // 确保数据刷入磁盘
    } else {
        qWarning() << "Failed to open file for writing:" << file.errorString();
    }
}

//解析电机报文
void NavCarManagerNode::parseMotorMessage(quint32 messageId, const quint8 data[], quint8 dataLen)
{
    if (dataLen < 8) {
        qDebug() << "电机数据长度不足，期望8字节，实际:" << dataLen;
        return;
    }

    // 解析电机组号
    int motorGroup = 0;
    switch (messageId) {
    case 0x4F110D0: motorGroup = 1; break;
    case 0x4F210D0: motorGroup = 2; break;
    case 0x4F310D0: motorGroup = 3; break;
    case 0x4F410D0: motorGroup = 4; break;
    default: return;
    }

    // 解析使能状态
    quint8 enableByte = static_cast<quint8>(data[0]);
    bool enabled = (enableByte == 0x0D || enableByte == 0x00); // 0D或00表示使能

    // 解析电机编号 (01或02)
    quint8 motorNumberByte = static_cast<quint8>(data[1]);
    int motorNumber = motorNumberByte;

    // 解析转速 (第3、4字节)
    quint16 speedRaw = static_cast<quint16>(static_cast<quint8>(data[4]) << 8 | static_cast<quint8>(data[5]));
    float rpm = 0;
    if (speedRaw > 32767) { // 65535/2 = 32767.5
        rpm = (65535.0f - speedRaw) / 10000.0f * 3000.0f / 24.7f;
    } else {
        rpm = speedRaw / 10000.0f * 3000.0f / 24.7f;
    }

    // 解析电流(第5、6字节)
    quint16 currentRaw1 = static_cast<quint16>(static_cast<quint8>(data[4]) << 8 | static_cast<quint8>(data[5]));
    float current1 = 0;
    if (currentRaw1 > 32767) { // 65535/2 = 32767.5
        current1 = static_cast<float>(currentRaw1 - 65535);
    } else {
        current1 = static_cast<float>(currentRaw1);
    }

    // 解析电流 (第7、8字节)
    quint16 currentRaw2 = static_cast<quint16>(static_cast<quint8>(data[6]) << 8 | static_cast<quint8>(data[7]));
    float current2 = 0;
    if (currentRaw2 > 32767) { // 65535/2 = 32767.5
        current2 = static_cast<float>(currentRaw2 - 65535);
    } else {
        current2 = static_cast<float>(currentRaw2);
    }

    emit canMotorDataUpdate(motorGroup, motorNumber, enabled, rpm, current1, current2);
}

//油压数据
void NavCarManagerNode::parseOilPressureMessage(quint32 messageId, const quint8 data[], quint8 dataLen)
{
    if (dataLen < 8) {
        return;
    }

    // 解析4个油压传感器数据 (每个占2字节)
    float pressure1 = 0, pressure2 = 0, pressure3 = 0, pressure4 = 0;
    
    // 油压传感器1 - 字节1-2
    if (dataLen >= 2) {
        quint16 pressureRaw1 = static_cast<quint16>(static_cast<quint8>(data[0]) << 8 | static_cast<quint8>(data[1]));
        pressure1 = pressureRaw1 * 0.04f;
    }
    
    // 油压传感器2 - 字节3-4
    if (dataLen >= 4) {
        quint16 pressureRaw2 = static_cast<quint16>(static_cast<quint8>(data[2]) << 8 | static_cast<quint8>(data[3]));
        pressure2 = pressureRaw2 * 0.04f;
    }
    
    // 油压传感器3 - 字节5-6
    if (dataLen >= 6) {
        quint16 pressureRaw3 = static_cast<quint16>(static_cast<quint8>(data[4]) << 8 | static_cast<quint8>(data[5]));
        pressure3 = pressureRaw3 * 0.04f;
    }
    
    // 油压传感器4 - 字节7-8
    if (dataLen >= 8) {
        quint16 pressureRaw4 = static_cast<quint16>(static_cast<quint8>(data[6]) << 8 | static_cast<quint8>(data[7]));
        pressure4 = pressureRaw4 * 0.04f;
    }

    emit canOilPressureUpdate(pressure1, pressure2, pressure3, pressure4);
}

//油温数据、系统状态
void NavCarManagerNode::parseHydraulicSystemMessage(quint32 messageId, const quint8 data[], quint8 dataLen)
{
    if (dataLen < 7) {
        qWarning() << "液压系统数据长度不足，期望至少7字节，实际:" << dataLen;
        return;
    }
    // byte1-2：油温数据，0.01摄氏度/bit
    float temperature = 0;
    if (dataLen >= 2) {
        quint16 tempRaw = static_cast<quint16>(static_cast<quint8>(data[0]) << 8 | static_cast<quint8>(data[1]));
        temperature = tempRaw * 0.01f;
    }
    emit canOilTemperatureUpdate(temperature);

    // byte3：模式数据
    int mode = 0;
    if (dataLen >= 3) {
        mode = static_cast<quint8>(data[2]);
    }
    emit canModeUpdate(mode);

    // byte4前4bit：液压锁1-4状态
    bool lock1 = false, lock2 = false, lock3 = false, lock4 = false;
    if (dataLen >= 4) {
        quint8 lockByte = static_cast<quint8>(data[3]);
        lock1 = (lockByte & 0x01) != 0;
        lock2 = (lockByte & 0x02) != 0;
        lock3 = (lockByte & 0x04) != 0;
        lock4 = (lockByte & 0x08) != 0;
    }
    emit canHydraulicLockUpdate(lock1, lock2, lock3, lock4);

    // byte5前4bit：四个灯状态
    bool light1 = false, light2 = false, light3 = false, light4 = false;
    if (dataLen >= 5) {
        quint8 lightByte = static_cast<quint8>(data[4]);
        light1 = (lightByte & 0x01) != 0;  // 左转前灯1
        light2 = (lightByte & 0x02) != 0;  // 左转后灯2
        light3 = (lightByte & 0x04) != 0;  // 右转前灯3
        light4 = (lightByte & 0x08) != 0;  // 右转后灯4
    }
    emit canLightUpdate(light1, light2, light3, light4);

    // byte6前4bit：三种急停状态
    bool remote = false, passive = false, safety = false, blockage = false;
    if (dataLen >= 6) {
        quint8 emergencyByte = static_cast<quint8>(data[5]);
        remote = (emergencyByte & 0x01) != 0;      // 遥控器
        passive = (emergencyByte & 0x02) != 0;     // 被动
        safety = (emergencyByte & 0x04) != 0;      // 安全
        blockage = (emergencyByte & 0x08) != 0;    // 堵塞信号
    }
    emit canEmergencyUpdate(remote, passive, safety, blockage);

    // byte7/8字节：油泵电机转速
    float pumpmotorspeed = 0;
    if (dataLen >= 7) {
        quint16 pumpmotorspeed = static_cast<quint16>(static_cast<quint8>(data[7]) << 8 | static_cast<quint8>(data[6]));
    }
    emit canbumpmotorspeedUpdate(pumpmotorspeed);
}

void NavCarManagerNode::parseEncoderMessage(quint32 messageId, const quint8 data[], quint8 dataLen)
{
    if (dataLen < 2) {
        return;
    }

    // 确定编码器ID
    int encoderId = 0;
    switch (messageId) {
    case 0x191: encoderId = 1; break;
    case 0x192: encoderId = 2; break;
    case 0x193: encoderId = 3; break;
    case 0x194: encoderId = 4; break;
    default: return;
    }

    quint8 HIGH = data[1];
    quint8 LOW = data[0];
    // 解析编码器数据：data[0]高位，data[1]低位
    quint16 encoderValue = static_cast<quint16>(static_cast<quint8>(data[1]) << 8 | static_cast<quint8>(data[0]));
    
    // 计算角度：(数值/8192*360-180)
    float angle = (static_cast<float>(encoderValue) / 8192.0f * 360.0f) - 180.0f;
    
    
    // 发射信号，参数为编码器ID和角度值
    emit canEncoderUpdate(encoderId, encoderValue, angle);
}

void NavCarManagerNode::parseOilPumpMessage(quint32 messageId, const quint8 data[], quint8 dataLen)
{
    if (dataLen < 8) {
        qWarning() << "油泵电机数据长度不足，期望至少8字节，实际:" << dataLen;
        return;
    }

    // 解析直流电压 (BYTE1高 + BYTE0低)
    float motorVoltage = 0;
    if (dataLen >= 2) {
        quint16 voltageRaw = static_cast<quint16>(static_cast<quint8>(data[1]) << 8 | static_cast<quint8>(data[0]));
        motorVoltage = static_cast<float>(voltageRaw); 
    }

    // 解析电机电流 (BYTE3高 + BYTE2低)
    float motorCurrent = 0;
    if (dataLen >= 4) {
        quint16 currentRaw = static_cast<quint16>(static_cast<quint8>(data[3]) << 8 | static_cast<quint8>(data[2]));
        motorCurrent = static_cast<float>(currentRaw); 
    }

    // 解析电机温度 (BYTE5高 + BYTE4低)
    float motorTemperature = 0;
    if (dataLen >= 6) {
        quint16 tempRaw = static_cast<quint16>(static_cast<quint8>(data[5]) << 8 | static_cast<quint8>(data[4]));
        motorTemperature = static_cast<float>(tempRaw); 
    }

    // 解析控制器温度 (BYTE7高 + BYTE6低)
    float controllerTemperature = 0;
    if (dataLen >= 8) {
        quint16 ctrlTempRaw = static_cast<quint16>(static_cast<quint8>(data[7]) << 8 | static_cast<quint8>(data[6]));
        controllerTemperature = static_cast<float>(ctrlTempRaw); 
    }

    emit canOilPumpUpdate(motorVoltage, motorCurrent, motorTemperature, controllerTemperature);
}

void NavCarManagerNode::parseBatteryStatusMessage(quint32 messageId, const quint8 data[], quint8 dataLen)
{
    if (dataLen < 8) {
        qWarning() << "电池状态数据长度不足，期望至少8字节，实际:" << dataLen;
        return;
    }

    // 解析SOC (BYTE5)
    int soc = 0;
    if (dataLen >= 5) {
        soc = static_cast<int>(static_cast<quint8>(data[4]) * 0.4f); // BYTE5转10进制后*0.4
    }

    // 解析SOE (BYTE6)
    int soe = 0;
    if (dataLen >= 6) {
        soe = static_cast<int>(static_cast<quint8>(data[5]) * 0.4f); // BYTE6转10进制后*0.4
    }

    // 解析SOH (BYTE7)
    int soh = 0;
    if (dataLen >= 7) {
        soh = static_cast<int>(static_cast<quint8>(data[6]) * 0.4f); // BYTE7转10进制后*0.4
    }

    // 解析LIFE (BYTE8)
    int life = 0;
    if (dataLen >= 8) {
        life = static_cast<int>(static_cast<quint8>(data[7]) * 0.4f); // BYTE8转10进制后*0.4
    }

    qDebug() << "电池状态 - SOC:" << soc << "%, SOE:" << soe << "%, SOH:" << soh << "%, LIFE:" << life << "%";

    emit canBatteryStatusUpdate(soc, soe, soh, life);
}

// ========== CAN模式切换处理 ==========
void NavCarManagerNode::handleCANModeChange(int newMode)
{
    // 检测模式变化
    if (newMode == lastCANMode) {
        return;  // 模式未变化，无需处理
    }
    
    int oldMode = lastCANMode;
    lastCANMode = newMode;
    
    qDebug() << "CAN模式切换：" << oldMode << " -> " << newMode;
    
    // 检测从手调模式切换到自调模式
    if (oldMode == CAN_MODE_MANUAL && newMode == CAN_MODE_AUTO) {
        qDebug() << "检测到从手动模式切换到自动模式，尝试恢复导航...";
        
        // 使用QMetaObject::invokeMethod跨线程调用ROSWorkManager的resumeNavigation
        QMetaObject::invokeMethod(&myROSWorkManager, "resumeNavigation", 
                                  Qt::QueuedConnection);
    }
    // 检测从自调模式切换到手调模式
    else if (oldMode == CAN_MODE_AUTO && newMode == CAN_MODE_MANUAL) {
        qDebug() << "检测到从自动模式切换到手动模式，暂停导航...";
        
        // 使用QMetaObject::invokeMethod跨线程调用ROSWorkManager的pauseNavigation
        QMetaObject::invokeMethod(&myROSWorkManager, "pauseNavigation", 
                                  Qt::QueuedConnection);
    }
    // 从其他模式切换到自调模式（如复位后直接切到自调）
    else if (newMode == CAN_MODE_AUTO && oldMode != 0) {
        qDebug() << "检测到切换到自动模式，检查是否需要恢复导航...";
        
        // 只有当ROSWorkManager处于暂停状态时才恢复
        if (myROSWorkManager.isNavigationPaused()) {
            QMetaObject::invokeMethod(&myROSWorkManager, "resumeNavigation", 
                                      Qt::QueuedConnection);
        }
    }
    // 从自调模式切换到其他模式（非手调）
    else if (oldMode == CAN_MODE_AUTO && newMode != CAN_MODE_MANUAL) {
        qDebug() << "检测到从自动模式切换到其他模式，暂停导航...";
        
        QMetaObject::invokeMethod(&myROSWorkManager, "pauseNavigation", 
                                  Qt::QueuedConnection);
    }
}
