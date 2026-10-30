// navcarmanagernode.h - 车辆管理节点（完整修改版）
#ifndef NAVCARMANAGERNODE_H
#define NAVCARMANAGERNODE_H

#include <QElapsedTimer>
#include <QObject>
#include <QThread>

#include "calikcontrolmodel.h"
#include "canlinhandle.h"
#include "data_broadcast_worker.h"  // 修改：使用Worker类替代Manager
#include "rosworkmanager.h"
#include "datalogger.h"

class SignalScheduler : public QObject {
    Q_OBJECT
public:
    explicit SignalScheduler(QObject* parent = nullptr)
        : QObject(parent), m_timer(new QTimer(this))
    {
        m_timer->setSingleShot(true);
        connect(m_timer, &QTimer::timeout, this, &SignalScheduler::sendNextSignal);
    }

    void scheduleSignal(qint64 nextDelayMs, quint8 arg1, const QByteArray& arg2,quint8 type,quint8 len) {
        if (QThread::currentThread() != thread()) {
            const QByteArray dataCopy = arg2;
            QMetaObject::invokeMethod(this, [this, nextDelayMs, arg1, dataCopy, type, len]() {
                scheduleSignal(nextDelayMs, arg1, dataCopy, type, len);
            }, Qt::QueuedConnection);
            return;
        }

        QMutexLocker locker(&m_mutex);
        m_queue.enqueue({nextDelayMs, arg1, arg2,type,len});

        if (!m_timer->isActive()) {
            m_elapsedTimer.start();
            m_timer->start(nextDelayMs);
        }
    }

signals:
    void SendLIN1MsgSignal(quint8, QByteArray);
    void SendRequestLIN1MsgSignal(quint8,int);

private slots:
    void sendNextSignal() {
        QMutexLocker locker(&m_mutex);
        if (m_queue.isEmpty()) return;

        const Task& task = m_queue.dequeue();
        if(task.type==0) {
            emit SendLIN1MsgSignal(task.arg1, task.arg2);
        } else {
            qDebug()<<"请求数据"<<task.arg1<<task.len;
            emit SendRequestLIN1MsgSignal(task.arg1,task.len);
        }

        if (!m_queue.isEmpty()) {
            const qint64 nextDelay = m_queue.head().delayMs;
            m_timer->start(nextDelay);
            qDebug()<<"实际间隔"<<m_elapsedTimer.elapsed();
            m_elapsedTimer.restart();
        }
    }

private:
    struct Task {
        qint64 delayMs;
        quint8 arg1;
        QByteArray arg2;
        quint8 type;
        int len;
    };

    QQueue<Task> m_queue;
    QTimer* m_timer;
    QElapsedTimer m_elapsedTimer;
    QMutex m_mutex;
};

class NavCarManagerNode : public QObject
{
    Q_OBJECT

public:
    explicit NavCarManagerNode(QObject *parent = nullptr);
    ~NavCarManagerNode();

    // ========== 原有控制接口 ==========
    void testClick();
    void setLINSensorParam();
    void setLINSensorEnable();
    void measureLINSensor();

    // ========== 超声波调试接口 ==========
    void startSaveUltrasonicData();
    void finishSaveUltrasonicData();
    void saveUltrasonicData(const QVector<float>& distances);
    QVector<float> getLatestUltraData();
    QString fileName = QString("ultrasonic_data_%1.csv")
                           .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
    QFile file;

    // ========== 数据广播管理接口（改为异步信号-槽模式）==========
    bool startDataBroadcastServer(quint16 port = 8888);
    void stopDataBroadcastServer();

    // 异步查询接口（结果通过信号返回）
    void requestServerStatus();
    void requestConnectedClientCount();
    void requestConnectedClientInfo();
    void requestConnectedClientIds();
    void requestDataBroadcastConfig();
    void requestDataBroadcastStatistics();
    void requestNetworkStatistics();

    // 控制接口（立即执行）
    void kickClient(quint32 clientId, const QString& reason = "管理员操作");
    void kickAllClients(const QString& reason = "服务器维护");
    void setClientStreamConfig(quint32 clientId, StreamType streamType, bool enabled);
    void setClientCameraSubscription(quint32 clientId, int cameraId, bool enabled);
    void setClientCompressionLevel(quint32 clientId, CompressionLevel level);
    void setDataBroadcastConfig(const BroadcastConfig& config);
    void pauseDataBroadcast(bool pause = true);
    void resumeDataBroadcast() { pauseDataBroadcast(false); }

private:
    // ========== 原有核心组件 ==========
    CANLINHandle CANLINWork;
    QThread CANLINWorkThread;
    SignalScheduler lin1SchedulerManager;
    CalIKControlModel myVelControlModel;
    ROSWorkManager myROSWorkManager;
    QThread myROSWorkThread;
    DataLogger myDataLogger;
    QThread myDataLoggerThread;

    quint16 m_pendingServerPort = 0;  // 待启动的服务器端口

    // ========== 修改：数据广播组件改为Worker模式 ==========
    DataBroadcastWorker* m_dataBroadcastWorker = nullptr;  // 使用Worker，无父对象
    bool m_dataBroadcastInitialized = false;
    QThread dataBroadcastWorkThread;

    // ========== 控制状态 ==========
    int controlMode = 0;
    double lastTheta = 0;
    
    // ========== CAN模式追踪（用于自动/手动模式切换）==========
    int lastCANMode = 0;  // 上一次CAN模式：1-复位, 2-手调, 3-自调, 4-原地转向, 5-斜行, 6-直行
    static constexpr int CAN_MODE_MANUAL = 2;  // 手调模式
    static constexpr int CAN_MODE_AUTO = 3;    // 自调模式

    // ========== LIN控制组件 ==========
    QTimer* myLIN1Timer;
    QTimer* myLIN2Timer;
    static constexpr int LIN_TIMER_MS = 150;
    UltrasonicDataContainer ultraFrontManager;
    UltrasonicDataContainer ultraRearManager;

    bool initializeDataBroadcast();
    void connectDataBroadcastSignals();
    void connectROSSignalsToWorker();
    void sendNewVelControlMsg(double vx,double rz,double controlMode);

    // CAN消息解析方法
    void parseMotorMessage(quint32 messageId, const quint8 data[], quint8 dataLen);
    void parseOilPressureMessage(quint32 messageId, const quint8 data[], quint8 dataLen);
    void parseHydraulicSystemMessage(quint32 messageId, const quint8 data[], quint8 dataLen);
    void parseEncoderMessage(quint32 messageId, const quint8 data[], quint8 dataLen);
    void parseOilPumpMessage(quint32 messageId, const quint8 data[], quint8 dataLen);
    void parseBatteryStatusMessage(quint32 messageId, const quint8 data[], quint8 dataLen);
private slots:
    // ========== LIN/CAN槽函数 ==========
    void responseAvailablePorts(QStringList availablePorts);
    void sendLIN1Msg(quint8 id,QByteArray data);
    void sendLIN2Msg(quint8 id,QByteArray data);
    void responseAvailableCANPorts(QStringList availablePorts);
    void sendCANMsg(quint32 id,QByteArray data,quint8 datalen);
    void sendCANMsg(CanMessage sendMsg);
    void initLIN1SensorParams();
    void initLIN2SensorParams();
    void mesuareLIN1Once();
    void mesuareLIN2Once();
    void handleLIN1SensorData(EchoData);
    void handleLIN2SensorData(EchoData);
    void ultraSensorDetectHandle_front();
    void ultraSensorDetectHandle_rear();
    void handleCANMsg(CanMessage rcvMsg);
    
    // ========== CAN模式切换处理 ==========
    void handleCANModeChange(int newMode);

    // ========== Worker初始化槽函数 ==========
    void onBroadcastThreadStarted();
    void onWorkerInitializeResult(bool success);

    // ========== 数据广播事件处理 ==========
    void onDataBroadcastServerStarted(quint16 port);
    void onDataBroadcastServerStopped();
    void onDataBroadcastServerError(const QString& error);
    void onBroadcastClientConnected(quint32 clientId, const QString& info);
    void onBroadcastClientDisconnected(quint32 clientId, const QString& reason);

public slots:
    // ========== 车辆控制槽函数 ==========
    void velDataHandle(double vx,double rz);
    void velManualDataHandle(double vx,double theta);
    void velDataHandle(double vx,double theta,int baseControlMode);
    void velApproachCMDHandle(double targetX, double targetY, double currentX,
                              double currentY, double currentYaw);
    void velOrientationCMDHandle(double targetYaw, double currentYaw);
    void sendSteeringControlMsg(SteeringParams,int);
    void sendMotorControlMsg(MotorParams);
    void sendVelControlMsg(VelControlParams);
    void initLinDevice();
    void startMesuareLIN1();
    void startMesuareLIN2();
    void stopMesuareLIN1();
    void stopMesuareLIN2();

signals:
    // ========== LIN/CAN信号 ==========
    void ConnectToLIN1PortSignal(QString,qint32);
    void DisConnectToLIN1Signal();
    void SendLIN1MsgSignal(quint8,QByteArray);
    void ConnectToLIN2PortSignal(QString,qint32);
    void DisConnectToLIN2Signal();
    void SendLIN2MsgSignal(quint8,QByteArray);
    void RefreshAvailablePortsSignal();
    void RefreshPortDataSignal(QStringList);
    void LogDataSignal(QString);
    void RequestFrameSignal(quint8, int);
    void OpenCANInterfaceSignal(QString);
    void CloseCANInterfaceSignal();
    void RefreshAvailableCANPortsSignal();
    void RefreshCANPortDataSignal(QStringList);
    void SendCANMsgSignal(quint32 id,QByteArray data,quint8 datalen);
    void CANLogMsgSignal(CanMessage LogMsg);

    // ========== 数据广播信号 ==========
    void dataBroadcastServerStarted(quint16 port);
    void dataBroadcastServerStopped();
    void dataBroadcastServerError(const QString& error);
    void broadcastClientConnected(quint32 clientId, const QString& info);
    void broadcastClientDisconnected(quint32 clientId, const QString& reason);
    void dataBroadcastStatisticsUpdated(const BroadcastStatistics& stats);

    // ========== 查询结果信号 ==========
    void serverStatusReceived(bool isRunning);
    void connectedClientCountReceived(int count);
    void connectedClientInfoReceived(const QStringList& clientInfo);
    void connectedClientIdsReceived(const QList<quint32>& clientIds);
    void dataBroadcastConfigReceived(const BroadcastConfig& config);
    void dataBroadcastStatisticsReceived(const BroadcastStatistics& stats);
    void networkStatisticsReceived(const NetworkStatistics& stats);

    // 速度更新信号
    void calculatedVelocityUpdate(double linear, double angular);

    // ========== CAN信号解析 ==========
    void canMotorDataUpdate(int motorGroup, int motorNumber, bool enabled, float rpm, float current1, float current2);
    void canOilPressureUpdate(float pressure1, float pressure2, float pressure3, float pressure4);
    void canOilTemperatureUpdate(float temperature);
    void canModeUpdate(int mode);  // 1:复位, 2:手调, 3:自调, 4:原地转向, 5:斜行, 6:直行
    void canHydraulicLockUpdate(bool lock1, bool lock2, bool lock3, bool lock4);
    void canLightUpdate(bool light1, bool light2, bool light3, bool light4);  // 左转前灯1, 左转后灯2, 右转前灯3, 右转后灯4
    void canEmergencyUpdate(bool remote, bool passive, bool safety,bool blockage);
    void canbumpmotorspeedUpdate(float pumpmotorspeed);
    void canEncoderUpdate(int encoderId, quint8 encoderValue, float angle);
    void canOilPumpUpdate(float motorVoltage, float motorCurrent, float motorTemperature, float controllerTemperature);
    void canBatteryStatusUpdate(int soc, int soe, int soh, int life);
    
    // ROS odom信号
    void odomUpdate(const nav_msgs::Odometry& odom);
};

#endif // NAVCARMANAGERNODE_H
