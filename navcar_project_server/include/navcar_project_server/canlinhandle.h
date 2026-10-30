#ifndef CANLINHANDLE_H
#define CANLINHANDLE_H

#include <QObject>
#include"lininterface.h"
#include"mylininterface_hw_mi.h"
#include"socketcanmanager.h"

//超声波传感器数据结构
struct EchoData {
    quint8 sensorID;     // 传感器ID
    quint16 flightTime1; // 回波1飞行时间
    quint8 amplitude1;   // 回波1信号幅值
    quint8 width1;       // 回波1信号宽度
    quint8 peakPos1;     // 回波1尖峰位置
    quint16 flightTime2; // 回波2飞行时间
    quint8 amplitude2;   // 回波2信号幅值
    quint8 width2;       // 回波2信号宽度
    quint8 peakPos2;     // 回波2尖峰位置
    quint8 residualTime; // 余振时间
    bool powerStatus;    // 电源检测状态
    // 新增状态标记
    bool isValid = true;       // 默认有效
    bool isUpdated = false;    // 默认未更新
    QDateTime timestamp;       // 数据保存时间
};

class UltrasonicDataContainer {
private:
    QVector<EchoData> dataList; // 存储0-6个传感器数据
    const int MAX_SIZE = 6;
    mutable QMutex mutex;       // 线程安全锁

public:
    UltrasonicDataContainer() {
        dataList.reserve(MAX_SIZE);
    }

    // 核心功能：通过sensorID匹配并更新数据（动态扩容/更新）
    bool setData(const EchoData& newData) {
        QMutexLocker locker(&mutex); // 加锁保证线程安全

        // 查找是否已存在相同ID的数据
        auto it = std::find_if(dataList.begin(), dataList.end(),
                               [&](const EchoData& data) { return data.sensorID == newData.sensorID; });

        if (it != dataList.end()) {
            // 更新现有数据
            *it = newData;
            it->isUpdated = true;
            it->timestamp = QDateTime::currentDateTime();
        } else {
            // 新增数据（不超过容量）
            if (dataList.size() >= MAX_SIZE) return false;
            EchoData data = newData;
            data.isUpdated = true;
            data.timestamp = QDateTime::currentDateTime();
            dataList.append(data);
        }
        return true;
    }

    // 标记指定传感器为无效
    void markInvalid(quint8 sensorID) {
        QMutexLocker locker(&mutex);
        auto it = std::find_if(dataList.begin(), dataList.end(),
                               [=](const EchoData& data) { return data.sensorID == sensorID; });
        if (it != dataList.end()) {
            it->isValid = false;
            it->timestamp = QDateTime::currentDateTime();
        }
    }

    // 检查所有传感器是否均已更新
    bool areAllUpdated() const {
        QMutexLocker locker(&mutex);
        return dataList.isEmpty() ||
               std::all_of(dataList.begin(), dataList.end(),
                           [](const EchoData& data) { return data.isUpdated; });
    }

    // 获取数据（返回数据和有效性状态）
    QPair<EchoData, bool> getData(quint8 sensorID) const {
        QMutexLocker locker(&mutex);
        auto it = std::find_if(dataList.begin(), dataList.end(),
                               [=](const EchoData& data) { return data.sensorID == sensorID; });
        if (it != dataList.end()) {
            return qMakePair(*it, it->isValid);
        }
        return qMakePair(EchoData(), false); // 未找到
    }

    // 重置更新标志
    void resetUpdateFlags() {
        QMutexLocker locker(&mutex);
        for (auto& data : dataList) {
            data.isUpdated = false;
        }
    }
};

class CANLINHandle : public QObject
{
    Q_OBJECT
public:
    explicit CANLINHandle(QObject *parent = nullptr);
    ~CANLINHandle();
    void cleanup();
private:
    MyLinInterface_HW_MI linInterface1;
    //MyLinInterface linInterface1; //LIN操作类(1通道）
    LinInterface linInterface2;//LIN操作类(2通道）
    SocketCanManager socketCANManager; //CAN操作类


private slots:
    //对外部接口
    //LIN1操作
    void connectToLIN1Port(QString portName, qint32 baudRate);//端口 波特率
    void disconnectToLIN1Port();
    void sendLIN1Msg(quint8 frameID, QByteArray data);
    //LIN2操作
    void connectToLIN2Port(QString portName, qint32 baudRate);//端口 波特率
    void disconnectToLIN2Port();
    void sendLIN2Msg(quint8 frameID, QByteArray data);
    //刷新可用设备列表
    void refreshAvailablePorts();
    //LIN传感器解析函数
    void parseUltrasonicData(quint8 sensorID, QByteArray frame);
    void parseUltrasonicData_Channel2(quint8 sensorID, QByteArray frame);
    //LIN1请求
    void onRequestFrame(quint8 id, int expectedDataLength);
    //CAN接口操作
    void openCANInterface(QString interfaceName);
    void closeCANInterface();
    void refreshAvailableCANPorts();
    void sendCANMsg(quint32 id,QByteArray data,quint8 datalen);

    //===两路LIN初始化，初始化探头参数===




signals:
    void ResponseAvailablePortsSignal(QStringList);
    void ResponseAvailableCANPortsSignal(QStringList);
    void LogDataSignal(QString);
    void frameReceived(LinFrame);
    void RecvLIN1FinalFrameSignal();
    void RecvLIN2FinalFrameSignal();
    void SendLIN1SensorData(EchoData);
    void SendLIN2SensorData(EchoData);
    void MessageReceivedSignal(CanMessage message);

};

#endif // CANLINHANDLE_H
