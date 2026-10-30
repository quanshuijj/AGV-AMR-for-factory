#include "navcar_project_server/canlinhandle.h"

CANLINHandle::CANLINHandle(QObject *parent)
    : QObject{parent},linInterface1(this),linInterface2(this),socketCANManager(this)
{
    connect(&linInterface1,SIGNAL(frameReceivedSignal(quint8,QByteArray)),this,
            SLOT(parseUltrasonicData(quint8,QByteArray)));
    connect(&socketCANManager,&SocketCanManager::messageReceived,this,&CANLINHandle::MessageReceivedSignal);
}

CANLINHandle::~CANLINHandle()
{
    cleanup();
}

void CANLINHandle::cleanup()
{
    disconnectToLIN1Port();
    disconnectToLIN2Port();
}

//LIN操作
void CANLINHandle::connectToLIN1Port(QString portName, qint32 baudRate)
{
    if(linInterface1.connectToPort(portName,baudRate))
    {
        qDebug()<<"LIN1连接成功";
        emit LogDataSignal("LIN1连接成功");
    }
    else
    {
        emit LogDataSignal("LIN1连接失败");
        qDebug() << "LIN1连接失败:" << linInterface1.getErrorString();
    }
}

void CANLINHandle::connectToLIN2Port(QString portName, qint32 baudRate)
{
    if(linInterface2.connectToPort(portName,baudRate))
    {
        qDebug()<<"LIN2连接成功";
        emit LogDataSignal("LIN2连接成功");
    }
    else
    {
        qDebug() << "LIN2连接失败:" << linInterface2.getErrorString();
        emit LogDataSignal("LIN2连接失败");
    }
}

void CANLINHandle::disconnectToLIN1Port()
{
    linInterface1.disconnectFromPort();
}

void CANLINHandle::disconnectToLIN2Port()
{
    linInterface2.disconnectFromPort();
}

void CANLINHandle::refreshAvailablePorts()
{
    qDebug()<<"刷新";
    QStringList availablePortsList=linInterface1.getAvailablePorts();
    qDebug()<<availablePortsList;
    emit ResponseAvailablePortsSignal(availablePortsList);
}

void CANLINHandle::sendLIN1Msg(quint8 frameID, QByteArray data)
{
    qDebug()<<"收到发送数据";
    if(linInterface1.sendFrame(frameID,data))
    {
        qDebug()<<"发送成功";
        emit LogDataSignal(QString("→ 发送帧 ID:0x%1 数据:%2")
            .arg(frameID, 2, 16, QChar('0'))
                               .arg(QString(data.toHex())));

    }
    else
    {
        qDebug()<<"发送失败";
        emit LogDataSignal("发送失败");
    }
}

void CANLINHandle::sendLIN2Msg(quint8 frameID, QByteArray data)
{
    if(linInterface2.sendFrame(frameID,data))
    {
        qDebug()<<"发送成功";
        emit LogDataSignal(QString("→ 发送帧 ID:0x%1 数据:%2")
                               .arg(frameID, 2, 16, QChar('0'))
                               .arg(QString(data.toHex())));

    }
    else
    {
        qDebug()<<"发送失败";
        emit LogDataSignal("发送失败");
    }
}

void CANLINHandle::parseUltrasonicData(quint8 sensorID, QByteArray frame)
{
    EchoData result;
    result.sensorID = sensorID;  // 存储传感器ID

    // 检查报文长度（至少8字节）
    if (frame.size() < 8) {
        qWarning() << "[" << sensorID << "] Invalid frame size:" << frame.size();
        return;
    }

    // 1. 解析回波1数据
    result.flightTime1 = static_cast<quint8>(frame[0]); // 低8位
    result.amplitude1 = static_cast<quint8>(frame[1]) & 0x3F; // 信号幅值(低6位)
    result.flightTime1 |= ((static_cast<quint8>(frame[1]) >> 6) & 0x03) << 8; // 添加高2位

    // 2. 解析回波2数据
    result.flightTime2 = static_cast<quint8>(frame[2]); // 低8位
    result.amplitude2 = static_cast<quint8>(frame[3]) & 0x3F; // 信号幅值(低6位)
    result.flightTime2 |= ((static_cast<quint8>(frame[3]) >> 6) & 0x03) << 8; // 添加高2位

    // 3. 解析信号宽度
    quint8 widthData = static_cast<quint8>(frame[4]);
    result.width1 = (widthData >> 4) & 0x0F;  // 回波1宽度(高4位)
    result.width2 = widthData & 0x0F;         // 回波2宽度(低4位)

    // 4. 解析尖峰位置
    quint8 peakData = static_cast<quint8>(frame[5]);
    result.peakPos1 = peakData & 0x0F;        // 回波1位置(低4位)
    result.peakPos2 = (peakData >> 4) & 0x0F; // 回波2位置(高4位)

    // 5. 解析其他数据
    result.residualTime = static_cast<quint8>(frame[6]) & 0x3F;  // 余振时间(低6位)
    result.powerStatus = static_cast<quint8>(frame[7]) & 0x01;   // 电源检测(最低位)

    //qDebug()<<"处理完一个完整数据"<<result.flightTime1;
    //qDebug()<<"计算距离"<<result.flightTime1 * 340 * 0.064f / 2000.0f;
    result.timestamp=QDateTime::currentDateTime();
    emit SendLIN1SensorData(result);
}

void CANLINHandle::parseUltrasonicData_Channel2(quint8 sensorID, QByteArray frame)
{
    EchoData result;
    result.sensorID = sensorID+6;  // 存储传感器ID

    // 检查报文长度（至少8字节）
    if (frame.size() < 8) {
        qWarning() << "[" << sensorID << "] Invalid frame size:" << frame.size();
        return;
    }

    // 1. 解析回波1数据
    result.flightTime1 = static_cast<quint8>(frame[0]); // 低8位
    result.amplitude1 = static_cast<quint8>(frame[1]) & 0x3F; // 信号幅值(低6位)
    result.flightTime1 |= ((static_cast<quint8>(frame[1]) >> 6) & 0x03) << 8; // 添加高2位

    // 2. 解析回波2数据
    result.flightTime2 = static_cast<quint8>(frame[2]); // 低8位
    result.amplitude2 = static_cast<quint8>(frame[3]) & 0x3F; // 信号幅值(低6位)
    result.flightTime2 |= ((static_cast<quint8>(frame[3]) >> 6) & 0x03) << 8; // 添加高2位

    // 3. 解析信号宽度
    quint8 widthData = static_cast<quint8>(frame[4]);
    result.width1 = (widthData >> 4) & 0x0F;  // 回波1宽度(高4位)
    result.width2 = widthData & 0x0F;         // 回波2宽度(低4位)

    // 4. 解析尖峰位置
    quint8 peakData = static_cast<quint8>(frame[5]);
    result.peakPos1 = peakData & 0x0F;        // 回波1位置(低4位)
    result.peakPos2 = (peakData >> 4) & 0x0F; // 回波2位置(高4位)

    // 5. 解析其他数据
    result.residualTime = static_cast<quint8>(frame[6]) & 0x3F;  // 余振时间(低6位)
    result.powerStatus = static_cast<quint8>(frame[7]) & 0x01;   // 电源检测(最低位)

    qDebug()<<"处理完一个完整数据"<<result.flightTime1;
    qDebug()<<"计算距离"<<result.flightTime1 * 340 * 0.064f / 2000.0f;
    result.timestamp=QDateTime::currentDateTime();
    emit SendLIN2SensorData(result);
}

void CANLINHandle::onRequestFrame(quint8 id, int expectedDataLength)
{
    linInterface1.requestFrame(id,expectedDataLength);
}

//CAN操作
void CANLINHandle::openCANInterface(QString interfaceName)
{
    socketCANManager.openInterface(interfaceName);
}

void CANLINHandle::closeCANInterface()
{
    socketCANManager.closeInterface();
}

void CANLINHandle::refreshAvailableCANPorts()
{
    qDebug()<<"刷新CAN口";
    QStringList availableCANPortsList=socketCANManager.getAvailableInterfaces();
    qDebug()<<availableCANPortsList;
    emit ResponseAvailableCANPortsSignal(availableCANPortsList);
}

void CANLINHandle::sendCANMsg(quint32 id,QByteArray data,quint8 datalen)
{
    CanMessage canMsg;
    canMsg.id=id;
    canMsg.dataLen=datalen;
    canMsg.isExtended=1;
    memcpy(canMsg.data,data.data(),datalen);
    if (!socketCANManager.sendMessage(canMsg)) {
        qWarning()<<"CAN报文发送失败，id:"<<QString::number(id, 16);
    }

}
