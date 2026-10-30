#include "navcar_project_server/mylininterface_hw_mi.h"

MyLinInterface_HW_MI::MyLinInterface_HW_MI(QObject *parent)
    : QObject{parent}
    , m_serialPort(new QSerialPort(this))
    , m_timeoutTimer(new QTimer(this))
    , m_state(Disconnected)
    , m_lastError(NoError)
    , m_currentMode(StandbyMode)
    , m_baudRate(19200)
    , m_timeout(DEFAULT_TIMEOUT)
    , m_waitingForResponse(false)
    , m_expectedFrameId(0)
    , m_expectedDataLength(8)
    , m_framesSent(0)
    , m_framesReceived(0)
    , m_errorCount(0)
{
    connect(m_serialPort, &QSerialPort::readyRead,
            this, &MyLinInterface_HW_MI::handleReadyRead);
    connect(m_serialPort, QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::errorOccurred),
            this, &MyLinInterface_HW_MI::handleError);
    connect(m_timeoutTimer, &QTimer::timeout,
            this, &MyLinInterface_HW_MI::handleTimeout);

    // 设置超时定时器为单次触发
    m_timeoutTimer->setSingleShot(true);
}

MyLinInterface_HW_MI::~MyLinInterface_HW_MI()
{
    disconnectFromPort();
}

bool MyLinInterface_HW_MI::connectToPort(const QString& portName, qint32 baudRate)
{
    QMutexLocker locker(&m_mutex);

    if (m_state == Connected) {
        disconnectFromPort();
    }

    m_serialPort->setPortName(portName);
    m_serialPort->setBaudRate(460800);  // LINTest-MI固定波特率
    m_serialPort->setDataBits(QSerialPort::Data8);
    m_serialPort->setParity(QSerialPort::NoParity);
    m_serialPort->setStopBits(QSerialPort::OneStop);
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_serialPort->open(QIODevice::ReadWrite)) {
        setError(PortOpenError, QString("无法打开串口: %1").arg(m_serialPort->errorString()));
        return false;
    }

    m_baudRate = baudRate;
    m_receiveBuffer.clear();
    resetStatistics();

    // 先进入待机模式，然后配置LIN参数
    if (!setLinMode(StandbyMode)) {
        m_serialPort->close();
        qDebug()<<"无法进入待机模式";
        setError(PortOpenError, "无法进入待机模式");
        return false;
    }
    qDebug()<<"进入待机模式";

    //QThread::msleep(150); // 模式切换间隔
    QTimer::singleShot(150, this, &MyLinInterface_HW_MI::setLINMasterMode);
    return true;
/*
    if (!initLINSetting(baudRate)) {
        m_serialPort->close();
        qDebug()<<"无法配置LIN参数";
        setError(PortOpenError, QString("无法配置LIN参数"));
        return false;
    }
    qDebug()<<"配置波特率";
    QThread::msleep(150); // 配置完成间隔*/
/*
    // 进入主机模式
    if (!setLinMode(MasterMode)) {
        m_serialPort->close();
        qDebug()<<"无法进入主机模式";
        setError(PortOpenError, "无法进入主机模式");
        return false;
    }
    QThread::msleep(300);
    setState(Connected);
    qDebug() << "LINTest-MI配置成功，波特率:" << baudRate;
    return true;
*/
}

void MyLinInterface_HW_MI::setLINMasterMode()
{
    // 进入主机模式
    if (!setLinMode(MasterMode)) {
        m_serialPort->close();
        qDebug()<<"无法进入主机模式";
        setError(PortOpenError, "无法进入主机模式");
        //return false;
    }
    QThread::msleep(150);
    setState(Connected);
    qDebug() << "LINTest-MI配置成功，波特率:" << 19200;
    //return true;
}

void MyLinInterface_HW_MI::disconnectFromPort()
{
    QMutexLocker locker(&m_mutex);

    if (m_serialPort->isOpen()) {
        // 退出到待机模式
        setLinMode(StandbyMode);
        QThread::msleep(100);
        m_serialPort->close();
    }

    m_timeoutTimer->stop();
    m_waitingForResponse = false;
    m_receiveBuffer.clear();
    setState(Disconnected);
}

bool MyLinInterface_HW_MI::isConnected() const
{
    return m_state == Connected;
}

MyLinInterface_HW_MI::LinState MyLinInterface_HW_MI::getState() const
{
    return m_state;
}

MyLinInterface_HW_MI::LinError MyLinInterface_HW_MI::getLastError() const
{
    return m_lastError;
}

QString MyLinInterface_HW_MI::getErrorString() const
{
    return m_errorString;
}

void MyLinInterface_HW_MI::handleReadyRead()
{
    QMutexLocker locker(&m_mutex);

    QByteArray data = m_serialPort->readAll();
    if (data.isEmpty()) {
        return;
    }

    m_receiveBuffer.append(data);
    /*
    qDebug() << "读取到数据:" << data.toHex(' ');
    qDebug() << "=== 接收数据调试 ===";
    qDebug() << "接收到原始数据:" << data.toHex(' ');
    qDebug() << "当前缓冲区大小:" << m_receiveBuffer.size();
    qDebug() << "缓冲区内容:" << m_receiveBuffer.toHex(' ');
*/
    processReceivedData();
}

void MyLinInterface_HW_MI::processReceivedData()
{
    const int MIN_BUFFER_SIZE = 2;

    while (m_receiveBuffer.size() >= MIN_BUFFER_SIZE) {
        // 查找命令包头：0x33(读取响应) 或 0x44(监听数据)
        int headerIndex = findPacketHeader(0x33);
        if (headerIndex == -1) {
            headerIndex = findPacketHeader(0x44);
        }

        if (headerIndex == -1) {
            // 未找到包头，清除缓冲区（保留最后一个字节）
            if (m_receiveBuffer.size() > 1) {
                m_receiveBuffer = m_receiveBuffer.right(1);
            }
            break;
        }

        // 移除包头前的无效数据
        if (headerIndex > 0) {
            m_receiveBuffer.remove(0, headerIndex);
            qDebug() << QString("移除%1字节无效数据").arg(headerIndex);
        }

        // 检查是否有完整的数据包
        if (m_receiveBuffer.size() < PACKET_SIZE) {
            qDebug() << QString("数据包不完整，当前%1字节，需要%2字节")
                            .arg(m_receiveBuffer.size()).arg(PACKET_SIZE);
            break;
        }

        // 提取完整数据包
        QByteArray packet = m_receiveBuffer.left(PACKET_SIZE);

        // 验证数据包
        if (validatePacket(packet)) {
            handleValidPacket(packet);
            m_framesReceived++;

            // 如果正在等待响应，停止超时定时器
            if (m_waitingForResponse) {
                m_timeoutTimer->stop();
                m_waitingForResponse = false;
            }
        } else {
            m_errorCount++;
            qDebug() << "数据包校验失败:" << packet.toHex(' ');
        }

        // 移除已处理的数据包
        m_receiveBuffer.remove(0, PACKET_SIZE);
    }
}

int MyLinInterface_HW_MI::findPacketHeader(quint8 cmdByte) const
{
    for (int i = 0; i <= m_receiveBuffer.size() - PACKET_SIZE; i++) {
        if (static_cast<quint8>(m_receiveBuffer.at(i)) == cmdByte) {
            return i;
        }
    }
    return -1;
}

bool MyLinInterface_HW_MI::validatePacket(const QByteArray& packet) const
{
    if (packet.size() != PACKET_SIZE) {
        return false;
    }

    // 验证命令头
    quint8 cmd = static_cast<quint8>(packet.at(0));
    if (cmd != 0x33 && cmd != 0x44) {
        return false;
    }

    // 计算并验证CRC（前15字节相加，取反加1）
    quint8 calculatedCRC = calculatePacketCRC(packet.left(15));
    quint8 receivedCRC = static_cast<quint8>(packet.at(15));

    if (calculatedCRC != receivedCRC) {
        qDebug() << QString("包CRC错误: 计算值=0x%1, 接收值=0x%2")
                        .arg(calculatedCRC, 2, 16, QChar('0'))
                        .arg(receivedCRC, 2, 16, QChar('0'));
        return false;
    }

    return true;
}

void MyLinInterface_HW_MI::handleValidPacket(const QByteArray& packet)
{
    //qDebug() << "接收到有效数据包:" << packet.toHex(' ');

    quint8 cmd = static_cast<quint8>(packet.at(0));
    quint8 channel = static_cast<quint8>(packet.at(1));
    quint8 id = static_cast<quint8>(packet.at(2));
    quint8 direction = static_cast<quint8>(packet.at(3));
    quint8 checksumType = static_cast<quint8>(packet.at(4));
    quint8 length = static_cast<quint8>(packet.at(5));

    // 提取数据
    QByteArray data = packet.mid(6, length);
    quint8 linChecksum = static_cast<quint8>(packet.at(14));

    // 验证LIN校验和
    bool checksumValid = false;
    if (checksumType == EnhancedChecksum) {
        quint8 pid = calPID(id);
        checksumValid = verifyChecksum(pid, data, linChecksum);
    } else if (checksumType == ClassicChecksum) {
        quint8 expectedChecksum = calculateChecksum_classic(data);
        checksumValid = (expectedChecksum == linChecksum);
    }

    if (!checksumValid && checksumType != ChecksumInvalid) {
        qDebug() << QString("LIN校验和错误 - ID:0x%1, 类型:%2")
                        .arg(id, 2, 16, QChar('0')).arg(checksumType);
    }

    // 构造有效负载
    QByteArray payload;
    payload.append(id);
    payload.append(length);
    payload.append(data);
/*
    qDebug() << QString("解析数据 - 命令:0x%1, 通道:%2, ID:0x%3, 长度:%4, 数据:%5")
                    .arg(cmd, 2, 16, QChar('0'))
                    .arg(channel)
                    .arg(id, 2, 16, QChar('0'))
                    .arg(length);
                    //.arg(data.toHex(' '));
*/
    // 发送信号
    emit dataReceived(packet, payload);
    emit frameReceivedSignal(id, data);
}

void MyLinInterface_HW_MI::handleError(QSerialPort::SerialPortError error)
{
    QString errorString;
    LinError linError = NoError;

    switch (error) {
    case QSerialPort::DeviceNotFoundError:
        linError = PortNotFound;
        errorString = "设备未找到";
        break;
    case QSerialPort::PermissionError:
        linError = PortOpenError;
        errorString = "权限错误";
        break;
    case QSerialPort::OpenError:
        linError = PortOpenError;
        errorString = "打开错误";
        break;
    case QSerialPort::WriteError:
        linError = WriteError;
        errorString = "写入错误";
        break;
    case QSerialPort::ReadError:
        linError = ReadError;
        errorString = "读取错误";
        break;
    case QSerialPort::ResourceError:
        linError = PortOpenError;
        errorString = "资源错误，设备可能被意外移除";
        disconnectFromPort();
        break;
    case QSerialPort::UnsupportedOperationError:
        linError = PortOpenError;
        errorString = "不支持的操作";
        break;
    case QSerialPort::TimeoutError:
        linError = TimeoutError;
        errorString = "操作超时";
        break;
    default:
        if (error != QSerialPort::NoError) {
            linError = PortOpenError;
            errorString = QString("未知串口错误: %1").arg(static_cast<int>(error));
        }
        break;
    }

    if (linError != NoError) {
        m_errorCount++;
        setError(linError, errorString);
        qDebug() << "串口错误:" << errorString;
    }
}

void MyLinInterface_HW_MI::handleTimeout()
{
    if (m_waitingForResponse) {
        m_waitingForResponse = false;
        m_errorCount++;
        setError(TimeoutError, "等待响应超时");
        qDebug() << "响应超时";
        emit responseTimeout();
    }
}

bool MyLinInterface_HW_MI::setLinMode(LinMode mode)
{
    QByteArray packet;
    packet.resize(PACKET_SIZE);
    packet.fill(0x00);

    // 设置工作模式命令: 0x11
    packet[0] = 0x11;
    packet[1] = static_cast<quint8>(mode);

    // 波特率参数（仅在待机模式下有效）
    if (mode == StandbyMode||MasterMode) {
        packet[2] = 0x4B;//(m_baudRate >> 8) & 0xFF;
        packet[3] = 0X00;//m_baudRate & 0xFF;
    }

    // 计算并设置CRC
    packet[15] = calculatePacketCRC(packet.left(15));

    qint64 bytesWritten = m_serialPort->write(packet);
    if (bytesWritten == -1) {
        qDebug() << "设置模式失败: 写入错误";
        return false;
    }
    // 或者使用更短的超时
    /*if (!m_serialPort->waitForBytesWritten(100)) {
        qDebug() << "写入超时警告，但继续执行";
        // 不返回false，继续执行
    }*/
/*
    if (!m_serialPort->waitForBytesWritten(1000)) {
        qDebug() << "设置模式失败: 写入超时";
        return false;
    }
*/
    m_currentMode = mode;
    qDebug() << QString("设置LIN模式: %1").arg(mode);
    qDebug() << "=== 发送数据调试 ===";
    qDebug() << "目标ID:" << QString("0x%1").arg(0x11, 2, 16, QChar('0'));
    qDebug() << "发送数据:" << packet.toHex(' ');
    qDebug() << "构建的16字节包:" << packet.toHex(' ');
    return true;
}

bool MyLinInterface_HW_MI::initLINSetting(qint32 baudRate)
{
    // 在待机模式下设置波特率
    return setLinMode(StandbyMode);
}

bool MyLinInterface_HW_MI::sendFrame(quint8 id, QByteArray data)
{
    QMutexLocker locker(&m_mutex);

    if (!isConnected()) {
        setError(PortOpenError, "设备未连接");
        return false;
    }

    if (data.size() > 8) {
        qDebug() << "数据长度超过8字节，将被截断";
        data = data.left(8);
    }

    QByteArray packet;
    packet.resize(PACKET_SIZE);
    packet.fill(0x00);

    // 主机发送数据命令格式
    packet[0] = 0x22;                    // 命令头
    packet[1] = 0x00;                    // 通道
    packet[2] = id; //calPID(id);//id;                      // ID地址
    packet[3] = 0x00;                    // LIN数据传输方向
    packet[4] = EnhancedChecksum;        // 校验和类型
    packet[5] = static_cast<quint8>(data.size()); // 数据长度

    // 复制数据（最多8字节）
    for (int i = 0; i < data.size() && i < 8; i++) {
        packet[6 + i] = data[i];
    }

    // 计算LIN校验和
    quint8 pid = calPID(id);
    packet[14] = calculateChecksum(pid, data);

    // 计算并设置包CRC
    packet[15] = calculatePacketCRC(packet.left(15));

    qint64 bytesWritten = m_serialPort->write(packet);
    if (bytesWritten == -1) {
        setError(WriteError, "发送消息失败");
        return false;
    }

    if (!m_serialPort->waitForBytesWritten(1000)) {
        setError(TimeoutError, "发送超时");
        return false;
    }

    m_framesSent++;
    /*
    qDebug() << QString("发送帧成功 - ID: 0x%1, PID: 0x%2, 数据: %3")
                    .arg(id, 2, 16, QChar('0'))
                    .arg(pid, 2, 16, QChar('0'));
                    //.arg(data.toHex(' ').toUpper());
    qDebug() << "=== 发送数据调试 ===";
    qDebug() << "目标ID:" << QString("0x%1").arg(id, 2, 16, QChar('0'));
    qDebug() << "发送数据:" << data.toHex(' ');
    qDebug() << "构建的16字节包:" << packet.toHex(' ');*/
    return true;
}

bool MyLinInterface_HW_MI::requestFrame(quint8 id, int expectedDataLength)
{
    QMutexLocker locker(&m_mutex);

    if (!isConnected()) {
        setError(PortOpenError, "设备未连接");
        return false;
    }

    if (expectedDataLength < 0 || expectedDataLength > 8) {
        expectedDataLength = 8;
    }

    QByteArray packet;
    packet.resize(PACKET_SIZE);
    packet.fill(0x00);

    // 读取从机数据命令格式
    packet[0] = 0x33;                    // 命令头
    packet[1] = 0x01;                    // 读取通道
    packet[2] = id;                      // ID地址
    packet[3] = 0x01;                    // LIN数据传输方向
    packet[4] = EnhancedChecksum;        // 校验和类型
    packet[5] = static_cast<quint8>(expectedDataLength); // 期望数据长度

    // 计算并设置包CRC
    packet[15] = calculatePacketCRC(packet.left(15));

    qint64 bytesWritten = m_serialPort->write(packet);
    if (bytesWritten == -1) {
        setError(WriteError, "发送请求失败");
        return false;
    }

    if (!m_serialPort->waitForBytesWritten(1000)) {
        setError(TimeoutError, "发送超时");
        return false;
    }

    // 设置等待响应状态
    m_waitingForResponse = true;
    m_expectedFrameId = id;
    m_expectedDataLength = expectedDataLength;
    m_timeoutTimer->start(m_timeout);

    m_framesSent++;
    qDebug() << QString("发送请求帧 - ID: 0x%1, 期望长度: %2")
                    .arg(id, 2, 16, QChar('0'))
                    .arg(expectedDataLength);

    return true;
}

QStringList MyLinInterface_HW_MI::getAvailablePorts() const
{
    QStringList ports;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        ports << info.portName();
    }
    return ports;
}

quint8 MyLinInterface_HW_MI::calPID(quint8 id)
{
    // 计算保护标识符 (带奇偶校验位)
    quint8 pid = id & 0x3F;  // 取低6位

    // 计算奇偶校验位
    quint8 p0 = (pid ^ (pid >> 1) ^ (pid >> 2) ^ (pid >> 4)) & 0x01;
    quint8 p1 = ~((pid >> 1) ^ (pid >> 3) ^ (pid >> 4) ^ (pid >> 5)) & 0x01;

    pid |= (p0 << 6) | (p1 << 7);
    return pid;
}

quint8 MyLinInterface_HW_MI::calculateChecksum(quint8 pid, const QByteArray& data)
{
    // 增强型校验和包含PID
    quint16 sum = pid;
    for (char byte : data) {
        sum += static_cast<quint8>(byte);
    }
    while (sum >> 8) {
        sum = (sum & 0xFF) + (sum >> 8);
    }
    return (~sum) & 0xFF;
}

quint8 MyLinInterface_HW_MI::calculateChecksum_classic(const QByteArray& data)
{
    // 经典校验和不包含PID
    quint16 sum = 0;
    for (char byte : data) {
        sum += static_cast<quint8>(byte);
    }
    while (sum >> 8) {
        sum = (sum & 0xFF) + (sum >> 8);
    }
    return (~sum) & 0xFF;
}

bool MyLinInterface_HW_MI::verifyChecksum(quint8 pid, const QByteArray& data, quint8 receivedChecksum)
{
    quint8 calculatedChecksum = calculateChecksum(pid, data);
    return calculatedChecksum == receivedChecksum;
}

quint8 MyLinInterface_HW_MI::calculatePacketCRC(const QByteArray& data) const
{
    // LINTest-MI协议CRC：前15字节相加，取反加1
    quint16 sum = 0;
    for (char byte : data) {
        sum += static_cast<quint8>(byte);
    }
    return static_cast<quint8>((~sum) + 1);
}

void MyLinInterface_HW_MI::setState(LinState state)
{
    if (m_state != state) {
        LinState oldState = m_state;
        m_state = state;
        qDebug() << QString("状态变化: %1 -> %2").arg(oldState).arg(state);
        emit stateChanged(state);
    }
}

void MyLinInterface_HW_MI::setError(LinError error, const QString& errorString)
{
    m_lastError = error;
    m_errorString = errorString;
    qDebug() << "错误:" << errorString;
    emit errorOccurred(error, errorString);
}

void MyLinInterface_HW_MI::resetStatistics()
{
    m_framesSent = 0;
    m_framesReceived = 0;
    m_errorCount = 0;
}
