#include "navcar_project_server/mylininterface.h"

MyLinInterface::MyLinInterface(QObject *parent)
    : QObject{parent}
    , m_serialPort(new QSerialPort(this))
    , m_timeoutTimer(new QTimer(this))
    , m_state(Disconnected)
    , m_lastError(NoError)
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
            this, &MyLinInterface::handleReadyRead);
    connect(m_serialPort, QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::errorOccurred),
            this, &MyLinInterface::handleError);
    connect(m_timeoutTimer, &QTimer::timeout,
            this, &MyLinInterface::handleTimeout);

    // 设置超时定时器为单次触发
    m_timeoutTimer->setSingleShot(true);
}

MyLinInterface::~MyLinInterface()
{
    disconnectFromPort();
}

bool MyLinInterface::connectToPort(const QString& portName, qint32 baudRate)
{
    QMutexLocker locker(&m_mutex);

    if (m_state == Connected) {
        disconnectFromPort();
    }

    m_serialPort->setPortName(portName);
    m_serialPort->setBaudRate(115200);  // 与串口调试器通信的波特率
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

    if (!initLINSetting(baudRate)) {
        m_serialPort->close();
        setError(PortOpenError, QString("无法配置LIN参数: %1").arg(m_serialPort->errorString()));
        return false;
    }

    setState(Connected);
    qDebug() << "配置参数成功";
    return true;
}

void MyLinInterface::disconnectFromPort()
{
    QMutexLocker locker(&m_mutex);

    if (m_serialPort->isOpen()) {
        m_serialPort->close();
    }

    m_timeoutTimer->stop();
    m_waitingForResponse = false;
    m_receiveBuffer.clear();
    setState(Disconnected);
}

bool MyLinInterface::isConnected() const
{
    return m_state == Connected;
}

MyLinInterface::LinState MyLinInterface::getState() const
{
    return m_state;
}

MyLinInterface::LinError MyLinInterface::getLastError() const
{
    return m_lastError;
}

QString MyLinInterface::getErrorString() const
{
    return m_errorString;
}

void MyLinInterface::handleReadyRead()
{
    QMutexLocker locker(&m_mutex);

    QByteArray data = m_serialPort->readAll();
    if (data.isEmpty()) {
        return;
    }

    m_receiveBuffer.append(data);
    qDebug() << "读取到数据:" << data.toHex(' ');

    processReceivedData();
}

void MyLinInterface::processReceivedData()
{
    const quint8 HEADER_BYTE1 = 0x5A;
    const quint8 HEADER_BYTE2 = 15;
    const int PACKET_SIZE = 15;
    const int MIN_BUFFER_SIZE = 2; // 至少需要2个字节来检查包头

    while (m_receiveBuffer.size() >= MIN_BUFFER_SIZE) {
        // 查找包头 0x5A 15
        int headerIndex = findPacketHeader(HEADER_BYTE1, HEADER_BYTE2);

        if (headerIndex == -1) {
            // 未找到包头，清除缓冲区（保留最后一个字节，防止包头被分割）
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
            qDebug() << QString("数据包不完整，当前%1字节，需要%2字节").arg(m_receiveBuffer.size()).arg(PACKET_SIZE);
            break; // 等待更多数据
        }

        // 提取完整数据包
        QByteArray packet = m_receiveBuffer.left(PACKET_SIZE);

        // 验证数据包
        if (validatePacket(packet)) {
            // 处理有效数据包
            handleValidPacket(packet);
            m_framesReceived++;

            // 如果正在等待响应，停止超时定时器
            if (m_waitingForResponse) {
                m_timeoutTimer->stop();
                m_waitingForResponse = false;
            }
        } else {
            // 无效数据包，增加错误计数
            m_errorCount++;
            qDebug() << "数据包校验失败:" << packet.toHex(' ');
        }
        // 移除已处理的数据包
        m_receiveBuffer.remove(0, PACKET_SIZE);
    }
}

int MyLinInterface::findPacketHeader(quint8 byte1, quint8 byte2) const
{
    for (int i = 0; i <= m_receiveBuffer.size() - 2; i++) {
        if (static_cast<quint8>(m_receiveBuffer.at(i)) == byte1 &&
            static_cast<quint8>(m_receiveBuffer.at(i + 1)) == byte2) {
            return i;
        }
    }
    return -1;
}

bool MyLinInterface::validatePacket(const QByteArray& packet) const
{
    if (packet.size() != 15) {
        return false;
    }

    // 验证包头
    if (static_cast<quint8>(packet.at(0)) != 0x5A ||
        static_cast<quint8>(packet.at(1)) != 15) {
        return false;
    }

    // 计算并验证校验和（前14字节）
    quint8 calculatedChecksum = 0;
    for (int i = 0; i < 14; i++) {
        calculatedChecksum += static_cast<quint8>(packet.at(i));
    }

    quint8 receivedChecksum = static_cast<quint8>(packet.at(14));
    if (calculatedChecksum != receivedChecksum) {
        qDebug() << QString("校验和错误: 计算值=0x%1, 接收值=0x%2")
                        .arg(calculatedChecksum, 2, 16, QChar('0'))
                        .arg(receivedChecksum, 2, 16, QChar('0'));
        return false;
    }

    return true;
}

void MyLinInterface::handleValidPacket(const QByteArray& packet)
{
    // 提取有效负载（跳过包头，取中间的数据部分）
    QByteArray payload = packet.mid(2, 12);

    qDebug() << "接收到有效数据包:" << packet.toHex(' ');
    qDebug() << "有效负载:" << payload.toHex(' ');
    quint8 id=packet.at(2);
    quint8 len=packet.at(3);
    QByteArray data=packet.mid(4,4+len);
    quint8 originalId = id & 0x3F;
    // 发送数据接收信号
    emit dataReceived(packet, payload);
    emit frameReceivedSignal(originalId,data);
}

void MyLinInterface::handleError(QSerialPort::SerialPortError error)
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

void MyLinInterface::handleTimeout()
{
    if (m_waitingForResponse) {
        m_waitingForResponse = false;
        m_errorCount++;
        setError(TimeoutError, "等待响应超时");
        qDebug() << "响应超时";
        emit responseTimeout();
    }
}

bool MyLinInterface::initLINSetting(qint32 baudRate)
{
    QByteArray byteArray;
    byteArray.resize(15);
    byteArray[0] = 0x5A;
    byteArray[1] = 15;
    byteArray[2] = 0x00;  // LIN 主机
    byteArray[3] = 0x03;  // LIN 波特率设置命令

    int timeout = DEFAULT_TIMEOUT;
    byteArray[4] = (timeout >> 24) & 0xFF;
    byteArray[5] = (timeout >> 16) & 0xFF;
    byteArray[6] = (timeout >> 8) & 0xFF;
    byteArray[7] = timeout & 0xFF;

    if (baudRate > 20000 || baudRate < 2000) {
        qDebug() << "波特率设置不在合理范围: " << baudRate;
        return false;
    }

    byteArray[8] = (baudRate >> 24) & 0xFF;
    byteArray[9] = (baudRate >> 16) & 0xFF;
    byteArray[10] = (baudRate >> 8) & 0xFF;
    byteArray[11] = baudRate & 0xFF;
    byteArray[12] = 1;
    byteArray[13] = 0xF0;

    // 计算校验和
    byteArray[14] = 0;
    quint8 checksum = 0;
    for (int i = 0; i < 14; i++) {
        checksum += static_cast<quint8>(byteArray[i]);
    }
    byteArray[14] = checksum;

    qint64 bytesWritten = m_serialPort->write(byteArray);
    if (bytesWritten == -1) {
        qDebug() << "配置失败: 写入错误";
        return false;
    }

    if (!m_serialPort->waitForBytesWritten(1000)) {
        qDebug() << "配置失败: 写入超时";
        return false;
    }

    qDebug() << "LIN配置数据已发送:" << byteArray.toHex(' ');
    return true;
}

bool MyLinInterface::sendFrame(quint8 id, QByteArray data)
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

    QByteArray sendMsg;
    sendMsg.resize(15);
    sendMsg[0] = 0x5A;
    sendMsg[1] = 15;

    quint8 pid = calPID(id);
    quint8 len = static_cast<quint8>(data.size());

    sendMsg[2] = pid;
    sendMsg[3] = len;

    // 复制数据
    for (int i = 0; i < data.size(); i++) {
        sendMsg[4 + i] = data[i];
    }

    // 不足8字节的部分补零
    for (int i = data.size(); i < 8; i++) {
        sendMsg[4 + i] = 0x00;
    }

    // 计算LIN校验和（只对实际数据计算，不包括补零部分）
    //sendMsg[12]=calculateChecksum_classic(pid,data);
    sendMsg[12] = calculateChecksum(pid, data);
    sendMsg[13] = 0x00;

    // 计算包校验和
    sendMsg[14] = 0;
    quint8 checksum = 0;
    for (int i = 0; i < 14; i++) {
        checksum += static_cast<quint8>(sendMsg[i]);
    }
    sendMsg[14] = checksum;

    qint64 bytesWritten = m_serialPort->write(sendMsg);
    if (bytesWritten == -1) {
        setError(WriteError, "发送消息失败");
        return false;
    }

    if (!m_serialPort->waitForBytesWritten(1000)) {
        setError(TimeoutError, "发送超时");
        return false;
    }

    m_framesSent++;
    qDebug() << QString("发送帧成功 - ID: 0x%1, PID: 0x%2, 数据: %3")
                    .arg(QString::number(id, 16).rightJustified(2, '0'))  // 手动补零
                    .arg(QString::number(pid, 16).rightJustified(2, '0'))
                    .arg(QString(data.toHex(' ').toUpper()));                      // 转大写确保格式一致
    /*
    qDebug() << QString("发送帧成功 - ID: 0x%1, PID: 0x%2, 数据: %3")
                    .arg(id, 2, 16, QChar('0'))
                    .arg(pid, 2, 16, QChar('0'))
                    .arg(data.toHex(' '));
*/
    return true;
}

bool MyLinInterface::requestFrame(quint8 id, int expectedDataLength)
{
    QMutexLocker locker(&m_mutex);

    if (!isConnected()) {
        setError(PortOpenError, "设备未连接");
        return false;
    }

    if (expectedDataLength < 0 || expectedDataLength > 8) {
        expectedDataLength = 8;
    }

    QByteArray sendMsg;
    sendMsg.resize(15);
    sendMsg[0] = 0x5A;
    sendMsg[1] = 15;

    quint8 pid = calPID(id);
    sendMsg[2] = pid;
    sendMsg[3] = static_cast<quint8>(expectedDataLength);

    // 请求帧没有数据，全部填零
    for (int i = 4; i < 12; i++) {
        sendMsg[i] = 0x00;
    }

    // 对于请求帧，校验和通常为0或特殊值
    sendMsg[12] = 0x00;
    sendMsg[13] = 0x01;

    // 计算包校验和
    sendMsg[14] = 0;
    quint8 checksum = 0;
    for (int i = 0; i < 14; i++) {
        checksum += static_cast<quint8>(sendMsg[i]);
    }
    sendMsg[14] = checksum;

    qint64 bytesWritten = m_serialPort->write(sendMsg);
    if (bytesWritten == -1) {
        qDebug()<<"发送请求失败";
        setError(WriteError, "发送请求失败");
        return false;
    }

    if (!m_serialPort->waitForBytesWritten(1000)) {
        qDebug()<<"发送失败";
        setError(TimeoutError, "发送超时");
        return false;
    }

    // 设置等待响应状态
    m_waitingForResponse = true;
    m_expectedFrameId = id;
    m_expectedDataLength = expectedDataLength;
    m_timeoutTimer->start(m_timeout);

    m_framesSent++;
    qDebug() << QString("发送请求帧 - ID: 0x%1, PID: 0x%2, 期望长度: %3")
                    .arg(id, 2, 16, QChar('0'))
                    .arg(pid, 2, 16, QChar('0'))
                    .arg(expectedDataLength);

    return true;
}

QStringList MyLinInterface::getAvailablePorts() const
{
    QStringList ports;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        ports << info.portName();
    }
    return ports;
}

quint8 MyLinInterface::calPID(quint8 id)
{
    // 计算保护标识符 (带奇偶校验位)
    quint8 pid = id & 0x3F;  // 取低6位

    // 计算奇偶校验位
    quint8 p0 = (pid ^ (pid >> 1) ^ (pid >> 2) ^ (pid >> 4)) & 0x01;
    quint8 p1 = ~((pid >> 1) ^ (pid >> 3) ^ (pid >> 4) ^ (pid >> 5)) & 0x01;

    pid |= (p0 << 6) | (p1 << 7);
    return pid;
}

quint8 MyLinInterface::calculateChecksum(quint8 pid, const QByteArray& data)
{
    quint16 sum = pid;  // 增强型校验和包含PID
    for (char byte : data) {
        sum += static_cast<quint8>(byte);
    }
    while (sum >> 8) {  // ✅ 正确的进位处理
        sum = (sum & 0xFF) + (sum >> 8);
    }
    return ~sum & 0xFF;  // ✅ 取反
    //return checksum;
}

quint8 MyLinInterface::calculateChecksum_classic(quint8 pid, const QByteArray& data)
{
    quint16 sum = 0;
    for (char byte : data) {
        sum += static_cast<quint8>(byte);
    }
    while (sum >> 8) {
        sum = (sum & 0xFF) + (sum >> 8);
    }
    quint8 checksum = (~sum) & 0xFF;  // 取反
    return checksum;
}

bool MyLinInterface::verifyChecksum(quint8 pid, const QByteArray& data, quint8 receivedChecksum)
{
    quint8 calculatedChecksum = calculateChecksum(pid, data);
    return calculatedChecksum == receivedChecksum;
}

void MyLinInterface::setState(LinState state)
{
    if (m_state != state) {
        LinState oldState = m_state;
        m_state = state;
        qDebug() << QString("状态变化: %1 -> %2").arg(oldState).arg(state);
        emit stateChanged(state);
    }
}

void MyLinInterface::setError(LinError error, const QString& errorString)
{
    m_lastError = error;
    m_errorString = errorString;
    qDebug() << "错误:" << errorString;
    emit errorOccurred(error, errorString);
}

void MyLinInterface::resetStatistics()
{
    m_framesSent = 0;
    m_framesReceived = 0;
    m_errorCount = 0;
}
