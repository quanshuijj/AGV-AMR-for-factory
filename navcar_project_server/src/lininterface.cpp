#include "navcar_project_server/lininterface.h"
#include <QSerialPort>
#include <QTimer>
#include <QDebug>
#include <QMutex>
#include <QThread>
LinInterface::LinInterface(QObject *parent)
    : QObject(parent)
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
    //m_serialPort=new QSerialPort(this);
    //m_timeoutTimer=new QTimer(this);
    // 连接信号
    connect(m_serialPort, &QSerialPort::readyRead,
            this, &LinInterface::handleReadyRead);
    connect(m_serialPort, QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::errorOccurred),
            this, &LinInterface::handleError);
    connect(m_timeoutTimer, &QTimer::timeout,
            this, &LinInterface::handleTimeout);

    m_timeoutTimer->setSingleShot(true);
}

LinInterface::~LinInterface()
{
    disconnectFromPort();
}

bool LinInterface::connectToPort(const QString& portName, qint32 baudRate)
{
    QMutexLocker locker(&m_mutex);

    if (m_state == Connected) {
        disconnectFromPort();
    }

    m_serialPort->setPortName(portName);
    m_serialPort->setBaudRate(baudRate);
    m_serialPort->setDataBits(QSerialPort::Data8);
    m_serialPort->setParity(QSerialPort::NoParity);
    m_serialPort->setStopBits(QSerialPort::OneStop);
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (m_serialPort->open(QIODevice::ReadWrite)) {
         m_baudRate = baudRate;
        setState(Connected);
        m_receiveBuffer.clear();
        resetStatistics();
        return true;
    } else {
        setError(PortOpenError, QString("无法打开串口: %1").arg(m_serialPort->errorString()));
        return false;
    }
}

void LinInterface::initLin1()
{
    QByteArray byteArray;
    byteArray.resize(15);
    byteArray[0]=0x5A;
    byteArray[1]=15;

}

void LinInterface::disconnectFromPort()
{
    QMutexLocker locker(&m_mutex);

    if (m_serialPort->isOpen()) {
        m_serialPort->close();
    }

    m_timeoutTimer->stop();
    m_waitingForResponse = false;
    setState(Disconnected);
}

bool LinInterface::isConnected() const
{
    return m_state == Connected;
}

QStringList LinInterface::getAvailablePorts() const
{
    QStringList ports;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos) {
        ports << info.portName();
    }
    return ports;
}

bool LinInterface::sendFrame(quint8 id, const QByteArray& data)
{
    if (data.size() > 8) {
        setError(FrameError, "数据长度不能超过8字节");
        return false;
    }

    LinFrame frame(id, data);
    return sendFrame(frame);
}

bool LinInterface::sendFrame(const LinFrame& frame)
{
    QMutexLocker locker(&m_mutex);

    if (m_state != Connected) {
        setError(PortNotFound, "串口未连接");
        return false;
    }

    setState(Transmitting);

    // 发送Break字段
    if (!writeBreakField()) {
        setError(WriteError, "发送Break字段失败");
        qDebug()<<"发送break失败";
        setState(Connected);
        return false;
    }

    // 发送Sync字段
    if (!writeSyncField()) {
        setError(WriteError, "发送Sync字段失败");
        setState(Connected);
        return false;
    }

    // 发送ID字段
    if (!writeIdentifierField(frame.id)) {
        setError(WriteError, "发送ID字段失败");
        qDebug()<<"失败3";
        setState(Connected);
        return false;
    }

    // 发送数据字段
    if (!writeDataField(frame.data)) {
        setError(WriteError, "发送数据字段失败");
        qDebug()<<"失败4";
        setState(Connected);
        return false;
    }

    // 发送校验和字段
    if (!writeChecksumField(frame.checksum)) {
        setError(WriteError, "发送校验和字段失败");
        qDebug()<<"失败5";
        setState(Connected);
        return false;
    }

    qDebug()<<"该帧checksum:"<<frame.checksum;

    m_framesSent++;
    setState(Connected);
    emit frameSent(frame.id);

    return true;
}

bool LinInterface::requestFrame(quint8 id, int expectedDataLength)
{
    QMutexLocker locker(&m_mutex);

    if (m_state != Connected) {
        setError(PortNotFound, "串口未连接");
        return false;
    }

    setState(Transmitting);

    // 发送Break字段
    if (!writeBreakField()) {
        setError(WriteError, "发送Break字段失败");
        setState(Connected);
        return false;
    }

    // 发送Sync字段
    if (!writeSyncField()) {
        setError(WriteError, "发送Sync字段失败");
        setState(Connected);
        return false;
    }

    // 发送ID字段
    if (!writeIdentifierField(id)) {
        setError(WriteError, "发送ID字段失败");
        setState(Connected);
        return false;
    }

    // 等待响应
    m_waitingForResponse = true;
    m_expectedFrameId = id;
    m_expectedDataLength = expectedDataLength;
    m_timeoutTimer->start(m_timeout);

    setState(Receiving);
    return true;
}

void LinInterface::setBaudRate(qint32 baudRate)
{
    m_baudRate = baudRate;
    if (m_serialPort->isOpen()) {
        m_serialPort->setBaudRate(baudRate);
    }
}

qint32 LinInterface::getBaudRate() const
{
    return m_baudRate;
}

void LinInterface::setTimeout(int ms)
{
    m_timeout = ms;
}

int LinInterface::getTimeout() const
{
    return m_timeout;
}

LinInterface::LinState LinInterface::getState() const
{
    return m_state;
}

LinInterface::LinError LinInterface::getLastError() const
{
    return m_lastError;
}

QString LinInterface::getErrorString() const
{
    return m_errorString;
}

void LinInterface::resetStatistics()
{
    m_framesSent = 0;
    m_framesReceived = 0;
    m_errorCount = 0;
}

void LinInterface::handleReadyRead()
{
    QByteArray data = m_serialPort->readAll();
    m_receiveBuffer.append(data);
    qDebug()<<"读取到数据";
    processReceivedData();
}

void LinInterface::handleError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError) {
        setError(ReadError, QString("串口错误: %1").arg(m_serialPort->errorString()));
        m_errorCount++;
    }
}

void LinInterface::handleTimeout()
{
    if (m_waitingForResponse) {
        m_waitingForResponse = false;
        setError(TimeoutError, "等待响应超时");
        setState(Connected);
        m_errorCount++;
    }
}

void LinInterface::setState(LinState state)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(state);
    }
}

void LinInterface::setError(LinError error, const QString& errorString)
{
    m_lastError = error;
    m_errorString = errorString;
    emit errorOccurred(error, errorString);
}

bool LinInterface::writeBreakField()
{
    // 发送Break字段 (至少13位的低电平)
    // 这里使用串口的break信号
    if (m_serialPort->setBreakEnabled(true)) {
        QThread::msleep(1);  // 等待1ms
        return m_serialPort->setBreakEnabled(false);
    }
    return false;
}

bool LinInterface::writeSyncField()
{
    QByteArray syncData;
    syncData.append(LIN_SYNC_BYTE);
    return m_serialPort->write(syncData) == syncData.size();
}

bool LinInterface::writeIdentifierField(quint8 id)
{
    // 计算保护标识符 (带奇偶校验位)
    quint8 pid = id & 0x3F;  // 取低6位

    // 计算奇偶校验位
    quint8 p0 = (pid ^ (pid >> 1) ^ (pid >> 2) ^ (pid >> 4)) & 0x01;
    quint8 p1 = ~((pid >> 1) ^ (pid >> 3) ^ (pid >> 4) ^ (pid >> 5)) & 0x01;

    pid |= (p0 << 6) | (p1 << 7);

    QByteArray idData;
    idData.append(pid);
    qDebug()<<"pid"<<pid;
    return m_serialPort->write(idData) == idData.size();
}

bool LinInterface::writeDataField(const QByteArray& data)
{
    if (data.isEmpty()) {
        return true;
    }
    return m_serialPort->write(data) == data.size();
}

bool LinInterface::writeChecksumField(quint8 checksum)
{
    QByteArray checksumData;
    checksumData.append(checksum);
    return m_serialPort->write(checksumData) == checksumData.size();
}

void LinInterface::processReceivedData()
{
    if (m_receiveBuffer.size() < 3) {  // 至少需要ID + 数据 + 校验和
        return;
    }
    qDebug()<<"收到数据";

    // 接收报文处理，转发到上层处理函数
    if (m_waitingForResponse) {
        LinFrame frame = parseFrame(m_receiveBuffer);
        qDebug()<<"完整报文收到";
        emit frameReceivedSignal(frame.id,frame.data);
        if (frame.isValid && frame.id == m_expectedFrameId) {
            m_timeoutTimer->stop();
            m_waitingForResponse = false;
            m_framesReceived++;
            setState(Connected);
            emit frameReceivedSignal(frame.id,frame.data);
            emit frameReceived(frame);
        }
    }
}

LinFrame LinInterface::parseFrame(const QByteArray& data)
{
    LinFrame frame;

    if (data.size() < 2) {
        return frame;
    }

    // 查找同步字节
    int syncIndex = data.indexOf(LIN_SYNC_BYTE);
    if (syncIndex < 0 || syncIndex + 2 >= data.size()) {
        return frame;
    }

    // 解析ID
    quint8 pid = static_cast<quint8>(data[syncIndex + 1]);
    frame.id = pid & 0x3F;  // 取低6位

    // 解析数据 (假设剩余字节为数据和校验和)
    int dataStart = syncIndex + 2;
    int dataLength = qMin(data.size() - dataStart - 1, m_expectedDataLength);
    if (dataLength > 0) {
        frame.data = data.mid(dataStart, dataLength);
    }

    // 解析校验和
    if (dataStart + dataLength < data.size()) {
        frame.checksum = static_cast<quint8>(data[dataStart + dataLength]);
        frame.isValid = frame.verifyChecksum();
    }

    return frame;
}

QByteArray LinInterface::createFrameBytes(const LinFrame& frame)
{
    QByteArray frameBytes;
    frameBytes.append(LIN_SYNC_BYTE);
    frameBytes.append(frame.id);
    frameBytes.append(frame.data);
    frameBytes.append(frame.checksum);
    return frameBytes;
}
