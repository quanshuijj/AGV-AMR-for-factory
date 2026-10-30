#include "navcar_project_server/socketcanmanager.h"
#include <QDateTime>
#include <QMutexLocker>
#include <QProcess>
#include <QRegularExpression>
#include <QDir>
#include <QTextStream>
#include <QDebug>

SocketCanManager::SocketCanManager(QObject* parent)
    : QObject(parent)
    , m_socketFd(-1)
    , m_socketNotifier(nullptr)
    , m_loopbackEnabled(true)
    , m_receiveOwnMessages(false)
    , m_nonBlocking(true)
    , m_errorMask(CAN_ERR_MASK)
    , m_currentState(STATE_CLOSED)
{
}

SocketCanManager::~SocketCanManager()
{
    closeInterface();
}

bool SocketCanManager::openInterface(const QString& interfaceName)
{
    if (m_socketFd >= 0) {
        setLastError("Interface already opened");
        return false;
    }

    // 创建socket
    m_socketFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socketFd < 0) {
        setLastError(QString("Failed to create socket: %1").arg(strerror(errno)));
        return false;
    }

    // 设置socket选项
    int loopback = m_loopbackEnabled ? 1 : 0;
    setsockopt(m_socketFd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    int recv_own_msgs = m_receiveOwnMessages ? 1 : 0;
    setsockopt(m_socketFd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

    // 设置错误过滤器
    setsockopt(m_socketFd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &m_errorMask, sizeof(m_errorMask));

    // 绑定到指定接口
    struct sockaddr_can addr;
    struct ifreq ifr;

    strcpy(ifr.ifr_name, interfaceName.toLocal8Bit().data());
    if (ioctl(m_socketFd, SIOCGIFINDEX, &ifr) < 0) {
        setLastError(QString("Interface %1 not found: %2").arg(interfaceName).arg(strerror(errno)));
        close(m_socketFd);
        m_socketFd = -1;
        return false;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(m_socketFd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        setLastError(QString("Failed to bind to interface %1: %2").arg(interfaceName).arg(strerror(errno)));
        close(m_socketFd);
        m_socketFd = -1;
        return false;
    }

    // 设置非阻塞模式
    if (m_nonBlocking) {
        int flags = fcntl(m_socketFd, F_GETFL, 0);
        fcntl(m_socketFd, F_SETFL, flags | O_NONBLOCK);
    }

    // 创建socket通知器用于异步接收
    m_socketNotifier = new QSocketNotifier(m_socketFd, QSocketNotifier::Read, this);
    connect(m_socketNotifier, &QSocketNotifier::activated, this, &SocketCanManager::onSocketReadyRead);
    m_socketNotifier->setEnabled(true);

    m_interfaceName = interfaceName;
    m_currentState = STATE_OPENED;
    clearStatistics();

    qDebug() << QString("CAN interface %1 opened successfully").arg(interfaceName);
    emit interfaceStateChanged(m_currentState);

    return true;
}

bool SocketCanManager::closeInterface()
{
    if (m_socketFd < 0) {
        return true;
    }

    if (m_socketNotifier) {
        m_socketNotifier->setEnabled(false);
        delete m_socketNotifier;
        m_socketNotifier = nullptr;
    }

    close(m_socketFd);
    m_socketFd = -1;
    m_interfaceName.clear();
    m_currentState = STATE_CLOSED;
    m_filters.clear();
    m_receiveBuffer.clear();

    qDebug() << "CAN interface closed";
    emit interfaceStateChanged(m_currentState);

    return true;
}

QStringList SocketCanManager::getAvailableInterfaces()
{
    QStringList interfaces;

    // 读取/proc/net/dev文件
    QFile file("/proc/net/dev");
    if (file.open(QIODevice::ReadOnly)) {
        QTextStream stream(&file);
        QString line;

        // 跳过前两行标题
        stream.readLine();
        stream.readLine();

        while (stream.readLineInto(&line)) {
            QStringList parts;
#if QT_VERSION >= QT_VERSION_CHECK(5,14,0)
            parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
#else
            parts = line.split(QRegularExpression("\\s+"), QString::SkipEmptyParts);
#endif
            if (parts.size() >= 2) {
                QString ifName = parts[0].trimmed();
                // 检查是否是CAN接口
                if (ifName.startsWith("can") || ifName.startsWith("vcan")) {
                    interfaces.append(ifName);
                }
            }
        }
    }

    // 也可以通过检查/sys/class/net目录
    QDir netDir("/sys/class/net");
    if (netDir.exists()) {
        QStringList entries = netDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
        for (const QString& entry : entries) {
            if ((entry.startsWith("can") || entry.startsWith("vcan")) && !interfaces.contains(entry)) {
                // 检查是否是CAN设备
                QFile typeFile(QString("/sys/class/net/%1/type").arg(entry));
                if (typeFile.open(QIODevice::ReadOnly)) {
                    QString type = typeFile.readAll().trimmed();
                    // CAN设备的type通常是280 (ARPHRD_CAN)
                    if (type == "280") {
                        interfaces.append(entry);
                    }
                }
            }
        }
    }

    interfaces.removeDuplicates();
    interfaces.sort();
    return interfaces;
}

bool SocketCanManager::configureInterface(const QString& interface, CanSpeed speed)
{
    QProcess process;

    // 先关闭接口
    process.start("ip", QStringList() << "link" << "set" << interface << "down");
    process.waitForFinished(3000);

    // 配置波特率
    QStringList args;
    args << "link" << "set" << interface << "type" << "can" << "bitrate" << QString::number(speed);

    process.start("ip", args);
    process.waitForFinished(3000);

    if (process.exitCode() != 0) {
        qWarning() << "Failed to configure interface" << interface
                   << ":" << process.readAllStandardError();
        return false;
    }

    return true;
}

bool SocketCanManager::setInterfaceUp(const QString& interface)
{
    QProcess process;
    process.start("ip", QStringList() << "link" << "set" << interface << "up");
    process.waitForFinished(3000);

    if (process.exitCode() != 0) {
        qWarning() << "Failed to bring up interface" << interface
                   << ":" << process.readAllStandardError();
        return false;
    }

    return true;
}

bool SocketCanManager::setInterfaceDown(const QString& interface)
{
    QProcess process;
    process.start("ip", QStringList() << "link" << "set" << interface << "down");
    process.waitForFinished(3000);

    return process.exitCode() == 0;
}

SocketCanManager::CanState SocketCanManager::getInterfaceState(const QString& interface)
{
    QFile stateFile(QString("/sys/class/net/%1/operstate").arg(interface));
    if (stateFile.open(QIODevice::ReadOnly)) {
        QString state = stateFile.readAll().trimmed().toLower();
        if (state == "up") {
            return STATE_OPENED;
        }
        else if (state == "down") {
            return STATE_CLOSED;
        }
    }

    // 检查CAN特定状态
    QFile canStateFile(QString("/sys/class/net/%1/can_state").arg(interface));
    if (canStateFile.open(QIODevice::ReadOnly)) {
        QString canState = canStateFile.readAll().trimmed().toLower();
        if (canState == "error-active") {
            return STATE_ERROR_ACTIVE;
        }
        else if (canState == "error-warning") {
            return STATE_ERROR_WARNING;
        }
        else if (canState == "error-passive") {
            return STATE_ERROR_PASSIVE;
        }
        else if (canState == "bus-off") {
            return STATE_BUS_OFF;
        }
        else if (canState == "stopped") {
            return STATE_STOPPED;
        }
        else if (canState == "sleeping") {
            return STATE_SLEEPING;
        }
    }

    return STATE_CLOSED;
}

bool SocketCanManager::sendMessage(const CanMessage& message)
{
    if (m_socketFd < 0) {
        setLastError("Interface not opened");
        return false;
    }

    struct can_frame frame = convertToCanFrame(message);

    ssize_t nbytes = write(m_socketFd, &frame, sizeof(frame));

    if (nbytes != sizeof(frame)) {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.txErrorCount++;
        setLastError(QString("Failed to send message: %1").arg(strerror(errno)));
        return false;
    }

    QMutexLocker locker(&m_statsMutex);
    m_statistics.txCount++;

    return true;
}

bool SocketCanManager::sendMessages(const QList<CanMessage>& messages)
{
    if (messages.isEmpty()) {
        return true;
    }

    int successCount = 0;
    for (const CanMessage& msg : messages) {
        if (sendMessage(msg)) {
            successCount++;
        }
    }

    return successCount == messages.size();
}

QList<CanMessage> SocketCanManager::receiveMessages(int maxCount)
{
    QList<CanMessage> messages;

    if (m_socketFd < 0) {
        setLastError("Interface not opened");
        return messages;
    }

    // 先从缓冲区获取
    while (!m_receiveBuffer.isEmpty() && messages.size() < maxCount) {
        messages.append(m_receiveBuffer.dequeue());
    }

    if (messages.size() >= maxCount) {
        return messages;
    }

    // 直接从socket读取
    struct can_frame frame;
    ssize_t nbytes;

    while (messages.size() < maxCount) {
        nbytes = read(m_socketFd, &frame, sizeof(frame));

        if (nbytes != sizeof(frame)) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 非阻塞模式下没有更多数据
                break;
            }
            else {
                QMutexLocker locker(&m_statsMutex);
                m_statistics.rxErrorCount++;
                setLastError(QString("Failed to receive message: %1").arg(strerror(errno)));
                break;
            }
        }

        if (frame.can_id & CAN_ERR_FLAG) {
            // 错误帧
            CanMessage errorMsg = convertFromCanFrame(frame);
            errorMsg.isError = true;
            processErrorFrame(frame);
            emit errorFrameReceived(errorMsg);

            QMutexLocker locker(&m_statsMutex);
            m_statistics.errorFrameCount++;
        }
        else {
            // 正常数据帧
            CanMessage msg = convertFromCanFrame(frame);
            messages.append(msg);

            QMutexLocker locker(&m_statsMutex);
            m_statistics.rxCount++;
        }
    }
    return messages;
}

bool SocketCanManager::setFilters(const QList<CanFilter>& filters)
{
    if (m_socketFd < 0) {
        setLastError("Interface not opened");
        return false;
    }

    if (filters.isEmpty()) {
        return removeFilters();
    }

    // 转换为Socket CAN过滤器格式
    QVector<struct can_filter> rawFilters;
    for (const CanFilter& filter : filters) {
        struct can_filter rawFilter;
        rawFilter.can_id = filter.id;
        rawFilter.can_mask = filter.mask;

        if (filter.isExtended) {
            rawFilter.can_id |= CAN_EFF_FLAG;
            rawFilter.can_mask |= CAN_EFF_FLAG;
        }

        rawFilters.append(rawFilter);
    }

    if (setsockopt(m_socketFd, SOL_CAN_RAW, CAN_RAW_FILTER,
                   rawFilters.data(), rawFilters.size() * sizeof(struct can_filter)) < 0) {
        setLastError(QString("Failed to set filters: %1").arg(strerror(errno)));
        return false;
    }

    m_filters = filters;
    qDebug() << QString("Applied %1 CAN filters").arg(filters.size());

    return true;
}

bool SocketCanManager::addFilter(const CanFilter& filter)
{
    QList<CanFilter> newFilters = m_filters;
    newFilters.append(filter);
    return setFilters(newFilters);
}

bool SocketCanManager::removeFilters()
{
    if (m_socketFd < 0) {
        setLastError("Interface not opened");
        return false;
    }

    // 移除所有过滤器（传递NULL指针）
    if (setsockopt(m_socketFd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) < 0) {
        setLastError(QString("Failed to remove filters: %1").arg(strerror(errno)));
        return false;
    }

    m_filters.clear();
    qDebug() << "All CAN filters removed";

    return true;
}

bool SocketCanManager::setLoopback(bool enable)
{
    m_loopbackEnabled = enable;

    if (m_socketFd >= 0) {
        int loopback = enable ? 1 : 0;
        if (setsockopt(m_socketFd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
            setLastError(QString("Failed to set loopback: %1").arg(strerror(errno)));
            return false;
        }
    }

    return true;
}

bool SocketCanManager::setReceiveOwnMessages(bool enable)
{
    m_receiveOwnMessages = enable;

    if (m_socketFd >= 0) {
        int recv_own_msgs = enable ? 1 : 0;
        if (setsockopt(m_socketFd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
            setLastError(QString("Failed to set receive own messages: %1").arg(strerror(errno)));
            return false;
        }
    }

    return true;
}

bool SocketCanManager::setErrorFilter(quint32 errorMask)
{
    m_errorMask = errorMask;

    if (m_socketFd >= 0) {
        if (setsockopt(m_socketFd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errorMask, sizeof(errorMask)) < 0) {
            setLastError(QString("Failed to set error filter: %1").arg(strerror(errno)));
            return false;
        }
    }

    return true;
}

bool SocketCanManager::setNonBlocking(bool enable)
{
    m_nonBlocking = enable;

    if (m_socketFd >= 0) {
        int flags = fcntl(m_socketFd, F_GETFL, 0);
        if (enable) {
            flags |= O_NONBLOCK;
        }
        else {
            flags &= ~O_NONBLOCK;
        }

        if (fcntl(m_socketFd, F_SETFL, flags) < 0) {
            setLastError(QString("Failed to set non-blocking mode: %1").arg(strerror(errno)));
            return false;
        }
    }

    return true;
}

SocketCanManager::Statistics SocketCanManager::getStatistics() const
{
    QMutexLocker locker(&m_statsMutex);
    return m_statistics;
}

void SocketCanManager::clearStatistics()
{
    QMutexLocker locker(&m_statsMutex);
    m_statistics = Statistics();
    emit statisticsChanged(m_statistics);
}

void SocketCanManager::onSocketReadyRead()
{
    struct can_frame frame;
    ssize_t nbytes;
    int readMsgCount=0;
    const int maxPerCount=20;
    // 直接从socket读取，不使用中间缓冲区
    while (readMsgCount<maxPerCount) {
        nbytes = read(m_socketFd, &frame, sizeof(frame));

        if (nbytes != sizeof(frame)) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 没有更多数据可读
                break;
            } else {
                QMutexLocker locker(&m_statsMutex);
                m_statistics.rxErrorCount++;
                setLastError(QString("Failed to receive message: %1").arg(strerror(errno)));
                break;
            }
        }
        readMsgCount++;

        if (frame.can_id & CAN_ERR_FLAG) {
            // 错误帧处理
            CanMessage errorMsg = convertFromCanFrame(frame);
            errorMsg.isError = true;
            processErrorFrame(frame);
            emit errorFrameReceived(errorMsg);

            QMutexLocker locker(&m_statsMutex);
            m_statistics.errorFrameCount++;
        } else {
            // 正常数据帧
            CanMessage msg = convertFromCanFrame(frame);

            // 只在缓冲区未满时才缓冲
            if (m_receiveBuffer.size() < MAX_BUFFER_SIZE) {
                m_receiveBuffer.enqueue(msg);
            } else {
                QMutexLocker locker(&m_statsMutex);
                m_statistics.droppedCount++;
            }

            // 发射信号和打印日志
            emit messageReceived(msg);

            QMutexLocker locker(&m_statsMutex);
            m_statistics.rxCount++;
        }
    }

    emit statisticsChanged(getStatistics());
    /*9.15修改版本
    QList<CanMessage> messages = receiveMessages(10);

    // 缓冲消息（如果缓冲区未满）
    for (const CanMessage& msg : messages) {
        if (m_receiveBuffer.size() < MAX_BUFFER_SIZE) {
            m_receiveBuffer.enqueue(msg);
        }
        else {
            QMutexLocker locker(&m_statsMutex);
            m_statistics.droppedCount++;
        }

        emit messageReceived(msg);
        qDebug()<<"收到CAN消息,id为:"<<msg.id;
    }

    if (!messages.isEmpty()) {
        emit messagesReceived(messages);
        emit statisticsChanged(getStatistics());
    }*/
}

void SocketCanManager::setLastError(const QString& error)
{
    m_lastError = error;
    emit errorOccurred(error);
    qWarning() << "SocketCanManager Error:" << error;
}

struct can_frame SocketCanManager::convertToCanFrame(const CanMessage& message)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    frame.can_id = message.id;
    frame.can_dlc = message.dataLen;

    if (message.isExtended) {
        frame.can_id |= CAN_EFF_FLAG;
    }

    if (message.isRemote) {
        frame.can_id |= CAN_RTR_FLAG;
    }

    memcpy(frame.data, message.data, message.dataLen);

    return frame;
}

CanMessage SocketCanManager::convertFromCanFrame(const struct can_frame& frame)
{
    CanMessage message;

    message.id = frame.can_id & CAN_EFF_MASK;
    message.dataLen = frame.can_dlc;
    message.isExtended = (frame.can_id & CAN_EFF_FLAG) != 0;
    message.isRemote = (frame.can_id & CAN_RTR_FLAG) != 0;
    message.isError = (frame.can_id & CAN_ERR_FLAG) != 0;
    message.timestamp = QDateTime::currentMSecsSinceEpoch();

    memcpy(message.data, frame.data, frame.can_dlc);

    return message;
}

void SocketCanManager::processErrorFrame(const struct can_frame& frame)
{
    quint32 errorClass = frame.can_id & CAN_ERR_MASK;
    QString description = getErrorDescription(errorClass);

    QMutexLocker locker(&m_errorMutex);
    m_recentErrors.append(CanError(errorClass, description));

    // 限制错误历史记录数量
    while (m_recentErrors.size() > MAX_ERROR_HISTORY) {
        m_recentErrors.removeFirst();
    }

    qWarning() << "CAN Error:" << description;
}

QString SocketCanManager::getErrorDescription(quint32 errorClass)
{
    QStringList errors;

    if (errorClass & CAN_ERR_TX_TIMEOUT) {
        errors << "TX timeout";
    }
    if (errorClass & CAN_ERR_LOSTARB) {
        errors << "Lost arbitration";
    }
    if (errorClass & CAN_ERR_CRTL) {
        errors << "Controller error";
    }
    if (errorClass & CAN_ERR_PROT) {
        errors << "Protocol violation";
    }
    if (errorClass & CAN_ERR_TRX) {
        errors << "Transceiver error";
    }
    if (errorClass & CAN_ERR_ACK) {
        errors << "ACK error";
    }
    if (errorClass & CAN_ERR_BUSOFF) {
        errors << "Bus off";
    }
    if (errorClass & CAN_ERR_BUSERROR) {
        errors << "Bus error";
    }
    if (errorClass & CAN_ERR_RESTARTED) {
        errors << "Controller restarted";
    }

    if (errors.isEmpty()) {
        return QString("Unknown error (0x%1)").arg(errorClass, 0, 16);
    }

    return errors.join(", ");
}

// CAN接口配置工具实现
QList<CanInterfaceConfig::InterfaceInfo> CanInterfaceConfig::getAllInterfaceInfo()
{
    QList<InterfaceInfo> interfaces;
    QStringList ifNames = SocketCanManager::getAvailableInterfaces();

    for (const QString& name : ifNames) {
        interfaces.append(getInterfaceInfo(name));
    }

    return interfaces;
}

CanInterfaceConfig::InterfaceInfo CanInterfaceConfig::getInterfaceInfo(const QString& interface)
{
    InterfaceInfo info;
    info.name = interface;
    info.isUp = false;
    info.speed = SocketCanManager::SPEED_500K;
    info.txQueueLen = 10;
    info.rxPackets = 0;
    info.txPackets = 0;
    info.rxErrors = 0;
    info.txErrors = 0;

    // 获取接口状态
    QFile operStateFile(QString("/sys/class/net/%1/operstate").arg(interface));
    if (operStateFile.open(QIODevice::ReadOnly)) {
        QString state = operStateFile.readAll().trimmed();
        info.isUp = (state == "up");
    }

    // 获取波特率
    QFile bitrateFile(QString("/sys/class/net/%1/can_bittiming").arg(interface));
    if (bitrateFile.open(QIODevice::ReadOnly)) {
        QString content = bitrateFile.readAll();
        QRegularExpression re("bitrate (\\d+)");
        QRegularExpressionMatch match = re.match(content);
        if (match.hasMatch()) {
            info.speed = static_cast<SocketCanManager::CanSpeed>(match.captured(1).toInt());
        }
    }

    // 获取驱动信息
    QFile driverFile(QString("/sys/class/net/%1/device/driver").arg(interface));
    if (driverFile.exists()) {
        QString driverPath = driverFile.symLinkTarget();
        info.driver = QFileInfo(driverPath).baseName();
    }

    // 获取队列长度
    QFile txQueueFile(QString("/sys/class/net/%1/tx_queue_len").arg(interface));
    if (txQueueFile.open(QIODevice::ReadOnly)) {
        info.txQueueLen = txQueueFile.readAll().trimmed().toUInt();
    }

    // 获取统计信息
    QFile statsFile(QString("/proc/net/dev"));
    if (statsFile.open(QIODevice::ReadOnly)) {
        QTextStream stream(&statsFile);
        QString line;

        while (stream.readLineInto(&line)) {
            if (line.contains(interface + ":")) {
                QStringList parts;
#if QT_VERSION >= QT_VERSION_CHECK(5,14,0)
                parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
#else
                parts = line.split(QRegularExpression("\\s+"), QString::SkipEmptyParts);
#endif

                if (parts.size() >= 17) {
                    info.rxPackets = parts[2].toULongLong();  // RX packets
                    info.rxErrors = parts[3].toULongLong();   // RX errors
                    info.txPackets = parts[10].toULongLong(); // TX packets
                    info.txErrors = parts[11].toULongLong();  // TX errors
                }
                break;
            }
        }
    }

    return info;
}

bool CanInterfaceConfig::createVirtualInterface(const QString& name)
{
    QProcess process;
    process.start("ip", QStringList() << "link" << "add" << "dev" << name << "type" << "vcan");
    process.waitForFinished(3000);

    if (process.exitCode() != 0) {
        qWarning() << "Failed to create virtual interface" << name
                   << ":" << process.readAllStandardError();
        return false;
    }

    return true;
}

bool CanInterfaceConfig::removeVirtualInterface(const QString& name)
{
    QProcess process;
    process.start("ip", QStringList() << "link" << "delete" << name);
    process.waitForFinished(3000);

    return process.exitCode() == 0;
}

bool CanInterfaceConfig::setTxQueueLength(const QString& interface, quint32 length)
{
    QProcess process;
    process.start("ip", QStringList() << "link" << "set" << interface << "txqueuelen" << QString::number(length));
    process.waitForFinished(3000);

    return process.exitCode() == 0;
}

bool CanInterfaceConfig::canDump(const QString& interface, int duration)
{
    QProcess process;
    QStringList args;
    args << "-T" << QString::number(duration * 1000) << interface;

    process.start("candump", args);
    process.waitForFinished((duration + 1) * 1000);

    if (process.exitCode() == 0) {
        qDebug() << "candump output:" << process.readAllStandardOutput();
        return true;
    }
    else {
        qWarning() << "candump failed:" << process.readAllStandardError();
        return false;
    }
}

bool CanInterfaceConfig::canSend(const QString& interface, const QString& canFrame)
{
    QProcess process;
    process.start("cansend", QStringList() << interface << canFrame);
    process.waitForFinished(3000);

    return process.exitCode() == 0;
}

QString CanInterfaceConfig::getCanUtilsVersion()
{
    QProcess process;
    process.start("candump", QStringList() << "--version");
    process.waitForFinished(3000);

    if (process.exitCode() == 0) {
        return process.readAllStandardOutput().trimmed();
    }

    return "Unknown";
}
