// networkmanager.cpp - 重构版本实现
#include "navcar_project_server/networkmanager.h"
#include "navcar_project_server/qt_version_compat.h"
#include <QApplication>
#include <QHostInfo>
#include <QNetworkInterface>

Q_LOGGING_CATEGORY(networkLog, "network")

NetworkManager::NetworkManager(QObject *parent)
    : QObject(parent)
    , m_tcpServer(nullptr)
    , m_udpSocket(nullptr)
    , m_clientSocket(nullptr)
    , m_h264CacheManager(nullptr)
    , m_h264FlushTimer(nullptr)
    , m_h264BufferSize(3)
    , m_networkThreadPool(nullptr)
    , m_nextClientId(1000)
    , m_serverState(ConnectionState::Disconnected)
    , m_clientState(ConnectionState::Disconnected)
    , m_protocol(NetworkProtocol::TCP)
    , m_serverPort(0)
    , m_heartbeatTimer(nullptr)
    , m_statusTimer(nullptr)
    , m_statisticsTimer(nullptr)
    , m_cleanupTimer(nullptr)
    , m_lastBytesTransferred(0)
    , m_packetSequenceNumber(0)
    , m_maxConnections(50)
    , m_connectionTimeoutMs(30000)
    , m_heartbeatEnabled(false)
    , m_statusBroadcastEnabled(false)
{
    // 初始化统计信息
    m_statistics = NetworkStatistics();
    m_statistics.startTime = QDateTime::currentDateTime();

    // 创建线程池
    m_networkThreadPool = new QThreadPool(this);
    m_networkThreadPool->setMaxThreadCount(2);

    // 创建定时器
    m_heartbeatTimer = new QTimer(this);
    m_statusTimer = new QTimer(this);
    m_statisticsTimer = new QTimer(this);
    m_cleanupTimer = new QTimer(this);
    m_h264FlushTimer = new QTimer(this);

    // 连接定时器信号
    connect(m_heartbeatTimer, &QTimer::timeout, this, &NetworkManager::onHeartbeatTimer);
    connect(m_statusTimer, &QTimer::timeout, this, &NetworkManager::onStatusTimer);
    connect(m_statisticsTimer, &QTimer::timeout, this, &NetworkManager::onStatisticsTimer);
    connect(m_cleanupTimer, &QTimer::timeout, this, &NetworkManager::onCleanupTimer);
    connect(m_h264FlushTimer, &QTimer::timeout, this, &NetworkManager::onH264FlushTimer);

    // 启动统计和清理定时器
    m_statisticsTimer->start(1000); // 每秒更新统计
    m_cleanupTimer->start(10000);   // 每10秒清理一次
    m_h264FlushTimer->start(33);    // 30fps刷新H264缓冲
}

NetworkManager::~NetworkManager()
{
    cleanup();
}

bool NetworkManager::initialize()
{
    logInfo("初始化网络管理器...");

    // 创建H264缓存管理器
    m_h264CacheManager = std::make_unique<H264CacheManager>(this);
    if (!m_h264CacheManager->initialize()) {
        logError("H264缓存管理器初始化失败");
        return false;
    }

    // 连接H264缓存管理器信号
    connect(m_h264CacheManager.get(), &H264CacheManager::keyFrameCached,
            this, &NetworkManager::onKeyFrameCached);
    connect(m_h264CacheManager.get(), &H264CacheManager::clientKeyFrameNeeded,
            this, &NetworkManager::onClientKeyFrameNeeded);
    connect(m_h264CacheManager.get(), &H264CacheManager::cacheStatsUpdated,
            this, &NetworkManager::onCacheStatsUpdated);

    // 启动运行时间计时器
    m_uptimeTimer.start();
    m_bandwidthTimer.start();

    logInfo("网络管理器初始化成功");
    return true;
}

void NetworkManager::cleanup()
{
    logInfo("清理网络管理器...");

    // 停止服务器
    stopServer();

    // 断开客户端连接
    disconnectFromHost();

    // 停止定时器
    if (m_heartbeatTimer) m_heartbeatTimer->stop();
    if (m_statusTimer) m_statusTimer->stop();
    if (m_statisticsTimer) m_statisticsTimer->stop();
    if (m_cleanupTimer) m_cleanupTimer->stop();
    if (m_h264FlushTimer) m_h264FlushTimer->stop();

    // 清理H264缓存管理器
    if (m_h264CacheManager) {
        m_h264CacheManager->cleanup();
        m_h264CacheManager.reset();
    }

    // 清理H264帧缓冲
    m_h264FrameBuffers.clear();

    // 清理客户端连接
    QMutexLocker locker(&m_clientsMutex);
    for (auto& client : m_clients) {
        if (client && client->tcpSocket) {
            client->tcpSocket->disconnectFromHost();
            client->tcpSocket->deleteLater();
        }
    }
    m_clients.clear();
    m_clientBuffers.clear();

    logInfo("网络管理器清理完成");
}

// ========== 服务器管理 ==========

bool NetworkManager::startTcpServer(quint16 port, const QHostAddress& address)
{
    if (m_tcpServer && m_tcpServer->isListening()) {
        logWarning("TCP服务器已在运行");
        return true;
    }

    m_tcpServer = new QTcpServer(this);
    connect(m_tcpServer, &QTcpServer::newConnection, this, &NetworkManager::onNewTcpConnection);

    if (!m_tcpServer->listen(address, port)) {
        QString error = QString("TCP服务器启动失败: %1").arg(m_tcpServer->errorString());
        logError(error);
        emit serverError(error);
        return false;
    }

    m_serverState = ConnectionState::Listening;
    m_serverAddress = address;
    m_serverPort = m_tcpServer->serverPort();
    m_protocol = NetworkProtocol::TCP;

    QString serverInfo = QString("TCP服务器启动成功 - 地址: %1:%2")
                             .arg(address.toString()).arg(m_serverPort);
    logInfo(serverInfo);
    emit serverStarted(m_serverPort, address.toString());

    return true;
}

void NetworkManager::stopServer()
{
    if (m_tcpServer) {
        if (m_tcpServer->isListening()) {
            m_tcpServer->close();
            logInfo("TCP服务器已停止");
        }
        m_tcpServer->deleteLater();
        m_tcpServer = nullptr;
    }

    if (m_udpSocket) {
        if (m_udpSocket->state() == QAbstractSocket::BoundState) {
            m_udpSocket->close();
            logInfo("UDP服务器已停止");
        }
        m_udpSocket->deleteLater();
        m_udpSocket = nullptr;
    }

    m_serverState = ConnectionState::Disconnected;
    emit serverStopped();
}

bool NetworkManager::isServerRunning() const
{
    return m_serverState == ConnectionState::Listening;
}

// ========== 压缩数据发送接口（核心接口）==========

void NetworkManager::sendCompressedPointCloud(const QByteArray& compressedData,
                                              const CompressedDataInfo& info,
                                              quint32 clientId)
{
    try {
        qDebug() << "=== 开始发送压缩点云 ===";

        if (compressedData.isEmpty()) {
            logWarning("点云压缩数据为空");
            return;
        }

        qDebug() << "1. 数据检查通过，大小:" << compressedData.size();

        // 测试点1：数据包创建
        qDebug() << "2. 准备创建NetDataPointCloud";
        NetDataPointCloud* packet = NetDataPointCloud::SetData(
            info.originalSize / sizeof(CrossPlatformPointCloud::Point),
            info.originalSize,
            static_cast<quint16>(info.quality),
            info.compressionRatio,
            compressedData
            );
        qDebug() << "3. NetDataPointCloud创建完成，指针:" << (void*)packet;

        if (!packet) {
            logError("创建点云数据包失败");
            return;
        }

        qDebug() << "4. 数据包有效，header.dataSize:" << packet->header.dataSize;

        // 测试点2：数据包转换
        QByteArray packetData((char*)packet, packet->header.dataSize);
        qDebug() << "5. 转换为QByteArray成功，大小:" << packetData.size();

        // 暂时注释掉实际发送，只测试到这里

        if (clientId == 0) {
            qDebug() << "9. 准备发送给所有客户端，当前客户端数:" << getClientCount();

            // 检查客户端状态
            {
                QMutexLocker locker(&m_clientsMutex);
                qDebug() << "10. 客户端映射大小:" << m_clients.size();
                for (auto it = m_clients.begin(); it != m_clients.end(); ++it) {
                    qDebug() << "    客户端ID:" << it.key() << "指针:" << (void*)it.value().get();
                }
            }

            publishToSubscribedClients(packetData, STREAM_POINTCLOUD);
            qDebug() << "11. publishToSubscribedClients调用完成";
        }

        qDebug() << "6. 跳过实际发送";

        // 测试点3：内存释放
        qDebug() << "7. 准备释放数据包";
        NetDataPointCloud::FreeData(packet);
        qDebug() << "8. 数据包释放完成";

        qDebug() << "=== 压缩点云发送测试完成 ===";

    } catch (const std::exception& e) {
        qCritical() << "发送压缩点云异常:" << e.what();
    } catch (...) {
        qCritical() << "发送压缩点云发生未知异常";
    }
    /*
    try {
        if (compressedData.isEmpty()) {
            logWarning("点云压缩数据为空");
            return;
        }

        qDebug() << "发送压缩点云，数据大小:" << compressedData.size();

        // 创建点云数据包
        NetDataPointCloud* packet = NetDataPointCloud::SetData(
            info.originalSize / sizeof(CrossPlatformPointCloud::Point),
            info.originalSize,
            static_cast<quint16>(info.quality),
            info.compressionRatio,
            compressedData
            );

        if (!packet) {
            logError("创建点云数据包失败");
            return;
        }

        QByteArray packetData((char*)packet, packet->header.dataSize);

        if (clientId == 0) {
            // 发送给所有订阅点云的客户端
            publishToSubscribedClients(packetData, STREAM_POINTCLOUD);
        } else {
            // 发送给指定客户端
            ClientConnection* client = findClient(clientId);
            if (client && client->streamState.isStreamActive(STREAM_POINTCLOUD)) {
                sendData(packetData, clientId);
            }
        }

        NetDataPointCloud::FreeData(packet);

        // 更新统计
        {
            QMutexLocker locker(&m_statsMutex);
            m_statistics.totalFramesSent++;
            updateSendStatistics(packetData.size(), clientId);
        }

        qDebug() << "压缩点云发送完成";

    } catch (const std::exception& e) {
        logError(QString("发送压缩点云异常: %1").arg(e.what()));
    } catch (...) {
        logError("发送压缩点云发生未知异常");
    }*/
    /*
    if (compressedData.isEmpty()) {
        logWarning("点云压缩数据为空");
        return;
    }

    // 创建点云数据包
    NetDataPointCloud* packet = NetDataPointCloud::SetData(
        info.originalSize / sizeof(CrossPlatformPointCloud::Point),
        info.originalSize,
        static_cast<quint16>(info.quality),
        info.compressionRatio,
        compressedData
        );

    QByteArray packetData((char*)packet, packet->header.dataSize);

    if (clientId == 0) {
        // 发送给所有订阅点云的客户端
        publishToSubscribedClients(packetData, STREAM_POINTCLOUD);
    } else {
        // 发送给指定客户端
        ClientConnection* client = findClient(clientId);
        if (client && client->streamState.isStreamActive(STREAM_POINTCLOUD)) {
            sendData(packetData, clientId);
        }
    }

    NetDataPointCloud::FreeData(packet);

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.totalFramesSent++;
        updateSendStatistics(packetData.size(), clientId);
    }

    logInfo(QString("发送压缩点云数据: %1 bytes -> %2 bytes (压缩比: %.2f)")
                .arg(info.originalSize).arg(compressedData.size()).arg(info.compressionRatio));*/
}

void NetworkManager::sendCompressedJPEG(const QByteArray& compressedData,
                                        int cameraId,
                                        const CompressedDataInfo& info,
                                        quint32 clientId)
{
    if (compressedData.isEmpty()) {
        logWarning("JPEG压缩数据为空");
        return;
    }

    // 创建图像数据包
    NetDataImageEx* packet = NetDataImageEx::SetData(
        1080, 1920, // 假设固定分辨率，实际应该从info中获取
        CODEC_JPEG,
        static_cast<quint8>(info.quality),
        compressedData,
        0 // JPEG没有帧序号
        );

    QByteArray packetData((char*)packet, packet->header.dataSize);

    if (clientId == 0) {
        // 发送给所有订阅该相机的客户端
        QMutexLocker locker(&m_clientsMutex);
        for (auto& client : m_clients) {
            if (client && client->state == ConnectionState::Connected &&
                client->streamState.isStreamActive(STREAM_IMAGE) &&
                shouldSendToClient(client->id, cameraId)) {
                sendData(packetData, client->id);
            }
        }
    } else {
        // 发送给指定客户端
        if (shouldSendToClient(clientId, cameraId)) {
            sendData(packetData, clientId);
        }
    }

    NetDataImageEx::FreeData(packet);

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.totalFramesSent++;
        updateSendStatistics(packetData.size(), clientId);
    }

    logInfo(QString("发送JPEG图像: 相机%1, %2 bytes (压缩比: %.2f)")
                .arg(cameraId).arg(compressedData.size()).arg(info.compressionRatio));
}

void NetworkManager::sendH264Frame(const QByteArray& h264Data,
                                   int cameraId,
                                   bool isKeyFrame,
                                   uint32_t frameSequence,
                                   const CompressedDataInfo& info)
{
    if (h264Data.isEmpty() || cameraId < 0 || cameraId >= 4) {
        logWarning("H264帧数据无效");
        return;
    }

    // 创建H264帧数据
    H264FrameData frameData;
    frameData.data = h264Data;
    frameData.cameraId = cameraId;
    frameData.isKeyFrame = isKeyFrame;
    frameData.frameSequence = frameSequence;
    frameData.timestamp = QDateTime::currentMSecsSinceEpoch();
    frameData.quality = info.quality;
    frameData.compressionRatio = info.compressionRatio;

    // 处理关键帧
    if (isKeyFrame) {
        // 立即清空缓冲并发送关键帧
        flushH264Frames(cameraId);

        // 缓存关键帧到H264CacheManager
        m_h264CacheManager->cacheKeyFrame(cameraId, h264Data, frameSequence, info.quality);

        // 立即分发关键帧
        distributeH264Frame(frameData);

        // 更新统计
        {
            QMutexLocker locker(&m_statsMutex);
            m_statistics.h264KeyFramesSent++;
        }

        logInfo(QString("发送H264关键帧: 相机%1, 序号%2, %3 bytes")
                    .arg(cameraId).arg(frameSequence).arg(h264Data.size()));
    } else {
        // P帧加入缓冲队列
        m_h264FrameBuffers[cameraId].enqueue(frameData);

        // 检查是否需要立即刷新（缓冲区满）
        if (m_h264FrameBuffers[cameraId].size() >= m_h264BufferSize) {
            flushH264Frames(cameraId);
        }

        logInfo(QString("缓冲H264 P帧: 相机%1, 序号%2, 缓冲区大小%3")
                    .arg(cameraId).arg(frameSequence).arg(m_h264FrameBuffers[cameraId].size()));
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.h264FramesSent++;
        m_statistics.totalFramesSent++;
    }
}

//===8.7新增 发送GPS数据
void NetworkManager::sendGPSData(const NetDataGPSData myGPSDataMsg,quint32 clientId)
{
    QByteArray packetData((char*)&myGPSDataMsg, myGPSDataMsg.header.dataSize);

    if (clientId == 0) {
        // 发送给所有订阅该相机的客户端
        QMutexLocker locker(&m_clientsMutex);
        for (auto& client : m_clients) {
            if (client && client->state == ConnectionState::Connected &&
                client->streamState.isStreamActive(STREAM_STATUS))  {
                sendData(packetData, client->id);
            }
        }
    } else {
        // 发送给指定客户端
        sendData(packetData, clientId);
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.totalFramesSent++;
        updateSendStatistics(packetData.size(), clientId);
    }

    logInfo(QString("发送GPS数据: 时间%1, %2")
                .arg(myGPSDataMsg.week).arg(myGPSDataMsg.second));
}

// ========== H.264专用管理 ==========

void NetworkManager::sendKeyFrameToClient(quint32 clientId, int cameraId)
{
    QByteArray keyFrameData = m_h264CacheManager->getLatestKeyFrame(cameraId, MEDIUM_QUALITY);
    if (!keyFrameData.isEmpty()) {
        // 创建H264数据包并发送
        NetDataImageEx* packet = NetDataImageEx::SetData(
            1080, 1920, CODEC_H264, static_cast<quint8>(MEDIUM_QUALITY),
            keyFrameData, 0
            );

        QByteArray packetData((char*)packet, packet->header.dataSize);
        sendData(packetData, clientId);

        NetDataImageEx::FreeData(packet);

        // 更新客户端状态
        ClientConnection* client = findClient(clientId);
        if (client) {
            client->streamState.needsKeyFrame[cameraId] = false;
            client->streamState.lastKeyFrameTime[cameraId] = QDateTime::currentMSecsSinceEpoch();
        }

        logInfo(QString("向客户端 %1 发送关键帧 (相机 %2)").arg(clientId).arg(cameraId));
        emit h264KeyFrameRequested(clientId, cameraId);
    } else {
        logWarning(QString("相机 %1 没有可用的关键帧").arg(cameraId));
        emit h264KeyFrameRequested(clientId, cameraId);
    }
}

void NetworkManager::sendKeyFrameToAll(int cameraId)
{
    QMutexLocker locker(&m_clientsMutex);
    for (auto& client : m_clients) {
        if (client && client->state == ConnectionState::Connected &&
            shouldSendToClient(client->id, cameraId)) {
            sendKeyFrameToClient(client->id, cameraId);
        }
    }
}

void NetworkManager::handleH264PacketLoss(quint32 clientId, int cameraId, uint32_t lostFrameId)
{
    // 通知H264缓存管理器
    m_h264CacheManager->handlePacketLoss(clientId, cameraId, lostFrameId);

    // 向客户端发送恢复用的关键帧
    sendKeyFrameToClient(clientId, cameraId);

    // 更新客户端状态
    ClientConnection* client = findClient(clientId);
    if (client) {
        client->streamState.consecutivePacketLoss[cameraId]++;
        client->streamState.needsKeyFrame[cameraId] = true;
        client->streamState.packetLossRate =
            static_cast<float>(client->streamState.consecutivePacketLoss[cameraId]) / 100.0f;
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.h264PacketLossRecoveries++;
    }

    emit h264PacketLossDetected(clientId, cameraId, lostFrameId);
    logWarning(QString("处理H264丢包恢复 - 客户端: %1, 相机: %2, 丢失帧: %3")
                   .arg(clientId).arg(cameraId).arg(lostFrameId));
}

void NetworkManager::setClientCameraSubscription(quint32 clientId, int cameraId, bool enabled)
{
    ClientConnection* client = findClient(clientId);
    if (client && cameraId >= 0 && cameraId < 4) {
        client->cameraSubscriptions[cameraId] = enabled;

        if (enabled && client->streamState.needsKeyFrame[cameraId]) {
            // 订阅时发送关键帧
            sendKeyFrameToClient(clientId, cameraId);
        }

        logInfo(QString("客户端 %1 相机 %2 订阅状态: %3")
                    .arg(clientId).arg(cameraId).arg(enabled ? "启用" : "禁用"));
    }
}

// ========== 客户端管理 ==========

QList<ClientConnection> NetworkManager::getConnectedClients() const
{
    QMutexLocker locker(&m_clientsMutex);
    QList<ClientConnection> clients;

    for (auto& client : m_clients) {
        if (client) {
            clients.append(*client);
        }
    }

    return clients;
}

ClientConnection NetworkManager::getClientInfo(quint32 clientId) const
{
    QMutexLocker locker(&m_clientsMutex);
    auto it = m_clients.find(clientId);
    if (it != m_clients.end() && *it) {
        return **it;
    }

    return ClientConnection(); // 返回默认构造的空对象
}

int NetworkManager::getClientCount() const
{
    QMutexLocker locker(&m_clientsMutex);
    return m_clients.size();
}

QList<quint32> NetworkManager::getConnectedClientIds() const
{
    QMutexLocker locker(&m_clientsMutex);
    return m_clients.keys();
}

// ========== 通用数据发送 ==========

bool NetworkManager::sendData(const QByteArray& data, quint32 clientId)
{
    if (data.isEmpty()) {
        return false;
    }

    if (clientId == 0) {
        // 发送给所有客户端
        return sendDataToAll(data);
    }

    ClientConnection* client = findClient(clientId);
    if (!client || !client->tcpSocket) {
        return false;
    }

    return sendRawData(client->tcpSocket, data);
}

bool NetworkManager::sendDataToAll(const QByteArray& data)
{
    if (data.isEmpty()) {
        return false;
    }

    QMutexLocker locker(&m_clientsMutex);
    bool allSuccess = true;

    for (auto& client : m_clients) {
        if (client && client->tcpSocket && client->state == ConnectionState::Connected) {
            if (!sendRawData(client->tcpSocket, data)) {
                allSuccess = false;
            }
        }
    }

    return allSuccess;
}

bool NetworkManager::sendPacket(quint8 type, const QByteArray& data, quint32 clientId)
{
    QByteArray packet = createPacket(type, data);
    return sendData(packet, clientId);
}

// ========== 流控制 ==========

void NetworkManager::enableClientStream(quint32 clientId, StreamType streamType, bool enabled)
{
    ClientConnection* client = findClient(clientId);
    if (!client) {
        return;
    }
    qDebug()<<"收到的流控制命令为:"<<streamType;

    switch (streamType) {
    case STREAM_POINTCLOUD:
        qDebug()<<"启动点云流";
        client->streamState.pointCloudEnabled = enabled;
        break;
    case STREAM_IMAGE:
        qDebug()<<"启动图像流";
        client->streamState.imageEnabled = enabled;
        if (enabled) {
            // 启用图像流时，为所有相机发送关键帧
            for (int i = 0; i < 4; i++) {
                if (client->cameraSubscriptions[i]) {
                    sendKeyFrameToClient(clientId, i);
                }
            }
        }
        break;
    case STREAM_STATUS:
        qDebug()<<"启动数据流";
        client->streamState.statusEnabled = enabled;
        break;
    default:
        break;
    }

    logInfo(QString("客户端 %1 流类型 %2 状态: %3")
                .arg(clientId).arg(streamType).arg(enabled ? "启用" : "禁用"));

    emit streamControlReceived(clientId, streamType, enabled ? ACTION_START : ACTION_STOP);
}

void NetworkManager::setClientStreamParameters(quint32 clientId, const ClientStreamState& params)
{
    ClientConnection* client = findClient(clientId);
    if (client) {
        client->streamState = params;
        logInfo(QString("更新客户端 %1 流参数").arg(clientId));
    }
}

ClientStreamState NetworkManager::getClientStreamState(quint32 clientId) const
{
    const ClientConnection* client = findClient(clientId);
    return client ? client->streamState : ClientStreamState();
}

// ========== 统计和监控 ==========

NetworkStatistics NetworkManager::getStatistics() const
{
    QMutexLocker locker(&m_statsMutex);
    NetworkStatistics stats = m_statistics;
    stats.uptimeSeconds = m_uptimeTimer.elapsed() / 1000;
    stats.activeConnections = getClientCount();
    return stats;
}

void NetworkManager::enableHeartbeat(int intervalMs)
{
    m_heartbeatEnabled = true;
    m_heartbeatTimer->start(intervalMs);
    logInfo(QString("启用心跳，间隔: %1ms").arg(intervalMs));
}

void NetworkManager::disableHeartbeat()
{
    m_heartbeatEnabled = false;
    m_heartbeatTimer->stop();
    logInfo("禁用心跳");
}

void NetworkManager::enableStatusBroadcast(int intervalMs)
{
    m_statusBroadcastEnabled = true;
    m_statusTimer->start(intervalMs);
    logInfo(QString("启用状态广播，间隔: %1ms").arg(intervalMs));
}

void NetworkManager::disableStatusBroadcast()
{
    m_statusBroadcastEnabled = false;
    m_statusTimer->stop();
    logInfo("禁用状态广播");
}

// ========== 配置设置 ==========

void NetworkManager::setMaxConnections(int maxConnections)
{
    m_maxConnections = maxConnections;
    if (m_tcpServer) {
        m_tcpServer->setMaxPendingConnections(maxConnections);
    }
}

void NetworkManager::setConnectionTimeout(int timeoutMs)
{
    m_connectionTimeoutMs = timeoutMs;
}

void NetworkManager::setH264FrameBufferSize(int bufferSize)
{
    m_h264BufferSize = qBound(1, bufferSize, 10);
    logInfo(QString("设置H264帧缓冲大小: %1").arg(m_h264BufferSize));
}

void NetworkManager::setH264FlushInterval(int intervalMs)
{
    m_h264FlushTimer->setInterval(intervalMs);
    logInfo(QString("设置H264刷新间隔: %1ms").arg(intervalMs));
}

// ========== 状态查询 ==========

bool NetworkManager::hasActiveClients() const
{
    return getClientCount() > 0;
}

QStringList NetworkManager::getConnectedClientInfo() const
{
    QStringList clientInfo;
    QMutexLocker locker(&m_clientsMutex);

    for (auto& client : m_clients) {
        if (client) {
            QString info = QString("客户端 %1: %2:%3 (连接时长: %4s)")
                               .arg(client->id)
                               .arg(client->address.toString())
                               .arg(client->port)
                               .arg(client->lastActivity.elapsed() / 1000);
            clientInfo.append(info);
        }
    }

    return clientInfo;
}

H264CacheManager::CacheStats NetworkManager::getH264CacheStats() const
{
    if (m_h264CacheManager) {
        return m_h264CacheManager->getStats();
    }
    return H264CacheManager::CacheStats();
}

// ========== 客户端控制槽 ==========

void NetworkManager::kickClient(quint32 clientId, const QString& reason)
{
    ClientConnection* client = findClient(clientId);
    if (client && client->tcpSocket) {
        logInfo(QString("踢出客户端 %1: %2").arg(clientId).arg(reason));
        client->tcpSocket->disconnectFromHost();
    }
}

void NetworkManager::kickAllClients(const QString& reason)
{
    QMutexLocker locker(&m_clientsMutex);
    logInfo(QString("踢出所有客户端: %1").arg(reason));

    for (auto& client : m_clients) {
        if (client && client->tcpSocket) {
            client->tcpSocket->disconnectFromHost();
        }
    }
}

// ========== 心跳和状态 ==========

void NetworkManager::sendHeartbeatToAll()
{
    NetworkHeartbeat heartbeat;
    heartbeat.sequenceNumber = ++m_packetSequenceNumber;
    heartbeat.timestamp = CrossPlatformTimestamp::now();

    QByteArray heartbeatData((char*)&heartbeat, sizeof(heartbeat));
    sendDataToAll(heartbeatData);
}

void NetworkManager::sendStatusToAll()
{
    NetworkStatus status;
    status.connectedClients = getClientCount();
    status.totalFramesSent = m_statistics.totalFramesSent;
    status.timestamp = CrossPlatformTimestamp::now();

    QByteArray statusData((char*)&status, sizeof(status));
    sendDataToAll(statusData);
}

// ========== 网络事件处理 ==========

void NetworkManager::onNewTcpConnection()
{
    if (!m_tcpServer) {
        return;
    }

    while (m_tcpServer->hasPendingConnections()) {
        QTcpSocket* socket = m_tcpServer->nextPendingConnection();
        if (socket) {
            if (getClientCount() >= m_maxConnections) {
                logWarning("连接数已达上限，拒绝新连接");
                socket->disconnectFromHost();
                socket->deleteLater();
                continue;
            }

            addClient(socket);
        }
    }
}

void NetworkManager::onTcpSocketReadyRead()
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) {
        return;
    }

    ClientConnection* client = findClient(socket);
    if (!client) {
        return;
    }

    QByteArray data = socket->readAll();
    if (!data.isEmpty()) {
        updateReceiveStatistics(data.size(), client->id);
        updateClientActivity(client->id);
        processIncomingData(data, client->id);
    }
}

void NetworkManager::onTcpSocketDisconnected()
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) {
        return;
    }

    ClientConnection* client = findClient(socket);
    if (client) {
        QString reason = "客户端主动断开连接";
        logInfo(QString("客户端 %1 断开连接: %2").arg(client->id).arg(reason));
        emit clientDisconnected(client->id, reason);
        removeClient(client->id);
    }

    socket->deleteLater();
}

void NetworkManager::onTcpSocketError(QAbstractSocket::SocketError error)
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) {
        return;
    }

    ClientConnection* client = findClient(socket);
    QString errorString = socket->errorString();

    if (client) {
        logError(QString("客户端 %1 网络错误: %2").arg(client->id).arg(errorString));
        emit clientError(client->id, errorString);
    } else {
        logError(QString("网络错误: %1").arg(errorString));
    }
}

// ========== 定时器事件 ==========

void NetworkManager::onHeartbeatTimer()
{
    if (m_heartbeatEnabled) {
        sendHeartbeatToAll();
    }
}

void NetworkManager::onStatusTimer()
{
    if (m_statusBroadcastEnabled) {
        sendStatusToAll();
    }
}

void NetworkManager::onStatisticsTimer()
{
    updateStatistics();

    QMutexLocker locker(&m_statsMutex);
    emit statisticsUpdated(m_statistics);
}

void NetworkManager::onCleanupTimer()
{
    checkClientTimeouts();
}

void NetworkManager::onH264FlushTimer()
{
    // 定期刷新所有相机的H264帧缓冲
    flushH264Frames(-1); // -1表示所有相机
}

// ========== H.264缓存管理器事件 ==========

void NetworkManager::onKeyFrameCached(int cameraId, uint32_t sequence, size_t dataSize)
{
    logInfo(QString("H264关键帧已缓存: 相机%1, 序号%2, 大小%3 bytes")
                .arg(cameraId).arg(sequence).arg(dataSize));
}

void NetworkManager::onClientKeyFrameNeeded(uint32_t clientId, int cameraId)
{
    sendKeyFrameToClient(clientId, cameraId);
}

void NetworkManager::onCacheStatsUpdated(const H264CacheManager::CacheStats& stats)
{
    // 可以在这里处理缓存统计更新
    Q_UNUSED(stats)
}

// ========== 私有方法实现 ==========

quint32 NetworkManager::generateClientId()
{
    return m_nextClientId++;
}

ClientConnection* NetworkManager::findClient(quint32 clientId)
{
    QMutexLocker locker(&m_clientsMutex);
    auto it = m_clients.find(clientId);
    return (it != m_clients.end() && *it) ? it->get() : nullptr;
}

const ClientConnection* NetworkManager::findClient(quint32 clientId) const
{
    QMutexLocker locker(&m_clientsMutex);
    auto it = m_clients.find(clientId);
    return (it != m_clients.end() && *it) ? it->get() : nullptr;
}

ClientConnection* NetworkManager::findClient(QTcpSocket* socket)
{
    QMutexLocker locker(&m_clientsMutex);
    for (auto& client : m_clients) {
        if (client && client->tcpSocket == socket) {
            return client.get();
        }
    }
    return nullptr;
}

void NetworkManager::addClient(QTcpSocket* socket)
{
    quint32 clientId = generateClientId();
    auto client = QSharedPointer<ClientConnection>::create();

    client->id = clientId;
    client->tcpSocket = socket;
    client->address = socket->peerAddress();
    client->port = socket->peerPort();
    client->state = ConnectionState::Connected;
    client->lastActivity.start();

    // 连接socket信号
    connect(socket, &QTcpSocket::readyRead, this, &NetworkManager::onTcpSocketReadyRead);
    connect(socket, &QTcpSocket::disconnected, this, &NetworkManager::onTcpSocketDisconnected);
    QT_TCP_ERROR_CONNECT(socket, this, &NetworkManager::onTcpSocketError);

    // 通知H264缓存管理器新客户端
    m_h264CacheManager->handleNewClient(clientId, MEDIUM_QUALITY);

    {
        QMutexLocker locker(&m_clientsMutex);
        m_clients[clientId] = client;
        m_clientBuffers[clientId] = QByteArray();
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.totalConnections++;
    }

    QString clientInfo = QString("%1:%2").arg(client->address.toString()).arg(client->port);
    logInfo(QString("新客户端连接: ID=%1, 地址=%2").arg(clientId).arg(clientInfo));
    emit clientConnected(clientId, client->address.toString(), client->port);
}

void NetworkManager::removeClient(quint32 clientId)
{
    QMutexLocker locker(&m_clientsMutex);

    // 通知H264缓存管理器客户端断开
    m_h264CacheManager->handleClientDisconnected(clientId);

    m_clients.remove(clientId);
    m_clientBuffers.remove(clientId);
}

void NetworkManager::updateClientActivity(quint32 clientId)
{
    ClientConnection* client = findClient(clientId);
    if (client) {
        client->lastActivity.restart();
    }
}

void NetworkManager::processH264Frame(const H264FrameData& frameData)
{
    // 这个方法在sendH264Frame中已经处理了逻辑
    // 这里保留作为扩展接口
    distributeH264Frame(frameData);
}

void NetworkManager::distributeH264Frame(const H264FrameData& frameData)
{
    qDebug() << "=== distributeH264Frame开始 ===";

    // 创建数据包
    NetDataImageEx* packet = NetDataImageEx::SetData(
        1080, 1920, CODEC_H264, static_cast<quint8>(frameData.quality),
        frameData.data, frameData.frameSequence
        );

    if (!packet) {
        qCritical() << "数据包创建失败";
        return;
    }

    QByteArray packetData((char*)packet, packet->header.dataSize);

    // 发送给客户端
    {
        QMutexLocker locker(&m_clientsMutex);

        for (auto& client : m_clients) {
            if (!client || client->state != ConnectionState::Connected ||
                !client->streamState.isStreamActive(STREAM_IMAGE) ||
                frameData.cameraId < 0 || frameData.cameraId >= 4 ||
                !client->cameraSubscriptions[frameData.cameraId] ||
                !client->streamState.shouldSendFrame()) {
                continue;
            }

            qDebug() << "发送给客户端" << client->id;

            bool sendResult = sendRawData(client->tcpSocket, packetData);

            if (sendResult) {
                // 直接更新统计，不调用可能死锁的函数
                {
                    QMutexLocker statsLocker(&m_statsMutex);
                    m_statistics.totalBytesSent += packetData.size();
                }

                // 直接更新客户端统计
                client->bytesSent += packetData.size();
                client->streamState.bytesSent += packetData.size();

                // 直接更新H264状态
                client->streamState.lastFrameSequence[frameData.cameraId] = frameData.frameSequence;
                if (frameData.isKeyFrame) {
                    client->streamState.needsKeyFrame[frameData.cameraId] = false;
                    client->streamState.lastKeyFrameTime[frameData.cameraId] = QDateTime::currentMSecsSinceEpoch();
                    client->streamState.consecutivePacketLoss[frameData.cameraId] = 0;
                }

                // 重置帧时间
                client->streamState.lastFrameTime.restart();

                // H264缓存管理器更新（如果不会导致死锁）
                // m_h264CacheManager->updateClientFrameStatus(client->id, frameData.cameraId, frameData.frameSequence);
            }
        }
    }

    NetDataImageEx::FreeData(packet);
    emit h264FrameSent(frameData.cameraId, frameData.frameSequence, 0, frameData.isKeyFrame);

    qDebug() << "=== distributeH264Frame完成 ===";
}
/* 9.12 更改统计信息BUG
void NetworkManager::distributeH264Frame(const H264FrameData& frameData)
{
    qDebug() << "=== distributeH264Frame开始 ===";
    qDebug() << "相机:" << frameData.cameraId << "序号:" << frameData.frameSequence
             << "关键帧:" << frameData.isKeyFrame;

    // 创建H264数据包
    NetDataImageEx* packet = NetDataImageEx::SetData(
        1080, 1920, CODEC_H264, static_cast<quint8>(frameData.quality),
        frameData.data, frameData.frameSequence
        );

    if (!packet) {
        qCritical() << "NetDataImageEx::SetData失败";
        return;
    }

    qDebug() << "数据包创建成功，大小:" << packet->header.dataSize;

    QByteArray packetData((char*)packet, packet->header.dataSize);

    qDebug() << "开始发送给客户端，客户端数:" << m_clients.size();

    // 发送给所有符合条件的客户端
    {
        QMutexLocker locker(&m_clientsMutex);
        qDebug() << "获得客户端互斥锁";

        for (auto& client : m_clients) {
            if (!client) {
                qDebug() << "客户端指针为空，跳过";
                continue;
            }

            qDebug() << "处理客户端:" << client->id << "状态:" << (int)client->state;

            // 检查连接状态
            bool isConnected = (client->state == ConnectionState::Connected);
            if (!isConnected) {
                qDebug() << "客户端" << client->id << "未连接，跳过";
                continue;
            }

            // 检查图像流是否激活
            bool streamActive = client->streamState.isStreamActive(STREAM_IMAGE);
            if (!streamActive) {
                qDebug() << "客户端" << client->id << "图像流未激活，跳过";
                continue;
            }

            // 直接检查相机订阅（避免调用findClient导致死锁）
            bool cameraSubscribed = false;
            if (frameData.cameraId >= 0 && frameData.cameraId < 4) {
                cameraSubscribed = client->cameraSubscriptions[frameData.cameraId];
            }

            if (!cameraSubscribed) {
                qDebug() << "客户端" << client->id << "未订阅相机" << frameData.cameraId << "，跳过";
                continue;
            }

            // 检查帧率限制
            bool shouldSendFrame = client->streamState.shouldSendFrame();
            if (!shouldSendFrame) {
                qDebug() << "客户端" << client->id << "帧率限制，跳过";
                continue;
            }

            qDebug() << "所有条件满足，向客户端" << client->id << "发送H264数据";

            // 发送数据
            bool sendResult = sendRawData(client->tcpSocket, packetData);
            qDebug() << "向客户端" << client->id << "发送结果:" << sendResult;

            if (sendResult) {
                // 更新客户端H264状态
                qDebug() << "更新客户端" << client->id << "H264状态";
                //updateClientH264State(client->id, frameData.cameraId,
                //                      frameData.frameSequence, frameData.isKeyFrame);

                // 更新发送统计
                //updateSendStatistics(packetData.size(), client->id);

                // 重置帧时间
                client->streamState.lastFrameTime.restart();

                qDebug() << "客户端" << client->id << "H264帧发送和状态更新完成";
            } else {
                qWarning() << "向客户端" << client->id << "发送数据失败";
            }
        }

        qDebug() << "客户端循环处理完成";
    }
    qDebug() << "释放客户端互斥锁";

    // 释放数据包内存
    NetDataImageEx::FreeData(packet);
    qDebug() << "数据包内存已释放";

    // 发送信号通知
    emit h264FrameSent(frameData.cameraId, frameData.frameSequence, 0, frameData.isKeyFrame);

    qDebug() << "=== distributeH264Frame完成 ===";

    //排查问题
    // 创建H264数据包
    NetDataImageEx* packet = NetDataImageEx::SetData(
        1080, 1920, CODEC_H264, static_cast<quint8>(frameData.quality),
        frameData.data, frameData.frameSequence
        );

    QByteArray packetData((char*)packet, packet->header.dataSize);

    // 发送给所有订阅该相机的客户端
    QMutexLocker locker(&m_clientsMutex);
    for (auto& client : m_clients) {

        //修改，关闭流处理进行测试

        if (client && client->state == ConnectionState::Connected &&
            client->streamState.isStreamActive(STREAM_IMAGE) &&
            shouldSendToClient(client->id, frameData.cameraId) &&
            client->streamState.shouldSendFrame()) {

            sendRawData(client->tcpSocket, packetData);
            updateClientH264State(client->id, frameData.cameraId,
                                  frameData.frameSequence, frameData.isKeyFrame);
            updateSendStatistics(packetData.size(), client->id);

            // 重置帧时间计时器
            client->streamState.lastFrameTime.restart();
        }
    }

    NetDataImageEx::FreeData(packet);

    emit h264FrameSent(frameData.cameraId, frameData.frameSequence, 0, frameData.isKeyFrame);
}*/

void NetworkManager::flushH264Frames(int cameraId)
{
    if (cameraId == -1) {
        // 刷新所有相机
        for (auto it = m_h264FrameBuffers.begin(); it != m_h264FrameBuffers.end(); ++it) {
            while (!it.value().isEmpty()) {
                distributeH264Frame(it.value().dequeue());
            }
        }
    } else {
        // 刷新指定相机
        auto& buffer = m_h264FrameBuffers[cameraId];
        while (!buffer.isEmpty()) {
            distributeH264Frame(buffer.dequeue());
        }
    }
}

bool NetworkManager::shouldSendToClient(quint32 clientId, int cameraId) const
{
    qDebug() << "=== shouldSendToClient检查 ===";
    qDebug() << "客户端ID:" << clientId << "相机ID:" << cameraId;

    if (cameraId < 0 || cameraId >= 4) {
        qDebug() << "相机ID超出范围";
        return false;
    }

    // 关键修改:删除findClient
    // const ClientConnection* client = findClient(clientId);  // 这里可能死锁

    // 直接在distributeH264Frame中传递client指针
    qDebug() << "=== shouldSendToClient完成 ===";
    return true; // 临时返回true来跳过检查
    /*
     * //测试
    if (cameraId < 0 || cameraId >= 4) {
        return false;
    }

    const ClientConnection* client = findClient(clientId);
    return client && client->cameraSubscriptions[cameraId];
*/
}

void NetworkManager::updateClientH264State(quint32 clientId, int cameraId,
                                           uint32_t frameSequence, bool isKeyFrame)
{
    ClientConnection* client = findClient(clientId);
    if (client && cameraId >= 0 && cameraId < 4) {
        client->streamState.lastFrameSequence[cameraId] = frameSequence;

        if (isKeyFrame) {
            client->streamState.needsKeyFrame[cameraId] = false;
            client->streamState.lastKeyFrameTime[cameraId] = QDateTime::currentMSecsSinceEpoch();
            client->streamState.consecutivePacketLoss[cameraId] = 0; // 重置丢包计数
        }

        // 更新H264缓存管理器状态
        m_h264CacheManager->updateClientFrameStatus(clientId, cameraId, frameSequence);
    }
}
/*
void NetworkManager::processIncomingData(const QByteArray& data, quint32 clientId)
{
    // 这里可以处理客户端发来的控制命令
    // 例如流控制请求、丢包报告等
    Q_UNUSED(data)
    Q_UNUSED(clientId)

    // TODO: 实现数据包解析和处理
}

void NetworkManager::processPacket(const NetworkPacketHeader& header,
                                   const QByteArray& payload, quint32 clientId)
{
    Q_UNUSED(header)
    Q_UNUSED(payload)
    Q_UNUSED(clientId)

    // TODO: 实现具体的数据包处理逻辑
}*/

bool NetworkManager::validatePacket(const NetworkPacketHeader& header, const QByteArray& data)
{
    return header.dataSize == static_cast<quint32>(data.size()); //&& header.version == 1;
}

bool NetworkManager::sendRawData(QTcpSocket* socket, const QByteArray& data)
{
    if (!socket || socket->state() != QAbstractSocket::ConnectedState) {
        return false;
    }

    qint64 bytesWritten = socket->write(data);
    socket->flush();

    return bytesWritten == data.size();
}

QByteArray NetworkManager::createPacket(quint8 type, const QByteArray& data)
{
    NetworkPacketHeader header;
    header.type = type;
    header.dataSize = sizeof(header) + data.size();
    header.sequenceNumber = ++m_packetSequenceNumber;
    header.timestamp = QDateTime::currentMSecsSinceEpoch();

    QByteArray packet;
    packet.append((char*)&header, sizeof(header));
    packet.append(data);

    return packet;
}

void NetworkManager::updateSendStatistics(qint64 bytesSent, quint32 clientId)
{
    QMutexLocker locker(&m_statsMutex);
    m_statistics.totalBytesSent += bytesSent;

    ClientConnection* client = findClient(clientId);
    if (client) {
        client->bytesSent += bytesSent;
        client->streamState.bytesSent += bytesSent;
    }
}

void NetworkManager::updateReceiveStatistics(qint64 bytesReceived, quint32 clientId)
{
    QMutexLocker locker(&m_statsMutex);
    m_statistics.totalBytesReceived += bytesReceived;

    ClientConnection* client = findClient(clientId);
    if (client) {
        client->bytesReceived += bytesReceived;
    }
}

void NetworkManager::updateStatistics()
{
    QMutexLocker locker(&m_statsMutex);
    m_statistics.averageBandwidthUsage = calculateBandwidth();
    m_statistics.activeConnections = getClientCount();
}

void NetworkManager::checkClientTimeouts()
{
    QMutexLocker locker(&m_clientsMutex);
    QList<quint32> timeoutClients;

    for (auto& client : m_clients) {
        if (client && client->lastActivity.elapsed() > m_connectionTimeoutMs) {
            timeoutClients.append(client->id);
        }
    }

    locker.unlock();

    // 移除超时客户端
    for (quint32 clientId : timeoutClients) {
        logWarning(QString("客户端 %1 连接超时").arg(clientId));
        emit connectionTimeout(clientId);
        kickClient(clientId, "连接超时");
    }
}

float NetworkManager::calculateBandwidth()
{
    qint64 currentBytes = m_statistics.totalBytesSent;
    qint64 deltaBytes = currentBytes - m_lastBytesTransferred;
    qint64 deltaTime = m_bandwidthTimer.restart();

    m_lastBytesTransferred = currentBytes;

    if (deltaTime > 0) {
        return static_cast<float>(deltaBytes * 1000) / (1024 * 1024 * deltaTime); // MB/s
    }

    return 0.0f;
}

void NetworkManager::publishToSubscribedClients(const QByteArray& data, StreamType streamType)
{
    QMutexLocker locker(&m_clientsMutex);

    for (auto& client : m_clients) {
        if (client && client->state == ConnectionState::Connected &&
            client->streamState.isStreamActive(streamType)) {

            sendRawData(client->tcpSocket, data);

            // 直接更新统计，避免调用updateSendStatistics
            {
                QMutexLocker statsLocker(&m_statsMutex);
                m_statistics.totalBytesSent += data.size();
            }
            client->bytesSent += data.size();
            client->streamState.bytesSent += data.size();
        }
    }
}
/* 9.12 更新，修复BUG
void NetworkManager::publishToSubscribedClients(const QByteArray& data, StreamType streamType)
{
    qDebug() << "=== publishToSubscribedClients开始 ===";
    qDebug() << "数据大小:" << data.size() << "流类型:" << streamType;

    QMutexLocker locker(&m_clientsMutex);
    qDebug() << "获取客户端锁成功，客户端数:" << m_clients.size();

    int processedCount = 0;
    for (auto& client : m_clients) {
        qDebug() << "处理客户端" << processedCount++;

        if (!client) {
            qDebug() << "客户端指针为空，跳过";
            continue;
        }

        qDebug() << "客户端ID:" << client->id << "状态:" << (int)client->state;


        if (client->state == ConnectionState::Connected &&
            client->streamState.isStreamActive(streamType)) {

            qDebug() << "客户端符合发送条件，准备发送";
            sendRawData(client->tcpSocket, data);
            qDebug() << "sendRawData调用完成";
            定位到BUG位置
            //updateSendStatistics(data.size(), client->id);
            qDebug() << "统计更新完成";
        } else {
            qDebug() << "客户端不符合发送条件";
        }
    }

    qDebug() << "=== publishToSubscribedClients完成 ===";

    QMutexLocker locker(&m_clientsMutex);
    for (auto& client : m_clients) {
        if (client && client->state == ConnectionState::Connected &&
            client->streamState.isStreamActive(streamType)) {
            sendRawData(client->tcpSocket, data);
            updateSendStatistics(data.size(), client->id);
        }
    }
}*/

QList<quint32> NetworkManager::getSubscribedClients(StreamType streamType)
{
    QList<quint32> subscribedClients;
    QMutexLocker locker(&m_clientsMutex);

    for (auto& client : m_clients) {
        if (client && client->state == ConnectionState::Connected &&
            client->streamState.isStreamActive(streamType)) {
            subscribedClients.append(client->id);
        }
    }

    return subscribedClients;
}

// ========== 日志方法 ==========

void NetworkManager::logInfo(const QString& message)
{
    qCInfo(networkLog) << message;
}

void NetworkManager::logWarning(const QString& message)
{
    qCWarning(networkLog) << message;
}

void NetworkManager::logError(const QString& message)
{
    qCCritical(networkLog) << message;
}


void NetworkManager::onUdpReadyRead()
{
    if (!m_udpSocket) {
        return;
    }

    while (m_udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(m_udpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        qint64 size = m_udpSocket->readDatagram(datagram.data(), datagram.size(),
                                                &sender, &senderPort);

        if (size > 0) {
            datagram.resize(size);
            emit udpDataReceived(datagram, sender, senderPort);
            updateReceiveStatistics(size, 0); // UDP没有特定客户端ID
        }
    }
}

void NetworkManager::onUdpError(QAbstractSocket::SocketError error)
{
    if (m_udpSocket) {
        QString errorString = m_udpSocket->errorString();
        logError(QString("UDP错误: %1").arg(errorString));
    }
}

void NetworkManager::onClientConnected()
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (socket && socket == m_clientSocket) {
        m_clientState = ConnectionState::Connected;
        logInfo(QString("已连接到服务器: %1:%2")
                    .arg(socket->peerAddress().toString())
                    .arg(socket->peerPort()));
    }
}

void NetworkManager::onClientDisconnected()
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (socket && socket == m_clientSocket) {
        m_clientState = ConnectionState::Disconnected;
        logInfo("与服务器连接已断开");
    }
}

void NetworkManager::onClientError(QAbstractSocket::SocketError error)
{
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (socket && socket == m_clientSocket) {
        QString errorString = socket->errorString();
        logError(QString("客户端连接错误: %1").arg(errorString));
        m_clientState = ConnectionState::Error;
    }
}

bool NetworkManager::startUdpServer(quint16 port, const QHostAddress& address)
{
    if (m_udpSocket && m_udpSocket->state() == QAbstractSocket::BoundState) {
        logWarning("UDP服务器已在运行");
        return true;
    }

    m_udpSocket = new QUdpSocket(this);
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &NetworkManager::onUdpReadyRead);
    QT_UDP_ERROR_CONNECT(m_udpSocket, this, &NetworkManager::onUdpError);

    if (!m_udpSocket->bind(address, port)) {
        QString error = QString("UDP服务器启动失败: %1").arg(m_udpSocket->errorString());
        logError(error);
        emit serverError(error);
        return false;
    }

    m_serverState = ConnectionState::Listening;
    m_serverAddress = address;
    m_serverPort = port;
    m_protocol = NetworkProtocol::UDP;

    QString serverInfo = QString("UDP服务器启动成功 - 地址: %1:%2")
                             .arg(address.toString()).arg(port);
    logInfo(serverInfo);
    emit serverStarted(port, address.toString());

    return true;
}

bool NetworkManager::connectToHost(const QString& hostName, quint16 port, NetworkProtocol protocol)
{
    if (m_clientSocket && m_clientSocket->state() == QAbstractSocket::ConnectedState) {
        logWarning("客户端已连接");
        return true;
    }

    m_clientSocket = new QTcpSocket(this);
    connect(m_clientSocket, &QTcpSocket::connected, this, &NetworkManager::onClientConnected);
    connect(m_clientSocket, &QTcpSocket::disconnected, this, &NetworkManager::onClientDisconnected);
    connect(m_clientSocket, &QTcpSocket::readyRead, this, &NetworkManager::onTcpSocketReadyRead);
    QT_TCP_ERROR_CONNECT(m_clientSocket, this, &NetworkManager::onClientError);

    m_clientState = ConnectionState::Connecting;
    m_protocol = protocol;
    m_clientSocket->connectToHost(hostName, port);

    logInfo(QString("正在连接到 %1:%2").arg(hostName).arg(port));
    return true;
}

void NetworkManager::disconnectFromHost()
{
    if (m_clientSocket) {
        if (m_clientSocket->state() != QAbstractSocket::UnconnectedState) {
            m_clientSocket->disconnectFromHost();
            logInfo("客户端连接已断开");
        }
        m_clientSocket->deleteLater();
        m_clientSocket = nullptr;
    }
    m_clientState = ConnectionState::Disconnected;
}

bool NetworkManager::isConnected() const
{
    return m_clientState == ConnectionState::Connected;
}

bool NetworkManager::sendUdpData(const QByteArray& data, const QHostAddress& address, quint16 port)
{
    if (!m_udpSocket || m_udpSocket->state() != QAbstractSocket::BoundState) {
        logWarning("UDP服务器未启动");
        return false;
    }

    qint64 bytesWritten = m_udpSocket->writeDatagram(data, address, port);
    if (bytesWritten == data.size()) {
        updateSendStatistics(bytesWritten, 0);
        return true;
    }

    logError(QString("UDP数据发送失败: %1").arg(m_udpSocket->errorString()));
    return false;
}

// ========== 其他缺失的辅助函数 ==========

void NetworkManager::processIncomingData(const QByteArray& data, quint32 clientId)
{
    if (data.size() < sizeof(NetworkPacketHeader)) {
        logWarning("接收到的数据包过小");
        return;
    }

    const NetworkPacketHeader* header = reinterpret_cast<const NetworkPacketHeader*>(data.constData());

    if (!validatePacket(*header, data)) {
        logWarning("无效的数据包");
        return;
    }

    QByteArray payload = data.mid(sizeof(NetworkPacketHeader));
    //processPacket(*header, payload, clientId);
    processPacket(*header, data, clientId);


    emit dataReceived(data, clientId);
    emit packetReceived(header->type,data, clientId);
}

void NetworkManager::processPacket(const NetworkPacketHeader& header, const QByteArray& fullPacket, quint32 clientId)
{
    switch (header.type) {
    case 's': // 流控制
        qDebug()<<"收到流控制命令,大小为:"<<fullPacket.size()<<sizeof(NetworkPacketHeader);
        if (fullPacket.size() >= sizeof(NetDataStreamControl)) {
            const NetDataStreamControl* control = reinterpret_cast<const NetDataStreamControl*>(fullPacket.constData());
            processStreamControlCommand(clientId, control);
        }
        break;

    case 'h': // 心跳
        emit heartbeatReceived(clientId);
        break;

    default:
        logInfo(QString("收到未知类型数据包: %1 从客户端 %2").arg((char)header.type).arg(clientId));
        break;
    }
}

void NetworkManager::processStreamControlCommand(quint32 clientId, const NetDataStreamControl* command)
{
    StreamType streamType = static_cast<StreamType>(command->streamType);
    StreamAction action = static_cast<StreamAction>(command->action);
    qDebug()<<"流处理参数命令get";

    switch (action) {
    case ACTION_START:
        enableClientStream(clientId, streamType, true);
        break;
    case ACTION_STOP:
        enableClientStream(clientId, streamType, false);
        break;
    case ACTION_PAUSE:
        // TODO: 实现暂停逻辑
        break;
    case ACTION_RESUME:
        // TODO: 实现恢复逻辑
        break;
    }

    emit streamControlReceived(clientId, streamType, action);
}



// ========== moc文件需要的信号槽连接辅助 ==========
// 这个函数可能在某些Qt版本中需要，用于确保信号槽正确连接
/*
void NetworkManager::connectClientSocket(QTcpSocket* socket)
{
    if (!socket) return;

    // 确保信号正确连接
    connect(socket, &QTcpSocket::readyRead,
            this, &NetworkManager::onTcpSocketReadyRead, Qt::UniqueConnection);
    connect(socket, &QTcpSocket::disconnected,
            this, &NetworkManager::onTcpSocketDisconnected, Qt::UniqueConnection);

    // 使用兼容性宏连接错误信号
    QT_TCP_ERROR_CONNECT(socket, this, &NetworkManager::onTcpSocketError);
}
*/
