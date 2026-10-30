// data_broadcast_manager.cpp - 重构版本实现
#include "navcar_project_server/data_broadcast_manager.h"
#include <QCoreApplication>
#include <QThread>

Q_LOGGING_CATEGORY(broadcastLog, "broadcast")

DataBroadcastManager::DataBroadcastManager(QObject *parent)
    : QObject(parent)
    , m_compressionManager(nullptr)
    , m_networkManager(nullptr)
    , m_initialized(false)
    , m_broadcastPaused(false)
    , m_statisticsTimer(nullptr)
{
    // 初始化统计信息
    m_statistics = BroadcastStatistics();
    m_statistics.startTime = QDateTime::currentDateTime();

    // 创建统计更新定时器
    m_statisticsTimer = new QTimer(this);
    connect(m_statisticsTimer, &QTimer::timeout, this, &DataBroadcastManager::onStatisticsUpdateTimer);
    m_statisticsTimer->start(1000); // 每秒更新统计

    // 启动运行时间计时器
    m_uptimeTimer.start();

    // 初始化性能监控计时器
    for (int i = 0; i < 4; i++) {
        m_lastFrameTime[i].start();
    }
    m_lastPointCloudTime.start();
}

DataBroadcastManager::~DataBroadcastManager()
{
    cleanup();
}

// ========== 初始化和配置 ==========

bool DataBroadcastManager::initialize(const BroadcastConfig& config)
{
    if (m_initialized) {
        logWarning("数据广播管理器已初始化");
        return true;
    }

    logInfo("初始化数据广播管理器...");

    // 保存配置
    m_config = config;

    // 设置组件
    if (!setupCompressionManager()) {
        logError("压缩管理器设置失败");
        return false;
    }

    if (!setupNetworkManager()) {
        logError("网络管理器设置失败");
        return false;
    }

    // 连接信号
    connectSignals();

    // 应用配置
    applyCompressionConfig();
    applyNetworkConfig();

    m_initialized = true;
    logInfo("数据广播管理器初始化成功");

    return true;
}

void DataBroadcastManager::cleanup()
{
    if (!m_initialized) {
        return;
    }

    logInfo("清理数据广播管理器...");

    // 停止网络服务器
    stopNetworkServer();

    // 停止定时器
    if (m_statisticsTimer) {
        m_statisticsTimer->stop();
    }

    // 清理压缩管理器
    if (m_compressionManager) {
        m_compressionManager->cleanup();
        m_compressionManager.reset();
    }

    // 清理网络管理器
    if (m_networkManager) {
        m_networkManager->cleanup();
        m_networkManager.reset();
    }

    m_initialized = false;
    logInfo("数据广播管理器清理完成");
}

void DataBroadcastManager::updateConfig(const BroadcastConfig& config)
{
    m_config = config;

    if (m_initialized) {
        applyCompressionConfig();
        applyNetworkConfig();
    }

    emit configurationUpdated(config);
    logInfo("广播配置已更新");
}

BroadcastConfig DataBroadcastManager::getConfig() const
{
    return m_config;
}

// ========== 网络服务器管理 ==========

bool DataBroadcastManager::startNetworkServer(quint16 port)
{
    if (!m_initialized) {
        logError("数据广播管理器未初始化");
        return false;
    }

    if (!m_networkManager) {
        logError("网络管理器不可用");
        return false;
    }

    bool success = m_networkManager->startTcpServer(port);
    if (success) {
        logInfo(QString("网络服务器已启动，端口: %1").arg(port));
        emit networkServerStarted(port);
    } else {
        logError(QString("网络服务器启动失败，端口: %1").arg(port));
        emit networkServerError("服务器启动失败");
    }

    return success;
}

void DataBroadcastManager::stopNetworkServer()
{
    if (m_networkManager) {
        m_networkManager->stopServer();
        logInfo("网络服务器已停止");
        emit networkServerStopped();
    }
}

bool DataBroadcastManager::isNetworkServerRunning() const
{
    return m_networkManager && m_networkManager->isServerRunning();
}

// ========== 数据广播接口 ==========

#ifdef ROS_BUILD
void DataBroadcastManager::broadcastPointCloud(QSharedPointer<sensor_msgs::PointCloud2> cloudMsg)
{
    qDebug()<<"接收新点云数据";
    if (!m_initialized || m_broadcastPaused || !cloudMsg) {
        return;
    }

    if (!m_config.enablePointCloudBroadcast) {
        return;
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.pointCloudFramesReceived++;
        m_statistics.totalDataReceived += cloudMsg->data.size();
    }

    // 转换ROS数据
    CrossPlatformPointCloud pointCloud = convertROSPointCloud(*cloudMsg);
    qDebug()<<"转换成新点云数据";

    if (!validatePointCloudData(pointCloud)) {
        logWarning("点云数据验证失败");
        return;
    }

    // 处理点云数据
    processPointCloudData(pointCloud);
}

void DataBroadcastManager::broadcastCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg, int cameraId)
{
    if (!m_initialized || m_broadcastPaused || !imageMsg) {
        return;
    }

    if (cameraId < 0 || cameraId >= 4 || !m_config.enableImageBroadcast[cameraId]) {
        return;
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.imageFramesReceived[cameraId]++;
        m_statistics.totalDataReceived += imageMsg->data.size();
    }

    // 转换ROS数据
    QImage image = convertROSImage(*imageMsg);

    if (!validateImageData(image, cameraId)) {
        logWarning(QString("相机%1图像数据验证失败").arg(cameraId));
        return;
    }
    //qDebug()<<"输出成QImage格式完成";

    // 处理图像数据
    processImageData(image, cameraId);
}

void DataBroadcastManager::broadcastGPSData(NetDataGPSData myGPSMsg)
{

}

#endif

void DataBroadcastManager::broadcastPointCloud(const CrossPlatformPointCloud& pointCloud)
{
    if (!m_initialized || m_broadcastPaused) {
        return;
    }

    if (!m_config.enablePointCloudBroadcast) {
        return;
    }

    if (!validatePointCloudData(pointCloud)) {
        logWarning("点云数据验证失败");
        return;
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.pointCloudFramesReceived++;
        m_statistics.totalDataReceived += pointCloud.size() * sizeof(CrossPlatformPointCloud::Point);
    }

    // 处理点云数据
    processPointCloudData(pointCloud);
}

void DataBroadcastManager::broadcastImage(const QImage& image, int cameraId)
{
    if (!m_initialized || m_broadcastPaused) {
        return;
    }

    if (cameraId < 0 || cameraId >= 4 || !m_config.enableImageBroadcast[cameraId]) {
        return;
    }

    if (!validateImageData(image, cameraId)) {
        logWarning(QString("相机%1图像数据验证失败").arg(cameraId));
        return;
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.imageFramesReceived[cameraId]++;
        m_statistics.totalDataReceived += image.sizeInBytes();
    }

    // 处理图像数据
    processImageData(image, cameraId);
}

void DataBroadcastManager::broadcastCompressedData(const QByteArray& data, NetworkDataType dataType, int cameraId)
{
    if (!m_initialized || m_broadcastPaused || data.isEmpty()) {
        return;
    }

    // 直接发送已压缩的数据
    if (m_networkManager) {
        // 根据数据类型选择发送方法
        switch (dataType) {
        case NetworkDataType::POINTCLOUD_COMPRESSED:
            // 这里需要补充压缩信息结构
            break;
        case NetworkDataType::IMAGE_JPEG:
            // 发送JPEG数据
            break;
        case NetworkDataType::IMAGE_H264:
            // 发送H264数据
            break;
        default:
            m_networkManager->sendDataToAll(data);
            break;
        }
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.totalFramesSent++;
        m_statistics.totalBytesSent += data.size();
    }
}

// ========== 广播控制 ==========

void DataBroadcastManager::setClientStreamConfig(quint32 clientId, StreamType streamType, bool enabled)
{
    if (m_networkManager) {
        m_networkManager->enableClientStream(clientId, streamType, enabled);
        logInfo(QString("设置客户端 %1 流类型 %2: %3")
                    .arg(clientId).arg(streamType).arg(enabled ? "启用" : "禁用"));
    }
}

void DataBroadcastManager::setClientCameraSubscription(quint32 clientId, int cameraId, bool enabled)
{
    if (m_networkManager) {
        m_networkManager->setClientCameraSubscription(clientId, cameraId, enabled);
        logInfo(QString("设置客户端 %1 相机 %2 订阅: %3")
                    .arg(clientId).arg(cameraId).arg(enabled ? "启用" : "禁用"));
    }
}

void DataBroadcastManager::setClientCompressionLevel(quint32 clientId, CompressionLevel level)
{
    if (m_networkManager) {
        ClientStreamState state = m_networkManager->getClientStreamState(clientId);
        state.compressionLevel = level;
        m_networkManager->setClientStreamParameters(clientId, state);
        logInfo(QString("设置客户端 %1 压缩级别: %2").arg(clientId).arg(level));
    }
}

// ========== 状态和统计 ==========

BroadcastStatistics DataBroadcastManager::getStatistics() const
{
    QMutexLocker locker(&m_statsMutex);
    BroadcastStatistics stats = m_statistics;
    stats.uptimeSeconds = m_uptimeTimer.elapsed() / 1000;

    if (m_networkManager) {
        stats.activeClients = m_networkManager->getClientCount();
    }

    return stats;
}

CompressionManager::CompressionStats DataBroadcastManager::getCompressionStatistics() const
{
    if (m_compressionManager) {
        return m_compressionManager->getStats();
    }
    return CompressionManager::CompressionStats();
}

NetworkStatistics DataBroadcastManager::getNetworkStatistics() const
{
    if (m_networkManager) {
        return m_networkManager->getStatistics();
    }
    return NetworkStatistics();
}

void DataBroadcastManager::resetStatistics()
{
    QMutexLocker locker(&m_statsMutex);
    m_statistics = BroadcastStatistics();
    m_statistics.startTime = QDateTime::currentDateTime();
    m_uptimeTimer.restart();

    if (m_compressionManager) {
        m_compressionManager->resetStats();
    }

    logInfo("广播统计信息已重置");
}

// ========== 控制接口 ==========

void DataBroadcastManager::pauseBroadcast(bool pause)
{
    if (m_broadcastPaused != pause) {
        m_broadcastPaused = pause;

        if (pause) {
            logInfo("数据广播已暂停");
            emit broadcastPaused();
        } else {
            logInfo("数据广播已恢复");
            emit broadcastResumed();
        }
    }
}

void DataBroadcastManager::enablePointCloudBroadcast(bool enable)
{
    m_config.enablePointCloudBroadcast = enable;
    logInfo(QString("点云广播: %1").arg(enable ? "启用" : "禁用"));
}

void DataBroadcastManager::enableImageBroadcast(int cameraId, bool enable)
{
    if (cameraId >= 0 && cameraId < 4) {
        m_config.enableImageBroadcast[cameraId] = enable;
        logInfo(QString("相机%1广播: %2").arg(cameraId).arg(enable ? "启用" : "禁用"));
    }
}

void DataBroadcastManager::enableImageBroadcastAll(bool enable)
{
    for (int i = 0; i < 4; i++) {
        m_config.enableImageBroadcast[i] = enable;
    }
    logInfo(QString("所有相机广播: %1").arg(enable ? "启用" : "禁用"));
}

// ========== 诊断和调试 ==========

QString DataBroadcastManager::getDiagnosticsInfo() const
{
    QString info;
    QTextStream stream(&info);

    stream << "=== 数据广播管理器诊断信息 ===\n";
    stream << "初始化状态: " << (m_initialized ? "已初始化" : "未初始化") << "\n";
    stream << "广播状态: " << (m_broadcastPaused ? "暂停" : "运行中") << "\n";
    stream << "运行时间: " << (m_uptimeTimer.elapsed() / 1000) << " 秒\n";

    if (m_networkManager) {
        stream << "网络服务器: " << (m_networkManager->isServerRunning() ? "运行中" : "已停止") << "\n";
        stream << "连接客户端数: " << m_networkManager->getClientCount() << "\n";
    }

    BroadcastStatistics stats = getStatistics();
    stream << "接收点云帧数: " << stats.pointCloudFramesReceived << "\n";
    stream << "处理点云帧数: " << stats.pointCloudFramesProcessed << "\n";
    stream << "发送总帧数: " << stats.totalFramesSent << "\n";
    stream << "网络吞吐量: " << stats.networkThroughput << " MB/s\n";

    return info;
}

void DataBroadcastManager::printStatistics() const
{
    QString diagnostics = getDiagnosticsInfo();
    logInfo(diagnostics);
}

// ========== 槽函数实现 ==========

void DataBroadcastManager::onConfigurationChanged(const BroadcastConfig& newConfig)
{
    updateConfig(newConfig);
}

void DataBroadcastManager::onClientConnected(quint32 clientId, const QString& address, quint16 port)
{
    logInfo(QString("客户端连接: ID=%1, 地址=%2:%3").arg(clientId).arg(address).arg(port));
    emit clientConnected(clientId, QString("%1:%2").arg(address).arg(port));
}

void DataBroadcastManager::onClientDisconnected(quint32 clientId, const QString& reason)
{
    logInfo(QString("客户端断开: ID=%1, 原因=%2").arg(clientId).arg(reason));
    emit clientDisconnected(clientId, reason);
}

void DataBroadcastManager::onNetworkError(const QString& error)
{
    logError(QString("网络错误: %1").arg(error));
    emit networkError(error);
}

// ========== 数据处理槽 ==========

void DataBroadcastManager::onPointCloudCompressionCompleted(const CompressionResult& result)
{
    try {
        qDebug() << "处理点云压缩结果，成功:" << result.success;

        if (!result.success) {
            handleCompressionError(result.errorMessage);
            return;
        }

        // 发送压缩后的点云数据
        if (m_networkManager) {
            CompressedDataInfo info;
            info.originalSize = result.originalSize;
            info.compressedSize = result.compressedSize;
            info.compressionRatio = result.compressionRatio();
            info.quality = static_cast<CompressionLevel>(result.compressionMethod == "draco" ? MEDIUM_QUALITY : LOW_QUALITY);
            info.compressionMethod = result.compressionMethod;
            info.processingTime = result.processingTime;

            qDebug() << "发送压缩点云到网络管理器";
            m_networkManager->sendCompressedPointCloud(result.data, info);
            qDebug() << "压缩点云发送完成";
        }

        // 更新统计
        {
            QMutexLocker locker(&m_statsMutex);
            m_statistics.pointCloudFramesProcessed++;
            m_statistics.totalFramesSent++;
            m_statistics.totalBytesSent += result.compressedSize;
        }

        // 发送信号
        emit pointCloudBroadcasted(result.originalSize, result.compressedSize, result.compressionRatio());

        logInfo(QString("点云压缩完成: %1 -> %2 bytes (%.2f%%), 耗时: %3ms")
                    .arg(result.originalSize).arg(result.compressedSize)
                    .arg(result.compressionRatio()).arg(result.processingTime));

    } catch (const std::exception& e) {
        qCritical() << "点云压缩完成处理异常:" << e.what();
        // 不要让异常传播，避免影响服务器运行
    } catch (...) {
        qCritical() << "点云压缩完成处理发生未知异常";
    }
    /*
    if (!result.success) {
        handleCompressionError(result.errorMessage);
        return;
    }

    // 发送压缩后的点云数据
    if (m_networkManager) {
        CompressedDataInfo info;
        info.originalSize = result.originalSize;
        info.compressedSize = result.compressedSize;
        info.compressionRatio = result.compressionRatio();
        info.quality = static_cast<CompressionLevel>(result.compressionMethod == "draco" ? MEDIUM_QUALITY : LOW_QUALITY);
        info.compressionMethod = result.compressionMethod;
        info.processingTime = result.processingTime;

        m_networkManager->sendCompressedPointCloud(result.data, info);
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.pointCloudFramesProcessed++;
        m_statistics.totalFramesSent++;
        m_statistics.totalBytesSent += result.compressedSize;
    }

    // 发送信号
    emit pointCloudBroadcasted(result.originalSize, result.compressedSize, result.compressionRatio());

    logInfo(QString("点云压缩完成: %1 -> %2 bytes (%.2f%%), 耗时: %3ms")
                .arg(result.originalSize).arg(result.compressedSize)
                .arg(result.compressionRatio()).arg(result.processingTime));*/
}

void DataBroadcastManager::onImageCompressionCompleted(const CompressionResult& result)
{
    if (!result.success) {
        handleCompressionError(result.errorMessage);
        return;
    }

    // 发送压缩后的图像数据
    if (m_networkManager) {
        CompressedDataInfo info;
        info.originalSize = result.originalSize;
        info.compressedSize = result.compressedSize;
        info.compressionRatio = result.compressionRatio();
        info.quality = static_cast<CompressionLevel>(result.compressionMethod == "h264" ? MEDIUM_QUALITY : LOW_QUALITY);
        info.compressionMethod = result.compressionMethod;
        info.processingTime = result.processingTime;

        if (result.compressionMethod == "h264") {
            // H264数据使用专用接口
            m_networkManager->sendH264Frame(result.data, result.cameraId,
                                            result.isKeyFrame, result.frameSequence, info);
        } else {
            // JPEG数据使用通用接口
            m_networkManager->sendCompressedJPEG(result.data, result.cameraId, info);
        }
    }

    // 更新统计
    {
        QMutexLocker locker(&m_statsMutex);
        m_statistics.imageFramesProcessed[result.cameraId]++;
        m_statistics.totalFramesSent++;
        m_statistics.totalBytesSent += result.compressedSize;
    }

    // 发送信号
    emit imageBroadcasted(result.cameraId, result.originalSize,
                          result.compressedSize, result.compressionRatio());

    logInfo(QString("图像压缩完成: 相机%1, %2 -> %3 bytes (%.2f%%), 耗时: %4ms")
                .arg(result.cameraId).arg(result.originalSize).arg(result.compressedSize)
                .arg(result.compressionRatio()).arg(result.processingTime));
}

void DataBroadcastManager::onH264FrameGenerated(int cameraId, uint32_t frameSequence,
                                                const QByteArray& data, bool isKeyFrame)
{
    // 这个槽函数在新架构中可能不需要，因为H264帧会通过压缩完成槽处理
    Q_UNUSED(cameraId)
    Q_UNUSED(frameSequence)
    Q_UNUSED(data)
    Q_UNUSED(isKeyFrame)

    emit h264FrameBroadcasted(cameraId, frameSequence, isKeyFrame);
}

void DataBroadcastManager::onStatisticsUpdateTimer()
{
    updateStatistics();
    updateProcessingRates();

    BroadcastStatistics stats = getStatistics();
    emit statisticsUpdated(stats);

    if (m_compressionManager) {
        CompressionManager::CompressionStats compStats = m_compressionManager->getStats();
        emit compressionStatisticsUpdated(compStats);
    }

    if (m_networkManager) {
        NetworkStatistics netStats = m_networkManager->getStatistics();
        emit networkStatisticsUpdated(netStats);
    }
}

// ========== 私有方法实现 ==========

bool DataBroadcastManager::setupCompressionManager()
{
    m_compressionManager = std::make_unique<CompressionManager>(this);

    if (!m_compressionManager->initialize()) {
        logError("压缩管理器初始化失败");
        return false;
    }

    logInfo("压缩管理器设置成功");
    return true;
}

bool DataBroadcastManager::setupNetworkManager()
{
    m_networkManager = std::make_unique<NetworkManager>(this);

    if (!m_networkManager->initialize()) {
        logError("网络管理器初始化失败");
        return false;
    }

    logInfo("网络管理器设置成功");
    return true;
}

void DataBroadcastManager::connectSignals()
{
    if (!m_compressionManager || !m_networkManager) {
        return;
    }

    // 连接压缩管理器信号
    connect(m_compressionManager.get(), &CompressionManager::pointCloudCompressionCompleted,
            this, &DataBroadcastManager::onPointCloudCompressionCompleted);
    connect(m_compressionManager.get(), &CompressionManager::imageCompressionCompleted,
            this, &DataBroadcastManager::onImageCompressionCompleted);
    connect(m_compressionManager.get(), &CompressionManager::h264FrameGenerated,
            this, &DataBroadcastManager::onH264FrameGenerated);
    connect(m_compressionManager.get(), &CompressionManager::compressionError,
            this, &DataBroadcastManager::handleCompressionError);

    // 连接网络管理器信号
    connect(m_networkManager.get(), &NetworkManager::serverStarted,
            this, &DataBroadcastManager::networkServerStarted);
    connect(m_networkManager.get(), &NetworkManager::serverStopped,
            this, &DataBroadcastManager::networkServerStopped);
    connect(m_networkManager.get(), &NetworkManager::serverError,
            this, &DataBroadcastManager::networkServerError);
    connect(m_networkManager.get(), &NetworkManager::clientConnected,
            this, &DataBroadcastManager::onClientConnected);
    connect(m_networkManager.get(), &NetworkManager::clientDisconnected,
            this, &DataBroadcastManager::onClientDisconnected);

    logInfo("信号连接完成");
}

void DataBroadcastManager::processPointCloudData(const CrossPlatformPointCloud& pointCloud)
{
    if (!m_compressionManager) {
        return;
    }

    // 异步压缩点云数据
    m_compressionManager->compressPointCloudAsync(
        pointCloud,
        m_config.pointCloudCompressionLevel,
        [this](const CompressionResult& result) {
            // 压缩完成后通过槽函数处理
            QMetaObject::invokeMethod(this, "onPointCloudCompressionCompleted",
                                      Qt::QueuedConnection, Q_ARG(CompressionResult, result));
        }
        );

    // 更新处理时间
    m_lastPointCloudTime.restart();
}

void DataBroadcastManager::processImageData(const QImage& image, int cameraId)
{
    if (!m_compressionManager) {
        return;
    }

    // 异步压缩图像数据
    m_compressionManager->compressImageAsync(
        image,
        cameraId,
        m_config.imageCodec,
        m_config.imageCompressionLevel,
        [this](const CompressionResult& result) {
            // 压缩完成后通过槽函数处理
            QMetaObject::invokeMethod(this, "onImageCompressionCompleted",
                                      Qt::QueuedConnection, Q_ARG(CompressionResult, result));
        }
        );

    // 更新处理时间
    if (cameraId >= 0 && cameraId < 4) {
        m_lastFrameTime[cameraId].restart();
    }
}

#ifdef ROS_BUILD
CrossPlatformPointCloud DataBroadcastManager::convertROSPointCloud(const sensor_msgs::PointCloud2& rosCloud)
{
    CrossPlatformPointCloud cloud;
    cloud.timestamp.seconds = rosCloud.header.stamp.sec;
    cloud.timestamp.microseconds = rosCloud.header.stamp.nsec / 1000;
    cloud.frameId = QString::fromStdString(rosCloud.header.frame_id);

    // 这里需要根据实际的ROS PointCloud2格式进行转换
    // 简化实现，实际应该解析fields等信息
    int pointStep = rosCloud.point_step;
    int numPoints = rosCloud.width * rosCloud.height;

    cloud.points.reserve(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        const uint8_t* ptr = &rosCloud.data[i * pointStep];

        // 假设是标准的XYZ+RGB格式
        float x = *reinterpret_cast<const float*>(ptr + 0);
        float y = *reinterpret_cast<const float*>(ptr + 4);
        float z = *reinterpret_cast<const float*>(ptr + 8);

        // RGB可能在不同的偏移位置
        uint8_t r = 255, g = 255, b = 255;
        if (pointStep >= 16) {
            uint32_t rgb = *reinterpret_cast<const uint32_t*>(ptr + 12);
            r = (rgb >> 16) & 0xFF;
            g = (rgb >> 8) & 0xFF;
            b = rgb & 0xFF;
        }

        cloud.points.append(CrossPlatformPointCloud::Point(x, y, z, r, g, b));
    }

    return cloud;
}

QImage DataBroadcastManager::convertROSImage(const sensor_msgs::Image& rosImage)
{
    // 根据ROS图像格式转换为QImage
    QImage::Format format = QImage::Format_RGB888;

    if (rosImage.encoding == "bgr8") {
        format = QImage::Format_RGB888;
    } else if (rosImage.encoding == "rgb8") {
        format = QImage::Format_RGB888;
    } else if (rosImage.encoding == "mono8") {
        format = QImage::Format_Grayscale8;
    }

    QImage image(rosImage.data.data(), rosImage.width, rosImage.height,
                 rosImage.step, format);

    // 如果是BGR格式，需要转换为RGB
    if (rosImage.encoding == "bgr8") {
        image = image.rgbSwapped();
    }

    // 灰度图转RGB
    if (rosImage.encoding == "mono8") {
        image = image.convertToFormat(QImage::Format_RGB888);
    }
    QImage baseImage = image.copy();
    QImage result = convertTo1080p(baseImage);
    return result;

    /*// 转换为标准1080p (1920x1080)
    QSize targetSize(1920, 1080);
    QImage result;
    if (image.width() != targetSize.width() || image.height() != targetSize.height()) {
        qDebug() << "转换图像尺寸: " << image.width() << "x" << image.height()
            << " -> " << targetSize.width() << "x" << targetSize.height();

        // 选择缩放模式
        //Qt::AspectRatioMode aspectMode = Qt::IgnoreAspectRatio;  // 拉伸到目标尺寸
        // 或者使用 Qt::KeepAspectRatio 保持宽高比，可能有黑边
        // 或者使用 Qt::KeepAspectRatioByExpanding 保持宽高比，可能裁剪
        Qt::AspectRatioMode aspectMode=Qt::KeepAspectRatio;

        result = image.scaled(targetSize, aspectMode, Qt::SmoothTransformation);
    } else {
        result = image.copy();
    }

    qDebug() << "最终图像: " << result.width() << "x" << result.height();
    qDebug() << "=== ROS图像转换完成 ===";
    return result;*/

    //qDebug()<<"Image转换成功"<<image.width()<<image.height();
    //return image.copy(); // 深拷贝确保数据安全
}
#endif

QImage DataBroadcastManager::convertTo1080p(const QImage& sourceImage)
{
    const QSize targetSize(1920, 1080);
    const double targetAspect = 1920.0 / 1080.0; // 16:9

    if (sourceImage.width() == targetSize.width() &&
        sourceImage.height() == targetSize.height()) {
        return sourceImage; // 已经是1080p
    }

    double sourceAspect = (double)sourceImage.width() / sourceImage.height();

    QImage result;

    if (qAbs(sourceAspect - targetAspect) < 0.01) {
        // 宽高比接近，直接缩放
        result = sourceImage.scaled(targetSize, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
        qDebug() << "直接缩放到1080p";

    } else if (sourceAspect > targetAspect) {
        // 源图像更宽，需要裁剪宽度
        int cropWidth = sourceImage.height() * targetAspect;
        int cropX = (sourceImage.width() - cropWidth) / 2;

        QRect cropRect(cropX, 0, cropWidth, sourceImage.height());
        QImage cropped = sourceImage.copy(cropRect);
        result = cropped.scaled(targetSize, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

        qDebug() << "裁剪宽度后缩放到1080p, 裁剪区域:" << cropRect;

    } else {
        // 源图像更高，需要裁剪高度
        int cropHeight = sourceImage.width() / targetAspect;
        int cropY = (sourceImage.height() - cropHeight) / 2;

        QRect cropRect(0, cropY, sourceImage.width(), cropHeight);
        QImage cropped = sourceImage.copy(cropRect);
        result = cropped.scaled(targetSize, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

        qDebug() << "裁剪高度后缩放到1080p, 裁剪区域:" << cropRect;
    }

    return result;
}

// ========== 统计更新 ==========

void DataBroadcastManager::updateStatistics()
{
    QMutexLocker locker(&m_statsMutex);

    // 更新运行时间
    m_statistics.uptimeSeconds = m_uptimeTimer.elapsed() / 1000;

    // 计算网络吞吐量
    if (m_networkManager) {
        NetworkStatistics netStats = m_networkManager->getStatistics();
        qint64 deltaTime = 1000; // 1秒
        if (deltaTime > 0) {
            m_statistics.networkThroughput =
                static_cast<float>(netStats.totalBytesSent) / (1024.0f * 1024.0f * deltaTime / 1000.0f);
        }
        m_statistics.activeClients = netStats.activeConnections;
    }

    // 计算平均处理时间
    if (m_statistics.pointCloudFramesProcessed > 0) {
        // 这里可以添加更详细的处理时间统计
    }
}

void DataBroadcastManager::updateProcessingRates()
{
    QMutexLocker locker(&m_statsMutex);

    // 计算点云处理频率
    qint64 pointCloudElapsed = m_lastPointCloudTime.elapsed();
    if (pointCloudElapsed > 0) {
        m_statistics.pointCloudProcessingRate = 1000.0f / pointCloudElapsed;
    }

    // 计算图像处理频率
    for (int i = 0; i < 4; i++) {
        qint64 imageElapsed = m_lastFrameTime[i].elapsed();
        if (imageElapsed > 0) {
            m_statistics.imageProcessingRate[i] = 1000.0f / imageElapsed;
        }
    }
}

// ========== 配置应用 ==========

void DataBroadcastManager::applyCompressionConfig()
{
    if (!m_compressionManager) {
        return;
    }

    // 设置Draco参数
    if (m_compressionManager->isDracoAvailable()) {
        // 根据质量级别设置量化位数
        int positionBits = 11, colorBits = 8;
        switch (m_config.pointCloudCompressionLevel) {
        case HIGH_QUALITY:
            positionBits = 14; colorBits = 10;
            break;
        case MEDIUM_QUALITY:
            positionBits = 11; colorBits = 8;
            break;
        case LOW_QUALITY:
            positionBits = 8; colorBits = 6;
            break;
        case MOBILE_OPTIMIZED:
            positionBits = 6; colorBits = 4;
            break;
        }
        m_compressionManager->setDracoQuantizationBits(positionBits, colorBits);
        m_compressionManager->setDracoCompressionSpeed(5); // 中等速度
    }

    // 设置H264参数
    if (m_compressionManager->isH264Available()) {
        for (int i = 0; i < 4; i++) {
            if (m_config.enableImageBroadcast[i]) {
                m_compressionManager->setH264Parameters(i, m_config.h264Bitrate,
                                                        m_config.imageFrameRate, m_config.h264GOP);

                // 初始化编码器
                QSize imageSize = m_config.imageMaxSize;
                m_compressionManager->initializeH264Encoder(i, imageSize.width(),
                                                            imageSize.height(), m_config.imageFrameRate,
                                                            m_config.h264Bitrate);
            }
        }
    }

    logInfo("压缩配置已应用");
}

void DataBroadcastManager::applyNetworkConfig()
{
    if (!m_networkManager) {
        return;
    }

    // 设置网络参数
    m_networkManager->setMaxConnections(m_config.maxClients);
    m_networkManager->setConnectionTimeout(m_config.connectionTimeout);

    // 设置H264缓冲参数
    m_networkManager->setH264FrameBufferSize(3); // 缓冲3帧
    m_networkManager->setH264FlushInterval(1000 / m_config.imageFrameRate); // 根据帧率设置刷新间隔

    // 设置心跳和状态广播
    if (m_config.enableHeartbeat) {
        m_networkManager->enableHeartbeat(m_config.heartbeatInterval);
    } else {
        m_networkManager->disableHeartbeat();
    }

    if (m_config.enableStatusBroadcast) {
        m_networkManager->enableStatusBroadcast(m_config.statusInterval);
    } else {
        m_networkManager->disableStatusBroadcast();
    }

    logInfo("网络配置已应用");
}

// ========== 数据验证 ==========

bool DataBroadcastManager::validateImageData(const QImage& image, int cameraId) const
{
    Q_UNUSED(cameraId)

    if (image.isNull()) {
        return false;
    }

    if (image.width() <= 0 || image.height() <= 0) {
        return false;
    }

    // 检查图像大小限制
    if (image.width() > m_config.imageMaxSize.width() ||
        image.height() > m_config.imageMaxSize.height()) {
        return false;
    }

    return true;
}

bool DataBroadcastManager::validatePointCloudData(const CrossPlatformPointCloud& pointCloud) const
{
    if (pointCloud.isEmpty()) {
        return false;
    }

    // 检查点云范围限制
    if (m_config.pointCloudMaxRange > 0) {
        for (const auto& point : pointCloud.points) {
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance > m_config.pointCloudMaxRange) {
                // 如果有点超出范围，可以选择过滤或拒绝整个点云
                // 这里选择继续处理，但可以根据需要修改
            }
        }
    }

    return true;
}

// ========== 错误处理 ==========

void DataBroadcastManager::handleCompressionError(const QString& error)
{
    QMutexLocker locker(&m_statsMutex);
    m_statistics.compressionErrors++;

    logError(QString("压缩错误: %1").arg(error));
    emit compressionError(error);
}

void DataBroadcastManager::handleNetworkError(const QString& error)
{
    QMutexLocker locker(&m_statsMutex);
    m_statistics.networkErrors++;

    logError(QString("网络错误: %1").arg(error));
    emit networkError(error);
}

// ========== 日志记录 ==========

void DataBroadcastManager::logInfo(const QString& message) const
{
    qCInfo(broadcastLog) << message;
}

void DataBroadcastManager::logWarning(const QString& message) const
{
    qCWarning(broadcastLog) << message;
}

void DataBroadcastManager::logError(const QString& message) const
{
    qCCritical(broadcastLog) << message;
}
