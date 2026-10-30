#include "navcar_project_server/h264_cache_manager.h"
#include <QDebug>
#include <algorithm>

Q_LOGGING_CATEGORY(h264CacheLog, "h264.cache")

H264CacheManager::H264CacheManager(QObject *parent)
    : QObject(parent)
    , m_maxMemoryUsage(10 * 1024 * 1024)  // 10MB
    , m_keyFrameMaxAge(5000)               // 5秒
    , m_maxClientsPerCamera(10)
    , m_cleanupTimer(nullptr)
    , m_cleanupInterval(2000)              // 2秒
    , m_keyFrameHitCount(0)
    , m_keyFrameMissCount(0)
    , m_initialized(false)
{
}

H264CacheManager::~H264CacheManager()
{
    cleanup();
}

bool H264CacheManager::initialize()
{
    if (m_initialized) {
        qCWarning(h264CacheLog) << "H264CacheManager已经初始化";
        return true;
    }
    
    try {
        // 初始化清理定时器
        m_cleanupTimer = new QTimer(this);
        connect(m_cleanupTimer, &QTimer::timeout, this, &H264CacheManager::performCacheCleanup);
        m_cleanupTimer->start(m_cleanupInterval);
        
        // 预分配相机缓存
        for (int i = 0; i < 4; i++) {
            m_latestKeyFrames[i] = KeyFrameCache();
        }
        
        m_initialized = true;
        
        qCInfo(h264CacheLog) << "H264CacheManager初始化成功";
        qCInfo(h264CacheLog) << "最大内存使用:" << (m_maxMemoryUsage / 1024 / 1024) << "MB";
        qCInfo(h264CacheLog) << "I帧最大生存时间:" << m_keyFrameMaxAge << "ms";
        qCInfo(h264CacheLog) << "缓存清理间隔:" << m_cleanupInterval << "ms";
        
        return true;
        
    } catch (const std::exception& e) {
        qCCritical(h264CacheLog) << "H264CacheManager初始化失败:" << e.what();
        return false;
    }
}

void H264CacheManager::cleanup()
{
    if (!m_initialized) return;
    
    qCInfo(h264CacheLog) << "清理H264CacheManager";
    
    // 停止定时器
    if (m_cleanupTimer) {
        m_cleanupTimer->stop();
        m_cleanupTimer->deleteLater();
        m_cleanupTimer = nullptr;
    }
    
    // 清理缓存数据
    {
        QMutexLocker cacheLocker(&m_cacheMutex);
        m_latestKeyFrames.clear();
    }
    
    // 清理客户端状态
    {
        QMutexLocker clientLocker(&m_clientMutex);
        m_clientStates.clear();
    }
    
    m_initialized = false;
    qCInfo(h264CacheLog) << "H264CacheManager清理完成";
}

// ========== 核心缓存接口实现 ==========

void H264CacheManager::cacheKeyFrame(int cameraId, const QByteArray& frameData, 
                                    uint32_t sequence, CompressionLevel quality)
{
    if (!m_initialized || frameData.isEmpty()) {
        return;
    }
    
    // 验证相机ID
    if (cameraId < 0 || cameraId >= 4) {
        qCWarning(h264CacheLog) << "无效的相机ID:" << cameraId;
        return;
    }
    
    // 验证是否为I帧
    if (!isKeyFrame(frameData)) {
        qCWarning(h264CacheLog) << "相机" << cameraId << "尝试缓存非I帧数据";
        return;
    }
    
    QMutexLocker locker(&m_cacheMutex);
    
    try {
        // 创建新的I帧缓存
        KeyFrameCache newFrame;
        newFrame.frameData = frameData;
        newFrame.sequence = sequence;
        newFrame.timestamp = QDateTime::currentMSecsSinceEpoch();
        newFrame.quality = quality;
        newFrame.dataSize = frameData.size();
        
        // 更新缓存
        KeyFrameCache& existingFrame = m_latestKeyFrames[cameraId];
        
        // 检查是否为更新的帧
        if (!existingFrame.isValid() || sequence > existingFrame.sequence) {
            existingFrame = newFrame;
            
            qCDebug(h264CacheLog) << QString("缓存相机%1 I帧: 序号=%2, 大小=%3KB")
                                         .arg(cameraId).arg(sequence).arg(frameData.size() / 1024);
            
            // 检查内存限制
            if (calculateTotalMemoryUsage() > m_maxMemoryUsage) {
                qCWarning(h264CacheLog) << "内存使用超限，执行清理";
                enforceMemoryLimit();
            }
            
            emit keyFrameCached(cameraId, sequence, frameData.size());
            
        } else {
            qCDebug(h264CacheLog) << QString("相机%1 I帧序号%2过旧，忽略 (当前:%3)")
                                         .arg(cameraId).arg(sequence).arg(existingFrame.sequence);
        }
        
    } catch (const std::exception& e) {
        qCCritical(h264CacheLog) << "缓存I帧异常:" << e.what();
    }
}

QByteArray H264CacheManager::getLatestKeyFrame(int cameraId, CompressionLevel quality)
{
    if (!m_initialized || cameraId < 0 || cameraId >= 4) {
        return QByteArray();
    }
    
    QMutexLocker locker(&m_cacheMutex);
    
    auto it = m_latestKeyFrames.find(cameraId);
    if (it != m_latestKeyFrames.end() && it->isValid()) {
        const KeyFrameCache& frame = it.value();
        
        // 检查帧是否过期
        if (!frame.isExpired(m_keyFrameMaxAge)) {
            m_keyFrameHitCount++;
            
            qCDebug(h264CacheLog) << QString("命中相机%1 I帧缓存: 序号=%2, 年龄=%3ms")
                                         .arg(cameraId).arg(frame.sequence)
                                         .arg(QDateTime::currentMSecsSinceEpoch() - frame.timestamp);
            
            return frame.frameData;
        } else {
            qCWarning(h264CacheLog) << QString("相机%1 I帧已过期: 年龄=%2ms")
                                           .arg(cameraId)
                                           .arg(QDateTime::currentMSecsSinceEpoch() - frame.timestamp);
        }
    }
    
    m_keyFrameMissCount++;
    qCDebug(h264CacheLog) << "相机" << cameraId << "I帧缓存未命中";
    
    return QByteArray();
}

bool H264CacheManager::isKeyFrame(const QByteArray& h264Data)
{
    if (h264Data.size() < 5) {
        return false;
    }
    
    try {
        uint8_t nalType;
        size_t nalStart;
        
        if (analyzeNALUnit(h264Data, nalType, nalStart)) {
            // NAL单元类型5 = IDR帧 (I帧)
            return (nalType == 5);
        }
        
    } catch (const std::exception& e) {
        qCWarning(h264CacheLog) << "分析NAL单元异常:" << e.what();
    }
    
    return false;
}

// ========== 客户端管理实现 ==========

void H264CacheManager::handleNewClient(uint32_t clientId, CompressionLevel preferredQuality)
{
    if (!m_initialized) return;
    
    QMutexLocker locker(&m_clientMutex);
    
    // 创建新的客户端状态
    ClientH264State newState;
    newState.preferredQuality = preferredQuality;
    
    m_clientStates[clientId] = newState;
    
    qCInfo(h264CacheLog) << QString("新客户端连接: ID=%1, 偏好质量=%2")
                                .arg(clientId).arg(preferredQuality);
    
    // 为所有相机发送需要I帧的信号
    for (int cameraId = 0; cameraId < 4; cameraId++) {
        emit clientKeyFrameNeeded(clientId, cameraId);
    }
}

void H264CacheManager::handleClientDisconnected(uint32_t clientId)
{
    QMutexLocker locker(&m_clientMutex);
    
    if (m_clientStates.remove(clientId) > 0) {
        qCInfo(h264CacheLog) << "客户端断开连接:" << clientId;
    }
}

void H264CacheManager::handlePacketLoss(uint32_t clientId, int cameraId, uint32_t lostFrameId)
{
    if (!m_initialized || cameraId < 0 || cameraId >= 4) return;
    
    QMutexLocker locker(&m_clientMutex);
    
    auto it = m_clientStates.find(clientId);
    if (it != m_clientStates.end()) {
        ClientH264State& state = it.value();
        
        state.consecutiveLostFrames[cameraId]++;
        state.needsKeyFrame[cameraId] = true;
        
        qCWarning(h264CacheLog) << QString("客户端%1相机%2丢包: 帧=%3, 连续丢包=%4")
                                       .arg(clientId).arg(cameraId).arg(lostFrameId)
                                       .arg(state.consecutiveLostFrames[cameraId]);
        
        // 根据相机优先级决定响应速度
        CameraPriority priority = getCameraPriority(cameraId);
        if (priority == FRONT_CAMERA || state.consecutiveLostFrames[cameraId] >= 3) {
            emit clientKeyFrameNeeded(clientId, cameraId);
        }
    }
}

void H264CacheManager::updateClientFrameStatus(uint32_t clientId, int cameraId, uint32_t frameId)
{
    if (cameraId < 0 || cameraId >= 4) return;
    
    QMutexLocker locker(&m_clientMutex);
    
    auto it = m_clientStates.find(clientId);
    if (it != m_clientStates.end()) {
        ClientH264State& state = it.value();
        
        state.lastReceivedFrame[cameraId] = frameId;
        state.lastFrameTime[cameraId] = QDateTime::currentMSecsSinceEpoch();
        state.consecutiveLostFrames[cameraId] = 0;  // 重置丢包计数
        state.needsKeyFrame[cameraId] = false;      // 清除I帧需求
    }
}

bool H264CacheManager::clientNeedsKeyFrame(uint32_t clientId, int cameraId)
{
    if (cameraId < 0 || cameraId >= 4) return false;
    
    QMutexLocker locker(&m_clientMutex);
    
    auto it = m_clientStates.find(clientId);
    if (it != m_clientStates.end()) {
        return it.value().needsKeyFrame[cameraId];
    }
    
    return true; // 未知客户端默认需要I帧
}

// ========== 私有方法实现 ==========

bool H264CacheManager::analyzeNALUnit(const QByteArray& data, uint8_t& nalType, size_t& nalStart)
{
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data.constData());
    const size_t size = data.size();
    
    // 查找NAL头: 0x00 0x00 0x00 0x01 或 0x00 0x00 0x01
    for (size_t i = 0; i < size - 4; i++) {
        if (bytes[i] == 0x00 && bytes[i+1] == 0x00) {
            if (bytes[i+2] == 0x00 && bytes[i+3] == 0x01) {
                // 4字节起始码
                nalStart = i + 4;
                if (nalStart < size) {
                    nalType = bytes[nalStart] & 0x1F;
                    return true;
                }
            } else if (bytes[i+2] == 0x01) {
                // 3字节起始码
                nalStart = i + 3;
                if (nalStart < size) {
                    nalType = bytes[nalStart] & 0x1F;
                    return true;
                }
            }
        }
    }
    
    return false;
}

size_t H264CacheManager::calculateTotalMemoryUsage() const
{
    size_t total = 0;
    
    for (auto it = m_latestKeyFrames.begin(); it != m_latestKeyFrames.end(); ++it) {
        if (it->isValid()) {
            total += it->dataSize;
        }
    }
    
    // 加上客户端状态的内存使用
    total += m_clientStates.size() * sizeof(ClientH264State);
    
    return total;
}

void H264CacheManager::enforceMemoryLimit()
{
    size_t currentUsage = calculateTotalMemoryUsage();
    
    if (currentUsage <= m_maxMemoryUsage) {
        return;
    }
    
    qCWarning(h264CacheLog) << QString("内存使用超限: %1MB > %2MB")
                                   .arg(currentUsage / 1024 / 1024)
                                   .arg(m_maxMemoryUsage / 1024 / 1024);
    
    // 按优先级清理：后向 > 左右 > 前向
    std::vector<int> cleanupOrder = {REAR_CAMERA, LEFT_CAMERA, RIGHT_CAMERA, FRONT_CAMERA};
    
    for (int cameraId : cleanupOrder) {
        if (calculateTotalMemoryUsage() <= m_maxMemoryUsage * 0.8) {
            break;
        }
        
        auto it = m_latestKeyFrames.find(cameraId);
        if (it != m_latestKeyFrames.end() && it->isValid()) {
            qCInfo(h264CacheLog) << "清理相机" << cameraId << "的I帧缓存以释放内存";
            it.value() = KeyFrameCache(); // 重置为空
        }
    }
    
    emit memoryLimitExceeded(currentUsage, m_maxMemoryUsage);
}

void H264CacheManager::performCacheCleanup()
{
    if (!m_initialized) return;
    
    try {
        // 清理过期帧
        cleanupExpiredFrames();
        
        // 清理非活跃客户端
        cleanupInactiveClients();
        
        // 更新统计信息
        updateStats();
        
    } catch (const std::exception& e) {
        qCWarning(h264CacheLog) << "缓存清理异常:" << e.what();
    }
}

void H264CacheManager::cleanupExpiredFrames()
{
    QMutexLocker locker(&m_cacheMutex);
    
    int expiredCount = 0;
    
    for (auto it = m_latestKeyFrames.begin(); it != m_latestKeyFrames.end(); ++it) {
        if (it->isValid() && it->isExpired(m_keyFrameMaxAge)) {
            qCDebug(h264CacheLog) << QString("清理相机%1过期I帧: 年龄=%2ms")
                                         .arg(it.key())
                                         .arg(QDateTime::currentMSecsSinceEpoch() - it->timestamp);
            
            it.value() = KeyFrameCache(); // 重置为空
            expiredCount++;
        }
    }
    
    if (expiredCount > 0) {
        qCInfo(h264CacheLog) << "清理了" << expiredCount << "个过期I帧";
    }
}

void H264CacheManager::cleanupInactiveClients()
{
    QMutexLocker locker(&m_clientMutex);
    
    const qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    const qint64 inactiveThreshold = 30000; // 30秒无活动
    
    auto it = m_clientStates.begin();
    while (it != m_clientStates.end()) {
        bool isInactive = true;
        
        // 检查所有相机是否都长时间无活动
        for (int i = 0; i < 4; i++) {
            if (it->lastFrameTime[i] > 0 && 
                (currentTime - it->lastFrameTime[i]) < inactiveThreshold) {
                isInactive = false;
                break;
            }
        }
        
        if (isInactive && (currentTime - it->connectionTime) > inactiveThreshold) {
            qCInfo(h264CacheLog) << "清理非活跃客户端:" << it.key();
            it = m_clientStates.erase(it);
        } else {
            ++it;
        }
    }
}

void H264CacheManager::updateStats()
{
    QMutexLocker statsLocker(&m_statsMutex);
    QMutexLocker cacheLocker(&m_cacheMutex);
    QMutexLocker clientLocker(&m_clientMutex);
    
    // 更新统计信息
    m_stats.totalKeyFramesCached = 0;
    m_stats.totalMemoryUsage = 0;
    m_stats.oldestFrameAge = 0;
    
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    qint64 oldestTime = currentTime;
    
    for (auto it = m_latestKeyFrames.begin(); it != m_latestKeyFrames.end(); ++it) {
        if (it->isValid()) {
            m_stats.totalKeyFramesCached++;
            m_stats.totalMemoryUsage += it->dataSize;
            
            if (it->timestamp < oldestTime) {
                oldestTime = it->timestamp;
            }
        }
    }
    
    m_stats.oldestFrameAge = currentTime - oldestTime;
    m_stats.activeClients = m_clientStates.size();
    
    // 计算命中率
    int totalRequests = m_keyFrameHitCount + m_keyFrameMissCount;
    if (totalRequests > 0) {
        m_stats.keyFrameHitRate = (m_keyFrameHitCount * 100) / totalRequests;
    }
    
    emit cacheStatsUpdated(m_stats);
}

H264CacheManager::CacheStats H264CacheManager::getStats() const
{
    QMutexLocker locker(&m_statsMutex);
    return m_stats;
}

void H264CacheManager::setCleanupInterval(int intervalMs)
{
    m_cleanupInterval = qMax(1000, intervalMs); // 最小1秒
    
    if (m_cleanupTimer) {
        m_cleanupTimer->setInterval(m_cleanupInterval);
    }
    
    qCInfo(h264CacheLog) << "缓存清理间隔设置为:" << m_cleanupInterval << "ms";
}
