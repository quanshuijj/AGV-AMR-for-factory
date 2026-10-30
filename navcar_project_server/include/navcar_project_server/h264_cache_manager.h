#ifndef H264_CACHE_MANAGER_H
#define H264_CACHE_MANAGER_H

#include <QObject>
#include <QByteArray>
#include <QMap>
#include <QMutex>
#include <QDateTime>
#include <QTimer>
#include <QLoggingCategory>

#include"networkprotocol.h"

Q_DECLARE_LOGGING_CATEGORY(h264CacheLog)

// 相机优先级定义
enum CameraPriority {
    FRONT_CAMERA = 0,    // 最高优先级 - 前向相机
    LEFT_CAMERA = 1,     // 中等优先级 - 左侧相机
    RIGHT_CAMERA = 2,    // 中等优先级 - 右侧相机
    REAR_CAMERA = 3      // 最低优先级 - 后向相机
};

// I帧缓存结构
struct KeyFrameCache {
    QByteArray frameData;           // I帧数据
    uint32_t sequence;              // 帧序号
    qint64 timestamp;               // 时间戳
    CompressionLevel quality;       // 压缩质量
    size_t dataSize;                // 数据大小
    
    KeyFrameCache() : sequence(0), timestamp(0), quality(MEDIUM_QUALITY), dataSize(0) {}
    
    bool isValid() const {
        return !frameData.isEmpty() && timestamp > 0;
    }
    
    bool isExpired(qint64 maxAge = 5000) const {
        return (QDateTime::currentMSecsSinceEpoch() - timestamp) > maxAge;
    }
};

// 客户端H.264状态
struct ClientH264State {
    uint32_t lastReceivedFrame[4];      // 每个相机的最后接收帧序号
    bool needsKeyFrame[4];              // 每个相机是否需要I帧
    qint64 lastFrameTime[4];            // 每个相机最后帧时间
    int consecutiveLostFrames[4];       // 连续丢帧计数
    CompressionLevel preferredQuality;   // 客户端偏好质量
    qint64 connectionTime;              // 连接时间
    
    ClientH264State() : preferredQuality(MEDIUM_QUALITY), connectionTime(0) {
        for (int i = 0; i < 4; i++) {
            lastReceivedFrame[i] = 0;
            needsKeyFrame[i] = true;  // 新客户端都需要I帧
            lastFrameTime[i] = 0;
            consecutiveLostFrames[i] = 0;
        }
        connectionTime = QDateTime::currentMSecsSinceEpoch();
    }
    
    // 获取相机优先级权重
    float getCameraPriorityWeight(int cameraId) const {
        switch (cameraId) {
        case FRONT_CAMERA: return 1.0f;    // 最高优先级
        case LEFT_CAMERA:  return 0.8f;    // 高优先级
        case RIGHT_CAMERA: return 0.8f;    // 高优先级
        case REAR_CAMERA:  return 0.6f;    // 中等优先级
        default: return 0.5f;
        }
    }
};

class H264CacheManager : public QObject
{
    Q_OBJECT
    
public:
    explicit H264CacheManager(QObject *parent = nullptr);
    ~H264CacheManager();
    
    // 初始化和清理
    bool initialize();
    void cleanup();
    
    // ========== 核心缓存接口 ==========
    
    // 缓存I帧
    void cacheKeyFrame(int cameraId, const QByteArray& frameData, uint32_t sequence, 
                      CompressionLevel quality = MEDIUM_QUALITY);
    
    // 获取最新I帧
    QByteArray getLatestKeyFrame(int cameraId, CompressionLevel quality = MEDIUM_QUALITY);
    
    // 检查是否为I帧
    bool isKeyFrame(const QByteArray& h264Data);
    
    // ========== 客户端管理 ==========
    
    // 新客户端连接处理
    void handleNewClient(uint32_t clientId, CompressionLevel preferredQuality = MEDIUM_QUALITY);
    
    // 客户端断开连接
    void handleClientDisconnected(uint32_t clientId);
    
    // 处理客户端丢包
    void handlePacketLoss(uint32_t clientId, int cameraId, uint32_t lostFrameId);
    
    // 更新客户端帧状态
    void updateClientFrameStatus(uint32_t clientId, int cameraId, uint32_t frameId);
    
    // 检查客户端是否需要I帧
    bool clientNeedsKeyFrame(uint32_t clientId, int cameraId);
    
    // ========== 状态查询 ==========
    
    // 获取缓存统计信息
    struct CacheStats {
        int totalKeyFramesCached;       // 总缓存I帧数
        size_t totalMemoryUsage;        // 总内存使用量
        int activeClients;              // 活跃客户端数
        qint64 oldestFrameAge;          // 最老帧的年龄
        int keyFrameHitRate;            // I帧命中率
        
        CacheStats() : totalKeyFramesCached(0), totalMemoryUsage(0), 
                      activeClients(0), oldestFrameAge(0), keyFrameHitRate(0) {}
    };
    
    CacheStats getStats() const;
    
    // 获取相机优先级
    static CameraPriority getCameraPriority(int cameraId) {
        return static_cast<CameraPriority>(qBound(0, cameraId, 3));
    }
    
    // 设置缓存配置
    void setMaxMemoryUsage(size_t maxBytes) { m_maxMemoryUsage = maxBytes; }
    void setKeyFrameMaxAge(qint64 maxAge) { m_keyFrameMaxAge = maxAge; }
    void setCleanupInterval(int intervalMs);
    
private slots:
    void performCacheCleanup();
    
private:
    // ========== 私有方法 ==========
    
    // H.264 NAL单元分析
    bool analyzeNALUnit(const QByteArray& data, uint8_t& nalType, size_t& nalStart);
    
    // 内存管理
    void enforceMemoryLimit();
    size_t calculateTotalMemoryUsage() const;
    
    // 缓存清理
    void cleanupExpiredFrames();
    void removeOldestFrames(int cameraId, int keepCount = 1);
    
    // 客户端状态管理
    ClientH264State* findClientState(uint32_t clientId);
    void cleanupInactiveClients();
    
    // 统计更新
    void updateStats();
    
private:
    // ========== 数据成员 ==========
    
    // 核心缓存数据
    QMap<int, KeyFrameCache> m_latestKeyFrames;     // 每相机最新I帧 [cameraId -> KeyFrame]
    QMap<uint32_t, ClientH264State> m_clientStates; // 客户端状态 [clientId -> State]
    
    // 线程安全
    mutable QMutex m_cacheMutex;        // 缓存数据保护
    mutable QMutex m_clientMutex;       // 客户端状态保护
    
    // 配置参数
    size_t m_maxMemoryUsage;            // 最大内存使用量 (默认10MB)
    qint64 m_keyFrameMaxAge;            // I帧最大生存时间 (默认5秒)
    int m_maxClientsPerCamera;          // 每相机最大客户端数 (默认10)
    
    // 清理定时器
    QTimer* m_cleanupTimer;
    int m_cleanupInterval;              // 清理间隔 (默认2秒)
    
    // 统计信息
    mutable QMutex m_statsMutex;
    CacheStats m_stats;
    int m_keyFrameHitCount;
    int m_keyFrameMissCount;
    
    // 初始化状态
    bool m_initialized;
    
signals:
    // 缓存事件信号
    void keyFrameCached(int cameraId, uint32_t sequence, size_t dataSize);
    void keyFrameRequested(int cameraId, uint32_t clientId);
    void memoryLimitExceeded(size_t currentUsage, size_t limit);
    void clientKeyFrameNeeded(uint32_t clientId, int cameraId);
    void cacheStatsUpdated(const CacheStats& stats);
};

#endif // H264_CACHE_MANAGER_H
