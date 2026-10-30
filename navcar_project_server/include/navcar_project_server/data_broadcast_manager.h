// databroadcastmanager.h - 数据广播管理器
#ifndef DATA_BROADCAST_MANAGER_H
#define DATA_BROADCAST_MANAGER_H

#include <memory>

#include <QDateTime>
#include <QElapsedTimer>
#include <QLoggingCategory>
#include <QMutex>
#include <QObject>
#include <QThreadPool>
#include <QTimer>

#ifdef ROS_BUILD
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#endif

#include "compressionmanager.h"
#include "networkmanager.h"
#include "networkprotocol.h"

Q_DECLARE_LOGGING_CATEGORY(broadcastLog)

struct BroadcastStatistics {
    qint64 pointCloudFramesReceived = 0;
    qint64 imageFramesReceived[4] = {0, 0, 0, 0};
    qint64 totalDataReceived = 0;
    qint64 pointCloudFramesProcessed = 0;
    qint64 imageFramesProcessed[4] = {0, 0, 0, 0};
    float averageProcessingTime = 0.0f;
    qint64 totalFramesSent = 0;
    qint64 totalBytesSent = 0;
    int activeClients = 0;
    float pointCloudProcessingRate = 0.0f;
    float imageProcessingRate[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float networkThroughput = 0.0f;
    qint64 compressionErrors = 0;
    qint64 networkErrors = 0;
    QDateTime startTime;
    qint64 uptimeSeconds = 0;
};

struct BroadcastConfig {
    bool enablePointCloudBroadcast = true;
    CompressionLevel pointCloudCompressionLevel = MEDIUM_QUALITY;
    float pointCloudMaxRange = 50.0f;
    bool enableImageBroadcast[4] = {true, true, true, true};
    VideoCodec imageCodec = CODEC_H264;
    CompressionLevel imageCompressionLevel = MEDIUM_QUALITY;
    QSize imageMaxSize = QSize(1920, 1080);
    int imageFrameRate = 30;
    int h264Bitrate = 2000000;
    int h264GOP = 30;
    bool enableHeartbeat = true;
    int heartbeatInterval = 5000;
    bool enableStatusBroadcast = true;
    int statusInterval = 1000;
    int maxClients = 50;
    int connectionTimeout = 30000;
    bool enableAdaptiveQuality = true;

    BroadcastConfig() = default;
};

class DataBroadcastManager : public QObject
{
    Q_OBJECT

public:
    explicit DataBroadcastManager(QObject *parent = nullptr);
    ~DataBroadcastManager();

    // ========== 初始化和配置 ==========
    bool initialize(const BroadcastConfig& config = BroadcastConfig());
    void cleanup();
    void setConfig(const BroadcastConfig& config) { m_config = config; }
    void updateConfig(const BroadcastConfig& config);
    BroadcastConfig getConfig() const;

    // ========== 网络服务器管理 ==========
    bool startNetworkServer(quint16 port = 8888);
    void stopNetworkServer();
    bool isNetworkServerRunning() const;

    // ========== 数据广播接口 ==========
#ifdef ROS_BUILD
    void broadcastPointCloud(QSharedPointer<sensor_msgs::PointCloud2> cloudMsg);
    void broadcastCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg, int cameraId);
    void broadcastGPSData(NetDataGPSData myGPSMsg);
#endif
    void broadcastPointCloud(const CrossPlatformPointCloud& pointCloud);
    void broadcastImage(const QImage& image, int cameraId);
    void broadcastCompressedData(const QByteArray& data, NetworkDataType dataType, int cameraId = -1);

    // ========== 广播控制 ==========
    void setClientStreamConfig(quint32 clientId, StreamType streamType, bool enabled);
    void setClientCameraSubscription(quint32 clientId, int cameraId, bool enabled);
    void setClientCompressionLevel(quint32 clientId, CompressionLevel level);
    NetworkManager* getNetworkManager() const { return m_networkManager.get(); }

    // ========== 状态和统计 ==========
    BroadcastStatistics getStatistics() const;
    CompressionManager::CompressionStats getCompressionStatistics() const;
    NetworkStatistics getNetworkStatistics() const;
    void resetStatistics();

    // ========== 控制接口 ==========
    void pauseBroadcast(bool pause = true);
    void resumeBroadcast() { pauseBroadcast(false); }
    bool isBroadcastPaused() const { return m_broadcastPaused; }
    void enablePointCloudBroadcast(bool enable);
    void enableImageBroadcast(int cameraId, bool enable);
    void enableImageBroadcastAll(bool enable);

    // ========== 诊断和调试 ==========
    QString getDiagnosticsInfo() const;
    void printStatistics() const;

    //=====转换辅助函数======
    QImage convertTo1080p(const QImage& sourceImage);

public slots:
    void onConfigurationChanged(const BroadcastConfig& newConfig);

private slots:
    void onPointCloudCompressionCompleted(const CompressionResult& result);
    void onImageCompressionCompleted(const CompressionResult& result);
    void onH264FrameGenerated(int cameraId, uint32_t frameSequence, const QByteArray& data, bool isKeyFrame);
    void onStatisticsUpdateTimer();
    void onClientConnected(quint32 clientId, const QString& address, quint16 port);
    void onClientDisconnected(quint32 clientId, const QString& reason);
    void onNetworkError(const QString& error);

private:
    bool setupCompressionManager();
    bool setupNetworkManager();
    void connectSignals();
    void processPointCloudData(const CrossPlatformPointCloud& pointCloud);
    void processImageData(const QImage& image, int cameraId);

#ifdef ROS_BUILD
    CrossPlatformPointCloud convertROSPointCloud(const sensor_msgs::PointCloud2& rosCloud);
    QImage convertROSImage(const sensor_msgs::Image& rosImage);
#endif

    void updateStatistics();
    void updateProcessingRates();
    void applyCompressionConfig();
    void applyNetworkConfig();
    bool validateImageData(const QImage& image, int cameraId) const;
    bool validatePointCloudData(const CrossPlatformPointCloud& pointCloud) const;
    void handleCompressionError(const QString& error);
    void handleNetworkError(const QString& error);
    void logInfo(const QString& message) const;
    void logWarning(const QString& message) const;
    void logError(const QString& message) const;

    std::unique_ptr<CompressionManager> m_compressionManager;
    std::unique_ptr<NetworkManager> m_networkManager;
    BroadcastConfig m_config;
    bool m_initialized;
    bool m_broadcastPaused;
    mutable QMutex m_statsMutex;
    BroadcastStatistics m_statistics;
    QElapsedTimer m_uptimeTimer;
    QTimer* m_statisticsTimer;
    QElapsedTimer m_lastFrameTime[4];
    QElapsedTimer m_lastPointCloudTime;

signals:
    void networkServerStarted(quint16 port);
    void networkServerStopped();
    void networkServerError(const QString& error);
    void clientConnected(quint32 clientId, const QString& info);
    void clientDisconnected(quint32 clientId, const QString& reason);
    void pointCloudBroadcasted(qint64 originalSize, qint64 compressedSize, float compressionRatio);
    void imageBroadcasted(int cameraId, qint64 originalSize, qint64 compressedSize, float compressionRatio);
    void h264FrameBroadcasted(int cameraId, uint32_t frameSequence, bool isKeyFrame);
    void statisticsUpdated(const BroadcastStatistics& stats);
    void compressionStatisticsUpdated(const CompressionManager::CompressionStats& stats);
    void networkStatisticsUpdated(const NetworkStatistics& stats);
    void compressionError(const QString& error);
    void networkError(const QString& error);
    void broadcastError(const QString& error);
    void broadcastPaused();
    void broadcastResumed();
    void configurationUpdated(const BroadcastConfig& config);
};

#endif // DATA_BROADCAST_MANAGER_H
