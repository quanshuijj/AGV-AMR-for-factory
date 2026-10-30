// networkmanager.h - 网络管理器
#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <memory>

#include <QElapsedTimer>
#include <QHostAddress>
#include <QLoggingCategory>
#include <QMap>
#include <QMutex>
#include <QObject>
#include <QQueue>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThreadPool>
#include <QTimer>
#include <QUdpSocket>
#include<QSize>

#include "h264_cache_manager.h"
#include "networkprotocol.h"

Q_DECLARE_LOGGING_CATEGORY(networkLog)

struct H264FrameData {
    QByteArray data;
    int cameraId;
    bool isKeyFrame;
    uint32_t frameSequence;
    qint64 timestamp;
    CompressionLevel quality;
    float compressionRatio;

    H264FrameData() : cameraId(0), isKeyFrame(false), frameSequence(0),
        timestamp(0), quality(MEDIUM_QUALITY), compressionRatio(0.0f) {}
};

struct ClientStreamState {
    bool pointCloudEnabled = false;
    bool imageEnabled = false;
    bool statusEnabled = false;
    bool isPaused = false;

    CompressionLevel compressionLevel = MEDIUM_QUALITY;
    VideoCodec videoCodec = CODEC_H264;
    int frameRate = 30;
    QSize maxImageSize = QSize(1920, 1080);
    float maxPointCloudRange = 50.0f;
    bool needsKeyFrame[4] = {true, true, true, true};
    uint32_t lastFrameSequence[4] = {0, 0, 0, 0};
    qint64 lastKeyFrameTime[4] = {0, 0, 0, 0};
    int consecutivePacketLoss[4] = {0, 0, 0, 0};
    qint64 bandwidthLimit = 0;
    qint64 framesSent = 0;
    qint64 bytesSent = 0;
    float packetLossRate = 0.0f;
    QElapsedTimer lastFrameTime;

    ClientStreamState() {
        lastFrameTime.start();
    }

    bool shouldSendFrame() const {
        if (isPaused || frameRate <= 0) return false;
        int interval = 1000 / frameRate;
        return lastFrameTime.elapsed() >= interval;
    }

    CompressionLevel getAdaptiveQuality() const {
        if (packetLossRate > 0.1f) return LOW_QUALITY;
        if (packetLossRate > 0.05f) return MEDIUM_QUALITY;
        return compressionLevel;
    }

    bool isStreamActive(StreamType type) const {
        if (isPaused) return false;
        switch (type) {
        case STREAM_POINTCLOUD: return pointCloudEnabled;
        case STREAM_IMAGE: return imageEnabled;
        case STREAM_STATUS: return statusEnabled;
        default: return false;
        }
    }
};

struct ClientConnection {
    quint32 id = 0;
    QTcpSocket* tcpSocket = nullptr;
    QHostAddress address;
    quint16 port = 0;
    ConnectionState state = ConnectionState::Disconnected;
    QElapsedTimer lastActivity;
    ClientStreamState streamState;
    qint64 bytesReceived = 0;
    qint64 bytesSent = 0;
    QVector<bool> cameraSubscriptions = {true, true, true, true};

    ClientConnection() {
        lastActivity.start();
    }
};


class NetworkManager : public QObject
{
    Q_OBJECT

public:
    explicit NetworkManager(QObject *parent = nullptr);
    ~NetworkManager();

    // ========== 初始化和清理 ==========
    bool initialize();
    void cleanup();

    // ========== 服务器管理 ==========
    bool startTcpServer(quint16 port = 8888, const QHostAddress& address = QHostAddress::Any);
    bool startUdpServer(quint16 port = 8889, const QHostAddress& address = QHostAddress::Any);
    void stopServer();
    bool isServerRunning() const;

    // ========== 客户端连接 ==========
    bool connectToHost(const QString& hostName, quint16 port,
                       NetworkProtocol protocol = NetworkProtocol::TCP);
    void disconnectFromHost();
    bool isConnected() const;

    // ========== 数据发送接口 ==========
    void sendCompressedPointCloud(const QByteArray& compressedData,
                                  const CompressedDataInfo& info,
                                  quint32 clientId = 0);
    void sendCompressedJPEG(const QByteArray& compressedData,
                            int cameraId,
                            const CompressedDataInfo& info,
                            quint32 clientId = 0);
    void sendH264Frame(const QByteArray& h264Data,
                       int cameraId,
                       bool isKeyFrame,
                       uint32_t frameSequence,
                       const CompressedDataInfo& info);
    //======新增：GPS数据发送
    void sendGPSData(const NetDataGPSData,quint32 clientId=0);
    // ========== H.264管理 ==========
    void sendKeyFrameToClient(quint32 clientId, int cameraId);
    void sendKeyFrameToAll(int cameraId);
    void handleH264PacketLoss(quint32 clientId, int cameraId, uint32_t lostFrameId);
    void setClientCameraSubscription(quint32 clientId, int cameraId, bool enabled);

    // ========== 客户端管理 ==========
    QList<ClientConnection> getConnectedClients() const;
    ClientConnection getClientInfo(quint32 clientId) const;
    int getClientCount() const;
    QList<quint32> getConnectedClientIds() const;

    // ========== 流控制 ==========
    void enableClientStream(quint32 clientId, StreamType streamType, bool enabled);
    void setClientStreamParameters(quint32 clientId, const ClientStreamState& params);
    ClientStreamState getClientStreamState(quint32 clientId) const;

    // ========== 统计和监控 ==========
    NetworkStatistics getStatistics() const;
    void enableHeartbeat(int intervalMs = 5000);
    void disableHeartbeat();
    void enableStatusBroadcast(int intervalMs = 1000);
    void disableStatusBroadcast();

    // ========== 配置设置 ==========
    void setMaxConnections(int maxConnections);
    void setConnectionTimeout(int timeoutMs);
    void setH264FrameBufferSize(int bufferSize = 3);
    void setH264FlushInterval(int intervalMs = 33);

    // ========== 通用数据发送 ==========
    bool sendData(const QByteArray& data, quint32 clientId = 0);
    bool sendDataToAll(const QByteArray& data);
    bool sendPacket(quint8 type, const QByteArray& data, quint32 clientId = 0);

    // ========== 状态查询 ==========
    bool hasActiveClients() const;
    QStringList getConnectedClientInfo() const;
    H264CacheManager::CacheStats getH264CacheStats() const;

public slots:
    void kickClient(quint32 clientId, const QString& reason = "管理员踢出");
    void kickAllClients(const QString& reason = "服务器维护");

private slots:
    // ========== 网络事件处理 ==========
    void onNewTcpConnection();
    void onTcpSocketReadyRead();
    void onTcpSocketDisconnected();
    void onTcpSocketError(QAbstractSocket::SocketError error);
    void onUdpReadyRead();
    void onUdpError(QAbstractSocket::SocketError error);
    void onClientConnected();
    void onClientDisconnected();
    void onClientError(QAbstractSocket::SocketError error);

    // ========== 定时器事件 ==========
    void onHeartbeatTimer();
    void onStatusTimer();
    void onStatisticsTimer();
    void onCleanupTimer();
    void onH264FlushTimer();

    // ========== H.264事件 ==========
    void onKeyFrameCached(int cameraId, uint32_t sequence, size_t dataSize);
    void onClientKeyFrameNeeded(uint32_t clientId, int cameraId);
    void onCacheStatsUpdated(const H264CacheManager::CacheStats& stats);

private:
    QTcpServer* m_tcpServer;
    QUdpSocket* m_udpSocket;
    QTcpSocket* m_clientSocket;
    std::unique_ptr<H264CacheManager> m_h264CacheManager;
    QMap<int, QQueue<H264FrameData>> m_h264FrameBuffers;
    QTimer* m_h264FlushTimer;
    int m_h264BufferSize;
    QThreadPool* m_networkThreadPool;
    QMap<quint32, QSharedPointer<ClientConnection>> m_clients;
    QMap<quint32, QByteArray> m_clientBuffers;
    mutable QMutex m_clientsMutex;
    quint32 m_nextClientId;
    ConnectionState m_serverState;
    ConnectionState m_clientState;
    NetworkProtocol m_protocol;
    QHostAddress m_serverAddress;
    quint16 m_serverPort;
    QTimer* m_heartbeatTimer;
    QTimer* m_statusTimer;
    QTimer* m_statisticsTimer;
    QTimer* m_cleanupTimer;
    mutable QMutex m_statsMutex;
    NetworkStatistics m_statistics;
    QElapsedTimer m_uptimeTimer;
    QElapsedTimer m_bandwidthTimer;
    qint64 m_lastBytesTransferred;
    quint32 m_packetSequenceNumber;
    int m_maxConnections;
    int m_connectionTimeoutMs;
    bool m_heartbeatEnabled;
    bool m_statusBroadcastEnabled;

    quint32 generateClientId();
    ClientConnection* findClient(quint32 clientId);
    const ClientConnection* findClient(quint32 clientId) const;
    ClientConnection* findClient(QTcpSocket* socket);
    void addClient(QTcpSocket* socket);
    void removeClient(quint32 clientId);
    void updateClientActivity(quint32 clientId);
    void processH264Frame(const H264FrameData& frameData);
    void distributeH264Frame(const H264FrameData& frameData);
    void flushH264Frames(int cameraId = -1);
    bool shouldSendToClient(quint32 clientId, int cameraId) const;
    void updateClientH264State(quint32 clientId, int cameraId, uint32_t frameSequence, bool isKeyFrame);
    void processIncomingData(const QByteArray& data, quint32 clientId);
    void processPacket(const NetworkPacketHeader& header, const QByteArray& payload, quint32 clientId);
    bool validatePacket(const NetworkPacketHeader& header, const QByteArray& data);
    bool sendRawData(QTcpSocket* socket, const QByteArray& data);
    bool sendUdpData(const QByteArray& data, const QHostAddress& address, quint16 port);
    QByteArray createPacket(quint8 type, const QByteArray& data);
    void updateSendStatistics(qint64 bytesSent, quint32 clientId);
    void updateReceiveStatistics(qint64 bytesReceived, quint32 clientId);
    void updateStatistics();
    void checkClientTimeouts();
    float calculateBandwidth();
    void publishToSubscribedClients(const QByteArray& data, StreamType streamType);
    QList<quint32> getSubscribedClients(StreamType streamType);
    void processStreamControlCommand(quint32 clientId, const NetDataStreamControl* command);
    void logInfo(const QString& message);
    void logWarning(const QString& message);
    void logError(const QString& message);
    void sendHeartbeatToAll();
    void sendStatusToAll();

signals:
    void serverStarted(quint16 port, const QString& address);
    void serverStopped();
    void serverError(const QString& error);
    void clientConnected(quint32 clientId, const QString& address, quint16 port);
    void clientDisconnected(quint32 clientId, const QString& reason);
    void clientError(quint32 clientId, const QString& error);
    void connectionTimeout(quint32 clientId);
    void dataReceived(const QByteArray& data, quint32 clientId);
    void dataSent(qint64 bytesSent, quint32 clientId);
    void packetReceived(quint8 type, const QByteArray& data, quint32 clientId);
    void h264FrameSent(int cameraId, uint32_t frameSequence, quint32 clientId, bool isKeyFrame);
    void h264PacketLossDetected(quint32 clientId, int cameraId, uint32_t lostFrameId);
    void h264KeyFrameRequested(quint32 clientId, int cameraId);
    void h264StreamRecovered(quint32 clientId, int cameraId);
    void streamControlReceived(quint32 clientId, StreamType streamType, StreamAction action);
    void statisticsUpdated(const NetworkStatistics& stats);
    void heartbeatReceived(quint32 clientId);
    void requestCurrentPointCloud();
    void requestCurrentImage(int cameraId = -1);
    void udpDataReceived(const QByteArray& data, const QHostAddress& sender, quint16 senderPort);
};

#endif // NETWORKMANAGER_H
