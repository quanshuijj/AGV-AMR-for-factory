// data_broadcast_worker.cpp - 数据广播工作线程类实现
#include "navcar_project_server/data_broadcast_worker.h"
#include <QDebug>
#include <QMutexLocker>

DataBroadcastWorker::DataBroadcastWorker(QObject *parent)
    : QObject(parent)
    , m_manager(nullptr)
    , m_initialized(false)
{
    qDebug() << "DataBroadcastWorker构造函数 - 线程ID:" << QThread::currentThreadId();
}

DataBroadcastWorker::~DataBroadcastWorker()
{
    qDebug() << "DataBroadcastWorker析构函数 - 线程ID:" << QThread::currentThreadId();
    cleanup();
}

// ========== 核心生命周期管理 ==========

void DataBroadcastWorker::initialize(const BroadcastConfig& config)
{
    QMutexLocker locker(&m_mutex);
    
    if (m_initialized) {
        qWarning() << "DataBroadcastWorker已经初始化";
        emit initializeResult(true);
        return;
    }

    try {
        qDebug() << "DataBroadcastWorker开始初始化 - 线程ID:" << QThread::currentThreadId();

        // 在工作线程中创建DataBroadcastManager（无父对象）
        m_manager = std::make_unique<DataBroadcastManager>(nullptr);

        // 初始化管理器
        bool success = m_manager->initialize(config);
        if (!success) {
            qCritical() << "DataBroadcastManager初始化失败";
            m_manager.reset();
            emit initializeResult(false);
            return;
        }

        // 连接管理器信号
        connectManagerSignals();

        m_initialized = true;
        qDebug() << "DataBroadcastWorker初始化成功";
        emit initializeResult(true);

    } catch (const std::exception& e) {
        qCritical() << "DataBroadcastWorker初始化异常:" << e.what();
        m_manager.reset();
        emit initializeResult(false);
    }
}

void DataBroadcastWorker::cleanup()
{
    QMutexLocker locker(&m_mutex);
    
    if (!m_initialized) {
        return;
    }

    qDebug() << "DataBroadcastWorker开始清理 - 线程ID:" << QThread::currentThreadId();

    if (m_manager) {
        m_manager->cleanup();
        m_manager.reset();
    }

    m_initialized = false;
    qDebug() << "DataBroadcastWorker清理完成";
}

// ========== 服务器控制接口 ==========

void DataBroadcastWorker::startServer(quint16 port)
{
    if (!m_manager) {
        qWarning() << "DataBroadcastManager未初始化，无法启动服务器";
        emit serverError("管理器未初始化");
        return;
    }

    qDebug() << "DataBroadcastWorker启动服务器，端口:" << port;
    bool success = m_manager->startNetworkServer(port);
    if (!success) {
        emit serverError("服务器启动失败");
    }
    // 成功信号会通过connectManagerSignals中的连接自动转发
}

void DataBroadcastWorker::stopServer()
{
    if (!m_manager) {
        qWarning() << "DataBroadcastManager未初始化";
        return;
    }

    qDebug() << "DataBroadcastWorker停止服务器";
    m_manager->stopNetworkServer();
    // 停止信号会通过connectManagerSignals中的连接自动转发
}

// ========== 数据广播接口 ==========

#ifdef ROS_BUILD
void DataBroadcastWorker::broadcastPointCloud(QSharedPointer<sensor_msgs::PointCloud2> cloudMsg)
{
    if (!m_manager || !cloudMsg) {
        qDebug() << "Worker未初始化，丢弃点云消息";
        return;
    }

    m_manager->broadcastPointCloud(cloudMsg);
}

void DataBroadcastWorker::broadcastFrontCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg)
{
    if (!m_manager || !imageMsg) {
        qDebug() << "Worker未初始化，丢弃图像消息";
        return;
    }

    m_manager->broadcastCameraImage(imageMsg, 0); // 前向相机ID=0
}

void DataBroadcastWorker::broadcastLeftCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg)
{
    if (!m_manager || !imageMsg) {
        return;
    }

    m_manager->broadcastCameraImage(imageMsg, 1); // 左侧相机ID=1
}

void DataBroadcastWorker::broadcastRightCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg)
{
    if (!m_manager || !imageMsg) {
        return;
    }

    m_manager->broadcastCameraImage(imageMsg, 2); // 右侧相机ID=2
}

void DataBroadcastWorker::broadcastRearCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg)
{
    if (!m_manager || !imageMsg) {
        return;
    }

    m_manager->broadcastCameraImage(imageMsg, 3); // 后向相机ID=3
}

void DataBroadcastWorker::broadcastGPSData(NetDataGPSData myGPSMsg)
{
    if (!m_manager) {
        return;
    }
    m_manager->broadcastGPSData(myGPSMsg);
}

#endif

void DataBroadcastWorker::broadcastPointCloud(const CrossPlatformPointCloud& pointCloud)
{
    if (!m_manager) {
        return;
    }

    m_manager->broadcastPointCloud(pointCloud);
}

void DataBroadcastWorker::broadcastImage(const QImage& image, int cameraId)
{
    if (!m_manager) {
        return;
    }

    m_manager->broadcastImage(image, cameraId);
}


// ========== 客户端管理接口 ==========

void DataBroadcastWorker::kickClient(quint32 clientId, const QString& reason)
{
    if (!m_manager) {
        return;
    }

    qDebug() << "DataBroadcastWorker踢出客户端:" << clientId << "原因:" << reason;
    
    NetworkManager* networkMgr = m_manager->getNetworkManager();
    if (networkMgr) {
        networkMgr->kickClient(clientId, reason);
    }
}

void DataBroadcastWorker::kickAllClients(const QString& reason)
{
    if (!m_manager) {
        return;
    }

    qDebug() << "DataBroadcastWorker踢出所有客户端，原因:" << reason;
    
    NetworkManager* networkMgr = m_manager->getNetworkManager();
    if (networkMgr) {
        networkMgr->kickAllClients(reason);
    }
}

void DataBroadcastWorker::setClientStreamConfig(quint32 clientId, StreamType streamType, bool enabled)
{
    if (!m_manager) {
        return;
    }

    m_manager->setClientStreamConfig(clientId, streamType, enabled);
}

void DataBroadcastWorker::setClientCameraSubscription(quint32 clientId, int cameraId, bool enabled)
{
    if (!m_manager) {
        return;
    }

    m_manager->setClientCameraSubscription(clientId, cameraId, enabled);
}

void DataBroadcastWorker::setClientCompressionLevel(quint32 clientId, CompressionLevel level)
{
    if (!m_manager) {
        return;
    }

    m_manager->setClientCompressionLevel(clientId, level);
}

// ========== 配置管理接口 ==========

void DataBroadcastWorker::updateConfig(const BroadcastConfig& config)
{
    if (!m_manager) {
        return;
    }

    qDebug() << "DataBroadcastWorker更新配置";
    m_manager->updateConfig(config);
}

void DataBroadcastWorker::pauseBroadcast(bool pause)
{
    if (!m_manager) {
        return;
    }

    qDebug() << "DataBroadcastWorker" << (pause ? "暂停" : "恢复") << "广播";
    m_manager->pauseBroadcast(pause);
}

// ========== 查询接口（异步）==========

void DataBroadcastWorker::requestServerStatus()
{
    bool isRunning = false;
    if (m_manager) {
        isRunning = m_manager->isNetworkServerRunning();
    }
    emit serverStatusResult(isRunning);
}

void DataBroadcastWorker::requestClientCount()
{
    int count = 0;
    if (m_manager) {
        NetworkManager* networkMgr = m_manager->getNetworkManager();
        if (networkMgr) {
            count = networkMgr->getClientCount();
        }
    }
    emit clientCountResult(count);
}

void DataBroadcastWorker::requestClientInfo()
{
    QStringList clientInfo;
    if (m_manager) {
        NetworkManager* networkMgr = m_manager->getNetworkManager();
        if (networkMgr) {
            clientInfo = networkMgr->getConnectedClientInfo();
        }
    }
    emit clientInfoResult(clientInfo);
}

void DataBroadcastWorker::requestClientIds()
{
    QList<quint32> clientIds;
    if (m_manager) {
        NetworkManager* networkMgr = m_manager->getNetworkManager();
        if (networkMgr) {
            clientIds = networkMgr->getConnectedClientIds();
        }
    }
    emit clientIdsResult(clientIds);
}

void DataBroadcastWorker::requestConfig()
{
    BroadcastConfig config;
    if (m_manager) {
        config = m_manager->getConfig();
    }
    emit configResult(config);
}

void DataBroadcastWorker::requestStatistics()
{
    BroadcastStatistics stats;
    if (m_manager) {
        stats = m_manager->getStatistics();
    }
    emit statisticsResult(stats);
}

void DataBroadcastWorker::requestNetworkStatistics()
{
    NetworkStatistics stats;
    if (m_manager) {
        NetworkManager* networkMgr = m_manager->getNetworkManager();
        if (networkMgr) {
            stats = networkMgr->getStatistics();
        }
    }
    emit networkStatisticsResult(stats);
}

// ========== 内部事件处理槽函数 ==========

void DataBroadcastWorker::onManagerServerStarted(quint16 port)
{
    qDebug() << "DataBroadcastWorker收到服务器启动信号，端口:" << port;
    emit serverStarted(port);
}

void DataBroadcastWorker::onManagerServerStopped()
{
    qDebug() << "DataBroadcastWorker收到服务器停止信号";
    emit serverStopped();
}

void DataBroadcastWorker::onManagerServerError(const QString& error)
{
    qWarning() << "DataBroadcastWorker收到服务器错误信号:" << error;
    emit serverError(error);
}

void DataBroadcastWorker::onManagerClientConnected(quint32 clientId, const QString& info)
{
    qDebug() << "DataBroadcastWorker收到客户端连接信号:" << clientId << info;
    emit clientConnected(clientId, info);
}

void DataBroadcastWorker::onManagerClientDisconnected(quint32 clientId, const QString& reason)
{
    qDebug() << "DataBroadcastWorker收到客户端断开信号:" << clientId << reason;
    emit clientDisconnected(clientId, reason);
}

void DataBroadcastWorker::onManagerPointCloudBroadcasted(qint64 originalSize, qint64 compressedSize, float compressionRatio)
{
    emit pointCloudBroadcasted(originalSize, compressedSize, compressionRatio);
}

void DataBroadcastWorker::onManagerImageBroadcasted(int cameraId, qint64 originalSize, qint64 compressedSize, float compressionRatio)
{
    emit imageBroadcasted(cameraId, originalSize, compressedSize, compressionRatio);
}

void DataBroadcastWorker::onManagerBroadcastPaused()
{
    qDebug() << "DataBroadcastWorker收到广播暂停信号";
    emit broadcastPaused();
}

void DataBroadcastWorker::onManagerBroadcastResumed()
{
    qDebug() << "DataBroadcastWorker收到广播恢复信号";
    emit broadcastResumed();
}

void DataBroadcastWorker::onManagerCompressionError(const QString& error)
{
    qWarning() << "DataBroadcastWorker收到压缩错误信号:" << error;
    emit compressionError(error);
}

void DataBroadcastWorker::onManagerNetworkError(const QString& error)
{
    qWarning() << "DataBroadcastWorker收到网络错误信号:" << error;
    emit networkError(error);
}

void DataBroadcastWorker::onManagerBroadcastError(const QString& error)
{
    qWarning() << "DataBroadcastWorker收到广播错误信号:" << error;
    emit broadcastError(error);
}

// ========== 私有方法实现 ==========

void DataBroadcastWorker::connectManagerSignals()
{
    if (!m_manager) {
        return;
    }

    qDebug() << "DataBroadcastWorker连接管理器信号";

    // 连接服务器状态信号
    connect(m_manager.get(), &DataBroadcastManager::networkServerStarted,
            this, &DataBroadcastWorker::onManagerServerStarted);
    connect(m_manager.get(), &DataBroadcastManager::networkServerStopped,
            this, &DataBroadcastWorker::onManagerServerStopped);
    connect(m_manager.get(), &DataBroadcastManager::networkServerError,
            this, &DataBroadcastWorker::onManagerServerError);

    // 连接客户端事件信号
    connect(m_manager.get(), &DataBroadcastManager::clientConnected,
            this, &DataBroadcastWorker::onManagerClientConnected);
    connect(m_manager.get(), &DataBroadcastManager::clientDisconnected,
            this, &DataBroadcastWorker::onManagerClientDisconnected);

    // 连接广播事件信号
    connect(m_manager.get(), &DataBroadcastManager::pointCloudBroadcasted,
            this, &DataBroadcastWorker::onManagerPointCloudBroadcasted);
    connect(m_manager.get(), &DataBroadcastManager::imageBroadcasted,
            this, &DataBroadcastWorker::onManagerImageBroadcasted);

    // 连接广播控制信号
    connect(m_manager.get(), &DataBroadcastManager::broadcastPaused,
            this, &DataBroadcastWorker::onManagerBroadcastPaused);
    connect(m_manager.get(), &DataBroadcastManager::broadcastResumed,
            this, &DataBroadcastWorker::onManagerBroadcastResumed);

    // 连接错误信号
    connect(m_manager.get(), &DataBroadcastManager::compressionError,
            this, &DataBroadcastWorker::onManagerCompressionError);
    connect(m_manager.get(), &DataBroadcastManager::networkError,
            this, &DataBroadcastWorker::onManagerNetworkError);
    connect(m_manager.get(), &DataBroadcastManager::broadcastError,
            this, &DataBroadcastWorker::onManagerBroadcastError);

    qDebug() << "DataBroadcastWorker管理器信号连接完成";
}
