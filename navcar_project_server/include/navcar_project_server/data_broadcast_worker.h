// data_broadcast_worker.h - 数据广播工作线程类
#ifndef DATA_BROADCAST_WORKER_H
#define DATA_BROADCAST_WORKER_H

#include <memory>
#include <QObject>
#include <QMutex>

#ifdef ROS_BUILD
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#endif

#include "data_broadcast_manager.h"
#include "networkprotocol.h"

/**
 * @brief 数据广播工作线程类
 * 
 * 这个类作为DataBroadcastManager的线程代理，在独立线程中管理数据广播功能。
 * 通过信号槽机制与主线程通信，避免阻塞车辆控制逻辑。
 */
class DataBroadcastWorker : public QObject
{
    Q_OBJECT

public:
    explicit DataBroadcastWorker(QObject *parent = nullptr);
    ~DataBroadcastWorker();

public slots:
    // ========== 核心生命周期管理 ==========
    /**
     * @brief 在工作线程中初始化数据广播管理器
     * @param config 广播配置
     */
    void initialize(const BroadcastConfig& config = BroadcastConfig());
    
    /**
     * @brief 清理数据广播管理器
     */
    void cleanup();

    // ========== 服务器控制接口 ==========
    /**
     * @brief 启动网络服务器
     * @param port 监听端口
     */
    void startServer(quint16 port);
    
    /**
     * @brief 停止网络服务器
     */
    void stopServer();

    // ========== 数据广播接口 ==========
#ifdef ROS_BUILD
    /**
     * @brief 广播ROS点云数据
     * @param cloudMsg ROS点云消息
     */
    void broadcastPointCloud(QSharedPointer<sensor_msgs::PointCloud2> cloudMsg);
    
    /**
     * @brief 广播前向相机图像
     * @param imageMsg ROS图像消息
     */
    void broadcastFrontCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg);
    
    /**
     * @brief 广播左侧相机图像
     * @param imageMsg ROS图像消息
     */
    void broadcastLeftCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg);
    
    /**
     * @brief 广播右侧相机图像
     * @param imageMsg ROS图像消息
     */
    void broadcastRightCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg);
    
    /**
     * @brief 广播后向相机图像
     * @param imageMsg ROS图像消息
     */
    void broadcastRearCameraImage(QSharedPointer<sensor_msgs::Image> imageMsg);

    /**
     * @brief 转发ROS消息信号
     * @param myGPSMsg GPS消息转发层
     */
    void broadcastGPSData(NetDataGPSData myGPSMsg);
#endif

    /**
     * @brief 广播跨平台点云数据
     * @param pointCloud 点云数据
     */
    void broadcastPointCloud(const CrossPlatformPointCloud& pointCloud);
    
    /**
     * @brief 广播图像数据
     * @param image 图像数据
     * @param cameraId 相机ID
     */
    void broadcastImage(const QImage& image, int cameraId);

    // ========== 客户端管理接口 ==========
    /**
     * @brief 踢出指定客户端
     * @param clientId 客户端ID
     * @param reason 踢出原因
     */
    void kickClient(quint32 clientId, const QString& reason);
    
    /**
     * @brief 踢出所有客户端
     * @param reason 踢出原因
     */
    void kickAllClients(const QString& reason);
    
    /**
     * @brief 设置客户端流配置
     * @param clientId 客户端ID
     * @param streamType 流类型
     * @param enabled 是否启用
     */
    void setClientStreamConfig(quint32 clientId, StreamType streamType, bool enabled);
    
    /**
     * @brief 设置客户端相机订阅
     * @param clientId 客户端ID
     * @param cameraId 相机ID
     * @param enabled 是否启用
     */
    void setClientCameraSubscription(quint32 clientId, int cameraId, bool enabled);
    
    /**
     * @brief 设置客户端压缩级别
     * @param clientId 客户端ID
     * @param level 压缩级别
     */
    void setClientCompressionLevel(quint32 clientId, CompressionLevel level);

    // ========== 配置管理接口 ==========
    /**
     * @brief 更新广播配置
     * @param config 新配置
     */
    void updateConfig(const BroadcastConfig& config);
    
    /**
     * @brief 暂停/恢复数据广播
     * @param pause true=暂停, false=恢复
     */
    void pauseBroadcast(bool pause);

    // ========== 查询接口（同步调用）==========
    /**
     * @brief 请求服务器运行状态
     * 结果通过 serverStatusResult 信号返回
     */
    void requestServerStatus();
    
    /**
     * @brief 请求客户端数量
     * 结果通过 clientCountResult 信号返回
     */
    void requestClientCount();
    
    /**
     * @brief 请求客户端信息列表
     * 结果通过 clientInfoResult 信号返回
     */
    void requestClientInfo();
    
    /**
     * @brief 请求客户端ID列表
     * 结果通过 clientIdsResult 信号返回
     */
    void requestClientIds();
    
    /**
     * @brief 请求广播配置
     * 结果通过 configResult 信号返回
     */
    void requestConfig();
    
    /**
     * @brief 请求广播统计信息
     * 结果通过 statisticsResult 信号返回
     */
    void requestStatistics();
    
    /**
     * @brief 请求网络统计信息
     * 结果通过 networkStatisticsResult 信号返回
     */
    void requestNetworkStatistics();


signals:
    // ========== 生命周期信号 ==========
    /**
     * @brief 初始化完成信号
     * @param success 是否成功
     */
    void initializeResult(bool success);

    // ========== 服务器状态信号 ==========
    /**
     * @brief 网络服务器启动信号
     * @param port 监听端口
     */
    void serverStarted(quint16 port);
    
    /**
     * @brief 网络服务器停止信号
     */
    void serverStopped();
    
    /**
     * @brief 网络服务器错误信号
     * @param error 错误信息
     */
    void serverError(const QString& error);

    // ========== 客户端事件信号 ==========
    /**
     * @brief 客户端连接信号
     * @param clientId 客户端ID
     * @param info 客户端信息
     */
    void clientConnected(quint32 clientId, const QString& info);
    
    /**
     * @brief 客户端断开信号
     * @param clientId 客户端ID
     * @param reason 断开原因
     */
    void clientDisconnected(quint32 clientId, const QString& reason);

    // ========== 广播事件信号 ==========
    /**
     * @brief 点云广播完成信号
     * @param originalSize 原始大小
     * @param compressedSize 压缩后大小
     * @param compressionRatio 压缩比
     */
    void pointCloudBroadcasted(qint64 originalSize, qint64 compressedSize, float compressionRatio);
    
    /**
     * @brief 图像广播完成信号
     * @param cameraId 相机ID
     * @param originalSize 原始大小
     * @param compressedSize 压缩后大小
     * @param compressionRatio 压缩比
     */
    void imageBroadcasted(int cameraId, qint64 originalSize, qint64 compressedSize, float compressionRatio);

    // ========== 广播控制信号 ==========
    /**
     * @brief 广播暂停信号
     */
    void broadcastPaused();
    
    /**
     * @brief 广播恢复信号
     */
    void broadcastResumed();

    // ========== 查询结果信号 ==========
    /**
     * @brief 服务器状态查询结果
     * @param isRunning 是否运行中
     */
    void serverStatusResult(bool isRunning);
    
    /**
     * @brief 客户端数量查询结果
     * @param count 客户端数量
     */
    void clientCountResult(int count);
    
    /**
     * @brief 客户端信息查询结果
     * @param clientInfo 客户端信息列表
     */
    void clientInfoResult(const QStringList& clientInfo);
    
    /**
     * @brief 客户端ID查询结果
     * @param clientIds 客户端ID列表
     */
    void clientIdsResult(const QList<quint32>& clientIds);
    
    /**
     * @brief 配置查询结果
     * @param config 当前配置
     */
    void configResult(const BroadcastConfig& config);
    
    /**
     * @brief 统计信息查询结果
     * @param stats 广播统计信息
     */
    void statisticsResult(const BroadcastStatistics& stats);
    
    /**
     * @brief 网络统计信息查询结果
     * @param stats 网络统计信息
     */
    void networkStatisticsResult(const NetworkStatistics& stats);

    // ========== 错误信号 ==========
    /**
     * @brief 压缩错误信号
     * @param error 错误信息
     */
    void compressionError(const QString& error);
    
    /**
     * @brief 网络错误信号
     * @param error 错误信息
     */
    void networkError(const QString& error);
    
    /**
     * @brief 广播错误信号
     * @param error 错误信息
     */
    void broadcastError(const QString& error);


private slots:
    // ========== 内部事件处理 ==========
    void onManagerServerStarted(quint16 port);
    void onManagerServerStopped();
    void onManagerServerError(const QString& error);
    void onManagerClientConnected(quint32 clientId, const QString& info);
    void onManagerClientDisconnected(quint32 clientId, const QString& reason);
    void onManagerPointCloudBroadcasted(qint64 originalSize, qint64 compressedSize, float compressionRatio);
    void onManagerImageBroadcasted(int cameraId, qint64 originalSize, qint64 compressedSize, float compressionRatio);
    void onManagerBroadcastPaused();
    void onManagerBroadcastResumed();
    void onManagerCompressionError(const QString& error);
    void onManagerNetworkError(const QString& error);
    void onManagerBroadcastError(const QString& error);

private:
    /**
     * @brief 连接管理器信号到Worker信号
     */
    void connectManagerSignals();

private:
    std::unique_ptr<DataBroadcastManager> m_manager;  ///< 数据广播管理器
    bool m_initialized;                               ///< 初始化状态
    mutable QMutex m_mutex;                          ///< 线程安全锁
};

#endif // DATA_BROADCAST_WORKER_H
