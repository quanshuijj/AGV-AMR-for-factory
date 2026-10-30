// compressionmanager.cpp - 重构版本实现
#include "navcar_project_server/compressionmanager.h"
#include "navcar_project_server/qt_version_compat.h"

Q_LOGGING_CATEGORY(compressionLog, "compression")

CompressionManager::CompressionManager(QObject *parent)
    : QObject(parent)
    , m_initialized(false)
    , m_dracoAvailable(false)
    , m_h264Available(false)
    , m_lowFrequencyMode(false)
    , m_dracoHelper(nullptr)
    , m_h264CacheManager(nullptr)
    , m_pointCloudThreadPool(nullptr)
    , m_imageThreadPool(nullptr)
    , m_frameSequenceGenerator(1000)
{
    // 初始化H264编码器上下文
    for (int i = 0; i < 4; i++) {
        m_h264Contexts[i] = H264EncoderContext();
        m_forceNextKeyFrame[i] = true; // 第一帧强制为关键帧
    }

    // 初始化统计信息
    m_stats = CompressionStats();

    // 创建线程池
    m_pointCloudThreadPool = new QThreadPool(this);
    m_pointCloudThreadPool->setMaxThreadCount(2); // 点云压缩最多2个线程
    m_pointCloudThreadPool->setObjectName("PointCloudCompressionPool");

    m_imageThreadPool = new QThreadPool(this);
    m_imageThreadPool->setMaxThreadCount(4); // 图像压缩最多4个线程（每个相机一个）
    m_imageThreadPool->setObjectName("ImageCompressionPool");

    // 启动计时器
    m_timer.start();
}

CompressionManager::~CompressionManager()
{
    cleanup();
}

// ========== 初始化和清理 ==========

bool CompressionManager::initialize()
{
    if (m_initialized) {
        qCWarning(compressionLog) << "压缩管理器已初始化";
        return true;
    }

    qCInfo(compressionLog) << "初始化压缩管理器...";

    // 初始化Draco
    if (!initializeDraco()) {
        qCWarning(compressionLog) << "Draco初始化失败，将使用简单压缩";
    }

    // 初始化H264
    if (!initializeH264()) {
        qCWarning(compressionLog) << "H264初始化失败，将使用JPEG压缩";
    }

    // 创建H264缓存管理器
    m_h264CacheManager = std::make_unique<H264CacheManager>(this);
    if (!m_h264CacheManager->initialize()) {
        qCWarning(compressionLog) << "H264缓存管理器初始化失败";
    }

    m_initialized = true;
    qCInfo(compressionLog) << "压缩管理器初始化完成";
    qCInfo(compressionLog) << "可用压缩方式: Draco=" << m_dracoAvailable
                          << ", H264=" << m_h264Available;

    return true;
}

void CompressionManager::cleanup()
{
    if (!m_initialized) {
        return;
    }

    qCInfo(compressionLog) << "清理压缩管理器...";

    // 等待所有任务完成
    if (m_pointCloudThreadPool) {
        m_pointCloudThreadPool->waitForDone(5000);
    }
    if (m_imageThreadPool) {
        m_imageThreadPool->waitForDone(5000);
    }

    // 清理Draco
    cleanupDraco();

    // 清理H264
    cleanupH264();

    // 清理H264缓存管理器
    if (m_h264CacheManager) {
        m_h264CacheManager->cleanup();
        m_h264CacheManager.reset();
    }

    // 清理任务队列
    QMutexLocker locker(&m_queueMutex);
    m_taskQueue.clear();

    m_initialized = false;
    qCInfo(compressionLog) << "压缩管理器清理完成";
}

// ========== 主要压缩接口 ==========

CompressionResult CompressionManager::compressPointCloud(const CrossPlatformPointCloud& cloud,
                                                         CompressionLevel level)
{
    CompressionResult result;
    result.originalSize = cloud.size() * sizeof(CrossPlatformPointCloud::Point);
    result.processingTime = m_timer.elapsed();

    if (cloud.isEmpty()) {
        result.errorMessage = "点云数据为空";
        return result;
    }

    if (m_dracoAvailable && m_dracoHelper) {
        // 使用Draco压缩
        DracoCompressionResult dracoResult = m_dracoHelper->compressPointCloud(cloud, level);
        if (dracoResult.success) {
            result.success = true;
            result.data = dracoResult.dracoData;
            result.compressedSize = dracoResult.compressedSize;
            result.compressionMethod = "draco";
            result.isDracoCompressed = true;
            result.quantizationBits = dracoResult.quantizationBits;
        } else {
            result.errorMessage = dracoResult.errorMessage;
        }
    } else {
        // 使用简单压缩
        result.data = compressPointCloudSimple(cloud);
        if (!result.data.isEmpty()) {
            result.success = true;
            result.compressedSize = result.data.size();
            result.compressionMethod = "simple";
        } else {
            result.errorMessage = "简单压缩失败";
        }
    }

    result.processingTime = m_timer.elapsed() - result.processingTime;
    updateStats(result);

    return result;
}

CompressionResult CompressionManager::compressImage(const QImage& image, int cameraId,
                                                    VideoCodec codec, CompressionLevel level)
{
    CompressionResult result;
    result.cameraId = cameraId;
    result.frameSequence = m_frameSequenceGenerator.fetch_add(1, std::memory_order_relaxed);
    result.processingTime = m_timer.elapsed();
    result.originalSize = image.sizeInBytes();

    if (image.isNull()) {
        result.errorMessage = "图像数据为空";
        return result;
    }

    if (cameraId < 0 || cameraId >= 4) {
        result.errorMessage = "相机ID无效";
        return result;
    }

    if (codec == CODEC_H264 && m_h264Available) {
        // 使用H264压缩
        bool forceKeyFrame = m_forceNextKeyFrame[cameraId];
        result = encodeH264Frame(cameraId, image, forceKeyFrame);

        if (result.success && forceKeyFrame) {
            m_forceNextKeyFrame[cameraId] = false;
        }
    } else {
        // 使用JPEG压缩
        int quality = getJPEGQuality(level);
        result.data = encodeImageToJPEG(image, quality);

        if (!result.data.isEmpty()) {
            result.success = true;
            result.compressedSize = result.data.size();
            result.compressionMethod = "jpeg";
        } else {
            result.errorMessage = "JPEG压缩失败";
        }
    }

    result.processingTime = m_timer.elapsed() - result.processingTime;
    updateStats(result);

    return result;
}

// ========== 异步压缩接口 ==========

void CompressionManager::compressPointCloudAsync(const CrossPlatformPointCloud& cloud,
                                                 CompressionLevel level,
                                                 std::function<void(const CompressionResult&)> callback)
{
    if (!callback) {
        qCWarning(compressionLog) << "异步点云压缩回调函数为空";
        return;
    }

    // 创建压缩任务
    auto task = [this, cloud, level, callback]() {
        CompressionResult result = compressPointCloud(cloud, level);

        // 在主线程中调用回调
        QMetaObject::invokeMethod(this, [callback, result]() {
            callback(result);
        }, Qt::QueuedConnection);
    };

    // 提交到线程池
    //m_pointCloudThreadPool->start(task);
    // 使用兼容性宏提交到线程池
    QT_THREADPOOL_START(m_pointCloudThreadPool, task);
}

void CompressionManager::compressImageAsync(const QImage& image, int cameraId,
                                            VideoCodec codec, CompressionLevel level,
                                            std::function<void(const CompressionResult&)> callback)
{
    if (!callback) {
        qCWarning(compressionLog) << "异步图像压缩回调函数为空";
        return;
    }

    // 创建压缩任务
    auto task = [this, image, cameraId, codec, level, callback]() {
        CompressionResult result = compressImage(image, cameraId, codec, level);

        // 在主线程中调用回调
        QMetaObject::invokeMethod(this, [callback, result]() {
            callback(result);
        }, Qt::QueuedConnection);
    };

    // 提交到线程池,添加兼容性定义
    //m_imageThreadPool->start(task);
    // 使用兼容性宏提交到线程池
    QT_THREADPOOL_START(m_imageThreadPool, task);
}

#ifdef ROS_BUILD
void CompressionManager::compressROSPointCloudAsync(const sensor_msgs::PointCloud2& rosCloud,
                                                    CompressionLevel level,
                                                    std::function<void(const CompressionResult&)> callback)
{
    // 转换ROS点云格式
    CrossPlatformPointCloud cloud = convertROSPointCloud(rosCloud);
    compressPointCloudAsync(cloud, level, callback);
}

void CompressionManager::compressROSImageAsync(const sensor_msgs::Image& rosImage, int cameraId,
                                               VideoCodec codec, CompressionLevel level,
                                               std::function<void(const CompressionResult&)> callback)
{
    // 转换ROS图像格式
    QImage image = convertROSImage(rosImage);
    compressImageAsync(image, cameraId, codec, level, callback);
}
#endif

// ========== 解压缩接口 ==========

CrossPlatformPointCloud CompressionManager::decompressPointCloud(const QByteArray& compressedData,
                                                                 const QString& method)
{
    if (compressedData.isEmpty()) {
        qCWarning(compressionLog) << "压缩数据为空";
        return CrossPlatformPointCloud();
    }

    QString detectedMethod = method;
    if (detectedMethod == "auto") {
        // 自动检测压缩方法
        if (m_dracoAvailable && m_dracoHelper) {
            // 尝试Draco解压
            CrossPlatformPointCloud cloud = m_dracoHelper->decompressToPointCloud(compressedData);
            if (!cloud.isEmpty()) {
                return cloud;
            }
        }
        // 尝试简单解压
        detectedMethod = "simple";
    }

    if (detectedMethod == "draco" && m_dracoAvailable && m_dracoHelper) {
        return m_dracoHelper->decompressToPointCloud(compressedData);
    } else if (detectedMethod == "simple") {
        return decompressPointCloudSimple(compressedData);
    }

    qCWarning(compressionLog) << "不支持的解压方法:" << detectedMethod;
    return CrossPlatformPointCloud();
}

QImage CompressionManager::decompressImage(const QByteArray& compressedData, VideoCodec codec)
{
    if (compressedData.isEmpty()) {
        qCWarning(compressionLog) << "压缩图像数据为空";
        return QImage();
    }

    if (codec == CODEC_JPEG) {
        // JPEG解压
        QImage image;
        if (image.loadFromData(compressedData, "JPEG")) {
            return image;
        }
    } else if (codec == CODEC_H264) {
        // H264解压需要外部解码器支持
        qCWarning(compressionLog) << "H264解压需要外部解码器";
    }

    return QImage();
}

QmlPointCloudData CompressionManager::decompressPointCloudToQml(const QByteArray& compressedData,
                                                                const QString& method)
{
    if (m_dracoAvailable && m_dracoHelper && (method == "draco" || method == "auto")) {
        return m_dracoHelper->decompressToQmlFormat(compressedData);
    }

    // 转换简单解压结果
    CrossPlatformPointCloud cloud = decompressPointCloudSimple(compressedData);
    return DracoHelper::convertToQmlFormat(cloud);
}

// ========== H264专用接口 ==========

bool CompressionManager::initializeH264Encoder(int cameraId, int width, int height,
                                               int fps, int bitrate)
{
    if (cameraId < 0 || cameraId >= 4) {
        qCWarning(compressionLog) << "H264编码器初始化：相机ID无效" << cameraId;
        return false;
    }

#ifdef ENABLE_FFMPEG
    QMutexLocker locker(&m_h264Mutexes[cameraId]);

    // 清理现有编码器
    m_h264Contexts[cameraId].cleanup();

    // 设置参数
    m_h264Contexts[cameraId].width = width;
    m_h264Contexts[cameraId].height = height;
    m_h264Contexts[cameraId].frameRate = fps;
    m_h264Contexts[cameraId].bitrate = bitrate;
    m_h264Contexts[cameraId].gopSize = fps; // 1秒一个关键帧

    // 尝试硬件编码器
    if (initializeH264HardwareEncoder(cameraId, width, height)) {
        qCInfo(compressionLog) << "相机" << cameraId << "H264硬件编码器初始化成功";
        return true;
    }

    // 回退到软件编码器
    if (initializeH264SoftwareEncoder(cameraId, width, height)) {
        qCInfo(compressionLog) << "相机" << cameraId << "H264软件编码器初始化成功";
        return true;
    }

    qCWarning(compressionLog) << "相机" << cameraId << "H264编码器初始化失败";
    return false;
#else
    qCWarning(compressionLog) << "FFmpeg未启用，无法初始化H264编码器";
    return false;
#endif
}

void CompressionManager::resetH264Encoder(int cameraId)
{
    if (cameraId == -1) {
        // 重置所有相机
        for (int i = 0; i < 4; i++) {
            resetH264Encoder(i);
        }
        return;
    }

    if (cameraId < 0 || cameraId >= 4) {
        return;
    }

    QMutexLocker locker(&m_h264Mutexes[cameraId]);
    m_h264Contexts[cameraId].cleanup();
    m_forceNextKeyFrame[cameraId] = true;

    qCInfo(compressionLog) << "相机" << cameraId << "H264编码器已重置";
}

bool CompressionManager::forceKeyFrame(int cameraId)
{
    if (cameraId < 0 || cameraId >= 4) {
        return false;
    }

    m_forceNextKeyFrame[cameraId] = true;
    qCInfo(compressionLog) << "相机" << cameraId << "下一帧将强制为关键帧";
    return true;
}

// ========== H264缓存管理 ==========

QByteArray CompressionManager::getLatestKeyFrame(int cameraId, CompressionLevel quality)
{
    if (m_h264CacheManager) {
        return m_h264CacheManager->getLatestKeyFrame(cameraId, quality);
    }
    return QByteArray();
}

bool CompressionManager::hasKeyFrameCache(int cameraId) const
{
    if (m_h264CacheManager) {
        QByteArray keyFrame = m_h264CacheManager->getLatestKeyFrame(cameraId);
        return !keyFrame.isEmpty();
    }
    return false;
}

void CompressionManager::clearKeyFrameCache(int cameraId)
{
    // H264CacheManager的清理功能将在其内部实现
    Q_UNUSED(cameraId)
}

// ========== 配置接口 ==========

void CompressionManager::setH264Parameters(int cameraId, int bitrate, int fps, int gopSize)
{
    if (cameraId < 0 || cameraId >= 4) {
        return;
    }

    QMutexLocker locker(&m_h264Mutexes[cameraId]);
    m_h264Contexts[cameraId].bitrate = bitrate;
    m_h264Contexts[cameraId].frameRate = fps;
    m_h264Contexts[cameraId].gopSize = gopSize;

    qCInfo(compressionLog) << "相机" << cameraId << "H264参数已更新: 码率=" << bitrate
                          << ", 帧率=" << fps << ", GOP=" << gopSize;
}

void CompressionManager::setDracoQuantizationBits(int positionBits, int colorBits)
{
    if (m_dracoHelper) {
        m_dracoHelper->setQuantizationBits(positionBits, colorBits);
        qCInfo(compressionLog) << "Draco量化位数已设置: 位置=" << positionBits
                              << ", 颜色=" << colorBits;
    }
}

void CompressionManager::setDracoCompressionSpeed(int speed)
{
    if (m_dracoHelper) {
        m_dracoHelper->setCompressionSpeed(speed);
        qCInfo(compressionLog) << "Draco压缩速度已设置:" << speed;
    }
}

// ========== 状态查询接口 ==========

bool CompressionManager::isDracoAvailable() const
{
    return m_dracoAvailable;
}

bool CompressionManager::isH264Available() const
{
    return m_h264Available;
}

bool CompressionManager::isH264HardwareAvailable() const
{
#ifdef ENABLE_FFMPEG
    // 检查是否有任何相机使用硬件编码
    for (int i = 0; i < 4; i++) {
        if (m_h264Contexts[i].hardwareAccelerated) {
            return true;
        }
    }
#endif
    return false;
}

QStringList CompressionManager::getAvailableH264Encoders() const
{
    QStringList encoders;

#ifdef ENABLE_FFMPEG
    // 检查可用的H264编码器
    const AVCodec* codec = nullptr;
    void* iter = nullptr;

    while ((codec = av_codec_iterate(&iter))) {
        if (codec->type == AVMEDIA_TYPE_VIDEO &&
            codec->id == AV_CODEC_ID_H264 &&
            av_codec_is_encoder(codec)) {
            encoders << QString::fromLocal8Bit(codec->name);
        }
    }
#endif

    return encoders;
}

int CompressionManager::getQueuedTaskCount() const
{
    QMutexLocker locker(&m_queueMutex);
    return m_taskQueue.size();
}

void CompressionManager::clearTaskQueue()
{
    QMutexLocker locker(&m_queueMutex);
    m_taskQueue.clear();
    qCInfo(compressionLog) << "任务队列已清空";
}

// ========== 统计信息 ==========

CompressionManager::CompressionStats CompressionManager::getStats() const
{
    QMutexLocker locker(&m_statsMutex);
    CompressionStats stats = m_stats;

    // 计算平均值
    if (stats.totalFramesProcessed > 0) {
        stats.averageCompressionRatio = (float)stats.totalBytesOriginal / stats.totalBytesCompressed;
        stats.averageProcessingTime = stats.totalProcessingTime / stats.totalFramesProcessed;
    }

    if (stats.dracoFramesProcessed > 0) {
        stats.dracoAverageRatio = stats.dracoTotalBytesOriginal / stats.dracoTotalBytesCompressed;
        stats.dracoAverageProcessingTime = stats.dracoTotalProcessingTime / stats.dracoFramesProcessed;
    }

    for (int i = 0; i < 4; i++) {
        if (stats.h264FramesProcessed[i] > 0) {
            stats.h264AverageRatio[i] = stats.h264TotalBytesOriginal[i] / stats.h264TotalBytesCompressed[i];
        }
        stats.h264EncoderActive[i] = m_h264Contexts[i].initialized;
    }

    return stats;
}

void CompressionManager::resetStats()
{
    QMutexLocker locker(&m_statsMutex);
    m_stats = CompressionStats();
    qCInfo(compressionLog) << "压缩统计信息已重置";
}

// ========== 私有实现方法 ==========

bool CompressionManager::initializeDraco()
{
#ifdef ENABLE_DRACO
    m_dracoHelper = std::make_unique<DracoHelper>(this);

    if (m_dracoHelper->initialize()) {
        m_dracoAvailable = true;

        // 连接Draco信号
        connect(m_dracoHelper.get(), &DracoHelper::compressionCompleted,
                this, &CompressionManager::onDracoCompressionCompleted);
        connect(m_dracoHelper.get(), &DracoHelper::errorOccurred,
                this, &CompressionManager::onDracoError);

        qCInfo(compressionLog) << "Draco压缩库初始化成功";
        return true;
    } else {
        qCWarning(compressionLog) << "Draco压缩库初始化失败";
        m_dracoHelper.reset();
    }
#else
    qCInfo(compressionLog) << "Draco压缩库未启用";
#endif

    return false;
}

bool CompressionManager::initializeH264()
{
#ifdef ENABLE_FFMPEG
    // 初始化FFmpeg
    avformat_network_init();

    // 检查H264编码器可用性
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        qCWarning(compressionLog) << "找不到H264编码器";
        return false;
    }

    m_h264Available = true;
    qCInfo(compressionLog) << "H264编码器初始化成功";
    return true;
#else
    qCInfo(compressionLog) << "FFmpeg未启用，H264编码器不可用";
    return false;
#endif
}

void CompressionManager::cleanupDraco()
{
    if (m_dracoHelper) {
        m_dracoHelper->cleanup();
        m_dracoHelper.reset();
    }
    m_dracoAvailable = false;
}

void CompressionManager::cleanupH264()
{
    for (int i = 0; i < 4; i++) {
        QMutexLocker locker(&m_h264Mutexes[i]);
        m_h264Contexts[i].cleanup();
    }
    m_h264Available = false;
}

#ifdef ENABLE_FFMPEG
bool CompressionManager::initializeH264HardwareEncoder(int cameraId, int width, int height)
{
    H264EncoderContext& ctx = m_h264Contexts[cameraId];

    // 尝试硬件编码器列表
    QStringList hwEncoders = {"h264_nvenc", "h264_qsv", "h264_vaapi", "h264_videotoolbox"};

    for (const QString& encoderName : hwEncoders) {
        const AVCodec* codec = avcodec_find_encoder_by_name(encoderName.toLocal8Bit().constData());
        if (!codec) {
            continue;
        }

        ctx.codecContext = avcodec_alloc_context3(codec);
        if (!ctx.codecContext) {
            continue;
        }

        // 设置编码参数
        ctx.codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
        ctx.codecContext->width = width;
        ctx.codecContext->height = height;
        ctx.codecContext->time_base = {1, ctx.frameRate};
        ctx.codecContext->framerate = {ctx.frameRate, 1};
        ctx.codecContext->bit_rate = ctx.bitrate;
        ctx.codecContext->gop_size = ctx.gopSize;
        ctx.codecContext->max_b_frames = 0; // 不使用B帧以降低延迟
        ctx.codecContext->pix_fmt = AV_PIX_FMT_YUV420P;

        // 根据不同的硬件编码器设置不同的选项
        if (encoderName == "h264_nvenc") {
            // NVENC特定设置
            av_opt_set(ctx.codecContext->priv_data, "preset", "llhp", 0); // low latency high performance
            av_opt_set(ctx.codecContext->priv_data, "rc", "cbr", 0); // constant bitrate
            av_opt_set(ctx.codecContext->priv_data, "delay", "0", 0); // 零延迟
            av_opt_set(ctx.codecContext->priv_data, "zerolatency", "1", 0);
            // 可选：设置GPU ID（如果有多个GPU）
            // av_opt_set_int(ctx.codecContext->priv_data, "gpu", 0, 0);
        }
        else if (encoderName == "h264_qsv") {
            // Intel Quick Sync特定设置
            av_opt_set(ctx.codecContext->priv_data, "preset", "veryfast", 0);
            av_opt_set(ctx.codecContext->priv_data, "async_depth", "1", 0); // 减少延迟
            av_opt_set(ctx.codecContext->priv_data, "look_ahead", "0", 0); // 禁用前瞻以减少延迟
        }
        else if (encoderName == "h264_vaapi") {
            // VA-API特定设置
            av_opt_set(ctx.codecContext->priv_data, "rc_mode", "CBR", 0);
            av_opt_set_int(ctx.codecContext->priv_data, "quality", 0, 0); // 0表示最快速度
        }
        else if (encoderName == "h264_videotoolbox") {
            // VideoToolbox (macOS)特定设置
            av_opt_set(ctx.codecContext->priv_data, "realtime", "1", 0);
            av_opt_set(ctx.codecContext->priv_data, "allow_sw", "0", 0); // 禁止回退到软件编码
        }

        // 打开编码器
        if (avcodec_open2(ctx.codecContext, codec, nullptr) >= 0) {
            // 分配帧
            ctx.frame = av_frame_alloc();
            ctx.packet = av_packet_alloc();

            if (ctx.frame && ctx.packet) {
                ctx.frame->format = ctx.codecContext->pix_fmt;
                ctx.frame->width = width;
                ctx.frame->height = height;

                if (av_frame_get_buffer(ctx.frame, 32) >= 0) {
                    // 硬件编码器创建swsContext进行格式转换
                    qDebug() << "为硬件编码器创建swsContext";
                    ctx.swsContext = sws_getContext(
                        width, height, AV_PIX_FMT_RGB24,        // 输入格式 BGRA->RGB24
                        width, height, AV_PIX_FMT_YUV420P,     // 输出格式
                        SWS_BILINEAR, nullptr, nullptr, nullptr
                        );

                    if (!ctx.swsContext) {
                        qCritical() << "硬件编码器swsContext创建失败";
                        ctx.cleanup();
                        continue; // 尝试下一个编码器
                    }
                    ctx.initialized = true;
                    ctx.hardwareAccelerated = true;
                    ctx.codecName = encoderName;

                    qCInfo(compressionLog) << "相机" << cameraId << "硬件H264编码器初始化成功:" << encoderName;
                    return true;
                }
            }
        }

        // 清理失败的尝试
        ctx.cleanup();
    }

    return false;
}

bool CompressionManager::initializeH264SoftwareEncoder(int cameraId, int width, int height)
{
    H264EncoderContext& ctx = m_h264Contexts[cameraId];

    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        return false;
    }

    ctx.codecContext = avcodec_alloc_context3(codec);
    if (!ctx.codecContext) {
        return false;
    }

    // 设置编码参数
    ctx.codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
    ctx.codecContext->width = width;
    ctx.codecContext->height = height;
    ctx.codecContext->time_base = {1, ctx.frameRate};
    ctx.codecContext->framerate = {ctx.frameRate, 1};
    ctx.codecContext->bit_rate = ctx.bitrate;
    ctx.codecContext->gop_size = ctx.gopSize;
    ctx.codecContext->max_b_frames = 0;
    ctx.codecContext->pix_fmt = AV_PIX_FMT_YUV420P;

    // 软件编码器优化（这些preset对x264有效）
    av_opt_set(ctx.codecContext->priv_data, "preset", "ultrafast", 0); // x264有效
    av_opt_set(ctx.codecContext->priv_data, "tune", "zerolatency", 0); // x264有效
    av_opt_set(ctx.codecContext->priv_data, "profile", "baseline", 0);

    if (avcodec_open2(ctx.codecContext, codec, nullptr) < 0) {
        ctx.cleanup();
        return false;
    }

    // 分配帧和包
    ctx.frame = av_frame_alloc();
    ctx.packet = av_packet_alloc();

    if (!ctx.frame || !ctx.packet) {
        ctx.cleanup();
        return false;
    }

    ctx.frame->format = ctx.codecContext->pix_fmt;
    ctx.frame->width = width;
    ctx.frame->height = height;

    if (av_frame_get_buffer(ctx.frame, 32) < 0) {
        ctx.cleanup();
        return false;
    }

    // 初始化颜色空间转换 更改AV_PIX_FMT_RGB24->RGB32 BGRA
    ctx.swsContext = sws_getContext(
        width, height, AV_PIX_FMT_RGB24,
        width, height, AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, nullptr, nullptr, nullptr
    );

    if (!ctx.swsContext) {
        qCritical() << "sws_getContext创建失败";
        ctx.cleanup();
        return false;
    }

    ctx.initialized = true;
    ctx.hardwareAccelerated = false;
    ctx.codecName = QString::fromLocal8Bit(codec->name);

    return true;
}

CompressionResult CompressionManager::encodeH264Frame(int cameraId, const QImage& image, bool forceKeyFrame)
{
    CompressionResult result;
    result.cameraId = cameraId;
    result.frameSequence = m_frameSequenceGenerator.fetch_add(1, std::memory_order_relaxed);
    result.originalSize = image.sizeInBytes();
    result.compressionMethod = "h264";

    if (cameraId < 0 || cameraId >= 4) {
        result.errorMessage = "相机ID无效";
        return result;
    }

    QMutexLocker locker(&m_h264Mutexes[cameraId]);
    H264EncoderContext& ctx = m_h264Contexts[cameraId];

    if (!ctx.initialized) {
        result.errorMessage = "H264编码器未初始化";
        return result;
    }

    // 转换QImage到AVFrame
    AVFrame* frame = convertImageToAVFrame(image, cameraId);
    if (!frame) {
        result.errorMessage = "图像格式转换失败";
        return result;
    }

    // 设置帧类型
    if (forceKeyFrame) {
        frame->pict_type = AV_PICTURE_TYPE_I;
        frame->key_frame = 1;
    } else {
        frame->pict_type = AV_PICTURE_TYPE_NONE;
        frame->key_frame = 0;
    }

    // 设置帧序号
    frame->pts = result.frameSequence;

    // 编码帧
    result = encodeFrameInternal(cameraId, frame, forceKeyFrame);

    // 清理
    av_frame_free(&frame);

    return result;
}

CompressionResult CompressionManager::encodeFrameInternal(int cameraId, AVFrame* frame, bool keyFrame)
{
    CompressionResult result;
    result.cameraId = cameraId;
    result.isKeyFrame = keyFrame;
    result.compressionMethod = "h264";

    H264EncoderContext& ctx = m_h264Contexts[cameraId];

    // 发送帧到编码器
    int ret = avcodec_send_frame(ctx.codecContext, frame);
    if (ret < 0) {
        result.errorMessage = QString("发送帧到编码器失败: %1").arg(ret);
        return result;
    }

    // 接收编码后的数据
    ret = avcodec_receive_packet(ctx.codecContext, ctx.packet);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        // 需要更多输入或编码结束
        result.errorMessage = "编码器需要更多输入";
        return result;
    } else if (ret < 0) {
        result.errorMessage = QString("从编码器接收数据失败: %1").arg(ret);
        return result;
    }

    // 复制编码数据
    result.data = QByteArray((char*)ctx.packet->data, ctx.packet->size);
    result.success = true;
    result.compressedSize = ctx.packet->size;
    result.frameSequence = ctx.packet->pts;

    // 检查是否为关键帧
    result.isKeyFrame = (ctx.packet->flags & AV_PKT_FLAG_KEY) != 0;

    // 释放包
    av_packet_unref(ctx.packet);

    return result;
}

AVFrame* CompressionManager::convertImageToAVFrame(const QImage& image, int cameraId)
{
    qDebug() << "=== convertImageToAVFrame开始 ===";
    qDebug() << "相机ID:" << cameraId;
    qDebug() << "输入图像:" << image.width() << "x" << image.height() << "格式:" << image.format();

    H264EncoderContext& ctx = m_h264Contexts[cameraId];

    qDebug() << "检查编码器上下文:";
    qDebug() << "  ctx.frame:" << (void*)ctx.frame;
    qDebug() << "  ctx.swsContext:" << (void*)ctx.swsContext;
    qDebug() << "  ctx.initialized:" << ctx.initialized;

    if (!ctx.frame || !ctx.swsContext) {
        qWarning() << "编码器上下文无效: frame=" << (void*)ctx.frame
                   << "swsContext=" << (void*)ctx.swsContext;
        return nullptr;
    }

    qDebug() << "编码器设置:" << ctx.codecContext->width << "x" << ctx.codecContext->height;

    // 尺寸检查
    if (image.width() != ctx.codecContext->width ||
        image.height() != ctx.codecContext->height) {
        qWarning() << "图像尺寸不匹配编码器设置:"
                   << "输入" << image.width() << "x" << image.height()
                   << "编码器" << ctx.codecContext->width << "x" << ctx.codecContext->height;
        return nullptr;
    }

    qDebug() << "尺寸检查通过，开始格式转换";

    // 转换QImage为RGB24格式
    QImage rgbImage = image.convertToFormat(QImage::Format_RGB888);
    qDebug() << "转换为RGB888格式:" << rgbImage.width() << "x" << rgbImage.height()
             << "格式:" << rgbImage.format();

    if (rgbImage.isNull()) {
        qWarning() << "RGB格式转换失败";
        return nullptr;
    }

    // 设置输入数据
    const uint8_t* srcData[4] = {rgbImage.constBits(), nullptr, nullptr, nullptr};
    int srcLinesize[4] = {rgbImage.bytesPerLine(), 0, 0, 0};

    qDebug() << "输入数据信息:";
    qDebug() << "  数据指针:" << (void*)srcData[0];
    qDebug() << "  行字节数:" << srcLinesize[0];
    qDebug() << "  期望行字节数:" << (rgbImage.width() * 3);

    // 设置输出数据
    uint8_t* dstData[4] = {ctx.frame->data[0], ctx.frame->data[1], ctx.frame->data[2], nullptr};
    int dstLinesize[4] = {ctx.frame->linesize[0], ctx.frame->linesize[1], ctx.frame->linesize[2], 0};

    qDebug() << "输出数据信息:";
    qDebug() << "  Y平面:" << (void*)dstData[0] << "步长:" << dstLinesize[0];
    qDebug() << "  U平面:" << (void*)dstData[1] << "步长:" << dstLinesize[1];
    qDebug() << "  V平面:" << (void*)dstData[2] << "步长:" << dstLinesize[2];

    // 颜色空间转换
    qDebug() << "开始颜色空间转换 RGB->YUV420P";
    int height = sws_scale(ctx.swsContext, srcData, srcLinesize, 0,
                           ctx.codecContext->height, dstData, dstLinesize);

    qDebug() << "sws_scale返回值:" << height << "(期望:" << ctx.codecContext->height << ")";

    if (height <= 0) {
        qWarning() << "颜色空间转换失败，返回值:" << height;
        return nullptr;
    }

    // 复制帧
    qDebug() << "开始复制AVFrame";
    AVFrame* frame = av_frame_alloc();
    if (!frame) {
        qWarning() << "AVFrame分配失败";
        return nullptr;
    }

    frame->format = ctx.frame->format;
    frame->width = ctx.frame->width;
    frame->height = ctx.frame->height;

    qDebug() << "AVFrame设置: 格式=" << frame->format
             << "尺寸=" << frame->width << "x" << frame->height;

    if (av_frame_get_buffer(frame, 32) < 0) {
        qWarning() << "AVFrame缓冲区分配失败";
        av_frame_free(&frame);
        return nullptr;
    }

    if (av_frame_copy(frame, ctx.frame) < 0) {
        qWarning() << "AVFrame复制失败";
        av_frame_free(&frame);
        return nullptr;
    }

    qDebug() << "=== convertImageToAVFrame成功完成 ===";
    return frame;
    /*
    H264EncoderContext& ctx = m_h264Contexts[cameraId];

    if (!ctx.frame || !ctx.swsContext) {
        return nullptr;
    }

    // 转换QImage为RGB24格式
    QImage rgbImage = image.convertToFormat(QImage::Format_RGB888);

    // 设置输入数据
    const uint8_t* srcData[4] = {rgbImage.constBits(), nullptr, nullptr, nullptr};
    int srcLinesize[4] = {rgbImage.bytesPerLine(), 0, 0, 0};

    // 设置输出数据
    uint8_t* dstData[4] = {ctx.frame->data[0], ctx.frame->data[1], ctx.frame->data[2], nullptr};
    int dstLinesize[4] = {ctx.frame->linesize[0], ctx.frame->linesize[1], ctx.frame->linesize[2], 0};

    // 颜色空间转换
    int height = sws_scale(ctx.swsContext, srcData, srcLinesize, 0,
                          ctx.codecContext->height, dstData, dstLinesize);

    if (height <= 0) {
        return nullptr;
    }

    // 复制帧
    AVFrame* frame = av_frame_alloc();
    if (!frame) {
        return nullptr;
    }

    frame->format = ctx.frame->format;
    frame->width = ctx.frame->width;
    frame->height = ctx.frame->height;

    if (av_frame_get_buffer(frame, 32) < 0) {
        av_frame_free(&frame);
        return nullptr;
    }

    if (av_frame_copy(frame, ctx.frame) < 0) {
        av_frame_free(&frame);
        return nullptr;
    }

    return frame;
    */
}

#ifdef ROS_BUILD
CompressionResult CompressionManager::encodeH264Frame(int cameraId, const cv::Mat& cvImage, bool forceKeyFrame)
{
    // 转换cv::Mat到QImage
    QImage qImage;

    if (cvImage.channels() == 3) {
        qImage = QImage(cvImage.data, cvImage.cols, cvImage.rows,
                       cvImage.step, QImage::Format_RGB888).rgbSwapped();
    } else if (cvImage.channels() == 1) {
        qImage = QImage(cvImage.data, cvImage.cols, cvImage.rows,
                       cvImage.step, QImage::Format_Grayscale8);
    } else {
        CompressionResult result;
        result.errorMessage = "不支持的图像通道数";
        return result;
    }

    return encodeH264Frame(cameraId, qImage, forceKeyFrame);
}

AVFrame* CompressionManager::convertCVMatToAVFrame(const cv::Mat& cvImage, int cameraId)
{
    QImage qImage;

    if (cvImage.channels() == 3) {
        qImage = QImage(cvImage.data, cvImage.cols, cvImage.rows,
                       cvImage.step, QImage::Format_RGB888).rgbSwapped();
    } else if (cvImage.channels() == 1) {
        qImage = QImage(cvImage.data, cvImage.cols, cvImage.rows,
                       cvImage.step, QImage::Format_Grayscale8);
    } else {
        return nullptr;
    }

    return convertImageToAVFrame(qImage, cameraId);
}
#endif
#endif // ENABLE_FFMPEG

// ========== ROS格式转换 ==========

#ifdef ROS_BUILD
CrossPlatformPointCloud CompressionManager::convertROSPointCloud(const sensor_msgs::PointCloud2& rosCloud)
{
    CrossPlatformPointCloud cloud;
    cloud.timestamp.seconds = rosCloud.header.stamp.sec;
    cloud.timestamp.microseconds = rosCloud.header.stamp.nsec / 1000;
    cloud.frameId = QString::fromStdString(rosCloud.header.frame_id);

    // 解析点云字段
    int xOffset = -1, yOffset = -1, zOffset = -1;
    int rgbOffset = -1, intensityOffset = -1;

    for (size_t i = 0; i < rosCloud.fields.size(); ++i) {
        const auto& field = rosCloud.fields[i];
        if (field.name == "x") xOffset = field.offset;
        else if (field.name == "y") yOffset = field.offset;
        else if (field.name == "z") zOffset = field.offset;
        else if (field.name == "rgb") rgbOffset = field.offset;
        else if (field.name == "intensity") intensityOffset = field.offset;
    }

    if (xOffset < 0 || yOffset < 0 || zOffset < 0) {
        qCWarning(compressionLog) << "点云缺少必要的坐标字段";
        return cloud;
    }

    int numPoints = rosCloud.width * rosCloud.height;
    cloud.points.reserve(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        const uint8_t* ptr = &rosCloud.data[i * rosCloud.point_step];

        // 提取坐标
        float x = *reinterpret_cast<const float*>(ptr + xOffset);
        float y = *reinterpret_cast<const float*>(ptr + yOffset);
        float z = *reinterpret_cast<const float*>(ptr + zOffset);

        // 提取颜色
        uint8_t r = 255, g = 255, b = 255;
        if (rgbOffset >= 0) {
            uint32_t rgb = *reinterpret_cast<const uint32_t*>(ptr + rgbOffset);
            r = (rgb >> 16) & 0xFF;
            g = (rgb >> 8) & 0xFF;
            b = rgb & 0xFF;
        } else if (intensityOffset >= 0) {
            float intensity = *reinterpret_cast<const float*>(ptr + intensityOffset);
            mapIntensityToColor(intensity, r, g, b);
        }

        cloud.points.append(CrossPlatformPointCloud::Point(x, y, z, r, g, b));
    }

    return cloud;
}

QImage CompressionManager::convertROSImage(const sensor_msgs::Image& rosImage)
{
    QImage::Format format = QImage::Format_RGB888;

    if (rosImage.encoding == "bgr8") {
        format = QImage::Format_RGB888;
    } else if (rosImage.encoding == "rgb8") {
        format = QImage::Format_RGB888;
    } else if (rosImage.encoding == "mono8") {
        format = QImage::Format_Grayscale8;
    } else {
        qCWarning(compressionLog) << "不支持的ROS图像编码:" << QString::fromStdString(rosImage.encoding);
        return QImage();
    }

    QImage image(rosImage.data.data(), rosImage.width, rosImage.height,
                 rosImage.step, format);

    // BGR需要转换为RGB
    if (rosImage.encoding == "bgr8") {
        image = image.rgbSwapped();
    }

    return image.copy(); // 深拷贝确保数据安全
}

cv::Mat CompressionManager::convertROSImageToCV(const sensor_msgs::Image& rosImage)
{
    int cvType = CV_8UC3;

    if (rosImage.encoding == "bgr8" || rosImage.encoding == "rgb8") {
        cvType = CV_8UC3;
    } else if (rosImage.encoding == "mono8") {
        cvType = CV_8UC1;
    } else {
        qCWarning(compressionLog) << "不支持的ROS图像编码:" << QString::fromStdString(rosImage.encoding);
        return cv::Mat();
    }

    cv::Mat image(rosImage.height, rosImage.width, cvType,
                  const_cast<uint8_t*>(rosImage.data.data()), rosImage.step);

    return image.clone(); // 深拷贝
}
#endif

// ========== 私有实现方法 ==========

QByteArray CompressionManager::encodeImageToJPEG(const QImage& image, int quality)
{
    QByteArray data;
    QBuffer buffer(&data);
    buffer.open(QIODevice::WriteOnly);

    if (image.save(&buffer, "JPEG", quality)) {
        return data;
    }

    qCWarning(compressionLog) << "JPEG编码失败";
    return QByteArray();
}

QByteArray CompressionManager::compressPointCloudSimple(const CrossPlatformPointCloud& cloud)
{
    if (cloud.isEmpty()) {
        return QByteArray();
    }

    // 使用Qt的数据流进行简单压缩
    QByteArray uncompressedData = cloud.toByteArray();
    return qCompress(uncompressedData, 6); // 压缩级别6
}

CrossPlatformPointCloud CompressionManager::decompressPointCloudSimple(const QByteArray& data)
{
    if (data.isEmpty()) {
        return CrossPlatformPointCloud();
    }

    QByteArray uncompressedData = qUncompress(data);
    if (uncompressedData.isEmpty()) {
        qCWarning(compressionLog) << "简单解压失败";
        return CrossPlatformPointCloud();
    }

    return CrossPlatformPointCloud::fromByteArray(uncompressedData);
}

void CompressionManager::logError(const QString& operation, const QString& error)
{
    QString message = QString("%1失败: %2").arg(operation).arg(error);
    qCCritical(compressionLog) << message;
    emit compressionError(message);
}

void CompressionManager::updateStats(const CompressionResult& result)
{
    QMutexLocker locker(&m_statsMutex);

    m_stats.totalFramesProcessed++;
    m_stats.totalBytesOriginal += result.originalSize;
    m_stats.totalBytesCompressed += result.compressedSize;
    m_stats.totalProcessingTime += result.processingTime;

    if (!result.success) {
        m_stats.totalErrors++;
        return;
    }

    // 分类统计
    if (result.compressionMethod == "draco") {
        m_stats.dracoFramesProcessed++;
        m_stats.dracoTotalBytesOriginal += result.originalSize;
        m_stats.dracoTotalBytesCompressed += result.compressedSize;
        m_stats.dracoTotalProcessingTime += result.processingTime;
    } else if (result.compressionMethod == "h264") {
        int cameraId = result.cameraId;
        if (cameraId >= 0 && cameraId < 4) {
            m_stats.h264FramesProcessed[cameraId]++;
            m_stats.h264TotalBytesOriginal[cameraId] += result.originalSize;
            m_stats.h264TotalBytesCompressed[cameraId] += result.compressedSize;

            if (result.isKeyFrame) {
                m_stats.h264KeyFrames[cameraId]++;
            }
        }
    }
}

int CompressionManager::getJPEGQuality(CompressionLevel level) const
{
    switch (level) {
    case HIGH_QUALITY: return 95;
    case MEDIUM_QUALITY: return 80;
    case LOW_QUALITY: return 60;
    case MOBILE_OPTIMIZED: return 40;
    default: return 80;
    }
}

void CompressionManager::mapIntensityToColor(float intensity, quint8& r, quint8& g, quint8& b)
{
    // 将强度值映射到彩色图
    intensity = qBound(0.0f, intensity, 1.0f);

    if (intensity < 0.25f) {
        // 蓝色到青色
        r = 0;
        g = static_cast<quint8>(intensity * 4 * 255);
        b = 255;
    } else if (intensity < 0.5f) {
        // 青色到绿色
        r = 0;
        g = 255;
        b = static_cast<quint8>((0.5f - intensity) * 4 * 255);
    } else if (intensity < 0.75f) {
        // 绿色到黄色
        r = static_cast<quint8>((intensity - 0.5f) * 4 * 255);
        g = 255;
        b = 0;
    } else {
        // 黄色到红色
        r = 255;
        g = static_cast<quint8>((1.0f - intensity) * 4 * 255);
        b = 0;
    }
}

// ========== H264编码器上下文清理 ==========

void H264EncoderContext::cleanup()
{
#ifdef ENABLE_FFMPEG
    if (frame) {
        av_frame_free(&frame);
        frame = nullptr;
    }

    if (packet) {
        av_packet_free(&packet);
        packet = nullptr;
    }

    if (swsContext) {
        sws_freeContext(swsContext);
        swsContext = nullptr;
    }

    if (codecContext) {
        avcodec_free_context(&codecContext);
        codecContext = nullptr;
    }

    if (hwDeviceCtx) {
        av_buffer_unref(&hwDeviceCtx);
        hwDeviceCtx = nullptr;
    }
#endif

    initialized = false;
    hardwareAccelerated = false;
    codecName.clear();
}

// ========== 槽函数实现 ==========

void CompressionManager::onDracoCompressionCompleted(const DracoCompressionResult& result)
{
    // 转换为标准压缩结果
    CompressionResult compressionResult;
    compressionResult.success = result.success;
    compressionResult.data = result.dracoData;
    compressionResult.originalSize = result.originalSize;
    compressionResult.compressedSize = result.compressedSize;
    compressionResult.processingTime = result.processingTime;
    compressionResult.errorMessage = result.errorMessage;
    compressionResult.compressionMethod = "draco";
    compressionResult.isDracoCompressed = true;
    compressionResult.quantizationBits = result.quantizationBits;

    emit pointCloudCompressionCompleted(compressionResult);
}

void CompressionManager::onDracoError(const QString& error)
{
    logError("Draco压缩", error);
}
