// compressionmanager.h - 修正版头文件
#ifndef COMPRESSION_MANAGER_H
#define COMPRESSION_MANAGER_H

#include <atomic>
#include <memory>

#include <QDateTime>
#include <QElapsedTimer>
#include <QFuture>
#include <QImage>
#include <QLoggingCategory>
#include <QMutex>
#include <QObject>
#include <QQueue>
#include <QThreadPool>
#include <QtConcurrent>

#ifdef ROS_BUILD
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#endif

#ifdef ENABLE_FFMPEG
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#endif

#include "draco_helper.h"
#include "h264_cache_manager.h"
#include "networkprotocol.h"

Q_DECLARE_LOGGING_CATEGORY(compressionLog)

struct CompressionResult {
    bool success;
    QByteArray data;
    qint64 originalSize;
    qint64 compressedSize;
    qint64 processingTime;
    QString errorMessage;
    QString compressionMethod;

    bool isDracoCompressed;
    int quantizationBits;
    bool isKeyFrame;
    uint32_t frameSequence;
    int cameraId;

    CompressionResult() : success(false), originalSize(0), compressedSize(0),
        processingTime(0), isDracoCompressed(false), quantizationBits(0),
        isKeyFrame(false), frameSequence(0), cameraId(-1) {}

    float compressionRatio() const {
        return originalSize > 0 ? (float)originalSize / compressedSize : 0.0f;
    }
};

struct H264EncoderContext {
    bool initialized = false;
    bool hardwareAccelerated = false;
    QString codecName;

#ifdef ENABLE_FFMPEG
    AVCodecContext* codecContext = nullptr;
    AVFrame* frame = nullptr;
    AVPacket* packet = nullptr;
    SwsContext* swsContext = nullptr;
    AVBufferRef* hwDeviceCtx = nullptr;
    AVPixelFormat hwPixelFormat = AV_PIX_FMT_NONE;
#endif

    int width = 0;
    int height = 0;
    int frameRate = 30;
    int bitrate = 2000000;
    int gopSize = 30;

    H264EncoderContext() = default;
    ~H264EncoderContext() { cleanup(); }
    void cleanup();
};

struct CompressionTask {
    enum Type { POINTCLOUD, IMAGE } type;
    QByteArray inputData;
    CompressionLevel level;
    VideoCodec codec;
    int cameraId = -1;
    uint32_t requestId = 0;
    qint64 timestamp;

    std::function<void(const CompressionResult&)> callback;

    CompressionTask() : timestamp(QDateTime::currentMSecsSinceEpoch()) {}
};

class CompressionManager : public QObject
{
    Q_OBJECT

public:
    explicit CompressionManager(QObject *parent = nullptr);
    ~CompressionManager();

    // ========== 初始化和清理 ==========
    bool initialize();
    void cleanup();

    // ========== 主要压缩接口 ==========
    CompressionResult compressPointCloud(const CrossPlatformPointCloud& cloud,
                                         CompressionLevel level = MEDIUM_QUALITY);
    CompressionResult compressImage(const QImage& image, int cameraId = 0,
                                    VideoCodec codec = CODEC_H264,
                                    CompressionLevel level = MEDIUM_QUALITY);

    void compressPointCloudAsync(const CrossPlatformPointCloud& cloud,
                                 CompressionLevel level,
                                 std::function<void(const CompressionResult&)> callback);
    void compressImageAsync(const QImage& image, int cameraId,
                            VideoCodec codec, CompressionLevel level,
                            std::function<void(const CompressionResult&)> callback);

#ifdef ROS_BUILD
    void compressROSPointCloudAsync(const sensor_msgs::PointCloud2& rosCloud,
                                    CompressionLevel level,
                                    std::function<void(const CompressionResult&)> callback);
    void compressROSImageAsync(const sensor_msgs::Image& rosImage, int cameraId,
                               VideoCodec codec, CompressionLevel level,
                               std::function<void(const CompressionResult&)> callback);
#endif

    // ========== 解压缩接口 ==========
    CrossPlatformPointCloud decompressPointCloud(const QByteArray& compressedData,
                                                 const QString& method = "auto");
    QImage decompressImage(const QByteArray& compressedData, VideoCodec codec);
    QmlPointCloudData decompressPointCloudToQml(const QByteArray& compressedData,
                                                const QString& method = "auto");

    // ========== H264专用接口 ==========
    bool initializeH264Encoder(int cameraId, int width = 1920, int height = 1080,
                               int fps = 30, int bitrate = 2000000);
    void resetH264Encoder(int cameraId = -1);
    bool forceKeyFrame(int cameraId);
    QByteArray getLatestKeyFrame(int cameraId, CompressionLevel quality = MEDIUM_QUALITY);
    bool hasKeyFrameCache(int cameraId) const;
    void clearKeyFrameCache(int cameraId = -1);

    // ========== 配置接口 ==========
    void setH264Parameters(int cameraId, int bitrate, int fps, int gopSize);
    void setDracoQuantizationBits(int positionBits, int colorBits = 8);
    void setDracoCompressionSpeed(int speed);
    void setLowFrequencyPointCloudMode(bool enabled) { m_lowFrequencyMode = enabled; }

    // ========== 状态查询接口 ==========
    bool isDracoAvailable() const;
    bool isH264Available() const;
    bool isH264HardwareAvailable() const;
    QStringList getAvailableH264Encoders() const;
    int getQueuedTaskCount() const;
    void clearTaskQueue();

    struct CompressionStats {
        qint64 totalFramesProcessed;
        qint64 totalBytesOriginal;
        qint64 totalBytesCompressed;
        float averageCompressionRatio;
        float averageProcessingTime;
        qint64 totalErrors;
        qint64 totalProcessingTime;

        qint64 dracoFramesProcessed;
        qint64 dracoTotalBytesOriginal;
        qint64 dracoTotalBytesCompressed;
        qint64 dracoTotalProcessingTime;
        float dracoAverageRatio;
        float dracoAverageProcessingTime;

        qint64 h264FramesProcessed[4];
        qint64 h264TotalBytesOriginal[4];
        qint64 h264TotalBytesCompressed[4];
        qint64 h264KeyFrames[4];
        float h264AverageRatio[4];
        bool h264EncoderActive[4];

        CompressionStats() {
            totalFramesProcessed = 0;
            totalBytesOriginal = 0;
            totalBytesCompressed = 0;
            averageCompressionRatio = 0;
            averageProcessingTime = 0;
            totalErrors = 0;
            totalProcessingTime = 0;

            dracoFramesProcessed = 0;
            dracoTotalBytesOriginal = 0;
            dracoTotalBytesCompressed = 0;
            dracoTotalProcessingTime = 0;
            dracoAverageRatio = 0;
            dracoAverageProcessingTime = 0;

            for (int i = 0; i < 4; i++) {
                h264FramesProcessed[i] = 0;
                h264TotalBytesOriginal[i] = 0;
                h264TotalBytesCompressed[i] = 0;
                h264KeyFrames[i] = 0;
                h264AverageRatio[i] = 0;
                h264EncoderActive[i] = false;
            }
        }
    };

    CompressionStats getStats() const;
    void resetStats();

private:
    // ========== H264实现 ==========
#ifdef ENABLE_FFMPEG
    bool initializeH264HardwareEncoder(int cameraId, int width, int height);
    bool initializeH264SoftwareEncoder(int cameraId, int width, int height);
    CompressionResult encodeH264Frame(int cameraId, const QImage& image, bool forceKeyFrame = false);
    CompressionResult encodeH264Frame(int cameraId, const cv::Mat& cvImage, bool forceKeyFrame = false);
    CompressionResult encodeFrameInternal(int cameraId, AVFrame* frame, bool keyFrame);
    AVFrame* convertImageToAVFrame(const QImage& image, int cameraId);
    AVFrame* convertCVMatToAVFrame(const cv::Mat& cvImage, int cameraId);
#endif

    // ========== ROS格式转换 ==========
#ifdef ROS_BUILD
    CrossPlatformPointCloud convertROSPointCloud(const sensor_msgs::PointCloud2& rosCloud);
    QImage convertROSImage(const sensor_msgs::Image& rosImage);
    cv::Mat convertROSImageToCV(const sensor_msgs::Image& rosImage);
#endif

    // ========== 私有实现方法 ==========
    void processTaskQueue();
    void submitTask(const CompressionTask& task);
    bool initializeDraco();
    bool initializeH264();
    void cleanupDraco();
    void cleanupH264();
    QByteArray encodeImageToJPEG(const QImage& image, int quality);
    QByteArray compressPointCloudSimple(const CrossPlatformPointCloud& cloud);
    CrossPlatformPointCloud decompressPointCloudSimple(const QByteArray& data);
    void logError(const QString& operation, const QString& error);
    void updateStats(const CompressionResult& result);
    int getJPEGQuality(CompressionLevel level) const;
    void mapIntensityToColor(float intensity, quint8& r, quint8& g, quint8& b);

    bool m_initialized;
    bool m_dracoAvailable;
    bool m_h264Available;
    bool m_lowFrequencyMode;

    std::unique_ptr<DracoHelper> m_dracoHelper;
    H264EncoderContext m_h264Contexts[4];
    QMutex m_h264Mutexes[4];
    bool m_forceNextKeyFrame[4];
    std::unique_ptr<H264CacheManager> m_h264CacheManager;
    QThreadPool* m_pointCloudThreadPool;
    QThreadPool* m_imageThreadPool;
    QQueue<CompressionTask> m_taskQueue;
    mutable QMutex m_queueMutex;
    mutable QMutex m_statsMutex;
    CompressionStats m_stats;
    QElapsedTimer m_timer;
    std::atomic<uint32_t> m_frameSequenceGenerator;

private slots:
    void onDracoCompressionCompleted(const DracoCompressionResult& result);
    void onDracoError(const QString& error);

signals:
    void pointCloudCompressionCompleted(const CompressionResult& result);
    void imageCompressionCompleted(const CompressionResult& result);
    void h264KeyFrameGenerated(int cameraId, uint32_t frameSequence, const QByteArray& data);
    void h264FrameGenerated(int cameraId, uint32_t frameSequence, const QByteArray& data, bool isKeyFrame);
    void compressionError(const QString& error);
    void h264EncoderError(int cameraId, const QString& error);
};

#endif // COMPRESSION_MANAGER_H
