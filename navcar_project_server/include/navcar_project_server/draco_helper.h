#ifndef DRACO_HELPER_H
#define DRACO_HELPER_H

#include <QObject>
#include <QByteArray>
#include <QLoggingCategory>
#include <QVector3D>
#include <QColor>
#include <QVariantList>
#include <QMutex>
#include<QElapsedTimer>

#ifdef ENABLE_DRACO
#include "draco/compression/encode.h"
#include "draco/compression/decode.h"
#include "draco/point_cloud/point_cloud_builder.h"
#include "draco/attributes/geometry_attribute.h"
#include "draco/core/vector_d.h"
#endif

#include "networkprotocol.h"

Q_DECLARE_LOGGING_CATEGORY(dracoLog)

// Draco压缩结果
struct DracoCompressionResult {
    bool success;
    QByteArray dracoData;           // 压缩后的Draco数据
    qint64 originalSize;            // 原始数据大小
    qint64 compressedSize;          // 压缩后大小
    qint64 processingTime;          // 处理时间(ms)
    QString errorMessage;
    
    // 压缩参数
    int quantizationBits;           // 量化位数
    int compressionLevel;           // 压缩级别
    float compressionRatio() const {
        return originalSize > 0 ? (float)originalSize / compressedSize : 0.0f;
    }
};

class DracoHelper : public QObject
{
    Q_OBJECT
    
public:
    explicit DracoHelper(QObject *parent = nullptr);
    ~DracoHelper();
    
    // 初始化和清理
    bool initialize();
    void cleanup();
    bool isAvailable() const { return m_initialized; }
    
    // 压缩接口 - 从CrossPlatformPointCloud压缩
    DracoCompressionResult compressPointCloud(
        const CrossPlatformPointCloud& cloud,
        CompressionLevel level = MEDIUM_QUALITY);
    
    // 解压缩接口 - 解压到CrossPlatformPointCloud
    CrossPlatformPointCloud decompressToPointCloud(const QByteArray& dracoData);
    
    // 解压缩接口 - 解压到QML显示格式
    QmlPointCloudData decompressToQmlFormat(const QByteArray& dracoData);
    
    // 工具函数 - 格式转换
    static QmlPointCloudData convertToQmlFormat(const CrossPlatformPointCloud& cloud);
    static CrossPlatformPointCloud convertFromQmlFormat(const QmlPointCloudData& qmlData);
    
    
    // 获取统计信息
    struct DracoStats {
        qint64 totalCompressions;
        qint64 totalDecompressions;
        qint64 totalBytesOriginal;
        qint64 totalBytesCompressed;
        float averageCompressionRatio;
        float averageCompressionTime;
        float averageDecompressionTime;
        qint64 totalErrors;
        
        DracoStats() : totalCompressions(0), totalDecompressions(0),
            totalBytesOriginal(0), totalBytesCompressed(0),
            averageCompressionRatio(0), averageCompressionTime(0),
            averageDecompressionTime(0), totalErrors(0) {}
    };
    
    DracoStats getStats() const;
    void resetStats();

    void setQuantizationBits(int positionBits, int colorBits);


    void setCompressionSpeed(int speed);


private:
    // Draco相关的内部函数
#ifdef ENABLE_DRACO
    std::unique_ptr<draco::PointCloud> createDracoPointCloud(
        const CrossPlatformPointCloud& cloud);
    
    CrossPlatformPointCloud extractFromDracoPointCloud(
        const draco::PointCloud& dracoCloud);
    
    QmlPointCloudData extractToQmlFormat(const draco::PointCloud& dracoCloud);
    
    draco::EncoderOptions getDracoEncoderOptions(CompressionLevel level);
    int getQuantizationBits(CompressionLevel level, bool isPosition = true);
#endif
    
    void updateStats(const DracoCompressionResult& result, bool isCompression);
    void logError(const QString& operation, const QString& error);
    
private:
    bool m_initialized;
    
    // 压缩参数
    int m_positionQuantizationBits;
    int m_colorQuantizationBits;
    int m_compressionSpeed;
    
    // 统计信息
    mutable QMutex m_statsMutex;
    DracoStats m_stats;
    
    QElapsedTimer m_timer;

signals:
    void compressionCompleted(const DracoCompressionResult& result);
    void decompressionCompleted(bool success);
    void errorOccurred(const QString& error);
};

#endif // DRACO_HELPER_H
