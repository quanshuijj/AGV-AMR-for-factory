#include"navcar_project_server/draco_helper.h"
#include <QDebug>
#include <QElapsedTimer>

#ifdef ENABLE_DRACO
#include "draco/compression/encode.h"
#include "draco/compression/decode.h"
#include "draco/point_cloud/point_cloud_builder.h"
#include "draco/attributes/geometry_attribute.h"
#include "draco/core/vector_d.h"
#include "draco/io/point_cloud_io.h"
#endif

Q_LOGGING_CATEGORY(dracoLog, "draco")

DracoHelper::DracoHelper(QObject *parent)
    : QObject(parent)
    , m_initialized(false)
    , m_positionQuantizationBits(11)  // 默认11位精度
    , m_colorQuantizationBits(8)      // 默认8位颜色精度
    , m_compressionSpeed(5)           // 默认中等速度
{
}

DracoHelper::~DracoHelper()
{
    cleanup();
}

bool DracoHelper::initialize()
{
#ifdef ENABLE_DRACO
    try {
        qCInfo(dracoLog) << "初始化Draco压缩库 v1.5.7";
        m_initialized = true;
        qCInfo(dracoLog) << "Draco初始化成功";
        return true;
    } catch (const std::exception& e) {
        qCCritical(dracoLog) << "Draco初始化失败:" << e.what();
        return false;
    }
#else
    qCWarning(dracoLog) << "Draco支持未编译";
    return false;
#endif
}

void DracoHelper::cleanup()
{
    if (m_initialized) {
        qCInfo(dracoLog) << "清理Draco资源";
        m_initialized = false;
    }
}

DracoCompressionResult DracoHelper::compressPointCloud(
    const CrossPlatformPointCloud& cloud, CompressionLevel level)
{
    DracoCompressionResult result;
    
    if (!m_initialized) {
        result.errorMessage = "Draco未初始化";
        return result;
    }
    
    if (cloud.isEmpty()) {
        result.errorMessage = "输入点云为空";
        return result;
    }
    
    m_timer.start();
    
#ifdef ENABLE_DRACO
    try {
        qCDebug(dracoLog) << QString("开始Draco压缩，点数: %1").arg(cloud.size());
        
        // 创建Draco点云
        auto dracoCloud = createDracoPointCloud(cloud);
        if (!dracoCloud) {
            result.errorMessage = "创建Draco点云失败";
            return result;
        }
        
        // 配置编码器
        draco::Encoder encoder;
        draco::EncoderOptions options = getDracoEncoderOptions(level);
        
        // 设置量化参数
        int posBits = getQuantizationBits(level, true);
        int colorBits = getQuantizationBits(level, false);
        
        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, posBits);
        encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, colorBits);
        encoder.SetSpeedOptions(m_compressionSpeed, m_compressionSpeed);
        
        // 压缩
        draco::EncoderBuffer buffer;
        const draco::Status status = encoder.EncodePointCloudToBuffer(*dracoCloud, &buffer);
        
        if (!status.ok()) {
            result.errorMessage = QString("Draco编码失败: %1").arg(status.error_msg());//.c_str());
            logError("Draco压缩", result.errorMessage);
            return result;
        }
        
        // 获取压缩数据
        result.dracoData = QByteArray(buffer.data(), buffer.size());
        result.originalSize = cloud.toByteArray().size();
        result.compressedSize = result.dracoData.size();
        result.processingTime = m_timer.elapsed();
        result.quantizationBits = posBits;
        result.compressionLevel = static_cast<int>(level);
        result.success = true;
        
        qCInfo(dracoLog) << QString("Draco压缩完成: %1KB -> %2KB, 压缩比: %3:1, 耗时: %4ms")
                                .arg(result.originalSize / 1024)
                                .arg(result.compressedSize / 1024)
                                .arg(result.compressionRatio(), 0, 'f', 1)
                                .arg(result.processingTime);
        
        updateStats(result, true);
        emit compressionCompleted(result);
        
    } catch (const std::exception& e) {
        result.errorMessage = QString("Draco压缩异常: %1").arg(e.what());
        logError("Draco压缩", result.errorMessage);
        result.processingTime = m_timer.elapsed();
    }
#else
    result.errorMessage = "Draco支持未编译";
#endif
    
    return result;
}

CrossPlatformPointCloud DracoHelper::decompressToPointCloud(const QByteArray& dracoData)
{
    CrossPlatformPointCloud result;
    
    if (!m_initialized) {
        qCWarning(dracoLog) << "Draco未初始化";
        return result;
    }
    
    if (dracoData.isEmpty()) {
        qCWarning(dracoLog) << "输入数据为空";
        return result;
    }
    
    m_timer.start();
    
#ifdef ENABLE_DRACO
    try {
        qCDebug(dracoLog) << QString("开始Draco解压缩，数据大小: %1 bytes").arg(dracoData.size());
        
        // 创建解码器缓冲区
        draco::DecoderBuffer buffer;
        buffer.Init(dracoData.constData(), dracoData.size());
        
        // 解码
        draco::Decoder decoder;
        auto statusWithCloud = decoder.DecodePointCloudFromBuffer(&buffer);
        
        if (!statusWithCloud.ok()) {
            QString error = QString("Draco解码失败: %1").arg(statusWithCloud.status().error_msg());//.c_str());
            logError("Draco解压缩", error);
            return result;
        }
        
        // 提取点云数据
        result = extractFromDracoPointCloud(*statusWithCloud.value());
        
        qint64 processingTime = m_timer.elapsed();
        qCDebug(dracoLog) << QString("Draco解压缩完成，点数: %1, 耗时: %2ms")
                                 .arg(result.size()).arg(processingTime);
        
        // 更新统计
        DracoCompressionResult stats;
        stats.success = true;
        stats.compressedSize = dracoData.size();
        stats.processingTime = processingTime;
        updateStats(stats, false);
        
        emit decompressionCompleted(true);
        
    } catch (const std::exception& e) {
        QString error = QString("Draco解压缩异常: %1").arg(e.what());
        logError("Draco解压缩", error);
        emit decompressionCompleted(false);
    }
#else
    qCWarning(dracoLog) << "Draco支持未编译";
#endif
    
    return result;
}

QmlPointCloudData DracoHelper::decompressToQmlFormat(const QByteArray& dracoData)
{
    QmlPointCloudData result;
    
    if (!m_initialized) {
        qCWarning(dracoLog) << "Draco未初始化";
        return result;
    }
    
    if (dracoData.isEmpty()) {
        qCWarning(dracoLog) << "输入数据为空";
        return result;
    }
    
    m_timer.start();
    
#ifdef ENABLE_DRACO
    try {
        qCDebug(dracoLog) << QString("开始Draco解压缩到QML格式，数据大小: %1 bytes").arg(dracoData.size());
        
        // 创建解码器缓冲区
        draco::DecoderBuffer buffer;
        buffer.Init(dracoData.constData(), dracoData.size());
        
        // 解码
        draco::Decoder decoder;
        auto statusWithCloud = decoder.DecodePointCloudFromBuffer(&buffer);
        
        if (!statusWithCloud.ok()) {
            QString error = QString("Draco解码失败: %1").arg(statusWithCloud.status().error_msg());//.c_str());
            logError("Draco解压缩", error);
            return result;
        }
        
        // 直接提取到QML格式
        result = extractToQmlFormat(*statusWithCloud.value());
        
        qint64 processingTime = m_timer.elapsed();
        qCDebug(dracoLog) << QString("Draco解压缩到QML格式完成，点数: %1, 耗时: %2ms")
                                 .arg(result.pointCount).arg(processingTime);
        
        // 更新统计
        DracoCompressionResult stats;
        stats.success = true;
        stats.compressedSize = dracoData.size();
        stats.processingTime = processingTime;
        updateStats(stats, false);
        
        emit decompressionCompleted(true);
        
    } catch (const std::exception& e) {
        QString error = QString("Draco解压缩异常: %1").arg(e.what());
        logError("Draco解压缩", error);
        emit decompressionCompleted(false);
    }
#else
    qCWarning(dracoLog) << "Draco支持未编译";
#endif
    
    return result;
}

#ifdef ENABLE_DRACO
std::unique_ptr<draco::PointCloud> DracoHelper::createDracoPointCloud(
    const CrossPlatformPointCloud& cloud)
{
    auto dracoCloud = std::make_unique<draco::PointCloud>();
    
    const int numPoints = cloud.size();
    dracoCloud->set_num_points(numPoints);
    
    // 添加位置属性
    draco::GeometryAttribute posAttr;
    posAttr.Init(draco::GeometryAttribute::POSITION, nullptr, 3, draco::DT_FLOAT32, false, sizeof(float) * 3, 0);
    const int posAttId = dracoCloud->AddAttribute(posAttr, true, numPoints);
    
    // 添加颜色属性
    draco::GeometryAttribute colorAttr;
    colorAttr.Init(draco::GeometryAttribute::COLOR, nullptr, 3, draco::DT_UINT8, false, sizeof(uint8_t) * 3, 0);
    const int colorAttId = dracoCloud->AddAttribute(colorAttr, true, numPoints);
    
    // 填充数据
    for (int i = 0; i < numPoints; ++i) {
        const auto& point = cloud.points[i];
        
        // 设置位置
        const float pos[3] = {point.x, point.y, point.z};
        dracoCloud->attribute(posAttId)->SetAttributeValue(draco::AttributeValueIndex(i), pos);
        
        // 设置颜色
        const uint8_t color[3] = {point.r, point.g, point.b};
        dracoCloud->attribute(colorAttId)->SetAttributeValue(draco::AttributeValueIndex(i), color);
    }
    
    return dracoCloud;
}

CrossPlatformPointCloud DracoHelper::extractFromDracoPointCloud(
    const draco::PointCloud& dracoCloud)
{
    CrossPlatformPointCloud result;
    
    const int numPoints = dracoCloud.num_points();
    result.points.reserve(numPoints);
    
    // 获取属性
    const draco::PointAttribute* posAttr = dracoCloud.GetNamedAttribute(draco::GeometryAttribute::POSITION);
    const draco::PointAttribute* colorAttr = dracoCloud.GetNamedAttribute(draco::GeometryAttribute::COLOR);
    
    for (int i = 0; i < numPoints; ++i) {
        CrossPlatformPointCloud::Point point;
        
        // 获取位置
        if (posAttr) {
            draco::Vector3f pos;
            posAttr->ConvertValue<float, 3>(draco::AttributeValueIndex(i), &pos[0]);
            point.x = pos[0];
            point.y = pos[1];
            point.z = pos[2];
        }
        
        // 获取颜色
        if (colorAttr) {
            draco::VectorD<uint8_t,3> color;
            colorAttr->ConvertValue<uint8_t, 3>(draco::AttributeValueIndex(i), &color[0]);
            point.r = color[0];
            point.g = color[1];
            point.b = color[2];
        }
        
        result.points.append(point);
    }
    
    result.timestamp = CrossPlatformTimestamp::now();
    return result;
}

QmlPointCloudData DracoHelper::extractToQmlFormat(const draco::PointCloud& dracoCloud)
{
    QmlPointCloudData result;
    
    const int numPoints = dracoCloud.num_points();
    result.reserve(numPoints);
    
    // 获取属性
    const draco::PointAttribute* posAttr = dracoCloud.GetNamedAttribute(draco::GeometryAttribute::POSITION);
    const draco::PointAttribute* colorAttr = dracoCloud.GetNamedAttribute(draco::GeometryAttribute::COLOR);
    
    for (int i = 0; i < numPoints; ++i) {
        float x = 0, y = 0, z = 0;
        float r = 1.0f, g = 1.0f, b = 1.0f;
        
        // 获取位置
        if (posAttr) {
            draco::Vector3f pos;
            posAttr->ConvertValue<float, 3>(draco::AttributeValueIndex(i), &pos[0]);
            x = pos[0];
            y = pos[1];
            z = pos[2];
        }
        
        // 获取颜色并归一化到0-1范围
        if (colorAttr) {
            draco::VectorD<uint8_t,3> color;
            colorAttr->ConvertValue<uint8_t, 3>(draco::AttributeValueIndex(i), &color[0]);
            r = color[0] / 255.0f;
            g = color[1] / 255.0f;
            b = color[2] / 255.0f;
        }
        
        result.addPoint(x, y, z, r, g, b);
    }
    
    return result;
}

draco::EncoderOptions DracoHelper::getDracoEncoderOptions(CompressionLevel level)
{
    draco::EncoderOptions options=draco::EncoderOptions::CreateDefaultOptions();
    
    switch (level) {
    case HIGH_QUALITY:
        options.SetGlobalInt("quantization_position", 14);
        options.SetGlobalInt("quantization_color", 10);
        break;
    case MEDIUM_QUALITY:
        options.SetGlobalInt("quantization_position", 11);
        options.SetGlobalInt("quantization_color", 8);
        break;
    case LOW_QUALITY:
        options.SetGlobalInt("quantization_position", 9);
        options.SetGlobalInt("quantization_color", 6);
        break;
    case MOBILE_OPTIMIZED:
        options.SetGlobalInt("quantization_position", 8);
        options.SetGlobalInt("quantization_color", 5);
        break;
    }
    
    return options;
}

int DracoHelper::getQuantizationBits(CompressionLevel level, bool isPosition)
{
    if (isPosition) {
        switch (level) {
        case HIGH_QUALITY: return 14;
        case MEDIUM_QUALITY: return 11;
        case LOW_QUALITY: return 9;
        case MOBILE_OPTIMIZED: return 8;
        }
    } else {
        switch (level) {
        case HIGH_QUALITY: return 10;
        case MEDIUM_QUALITY: return 8;
        case LOW_QUALITY: return 6;
        case MOBILE_OPTIMIZED: return 5;
        }
    }
    return 11; // 默认值
}
#endif

void DracoHelper::setQuantizationBits(int positionBits, int colorBits)
{
    m_positionQuantizationBits = qBound(8, positionBits, 16);
    m_colorQuantizationBits = qBound(4, colorBits, 12);
    
    qCDebug(dracoLog) << QString("设置量化位数 - 位置: %1, 颜色: %2")
                             .arg(m_positionQuantizationBits).arg(m_colorQuantizationBits);
}

void DracoHelper::setCompressionSpeed(int speed)
{
    m_compressionSpeed = qBound(0, speed, 10);
    qCDebug(dracoLog) << "设置压缩速度:" << m_compressionSpeed;
}

QmlPointCloudData DracoHelper::convertToQmlFormat(const CrossPlatformPointCloud& cloud)
{
    QmlPointCloudData result;
    result.reserve(cloud.size());
    
    for (const auto& point : cloud.points) {
        result.addPoint(point.x, point.y, point.z,
                       point.r / 255.0f, point.g / 255.0f, point.b / 255.0f);
    }
    
    return result;
}

CrossPlatformPointCloud DracoHelper::convertFromQmlFormat(const QmlPointCloudData& qmlData)
{
    CrossPlatformPointCloud result;
    result.points.reserve(qmlData.pointCount);
    
    for (int i = 0; i < qmlData.pointCount; ++i) {
        CrossPlatformPointCloud::Point point;
        point.x = qmlData.positions[i * 3].toFloat();
        point.y = qmlData.positions[i * 3 + 1].toFloat();
        point.z = qmlData.positions[i * 3 + 2].toFloat();
        point.r = static_cast<quint8>(qmlData.colors[i * 3].toFloat() * 255);
        point.g = static_cast<quint8>(qmlData.colors[i * 3 + 1].toFloat() * 255);
        point.b = static_cast<quint8>(qmlData.colors[i * 3 + 2].toFloat() * 255);
        result.points.append(point);
    }
    
    result.timestamp = CrossPlatformTimestamp::now();
    return result;
}

void DracoHelper::updateStats(const DracoCompressionResult& result, bool isCompression)
{
    QMutexLocker locker(&m_statsMutex);
    
    if (isCompression) {
        m_stats.totalCompressions++;
        m_stats.totalBytesOriginal += result.originalSize;
        m_stats.totalBytesCompressed += result.compressedSize;
        
        m_stats.averageCompressionRatio = m_stats.totalBytesOriginal > 0 ?
            (float)m_stats.totalBytesOriginal / m_stats.totalBytesCompressed : 0.0f;
        
        m_stats.averageCompressionTime =
            (m_stats.averageCompressionTime * (m_stats.totalCompressions - 1) + result.processingTime)
            / m_stats.totalCompressions;
    } else {
        m_stats.totalDecompressions++;
        m_stats.averageDecompressionTime =
            (m_stats.averageDecompressionTime * (m_stats.totalDecompressions - 1) + result.processingTime)
            / m_stats.totalDecompressions;
    }
    
    if (!result.success) {
        m_stats.totalErrors++;
    }
}

DracoHelper::DracoStats DracoHelper::getStats() const
{
    QMutexLocker locker(&m_statsMutex);
    return m_stats;
}

void DracoHelper::resetStats()
{
    QMutexLocker locker(&m_statsMutex);
    m_stats = DracoStats();
}

void DracoHelper::logError(const QString& operation, const QString& error)
{
    QString logMsg = QString("[%1] %2: %3")
        .arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"))
        .arg(operation)
        .arg(error);
    
    qCCritical(dracoLog) << logMsg;
    emit errorOccurred(error);
}

