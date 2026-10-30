// networkprotocol.h - 修正版本，补充缺失的枚举
#ifndef NETWORKPROTOCOL_H
#define NETWORKPROTOCOL_H

#include <QtGlobal>
#include<QString>
#include<QTime>
#include<QPoint>
#include <QVariantList>
#include <QDataStream>

//=========网络协议定义类==========
// 网络协议类型
enum class NetworkProtocol {
    TCP = 0,
    UDP = 1
};

// 连接状态
enum class ConnectionState {
    Disconnected = 0,
    Connecting = 1,
    Connected = 2,
    Listening = 3,
    Error = 4
};

// 数据流类型
enum StreamType {
    STREAM_POINTCLOUD = 0x01,
    STREAM_IMAGE = 0x02,
    STREAM_COMBINED = 0x03,
    STREAM_STATUS = 0x04,
    STREAM_ALL = 0xFF
};

// 流动作
enum StreamAction {
    ACTION_STOP = 0,
    ACTION_START = 1,
    ACTION_PAUSE = 2,
    ACTION_RESUME = 3
};

// 网络数据类型 - 新增，用于区分不同的压缩数据类型
enum NetworkDataType {
    POINTCLOUD_RAW = 0,
    POINTCLOUD_COMPRESSED = 1,
    POINTCLOUD_DRACO = 2,
    IMAGE_RAW = 10,
    IMAGE_JPEG = 11,
    IMAGE_H264 = 12,
    STATUS_DATA = 20,
    HEARTBEAT_DATA = 21,
    CONTROL_DATA = 30
};

struct NetworkStatistics {
    qint64 totalBytesSent = 0;
    qint64 totalBytesReceived = 0;
    qint64 totalFramesSent = 0;
    qint64 totalConnections = 0;
    qint64 activeConnections = 0;
    qint64 totalErrors = 0;
    qint64 uptimeSeconds = 0;
    float averageBandwidthUsage = 0.0f;
    QDateTime startTime;
    qint64 h264FramesSent = 0;
    qint64 h264KeyFramesSent = 0;
    float h264AverageCompressionRatio = 0.0f;
    int h264PacketLossRecoveries = 0;
};


// 跨平台时间戳
struct CrossPlatformTimestamp {
    qint64 seconds;
    qint32 microseconds;

    CrossPlatformTimestamp() : seconds(0), microseconds(0) {}

    static CrossPlatformTimestamp now() {
        QDateTime dt = QDateTime::currentDateTime();
        CrossPlatformTimestamp ts;
        ts.seconds = dt.toSecsSinceEpoch();
        ts.microseconds = (dt.toMSecsSinceEpoch() % 1000) * 1000;
        return ts;
    }

    QDateTime toDateTime() const {
        return QDateTime::fromSecsSinceEpoch(seconds).addMSecs(microseconds / 1000);
    }
};

// 跨平台点云数据结构
struct CrossPlatformPointCloud {
    struct Point {
        float x, y, z;
        quint8 r, g, b;

        Point() : x(0), y(0), z(0), r(255), g(255), b(255) {}
        Point(float x_, float y_, float z_, quint8 r_=255, quint8 g_=255, quint8 b_=255)
            : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_) {}
    };

    QList<Point> points;
    CrossPlatformTimestamp timestamp;
    QString frameId;

    int size() const { return points.size(); }
    bool isEmpty() const { return points.isEmpty(); }
    void clear() { points.clear(); }

    QByteArray toByteArray() const {
        QByteArray dataArray;
        QDataStream stream(&dataArray, QIODevice::WriteOnly);
        stream.setVersion(QDataStream::Qt_5_12); // 选择最低版本作为基准
        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        stream.setByteOrder(QDataStream::LittleEndian);

        // 写入时间戳
        stream << timestamp.seconds << timestamp.microseconds;

        // 写入坐标系ID
        stream << frameId;

        // 写入点数
        quint32 pointCount = points.size();
        stream << pointCount;

        // 写入每个点的数据
        for (const Point& p : points) {
            stream << p.x << p.y << p.z << p.r << p.g << p.b;
        }

        return dataArray;
    }

    static CrossPlatformPointCloud fromByteArray(const QByteArray& data) {
        CrossPlatformPointCloud cloud;
        QDataStream stream(data);
        stream.setVersion(QDataStream::Qt_5_12); // 选择最低版本作为基准
        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        stream.setByteOrder(QDataStream::LittleEndian);

        // 读取时间戳
        stream >> cloud.timestamp.seconds >> cloud.timestamp.microseconds;

        // 读取坐标系ID
        stream >> cloud.frameId;

        // 读取点数
        quint32 pointCount;
        stream >> pointCount;
        cloud.points.reserve(pointCount);

        // 读取每个点的数据
        for (quint32 i = 0; i < pointCount; ++i) {
            Point p;
            stream >> p.x >> p.y >> p.z >> p.r >> p.g >> p.b;
            cloud.points.append(p);
        }

        return cloud;
    }
};

// QML显示用的点云数据结构
struct QmlPointCloudData {
    QVariantList positions;  // [x1,y1,z1, x2,y2,z2, ...]
    QVariantList colors;     // [r1,g1,b1, r2,g2,b2, ...] (0-1范围)
    int pointCount;

    QmlPointCloudData() : pointCount(0) {}

    void clear() {
        positions.clear();
        colors.clear();
        pointCount = 0;
    }

    void reserve(int count) {
        positions.reserve(count * 3);
        colors.reserve(count * 3);
    }

    void addPoint(float x, float y, float z, float r, float g, float b) {
        positions << x << y << z;
        colors << r << g << b;
        pointCount++;
    }
};

// 压缩级别和编码器
enum CompressionLevel {
    HIGH_QUALITY = 0,
    MEDIUM_QUALITY = 1,
    LOW_QUALITY = 2,
    MOBILE_OPTIMIZED = 3
};

enum VideoCodec {
    CODEC_JPEG = 0,
    CODEC_H264 = 1
};

// 数据包头结构
#pragma pack(push, 1)
struct NetworkPacketHeader {
    quint8 version;          // 协议版本
    quint8 type;             // 数据类型
    quint16 flags;           // 标志位
    quint32 dataSize;        // 数据大小
    quint32 sequenceNumber;  // 序列号
    quint64 timestamp;       // 时间戳

    NetworkPacketHeader()
        : version(1), type(0), flags(0), dataSize(0), sequenceNumber(0), timestamp(0) {}
};

// 流控制命令
struct NetDataStreamControl {
    NetworkPacketHeader header;
    quint8 streamType;
    quint8 action;
    quint8 compressionLevel;
    quint8 videoCodec;
    quint16 frameRate;
    quint32 duration;
    quint16 imageWidth;
    quint16 imageHeight;
    float pointCloudRange;

    NetDataStreamControl() {
        header.type = 's';
        header.dataSize = sizeof(*this);
        streamType = 0;
        action = 0;
        compressionLevel = MEDIUM_QUALITY;
        videoCodec = CODEC_JPEG;
        frameRate = 10;
        duration = 0;
        imageWidth = 640;
        imageHeight = 480;
        pointCloudRange = 50.0f;
    }
};

// 心跳数据包
struct NetworkHeartbeat {
    NetworkPacketHeader header;
    quint32 sequenceNumber;
    CrossPlatformTimestamp timestamp;

    NetworkHeartbeat() {
        header.type = 'h';
        header.dataSize = sizeof(*this);
        sequenceNumber = 0;
        timestamp = CrossPlatformTimestamp::now();
    }
};

// 服务器状态
struct NetworkStatus {
    NetworkPacketHeader header;
    quint32 connectedClients;
    float cpuUsage;
    float memoryUsage;
    float bandwidthUsage;
    quint32 totalFramesSent;
    CrossPlatformTimestamp timestamp;

    NetworkStatus() {
        header.type = 't';
        header.dataSize = sizeof(*this);
        connectedClients = 0;
        cpuUsage = 0.0f;
        memoryUsage = 0.0f;
        bandwidthUsage = 0.0f;
        totalFramesSent = 0;
        timestamp = CrossPlatformTimestamp::now();
    }
};

// 点云数据包
struct NetDataPointCloud {
    NetworkPacketHeader header;
    quint32 pointCount;
    quint32 originalSize;
    quint16 compressionLevel;
    float compressionRatio;
    CrossPlatformTimestamp timestamp;
    uchar data[];

    static NetDataPointCloud* SetData(quint32 pointCount, quint32 originalSize,
                                      quint16 level, float ratio,
                                      const QByteArray& compressedData) {
        int totalSize = sizeof(NetDataPointCloud) + compressedData.size();
        NetDataPointCloud* packet = (NetDataPointCloud*)malloc(totalSize);

        packet->header.type = 'p';
        packet->header.dataSize = totalSize;
        packet->pointCount = pointCount;
        packet->originalSize = originalSize;
        packet->compressionLevel = level;
        packet->compressionRatio = ratio;
        packet->timestamp = CrossPlatformTimestamp::now();

        memcpy(packet->data, compressedData.constData(), compressedData.size());
        return packet;
    }

    static void FreeData(NetDataPointCloud* data) {
        if (data) {
            free(data);
        }
    }
};

// 扩展网络协议以支持Draco格式
struct NetDataDracoPointCloud {
    NetworkPacketHeader header;
    quint32 originalPointCount;     // 原始点数
    quint32 originalSize;           // 原始数据大小
    quint16 compressionLevel;       // 压缩级别
    quint16 quantizationBits;       // 量化位数
    float compressionRatio;         // 压缩比
    CrossPlatformTimestamp timestamp;
    uchar dracoData[];              // Draco压缩数据

    static NetDataDracoPointCloud* createPacket(
        quint32 pointCount, quint32 originalSize,
        quint16 level, quint16 quantBits, float ratio,
        const QByteArray& dracoData) {

        int totalSize = sizeof(NetDataDracoPointCloud) + dracoData.size();
        NetDataDracoPointCloud* packet = (NetDataDracoPointCloud*)malloc(totalSize);

        packet->header.type = 'd';  // 'd' for draco
        packet->header.version = 1;
        packet->header.dataSize = totalSize;
        packet->originalPointCount = pointCount;
        packet->originalSize = originalSize;
        packet->compressionLevel = level;
        packet->quantizationBits = quantBits;
        packet->compressionRatio = ratio;
        packet->timestamp = CrossPlatformTimestamp::now();

        memcpy(packet->dracoData, dracoData.constData(), dracoData.size());
        return packet;
    }

    static void freePacket(NetDataDracoPointCloud* packet) {
        if (packet) {
            free(packet);
        }
    }

    QByteArray getDracoData() const {
        int dracoSize = header.dataSize - sizeof(NetDataDracoPointCloud);
        return QByteArray((const char*)dracoData, dracoSize);
    }
};

// 图像数据包
struct NetDataImageEx {
    NetworkPacketHeader header;
    quint16 height;
    quint16 width;
    quint8 videoCodec;
    quint8 quality;
    float compressionRatio;
    quint32 frameId;
    CrossPlatformTimestamp timestamp;
    uchar data[];

    static NetDataImageEx* SetData(quint16 height, quint16 width,
                                   quint8 codec, quint8 quality,
                                   const QByteArray& imageData,
                                   quint32 frameId = 0) {
        int totalSize = sizeof(NetDataImageEx) + imageData.size();
        NetDataImageEx* packet = (NetDataImageEx*)malloc(totalSize);

        packet->header.type = 'i';
        packet->header.dataSize = totalSize;
        packet->height = height;
        packet->width = width;
        packet->videoCodec = codec;
        packet->quality = quality;
        packet->frameId = frameId;
        packet->timestamp = CrossPlatformTimestamp::now();

        memcpy(packet->data, imageData.constData(), imageData.size());
        return packet;
    }

    static void FreeData(NetDataImageEx* data) {
        if (data) {
            free(data);
        }
    }
};

//GPS数据包
struct NetDataGPSData{
    NetworkPacketHeader header;
    quint16 week;                                                 // gps 周
    double second;                                              // gps 周内秒
    double latitude;                                            // 纬度(deg)
    double longitude;                                           // 经度(deg)
    float altitude;                                            // 海拔(m)
    double heading2;                                           //车体中心航向角
    float hdop;                                                // 水平精度因子
    float pdop;                                                // 位置精度因子
    float vdop;                                                // 垂直精度因子
    float tdop;                                                // 种差精度因子
    float gdop;                                                // 几何精度因子
    quint8 status;                                             //gnss状态
};

//路径点轨迹数据包
struct NetDataTrajData{
    NetworkPacketHeader header;
    int pointNum; //总点个数
    QList<QPoint> pointData;
};

#pragma pack(pop)

#endif // NETWORKPROTOCOL_H
