#ifndef LININTERFACE_H
#define LININTERFACE_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>
#include <QQueue>
#include <QMutex>
#include <QThread>
#include <QDebug>

// LIN报文结构
struct LinFrame {
    quint8 id;           // 帧ID (0-63)
    QByteArray data;     // 数据字段 (0-8字节)
    quint8 checksum;     // 校验和
    bool isValid;        // 帧是否有效

    LinFrame() : id(0), checksum(0), isValid(false) {}
    LinFrame(quint8 frameId, const QByteArray& frameData)
        : id(frameId), data(frameData), checksum(0), isValid(true) {
        calculateChecksum();
    }
/*
    // 计算校验和 (经典校验和)
    void calculateChecksum() {
        quint16 sum = 0;  // 经典校验和初始化为0（不包含ID）
        for (int i = 0; i < data.size(); ++i) {
            sum += static_cast<quint8>(data[i]);
            if (sum > 0xFF) {
                sum = (sum & 0xFF) + 1;  // 进位回卷（Carry Wrap-around）
            }
        }
        checksum = (~sum) & 0xFF;  // 取反后取低8位
    }

    // 验证校验和
    bool verifyChecksum() const {
        quint16 sum = 0;  // 经典校验和初始化为0（不包含ID）
        for (int i = 0; i < data.size(); ++i) {
            sum += static_cast<quint8>(data[i]);
            if (sum > 0xFF) {
                sum = (sum & 0xFF) + 1;
            }
        }
        sum += checksum;          // 添加接收到的校验和
        return ((sum & 0xFF) == 0xFF);  // 结果应为0xFF（模256和为0）
    }*/
    // 计算PID（奇偶校验位）
    quint8 calculatePID(quint8 id) const {
        quint8 pid = id & 0x3F;  // 取低6位
        // 计算奇偶校验位
        quint8 p0 = (pid ^ (pid >> 1) ^ (pid >> 2) ^ (pid >> 4)) & 0x01;
        quint8 p1 = ~((pid >> 1) ^ (pid >> 3) ^ (pid >> 4) ^ (pid >> 5)) & 0x01;
        pid |= (p0 << 6) | (p1 << 7);
        return pid;
    }

    // 计算校验和 (增强型校验和)
    void calculateChecksum() {
        quint16 sum = calculatePID(id);  // 增强型校验和包含ID(V1.0修改，ID替换成PID）
        for (int i = 0; i < data.size(); ++i) {
            sum += static_cast<quint8>(data[i]);
            if (sum > 255) {
                sum = (sum & 0xFF) + 1;  // 进位加1
            }
        }
        checksum = (~sum) & 0xFF;  // 取反
    }

    // 验证校验和
    bool verifyChecksum() const {
        quint16 sum = calculatePID(id);  // 增强型校验和包含ID(V1.0修改，ID替换成PID）
        for (int i = 0; i < data.size(); ++i) {
            sum += static_cast<quint8>(data[i]);
            if (sum > 255) {
                sum = (sum & 0xFF) + 1;
            }
        }
        sum += checksum;
        return ((sum & 0xFF) == 0xFF);
    }
};

class LinInterface : public QObject
{
    Q_OBJECT

public:
    enum LinState {
        Disconnected,
        Connected,
        Transmitting,
        Receiving,
        Error
    };

    enum LinError {
        NoError,
        PortOpenError,
        PortNotFound,
        WriteError,
        ReadError,
        ChecksumError,
        TimeoutError,
        FrameError
    };

    explicit LinInterface(QObject *parent = nullptr);
    ~LinInterface();

    // 连接管理
    bool connectToPort(const QString& portName, qint32 baudRate = 19200);
    void disconnectFromPort();
    bool isConnected() const;

    // 获取可用串口列表
    QStringList getAvailablePorts() const;

    // 发送数据
    bool sendFrame(quint8 id, const QByteArray& data);
    bool sendFrame(const LinFrame& frame);

    // 请求数据 (作为主节点请求从节点数据)
    bool requestFrame(quint8 id, int expectedDataLength = 8);

    // 设置/获取参数
    void setBaudRate(qint32 baudRate);
    qint32 getBaudRate() const;
    void setTimeout(int ms);
    int getTimeout() const;

    // 状态查询
    LinState getState() const;
    LinError getLastError() const;
    QString getErrorString() const;

    // 统计信息
    quint32 getFramesSent() const { return m_framesSent; }
    quint32 getFramesReceived() const { return m_framesReceived; }
    quint32 getErrorCount() const { return m_errorCount; }
    void resetStatistics();

signals:
    void frameReceived(const LinFrame& frame);
    void frameSent(quint8 id);
    void errorOccurred(LinError error, const QString& errorString);
    void stateChanged(LinState state);
    void frameReceivedSignal(quint8 id,QByteArray recdata);

private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);
    void handleTimeout();

private:
    // 内部方法
    void setState(LinState state);
    void setError(LinError error, const QString& errorString);
    bool writeBreakField();
    bool writeSyncField();
    bool writeIdentifierField(quint8 id);
    bool writeDataField(const QByteArray& data);
    bool writeChecksumField(quint8 checksum);
    void processReceivedData();
    void initLin1();
    LinFrame parseFrame(const QByteArray& data);
    QByteArray createFrameBytes(const LinFrame& frame);

    quint8 calculatePID(quint8 id) const {
        quint8 pid = id & 0x3F;  // 取低6位
        // 计算奇偶校验位
        quint8 p0 = (pid ^ (pid >> 1) ^ (pid >> 2) ^ (pid >> 4)) & 0x01;
        quint8 p1 = ~((pid >> 1) ^ (pid >> 3) ^ (pid >> 4) ^ (pid >> 5)) & 0x01;
        pid |= (p0 << 6) | (p1 << 7);
        return pid;
    }

    // 成员变量
    QSerialPort* m_serialPort;
    QTimer* m_timeoutTimer;
    QMutex m_mutex;

    LinState m_state;
    LinError m_lastError;
    QString m_errorString;

    qint32 m_baudRate;
    int m_timeout;

    QByteArray m_receiveBuffer;
    bool m_waitingForResponse;
    quint8 m_expectedFrameId;
    int m_expectedDataLength;

    // 统计
    quint32 m_framesSent;
    quint32 m_framesReceived;
    quint32 m_errorCount;

    // LIN协议常量
    static const quint8 LIN_SYNC_BYTE = 0x55;
    static const quint8 LIN_BREAK_DURATION = 13;  // 13位时间
    static const int DEFAULT_TIMEOUT = 50;       // 100ms
};

#endif // LININTERFACE_H
