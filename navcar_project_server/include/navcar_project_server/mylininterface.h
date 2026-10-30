#ifndef MYLININTERFACE_H
#define MYLININTERFACE_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>
#include <QQueue>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <QDebug>

class MyLinInterface : public QObject
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
    Q_ENUM(LinState)

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
    Q_ENUM(LinError)

    explicit MyLinInterface(QObject *parent = nullptr);
    ~MyLinInterface();

    // 连接管理
    bool connectToPort(const QString& portName, qint32 baudRate = 19200);
    void disconnectFromPort();
    bool isConnected() const;

    // 获取可用串口列表
    QStringList getAvailablePorts() const;

    // 状态查询
    LinState getState() const;
    LinError getLastError() const;
    QString getErrorString() const;

    // 统计信息
    quint32 getFramesSent() const { return m_framesSent; }
    quint32 getFramesReceived() const { return m_framesReceived; }
    quint32 getErrorCount() const { return m_errorCount; }
    void resetStatistics();

    // 发送数据
    bool sendFrame(quint8 id, QByteArray data);

    // 请求数据 (作为主节点请求从节点数据)
    bool requestFrame(quint8 id, int expectedDataLength = 8);

    // 设置超时时间
    void setTimeout(int timeout) { m_timeout = timeout; }
    int getTimeout() const { return m_timeout; }

public slots:
    // 公共槽函数，允许外部调用
    void clearReceiveBuffer() {
        QMutexLocker locker(&m_mutex);
        m_receiveBuffer.clear();
    }

signals:
    void errorOccurred(LinError error, const QString& errorString);
    void stateChanged(LinState state);
    void dataReceived(const QByteArray& fullPacket, const QByteArray& payload);
    void frameReceivedSignal(quint8,QByteArray);
    void responseTimeout();

private:
    // 初始化和配置
    bool initLINSetting(qint32 baudRate = 19200);

    // PID和校验和计算
    quint8 calPID(quint8 id);
    quint8 calculateChecksum(quint8 pid, const QByteArray& data);
    quint8 calculateChecksum_classic(quint8 pid, const QByteArray& data);
    bool verifyChecksum(quint8 pid, const QByteArray& data, quint8 receivedChecksum);

    // 数据处理
    void processReceivedData();
    int findPacketHeader(quint8 byte1, quint8 byte2) const;
    bool validatePacket(const QByteArray& packet) const;
    void handleValidPacket(const QByteArray& packet);

    // 状态管理
    void setState(LinState state);
    void setError(LinError error, const QString& errorString);

    // 成员变量
    QSerialPort* m_serialPort;
    QTimer* m_timeoutTimer;
    mutable QMutex m_mutex;  // 添加mutable以支持const函数中的锁定

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


    static const int DEFAULT_TIMEOUT = 1000;  // 增加默认超时时间到1秒

private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);
    void handleTimeout();
};

#endif // MYLININTERFACE_H
