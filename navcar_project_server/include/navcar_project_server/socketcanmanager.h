#ifndef SOCKETCANMANAGER_H
#define SOCKETCANMANAGER_H

#include <QObject>
#include <QTimer>
#include <QThread>
#include <QMutex>
#include <QQueue>
#include <QDebug>
#include <QSocketNotifier>
#include <QStringList>
#include<QDateTime>

// Linux Socket CAN 头文件
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <unistd.h>
#include <fcntl.h>

// CAN消息结构体
struct CanMessage {
    quint32 id;              // CAN ID
    quint8 data[8];          // 数据
    quint8 dataLen;          // 数据长度 (0-8)
    bool isExtended;         // 扩展帧标志
    bool isRemote;           // 远程帧标志
    bool isError;            // 错误帧标志
    quint64 timestamp;       // 时间戳 (毫秒)

    CanMessage() {
        id = 0;
        dataLen = 0;
        isExtended = false;
        isRemote = false;
        isError = false;
        timestamp = 0;
        memset(data, 0, sizeof(data));
    }
};

// CAN过滤器
struct CanFilter {
    quint32 id;              // CAN ID
    quint32 mask;            // 掩码
    bool isExtended;         // 扩展帧

    CanFilter(quint32 canId = 0, quint32 canMask = 0, bool ext = false)
        : id(canId), mask(canMask), isExtended(ext) {}
};

class SocketCanManager : public QObject
{
    Q_OBJECT

public:
    enum CanSpeed {
        SPEED_10K = 10000,
        SPEED_20K = 20000,
        SPEED_50K = 50000,
        SPEED_100K = 100000,
        SPEED_125K = 125000,
        SPEED_250K = 250000,
        SPEED_500K = 500000,
        SPEED_800K = 800000,
        SPEED_1000K = 1000000
    };

    enum CanState {
        STATE_CLOSED,
        STATE_OPENED,
        STATE_ERROR_ACTIVE,
        STATE_ERROR_WARNING,
        STATE_ERROR_PASSIVE,
        STATE_BUS_OFF,
        STATE_STOPPED,
        STATE_SLEEPING
    };

    explicit SocketCanManager(QObject *parent = nullptr);
    ~SocketCanManager();

    // 接口管理
    bool openInterface(const QString &interfaceName);
    bool closeInterface();
    bool isInterfaceOpen() const { return m_socketFd >= 0; }
    QString getCurrentInterface() const { return m_interfaceName; }

    // 静态工具方法
    static QStringList getAvailableInterfaces();
    static bool configureInterface(const QString &interface, CanSpeed speed);
    static bool setInterfaceUp(const QString &interface);
    static bool setInterfaceDown(const QString &interface);
    static CanState getInterfaceState(const QString &interface);

    // 数据收发
    bool sendMessage(const CanMessage &message);
    bool sendMessages(const QList<CanMessage> &messages);
    QList<CanMessage> receiveMessages(int maxCount = 100);

    // 过滤器管理
    bool setFilters(const QList<CanFilter> &filters);
    bool addFilter(const CanFilter &filter);
    bool removeFilters();
    QList<CanFilter> getFilters() const { return m_filters; }

    // 配置选项
    bool setLoopback(bool enable);
    bool setReceiveOwnMessages(bool enable);
    bool setErrorFilter(quint32 errorMask = CAN_ERR_MASK);
    bool setNonBlocking(bool enable);

    // 状态查询
    QString getLastError() const { return m_lastError; }

    // 统计信息
    struct Statistics {
        quint64 txCount;
        quint64 rxCount;
        quint64 txErrorCount;
        quint64 rxErrorCount;
        quint64 errorFrameCount;
        quint64 droppedCount;

        Statistics() : txCount(0), rxCount(0), txErrorCount(0),
            rxErrorCount(0), errorFrameCount(0), droppedCount(0) {}
    };

    Statistics getStatistics() const;
    void clearStatistics();

    // 错误处理
    struct CanError {
        quint32 errorClass;
        QString description;
        QDateTime timestamp;

        CanError(quint32 cls = 0, const QString &desc = "",
                 const QDateTime &time = QDateTime::currentDateTime())
            : errorClass(cls), description(desc), timestamp(time) {}
    };

    QList<CanError> getRecentErrors() const { return m_recentErrors; }
    void clearErrors() { m_recentErrors.clear(); }

signals:
    void messageReceived(CanMessage message);
    void messagesReceived(const QList<CanMessage> &messages);
    void errorFrameReceived(const CanMessage &errorFrame);
    void errorOccurred(const QString &error);
    void interfaceStateChanged(CanState state);
    void statisticsChanged(const Statistics &stats);

private slots:
    void onSocketReadyRead();

private:
    // 内部方法
    void setLastError(const QString &error);
    struct can_frame convertToCanFrame(const CanMessage &message);
    CanMessage convertFromCanFrame(const struct can_frame &frame);
    void processErrorFrame(const struct can_frame &frame);
    QString getErrorDescription(quint32 errorClass);
    void updateStatistics();

    // Socket相关
    int m_socketFd;
    QString m_interfaceName;
    QSocketNotifier *m_socketNotifier;

    // 配置
    QList<CanFilter> m_filters;
    bool m_loopbackEnabled;
    bool m_receiveOwnMessages;
    bool m_nonBlocking;
    quint32 m_errorMask;

    // 状态
    QString m_lastError;
    CanState m_currentState;

    // 统计信息
    mutable QMutex m_statsMutex;
    Statistics m_statistics;

    // 错误记录
    mutable QMutex m_errorMutex;
    QList<CanError> m_recentErrors;
    static const int MAX_ERROR_HISTORY = 100;

    // 接收缓冲
    QQueue<CanMessage> m_receiveBuffer;
    static const int MAX_BUFFER_SIZE = 10000;
};

// 辅助类：CAN接口配置工具
class CanInterfaceConfig
{
public:
    struct InterfaceInfo {
        QString name;
        bool isUp;
        SocketCanManager::CanSpeed speed;
        QString driver;
        quint32 txQueueLen;
        quint64 rxPackets;
        quint64 txPackets;
        quint64 rxErrors;
        quint64 txErrors;
    };

    static QList<InterfaceInfo> getAllInterfaceInfo();
    static InterfaceInfo getInterfaceInfo(const QString &interface);
    static bool createVirtualInterface(const QString &name);
    static bool removeVirtualInterface(const QString &name);
    static bool setTxQueueLength(const QString &interface, quint32 length);

    // CAN-utils包装方法
    static bool canDump(const QString &interface, int duration = 10);
    static bool canSend(const QString &interface, const QString &canFrame);
    static QString getCanUtilsVersion();
};

#endif // SOCKETCANMANAGER_H
