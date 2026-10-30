// qt_version_compat.h - Qt版本兼容性处理 (修复宏参数问题)
#ifndef QT_VERSION_COMPAT_H
#define QT_VERSION_COMPAT_H

#include <QtGlobal>
#include <QAbstractSocket>

// ========== Qt 5.12/5.15/6.x 兼容性宏定义 ==========

// 1. QAbstractSocket::error() vs errorOccurred() 信号兼容
#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
// Qt 5.12 及更早版本使用 error() 信号
#define QT_SOCKET_ERROR_SIGNAL error
#define QT_SOCKET_ERROR_CONNECT(socket, slot) \
connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error), slot)
#else
// Qt 5.15+ 使用 errorOccurred() 信号
#define QT_SOCKET_ERROR_SIGNAL errorOccurred
#define QT_SOCKET_ERROR_CONNECT(socket, slot) \
connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::errorOccurred), slot)
#endif

// 2. QTcpSocket/QUdpSocket error 信号兼容 - 修复：支持3个参数
#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
#define QT_TCP_ERROR_CONNECT(socket, receiver, slot) \
    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error), receiver, slot)
#define QT_UDP_ERROR_CONNECT(socket, receiver, slot) \
    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QUdpSocket::error), receiver, slot)
#else
#define QT_TCP_ERROR_CONNECT(socket, receiver, slot) \
    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred), receiver, slot)
#define QT_UDP_ERROR_CONNECT(socket, receiver, slot) \
    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QUdpSocket::errorOccurred), receiver, slot)
#endif

// 3. QThreadPool lambda支持兼容性 - 新增！
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
// Qt 5.15+ 支持直接传入lambda/std::function
#define QT_THREADPOOL_START(pool, func) pool->start(func)
#else
// Qt 5.12 需要QRunnable包装器
class QtCompatRunnable : public QRunnable {
public:
    explicit QtCompatRunnable(std::function<void()> func)
        : m_func(std::move(func)) {
        setAutoDelete(true);
    }

    void run() override {
        if (m_func) {
            m_func();
        }
    }

private:
    std::function<void()> m_func;
};

#define QT_THREADPOOL_START(pool, func) \
pool->start(new QtCompatRunnable(func))
#endif

// 3. QList vs QVector 兼容 (Qt6中QVector成为QList的别名)
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QList>
    template<typename T> using QCompactList = QList<T>;
#else
#include <QVector>
#include <QList>
    template<typename T> using QCompactList = QVector<T>; // Qt5中QVector更高效
#endif

// 4. QString::SkipEmptyParts vs QString::SplitBehavior 兼容
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
#define QT_STRING_SKIP_EMPTY_PARTS Qt::SkipEmptyParts
#else
#define QT_STRING_SKIP_EMPTY_PARTS QString::SkipEmptyParts
#endif

// 5. QDateTime::fromSecsSinceEpoch 兼容
#if QT_VERSION >= QT_VERSION_CHECK(5, 8, 0)
#define QT_DATETIME_FROM_SECS(secs) QDateTime::fromSecsSinceEpoch(secs)
#else
#define QT_DATETIME_FROM_SECS(secs) QDateTime::fromTime_t(secs)
#endif

// 6. QMetaObject::invokeMethod 兼容
#if QT_VERSION >= QT_VERSION_CHECK(5, 10, 0)
#define QT_INVOKE_METHOD(obj, func, ...) QMetaObject::invokeMethod(obj, func, ##__VA_ARGS__)
#else
#define QT_INVOKE_METHOD(obj, func, ...) QMetaObject::invokeMethod(obj, #func, ##__VA_ARGS__)
#endif

// 7. QRandomGenerator 兼容 (Qt 5.10+)
#if QT_VERSION >= QT_VERSION_CHECK(5, 10, 0)
#include <QRandomGenerator>
#define QT_RANDOM_BOUNDED(max) QRandomGenerator::global()->bounded(max)
#define QT_RANDOM_GENERATE() QRandomGenerator::global()->generate()
#else
#include <QTime>
#include <cstdlib>
inline void qtRandomSeed() {
    static bool seeded = false;
    if (!seeded) {
        qsrand(QTime::currentTime().msec());
        seeded = true;
    }
}
#define QT_RANDOM_BOUNDED(max) (qtRandomSeed(), qrand() % (max))
#define QT_RANDOM_GENERATE() (qtRandomSeed(), qrand())
#endif

// 8. QThread::create 兼容 (Qt 5.10+)
#if QT_VERSION >= QT_VERSION_CHECK(5, 10, 0)
#define QT_THREAD_CREATE(func) QThread::create(func)
#else
// 为Qt 5.12提供简单的QThread::create替代
class QtCompatThread : public QThread {
public:
    QtCompatThread(std::function<void()> func) : m_func(func) {}
protected:
    void run() override { m_func(); }
private:
    std::function<void()> m_func;
};
#define QT_THREAD_CREATE(func) (new QtCompatThread(func))
#endif

// 9. QSharedPointer::create 确保兼容
#if QT_VERSION >= QT_VERSION_CHECK(5, 1, 0)
#define QT_MAKE_SHARED(Type, ...) QSharedPointer<Type>::create(__VA_ARGS__)
#else
#define QT_MAKE_SHARED(Type, ...) QSharedPointer<Type>(new Type(__VA_ARGS__))
#endif

// 10. 容器的 reserve() 兼容
#define QT_CONTAINER_RESERVE(container, size) \
do { \
        if ((size) > 0) { \
            (container).reserve(size); \
    } \
} while(0)

// 11. 强制内联宏（跨编译器）
#ifdef Q_CC_MSVC
#define QT_FORCE_INLINE __forceinline
#elif defined(Q_CC_GNU)
#define QT_FORCE_INLINE inline __attribute__((always_inline))
#else
#define QT_FORCE_INLINE inline
#endif

// 12. 属性宏兼容
#if !defined(QT_DEPRECATED_X)
#  if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
#    define QT_DEPRECATED_X(text) Q_DECL_DEPRECATED_X(text)
#  else
#    define QT_DEPRECATED_X(text) Q_DECL_DEPRECATED
#  endif
#endif

// 13. ROS环境下的特殊处理
#ifdef ROS_BUILD
// ROS通常使用较老的Qt版本，需要特殊处理
#ifndef QT_NO_DEPRECATED_WARNINGS
#define QT_NO_DEPRECATED_WARNINGS
#endif

// ROS环境下禁用某些Qt6特性
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#warning "ROS build with Qt6 may have compatibility issues"
#endif
#endif

// 14. 调试输出兼容
#if QT_VERSION >= QT_VERSION_CHECK(5, 2, 0)
#define QT_DEBUG_OUTPUT(category, msg) qCDebug(category) << msg
#define QT_WARNING_OUTPUT(category, msg) qCWarning(category) << msg
#define QT_CRITICAL_OUTPUT(category, msg) qCCritical(category) << msg
#else
#define QT_DEBUG_OUTPUT(category, msg) qDebug() << msg
#define QT_WARNING_OUTPUT(category, msg) qWarning() << msg
#define QT_CRITICAL_OUTPUT(category, msg) qCritical() << msg
#endif

// 15. 类型安全的强制转换
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#define QT_CAST_INT(value) qsizetype(value)
#else
#define QT_CAST_INT(value) int(value)
#endif

#endif // QT_VERSION_COMPAT_H
