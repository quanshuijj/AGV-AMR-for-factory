#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QMap>
#include <QMutex>
#include <QDateTime>
#include <QDir>
#include <QString>
#include "socketcanmanager.h"

// Qt版本兼容性头文件
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QStringConverter>
#else
#include <QTextCodec>
#endif


// 数据类型枚举
enum class DataType {
    BMS,                    // BMS数据
    WalkMotor,             // 行走电机数据
    HydraulicPressure,     // 液压油压传感器
    HydraulicTemperature,  // 液压油温传感器
    AngleEncoder,          // 角度编码器
    PumpMotor,             // 油泵电机参数
    GeneralLog             // 通用日志
};

class DataLogger : public QObject
{
    Q_OBJECT

public:
    explicit DataLogger(QObject *parent = nullptr);
    ~DataLogger();

    // 设置日志保存目录
    void setLogDirectory(const QString& dir);

    // 启动数据记录（通常在新线程中调用）
    void startLogging();

    // 停止数据记录
    void stopLogging();

    // 设置自动保存间隔（毫秒）
    void setAutoSaveInterval(int intervalMs = 5000);

public slots:
    // 槽函数1：接收CAN消息
    void onCanMessage(CanMessage canMsg);

    // 槽函数2：接收消息类型和内容
    void onTypeMessage(DataType type, const QString& content);

    // 槽函数3：通用日志接口
    void onGeneralLog(const QString& logMessage);

    // 线程安全的日志记录接口
    void logData(DataType type, const QString& data);

private slots:
    // 定时保存数据
    void onAutoSave();

private:
    // 初始化文件
    void initializeFiles();

    // 关闭所有文件
    void closeAllFiles();

    // 获取数据类型对应的文件名
    QString getFileName(DataType type) const;

    // 获取数据类型对应的显示名称
    QString getTypeName(DataType type) const;

    // 写入数据到指定类型的文件
    void writeToFile(DataType type, const QString& data);

    // 创建带时间戳的日志条目
    QString createLogEntry(const QString& data) const;

    // 解析CAN消息（根据实际需求实现）
    QString parseCanMessage(const CanMessage& canMsg);

private:
    QString m_logDirectory;                    // 日志目录
    QMap<DataType, QFile*> m_files;           // 文件映射
    QMap<DataType, QTextStream*> m_streams;   // 文件流映射
    QTimer* m_autoSaveTimer;                  // 自动保存定时器
    QMutex m_mutex;                           // 线程安全互斥锁
    bool m_isLogging;                         // 记录状态

    static const QMap<DataType, QString> s_fileNames;    // 文件名映射
    static const QMap<DataType, QString> s_typeNames;    // 类型名映射
};

#endif // DATALOGGER_H
