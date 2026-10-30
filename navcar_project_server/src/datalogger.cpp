#include "navcar_project_server/datalogger.h"
#include <QDebug>
#include <QStandardPaths>

// 静态成员初始化
const QMap<DataType, QString> DataLogger::s_fileNames = {
    {DataType::BMS, "bms_data.log"},
    {DataType::WalkMotor, "walk_motor_data.log"},
    {DataType::HydraulicPressure, "hydraulic_pressure_data.log"},
    {DataType::HydraulicTemperature, "hydraulic_temperature_data.log"},
    {DataType::AngleEncoder, "angle_encoder_data.log"},
    {DataType::PumpMotor, "pump_motor_data.log"},
    {DataType::GeneralLog, "general.log"}
};

const QMap<DataType, QString> DataLogger::s_typeNames = {
    {DataType::BMS, "BMS"},
    {DataType::WalkMotor, "行走电机"},
    {DataType::HydraulicPressure, "液压油压传感器"},
    {DataType::HydraulicTemperature, "液压油温传感器"},
    {DataType::AngleEncoder, "角度编码器"},
    {DataType::PumpMotor, "油泵电机参数"},
    {DataType::GeneralLog, "通用日志"}
};

DataLogger::DataLogger(QObject *parent)
    : QObject(parent)
    , m_logDirectory(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/logs")
    , m_autoSaveTimer(new QTimer(this))
    , m_isLogging(false)
{
    // 设置定时器
    m_autoSaveTimer->setSingleShot(false);
    connect(m_autoSaveTimer, &QTimer::timeout, this, &DataLogger::onAutoSave);

    // 创建日志目录
    QDir dir;
    if (!dir.exists(m_logDirectory)) {
        dir.mkpath(m_logDirectory);
    }
}

DataLogger::~DataLogger()
{
    stopLogging();
    closeAllFiles();
}

void DataLogger::setLogDirectory(const QString& dir)
{
    QMutexLocker locker(&m_mutex);

    if (m_isLogging) {
        qWarning() << "Cannot change directory while logging is active";
        return;
    }

    m_logDirectory = dir;

    // 创建目录
    QDir qdir;
    if (!qdir.exists(m_logDirectory)) {
        qdir.mkpath(m_logDirectory);
    }
}

void DataLogger::startLogging()
{
    QMutexLocker locker(&m_mutex);

    if (m_isLogging) {
        qWarning() << "Logging is already active";
        return;
    }

    qDebug() << "Starting data logging in directory:" << m_logDirectory;

    initializeFiles();
    m_isLogging = true;
    m_autoSaveTimer->start();

    // 写入启动标记
    QString startMsg = QString("=== 数据记录开始 %1 ===")
                           .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));

    for (auto type : s_fileNames.keys()) {
        writeToFile(type, startMsg);
    }
}

void DataLogger::stopLogging()
{
    QMutexLocker locker(&m_mutex);

    if (!m_isLogging) {
        return;
    }

    qDebug() << "Stopping data logging";

    // 写入结束标记
    QString endMsg = QString("=== 数据记录结束 %1 ===")
                         .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));

    for (auto type : s_fileNames.keys()) {
        writeToFile(type, endMsg);
    }

    m_autoSaveTimer->stop();

    // 强制保存并关闭文件
    for (auto stream : m_streams.values()) {
        if (stream) {
            stream->flush();
        }
    }

    m_isLogging = false;
    closeAllFiles();
}

void DataLogger::setAutoSaveInterval(int intervalMs)
{
    m_autoSaveTimer->setInterval(intervalMs);
}

void DataLogger::onCanMessage(CanMessage canMsg)
{
    if (!m_isLogging) return;
    QString parsedData=""; //= parseCanMessage(canMsg);

    // 根据CAN ID判断数据类型（这里需要根据实际项目的CAN协议来实现）
    DataType type = DataType::GeneralLog; // 默认类型

    switch (canMsg.id) {
    case 0x04F110D0:
    {
        type=DataType::WalkMotor;
        int motorI11=canMsg.data[0]*256+canMsg.data[1];
        int motorI12=canMsg.data[2]*256+canMsg.data[3];
        int motorI21=canMsg.data[4]*256+canMsg.data[5];
        int motorI22=canMsg.data[6]*256+canMsg.data[7];
        parsedData="电机11: "+QString::number(motorI11)+" "+"电机12: "+QString::number(motorI12)+" "
                     +"电机21: "+QString::number(motorI21)+" "+"电机22: "+QString::number(motorI22)+" ";
        break;
    }
    case 0x04F210D0:
    {
        type=DataType::WalkMotor;
        int motorI31=canMsg.data[0]*256+canMsg.data[1];
        int motorI32=canMsg.data[2]*256+canMsg.data[3];
        int motorI41=canMsg.data[4]*256+canMsg.data[5];
        int motorI42=canMsg.data[6]*256+canMsg.data[7];
        parsedData="电机31: "+QString::number(motorI31)+" "+"电机32: "+QString::number(motorI32)+" "
                     +"电机41: "+QString::number(motorI41)+" "+"电机42: "+QString::number(motorI42)+" ";
        break;
    }
    case 0x4F310D0:
    {
        type=DataType::HydraulicPressure;
        float hp1=(canMsg.data[0]*256+canMsg.data[1])*0.04;
        float hp2=(canMsg.data[2]*256+canMsg.data[3])*0.04;
        float hp3=(canMsg.data[4]*256+canMsg.data[5])*0.04;
        float hp4=(canMsg.data[6]*256+canMsg.data[7])*0.04;
        parsedData="油压传感器1: "+QString::number(hp1)+" "+"油压传感器2: "+QString::number(hp2)+" "
                     +"油压传感器3: "+QString::number(hp3)+" "+"油压传感器4: "+QString::number(hp4)+" ";
        break;
    }
    case 0x4F410D0:
    {
        type=DataType::HydraulicTemperature;
        float ht1=(canMsg.data[0]*256+canMsg.data[1])*0.01;
        float ht2=(canMsg.data[2]*256+canMsg.data[3])*0.01;
        float ht3=(canMsg.data[4]*256+canMsg.data[5])*0.01;
        float ht4=(canMsg.data[6]*256+canMsg.data[7])*0.01;
        parsedData="油温传感器1: "+QString::number(ht1)+" "+"油温传感器2: "+QString::number(ht2)+" "
                     +"油温传感器3: "+QString::number(ht3)+" "+"油温传感器4: "+QString::number(ht4)+" ";
        break;
    }
    case 0x10F8109A:
    case 0x10F8108D:
        break;
    case 0x181B9CF3:
    {
        type=DataType::BMS;
        float BV=(canMsg.data[0]*256+canMsg.data[1])*0.01;
        float BI=(canMsg.data[2]*256+canMsg.data[3])*0.1-3200;
        float SOC=(canMsg.data[4])*0.4;
        float SOE=(canMsg.data[5])*0.4;
        float SOH=(canMsg.data[6])*0.4;
        parsedData="BMS电压（V）: "+QString::number(BV)+" "+"BMS电流（A）: "+QString::number(BI)+" "
                     +"SOC（%）: "+QString::number(SOC)+" "+"SOE（%）: "+QString::number(SOE)+" "+"SOH（%）: "+QString::number(SOH)+" ";
        break;
    }
    default:
        break;
    }

    logData(type, parsedData);
}

void DataLogger::onTypeMessage(DataType type, const QString& content)
{
    if (!m_isLogging) return;

    logData(type, content);
}

void DataLogger::onGeneralLog(const QString& logMessage)
{
    if (!m_isLogging) return;

    logData(DataType::GeneralLog, logMessage);
}

void DataLogger::logData(DataType type, const QString& data)
{
    QMutexLocker locker(&m_mutex);

    if (!m_isLogging) return;

    writeToFile(type, data);
}

void DataLogger::onAutoSave()
{
    QMutexLocker locker(&m_mutex);

    // 刷新所有文件流
    for (auto stream : m_streams.values()) {
        if (stream) {
            stream->flush();
        }
    }

    qDebug() << "Auto save completed at" << QDateTime::currentDateTime().toString();
}

void DataLogger::initializeFiles()
{
    // 创建带日期的子目录
    QString dateDir = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    QString fullPath = m_logDirectory + "/" + dateDir;

    QDir dir;
    if (!dir.exists(fullPath)) {
        dir.mkpath(fullPath);
    }

    // 为每种数据类型创建文件
    for (auto it = s_fileNames.begin(); it != s_fileNames.end(); ++it) {
        DataType type = it.key();
        QString fileName = it.value();

        // 添加时间戳到文件名
        QString timeStamp = QDateTime::currentDateTime().toString("hhmmss");
        QString fullFileName = fileName;
        int dotIndex = fileName.lastIndexOf('.');
        if (dotIndex != -1) {
            fullFileName = fileName.left(dotIndex) + "_" + timeStamp + fileName.mid(dotIndex);
        } else {
            fullFileName += "_" + timeStamp;
        }

        QString filePath = fullPath + "/" + fullFileName;

        QFile* file = new QFile(filePath);
        if (file->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
            QTextStream* stream = new QTextStream(file);

            // 设置编码 - 兼容不同Qt版本
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            // Qt 6: 默认已经是UTF-8，可以显式设置
            stream->setEncoding(QStringConverter::Utf8);
#else
            // Qt 5: 使用setCodec
            stream->setCodec("UTF-8");
#endif

            m_files[type] = file;
            m_streams[type] = stream;

            // 写入文件头
            QString header = QString("# %1 数据记录文件").arg(getTypeName(type));
            *stream << header << "\n";
            *stream << QString("# 创建时间: %1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")) << "\n";
            *stream << "# 格式: 时间戳 数据内容" << "\n";
            *stream << QString(50, '-') << "\n";
            stream->flush();

            qDebug() << "Initialized file for" << getTypeName(type) << ":" << filePath;
        } else {
            qCritical() << "Failed to open file:" << filePath;
            delete file;
        }
    }
}

void DataLogger::closeAllFiles()
{
    // 关闭所有文件流
    for (auto stream : m_streams.values()) {
        if (stream) {
            stream->flush();
            delete stream;
        }
    }
    m_streams.clear();

    // 关闭所有文件
    for (auto file : m_files.values()) {
        if (file) {
            file->close();
            delete file;
        }
    }
    m_files.clear();
}

QString DataLogger::getFileName(DataType type) const
{
    return s_fileNames.value(type, "unknown.log");
}

QString DataLogger::getTypeName(DataType type) const
{
    return s_typeNames.value(type, "未知类型");
}

void DataLogger::writeToFile(DataType type, const QString& data)
{
    if (!m_streams.contains(type)) {
        return;
    }

    QTextStream* stream = m_streams[type];
    if (stream) {
        QString logEntry = createLogEntry(data);
        *stream << logEntry << "\n";

        // 定期刷新缓冲区（可选，根据性能需求调整）
        static int writeCount = 0;
        if (++writeCount % 10 == 0) {
            stream->flush();
        }
    }
}

QString DataLogger::createLogEntry(const QString& data) const
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
    return QString("%1 %2").arg(timestamp, data);
}

QString DataLogger::parseCanMessage(const CanMessage& canMsg)
{
    /*// 将CAN消息解析为字符串格式
    QString dataHex;
    for (int i = 0; i < canMsg.data.size(); ++i) {
        dataHex += QString("%1").arg(static_cast<quint8>(canMsg.data[i]), 2, 16, QChar('0')).toUpper();
        if (i < canMsg.data.size() - 1) {
            dataHex += " ";
        }
    }

    return QString("CAN_ID:0x%1 Data:[%2] Length:%3")
        .arg(canMsg.id, 3, 16, QChar('0')).toUpper()
        .arg(dataHex)
        .arg(canMsg.data.size());*/
}
