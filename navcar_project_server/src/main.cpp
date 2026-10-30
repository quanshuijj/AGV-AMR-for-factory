#include "navcar_project_server/mainwindow.h"

#include <QApplication>
#include <ros/ros.h>

#include <QLoggingCategory>

// 声明值类型
Q_DECLARE_METATYPE(QSharedPointer<sensor_msgs::PointCloud2>);
Q_DECLARE_METATYPE(QSharedPointer<sensor_msgs::Image>);
Q_DECLARE_METATYPE(EchoData);
Q_DECLARE_METATYPE(BroadcastConfig);
Q_DECLARE_METATYPE(NetDataGPSData);
Q_DECLARE_METATYPE(CompressionResult);
Q_DECLARE_METATYPE(DracoCompressionResult)
Q_DECLARE_METATYPE(CrossPlatformPointCloud)
Q_DECLARE_METATYPE(CompressionLevel)
Q_DECLARE_METATYPE(CanMessage)

int main(int argc, char *argv[])
{
    // 运行时终端只保留导航相关的关键信息；设备/UI调试输出默认关闭。
    QLoggingCategory::setFilterRules(QStringLiteral(
        "*.debug=false\n"
        "default.info=false\n"
        "network.info=false\n"
        "broadcast.info=false\n"
        "compression.info=false\n"
        "draco.info=false\n"
        "h264.cache.info=false\n"
        "rosData.info=true\n"
        "globalPath.info=true\n"
    ));

    QApplication a(argc, argv);
    ros::init(argc,argv,"navcar_manager_node");
    //注册PointCloud2及其引用类型
    //qRegisterMetaType<sensor_msgs::PointCloud2>("sensor_msgs::PointCloud2");
    qRegisterMetaType<QSharedPointer<sensor_msgs::PointCloud2>>();
    qRegisterMetaType<QSharedPointer<sensor_msgs::Image>>();
    qRegisterMetaType<EchoData>();
    qRegisterMetaType<BroadcastConfig>();
    qRegisterMetaType<NetDataGPSData>();
    qRegisterMetaType<CompressionResult>();
    qRegisterMetaType<DracoCompressionResult>();
    qRegisterMetaType<CrossPlatformPointCloud>();
    qRegisterMetaType<CompressionLevel>();
    qRegisterMetaType<CanMessage>();
    MainWindow w;
    w.show();
    return a.exec();
}
