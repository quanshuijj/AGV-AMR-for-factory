#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QThread>
#include"navcarmanagernode.h"
// 在MainWindow类中添加
#include "mapdisplaywidget.h"
#include "pointmanager.h"
#include <QFileDialog>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>


QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void refreshPortData(QStringList portData);
    void logDataUpdate(QString);

    void refreshCANPortData(QStringList portData);

    void on_refreshLin1Button_clicked();

    void on_connectLin1Button_clicked();

    void on_disconnectLin1Button_clicked();

    void on_sendMsgLin1Button_clicked();

    void on_paramDatapushButton_clicked();

    void on_readTestpushButton_clicked();

    void on_refreshCANButton_clicked();

    void on_connectCANButton_clicked();

    void on_disconnectCANButton_clicked();

    void on_sendMsgCANButton_clicked();

    void on_frontBtn_clicked();

    void on_rearBtn_clicked();

    void on_leftBtn_clicked();

    void on_rightBtn_clicked();

    void on_stopBtn_clicked();

    void on_showLatestSensorDataBtn_clicked();

    void on_startListenBtn_clicked();

    void on_stopListenBtn_clicked();


    ///显示信息更新
    void updateMotorDisplay(int, int, bool, float, float, float);

    void updateOilPressureDisplay(float, float, float, float);

    void updateOilTemperatureDisplay(float);

    void updateModeDisplay(int);

    void updateHydraulicLockDisplay(bool, bool, bool, bool);

    void updateLightDisplay(bool, bool, bool, bool);
       
    void updateEmergencyDisplay(bool, bool, bool, bool);

    void updateBumpMotorSpeedDisplay(float);

    void updateEncoderDisplay(int, quint8,float);

    void updateOilPumpDisplay(float, float, float, float);

    void updateBatteryStatusDisplay(int, int, int, int);

    /// ROS地图回调函数
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    
    /// odom数据回调
    void updateOdomDisplay(const nav_msgs::Odometry& odom);
    /// 地图管理
    void on_loadMapButton_clicked();
    void on_saveMapButton_clicked();
    void on_clearMapButton_clicked();
    /// 地图状态更新
    void updateMapStatus();
    ///点管理
    void on_addFixedPointButton_clicked();
    void on_addCustomPointButton_clicked();
    void on_removePointButton_clicked();

    void updatePointList();
    void onPointAdded(const NavigationPoint& point);
    void onPointRemoved(int pointId);
    void showPointListContextMenu(const QPoint& pos);
    
    ///路径规划
    void on_planPathButton_clicked();
    void on_selectStartPointButton_clicked();
    void on_selectTargetPointButton_clicked();
    void onMapPointClicked(const QPointF &pos);
    
    // 速度显示
    void updateVelocityDisplay(double linear, double angular);

    // 开始导航按钮槽函数
    void on_startNavigationButton_clicked();
private:
    Ui::MainWindow *ui;
    NavCarManagerNode navCarManagerNode;
    QThread navCarManagerNodeThread;
    MapDisplayWidget* mapWidget;
    PointManager* pointManager;
    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber poseSub;
    ros::Subscriber pathSub;
    ros::Publisher goalPub;

    // 轨迹相关订阅
    ros::Subscriber globalPathSub;
    ros::Subscriber tebGlobalPathSub;
    ros::Subscriber tebLocalPathSub;
    ros::Subscriber laserOdomSub;
    ros::Subscriber originalLaserOdomSub;
    // ros::Subscriber moveBaseFeedbackSub;
    // ros::Subscriber moveBaseResultSub;
    // ros::Subscriber moveBaseStatusSub;

    // 轨迹相关回调
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void tebGlobalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void tebLocalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void laserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void originalLaserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    // void moveBaseFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
    // void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
    // void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

    // 辅助函数
    void addPointToList(const NavigationPoint& point);
    
    // 路径规划相关
    int startPointId = -1;      // 起点ID
    int goalPointId = -1;       // 终点ID

    enum SelectionMode {
        ModeNone,
        ModeStartPoint,
        ModeTargetPoint,
        ModeAddFixedPoint,
        ModeAddCustomPoint
    };
    SelectionMode m_selectionMode = ModeNone;

    void updatePathPlanningStatus();

    QPointF getSelectedMapPosition() const;
signals:
    //LIN1 信号
    void ConnectToLIN1PortSignal(QString,qint32);
    void DisConnectToLIN1Signal();
    void SendLIN1MsgSignal(quint8,QByteArray);
    //LIN2 信号
    void ConnectToLIN2PortSignal(QString,qint32);
    void DisConnectToLIN2Signal();
    void SendLIN2MsgSignal(quint8,QByteArray);

    void RefreshAvailablePortsSignal();

    //CAN操作
    void RefreshAvailableCANPortsSignal();
};
#endif // MAINWINDOW_H
