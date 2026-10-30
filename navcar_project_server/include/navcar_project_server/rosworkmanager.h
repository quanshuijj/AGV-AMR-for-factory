#ifndef ROSWORKMANAGER_H
#define ROSWORKMANAGER_H

#include <QObject>
#include <ros/ros.h>
#include <QTimer>
#include <QQueue>
#include <QList>
#include <QMutex>
#include <QMutexLocker>
#include <algorithm>
#include <sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <QLoggingCategory>
#include <QDateTime>
#include <cmath>
#include <limits>

// 引入GlobalPathNavigator
#include "navcar_project_server/globalpathnavigator.h"
#include "networkprotocol.h"
#include "chcnav/hcinspvatzcb.h"

Q_DECLARE_LOGGING_CATEGORY(rosDataLog)

// 导航模式枚举（扩展版本）
enum NavigationMode {
    NAV_IDLE = 0,                    // 空闲状态
    NAV_TEB_MODE = 1,                // 阶段1：TEB领航（直线+大转弯）
    NAV_GLOBAL_PLANNER_MODE = 2,     // 阶段2：GlobalPath直接跟踪
    NAV_APPROACH_MODE = 3,           // 阶段3：斜行接近（3-5m范围）
    NAV_ORIENTATION_MODE = 4,        // 阶段4：姿态调整（原地旋转）
    NAV_REACHED = 5                  // 到达目标
};

class ROSWorkManager : public QObject
{
    Q_OBJECT

public:
    explicit ROSWorkManager(QObject *parent = nullptr);
    ~ROSWorkManager();

    // 公共方法
    void startROSWork();
    void stopROSWork();
    bool isROSWorking() const { return isROSWork; }

    // 导航模式控制
    //void setNavigationMode(NavigationMode mode);
    NavigationMode getCurrentNavigationMode() const { return currentNavMode; }
    QString getNavigationModeString() const;

    // GlobalPathNavigator访问
    GlobalPathNavigator* getGlobalPathNavigator() { return globalPathNav; }

    // 获取当前状态
    double getCurrentVelX() const { return currentVelX; }
    double getCurrentVelZ() const { return currentVelZ; }
    bool isMotorCurrentlyActive() const { return isMotorActive; }
    qint64 getLastCommandTime() const { return lastCmdTime; }

    // 导航状态相关
    double getFilteredYaw() const;
    geometry_msgs::Point getFilteredPosition() const;
    double getDistanceToTarget() const;

    // 导航暂停状态查询
    bool isNavigationPaused() const { return navigationPaused; }

public slots:
    // 导航恢复/暂停控制 - 用于自动/手动模式切换（作为槽函数以支持跨线程调用）
    void resumeNavigation();      // 恢复导航（从手动模式切换回自动模式时调用）
    void pauseNavigation();       // 暂停导航（切换到手动模式时调用）

private:
    // ROS相关
    ros::NodeHandle n;
    ros::Subscriber velCMDSub;
    ros::Subscriber cloudPointSub;
    ros::Subscriber imuDataSub;
    ros::Subscriber odomSub;                    // 里程计订阅器
    ros::Subscriber moveBaseGoalSub;            // move_base目标订阅器
    ros::Subscriber globalPathSub;              // 全局路径订阅器
    ros::Subscriber navModeSub;                 // 导航模式切换订阅器
    ros::Subscriber camFrontSub;
    ros::Subscriber camLeftSub;
    ros::Subscriber camRightSub;
    ros::Subscriber camRearSub;
    ros::Subscriber gpsSub;

    // ========== 发布者（用于重新规划）==========
    ros::Publisher goalPub;                     // 用于重新发送目标触发重新规划
    ros::Publisher smoothedTebLocalPlanPub;     // 发布控制层实际采纳的平滑TEB路径，便于调试
    ros::ServiceClient clearCostmapsClient;      // 清理move_base代价地图，处理动态障碍残留

    // TF2监听器
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

    // ========== Costmap订阅（用于障碍物检测/调试）==========
    ros::Subscriber localCostmapSub;
    nav_msgs::OccupancyGrid latestCostmap;
    mutable QMutex costmapMutex;
    bool hasCostmapData = false;
    QTime lastCostmapTime;
    QTime lastCostmapClearTime;
    QTime lastDirectLidarObstacleTime;
    
    // ========== TEB Local Plan订阅（用于实时避障跟踪）==========
    ros::Subscriber tebLocalPlanSub;
    nav_msgs::Path tebLocalPlan;                // TEB局部规划路径（已避障）
    nav_msgs::Path sbplGlobalPath;              // 保存SBPL全局路径（用于避障后恢复）
    bool hasTebLocalPlan = false;               // 是否有有效的TEB local_plan
    bool isAvoidingObstacle = false;            // 当前是否正在避障（跟踪TEB路径）
    QTime lastTebLocalPlanTime;                 // 最后收到TEB local_plan的时间
    QTime lastAvoidancePathUpdateTime;           // 上次采纳TEB避障路径的时间
    QTime obstacleDetectedTime;                 // 检测到障碍物的时间
    QTime pathBlockedSince;                     // 全局路径走廊开始被阻断的时间
    QTime lastGlobalReplanRequestTime;          // 上次请求全局重规划的时间
    QTime lastMissingGlobalPathWarningTime;     // GlobalPath空路径告警节流
    bool globalReplanPending = false;           // 已触发全局重规划，等待新的SBPL路径
    int consecutivePositionJumpCount = 0;       // 连续位置跳变次数，用于重定位后重置滤波
    int consecutiveYawJumpCount = 0;            // 连续航向跳变次数，用于重定位后重置滤波
    static constexpr int TEB_LOCAL_PLAN_TIMEOUT_MS = 1000;  // TEB local_plan超时时间(ms)
    static constexpr double TEB_LOCAL_PLAN_RESAMPLE_SPACING = 0.25; // 采纳TEB局部路径前先加密，减少折线跟踪
    static constexpr int TEB_LOCAL_PLAN_SMOOTH_WINDOW = 2; // TEB局部路径平滑窗口，减少折线抖动
    static constexpr int TEB_LOCAL_PLAN_SMOOTH_ITERATIONS = 2; // 平滑迭代次数
    static constexpr int AVOIDANCE_PATH_UPDATE_INTERVAL_MS = 8000; // 锁定避障路径一段时间，避免频繁换方案
    static constexpr int OBSTACLE_CLEAR_DELAY_MS = 1500;   // 障碍物消失后延迟恢复时间(ms)
    
    // ========== 障碍物检测参数（保留用于调试）==========
    static constexpr double OBSTACLE_CHECK_DISTANCE = 5.0;   // 近场才触发避障，远处先按全局路径走
    static constexpr double COLLISION_STOP_DISTANCE = 2.5;   // 预测可能碰撞时立即刹停
    static constexpr double OBSTACLE_SAFE_DISTANCE = 4.0;    // 安全距离(m)
    static constexpr int OBSTACLE_COST_THRESHOLD = 80;       // 障碍物代价阈值
    static constexpr double AVOIDANCE_CAUTION_SPEED = 0.18;  // 局部路径可疑但仍可尝试时的谨慎绕障速度
    static constexpr int COSTMAP_CLEAR_INTERVAL_MS = 2000;   // 动态障碍残留清理间隔
    static constexpr int PATH_BLOCK_CONFIRM_MS = 1200;      // 连续阻断超过该时间才认定真的需要脱离全局路径
    static constexpr int GLOBAL_REPLAN_REQUEST_INTERVAL_MS = 1500; // 重规划请求节流

    // 定时器
    QTimer *mainTimer;              // 主定时器 (50Hz)

    quint8 pointCloudCountNum=0;

    // 控制状态
    bool isROSWork = false;         // ROS工作状态
    bool isMotorActive = false;     // 电机激活状态

    // 控制指令缓存MIN
    double currentVelX = 0.0;       // 当前线速度
    double currentVelZ = 0.0;       // 当前角速度
    qint64 lastCmdTime = 0;         // 最后收到cmd_vel的时间戳 (ms)

    // ========== 导航状态管理 ==========
    NavigationMode currentNavMode = NAV_IDLE;           // 当前导航模式
    NavigationMode lastNavMode = NAV_IDLE;              // 上次导航模式
    NavigationMode savedNavModeBeforePause = NAV_IDLE;  // 暂停前的导航模式（用于恢复）
    geometry_msgs::PoseStamped targetPose;              // 目标位姿
    nav_msgs::Odometry currentOdom;                     // 当前里程计数据
    bool hasValidTarget = false;                        // 是否有有效目标
    bool hasValidOdom = false;                          // 是否有有效里程计
    bool hasValidGlobalPath = false;                    // 是否有有效全局路径
    bool navigationPaused = false;                      // 导航是否被暂停（手动模式时）

    // ========== GlobalPathNavigator集成 ==========
    GlobalPathNavigator* globalPathNav;                 // 全局路径导航器
    bool globalPathEnabled = false;                     // 全局路径导航是否启用
    QTime lastGlobalPathTime;                          // 上次收到全局路径时间
    QTime lastGlobalPathAdoptionTime;                  // 上次采纳全局路径作为跟踪路径的时间
    static constexpr int GLOBAL_PATH_TIMEOUT_MS = 20000; // 全局路径超时时间
    static constexpr int GLOBAL_PATH_ADOPTION_INTERVAL_MS = 3000; // 液压转向慢，低频采纳全局重规划

    // ========== 滤波器相关 ==========
    QQueue<double> yawHistory;                      // 航向角历史数据
    QQueue<geometry_msgs::Point> posHistory;        // 位置历史数据
    QQueue<qint64> timeHistory;                     // 时间戳历史
    static constexpr int FILTER_WINDOW = 5;        // 滤波窗口大小
    static constexpr double MAX_YAW_JUMP = M_PI/4;  // 最大航向角跳变阈值
    static constexpr double MAX_POS_JUMP = 2.0;     // 最大位置跳变阈值(m)

    // ========== 距离和角度阈值 ==========
    static constexpr double APPROACH_DISTANCE_ENTER = 4.0;   // 进入斜行模式距离
    static constexpr double APPROACH_DISTANCE_EXIT = 4.5;    // 退出斜行模式距离（滞回）
    static constexpr double REACHED_DISTANCE = 0.5;          // 到达判断距离
    static constexpr double ORIENTATION_TOLERANCE = 0.15;    // 姿态容忍度（约8.6度）
    static constexpr double APPROACH_SPEED = 0.15;           // 斜行接近速度
    static constexpr double ORIENTATION_SPEED = 0.05;        // 姿态调整角速度

    // 配置参数
    static constexpr int CMD_TIMEOUT_MS = 500;      // cmd_vel超时时间(ms)
    static constexpr int MAIN_TIMER_MS = 20;        // 主定时器间隔(ms) = 50Hz
    static constexpr double MIN_VEL_THRESHOLD = 0.005; // 最小速度阈值

    //数据下采样系数
    int GPSDataCount=0;
    static constexpr int GPS_DOWN_SAMPLING_PARAMS=10;

    // ========== 加速度监控 ==========
    double lastLinearVel = 0.0;
    double lastAngularVel = 0.0;
    QTime lastVelTime;
    static constexpr double MAX_LINEAR_ACCEL = 0.015;   // m/s²
    static constexpr double MAX_ANGULAR_ACCEL = 0.01;  // rad/s²

    // ========== 转向角度限制 ==========
    static constexpr double MAX_STEERING_ANGLE_DEG = 25.0;  // 最大转向角25度，对应最小转弯半径3.2m
    static constexpr double WHEELBASE = 2.5;
    static constexpr double MAX_STEERING_ANGLE_RAD = MAX_STEERING_ANGLE_DEG * M_PI / 180.0;

    // ROS回调函数
    void imuDataCallback(const sensor_msgs::Imu& imuData);
    void velCMDCallback(const geometry_msgs::Twist& velCMD);
    void odomCallback(const nav_msgs::Odometry& odom);
    void moveBaseGoalCallback(const move_base_msgs::MoveBaseActionGoal& goal);
    void simpleGoalCallback(const geometry_msgs::PoseStamped& goal);  // 新增：处理RVIZ简单目标
    void globalPathCallback(const nav_msgs::Path& path);
    void tebLocalPlanCallback(const nav_msgs::Path& path);  // TEB局部规划路径回调
    void navModeCallback(const std_msgs::Int32& mode);

    void pointCloudCallback(const sensor_msgs::PointCloud2& cloud);
    void camFrontCallback(const sensor_msgs::Image& img);
    void camLeftCallback(const sensor_msgs::Image& img);
    void camRightCallback(const sensor_msgs::Image& img);
    void camRearCallback(const sensor_msgs::Image& img);

    void gpsDataCallback(const chcnav::hcinspvatzcb::ConstPtr &msg);
    void localCostmapCallback(const nav_msgs::OccupancyGrid& costmap);  // costmap回调（调试用）

    // ========== TEB Local Plan处理 ==========
    bool isTebLocalPlanValid() const;           // 检查TEB local_plan是否有效
    nav_msgs::Path transformPathToFrame(const nav_msgs::Path& path, const std::string& targetFrame) const; // 统一控制路径坐标系
    nav_msgs::Path densifyPath(const nav_msgs::Path& path, double spacing) const; // 对局部路径插值加密
    nav_msgs::Path smoothPath(const nav_msgs::Path& path, int window, int iterations) const; // 平滑局部路径，减少折线
    int getCostmapValue(double x, double y);    // 获取指定位置的costmap值
    bool getRobotPoseInOdom(geometry_msgs::PoseStamped& pose) const;  // 获取odom坐标系下的机器人位置
    
    // ========== 障碍物检测方法 ==========
    bool checkObstacleAhead(double checkDistance);  // 检测前方是否有障碍物
    bool isPathBlockedAhead(const nav_msgs::Path& path, double checkDistance, const QString& pathName); // 检查指定路径走廊是否被障碍阻断
    bool isGlobalPathBlockedAhead(double checkDistance); // 检查全局路径走廊是否被障碍阻断
    bool isTebLocalPlanBlockedAhead(double checkDistance); // 检查TEB局部路径走廊是否被障碍阻断
    bool isTebLocalPlanDivergingFromGlobalPath(double minDeviation, double checkDistance) const; // TEB是否正在绕离全局路径
    bool restoreGlobalPathIfUsable(const QString& reason); // 全局路径未被阻断时强制恢复GlobalPath
    bool isPathClear();                             // 检查当前跟踪路径是否畅通
    bool clearNavigationCostmaps(const QString& reason); // 主动清理导航代价地图

    // ========== 导航控制方法 ==========
    void updateNavigationState();                   // 更新导航状态
    void handleIdleMode();                          // 处理空闲模式
    void handleTebMode();                          // 处理TEB模式
    void handleGlobalPlannerMode();                // 处理GlobalPath模式
    void handleApproachMode();                     // 处理斜行接近模式
    void handleOrientationMode();                  // 处理姿态调整模式
    void handleReachedMode();                      // 处理到达模式

    // ========== 模式切换管理 ==========
    void switchNavigationMode(NavigationMode newMode);
    bool canSwitchToMode(NavigationMode newMode) const;
    void onNavigationModeChanged(NavigationMode oldMode, NavigationMode newMode);

    // ========== GlobalPath集成方法 ==========
    void initializeGlobalPathNavigator();
    void enableGlobalPathMode(bool enable);
    bool isGlobalPathReady() const;
    void updateGlobalPathStatus();
    bool requestGlobalReplan(const QString& reason);

    // ========== 计算方法 ==========
    double calculateDistanceToTarget() const;      // 计算到目标的距离
    double calculateTargetYaw() const;             // 计算目标航向角
    double calculateYawError() const;              // 计算航向角误差
    double normalizeAngle(double angle) const;     // 角度归一化
    double quaternionToYaw(const geometry_msgs::Quaternion& q) const; // 四元数转航向角

    // ========== TF获取方法 ==========
    bool getRobotPoseInMap(geometry_msgs::PoseStamped& pose) const; // 获取机器人在map坐标系下的位姿

    // ========== 滤波方法 ==========
    void updateFilterBuffers();                   // 更新滤波缓冲区
    bool detectYawJump(double newYaw) const;      // 检测航向角跳变
    bool detectPositionJump(const geometry_msgs::Point& newPos) const; // 检测位置跳变
    double medianFilter(const QQueue<double>& data) const; // 中值滤波
    geometry_msgs::Point averageFilter(const QQueue<geometry_msgs::Point>& data) const; // 平均滤波

    // ========== 安全检查方法 ==========
    bool isSteeringAngleSafe(double vx, double rz) const;
    geometry_msgs::Twist limitSteeringAngle(const geometry_msgs::Twist& cmd) const;
    double calculateSteeringAngle(double vx, double rz) const;
    double calculateMaxAngularVel(double vx) const;
    void monitorAcceleration(double currentLinear, double currentAngular);

    // 内部方法
    void sendStopCommand();         // 发送停止指令
    bool isCommandTimeout() const;  // 检查是否超时
    void logMotorStatus(const QString& action) const; // 记录电机状态
    void logNavigationState(const QString& info) const; // 记录导航状态

private slots:
    void mainLoop();                // 主循环：ROS消息处理 + 导航控制 + 超时检查

    // GlobalPathNavigator信号槽
    void onGlobalPathTrackingStateChanged(GlobalPathNavigator::TrackingState state);
    void onGlobalPathProgressUpdated(double progress);
    void onGlobalPathCompleted();
    void onGlobalPathFailed(const QString& reason);
    //切换模式
    void setNavigationMode(NavigationMode mode);

signals:
    // 电机控制信号
    void SetVelCMDSignal(double vx, double rz);

    // 导航控制信号
    void SetApproachCMDSignal(double targetX, double targetY, double currentX,
                              double currentY, double currentYaw);
    void SetOrientationCMDSignal(double targetYaw, double currentYaw);

    // 状态信号
    void MotorStartedSignal();      // 电机启动信号
    void MotorStoppedSignal();      // 电机停止信号
    void CmdTimeoutSignal();        // 指令超时信号
    void ROSWorkStartedSignal();    // ROS工作启动信号
    void ROSWorkStoppedSignal();    // ROS工作停止信号

    // 导航状态信号
    void NavigationModeChanged(NavigationMode newMode, const QString& info);
    void NavigationCompleted();     // 导航完成信号
    void NavigationTargetSet(const geometry_msgs::PoseStamped& target);
    void FilterJumpDetected(const QString& type, double value);

    // GlobalPath相关信号
    void GlobalPathReceived(const nav_msgs::Path& path);
    void GlobalPathTrackingProgress(double progress);
    void GlobalPathTrackingCompleted();
    void GlobalPathTrackingFailed(const QString& reason);
    void ObstacleDetectedSignal(bool detected, double distance);  // 新增：障碍物检测信号

    // 数据信号
    void ImuDataReceivedSignal(const sensor_msgs::Imu& imuData);
    void VelCmdReceivedSignal(double vx, double rz);
    void OdomReceivedSignal(const nav_msgs::Odometry& odom);
    void FilteredDataUpdated(double filteredYaw, const geometry_msgs::Point& filteredPos);

    //传递图片，传递点云到主处理线程
    void NewPointCloudRecievd(QSharedPointer<sensor_msgs::PointCloud2> cloudMsg);
    void NewCamFrontRecievd(QSharedPointer<sensor_msgs::Image> img1Msg);
    void NewCamLeftRecievd(QSharedPointer<sensor_msgs::Image> img2Msg);
    void NewCamRightRecievd(QSharedPointer<sensor_msgs::Image> img3Msg);
    void NewCamRearRecievd(QSharedPointer<sensor_msgs::Image> img4Msg);

    //传递GPS数据到主线程
    void NewGPSReciev(NetDataGPSData myGPSMsg);
};

#endif // ROSWORKMANAGER_H
