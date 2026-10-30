#ifndef GLOBALPATHNAVIGATOR_H
#define GLOBALPATHNAVIGATOR_H

#include <QObject>
#include <QVector>
#include <QTime>
#include <QLoggingCategory>
#include <QtMath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

Q_DECLARE_LOGGING_CATEGORY(globalPathLog)

/**
 * @brief 全局路径跟踪算法类
 * 专注于路径跟踪算法，不处理ROS通信
 * 作为ROSWorkManager的导航策略模块
 * 修改版：以转向角度为主要控制参数
 */
class GlobalPathNavigator : public QObject
{
    Q_OBJECT

public:
    explicit GlobalPathNavigator(QObject *parent = nullptr);
    ~GlobalPathNavigator() = default;

    // ========== 路径跟踪状态枚举 ==========
    enum TrackingState {
        TRACKING_IDLE = 0,           // 空闲状态
        TRACKING_FOLLOWING = 1,      // 路径跟踪中
        TRACKING_APPROACHING = 2,    // 接近目标点
        TRACKING_COMPLETED = 3,      // 跟踪完成
        TRACKING_FAILED = 4          // 跟踪失败
    };

    // ========== 控制算法枚举 ==========
    enum ControlAlgorithm {
        PURE_PURSUIT = 0,            // Pure Pursuit算法
        STANLEY_CONTROLLER = 1,      // Stanley控制器
        DIRECT_STEERING = 2,         // 直接转向控制
        LOOK_AHEAD = 3              // 前瞻控制
    };

    // ========== 控制输出结构 ==========
    struct ControlOutput {
        double linearVel = 0.0;      // 线速度 (m/s)
        double angularVel = 0.0;     // 角速度 (rad/s) - 由转向角度计算得出
        double steeringAngle = 0.0;  // 转向角度 (rad) - 主要控制参数
        int targetIndex = -1;        // 当前目标点索引
        double crossTrackError = 0.0; // 横向误差 (m)
        double headingError = 0.0;   // 航向误差 (rad)
        double distanceToGoal = 0.0; // 到目标距离 (m)
        bool isValid = false;        // 输出是否有效
    };

    // ========== 路径状态结构 ==========
    struct PathStatus {
        TrackingState state = TRACKING_IDLE;
        double progress = 0.0;       // 路径进度 (0-100%)
        double totalLength = 0.0;    // 总路径长度 (m)
        double remainingDistance = 0.0; // 剩余距离 (m)
        int currentIndex = 0;        // 当前路径点索引
        int totalPoints = 0;         // 总路径点数
        QString statusInfo;          // 状态信息
    };

    // ========== 公共接口方法 ==========
    // 路径管理
    bool setGlobalPath(const nav_msgs::Path& path);
    bool setGoalPose(const geometry_msgs::PoseStamped& goal);
    void clearPath();
    bool hasValidPath() const { return !pathPoints.isEmpty(); }

    // 控制计算（核心方法）
    ControlOutput calculateControl(const geometry_msgs::PoseStamped& currentPose,
                                   const geometry_msgs::Twist& currentVel = geometry_msgs::Twist());

    // 状态查询
    PathStatus getPathStatus() const;
    TrackingState getCurrentState() const { return currentState; }
    double getPathProgress() const;
    bool isTrackingComplete() const { return currentState == TRACKING_COMPLETED; }

    // 参数设置
    void setControlAlgorithm(ControlAlgorithm algorithm) { controlAlgorithm = algorithm; }
    void setLookaheadDistance(double distance) { lookaheadDistance = qBound(minLookaheadDist, distance, maxLookaheadDist); }
    void setMaxVelocity(double linearVel, double angularVel);
    void setTolerances(double waypointTol, double goalTol);

    // 转向角度参数设置（新增）
    void setSteeringLimits(double maxAngle, double maxRate);
    void setVehicleParams(double wheelbase, double wheelRadius);

    // 液压转向适配参数
    void setHydraulicParams(double smoothFactor, double maxAccel, double delay);

private:
    // ========== 路径数据 ==========
    nav_msgs::Path originalPath;                    // 原始路径
    QVector<geometry_msgs::PoseStamped> pathPoints; // 处理后的路径点
    geometry_msgs::PoseStamped goalPose;           // 目标位姿

    // ========== 跟踪状态 ==========
    TrackingState currentState = TRACKING_IDLE;
    ControlAlgorithm controlAlgorithm = PURE_PURSUIT;
    int currentTargetIndex = 0;                    // 当前目标点索引
    double totalPathLength = 0.0;                  // 总路径长度
    double accumulatedDistance = 0.0;              // 累计行驶距离

    // ========== 车辆参数（基于CalIKControlModel）==========
    double wheelBase = 2.5;                       // 前后轴距 (m) - 来自CalIKControlModel::L
    double wheelRadius = 0.226;                   // 轮半径 (m) - 来自CalIKControlModel::WHEEL_RADIUS
    double trackWidth = 1.25;                     // 转向机构间距 (m) - 来自CalIKControlModel::W

    // ========== 转向角度限制参数（新增）==========
    double maxSteeringAngle = 28.0 * M_PI / 180;  // 最大转向角度 (28度) - 对应最小转弯半径4.5m
    double minSteeringAngle = -28.0 * M_PI / 180; // 最小转向角度 (-28度) - 对称转向
    double maxSteeringRate = 0.21;                // 最大转向角度变化率 (rad/s) - 30度/2.5秒=12度/秒
    double lastSteeringAngle = 0.0;               // 上次转向角度

    // ========== 控制参数 ==========
    // 基础参数 - 大型车辆需要更大的前瞻距离
    double lookaheadDistance = 8.0;               // 前瞻距离（3.0→5.0，大型车辆需要更远前瞻）
    double minLookaheadDist = 3.0;                // 最小前瞻距离（1.5→3.0）
    double maxLookaheadDist = 10.0;               // 最大前瞻距离（6.0→10.0）
    double lookaheadGain = 1.5;                   // 前瞻增益（1.2→1.5）

    // 速度限制（基于CalIKControlModel）
    double maxLinearVel = 1.2;                   // 最大线速度
    const double MAX_VEL_X = 1.2;                // 最大速度限制
    double minLinearVel = 0.10;                  // 最小线速度，避免全局跟踪时蠕行
    double turnMinLinearVel = 0.25;              // 大转弯时保持的稳定低速
    double maxLinearAccel = 0.12;                // 线速度变化率限制，避免急停急启
    double maxAngularVel = 0.45;                  // 最大角速度（1.2m/s、25度转向角约0.45rad/s）
    double cruisingSpeed = 1.0;                   // 全局路径巡航速度
    double approachSpeed = 0.30;                  // 接近速度（0.20→0.30）
    mutable double lastLinearCmd = 0.0;          // 上次输出线速度，用于速度斜率限制
    mutable QTime lastLinearLimitTime;           // 上次限速时间

    // 容忍度参数 - 大型车辆需要更大的容忍度
    double waypointTolerance = 2.0;               // 路径点容忍度
    double goalTolerance = 0.5;                   // 目标容忍度
    double pathSkipDistance = 4.0;                // 路径点跳跃距离
    double finalApproachDistance = 3.0;           // 最终接近距离
    double maxCrossTrackError = 5.0;              // 最大横向误差（8.0→5.0，更早触发重规划）
    double maxHeadingError = 2.0;                 // 最大航向误差（约115度，超过则失败）

    // ========== 液压转向适配参数 ==========
    // 注意：平滑系数越大越平滑，越小越敏感
    double angularSmoothFactor = 0.8;             // 角速度平滑系数（0.7→0.8，更平滑）
    double steeringSmoothFactor = 0.8;            // 转向角度平滑系数（0.7→0.8，减少转向抖动）
    double maxAngularAccel = 0.10;                // 最大角加速度（0.08→0.06，更平滑）
    double steeringDelay = 0.15;                   // 转向延迟
    double lastAngularCmd = 0.0;                  // 上次角速度命令

    // ========== 状态监控 ==========
    geometry_msgs::PoseStamped lastPose;          // 上次位置
    QTime lastUpdateTime;                         // 上次更新时间
    double stuckTimeThreshold = 5.0;              // 卡住时间阈值
    double stuckDistanceThreshold = 0.2;          // 卡住距离阈值

    // ========== 核心算法方法 ==========
    // 路径处理
    void processPath();
    double calculatePathLength() const;
    void updatePathProgress(const geometry_msgs::PoseStamped& currentPose);

    // 目标点查找
    int findTargetWaypoint(const geometry_msgs::PoseStamped& currentPose);
    int findNearestWaypointIndex(const geometry_msgs::PoseStamped& currentPose) const; // 查找最近路径点
    geometry_msgs::PoseStamped findLookaheadPoint(const geometry_msgs::PoseStamped& currentPose);

    // 控制算法实现（修改为输出转向角度）
    ControlOutput purePursuitControl(const geometry_msgs::PoseStamped& currentPose,
                                     const geometry_msgs::PoseStamped& targetPoint);
    ControlOutput stanleyControl(const geometry_msgs::PoseStamped& currentPose,
                                 const geometry_msgs::PoseStamped& targetPoint);
    ControlOutput directSteeringControl(const geometry_msgs::PoseStamped& currentPose,
                                        const geometry_msgs::PoseStamped& targetPoint);
    ControlOutput lookAheadControl(const geometry_msgs::PoseStamped& currentPose,
                                   const geometry_msgs::PoseStamped& targetPoint);

    // 转向角度适配（新增）
    ControlOutput applySteeringAdaptation(const ControlOutput& rawOutput);
    double smoothSteeringCommand(double targetSteering);
    double predictiveSteeringControl(double targetSteering);

    // 角速度转换（新增）
    double steeringToAngularVel(double steeringAngle, double linearVel) const;

    // 液压转向适配（保留但调整）
    ControlOutput applyHydraulicAdaptation(const ControlOutput& rawOutput);
    double smoothAngularCommand(double targetAngular);
    double predictiveAngularControl(double targetAngular);

    // 状态管理
    void updateTrackingState(const geometry_msgs::PoseStamped& currentPose);
    bool isGoalReached(const geometry_msgs::PoseStamped& currentPose) const;
    bool isTrackingFailed(const geometry_msgs::PoseStamped& currentPose) const;

    // ========== 工具函数 ==========
    double calculateDistance(const geometry_msgs::PoseStamped& p1,
                             const geometry_msgs::PoseStamped& p2) const;
    double calculateYaw(const geometry_msgs::PoseStamped& pose) const;
    double calculateYawError(const geometry_msgs::PoseStamped& current,
                             const geometry_msgs::PoseStamped& target) const;
    double calculateCrossTrackError(const geometry_msgs::PoseStamped& currentPose,
                                    const geometry_msgs::PoseStamped& pathPoint) const;
    double normalizeAngle(double angle) const;
    double interpolateSpeed(double crossTrackError, double headingError) const;
    double limitLinearVelocityRate(double targetVel) const;

    // 安全检查（修改为转向角度为主）
    ControlOutput applySafetyLimits(const ControlOutput& output) const;
    bool isValidOutput(const ControlOutput& output) const;

signals:
    // 状态变化信号
    void trackingStateChanged(TrackingState newState);
    void pathProgressUpdated(double progress);
    void trackingCompleted();
    void trackingFailed(const QString& reason);

    // 调试信号
    void debugInfo(const QString& info);
    void targetPointChanged(int newIndex, const geometry_msgs::PoseStamped& point);
};

#endif // GLOBALPATHNAVIGATOR_H
