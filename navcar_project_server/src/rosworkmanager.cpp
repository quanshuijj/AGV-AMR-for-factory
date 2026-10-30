#include "navcar_project_server/rosworkmanager.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <QThread>

Q_LOGGING_CATEGORY(rosDataLog, "rosData")

namespace {
int getCostmapValueFromGrid(const nav_msgs::OccupancyGrid& costmap, double x, double y)
{
    if (costmap.info.resolution <= 0.0 ||
        costmap.info.width == 0 ||
        costmap.info.height == 0 ||
        costmap.data.empty()) {
        return -1;
    }

    const double originX = costmap.info.origin.position.x;
    const double originY = costmap.info.origin.position.y;
    const double resolution = costmap.info.resolution;

    const int mapX = static_cast<int>((x - originX) / resolution);
    const int mapY = static_cast<int>((y - originY) / resolution);

    if (mapX < 0 || mapX >= static_cast<int>(costmap.info.width) ||
        mapY < 0 || mapY >= static_cast<int>(costmap.info.height)) {
        return -1;
    }

    const size_t index = static_cast<size_t>(mapY) * costmap.info.width + static_cast<size_t>(mapX);
    if (index >= costmap.data.size()) {
        return -1;
    }

    return costmap.data[index];
}
}

ROSWorkManager::ROSWorkManager(QObject *parent)
    : QObject{parent}
{
    // 初始化TF监听器（必须在订阅话题之前）
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    
    // 初始化主定时器
    mainTimer = new QTimer(this);
    mainTimer->setInterval(MAIN_TIMER_MS);
    mainTimer->setSingleShot(false);

    // 连接主循环
    connect(mainTimer, &QTimer::timeout, this, &ROSWorkManager::mainLoop);

    // 订阅ROS话题
    try {
        imuDataSub = n.subscribe("/myData", 1, &ROSWorkManager::imuDataCallback, this);
        velCMDSub = n.subscribe("/cmd_vel", 1, &ROSWorkManager::velCMDCallback, this);
        odomSub = n.subscribe("/global_odometry", 1, &ROSWorkManager::odomCallback, this);  // 使用map坐标系下的全局里程计（与全局路径坐标系一致）
        moveBaseGoalSub = n.subscribe("/move_base/goal", 1, &ROSWorkManager::moveBaseGoalCallback, this);  // 跟踪waypoint_route_manager发送的阶段ActionGoal
        globalPathSub = n.subscribe("/move_base_node/SBPLLatticePlanner/plan", 1, &ROSWorkManager::globalPathCallback, this);
        navModeSub = n.subscribe("/nav_mode_switch", 1, &ROSWorkManager::navModeCallback, this);
        
        // 订阅TEB的local_plan用于实时避障跟踪
        // GlobalPath模式将跟踪这个已经避开障碍物的轨迹
        tebLocalPlanSub = n.subscribe("/move_base_node/TebLocalPlannerROS/local_plan", 1, &ROSWorkManager::tebLocalPlanCallback, this);
        
        // 订阅local_costmap用于障碍物检测（保留用于调试）
        localCostmapSub = n.subscribe("/move_base_node/local_costmap/costmap", 1, &ROSWorkManager::localCostmapCallback, this);
        clearCostmapsClient = n.serviceClient<std_srvs::Empty>("/move_base_node/clear_costmaps");

        // 使用过滤后点云做直接动态障碍让行判断，避免自车/地面点干扰。
        cloudPointSub=n.subscribe("/velodyne_points_filtered",1,&ROSWorkManager::pointCloudCallback,this);
        //camFrontSub=n.subscribe("/camera_raw0",1,&ROSWorkManager::camFrontCallback,this);
        camFrontSub=n.subscribe("/image_raw",1,&ROSWorkManager::camFrontCallback,this);
        camLeftSub=n.subscribe("/camera_raw3",1,&ROSWorkManager::camLeftCallback,this);
        camRightSub=n.subscribe("/camera_raw2",1,&ROSWorkManager::camRightCallback,this);
        camRearSub=n.subscribe("/camera_raw1",1,&ROSWorkManager::camRearCallback,this);

        gpsSub=n.subscribe("",1,&ROSWorkManager::gpsDataCallback,this);

        // 初始化发布者（用于重新规划）
        goalPub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        smoothedTebLocalPlanPub = n.advertise<nav_msgs::Path>("/navcar_manager/teb_local_plan_smoothed", 1);

        qCDebug(rosDataLog) << "ROS话题订阅成功";
    } catch (const std::exception& e) {
        qCCritical(rosDataLog) << "ROS话题订阅失败:" << e.what();
    }

    // 初始化GlobalPathNavigator
    initializeGlobalPathNavigator();

    // 初始化时间戳
    lastCmdTime = QDateTime::currentMSecsSinceEpoch();
    lastVelTime = QTime::currentTime();
    lastGlobalPathTime = QTime::currentTime();
    lastGlobalPathAdoptionTime = QTime::currentTime();
    lastCostmapTime = QTime::currentTime();
    lastTebLocalPlanTime = QTime::currentTime();
    lastAvoidancePathUpdateTime = QTime::currentTime();
    obstacleDetectedTime = QTime::currentTime();

    qCInfo(rosDataLog) << "ROSWorkManager初始化完成，支持导航模式：" << getNavigationModeString();
}

ROSWorkManager::~ROSWorkManager()
{
    qCInfo(rosDataLog) << "ROSWorkManager开始析构";
    stopROSWork();

    // 确保发送停止指令
    sendStopCommand();

    // 清理GlobalPathNavigator
    if (globalPathNav) {
        delete globalPathNav;
        globalPathNav = nullptr;
    }
    
    // 清理TF监听器
    if (tfListener) {
        delete tfListener;
        tfListener = nullptr;
    }

    qCInfo(rosDataLog) << "ROSWorkManager析构完成";
}

// ========== GlobalPathNavigator初始化 ==========

void ROSWorkManager::initializeGlobalPathNavigator()
{
    // 创建GlobalPathNavigator实例
    globalPathNav = new GlobalPathNavigator(this);

    // 设置液压转向参数 - 匹配实际硬件能力
    // 平滑系数0.8（更平滑，减少转向过度），最大角加速度0.06rad/s²，延迟0.3s
    globalPathNav->setHydraulicParams(0.85, 0.06, 0.4);
    
    // 设置速度限制 - 最大线速度1.2m/s，最大角速度0.45rad/s
    globalPathNav->setMaxVelocity(1.2, 0.45);
    
    // 设置转向限制 - 最大转向角28°(0.489rad)，转向速率0.21rad/s（30°/2.5s）
    globalPathNav->setSteeringLimits(25.0 * M_PI / 180.0, 0.10);
    
    globalPathNav->setTolerances(1.0, 0.5);

    // 设置控制算法
    globalPathNav->setControlAlgorithm(GlobalPathNavigator::PURE_PURSUIT);

    // 连接GlobalPathNavigator信号
    connect(globalPathNav, &GlobalPathNavigator::trackingStateChanged,
            this, &ROSWorkManager::onGlobalPathTrackingStateChanged);
    connect(globalPathNav, &GlobalPathNavigator::pathProgressUpdated,
            this, &ROSWorkManager::onGlobalPathProgressUpdated);
    connect(globalPathNav, &GlobalPathNavigator::trackingCompleted,
            this, &ROSWorkManager::onGlobalPathCompleted);
    connect(globalPathNav, &GlobalPathNavigator::trackingFailed,
            this, &ROSWorkManager::onGlobalPathFailed);

    qCDebug(rosDataLog) << "GlobalPathNavigator初始化完成";
}

// ========== 主循环实现 ==========

void ROSWorkManager::mainLoop()
{
    try {
        // 1. 处理ROS消息
        ros::spinOnce();

        // 2. 检查ROS状态
        if (!ros::ok()) {
            qCWarning(rosDataLog) << "ROS节点异常，停止工作";
            stopROSWork();
            return;
        }

        // 3. 更新导航状态（仅在有有效目标和里程计时）
        if (hasValidTarget && hasValidOdom) {
            updateNavigationState();
        }

        // 4. 电机控制逻辑 (根据导航模式分发)
        qint64 currentTime = QDateTime::currentMSecsSinceEpoch();

        // 检查指令超时（仅对TEB模式有效，因为TEB模式依赖外部cmd_vel）
        bool shouldCheckTimeout = (currentNavMode == NAV_TEB_MODE && isMotorActive);
        if (shouldCheckTimeout && isCommandTimeout()) {
            // 超时情况：立即停止电机并发送相关信号
            qCWarning(rosDataLog) << "指令超时检测，停止时间:"
                                  << (currentTime - lastCmdTime) << "ms";

            // 立即停止
            currentVelX = 0.0;
            currentVelZ = 0.0;
            isMotorActive = false;

            // 发送停止指令
            emit SetVelCMDSignal(0.0, 0.0);

            // 发送状态信号
            emit CmdTimeoutSignal();
            emit MotorStoppedSignal();

            logMotorStatus("超时停止");
            return; // 超时后不再执行后续逻辑
        }

        // 在导航模式下，即使isMotorActive为false也应该执行控制逻辑
        // 对于TEB模式，需要等待外部cmd_vel；对于其他模式，自主计算控制指令
        bool shouldExecuteControl = false;
        if (currentNavMode == NAV_TEB_MODE) {
            // TEB模式：需要isMotorActive为true（表示收到了cmd_vel）
            shouldExecuteControl = isMotorActive;
        } else if (currentNavMode != NAV_IDLE && currentNavMode != NAV_REACHED) {
            // 其他导航模式：只要有有效目标和里程计就执行
            shouldExecuteControl = (hasValidTarget && hasValidOdom);
        } else {
            // IDLE和REACHED模式：总是执行
            shouldExecuteControl = true;
        }

        if (shouldExecuteControl) {
            // 根据导航模式选择控制策略
            switch (currentNavMode) {
            case NAV_IDLE:
                handleIdleMode();
                break;

            case NAV_TEB_MODE:
                handleTebMode();
                break;

            case NAV_GLOBAL_PLANNER_MODE:
                handleGlobalPlannerMode();
                break;

            case NAV_APPROACH_MODE:
                handleApproachMode();
                break;

            case NAV_ORIENTATION_MODE:
                handleOrientationMode();
                break;

            case NAV_REACHED:
                handleReachedMode();
                break;
            }
        }

        // 5. 更新GlobalPath状态
        updateGlobalPathStatus();

    } catch (const std::exception& e) {
        qCCritical(rosDataLog) << "主循环异常:" << e.what();
        sendStopCommand();
    }
}

// ========== 导航模式处理方法 ==========

void ROSWorkManager::handleIdleMode()
{
    // 空闲模式：检查是否有cmd_vel指令
    // 如果有有效的cmd_vel（isMotorActive=true），则转发指令而不是发送零速度
    if (isMotorActive && (std::abs(currentVelX) > MIN_VEL_THRESHOLD || 
                          std::abs(currentVelZ) > MIN_VEL_THRESHOLD)) {
        if (restoreGlobalPathIfUsable("IDLE模式收到有效cmd_vel，优先恢复全局路径")) {
            setNavigationMode(NAV_GLOBAL_PLANNER_MODE);
            qCInfo(rosDataLog) << "IDLE模式收到有效cmd_vel，优先切换到GlobalPath模式";
        } else {
            qCInfo(rosDataLog) << "IDLE模式收到有效cmd_vel，但无可用全局路径，切换到TEB模式";
            setNavigationMode(NAV_TEB_MODE);
        }
        emit SetVelCMDSignal(currentVelX, currentVelZ);
    } else {
        // 真正的空闲状态，输出零速度
        emit SetVelCMDSignal(0.0, 0.0);
    }
}

void ROSWorkManager::handleTebMode()
{
    const bool obstacleDetected = checkObstacleAhead(OBSTACLE_CHECK_DISTANCE);
    const bool pathBlocked = isGlobalPathBlockedAhead(OBSTACLE_CHECK_DISTANCE);
    const QTime now = QTime::currentTime();
    if (pathBlocked) {
        if (!pathBlockedSince.isValid()) {
            pathBlockedSince = now;
        }
    } else {
        pathBlockedSince = QTime();
    }
    const bool pathBlockedConfirmed = pathBlocked &&
        pathBlockedSince.isValid() &&
        pathBlockedSince.msecsTo(now) >= PATH_BLOCK_CONFIRM_MS;

    // TEB模式：全局路径走廊不再被阻断时，立即恢复GlobalPath跟踪。
    if (hasValidTarget &&
        hasValidOdom &&
        globalPathNav &&
        !sbplGlobalPath.poses.empty() &&
        !pathBlocked) {
        if (globalPathNav->setGlobalPath(sbplGlobalPath)) {
            hasValidGlobalPath = true;
            lastGlobalPathTime = QTime::currentTime();
            lastGlobalPathAdoptionTime = lastGlobalPathTime;
            globalPathNav->setGoalPose(targetPose);
            isAvoidingObstacle = false;
            enableGlobalPathMode(true);
            setNavigationMode(NAV_GLOBAL_PLANNER_MODE);
            qCInfo(rosDataLog) << "TEB模式检测到全局路径走廊已畅通，恢复GlobalPath跟踪";
            return;
        }
    }

    if (pathBlockedConfirmed) {
        if (isTebLocalPlanValid() && !isTebLocalPlanBlockedAhead(OBSTACLE_CHECK_DISTANCE)) {
            if (globalPathNav && globalPathNav->setGlobalPath(tebLocalPlan)) {
                isAvoidingObstacle = true;
                obstacleDetectedTime = now;
                globalPathNav->setGoalPose(targetPose);
                enableGlobalPathMode(true);
                setNavigationMode(NAV_GLOBAL_PLANNER_MODE);
                emit ObstacleDetectedSignal(true, OBSTACLE_CHECK_DISTANCE);
                qCInfo(rosDataLog) << "TEB模式检测到全局路径被阻断，TEB局部路径可通行，切换为局部路径跟踪";
                return;
            }
        }

        if (isTebLocalPlanValid() && globalPathNav && globalPathNav->setGlobalPath(tebLocalPlan)) {
            isAvoidingObstacle = true;
            obstacleDetectedTime = now;
            globalPathNav->setGoalPose(targetPose);
            enableGlobalPathMode(true);
            setNavigationMode(NAV_GLOBAL_PLANNER_MODE);
            emit ObstacleDetectedSignal(true, OBSTACLE_CHECK_DISTANCE);
            qCWarning(rosDataLog) << "TEB模式局部路径走廊可疑，改为低速谨慎跟踪TEB路径";
            return;
        }

        clearNavigationCostmaps("TEB模式检测到全局路径被阻断");
        emit SetVelCMDSignal(0.0, 0.0);
        emit ObstacleDetectedSignal(true, OBSTACLE_CHECK_DISTANCE);
        qCWarning(rosDataLog) << "TEB模式无可用局部路径，停车等待障碍物清除";
        return;
    }

    // TEB模式：直接转发TEB的指令，障碍判断交给costmap/TEB处理。
    emit SetVelCMDSignal(currentVelX, currentVelZ);
}

void ROSWorkManager::handleGlobalPlannerMode()
{
    // 调试：确认进入GlobalPath模式
    static int gpDebugCounter = 0;
    if (++gpDebugCounter % 50 == 0) {
        qCInfo(rosDataLog) << "GlobalPath模式运行中, hasCostmapData=" << hasCostmapData 
                           << ", hasValidOdom=" << hasValidOdom
                           << ", hasTebLocalPlan=" << hasTebLocalPlan
                           << ", isAvoidingObstacle=" << isAvoidingObstacle;
    }

    if (!globalPathNav || !globalPathNav->hasValidPath()) {
        const QTime now = QTime::currentTime();
        if (!lastMissingGlobalPathWarningTime.isValid() ||
            lastMissingGlobalPathWarningTime.msecsTo(now) > 1000) {
            qCWarning(rosDataLog) << "GlobalPath模式无有效跟踪路径，停车并退出GlobalPath等待重规划"
                                  << "| hasValidGlobalPath:" << hasValidGlobalPath
                                  << "| sbpl路径点数:" << sbplGlobalPath.poses.size()
                                  << "| teb路径有效:" << isTebLocalPlanValid()
                                  << "| hasValidTarget:" << hasValidTarget
                                  << "| hasValidOdom:" << hasValidOdom;
            lastMissingGlobalPathWarningTime = now;
        }

        hasValidGlobalPath = false;
        globalPathEnabled = false;
        isAvoidingObstacle = false;
        emit SetVelCMDSignal(0.0, 0.0);

        if (hasValidTarget) {
            requestGlobalReplan("GlobalPath模式无有效跟踪路径");
            setNavigationMode(NAV_TEB_MODE);
        } else {
            setNavigationMode(NAV_IDLE);
        }
        return;
    }
    
    // ========== 障碍物检测与动态让行 ==========
    // 1. 正常情况：跟踪SBPL全局路径。
    // 2. 全局路径被挡住：优先临时采纳TEB局部路径绕障。
    // 3. 只有TEB也没有可用解时，近距离障碍才保持停车。
    bool pathBlocked = isGlobalPathBlockedAhead(OBSTACLE_CHECK_DISTANCE);
    const QTime now = QTime::currentTime();
    if (pathBlocked) {
        if (!pathBlockedSince.isValid()) {
            pathBlockedSince = now;
        }
    } else {
        pathBlockedSince = QTime();
    }
    const bool pathBlockedConfirmed = pathBlocked &&
        pathBlockedSince.isValid() &&
        pathBlockedSince.msecsTo(now) >= PATH_BLOCK_CONFIRM_MS;
    const bool tebPathDiverged =
        isTebLocalPlanDivergingFromGlobalPath(0.65, OBSTACLE_CHECK_DISTANCE);
    const bool shouldUseTebBypass = pathBlockedConfirmed || tebPathDiverged;
    const bool tebPathUsable =
        isTebLocalPlanValid() &&
        !isTebLocalPlanBlockedAhead(OBSTACLE_CHECK_DISTANCE);
    const bool nearCollision = checkObstacleAhead(COLLISION_STOP_DISTANCE);
    
    if (pathBlocked && !pathBlockedConfirmed && !tebPathDiverged) {
        qCDebug(rosDataLog) << "检测到全局路径局部阻断，但尚未持续到确认阈值，优先继续跟踪全局路径";
    } else if (shouldUseTebBypass) {
        bool adoptedTebPath = false;
        if (tebPathUsable) {
            const bool shouldUpdateAvoidancePath =
                !isAvoidingObstacle ||
                lastAvoidancePathUpdateTime.msecsTo(now) > AVOIDANCE_PATH_UPDATE_INTERVAL_MS;
            if (!shouldUpdateAvoidancePath) {
                adoptedTebPath = true;
            } else if (globalPathNav->setGlobalPath(tebLocalPlan)) {
                isAvoidingObstacle = true;
                adoptedTebPath = true;
                lastAvoidancePathUpdateTime = now;
                obstacleDetectedTime = now;
                globalPathNav->setGoalPose(targetPose);
                emit ObstacleDetectedSignal(true, OBSTACLE_CHECK_DISTANCE);
                qCInfo(rosDataLog) << (tebPathDiverged
                                           ? "TEB局部路径已绕离全局路径，跟踪TEB避障路径"
                                           : "全局路径走廊被阻断，TEB局部路径可通行，临时跟踪TEB局部路径");
            }
        }

        if (!adoptedTebPath) {
            requestGlobalReplan(tebPathDiverged ? "TEB局部路径已绕离全局路径" : "全局路径走廊被阻断");
            isAvoidingObstacle = false;
            if (isTebLocalPlanValid() && globalPathNav->setGlobalPath(tebLocalPlan)) {
                isAvoidingObstacle = true;
                lastAvoidancePathUpdateTime = now;
                obstacleDetectedTime = now;
                globalPathNav->setGoalPose(targetPose);
                emit ObstacleDetectedSignal(true, OBSTACLE_CHECK_DISTANCE);
                qCWarning(rosDataLog) << "TEB局部路径走廊可疑，选择低速谨慎绕障而非直接停车";
            } else {
                emit SetVelCMDSignal(0.0, 0.0);
                emit ObstacleDetectedSignal(true, OBSTACLE_CHECK_DISTANCE);
                qCWarning(rosDataLog) << "全局路径走廊被阻断且无有效TEB局部路径，停车等待新的全局路径";
                return;
            }
        }
    } else if (!pathBlockedConfirmed && isAvoidingObstacle) {
        // 全局路径走廊恢复畅通，延迟一段时间后恢复SBPL路径
        int timeSinceObstacle = obstacleDetectedTime.msecsTo(QTime::currentTime());
        if (timeSinceObstacle > OBSTACLE_CLEAR_DELAY_MS) {
            // 恢复跟踪SBPL全局路径
            if (!sbplGlobalPath.poses.empty() && globalPathNav->setGlobalPath(sbplGlobalPath)) {
                hasValidGlobalPath = true;
                lastGlobalPathTime = QTime::currentTime();
                lastGlobalPathAdoptionTime = lastGlobalPathTime;
                isAvoidingObstacle = false;
                qCInfo(rosDataLog) << "✓ 全局路径走廊已畅通，恢复跟踪SBPL全局路径";
                emit ObstacleDetectedSignal(false, 0.0);
            }
        }
    }

    // 只要已经采纳了可通行TEB旁路，就允许继续绕行；
    // 只有没有可用旁路时，近距离障碍才保持停车。
    if (nearCollision && !(isAvoidingObstacle && tebPathUsable)) {
        clearNavigationCostmaps("近距离碰撞检测");
        isAvoidingObstacle = false;
        emit SetVelCMDSignal(0.0, 0.0);
        emit ObstacleDetectedSignal(true, COLLISION_STOP_DISTANCE);
        qCWarning(rosDataLog) << "前方障碍进入碰撞距离且暂无可用旁路，停车等待";
        return;
    }
    
    // ===== 执行路径跟踪控制（无论跟踪哪条路径）=====
    
    // 通过TF获取map坐标系下的机器人位姿
    geometry_msgs::PoseStamped currentPose;
    if (!getRobotPoseInMap(currentPose)) {
        // TF获取失败，使用里程计数据作为备用
        currentPose.pose = currentOdom.pose.pose;
        currentPose.header = currentOdom.header;
        qCWarning(rosDataLog) << "TF获取失败，使用里程计数据";
    }
    
    // 计算控制指令
    auto controlOutput = globalPathNav->calculateControl(currentPose, currentOdom.twist.twist);

    if (controlOutput.isValid) {
        // 应用转向角度限制
        geometry_msgs::Twist safeCmd;
        safeCmd.linear.x = controlOutput.linearVel;
        safeCmd.angular.z = controlOutput.angularVel;
        
        // 避障时适当降速
        if (isAvoidingObstacle) {
            safeCmd.linear.x = std::min(safeCmd.linear.x, 0.8);  // 避障时降速
            if (isTebLocalPlanBlockedAhead(OBSTACLE_CHECK_DISTANCE)) {
                safeCmd.linear.x = std::min(safeCmd.linear.x, AVOIDANCE_CAUTION_SPEED);
                safeCmd.angular.z = qBound(-0.035, safeCmd.angular.z, 0.035);
            }
        }

        safeCmd = limitSteeringAngle(safeCmd);

        emit SetVelCMDSignal(safeCmd.linear.x, safeCmd.angular.z);

        // 检查是否完成
        if (globalPathNav->isTrackingComplete()) {
            qCInfo(rosDataLog) << "GlobalPath跟踪完成";
            isAvoidingObstacle = false;
            setNavigationMode(NAV_REACHED);
        }
    } else {
        emit SetVelCMDSignal(0.0, 0.0);
        // 详细诊断信息
        auto status = globalPathNav->getPathStatus();
        qCWarning(rosDataLog) << "GlobalPath控制输出无效"
                              << "| 路径有效:" << globalPathNav->hasValidPath()
                              << "| 跟踪状态:" << status.state
                              << "| 路径点数:" << status.totalPoints
                              << "| 当前索引:" << status.currentIndex
                              << "| 状态信息:" << status.statusInfo;
    }
}

void ROSWorkManager::handleApproachMode()
{
    // 斜行接近模式：发送目标位置信息给底盘控制器
    if (hasValidOdom && hasValidTarget) {
        geometry_msgs::Point filteredPos = getFilteredPosition();
        double filteredYaw = getFilteredYaw();

        emit SetApproachCMDSignal(
            targetPose.pose.position.x,
            targetPose.pose.position.y,
            filteredPos.x,
            filteredPos.y,
            filteredYaw
            );

        qCDebug(rosDataLog) << "斜行模式: 目标(" << targetPose.pose.position.x
                            << "," << targetPose.pose.position.y
                            << ") 当前(" << filteredPos.x
                            << "," << filteredPos.y << ")";
    }
}

void ROSWorkManager::handleOrientationMode()
{
    // 姿态调整模式：只调整角度
    if (hasValidOdom && hasValidTarget) {
        double targetYaw = quaternionToYaw(targetPose.pose.orientation);
        double currentYaw = getFilteredYaw();

        emit SetOrientationCMDSignal(targetYaw, currentYaw);

        double yawError = calculateYawError();
        qCDebug(rosDataLog) << "姿态调整: 目标角度=" << (targetYaw * 180.0 / M_PI)
                            << "° 当前角度=" << (currentYaw * 180.0 / M_PI)
                            << "° 误差=" << (yawError * 180.0 / M_PI) << "°";
    }
}

void ROSWorkManager::handleReachedMode()
{
    // 导航完成，停止所有运动
    emit SetVelCMDSignal(0.0, 0.0);
    isMotorActive = false;
    emit NavigationCompleted();
    logNavigationState("导航任务完成");
}

// ========== ROS回调函数实现 ==========

void ROSWorkManager::globalPathCallback(const nav_msgs::Path& path)
{
    if (path.poses.empty()) {
        qCWarning(rosDataLog) << "收到空的全局路径";
        hasValidGlobalPath = false;
        return;
    }

    // 保存SBPL全局路径（用于避障后恢复），但不要每次回调都替换当前跟踪路径。
    sbplGlobalPath = path;
    hasValidGlobalPath = true;
    lastGlobalPathTime = QTime::currentTime();

    QTime now = QTime::currentTime();
    bool shouldAdoptPath = (globalReplanPending || !isAvoidingObstacle) &&
                           globalPathNav &&
                           (globalReplanPending ||
                            !globalPathNav->hasValidPath() ||
                            currentNavMode == NAV_IDLE ||
                            currentNavMode == NAV_REACHED ||
                            lastGlobalPathAdoptionTime.msecsTo(now) > GLOBAL_PATH_ADOPTION_INTERVAL_MS);

    if (shouldAdoptPath && globalPathNav->setGlobalPath(path)) {
        lastGlobalPathAdoptionTime = now;
        globalReplanPending = false;
        emit GlobalPathReceived(path);
        qCDebug(rosDataLog) << "采纳全局路径，点数：" << path.poses.size()
                            << "当前模式：" << getNavigationModeString()
                            << "采纳间隔(ms):" << GLOBAL_PATH_ADOPTION_INTERVAL_MS;
        
        // 如果当前是GlobalPath模式，确保导航器状态正确
        if (currentNavMode == NAV_GLOBAL_PLANNER_MODE) {
            enableGlobalPathMode(true);
        }
        // 如果当前是空闲模式，且有有效目标，自动切换到GlobalPath模式
        else if (currentNavMode == NAV_IDLE && hasValidTarget && hasValidOdom) {
            setNavigationMode(NAV_GLOBAL_PLANNER_MODE);
            if (globalPathNav && hasValidTarget) {
                globalPathNav->setGoalPose(targetPose);
            }
            enableGlobalPathMode(true);
            qCInfo(rosDataLog) << "收到全局路径，自动切换到GlobalPath跟踪模式";
        }
        else if (currentNavMode == NAV_TEB_MODE && restoreGlobalPathIfUsable("TEB模式收到可通行全局路径")) {
            switchNavigationMode(NAV_GLOBAL_PLANNER_MODE);
        }
    } else {
        qCDebug(rosDataLog) << "收到全局路径但暂不采纳，点数：" << path.poses.size()
                            << "当前模式：" << getNavigationModeString()
                            << "避障=" << isAvoidingObstacle;
    }
}

void ROSWorkManager::navModeCallback(const std_msgs::Int32& mode)
{
    NavigationMode newMode = static_cast<NavigationMode>(mode.data);

    if (newMode >= NAV_IDLE && newMode <= NAV_REACHED) {
        setNavigationMode(newMode);
        qCInfo(rosDataLog) << "通过ROS话题切换导航模式到：" << getNavigationModeString();
    } else {
        qCWarning(rosDataLog) << "无效的导航模式：" << mode.data;
    }
}

void ROSWorkManager::gpsDataCallback(const chcnav::hcinspvatzcb::ConstPtr &msg)
{
    GPSDataCount++;
    if(GPSDataCount<GPS_DOWN_SAMPLING_PARAMS) return;
    NetDataGPSData myGPSData;
    myGPSData.week=msg->week;
    myGPSData.second=msg->second;
    myGPSData.latitude=msg->latitude;
    myGPSData.longitude=msg->longitude;
    myGPSData.altitude=msg->altitude;
    myGPSData.heading2=msg->heading2;
    myGPSData.hdop=msg->hdop;
    myGPSData.pdop=msg->pdop;
    myGPSData.vdop=msg->vdop;
    myGPSData.tdop=msg->tdop;
    myGPSData.gdop=msg->gdop;
    myGPSData.status=msg->stat[1];
    myGPSData.header.dataSize=sizeof(myGPSData);
    emit NewGPSReciev(myGPSData);
    GPSDataCount=0;
}

// ========== 导航模式管理 ==========

void ROSWorkManager::setNavigationMode(NavigationMode mode)
{
    if (mode == NAV_TEB_MODE && restoreGlobalPathIfUsable("拦截TEB切换请求")) {
        if (currentNavMode != NAV_GLOBAL_PLANNER_MODE) {
            switchNavigationMode(NAV_GLOBAL_PLANNER_MODE);
        }
        return;
    }

    if (mode == currentNavMode) {
        return; // 模式相同，无需切换
    }

    /*if (!canSwitchToMode(mode)) {
        qCWarning(rosDataLog) << "无法切换到模式：" << static_cast<int>(mode)
            << "当前状态不满足条件";
        return;
    }*/

    switchNavigationMode(mode);
}

bool ROSWorkManager::canSwitchToMode(NavigationMode newMode) const
{
    switch (newMode) {
    case NAV_IDLE:
        return true; // 总是可以切换到空闲模式

    case NAV_TEB_MODE:
        return hasValidTarget; // 需要有目标

    case NAV_GLOBAL_PLANNER_MODE:
        return hasValidTarget && hasValidOdom && hasValidGlobalPath; // 需要完整状态

    case NAV_APPROACH_MODE:
        return hasValidTarget && hasValidOdom && (calculateDistanceToTarget() <= APPROACH_DISTANCE_ENTER);

    case NAV_ORIENTATION_MODE:
        return hasValidTarget && hasValidOdom && (calculateDistanceToTarget() <= REACHED_DISTANCE);

    case NAV_REACHED:
        return hasValidTarget && hasValidOdom;

    default:
        return false;
    }
}

void ROSWorkManager::switchNavigationMode(NavigationMode newMode)
{
    NavigationMode oldMode = currentNavMode;
    currentNavMode = newMode;

    // 执行模式切换逻辑
    onNavigationModeChanged(oldMode, newMode);

    // 发送信号
    emit NavigationModeChanged(newMode, getNavigationModeString());

    auto modeName = [](NavigationMode mode) {
        switch (mode) {
        case NAV_IDLE: return QStringLiteral("空闲模式");
        case NAV_TEB_MODE: return QStringLiteral("TEB导航模式");
        case NAV_GLOBAL_PLANNER_MODE: return QStringLiteral("GlobalPath跟踪模式");
        case NAV_APPROACH_MODE: return QStringLiteral("斜行接近模式");
        case NAV_ORIENTATION_MODE: return QStringLiteral("姿态调整模式");
        case NAV_REACHED: return QStringLiteral("到达目标");
        default: return QStringLiteral("未知模式");
        }
    };
    qCInfo(rosDataLog) << "导航模式切换:" << modeName(oldMode) << "->" << modeName(newMode);

    logNavigationState(QString("模式切换：%1 → %2")
                           .arg(static_cast<int>(oldMode))
                           .arg(static_cast<int>(newMode)));
}

void ROSWorkManager::onNavigationModeChanged(NavigationMode oldMode, NavigationMode newMode)
{
    // 退出旧模式的清理工作
    switch (oldMode) {
    case NAV_GLOBAL_PLANNER_MODE:
        enableGlobalPathMode(false);
        break;
    default:
        break;
    }

    // 进入新模式的初始化工作
    switch (newMode) {
    case NAV_GLOBAL_PLANNER_MODE:
        enableGlobalPathMode(true);
        break;

    case NAV_IDLE:
        sendStopCommand();
        break;

    default:
        break;
    }
}

QString ROSWorkManager::getNavigationModeString() const
{
    switch (currentNavMode) {
    case NAV_IDLE: return "空闲模式";
    case NAV_TEB_MODE: return "TEB导航模式";
    case NAV_GLOBAL_PLANNER_MODE: return "GlobalPath跟踪模式";
    case NAV_APPROACH_MODE: return "斜行接近模式";
    case NAV_ORIENTATION_MODE: return "姿态调整模式";
    case NAV_REACHED: return "到达目标";
    default: return "未知模式";
    }
}

// ========== GlobalPath管理方法 ==========

void ROSWorkManager::enableGlobalPathMode(bool enable)
{
    globalPathEnabled = enable;

    if (enable && hasValidGlobalPath && hasValidOdom) {
        // 设置目标位姿给GlobalPathNavigator
        if (hasValidTarget) {
            globalPathNav->setGoalPose(targetPose);
        }

        qCDebug(rosDataLog) << "GlobalPath模式已启用";
    } else {
        qCDebug(rosDataLog) << "GlobalPath模式已禁用";
    }
}

bool ROSWorkManager::isGlobalPathReady() const
{
    QTime currentTime = QTime::currentTime();
    int timeSinceLastPath = lastGlobalPathTime.msecsTo(currentTime);

    return hasValidGlobalPath &&
           globalPathEnabled &&
           globalPathNav &&
           globalPathNav->hasValidPath() &&
           (timeSinceLastPath < GLOBAL_PATH_TIMEOUT_MS);
}

bool ROSWorkManager::restoreGlobalPathIfUsable(const QString& reason)
{
    if (!hasValidTarget || !hasValidOdom || !globalPathNav || sbplGlobalPath.poses.empty()) {
        return false;
    }

    if (globalReplanPending) {
        return false;
    }

    if (isGlobalPathBlockedAhead(OBSTACLE_CHECK_DISTANCE)) {
        return false;
    }

    if (!globalPathNav->setGlobalPath(sbplGlobalPath)) {
        return false;
    }

    QTime now = QTime::currentTime();
    hasValidGlobalPath = true;
    globalPathEnabled = true;
    lastGlobalPathTime = now;
    lastGlobalPathAdoptionTime = now;
    isAvoidingObstacle = false;
    globalPathNav->setGoalPose(targetPose);
    enableGlobalPathMode(true);

    qCInfo(rosDataLog) << "全局路径走廊未阻断，恢复GlobalPath模式，原因:" << reason
                       << "路径点数:" << sbplGlobalPath.poses.size();
    return true;
}

bool ROSWorkManager::requestGlobalReplan(const QString& reason)
{
    if (!hasValidTarget) {
        return false;
    }

    QTime now = QTime::currentTime();
    if (lastGlobalReplanRequestTime.isValid() &&
        lastGlobalReplanRequestTime.msecsTo(now) < GLOBAL_REPLAN_REQUEST_INTERVAL_MS) {
        globalReplanPending = true;
        return false;
    }

    clearNavigationCostmaps(reason);
    goalPub.publish(targetPose);
    lastGlobalReplanRequestTime = now;
    globalReplanPending = true;
    hasValidGlobalPath = false;

    qCWarning(rosDataLog) << "触发全局重规划请求，原因:" << reason
                          << "目标=(" << targetPose.pose.position.x
                          << "," << targetPose.pose.position.y << ")";
    return true;
}

void ROSWorkManager::updateGlobalPathStatus()
{
    // 检查GlobalPath是否超时
    if (hasValidGlobalPath) {
        QTime currentTime = QTime::currentTime();
        int timeSinceLastPath = lastGlobalPathTime.msecsTo(currentTime);

        if (timeSinceLastPath > GLOBAL_PATH_TIMEOUT_MS) {
            if (!sbplGlobalPath.poses.empty() && hasValidTarget && hasValidOdom && !isGlobalPathBlockedAhead(OBSTACLE_CHECK_DISTANCE)) {
                hasValidGlobalPath = true;
                lastGlobalPathTime = currentTime;
                qCInfo(rosDataLog) << "GlobalPath超时但保存路径仍可用且前方无障碍，继续GlobalPath模式";
                return;
            }

            hasValidGlobalPath = false;
            qCWarning(rosDataLog) << "GlobalPath超时且无法复用保存路径，切换回TEB模式";
            if (currentNavMode == NAV_GLOBAL_PLANNER_MODE) {
                setNavigationMode(NAV_TEB_MODE);
            }
        }
    }
}

// ========== GlobalPathNavigator信号槽处理 ==========

void ROSWorkManager::onGlobalPathTrackingStateChanged(GlobalPathNavigator::TrackingState state)
{
    QString stateStr;
    switch (state) {
    case GlobalPathNavigator::TRACKING_IDLE:
        stateStr = "空闲";
        break;
    case GlobalPathNavigator::TRACKING_FOLLOWING:
        stateStr = "跟踪中";
        break;
    case GlobalPathNavigator::TRACKING_APPROACHING:
        stateStr = "接近目标";
        break;
    case GlobalPathNavigator::TRACKING_COMPLETED:
        stateStr = "跟踪完成";
        break;
    case GlobalPathNavigator::TRACKING_FAILED:
        stateStr = "跟踪失败";
        break;
    }

    qCDebug(rosDataLog) << "GlobalPath跟踪状态变更：" << stateStr;

    // 根据状态决定是否需要切换导航模式
    if (state == GlobalPathNavigator::TRACKING_COMPLETED) {
        setNavigationMode(NAV_REACHED);
    } else if (state == GlobalPathNavigator::TRACKING_FAILED) {
        if (globalReplanPending) {
            emit SetVelCMDSignal(0.0, 0.0);
            qCWarning(rosDataLog) << "GlobalPath跟踪失败，但系统正在等待新的全局路径，保持停车";
            return;
        }

        requestGlobalReplan("GlobalPath跟踪失败");
        emit SetVelCMDSignal(0.0, 0.0);
        qCWarning(rosDataLog) << "GlobalPath跟踪失败，停止并请求新的全局路径";
    }
}

void ROSWorkManager::onGlobalPathProgressUpdated(double progress)
{
    emit GlobalPathTrackingProgress(progress);
    qCDebug(rosDataLog) << "GlobalPath跟踪进度：" << QString::number(progress, 'f', 1) << "%";
}

void ROSWorkManager::onGlobalPathCompleted()
{
    emit GlobalPathTrackingCompleted();
    qCInfo(rosDataLog) << "GlobalPath跟踪完成";
    setNavigationMode(NAV_REACHED);
}

void ROSWorkManager::onGlobalPathFailed(const QString& reason)
{
    emit GlobalPathTrackingFailed(reason);
    qCWarning(rosDataLog) << "GlobalPath跟踪失败：" << reason;

    if (globalReplanPending) {
        emit SetVelCMDSignal(0.0, 0.0);
        qCWarning(rosDataLog) << "GlobalPath失败时正在等待新的全局路径，保持停车";
        return;
    }

    requestGlobalReplan(QString("GlobalPath失败: %1").arg(reason));
    emit SetVelCMDSignal(0.0, 0.0);
}

// ========== 原有回调函数（已更新） ==========

void ROSWorkManager::velCMDCallback(const geometry_msgs::Twist& velCMD)
{
    try {
        // 应用转向角度限制
        geometry_msgs::Twist safeCmd = limitSteeringAngle(velCMD);

        // 监控加速度
        monitorAcceleration(safeCmd.linear.x, safeCmd.angular.z);

        // 更新控制指令
        double newVelX = safeCmd.linear.x;
        double newVelZ = safeCmd.angular.z;

        // 更新时间戳
        lastCmdTime = QDateTime::currentMSecsSinceEpoch();

        // 判断是否为有效的运动指令
        bool hasValidCommand = (std::abs(newVelX) > MIN_VEL_THRESHOLD ||
                                std::abs(newVelZ) > MIN_VEL_THRESHOLD);

        if (hasValidCommand) {
            // 有效运动指令
            if (!isMotorActive) {
                // 电机从停止状态启动
                isMotorActive = true;
                emit MotorStartedSignal();
                logMotorStatus("启动");
            }

            currentVelX = newVelX;
            currentVelZ = newVelZ;

        } else {
            // 停止指令 (速度为0或接近0)
            if (isMotorActive) {
                // 电机从运行状态停止
                isMotorActive = false;
                emit MotorStoppedSignal();
                logMotorStatus("正常停止");
            }

            currentVelX = 0.0;
            currentVelZ = 0.0;
        }

        // 发送数据接收信号
        emit VelCmdReceivedSignal(currentVelX, currentVelZ);

        qCDebug(rosDataLog) << "收到VEL消息: vx=" << newVelX
                            << "vz=" << newVelZ
                            << "电机状态:" << (isMotorActive ? "运行" : "停止")
                            << "导航模式:" << getNavigationModeString();

    } catch (const std::exception& e) {
        qCCritical(rosDataLog) << "VelCMD回调异常:" << e.what();
        sendStopCommand();
    }
}

void ROSWorkManager::odomCallback(const nav_msgs::Odometry& odom)
{
    try {
        // 更新当前里程计数据
        currentOdom = odom;
        hasValidOdom = true;

        // 提取航向角
        double currentYaw = quaternionToYaw(odom.pose.pose.orientation);

        // 检测跳变
        if (!yawHistory.isEmpty()) {
            if (detectYawJump(currentYaw)) {
                ++consecutiveYawJumpCount;
                emit FilterJumpDetected("航向角", currentYaw);
                if (consecutiveYawJumpCount < 3) {
                    qCWarning(rosDataLog) << "检测到航向角跳变:" << currentYaw;
                    return; // 不更新滤波器
                }

                yawHistory.clear();
                posHistory.clear();
                timeHistory.clear();
                consecutiveYawJumpCount = 0;
                consecutivePositionJumpCount = 0;
                qCWarning(rosDataLog) << "连续检测到航向角跳变，重置滤波窗口并接受新定位";
            }

            if (detectPositionJump(odom.pose.pose.position)) {
                ++consecutivePositionJumpCount;
                emit FilterJumpDetected("位置",
                                        sqrt(pow(odom.pose.pose.position.x, 2) + pow(odom.pose.pose.position.y, 2)));
                if (consecutivePositionJumpCount < 3) {
                    qCWarning(rosDataLog) << "检测到位置跳变";
                    return; // 不更新滤波器
                }

                yawHistory.clear();
                posHistory.clear();
                timeHistory.clear();
                consecutiveYawJumpCount = 0;
                consecutivePositionJumpCount = 0;
                qCWarning(rosDataLog) << "连续检测到位置跳变，重置滤波窗口并接受新定位";
            }
        }

        consecutiveYawJumpCount = 0;
        consecutivePositionJumpCount = 0;

        // 更新滤波缓冲区
        yawHistory.enqueue(currentYaw);
        posHistory.enqueue(odom.pose.pose.position);
        timeHistory.enqueue(QDateTime::currentMSecsSinceEpoch());

        // 维护滤波窗口大小
        while (yawHistory.size() > FILTER_WINDOW) {
            yawHistory.dequeue();
            posHistory.dequeue();
            timeHistory.dequeue();
        }

        // 发送滤波后的数据
        emit FilteredDataUpdated(getFilteredYaw(), getFilteredPosition());
        emit OdomReceivedSignal(odom);

        if (hasValidTarget) {
            qCDebug(rosDataLog) << "收到里程计数据，滤波后航向角:" << getFilteredYaw()
                                << "距离目标:" << getDistanceToTarget();
        } else {
            qCDebug(rosDataLog) << "收到里程计数据，滤波后航向角:" << getFilteredYaw()
                                << "当前尚无有效导航目标";
        }

    } catch (const std::exception& e) {
        qCWarning(rosDataLog) << "里程计回调异常:" << e.what();
    }
}

void ROSWorkManager::moveBaseGoalCallback(const move_base_msgs::MoveBaseActionGoal& goal)
{
    try {
        // 设置新的导航目标
        targetPose = goal.goal.target_pose;
        hasValidTarget = true;

        // 根据当前模式设置目标
        if (currentNavMode == NAV_GLOBAL_PLANNER_MODE && globalPathNav) {
            globalPathNav->setGoalPose(targetPose);
        }

        emit NavigationTargetSet(targetPose);

        qCInfo(rosDataLog) << "收到新导航目标(ActionGoal): x=" << targetPose.pose.position.x
                           << "y=" << targetPose.pose.position.y
                           << "当前模式:" << getNavigationModeString();

        // 新阶段目标到来时，即使刚到达上一窗口目标，也要重新进入导航。
        if ((currentNavMode == NAV_IDLE || currentNavMode == NAV_REACHED) && hasValidOdom) {
            // 如果有全局路径，优先使用GlobalPath模式，否则使用TEB模式
            if (hasValidGlobalPath && globalPathNav && globalPathNav->hasValidPath()) {
                setNavigationMode(NAV_GLOBAL_PLANNER_MODE);
                if (globalPathNav) {
                    globalPathNav->setGoalPose(targetPose);
                }
                qCInfo(rosDataLog) << "自动切换到GlobalPath跟踪模式";
            } else {
                setNavigationMode(NAV_TEB_MODE);
                qCInfo(rosDataLog) << "自动切换到TEB导航模式";
            }
        }

    } catch (const std::exception& e) {
        qCWarning(rosDataLog) << "导航目标回调异常:" << e.what();
    }
}

void ROSWorkManager::simpleGoalCallback(const geometry_msgs::PoseStamped& goal)
{
    try {
        // 设置新的导航目标（直接使用PoseStamped）
        targetPose = goal;
        hasValidTarget = true;

        // 根据当前模式设置目标
        if (currentNavMode == NAV_GLOBAL_PLANNER_MODE && globalPathNav) {
            globalPathNav->setGoalPose(targetPose);
        }

        emit NavigationTargetSet(targetPose);

        qCInfo(rosDataLog) << "收到新导航目标(SimpleGoal): x=" << targetPose.pose.position.x
                           << "y=" << targetPose.pose.position.y
                           << "当前模式:" << getNavigationModeString()
                           << "hasValidOdom:" << hasValidOdom;

        // 🔧 修改：空闲模式或到达目标模式都应该自动开始新导航
        // NAV_REACHED模式表示上一次导航已完成，收到新目标时应该开始新的导航
        if (currentNavMode == NAV_IDLE || currentNavMode == NAV_REACHED) {
            // 如果有全局路径且有有效里程计，优先使用GlobalPath模式
            if (hasValidOdom && hasValidGlobalPath && globalPathNav && globalPathNav->hasValidPath()) {
                setNavigationMode(NAV_GLOBAL_PLANNER_MODE);
                if (globalPathNav) {
                    globalPathNav->setGoalPose(targetPose);
                }
                qCInfo(rosDataLog) << "自动切换到GlobalPath跟踪模式（从" 
                                   << (currentNavMode == NAV_REACHED ? "到达目标" : "空闲") << "状态）";
            } else {
                // 默认使用TEB模式（即使没有里程计，TEB模式也可以转发cmd_vel）
                setNavigationMode(NAV_TEB_MODE);
                qCInfo(rosDataLog) << "自动切换到TEB导航模式（从"
                                   << (currentNavMode == NAV_REACHED ? "到达目标" : "空闲") << "状态）";
            }
        }

    } catch (const std::exception& e) {
        qCWarning(rosDataLog) << "导航目标回调异常:" << e.what();
    }
}

void ROSWorkManager::pointCloudCallback(const sensor_msgs::PointCloud2& cloud)
{
    //发送到主线程中转处理
    pointCloudCountNum++;
    if(pointCloudCountNum<10)
    {
        return;
    }
    auto newPointCloudPtr=QSharedPointer<sensor_msgs::PointCloud2>::create(cloud);
    emit NewPointCloudRecievd(newPointCloudPtr);
    pointCloudCountNum=0;
}

void ROSWorkManager::camFrontCallback(const sensor_msgs::Image& img)
{
    //发送到主线程中转处理
    //qDebug()<<"新图片图像get";
    auto newImagePtr=QSharedPointer<sensor_msgs::Image>::create(img);
    emit NewCamFrontRecievd(newImagePtr);
}

void ROSWorkManager::camLeftCallback(const sensor_msgs::Image& img)
{
    //发送到主线程中转处理
    auto newImagePtr=QSharedPointer<sensor_msgs::Image>::create(img);
    emit NewCamFrontRecievd(newImagePtr);
}

void ROSWorkManager::camRightCallback(const sensor_msgs::Image& img)
{
    //发送到主线程中转处理
    auto newImagePtr=QSharedPointer<sensor_msgs::Image>::create(img);
    emit NewCamFrontRecievd(newImagePtr);
}

void ROSWorkManager::camRearCallback(const sensor_msgs::Image& img)
{
    //发送到主线程中转处理
    auto newImagePtr=QSharedPointer<sensor_msgs::Image>::create(img);
    emit NewCamFrontRecievd(newImagePtr);
}

void ROSWorkManager::imuDataCallback(const sensor_msgs::Imu& imuData)
{
    try {
        // IMU数据处理
        emit ImuDataReceivedSignal(imuData);

    } catch (const std::exception& e) {
        qCWarning(rosDataLog) << "IMU回调异常:" << e.what();
    }
}

// ========== 导航状态更新 ==========

void ROSWorkManager::updateNavigationState()
{
    if (!hasValidTarget || !hasValidOdom) {
        return;
    }

    double distance = calculateDistanceToTarget();
    NavigationMode suggestedMode = currentNavMode;

    // 基于距离的自动模式切换逻辑
    switch (currentNavMode) {
    case NAV_IDLE:
        // 如果当前是空闲模式，且有有效目标，自动切换到导航模式
        if (hasValidGlobalPath && globalPathNav && globalPathNav->hasValidPath() &&
            !isGlobalPathBlockedAhead(OBSTACLE_CHECK_DISTANCE)) {
            suggestedMode = NAV_GLOBAL_PLANNER_MODE;
        } else if (restoreGlobalPathIfUsable("空闲状态发现保存全局路径可通行")) {
            suggestedMode = NAV_GLOBAL_PLANNER_MODE;
        } else {
            suggestedMode = NAV_TEB_MODE;
        }
        break;

    case NAV_TEB_MODE:
        // 如果收到全局路径，优先切换到GlobalPath模式
        if (hasValidGlobalPath && globalPathNav && globalPathNav->hasValidPath() &&
            !isGlobalPathBlockedAhead(OBSTACLE_CHECK_DISTANCE)) {
            suggestedMode = NAV_GLOBAL_PLANNER_MODE;
        } else if (restoreGlobalPathIfUsable("TEB状态周期复核全局路径可通行")) {
            suggestedMode = NAV_GLOBAL_PLANNER_MODE;
        } else if (distance <= APPROACH_DISTANCE_ENTER) {
            suggestedMode = NAV_APPROACH_MODE;
        }
        break;

    case NAV_GLOBAL_PLANNER_MODE:
        if (distance <= APPROACH_DISTANCE_ENTER) {
            suggestedMode = NAV_APPROACH_MODE;
        }
        break;

    case NAV_APPROACH_MODE:
        if (distance > APPROACH_DISTANCE_EXIT) {
            // 滞回，返回之前的模式
            suggestedMode = hasValidGlobalPath ? NAV_GLOBAL_PLANNER_MODE : NAV_TEB_MODE;
        } else if (distance <= REACHED_DISTANCE) {
            suggestedMode = NAV_ORIENTATION_MODE;
        }
        break;

    case NAV_ORIENTATION_MODE:
    {
        double yawError = std::abs(calculateYawError());
        if (yawError <= ORIENTATION_TOLERANCE) {
            suggestedMode = NAV_REACHED;
        } else if (distance > REACHED_DISTANCE) {
            suggestedMode = NAV_APPROACH_MODE;
        }
    }
    break;

    case NAV_REACHED:
        // 到达状态不自动切换
        break;
    }

    // 应用建议的模式切换
    if (suggestedMode != currentNavMode) {
        setNavigationMode(suggestedMode);
    }
}

// ========== 安全检查方法实现 ==========

bool ROSWorkManager::isSteeringAngleSafe(double vx, double rz) const
{
    double steeringAngle = calculateSteeringAngle(vx, rz);
    return fabs(steeringAngle) <= MAX_STEERING_ANGLE_RAD;
}

geometry_msgs::Twist ROSWorkManager::limitSteeringAngle(const geometry_msgs::Twist& cmd) const
{
    geometry_msgs::Twist limitedCmd = cmd;

    // 当速度很低或为零时，不限制角速度（允许原地转向或低速大角度转弯）
    if (fabs(cmd.linear.x) < 0.05) {
        // 低速模式：只限制角速度的绝对值，不基于转向角度限制
        double maxLowSpeedAngular = 0.45;  // 低速时允许较大的角速度
        limitedCmd.angular.z = qBound(-maxLowSpeedAngular, cmd.angular.z, maxLowSpeedAngular);
        return limitedCmd;
    }

    if (!isSteeringAngleSafe(cmd.linear.x, cmd.angular.z)) {
        // 计算当前速度下的最大角速度
        double maxRz = calculateMaxAngularVel(fabs(cmd.linear.x));

        // 限制角速度
        if (cmd.angular.z > maxRz) {
            limitedCmd.angular.z = maxRz;
        } else if (cmd.angular.z < -maxRz) {
            limitedCmd.angular.z = -maxRz;
        }

        double originalAngle = calculateSteeringAngle(cmd.linear.x, cmd.angular.z) * 180.0 / M_PI;
        double limitedAngle = calculateSteeringAngle(limitedCmd.linear.x, limitedCmd.angular.z) * 180.0 / M_PI;

        qCWarning(rosDataLog) << "转向角限制：原始=" << QString::number(originalAngle, 'f', 1)
                              << "° 限制后=" << QString::number(limitedAngle, 'f', 1) << "°";
    }

    return limitedCmd;
}

double ROSWorkManager::calculateSteeringAngle(double vx, double rz) const
{
    if (fabs(vx) < 0.01) return 0.0;  // 避免除零
    return atan(WHEELBASE * rz / (2.0*vx));
}

double ROSWorkManager::calculateMaxAngularVel(double vx) const
{
    // 最小保证角速度，确保低速时仍能转弯
    static constexpr double MIN_ANGULAR_VEL = 0.15;
    
    if (fabs(vx) < 0.005) return MIN_ANGULAR_VEL;  // 极低速时仍允许转向
    
    double maxRz = 2.0 * vx * tan(MAX_STEERING_ANGLE_RAD) / WHEELBASE;
    
    // 确保至少有最小角速度
    return qMax(maxRz, MIN_ANGULAR_VEL);
}

void ROSWorkManager::monitorAcceleration(double currentLinear, double currentAngular)
{
    QTime currentTime = QTime::currentTime();

    if (lastVelTime.isValid()) {
        double dt = lastVelTime.msecsTo(currentTime) / 1000.0;  // 转换为秒

        if (dt > 0.001 && dt < 1.0) {  // 合理的时间间隔
            // 计算实际加速度
            double linearAccel = (currentLinear - lastLinearVel) / dt;
            double angularAccel = (currentAngular - lastAngularVel) / dt;

            // 检查是否超限
            if (fabs(linearAccel) > MAX_LINEAR_ACCEL) {
                qCWarning(rosDataLog) << "线加速度超限:" << QString::number(linearAccel, 'f', 4)
                                      << "m/s² (限制:" << MAX_LINEAR_ACCEL << ")";
            }

            if (fabs(angularAccel) > MAX_ANGULAR_ACCEL) {
                qCWarning(rosDataLog) << "角加速度超限:" << QString::number(angularAccel, 'f', 4)
                                      << "rad/s² (限制:" << MAX_ANGULAR_ACCEL << ")";
            }
        }
    }

    // 更新历史数据
    lastLinearVel = currentLinear;
    lastAngularVel = currentAngular;
    lastVelTime = currentTime;
}

// ========== 计算方法实现 ==========
bool ROSWorkManager::getRobotPoseInMap(geometry_msgs::PoseStamped& pose) const
{
    try {
        // 获取 map -> base_footprint 的变换
        geometry_msgs::TransformStamped transformStamped = 
            tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(0.1));
        
        // 填充位姿信息
        pose.header.stamp = transformStamped.header.stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation = transformStamped.transform.rotation;
        
        return true;
    } catch (tf2::TransformException& ex) {
        qCWarning(rosDataLog) << "TF变换失败:" << ex.what();
        return false;
    }
}

bool ROSWorkManager::getRobotPoseInOdom(geometry_msgs::PoseStamped& pose) const
{
    try {
        // 获取 odom -> base_footprint 的变换（用于与local_costmap坐标系匹配）
        geometry_msgs::TransformStamped transformStamped = 
            tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0), ros::Duration(0.1));
        
        // 填充位姿信息
        pose.header.stamp = transformStamped.header.stamp;
        pose.header.frame_id = "odom";
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation = transformStamped.transform.rotation;
        
        return true;
    } catch (tf2::TransformException& ex) {
        qCWarning(rosDataLog) << "TF变换(odom)失败:" << ex.what();
        return false;
    }
}

double ROSWorkManager::calculateDistanceToTarget() const
{
    if (!hasValidTarget || !hasValidOdom) {
        return std::numeric_limits<double>::max();
    }

    geometry_msgs::Point currentPos = getFilteredPosition();
    double deltaX = targetPose.pose.position.x - currentPos.x;
    double deltaY = targetPose.pose.position.y - currentPos.y;

    return sqrt(deltaX * deltaX + deltaY * deltaY);
}

double ROSWorkManager::calculateTargetYaw() const
{
    if (!hasValidTarget || !hasValidOdom) {
        return 0.0;
    }

    geometry_msgs::Point currentPos = getFilteredPosition();
    double deltaX = targetPose.pose.position.x - currentPos.x;
    double deltaY = targetPose.pose.position.y - currentPos.y;

    return atan2(deltaY, deltaX);
}

double ROSWorkManager::calculateYawError() const
{
    if (!hasValidTarget || !hasValidOdom) {
        return 0.0;
    }

    double targetYaw = quaternionToYaw(targetPose.pose.orientation);
    double currentYaw = getFilteredYaw();

    return normalizeAngle(targetYaw - currentYaw);
}

double ROSWorkManager::normalizeAngle(double angle) const
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double ROSWorkManager::quaternionToYaw(const geometry_msgs::Quaternion& q) const
{
    tf2::Quaternion tfQ(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 matrix(tfQ);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

// ========== 滤波方法实现 ==========

double ROSWorkManager::getFilteredYaw() const
{
    if (yawHistory.isEmpty()) {
        return 0.0;
    }

    if (yawHistory.size() == 1) {
        return yawHistory.first();
    }

    // 使用中值滤波
    return medianFilter(yawHistory);
}

geometry_msgs::Point ROSWorkManager::getFilteredPosition() const
{
    geometry_msgs::Point result;
    result.x = result.y = result.z = 0.0;

    if (posHistory.isEmpty()) {
        return result;
    }

    if (posHistory.size() == 1) {
        return posHistory.first();
    }

    // 使用平均滤波
    return averageFilter(posHistory);
}

bool ROSWorkManager::detectYawJump(double newYaw) const
{
    if (yawHistory.isEmpty()) {
        return false;
    }

    double lastYaw = yawHistory.last();
    double yawDiff = std::abs(normalizeAngle(newYaw - lastYaw));

    return yawDiff > MAX_YAW_JUMP;
}

bool ROSWorkManager::detectPositionJump(const geometry_msgs::Point& newPos) const
{
    if (posHistory.isEmpty()) {
        return false;
    }

    geometry_msgs::Point lastPos = posHistory.last();
    double deltaX = newPos.x - lastPos.x;
    double deltaY = newPos.y - lastPos.y;
    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    return distance > MAX_POS_JUMP;
}

double ROSWorkManager::medianFilter(const QQueue<double>& data) const
{
    if (data.isEmpty()) return 0.0;

    // 将QQueue转换为QList（兼容Qt5和Qt6）
    QList<double> sortedData;
    for (const auto& value : data) {
        sortedData.append(value);
    }
    std::sort(sortedData.begin(), sortedData.end());

    int size = sortedData.size();
    if (size % 2 == 0) {
        return (sortedData[size/2 - 1] + sortedData[size/2]) / 2.0;
    } else {
        return sortedData[size/2];
    }
}

geometry_msgs::Point ROSWorkManager::averageFilter(const QQueue<geometry_msgs::Point>& data) const
{
    geometry_msgs::Point result;
    result.x = result.y = result.z = 0.0;

    if (data.isEmpty()) return result;

    for (const auto& point : data) {
        result.x += point.x;
        result.y += point.y;
        result.z += point.z;
    }

    int size = data.size();
    result.x /= size;
    result.y /= size;
    result.z /= size;

    return result;
}

double ROSWorkManager::getDistanceToTarget() const
{
    return calculateDistanceToTarget();
}

// ========== 原有方法保持不变 ==========

void ROSWorkManager::startROSWork()
{
    if (QThread::currentThread() != thread()) {
        QTimer::singleShot(0, this, &ROSWorkManager::startROSWork);
        return;
    }

    if (!isROSWork) {
        try {
            // 检查ROS状态
            if (!ros::ok()) {
                qCCritical(rosDataLog) << "ROS未初始化，无法启动工作";
                return;
            }

            // 启动主定时器
            mainTimer->start();
            isROSWork = true;

            // 重置状态
            lastCmdTime = QDateTime::currentMSecsSinceEpoch();

            emit ROSWorkStartedSignal();
            qCInfo(rosDataLog) << "ROS工作管理器启动成功，定时器间隔:" << MAIN_TIMER_MS << "ms";

        } catch (const std::exception& e) {
            qCCritical(rosDataLog) << "启动ROS工作失败:" << e.what();
            isROSWork = false;
        }
    } else {
        qCDebug(rosDataLog) << "ROS工作管理器已在运行，忽略重复启动";
    }
}

void ROSWorkManager::stopROSWork()
{
    if (isROSWork) {
        try {
            // 停止主定时器
            if (mainTimer->isActive()) {
                mainTimer->stop();
            }

            // 发送停止指令
            sendStopCommand();

            // 更新状态
            isROSWork = false;

            emit ROSWorkStoppedSignal();
            qCInfo(rosDataLog) << "ROS工作管理器已停止";


        } catch (const std::exception& e) {
            qCCritical(rosDataLog) << "停止ROS工作异常:" << e.what();
        }
    }
}

void ROSWorkManager::sendStopCommand()
{
    try {
        // 重置控制参数
        currentVelX = 0.0;
        currentVelZ = 0.0;

        if (isMotorActive) {
            isMotorActive = false;
            emit MotorStoppedSignal();
            logMotorStatus("强制停止");
        }

        // 发送停止指令
        emit SetVelCMDSignal(0.0, 0.0);

        qCDebug(rosDataLog) << "发送停止指令完成";

    } catch (const std::exception& e) {
        qCCritical(rosDataLog) << "发送停止指令异常:" << e.what();
    }
}

bool ROSWorkManager::isCommandTimeout() const
{
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    return (currentTime - lastCmdTime) > CMD_TIMEOUT_MS;
}

void ROSWorkManager::logMotorStatus(const QString& action) const
{
    qCInfo(rosDataLog) << "电机状态变化:" << action
                       << "当前速度: vx=" << currentVelX
                       << "vz=" << currentVelZ
                       << "导航模式:" << getNavigationModeString()
                       << "时间戳:" << QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
}

void ROSWorkManager::logNavigationState(const QString& info) const
{
    qCInfo(rosDataLog) << "导航状态:" << getNavigationModeString()
        << "信息:" << info
        << "距离目标:" << QString::number(getDistanceToTarget(), 'f', 2) << "m"
        << "时间戳:" << QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
}

// ========== 导航暂停/恢复方法（用于自动/手动模式切换）==========

void ROSWorkManager::pauseNavigation()
{
    if (navigationPaused) {
        qCDebug(rosDataLog) << "导航已经处于暂停状态";
        return;
    }

    // 保存当前导航模式以便恢复
    savedNavModeBeforePause = currentNavMode;
    navigationPaused = true;

    // 发送停止指令
    sendStopCommand();

    // 将导航模式设置为空闲，但不清除目标和路径信息
    currentNavMode = NAV_IDLE;
    isMotorActive = false;

    qCInfo(rosDataLog) << "导航已暂停，保存的模式:" << static_cast<int>(savedNavModeBeforePause)
                       << "目标有效:" << hasValidTarget
                       << "路径有效:" << hasValidGlobalPath;

    emit NavigationModeChanged(NAV_IDLE, "导航暂停（手动模式）");
}

void ROSWorkManager::resumeNavigation()
{
    if (!navigationPaused) {
        qCDebug(rosDataLog) << "导航未处于暂停状态，无需恢复";
        return;
    }

    navigationPaused = false;
    isAvoidingObstacle = false;  // 重置避障状态

    qCInfo(rosDataLog) << "尝试恢复导航，保存的模式:" << static_cast<int>(savedNavModeBeforePause)
                       << "目标有效:" << hasValidTarget
                       << "路径有效:" << hasValidGlobalPath
                       << "里程计有效:" << hasValidOdom;

    // 重置超时计时器，避免立即触发超时
    lastCmdTime = QDateTime::currentMSecsSinceEpoch();

    // 根据当前状态决定恢复到哪个模式
    if (hasValidTarget && hasValidOdom) {
        // 有有效目标，检查全局路径
        if (hasValidGlobalPath && globalPathNav && globalPathNav->hasValidPath()) {
            // 全局路径有效，恢复到GlobalPath模式
            qCInfo(rosDataLog) << "恢复到GlobalPath跟踪模式";
            
            // 更新GlobalPathNavigator的当前位置
            geometry_msgs::PoseStamped currentPose;
            if (getRobotPoseInMap(currentPose)) {
                // 让GlobalPathNavigator重新计算从当前位置开始的路径跟踪
                globalPathNav->setGoalPose(targetPose);
            }
            
            setNavigationMode(NAV_GLOBAL_PLANNER_MODE);
        } else if (savedNavModeBeforePause == NAV_TEB_MODE || 
                   savedNavModeBeforePause == NAV_GLOBAL_PLANNER_MODE) {
            // 尝试恢复到TEB模式
            qCInfo(rosDataLog) << "恢复到TEB导航模式";
            setNavigationMode(NAV_TEB_MODE);
        } else if (savedNavModeBeforePause == NAV_APPROACH_MODE ||
                   savedNavModeBeforePause == NAV_ORIENTATION_MODE) {
            // 恢复到接近或姿态调整模式
            double distanceToTarget = calculateDistanceToTarget();
            if (distanceToTarget <= REACHED_DISTANCE) {
                qCInfo(rosDataLog) << "已到达目标附近，进入姿态调整模式";
                setNavigationMode(NAV_ORIENTATION_MODE);
            } else if (distanceToTarget <= APPROACH_DISTANCE_ENTER) {
                qCInfo(rosDataLog) << "恢复到斜行接近模式";
                setNavigationMode(NAV_APPROACH_MODE);
            } else {
                qCInfo(rosDataLog) << "恢复到TEB导航模式";
                setNavigationMode(NAV_TEB_MODE);
            }
        } else {
            // 默认恢复到TEB模式
            qCInfo(rosDataLog) << "恢复到默认TEB模式";
            setNavigationMode(NAV_TEB_MODE);
        }
    } else if (hasValidTarget) {
        // 有目标但没有里程计，等待里程计数据
        qCWarning(rosDataLog) << "等待里程计数据...";
        setNavigationMode(NAV_IDLE);
    } else {
        // 没有有效目标，保持空闲
        qCInfo(rosDataLog) << "无有效目标，保持空闲模式";
        setNavigationMode(NAV_IDLE);
    }

    emit NavigationModeChanged(currentNavMode, "导航恢复（自动模式）");
}

// ==================== TEB Local Plan回调与处理 ====================

nav_msgs::Path ROSWorkManager::transformPathToFrame(const nav_msgs::Path& path, const std::string& targetFrame) const
{
    if (path.poses.empty() || targetFrame.empty()) {
        return path;
    }

    const std::string sourceFrame = path.header.frame_id.empty()
        ? path.poses.front().header.frame_id
        : path.header.frame_id;
    if (sourceFrame == targetFrame) {
        nav_msgs::Path sameFramePath = path;
        sameFramePath.header.frame_id = targetFrame;
        for (auto& pose : sameFramePath.poses) {
            pose.header.frame_id = targetFrame;
        }
        return sameFramePath;
    }

    nav_msgs::Path transformed;
    transformed.header = path.header;
    transformed.header.frame_id = targetFrame;
    transformed.poses.reserve(path.poses.size());

    for (const auto& originalPose : path.poses) {
        geometry_msgs::PoseStamped pose = originalPose;
        if (pose.header.frame_id.empty()) {
            pose.header.frame_id = sourceFrame;
        }
        pose.header.stamp = ros::Time(0);

        try {
            geometry_msgs::PoseStamped transformedPose;
            tfBuffer.transform(pose, transformedPose, targetFrame, ros::Duration(0.02));
            transformed.poses.push_back(transformedPose);
        } catch (tf2::TransformException& ex) {
            qCWarning(rosDataLog) << "TEB局部路径坐标转换失败:"
                                  << QString::fromStdString(pose.header.frame_id)
                                  << "->" << QString::fromStdString(targetFrame)
                                  << ex.what();
            return nav_msgs::Path();
        }
    }

    return transformed;
}

nav_msgs::Path ROSWorkManager::densifyPath(const nav_msgs::Path& path, double spacing) const
{
    if (spacing <= 1e-6 || path.poses.size() < 2) {
        return path;
    }

    nav_msgs::Path densePath;
    densePath.header = path.header;
    densePath.poses.reserve(path.poses.size() * 3);
    densePath.poses.push_back(path.poses.front());

    for (size_t i = 1; i < path.poses.size(); ++i) {
        const auto& prev = path.poses[i - 1];
        const auto& curr = path.poses[i];
        const double x0 = prev.pose.position.x;
        const double y0 = prev.pose.position.y;
        const double x1 = curr.pose.position.x;
        const double y1 = curr.pose.position.y;
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double distance = std::hypot(dx, dy);
        const int insertCount = std::max(0, static_cast<int>(std::floor(distance / spacing)) - 1);
        const double yaw = std::atan2(dy, dx);
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, yaw);

        for (int j = 1; j <= insertCount; ++j) {
            const double ratio = static_cast<double>(j) / static_cast<double>(insertCount + 1);
            geometry_msgs::PoseStamped pose = curr;
            pose.pose.position.x = x0 + ratio * dx;
            pose.pose.position.y = y0 + ratio * dy;
            pose.pose.position.z = prev.pose.position.z + ratio * (curr.pose.position.z - prev.pose.position.z);
            pose.pose.orientation = tf2::toMsg(quat);
            densePath.poses.push_back(pose);
        }

        densePath.poses.push_back(curr);
    }

    return densePath;
}

nav_msgs::Path ROSWorkManager::smoothPath(const nav_msgs::Path& path, int window, int iterations) const
{
    if (path.poses.size() < 3 || window <= 0 || iterations <= 0) {
        return path;
    }

    nav_msgs::Path smoothed = path;
    for (int iter = 0; iter < iterations; ++iter) {
        nav_msgs::Path next = smoothed;
        for (int i = 1; i < static_cast<int>(smoothed.poses.size()) - 1; ++i) {
            double sumX = 0.0;
            double sumY = 0.0;
            double sumZ = 0.0;
            int count = 0;
            const int begin = std::max(0, i - window);
            const int end = std::min(static_cast<int>(smoothed.poses.size()) - 1, i + window);
            for (int j = begin; j <= end; ++j) {
                sumX += smoothed.poses[j].pose.position.x;
                sumY += smoothed.poses[j].pose.position.y;
                sumZ += smoothed.poses[j].pose.position.z;
                ++count;
            }
            if (count > 0) {
                next.poses[i].pose.position.x = sumX / count;
                next.poses[i].pose.position.y = sumY / count;
                next.poses[i].pose.position.z = sumZ / count;
            }
        }
        smoothed = next;
    }

    for (size_t i = 0; i < smoothed.poses.size(); ++i) {
        size_t prevIndex = (i == 0) ? i : i - 1;
        size_t nextIndex = (i + 1 < smoothed.poses.size()) ? i + 1 : i;
        const auto& prev = smoothed.poses[prevIndex].pose.position;
        const auto& next = smoothed.poses[nextIndex].pose.position;
        const double yaw = std::atan2(next.y - prev.y, next.x - prev.x);
        if (std::isfinite(yaw)) {
            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, yaw);
            smoothed.poses[i].pose.orientation = tf2::toMsg(quat);
        }
    }

    return smoothed;
}

void ROSWorkManager::tebLocalPlanCallback(const nav_msgs::Path& path)
{
    if (path.poses.empty()) {
        hasTebLocalPlan = false;
        return;
    }
    
    nav_msgs::Path processedPlan = smoothPath(
        densifyPath(path, TEB_LOCAL_PLAN_RESAMPLE_SPACING),
        TEB_LOCAL_PLAN_SMOOTH_WINDOW,
        TEB_LOCAL_PLAN_SMOOTH_ITERATIONS);
    tebLocalPlan = transformPathToFrame(processedPlan, "map");
    if (tebLocalPlan.poses.empty()) {
        hasTebLocalPlan = false;
        return;
    }

    smoothedTebLocalPlanPub.publish(tebLocalPlan);
    hasTebLocalPlan = true;
    lastTebLocalPlanTime = QTime::currentTime();
    
    // TEB local_plan只在避障状态时由handleGlobalPlannerMode使用
    // 这里只保存路径，不主动更新跟踪路径
    qCDebug(rosDataLog) << "收到TEB local_plan，原始点数：" << path.poses.size()
                        << "加密后点数：" << tebLocalPlan.poses.size();
}

bool ROSWorkManager::isTebLocalPlanValid() const
{
    if (!hasTebLocalPlan) return false;
    
    // 检查是否超时
    int elapsed = lastTebLocalPlanTime.msecsTo(QTime::currentTime());
    return elapsed < TEB_LOCAL_PLAN_TIMEOUT_MS;
}

// ==================== 障碍物检测相关方法（保留用于调试）====================

void ROSWorkManager::localCostmapCallback(const nav_msgs::OccupancyGrid& msg)
{
    QMutexLocker locker(&costmapMutex);
    // 保存最新的局部代价地图数据
    latestCostmap = msg;
    // 标记已接收到代价地图数据
    hasCostmapData = true;
    // 记录本次接收时间用于超时判断
    lastCostmapTime = QTime::currentTime();
}

int ROSWorkManager::getCostmapValue(double x, double y)
{
    QMutexLocker locker(&costmapMutex);

    if (!hasCostmapData) {
        return -1;  // 无数据
    }
    return getCostmapValueFromGrid(latestCostmap, x, y);
}

// ==================== 障碍物检测方法 ====================

bool ROSWorkManager::checkObstacleAhead(double checkDistance)
{
    nav_msgs::OccupancyGrid costmap;
    QTime costmapTime;
    {
        QMutexLocker locker(&costmapMutex);
        if (!hasCostmapData || !hasValidOdom) {
            return false;
        }

        costmap = latestCostmap;
        costmapTime = lastCostmapTime;
    }

    if (costmap.info.resolution <= 0.0) {
        return false;
    }

    // 检查costmap数据是否超时
    int costmapAge = costmapTime.msecsTo(QTime::currentTime());
    if (costmapAge > 1000) {  // costmap超过1秒未更新
        return false;
    }
    
    // 获取机器人当前位姿
    geometry_msgs::PoseStamped robotPose;
    if (!getRobotPoseInOdom(robotPose)) {
        return false;
    }
    
    double robotX = robotPose.pose.position.x;
    double robotY = robotPose.pose.position.y;
    double robotYaw = quaternionToYaw(robotPose.pose.orientation);
    
    // 在机器人前方扇形区域检测障碍物
    // 检测范围：前方checkDistance米，左右各30度
    double resolution = costmap.info.resolution;
    int numChecks = static_cast<int>(checkDistance / resolution);
    
    for (int i = 1; i <= numChecks; ++i) {
        double dist = i * resolution;
        
        // 检测前方多个角度
        for (double angleOffset = -0.5; angleOffset <= 0.5; angleOffset += 0.1) {  // -30°到+30°
            double checkAngle = robotYaw + angleOffset;
            double checkX = robotX + dist * cos(checkAngle);
            double checkY = robotY + dist * sin(checkAngle);
            
            int cost = getCostmapValueFromGrid(costmap, checkX, checkY);
            
            // 检测到障碍物（代价值超过阈值）
            if (cost >= OBSTACLE_COST_THRESHOLD) {
                qCDebug(rosDataLog) << "检测到障碍物: 距离=" << dist 
                                    << "m, 角度=" << (angleOffset * 180.0 / M_PI) << "°"
                                    << "代价=" << cost;
                return true;
            }
        }
    }
    
    return false;
}

bool ROSWorkManager::isPathClear()
{
    // 检查当前跟踪路径前方一段距离是否畅通
    return !isGlobalPathBlockedAhead(OBSTACLE_SAFE_DISTANCE);
}

bool ROSWorkManager::isTebLocalPlanDivergingFromGlobalPath(double minDeviation, double checkDistance) const
{
    if (!isTebLocalPlanValid() ||
        tebLocalPlan.poses.size() < 2 ||
        sbplGlobalPath.poses.size() < 2) {
        return false;
    }

    geometry_msgs::PoseStamped robotPose;
    if (!getRobotPoseInMap(robotPose)) {
        return false;
    }

    auto toMapPoint = [this](const geometry_msgs::PoseStamped& input,
                             const std::string& pathFrame,
                             geometry_msgs::Point& output) -> bool {
        geometry_msgs::PoseStamped pose = input;
        if (pose.header.frame_id.empty()) {
            pose.header.frame_id = pathFrame.empty() ? "map" : pathFrame;
        }

        try {
            if (pose.header.frame_id != "map") {
                pose.header.stamp = ros::Time(0);
                geometry_msgs::PoseStamped mapPose;
                tfBuffer.transform(pose, mapPose, "map", ros::Duration(0.02));
                output = mapPose.pose.position;
            } else {
                output = pose.pose.position;
            }
            return true;
        } catch (tf2::TransformException& ex) {
            qCDebug(rosDataLog) << "路径点转换到map失败:" << ex.what();
            return false;
        }
    };

    QVector<geometry_msgs::Point> tebPoints;
    tebPoints.reserve(static_cast<int>(tebLocalPlan.poses.size()));
    for (const auto& pose : tebLocalPlan.poses) {
        geometry_msgs::Point point;
        if (toMapPoint(pose, tebLocalPlan.header.frame_id, point)) {
            tebPoints.append(point);
        }
    }

    QVector<geometry_msgs::Point> globalPoints;
    globalPoints.reserve(static_cast<int>(sbplGlobalPath.poses.size()));
    for (const auto& pose : sbplGlobalPath.poses) {
        geometry_msgs::Point point;
        if (toMapPoint(pose, sbplGlobalPath.header.frame_id, point)) {
            globalPoints.append(point);
        }
    }

    if (tebPoints.size() < 2 || globalPoints.size() < 2) {
        return false;
    }

    int nearestTebIndex = -1;
    double nearestTebDistance = std::numeric_limits<double>::max();
    for (int i = 0; i < tebPoints.size(); ++i) {
        const double dx = tebPoints[i].x - robotPose.pose.position.x;
        const double dy = tebPoints[i].y - robotPose.pose.position.y;
        const double distance = std::hypot(dx, dy);
        if (distance < nearestTebDistance) {
            nearestTebDistance = distance;
            nearestTebIndex = i;
        }
    }

    if (nearestTebIndex < 0) {
        return false;
    }

    double accumulated = 0.0;
    double maxDeviation = 0.0;
    int checkedPoints = 0;

    for (int i = nearestTebIndex; i < tebPoints.size(); ++i) {
        if (i > nearestTebIndex) {
            accumulated += std::hypot(tebPoints[i].x - tebPoints[i - 1].x,
                                      tebPoints[i].y - tebPoints[i - 1].y);
        }
        if (accumulated > checkDistance) {
            break;
        }

        double nearestGlobalDistance = std::numeric_limits<double>::max();
        for (const auto& globalPoint : globalPoints) {
            const double distance = std::hypot(tebPoints[i].x - globalPoint.x,
                                               tebPoints[i].y - globalPoint.y);
            nearestGlobalDistance = std::min(nearestGlobalDistance, distance);
        }

        maxDeviation = std::max(maxDeviation, nearestGlobalDistance);
        ++checkedPoints;
    }

    const bool diverging = checkedPoints >= 3 && maxDeviation >= minDeviation;
    if (diverging) {
        qCDebug(rosDataLog) << "TEB局部路径相对全局路径出现绕行偏移，最大偏移="
                            << maxDeviation << "m";
    }
    return diverging;
}

bool ROSWorkManager::isPathBlockedAhead(const nav_msgs::Path& path, double checkDistance, const QString& pathName)
{
    nav_msgs::OccupancyGrid costmap;
    QTime costmapTime;
    {
        QMutexLocker locker(&costmapMutex);
        if (!hasCostmapData || !hasValidOdom || path.poses.empty()) {
            return false;
        }

        costmap = latestCostmap;
        costmapTime = lastCostmapTime;
    }

    if (costmap.info.resolution <= 0.0 ||
        costmap.info.width == 0 ||
        costmap.info.height == 0 ||
        costmap.data.empty()) {
        return false;
    }

    if (costmapTime.msecsTo(QTime::currentTime()) > 1000) {
        return false;
    }

    geometry_msgs::PoseStamped robotPose;
    if (!getRobotPoseInOdom(robotPose)) {
        return false;
    }

    const std::string costmapFrame = costmap.header.frame_id.empty()
        ? "odom"
        : costmap.header.frame_id;
    try {
        if (robotPose.header.frame_id != costmapFrame) {
            robotPose.header.stamp = ros::Time(0);
            tfBuffer.transform(robotPose, robotPose, costmapFrame, ros::Duration(0.02));
        }
    } catch (tf2::TransformException& ex) {
        qCDebug(rosDataLog) << "机器人位姿转换到costmap失败:" << ex.what();
        return false;
    }

    double nearestDist = std::numeric_limits<double>::max();
    int nearestIndex = -1;
    QVector<geometry_msgs::PoseStamped> pathInCostmap;
    pathInCostmap.reserve(static_cast<int>(path.poses.size()));

    for (size_t i = 0; i < path.poses.size(); ++i) {
        geometry_msgs::PoseStamped pose = path.poses[i];
        if (pose.header.frame_id.empty()) {
            pose.header.frame_id = path.header.frame_id;
        }
        try {
            if (!pose.header.frame_id.empty() && pose.header.frame_id != costmapFrame) {
                pose.header.stamp = ros::Time(0);
                tfBuffer.transform(pose, pose, costmapFrame, ros::Duration(0.02));
            }
        } catch (tf2::TransformException& ex) {
            qCDebug(rosDataLog) << pathName << "路径点转换到costmap失败:" << ex.what();
            continue;
        }

        pathInCostmap.append(pose);
        const double dx = pose.pose.position.x - robotPose.pose.position.x;
        const double dy = pose.pose.position.y - robotPose.pose.position.y;
        const double dist = std::hypot(dx, dy);
        if (dist < nearestDist) {
            nearestDist = dist;
            nearestIndex = pathInCostmap.size() - 1;
        }
    }

    if (nearestIndex < 0) {
        return false;
    }

    double accumulated = 0.0;
    constexpr double corridorHalfWidth = 1.4;
    constexpr double lateralStep = 0.35;

    for (int i = nearestIndex; i < pathInCostmap.size(); ++i) {
        if (i > nearestIndex) {
            const auto& prev = pathInCostmap[i - 1].pose.position;
            const auto& curr = pathInCostmap[i].pose.position;
            accumulated += std::hypot(curr.x - prev.x, curr.y - prev.y);
        }
        if (accumulated > checkDistance) {
            break;
        }

        const auto& curr = pathInCostmap[i].pose.position;
        const auto& prev = pathInCostmap[qMax(nearestIndex, i - 1)].pose.position;
        const auto& next = pathInCostmap[qMin(i + 1, pathInCostmap.size() - 1)].pose.position;
        double yaw = std::atan2(next.y - prev.y, next.x - prev.x);
        if (!std::isfinite(yaw)) {
            yaw = quaternionToYaw(pathInCostmap[i].pose.orientation);
        }

        const double normalX = -std::sin(yaw);
        const double normalY = std::cos(yaw);
        for (double offset = -corridorHalfWidth; offset <= corridorHalfWidth; offset += lateralStep) {
            const double x = curr.x + normalX * offset;
            const double y = curr.y + normalY * offset;
            const int cost = getCostmapValueFromGrid(costmap, x, y);
            if (cost >= OBSTACLE_COST_THRESHOLD) {
                qCDebug(rosDataLog) << pathName << "路径走廊被障碍阻断: 前方距离=" << accumulated
                                    << "m, 横向偏移=" << offset
                                    << "m, cost=" << cost;
                return true;
            }
        }
    }

    return false;
}

bool ROSWorkManager::isGlobalPathBlockedAhead(double checkDistance)
{
    return isPathBlockedAhead(sbplGlobalPath, checkDistance, "全局");
}

bool ROSWorkManager::isTebLocalPlanBlockedAhead(double checkDistance)
{
    if (!isTebLocalPlanValid()) {
        return true;
    }
    return isPathBlockedAhead(tebLocalPlan, checkDistance, "TEB局部");
}

bool ROSWorkManager::clearNavigationCostmaps(const QString& reason)
{
    QTime now = QTime::currentTime();
    if (lastCostmapClearTime.isValid() &&
        lastCostmapClearTime.msecsTo(now) < COSTMAP_CLEAR_INTERVAL_MS) {
        return false;
    }

    if (!clearCostmapsClient.exists()) {
        qCWarning(rosDataLog) << "clear_costmaps服务不可用，无法清理动态障碍残留，原因:" << reason;
        lastCostmapClearTime = now;
        return false;
    }

    std_srvs::Empty srv;
    if (!clearCostmapsClient.call(srv)) {
        qCWarning(rosDataLog) << "调用clear_costmaps失败，原因:" << reason;
        lastCostmapClearTime = now;
        return false;
    }

    lastCostmapClearTime = now;
    hasCostmapData = false;
    qCInfo(rosDataLog) << "已主动清理move_base代价地图，等待传感器重新标记，原因:" << reason;
    return true;
}
