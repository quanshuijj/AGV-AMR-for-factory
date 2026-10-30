#include "navcar_project_server/globalpathnavigator.h"
#include <limits>

Q_LOGGING_CATEGORY(globalPathLog, "globalPath")

GlobalPathNavigator::GlobalPathNavigator(QObject *parent)
    : QObject{parent}
{
    lastUpdateTime = QTime::currentTime();
    qCDebug(globalPathLog) << "全局路径跟踪器初始化完成，使用转向角度控制模式";
}

// ========== 公共接口实现 ==========

bool GlobalPathNavigator::setGlobalPath(const nav_msgs::Path& path)
{
    if (path.poses.empty()) {
        qCWarning(globalPathLog) << "设置的路径为空";
        return false;
    }

    const bool resetTracking = pathPoints.isEmpty() ||
                               currentState == TRACKING_IDLE ||
                               currentState == TRACKING_COMPLETED ||
                               currentState == TRACKING_FAILED;

    originalPath = path;
    processPath();

    if (!pathPoints.isEmpty()) {
        currentState = TRACKING_IDLE;
        if (resetTracking) {
            currentTargetIndex = 0;
            accumulatedDistance = 0.0;
            lastAngularCmd = 0.0;
            lastSteeringAngle = 0.0;
            lastLinearCmd = 0.0;
            lastLinearLimitTime = QTime();
        } else {
            currentTargetIndex = qBound(0, currentTargetIndex, pathPoints.size() - 1);
        }

        emit trackingStateChanged(currentState);
        qCDebug(globalPathLog) << "路径设置成功，处理后路径点数：" << pathPoints.size()
                               << "总长度：" << totalPathLength << "m";
        return true;
    }

    return false;
}

bool GlobalPathNavigator::setGoalPose(const geometry_msgs::PoseStamped& goal)
{
    goalPose = goal;
    qCDebug(globalPathLog) << "目标点设置：(" << goal.pose.position.x
                           << ", " << goal.pose.position.y << ")";
    return true;
}

void GlobalPathNavigator::clearPath()
{
    pathPoints.clear();
    originalPath.poses.clear();
    currentState = TRACKING_IDLE;
    currentTargetIndex = 0;
    totalPathLength = 0.0;
    accumulatedDistance = 0.0;
    lastAngularCmd = 0.0;
    lastSteeringAngle = 0.0;  // 重置转向角度
    lastLinearCmd = 0.0;
    lastLinearLimitTime = QTime();

    emit trackingStateChanged(currentState);
    qCDebug(globalPathLog) << "路径已清空";
}

GlobalPathNavigator::ControlOutput GlobalPathNavigator::calculateControl(
    const geometry_msgs::PoseStamped& currentPose,
    const geometry_msgs::Twist& currentVel)
{
    ControlOutput output;

    if (pathPoints.isEmpty()) {
        qCWarning(globalPathLog) << "无有效路径，无法计算控制";
        return output;
    }

    // 更新跟踪状态
    updateTrackingState(currentPose);
    updatePathProgress(currentPose);

    // 根据状态决定控制策略
    switch (currentState) {
    case TRACKING_IDLE:
        currentState = TRACKING_FOLLOWING;
        emit trackingStateChanged(currentState);
        // 继续执行FOLLOWING逻辑
        [[fallthrough]];

    case TRACKING_FOLLOWING:
    case TRACKING_APPROACHING:
    {
        // 查找目标点
        int targetIndex = findTargetWaypoint(currentPose);
        if (targetIndex >= 0 && targetIndex < pathPoints.size()) {
            geometry_msgs::PoseStamped targetPoint = pathPoints[targetIndex];

            // 根据选择的算法计算控制
            switch (controlAlgorithm) {
            case PURE_PURSUIT:
                output = purePursuitControl(currentPose, targetPoint);
                break;
            case STANLEY_CONTROLLER:
                output = stanleyControl(currentPose, targetPoint);
                break;
            case DIRECT_STEERING:
                output = directSteeringControl(currentPose, targetPoint);
                break;
            case LOOK_AHEAD:
                output = lookAheadControl(currentPose, targetPoint);
                break;
            }

            output.targetIndex = targetIndex;
        }
    }
    break;

    case TRACKING_COMPLETED:
        // 已完成，输出零速度
        output.linearVel = 0.0;
        output.angularVel = 0.0;
        output.steeringAngle = 0.0;
        output.isValid = true;
        break;

    case TRACKING_FAILED:
        // 失败，输出零速度
        output.linearVel = 0.0;
        output.angularVel = 0.0;
        output.steeringAngle = 0.0;
        output.isValid = false;
        break;
    }

    // 应用转向角度适配和安全限制
    if (output.isValid) {
        output = applySteeringAdaptation(output);
        output = applySafetyLimits(output);
    }

    // 更新历史数据
    lastPose = currentPose;
    lastUpdateTime = QTime::currentTime();

    return output;
}

GlobalPathNavigator::PathStatus GlobalPathNavigator::getPathStatus() const
{
    PathStatus status;
    status.state = currentState;
    status.progress = getPathProgress();
    status.totalLength = totalPathLength;
    status.currentIndex = currentTargetIndex;
    status.totalPoints = pathPoints.size();

    if (currentTargetIndex >= 0 && currentTargetIndex < pathPoints.size()) {
        status.remainingDistance = totalPathLength - accumulatedDistance;
    }

    // 设置状态信息
    switch (currentState) {
    case TRACKING_IDLE:
        status.statusInfo = "等待开始";
        break;
    case TRACKING_FOLLOWING:
        status.statusInfo = QString("跟踪中 %1/%2").arg(currentTargetIndex).arg(pathPoints.size());
        break;
    case TRACKING_APPROACHING:
        status.statusInfo = "接近目标";
        break;
    case TRACKING_COMPLETED:
        status.statusInfo = "跟踪完成";
        break;
    case TRACKING_FAILED:
        status.statusInfo = "跟踪失败";
        break;
    }

    return status;
}

double GlobalPathNavigator::getPathProgress() const
{
    if (totalPathLength <= 0.0) return 0.0;
    return qMin(100.0, (accumulatedDistance / totalPathLength) * 100.0);
}

// ========== 参数设置方法 ==========

void GlobalPathNavigator::setMaxVelocity(double linearVel, double angularVel)
{
    maxLinearVel = qMax(0.01, qMin(1.25, linearVel));  // 限制在CalIKControlModel范围内
    maxAngularVel = qMax(0.01, angularVel);
    qCDebug(globalPathLog) << "速度限制设置：线速度=" << maxLinearVel
                           << "角速度=" << maxAngularVel;
}

void GlobalPathNavigator::setTolerances(double waypointTol, double goalTol)
{
    waypointTolerance = qMax(0.1, waypointTol);
    goalTolerance = qMax(0.1, goalTol);
    qCDebug(globalPathLog) << "容忍度设置：路径点=" << waypointTolerance
                           << "目标点=" << goalTolerance;
}

void GlobalPathNavigator::setSteeringLimits(double maxAngle, double maxRate)
{
    // 限制在CalIKControlModel的范围内 (最大28度，对应4.5m最小转弯半径)
    maxSteeringAngle = qMax(0.1, qMin(25.0 * M_PI / 180, fabs(maxAngle)));
    minSteeringAngle = -maxSteeringAngle;
    maxSteeringRate = qMax(0.01, maxRate);

    qCDebug(globalPathLog) << "转向角度限制设置：最大角度=" << maxSteeringAngle * 180/M_PI << "度"
                           << "最大变化率=" << maxSteeringRate << "rad/s";
}

void GlobalPathNavigator::setVehicleParams(double wheelbase, double wheelRadius)
{
    wheelBase = qMax(0.5, wheelbase);
    this->wheelRadius = qMax(0.1, wheelRadius);

    qCDebug(globalPathLog) << "车辆参数设置：轴距=" << wheelBase
                           << "轮半径=" << this->wheelRadius;
}

void GlobalPathNavigator::setHydraulicParams(double smoothFactor, double maxAccel, double delay)
{
    angularSmoothFactor = qBound(0.1, smoothFactor, 0.9);
    steeringSmoothFactor = qBound(0.1, smoothFactor, 0.9);  // 同样应用于转向角度
    maxAngularAccel = qMax(0.01, maxAccel);
    steeringDelay = qMax(0.0, delay);
    qCDebug(globalPathLog) << "液压参数设置：平滑=" << angularSmoothFactor
                           << "加速度=" << maxAngularAccel << "延迟=" << steeringDelay;
}

// ========== 路径处理方法 ==========

void GlobalPathNavigator::processPath()
{
    pathPoints.clear();

    if (originalPath.poses.empty()) {
        totalPathLength = 0.0;
        return;
    }

    // 路径点过滤和优化
    pathPoints.append(originalPath.poses[0]);  // 起始点

    for (int i = 1; i < originalPath.poses.size(); ++i) {
        double dist = calculateDistance(pathPoints.last(), originalPath.poses[i]);

        // 过滤掉过近的点，保留关键转向点
        if (dist > 0.3) {  // 最小间距30cm
            pathPoints.append(originalPath.poses[i]);
        }
    }

    // 计算总路径长度
    totalPathLength = calculatePathLength();

    qCDebug(globalPathLog) << "路径处理完成：原始点数=" << originalPath.poses.size()
                           << "处理后点数=" << pathPoints.size()
                           << "总长度=" << totalPathLength << "m";
}

double GlobalPathNavigator::calculatePathLength() const
{
    double length = 0.0;
    for (int i = 1; i < pathPoints.size(); ++i) {
        length += calculateDistance(pathPoints[i-1], pathPoints[i]);
    }
    return length;
}

void GlobalPathNavigator::updatePathProgress(const geometry_msgs::PoseStamped& currentPose)
{
    if (pathPoints.isEmpty()) {
        return;
    }

    int nearestIdx = findNearestWaypointIndex(currentPose);
    if (nearestIdx < 0) {
        return;
    }

    double pathDistance = 0.0;
    for (int i = 1; i <= nearestIdx && i < pathPoints.size(); ++i) {
        pathDistance += calculateDistance(pathPoints[i - 1], pathPoints[i]);
    }

    if (nearestIdx < pathPoints.size()) {
        pathDistance += qMin(calculateDistance(pathPoints[nearestIdx], currentPose),
                             nearestIdx + 1 < pathPoints.size()
                                 ? calculateDistance(pathPoints[nearestIdx], pathPoints[nearestIdx + 1])
                                 : 0.0);
    }

    double newAccumulatedDistance = qBound(0.0, pathDistance, totalPathLength);
    if (fabs(newAccumulatedDistance - accumulatedDistance) > 0.05) {
        accumulatedDistance = newAccumulatedDistance;
        emit pathProgressUpdated(getPathProgress());

        qCDebug(globalPathLog) << "路径进度更新：" << QString::number(getPathProgress(), 'f', 1) << "%";
    }
}

// ========== 目标点查找方法 ==========

int GlobalPathNavigator::findTargetWaypoint(const geometry_msgs::PoseStamped& currentPose)
{
    if (pathPoints.isEmpty()) {
        return -1;
    }

    // 从当前目标索引开始查找
    for (int i = currentTargetIndex; i < pathPoints.size(); ++i) {
        double distance = calculateDistance(currentPose, pathPoints[i]);

        // 跳过已经经过的点
        if (distance < pathSkipDistance && i < pathPoints.size() - 1) {
            continue;
        }

        // 找到合适的前瞻点
        if (distance >= lookaheadDistance || i == pathPoints.size() - 1) {
            if (i != currentTargetIndex) {
                currentTargetIndex = i;
                emit targetPointChanged(i, pathPoints[i]);
            }
            return i;
        }
    }

    // 返回最后一个点
    currentTargetIndex = pathPoints.size() - 1;
    return currentTargetIndex;
}

int GlobalPathNavigator::findNearestWaypointIndex(const geometry_msgs::PoseStamped& currentPose) const
{
    if (pathPoints.isEmpty()) {
        return -1;
    }

    int nearestIdx = 0;
    double minDist = std::numeric_limits<double>::max();

    for (int i = 0; i < pathPoints.size(); ++i) {
        double dist = calculateDistance(currentPose, pathPoints[i]);
        if (dist < minDist) {
            minDist = dist;
            nearestIdx = i;
        }
    }

    return nearestIdx;
}

// ========== 控制算法实现 ==========

GlobalPathNavigator::ControlOutput GlobalPathNavigator::purePursuitControl(
    const geometry_msgs::PoseStamped& currentPose,
    const geometry_msgs::PoseStamped& targetPoint)
{
    ControlOutput output;
    // 计算到目标点的距离和方向
    double dx = targetPoint.pose.position.x - currentPose.pose.position.x;
    double dy = targetPoint.pose.position.y - currentPose.pose.position.y;
    double distance = sqrt(dx*dx + dy*dy);

    if (distance < waypointTolerance) {
        output.isValid = true;
        return output;  // 已接近目标点
    }

    // 当前车辆朝向
    double currentYaw = calculateYaw(currentPose);
    double targetYaw = atan2(dy, dx);
    double yawError = normalizeAngle(targetYaw - currentYaw);

    // 动态前瞻距离
    double adaptiveLookahead = qBound(minLookaheadDist,
                                      lookaheadGain * cruisingSpeed + lookaheadDistance,
                                      maxLookaheadDist);

    // 计算横向误差
    double crossTrackError = distance * sin(yawError);

    // Pure Pursuit曲率计算
    double curvature = 2.0 * crossTrackError / (adaptiveLookahead * adaptiveLookahead);

    // 转向角度计算（Ackermann转向）
    //output.steeringAngle = atan(wheelBase * curvature);
    //四轮转向模型，与前轮转向不同，修改(2026.04.17)
    output.steeringAngle = atan(wheelBase * curvature/2.0);
    
    // ===== 大角度转弯优化策略 =====
    // 优先使用正向转向，只有在极端情况（接近完全掉头）才考虑倒车
    if (fabs(yawError) > M_PI / 4) {  // 航向误差超过45度
        // 混合控制：结合Pure Pursuit和直接航向控制，使用更大的转向角
        double directSteeringAngle = yawError * 0.8;  // 增大直接控制增益：0.6→0.8
        double blendFactor = qMin(1.0, (fabs(yawError) - M_PI/4) / (M_PI/4));  // 0-1渐变
        output.steeringAngle = (1.0 - blendFactor) * output.steeringAngle + blendFactor * directSteeringAngle;
        
        // 限制转向角度到物理极限，优先正向通过弯道
        output.steeringAngle = qBound(-maxSteeringAngle, output.steeringAngle, maxSteeringAngle);
        
        qDebug() << "大角度正向转弯: 航向误差=" << yawError * 180/M_PI << "度"
                 << "混合因子=" << blendFactor
                 << "转向角=" << output.steeringAngle * 180/M_PI << "度";
        
        // ===== 极端掉头策略（仅航向误差超过165度时触发）=====
        // 只有当车辆几乎完全背对目标方向时，才使用倒车调整
        if (fabs(yawError) > 165.0 * M_PI / 180.0) {  // 165度 ≈ 2.88弧度
            // 使用最大转向角度，并将速度设为负（倒车）
            output.steeringAngle = (yawError > 0) ? -maxSteeringAngle : maxSteeringAngle;
            output.linearVel = -0.25;  // 较慢的倒车速度
            output.angularVel = steeringToAngularVel(output.steeringAngle, output.linearVel);
            
            qDebug() << "【极端掉头-倒车调整】航向误差=" << yawError * 180/M_PI << "度"
                     << "（超过165度才触发）";
            
            output.crossTrackError = crossTrackError;
            output.headingError = yawError;
            output.distanceToGoal = distance;
            output.isValid = true;
            return output;
        }
    }

    // 速度控制策略
    double speedFactor = interpolateSpeed(fabs(crossTrackError), fabs(yawError));
    output.linearVel = cruisingSpeed * speedFactor;

    // 根据转向角度和线速度计算角速度
    output.angularVel = steeringToAngularVel(output.steeringAngle, output.linearVel);

    // 记录误差信息
    output.crossTrackError = crossTrackError;
    output.headingError = yawError;
    output.distanceToGoal = distance;
    output.isValid = true;

    return output;
}

GlobalPathNavigator::ControlOutput GlobalPathNavigator::stanleyControl(
    const geometry_msgs::PoseStamped& currentPose,
    const geometry_msgs::PoseStamped& targetPoint)
{
    ControlOutput output;

    double yawError = calculateYawError(currentPose, targetPoint);
    double crossTrackError = calculateCrossTrackError(currentPose, targetPoint);
    double distance = calculateDistance(currentPose, targetPoint);

    // Stanley控制器参数
    double k = 2.0;  // 横向误差增益
    double ks = 1.0; // 软化参数

    // Stanley转向角度控制
    output.steeringAngle = yawError + atan2(k * crossTrackError, cruisingSpeed + ks);

    // 速度控制
    output.linearVel = cruisingSpeed * (1.0 - qMin(1.0, fabs(output.steeringAngle) / (M_PI/2)));

    // 计算角速度
    output.angularVel = steeringToAngularVel(output.steeringAngle, output.linearVel);

    output.crossTrackError = crossTrackError;
    output.headingError = yawError;
    output.distanceToGoal = distance;
    output.isValid = true;

    return output;
}

GlobalPathNavigator::ControlOutput GlobalPathNavigator::directSteeringControl(
    const geometry_msgs::PoseStamped& currentPose,
    const geometry_msgs::PoseStamped& targetPoint)
{
    ControlOutput output;

    double yawError = calculateYawError(currentPose, targetPoint);
    double distance = calculateDistance(currentPose, targetPoint);

    if (distance > waypointTolerance) {
        // 直接使用航向误差作为转向角度（适当缩放）
        output.steeringAngle = yawError * 0.8;  // 降低增益适应液压转向

        // 根据转向角度调整速度
        double speedFactor = 1.0 - qMin(1.0, fabs(output.steeringAngle) / (M_PI/2));
        output.linearVel = cruisingSpeed * speedFactor;

        // 计算角速度
        output.angularVel = steeringToAngularVel(output.steeringAngle, output.linearVel);
    }

    output.headingError = yawError;
    output.distanceToGoal = distance;
    output.isValid = true;

    return output;
}

GlobalPathNavigator::ControlOutput GlobalPathNavigator::lookAheadControl(
    const geometry_msgs::PoseStamped& currentPose,
    const geometry_msgs::PoseStamped& targetPoint)
{
    // 前瞻控制（类似Pure Pursuit但更激进）
    return purePursuitControl(currentPose, targetPoint);
}

// ========== 转向角度适配 ==========

GlobalPathNavigator::ControlOutput GlobalPathNavigator::applySteeringAdaptation(
    const ControlOutput& rawOutput)
{
    ControlOutput adaptedOutput = rawOutput;

    // 转向角度平滑滤波
    adaptedOutput.steeringAngle = smoothSteeringCommand(rawOutput.steeringAngle);

    // 预测性控制（补偿液压延迟）
    adaptedOutput.steeringAngle = predictiveSteeringControl(adaptedOutput.steeringAngle);

    // 重新计算角速度
    adaptedOutput.angularVel = steeringToAngularVel(adaptedOutput.steeringAngle, adaptedOutput.linearVel);

    return adaptedOutput;
}

double GlobalPathNavigator::smoothSteeringCommand(double targetSteering)
{
    // 指数移动平均滤波
    double smoothedSteering = steeringSmoothFactor * lastSteeringAngle +
                              (1.0 - steeringSmoothFactor) * targetSteering;

    // 转向角度变化率限制（增大到0.3，让转向更快响应）
    double steeringDelta = smoothedSteering - lastSteeringAngle;
    double maxDelta = maxSteeringRate * 0.3;  // 0.1→0.3，每周期允许更大变化

    if (fabs(steeringDelta) > maxDelta) {
        smoothedSteering = lastSteeringAngle + (steeringDelta > 0 ? maxDelta : -maxDelta);
    }

    lastSteeringAngle = smoothedSteering;
    return smoothedSteering;
}

double GlobalPathNavigator::predictiveSteeringControl(double targetSteering)
{
    // 预测控制：根据延迟时间提前发送命令
    double compensationFactor = 1.0 + (steeringDelay * 0.3);  // 基于延迟的补偿
    return targetSteering * compensationFactor;
}

double GlobalPathNavigator::steeringToAngularVel(double steeringAngle, double linearVel) const
{
    // Ackermann转向模型：ω = v * tan(δ) / L
    if (fabs(linearVel) < 0.01) {
        return 0.0;
    }
    return 2*linearVel * tan(steeringAngle) / wheelBase;
}

// ========== 液压转向适配（保留）==========

GlobalPathNavigator::ControlOutput GlobalPathNavigator::applyHydraulicAdaptation(
    const ControlOutput& rawOutput)
{
    ControlOutput adaptedOutput = rawOutput;

    // 角速度平滑滤波（作为备用）
    adaptedOutput.angularVel = smoothAngularCommand(rawOutput.angularVel);

    // 预测性控制（补偿液压延迟）
    adaptedOutput.angularVel = predictiveAngularControl(adaptedOutput.angularVel);

    return adaptedOutput;
}

double GlobalPathNavigator::smoothAngularCommand(double targetAngular)
{
    // 指数移动平均滤波
    double smoothedAngular = angularSmoothFactor * lastAngularCmd +
                             (1.0 - angularSmoothFactor) * targetAngular;

    // 角加速度限制
    double angularDelta = smoothedAngular - lastAngularCmd;
    double maxDelta = maxAngularAccel * 0.1;  // 假设100ms控制周期

    if (fabs(angularDelta) > maxDelta) {
        smoothedAngular = lastAngularCmd + (angularDelta > 0 ? maxDelta : -maxDelta);
    }

    lastAngularCmd = smoothedAngular;
    return smoothedAngular;
}

double GlobalPathNavigator::predictiveAngularControl(double targetAngular)
{
    // 预测控制：根据延迟时间提前发送命令
    double compensationFactor = 1.0 + (steeringDelay * 0.3);  // 基于延迟的补偿
    return targetAngular * compensationFactor;
}

// ========== 状态管理 ==========

void GlobalPathNavigator::updateTrackingState(const geometry_msgs::PoseStamped& currentPose)
{
    TrackingState newState = currentState;

    if (pathPoints.isEmpty()) {
        newState = TRACKING_IDLE;
    } else if (isGoalReached(currentPose)) {
        newState = TRACKING_COMPLETED;
    } else if (currentState == TRACKING_FAILED) {
        // 失败状态下尝试恢复：重新查找最近路径点并恢复跟踪
        int nearestIdx = findNearestWaypointIndex(currentPose);
        if (nearestIdx >= 0) {
            currentTargetIndex = nearestIdx;
            newState = TRACKING_FOLLOWING;
            qCInfo(globalPathLog) << "尝试从失败状态恢复，重新定位到路径点：" << nearestIdx;
        }
    } else if (isTrackingFailed(currentPose)) {
        newState = TRACKING_FAILED;
    } else {
        // 检查是否接近目标
        double distanceToGoal = calculateDistance(currentPose, goalPose);
        if (distanceToGoal <= finalApproachDistance) {
            newState = TRACKING_APPROACHING;
        } else if (currentState != TRACKING_FOLLOWING) {
            newState = TRACKING_FOLLOWING;
        }
    }

    if (newState != currentState) {
        currentState = newState;
        emit trackingStateChanged(currentState);

        if (currentState == TRACKING_COMPLETED) {
            emit trackingCompleted();
        } else if (currentState == TRACKING_FAILED) {
            emit trackingFailed("跟踪状态异常");
        }

        qCDebug(globalPathLog) << "跟踪状态变更：" << currentState;
    }
}

bool GlobalPathNavigator::isGoalReached(const geometry_msgs::PoseStamped& currentPose) const
{
    if (goalPose.pose.position.x == 0.0 && goalPose.pose.position.y == 0.0) {
        // 检查是否到达路径终点
        if (!pathPoints.isEmpty()) {
            double distance = calculateDistance(currentPose, pathPoints.last());
            return distance < goalTolerance;
        }
        return false;
    }

    double distance = calculateDistance(currentPose, goalPose);
    return distance < goalTolerance;
}

bool GlobalPathNavigator::isTrackingFailed(const geometry_msgs::PoseStamped& currentPose) const
{
    if (pathPoints.isEmpty()) {
        return false;
    }
    
    // 1. 检查横向误差是否过大 - 使用最近路径点
    double minDist = std::numeric_limits<double>::max();
    int nearestIdx = 0;
    for (int i = 0; i < pathPoints.size(); ++i) {
        double dist = calculateDistance(currentPose, pathPoints[i]);
        if (dist < minDist) {
            minDist = dist;
            nearestIdx = i;
        }
    }
    
    // 横向距离过大
    if (minDist > maxCrossTrackError) {
        qCWarning(globalPathLog) << "跟踪失败：最近路径点距离=" << minDist << "m 超过阈值=" << maxCrossTrackError << "m";
        return true;
    }
    
    // 2. 检查航向误差是否过大（车辆是否背对目标方向）
    if (nearestIdx < pathPoints.size() - 1) {
        // 计算路径方向（从最近点指向下一个点）
        const auto& nearestPoint = pathPoints[nearestIdx];
        const auto& nextPoint = pathPoints[qMin(nearestIdx + 1, pathPoints.size() - 1)];
        double pathYaw = atan2(nextPoint.pose.position.y - nearestPoint.pose.position.y,
                               nextPoint.pose.position.x - nearestPoint.pose.position.x);
        
        // 计算当前航向
        double currentYaw = calculateYaw(currentPose);
        
        // 计算航向误差
        double headingErr = fabs(normalizeAngle(currentYaw - pathYaw));
        
        // 航向误差超过阈值（约115度），车辆基本背对路径方向
        if (headingErr > maxHeadingError) {
            qCWarning(globalPathLog) << "跟踪失败：航向误差=" << (headingErr * 180.0 / M_PI) 
                                     << "度 超过阈值=" << (maxHeadingError * 180.0 / M_PI) << "度";
            return true;
        }
    }

    return false;
}

// ========== 工具函数 ==========

double GlobalPathNavigator::calculateDistance(const geometry_msgs::PoseStamped& p1,
                                              const geometry_msgs::PoseStamped& p2) const
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return sqrt(dx*dx + dy*dy);
}

double GlobalPathNavigator::calculateYaw(const geometry_msgs::PoseStamped& pose) const
{
    const auto& q = pose.pose.orientation;
    return atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double GlobalPathNavigator::calculateYawError(const geometry_msgs::PoseStamped& current,
                                              const geometry_msgs::PoseStamped& target) const
{
    double dx = target.pose.position.x - current.pose.position.x;
    double dy = target.pose.position.y - current.pose.position.y;
    double targetYaw = atan2(dy, dx);
    double currentYaw = calculateYaw(current);
    return normalizeAngle(targetYaw - currentYaw);
}

double GlobalPathNavigator::calculateCrossTrackError(const geometry_msgs::PoseStamped& currentPose,
                                                     const geometry_msgs::PoseStamped& pathPoint) const
{
    double dx = pathPoint.pose.position.x - currentPose.pose.position.x;
    double dy = pathPoint.pose.position.y - currentPose.pose.position.y;
    double distance = sqrt(dx*dx + dy*dy);

    double currentYaw = calculateYaw(currentPose);
    double targetYaw = atan2(dy, dx);
    double yawError = normalizeAngle(targetYaw - currentYaw);

    return distance * sin(yawError);
}

double GlobalPathNavigator::normalizeAngle(double angle) const
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double GlobalPathNavigator::interpolateSpeed(double crossTrackError, double headingError) const
{
    // 基于误差的速度插值。液压慢转向车辆需要保持一定前进速度，
    // 否则容易进入低速频繁修正。
    double crossTrackFactor = 1.0 - qMin(0.4, fabs(crossTrackError) / 5.0);
    double headingFactor = 1.0 - qMin(0.35, fabs(headingError) / M_PI);

    return qMax(0.6, crossTrackFactor * headingFactor);
}

double GlobalPathNavigator::limitLinearVelocityRate(double targetVel) const
{
    QTime now = QTime::currentTime();
    if (!lastLinearLimitTime.isValid()) {
        lastLinearLimitTime = now;
        double maxDelta = maxLinearAccel * 0.02;
        if (fabs(targetVel) > maxDelta) {
            targetVel = targetVel > 0.0 ? maxDelta : -maxDelta;
        }
        lastLinearCmd = targetVel;
        return targetVel;
    }

    double dt = lastLinearLimitTime.msecsTo(now) / 1000.0;
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.02;
    }

    double maxDelta = maxLinearAccel * dt;
    double delta = targetVel - lastLinearCmd;
    if (fabs(delta) > maxDelta) {
        targetVel = lastLinearCmd + (delta > 0.0 ? maxDelta : -maxDelta);
    }

    lastLinearCmd = targetVel;
    lastLinearLimitTime = now;
    return targetVel;
}

// ========== 安全检查 ==========

GlobalPathNavigator::ControlOutput GlobalPathNavigator::applySafetyLimits(
    const ControlOutput& output) const
{
    ControlOutput safeOutput = output;

    // 转向角度限制（主要限制）
    safeOutput.steeringAngle = qBound(minSteeringAngle, output.steeringAngle, maxSteeringAngle);
    // 速度限制
    safeOutput.linearVel = qBound(-MAX_VEL_X, output.linearVel, MAX_VEL_X);
    double steeringRatio = qMin(1.0, fabs(safeOutput.steeringAngle) / maxSteeringAngle);
    double turnSpeedLimit = maxLinearVel - steeringRatio * (maxLinearVel - turnMinLinearVel);
    if (safeOutput.linearVel >= 0.0) {
        safeOutput.linearVel = qMin(safeOutput.linearVel, turnSpeedLimit);
    } else {
        safeOutput.linearVel = qMax(safeOutput.linearVel, -turnSpeedLimit);
    }
    safeOutput.linearVel = limitLinearVelocityRate(safeOutput.linearVel);
    /*if(output.linearVel>maxLinearVel)
    {
        safeOutput.linearVel=maxLinearVel;
    }
    else if(output.linearVel<-maxLinearVel)
    {
        safeOutput.linearVel=-maxLinearVel;
    }*/
    // 重新计算角速度（基于限制后的转向角度）
    if (fabs(safeOutput.linearVel) > 0.01) {
        safeOutput.angularVel = steeringToAngularVel(safeOutput.steeringAngle, safeOutput.linearVel);
    } else {
        safeOutput.angularVel = 0.0;
    }

    // 角速度二次限制（防止极端情况）
    safeOutput.angularVel = qBound(-maxAngularVel, safeOutput.angularVel, maxAngularVel);

    // 确保最小速度
    if (fabs(safeOutput.linearVel) > 0.01 && fabs(safeOutput.linearVel) < minLinearVel) {
        safeOutput.linearVel = safeOutput.linearVel > 0 ? minLinearVel : -minLinearVel;
    }

    return safeOutput;
}

bool GlobalPathNavigator::isValidOutput(const ControlOutput& output) const
{
    return output.isValid &&
           !std::isnan(output.linearVel) && !std::isnan(output.angularVel) &&
           !std::isnan(output.steeringAngle) &&
           !std::isinf(output.linearVel) && !std::isinf(output.angularVel) &&
           !std::isinf(output.steeringAngle);
}
