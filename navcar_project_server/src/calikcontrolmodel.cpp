#include "navcar_project_server/calikcontrolmodel.h"

CalIKControlModel::CalIKControlModel(QObject *parent)
    : QObject{parent}
{

}

bool CalIKControlModel::calControlParams(double vx,double rz,int controlMode,ControlSteeringSpeed &mySteeringSpeed,ControlMotorSpeed &myMotorSpeed)
{
    if(controlMode==0) //四轮
    {
        if(rz!=0)
        {
            double R=fabs(vx/rz);
            if(R<minRadius/2)
            {
                qDebug()<<"小于最小转弯半径,使用最大转角";
                if(rz>0)
                {
                    calControlParams(vx,MAXSTEERINGANGLE,0,mySteeringSpeed,myMotorSpeed,1);
                }
                else
                {
                    calControlParams(vx,-MAXSTEERINGANGLE,0,mySteeringSpeed,myMotorSpeed,1);
                }
            }
            if(vx!=0)
            {
                double theta=atan(L*rz/(2*vx));//*180/M_PI;

                if(theta>MAXSTEERINGANGLE)
                {
                    theta=MAXSTEERINGANGLE;
                }
                if(theta<-MAXSTEERINGANGLE)
                {
                    theta=-MAXSTEERINGANGLE;
                }
                rz=tan(theta)*vx*2.0/L;
                //修改1
                mySteeringSpeed.deltaa=atan(L/((2*vx/rz)-W/2));
                mySteeringSpeed.deltab=atan(L/((2*vx/rz)+W/2));
                myMotorSpeed=calActualHWSpeed(vx,rz,controlMode,mySteeringSpeed);
                //qDebug()<<"电机控制"<<myMotorSpeed.vlf1<<myMotorSpeed.vrf1;
                return 1;
                /*
                if(calControlParams(vx,theta,0,mySteeringSpeed,myMotorSpeed,1))
                {
                    return 1;
                }
                else
                {
                    return 0;
                }*/
                /*
                mySteeringSpeed.delta=atan(L*rz/2*vx);
                //添加修改1
                if(mySteeringSpeed.delta>M_PI/6)
                {
                    mySteeringSpeed.delta=M_PI/6;
                }
                else if(mySteeringSpeed.delta<-M_PI/6)
                {
                    mySteeringSpeed.delta=-M_PI/6;
                }
                rz=tan(mySteeringSpeed.delta)*vx*2.0/L;
                //修改1
                mySteeringSpeed.deltaa=atan(L/((2*vx/rz)-W/2));
                mySteeringSpeed.deltab=atan(L/((2*vx/rz)+W/2));
                myMotorSpeed=calActualHWSpeed(vx,rz,controlMode,mySteeringSpeed);
                //qDebug()<<"电机控制"<<myMotorSpeed.vlf1<<myMotorSpeed.vrf1;
                return 1;*/
            }
            else
            {
                qDebug()<<"非原地转向模式";
            }
        }
        else
        {
            myMotorSpeed=calActualHWSpeed(vx,rz,controlMode,mySteeringSpeed);
            return 1;
        }
    }
    else if(controlMode==1) //前轮转向
    {
        if(rz!=0)
        {
            double R=fabs(vx/rz);
            if(R<minRadius)
            {
                qDebug()<<"小于最小转弯半径";
            }
            mySteeringSpeed.delta=atan(L*rz/vx);
            mySteeringSpeed.deltaa=atan(L/((vx/rz)-W/2));
            mySteeringSpeed.deltab=atan(L/((vx/rz)+W/2));
            myMotorSpeed=calActualHWSpeed(vx,rz,controlMode,mySteeringSpeed);
            return 1;
        }
        else
        {
            myMotorSpeed=calActualHWSpeed(vx,rz,controlMode,mySteeringSpeed);
            return 1;
        }
    }
    else if(controlMode==2) //原地旋转
    {
        ///
    }
    else if(controlMode==3) //斜行
    {
        qDebug()<<"应当使用theta计算方式";
    }
    qDebug()<<"未定义模式";
    return -1;
}

ControlMotorSpeed CalIKControlModel::calActualHWSpeed(double vx,double rz,int controlMode,ControlSteeringSpeed mySpeed)
{
    ControlMotorSpeed myMotorSpeed;
    if(controlMode==0) //四轮
    {
        if(rz!=0)
        {
            double R=vx/rz;
            double R0=sqrt((R-W/2)*(R-W/2)+L*L/4)*R/fabs(R);
            double R1=sqrt((R+W/2)*(R+W/2)+L*L/4)*R/fabs(R);
            myMotorSpeed.vlf1=fabs((R0-W2/2)*rz);
            myMotorSpeed.vlf2=fabs((R0+W2/2)*rz);
            myMotorSpeed.vrf1=fabs((R1-W2/2)*rz);
            myMotorSpeed.vrf2=fabs((R1+W2/2)*rz);
            myMotorSpeed.setSameData();
            if(vx<0)
            {
                myMotorSpeed.setInvData();
            }
            return myMotorSpeed;
        }
        else
        {
            myMotorSpeed.vlf1=vx;
            myMotorSpeed.vlf2=vx;
            myMotorSpeed.vrf1=vx;
            myMotorSpeed.vrf2=vx;
            myMotorSpeed.setSameData();
            if(vx<0)
            {
                //myMotorSpeed.setInvData();
            }
            return myMotorSpeed;
        }
    }
    else if(controlMode==1) //前轮转向
    {
        if(rz!=0)
        {
            double R=vx/rz;
            double R1=sqrt((sqrt(R*R-L*L/4)-W/2)*(sqrt(R*R-L*L/4)-W/2)+L*L)*R/fabs(R);
            double R2=sqrt((sqrt(R*R-L*L/4)+W/2)*(sqrt(R*R-L*L/4)+W/2)+L*L)*R/fabs(R);
            double R3=sqrt(R*R-L*L/4)*R/fabs(R)-W/2;
            double R4=sqrt(R*R-L*L/4)*R/fabs(R)+W/2;
            myMotorSpeed.vlf1=fabs((R1-W2/2)*rz);
            myMotorSpeed.vlf2=fabs((R1+W2/2)*rz);
            myMotorSpeed.vrf1=fabs((R2-W2/2)*rz);
            myMotorSpeed.vrf2=fabs((R2+W2/2)*rz);
            myMotorSpeed.vlr1=fabs((R3-W2/2)*rz);
            myMotorSpeed.vlr2=fabs((R3+W2/2)*rz);
            myMotorSpeed.vrr1=fabs((R4-W2/2)*rz);
            myMotorSpeed.vrr2=fabs((R4+W2/2)*rz);
            if(vx<0)
            {
                myMotorSpeed.setInvData();
            }
            return myMotorSpeed;
        }
        else
        {
            myMotorSpeed.vlf1=vx;
            myMotorSpeed.vlf2=vx;
            myMotorSpeed.vrf1=vx;
            myMotorSpeed.vrf2=vx;
            myMotorSpeed.setSameData();
            if(vx<0)
            {
                //myMotorSpeed.setInvData();
            }
            return myMotorSpeed;
        }
    }
    else if(controlMode==2) //原地旋转
    {
        ///
    }
    else if(controlMode==3) //遥控模式
    {
        if(rz!=0)
        {
            double R=vx/rz;
            double R0=sqrt((R-W/2)*(R-W/2)+L*L/4)*R/fabs(R);
            double R1=sqrt((R+W/2)*(R+W/2)+L*L/4)*R/fabs(R);
            myMotorSpeed.vlf1=fabs((R0-W2/2)*rz);
            myMotorSpeed.vlf2=fabs((R0+W2/2)*rz);
            myMotorSpeed.vrf1=fabs((R1-W2/2)*rz);
            myMotorSpeed.vrf2=fabs((R1+W2/2)*rz);
            myMotorSpeed.setSameData();
            if(vx<0)
            {
                myMotorSpeed.setInvData();
            }
            return myMotorSpeed;
        }
        else
        {
            myMotorSpeed.vlf1=vx;
            myMotorSpeed.vlf2=vx;
            myMotorSpeed.vrf1=vx;
            myMotorSpeed.vrf2=vx;
            myMotorSpeed.setSameData();
            if(vx<0)
            {
                myMotorSpeed.setInvData();
            }
            return myMotorSpeed;
        }
    }
    qDebug()<<"未定义行为";
    return myMotorSpeed;
}

bool CalIKControlModel::calControlParams(double vx,double theta,int controlMode,ControlSteeringSpeed &mySteeringSpeed,ControlMotorSpeed &myMotorSpeed,bool isTheta)
{
    switch (controlMode) {
    case 0:
    {
        //四轮阿卡曼
        if((theta>-M_PI/4)&&(theta<M_PI/4))
        {
            //double tmpx=0;
            if(fabs(vx)<MIN_CAL_VELX)
            {
                if(vx>0)
                {
                    //tmpx=MIN_CAL_VELX;
                    vx=MIN_CAL_VELX;
                }
                else
                {
                    //tmpx=-MIN_CAL_VELX;
                    vx=-MIN_CAL_VELX;
                }
            }
            double rz=2*vx*tan(theta)/L;
            if(calControlParams(vx,rz,controlMode,mySteeringSpeed,myMotorSpeed))
            {
                return 1;
            }
            else
            {
                return -1;
            }

            /* v1,0;
        double rz=2*vx*tan(theta);
        mySteeringSpeed.delta=atan(L*rz/2*tmpx);
        mySteeringSpeed.deltaa=atan(L/((2*tmpx/rz)-W/2));
        mySteeringSpeed.deltab=atan(L/((2*tmpx/rz)+W/2));
        myMotorSpeed=calActualHWSpeed(vx,rz,3,mySteeringSpeed);
         */

        }
        else
        {
            qDebug()<<"输入角度超限";
            return -1;
        }
        return -1;
        break;
    }
    case 1:
    {
        //两轮阿卡曼
        break;
    }
    case 2:
    {
        //原地旋转？
        break;
    }
    case 3:
    {
        //斜行 逻辑：先指向目标位置，再给速度（下位机执行）

    }
    default:
        qDebug()<<"未定义行为";
        break;
    }
    return -1;
}

void CalIKControlModel::checkControlParams(ControlSteeringSpeed &mySteeringSpeed,ControlMotorSpeed &myMotorSpeed)
{
    //检验数据范围
}


SteeringParams CalIKControlModel::convertToActualSteeringMsg(ControlSteeringSpeed mySteeringSpeed)
{
    double thetaa=mySteeringSpeed.deltaa/M_PI*180; //rad->deg
    double thetab=mySteeringSpeed.deltab/M_PI*180; //rad->deg
    if(thetaa>25)
    {
        qDebug()<<"超过最大转向角";
        thetaa=25;
    }
    if(thetab>25)
    {
        qDebug()<<"超过最大转向角";
        thetab=25;
    }
    if(thetaa<-25)
    {
        qDebug()<<"超过最大转向角";
        thetaa=-25;
    }
    if(thetab<-25)
    {
        qDebug()<<"超过最大转向角";
        thetab=-25;
    }
    SteeringParams mySteeringParams;
    //修改1，正负互换
    if(mySteeringSpeed.isInv)
    {
        mySteeringParams.left_front_steering=-thetaa/STEERING_RATION_PARAMS+128;
        mySteeringParams.right_front_steering=-thetab/STEERING_RATION_PARAMS+128;
        mySteeringParams.right_rear_steering=thetab/STEERING_RATION_PARAMS+128;
        mySteeringParams.left_rear_steering=thetaa/STEERING_RATION_PARAMS+128;
    }
    else
    {
        mySteeringParams.left_front_steering=-thetaa/STEERING_RATION_PARAMS+128;
        mySteeringParams.right_front_steering=-thetab/STEERING_RATION_PARAMS+128;
        mySteeringParams.right_rear_steering=-thetab/STEERING_RATION_PARAMS+128;
        mySteeringParams.left_rear_steering=-thetaa/STEERING_RATION_PARAMS+128;
    }
    return mySteeringParams;
}

SteeringParams CalIKControlModel::convertToActualSteeringMsg_Mode_Spot(ControlSteeringSpeed mySteeringSpeed)
{
    double thetaa=mySteeringSpeed.deltaa/M_PI*180; //rad->deg
    double thetab=mySteeringSpeed.deltab/M_PI*180; //rad->deg
    SteeringParams mySteeringParams;
    //修改1，正负互换
    if(mySteeringSpeed.isInv)
    {
        mySteeringParams.left_front_steering=-thetaa/STEERING_SPOT_RATION_PARAMS+128;
        mySteeringParams.right_front_steering=-thetab/STEERING_SPOT_RATION_PARAMS+128;
        mySteeringParams.right_rear_steering=thetab/STEERING_SPOT_RATION_PARAMS+128;
        mySteeringParams.left_rear_steering=thetaa/STEERING_SPOT_RATION_PARAMS+128;
    }
    else
    {
        mySteeringParams.left_front_steering=-thetaa/STEERING_SPOT_RATION_PARAMS+128;
        mySteeringParams.right_front_steering=-thetab/STEERING_SPOT_RATION_PARAMS+128;
        mySteeringParams.right_rear_steering=-thetab/STEERING_SPOT_RATION_PARAMS+128;
        mySteeringParams.left_rear_steering=-thetaa/STEERING_SPOT_RATION_PARAMS+128;
    }
    return mySteeringParams;
}

MotorParams CalIKControlModel::convertToActualMotorMsg(ControlMotorSpeed myMotorSpeed)
{
    MotorParams myMotorParams;
    myMotorParams.left_front_motor1=-(myMotorSpeed.vlf1*60/wheelCircumference)*REDUCTION_RATION/MOTOR_RATION_PARAMS+128;
    myMotorParams.left_front_motor2=-(myMotorSpeed.vlf2*60/wheelCircumference)*REDUCTION_RATION/MOTOR_RATION_PARAMS+128;
    myMotorParams.right_front_motor1=-(myMotorSpeed.vrf1*60/wheelCircumference)*REDUCTION_RATION/MOTOR_RATION_PARAMS+128;
    myMotorParams.right_front_motor2=-(myMotorSpeed.vrf2*60/wheelCircumference)*REDUCTION_RATION/MOTOR_RATION_PARAMS+128;

    myMotorParams.left_rear_motor1=-(myMotorSpeed.vlr1*60/wheelCircumference)*REDUCTION_RATION/MOTOR_RATION_PARAMS+128;
    myMotorParams.left_rear_motor2=-(myMotorSpeed.vlr2*60/wheelCircumference)*REDUCTION_RATION/MOTOR_RATION_PARAMS+128;
    myMotorParams.right_rear_motor1=-(myMotorSpeed.vrr1*60/wheelCircumference)*REDUCTION_RATION/MOTOR_RATION_PARAMS+128;
    myMotorParams.right_rear_motor2=-(myMotorSpeed.vrr2*60/wheelCircumference)*REDUCTION_RATION/MOTOR_RATION_PARAMS+128;
    return myMotorParams;
}
//=========更新三段模式的相关辅助API==========
// 斜行接近模式实现
bool CalIKControlModel::calApproachParams(double targetX, double targetY, double currentX,
                                          double currentY, double currentYaw,
                                          ControlSteeringSpeed &mySteeringSpeed,
                                          ControlMotorSpeed &myMotorSpeed)
{
    // 计算到目标点的距离和方向
    double distance = calculateDistance(currentX, currentY, targetX, targetY);

    // 检查距离是否合理
    if (distance < MIN_APPROACH_DISTANCE) {
        qDebug() << "距离目标过近，停止斜行接近";
        mySteeringSpeed = ControlSteeringSpeed(); // 重置为0
        myMotorSpeed = ControlMotorSpeed();
        return false;
    }

    // 计算目标角度（从当前位置指向目标点）
    double targetAngle = calculateTargetAngle(targetX, targetY, currentX, currentY);

    // 计算相对于当前朝向的转向角
    double relativeAngle = normalizeAngle(targetAngle - currentYaw);

    // 检查转向角是否在合理范围内
    if (fabs(relativeAngle) > MAX_APPROACH_ANGLE) {
        qDebug() << "接近角度过大：" << (relativeAngle * 180.0 / M_PI) << "度";
        relativeAngle = relativeAngle > 0 ? MAX_APPROACH_ANGLE : -MAX_APPROACH_ANGLE;
    }

    // 限制转向角度到物理极限
    double limitedAngle = limitSteeringAngle(relativeAngle);

    // 设置四轮转向参数（四轮同向转向）
    mySteeringSpeed.delta = limitedAngle;
    mySteeringSpeed.deltaa = limitedAngle;
    mySteeringSpeed.deltab = limitedAngle;
    mySteeringSpeed.isInv=false;

    // 计算接近速度（距离越远速度越快，但有上限）
    double approachVel = qMin(APPROACH_SPEED, distance * 0.1); // 距离(m) * 0.1 = 速度(m/s)
    approachVel = qMax(0.05, approachVel); // 最小速度

    // 根据转向角调整速度（转向角越大速度越慢）
    double speedFactor = 1.0 - fabs(limitedAngle) / MAXSTEERINGANGLE;
    approachVel *= qMax(0.3, speedFactor); // 最低保持30%速度

    // 设置所有电机同速前进
    myMotorSpeed.vlf1 = approachVel;
    myMotorSpeed.vlf2 = approachVel;
    myMotorSpeed.vrf1 = approachVel;
    myMotorSpeed.vrf2 = approachVel;
    myMotorSpeed.setSameData(); // 前后轮相同

    qDebug() << "斜行接近：目标角度" << (targetAngle * 180.0 / M_PI)
             << "° 转向角" << (limitedAngle * 180.0 / M_PI)
             << "° 速度" << approachVel << "m/s";

    return true;
}

// 姿态调整模式实现
bool CalIKControlModel::calOrientationParams(double targetYaw, double currentYaw,
                                             ControlSteeringSpeed &mySteeringSpeed,
                                             ControlMotorSpeed &myMotorSpeed)
{
    // 计算角度误差
    double yawError = normalizeAngle(targetYaw - currentYaw);

    // 如果角度误差很小，不需要调整
    if (fabs(yawError) < 0.05) { // 约3度
        qDebug() << "角度误差很小，无需调整";
        mySteeringSpeed = ControlSteeringSpeed();
        myMotorSpeed = ControlMotorSpeed();
        return true;
    }
//=====逻辑更新，使用固定的外界圆切线,确定转角=======
    // 原地旋转：设置转向角为最大（或根据误差调整）
    /*
    double steerAngle = yawError > 0 ? MAXSTEERINGANGLE : -MAXSTEERINGANGLE;

    // 如果误差较小，使用比例控制
    if (fabs(yawError) < M_PI/4) { // 小于45度时使用比例控制
        steerAngle = yawError * 2.0; // 比例系数2.0
        steerAngle = limitSteeringAngle(steerAngle);
    }
*/

    // 设置四轮转向参数
    mySteeringSpeed.delta = rotateThtea;
    mySteeringSpeed.deltaa = -rotateThtea;
    mySteeringSpeed.deltab = rotateThtea;

    // 原地旋转速度控制
    double rotationSpeed = ORIENTATION_SPEED;

    // 根据角度误差调整旋转速度
    double speedFactor = qMin(1.0, fabs(yawError) / (M_PI/2)); // 误差越大速度越快
    rotationSpeed *= qMax(0.2, speedFactor); // 最低20%速度

    // 设置差速旋转（左右轮反向）
    if (yawError > 0) {
        // 逆时针旋转：左轮后退，右轮前进
        myMotorSpeed.vlf1 = -rotationSpeed;
        myMotorSpeed.vlf2 = -rotationSpeed;
        myMotorSpeed.vrf1 = rotationSpeed;
        myMotorSpeed.vrf2 = rotationSpeed;
    } else {
        // 顺时针旋转：左轮前进，右轮后退
        myMotorSpeed.vlf1 = rotationSpeed;
        myMotorSpeed.vlf2 = rotationSpeed;
        myMotorSpeed.vrf1 = -rotationSpeed;
        myMotorSpeed.vrf2 = -rotationSpeed;
    }
    myMotorSpeed.setSameData(); // 前后轮相同

    qDebug() << "姿态调整：角度误差" << (yawError * 180.0 / M_PI)
             //<< "° 转向角" << (steerAngle * 180.0 / M_PI)
             << "° 旋转速度" << rotationSpeed << "m/s";

    return true;
}

// ========== 辅助方法实现 ==========

double CalIKControlModel::calculateDistance(double x1, double y1, double x2, double y2) const
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

double CalIKControlModel::calculateTargetAngle(double targetX, double targetY,
                                               double currentX, double currentY) const
{
    double dx = targetX - currentX;
    double dy = targetY - currentY;
    return atan2(dy, dx);
}

double CalIKControlModel::normalizeAngle(double angle) const
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double CalIKControlModel::limitSteeringAngle(double angle) const
{
    return qBound(-MAXSTEERINGANGLE, angle, MAXSTEERINGANGLE);
}

//8.6更新，新版API
VelControlParams CalIKControlModel::calAKMControlParams_new(double vx,double rz)
{
    double theta;
    if(vx==0)
    {
        theta=atan(L*rz/(2*MIN_CAL_VELX))/M_PI*180;
    }
    else
    {
        theta=atan(L*rz/(2*vx))/M_PI*180;
    }
    VelControlParams controlParams;
    
    // ===== 倒车转向优化（仅在已确定需要倒车时适度放大）=====
    // 当倒车（vx < 0）且需要较大转向角度时，适度放大转向角度
    // 注意：此处仅优化倒车效率，不会主动触发倒车行为
    if(vx < 0 && fabs(theta) > 15.0)  // 倒车且转向角度大于15度（提高阈值）
    {
        // 转向角度放大系数：更保守，最大放大1.3倍（原1.5倍）
        double amplifyFactor = 1.0 + 0.3 * (fabs(theta) - 15.0) / (MAXSTEERINGANGLE_DEG - 15.0);
        amplifyFactor = qBound(1.0, amplifyFactor, 1.3);  // 限制在1.0-1.3倍
        theta = theta * amplifyFactor;
        qDebug() << "倒车转向优化: 放大系数=" << amplifyFactor 
                 << " 调整后角度=" << theta << "°";
    }
    
    theta=qBound(-MAXSTEERINGANGLE_DEG,theta,MAXSTEERINGANGLE_DEG);
    vx=qBound(-MAX_VEL_X,vx,MAX_VEL_X);
    controlParams.controlMode=1;
    if(vx>=0)
    {
        controlParams.velParams_direction=0;
    }
    else
    {
        controlParams.velParams_direction=1;
    }
    if(theta>=0)
    {
        controlParams.theta_direction=0;
    }
    else
    {
        controlParams.theta_direction=1;
    }
    controlParams.velParams_unsigned=fabs(vx)/VEL_RATION_PARAMS_NEW;
    controlParams.theta_unsigned=fabs(theta)/STEERING_RATION_PARAMS_NEW;
    return controlParams;
}

VelControlParams CalIKControlModel::calAKMThetaControlParams_new(double vx,double theta)
{
    VelControlParams controlParams;
    
    // ===== 倒车转向优化（仅在已确定需要倒车时适度放大）=====
    // 当倒车（vx < 0）且需要较大转向角度时，适度放大转向角度
    if(vx < 0 && fabs(theta) > 15.0)  // 倒车且转向角度大于15度（提高阈值）
    {
        // 转向角度放大系数：更保守，最大放大1.3倍
        double amplifyFactor = 1.0 + 0.3 * (fabs(theta) - 15.0) / (MAXSTEERINGANGLE_DEG - 15.0);
        amplifyFactor = qBound(1.0, amplifyFactor, 1.3);  // 限制在1.0-1.3倍
        theta = theta * amplifyFactor;
        qDebug() << "倒车转向优化(Theta模式): 放大系数=" << amplifyFactor 
                 << " 调整后角度=" << theta << "°";
    }
    
    theta=qBound(-MAXSTEERINGANGLE_DEG,theta,MAXSTEERINGANGLE_DEG);
    vx=qBound(-MAX_VEL_X,vx,MAX_VEL_X);
    controlParams.controlMode=1;
    if(vx>=0)
    {
        controlParams.velParams_direction=0;
    }
    else
    {
        controlParams.velParams_direction=1;
    }
    if(theta>=0)
    {
        controlParams.theta_direction=0;
    }
    else
    {
        controlParams.theta_direction=1;
    }
    controlParams.velParams_unsigned=fabs(vx)/VEL_RATION_PARAMS_NEW;
    controlParams.theta_unsigned=fabs(theta)/STEERING_RATION_PARAMS_NEW;
    return controlParams;
}

VelControlParams CalIKControlModel::calApproachParams_new(double targetX, double targetY, double currentX,
                                       double currentY, double currentYaw)
{
    VelControlParams controlParams;
    controlParams.controlMode=4;
    // 计算到目标点的距离和方向
    double distance = calculateDistance(currentX, currentY, targetX, targetY);

    // 检查距离是否合理
    if (distance < MIN_APPROACH_DISTANCE) {
        qDebug() << "距离目标过近，停止斜行接近";
        controlParams.setZero();
        return controlParams;
    }

    // 计算目标角度（从当前位置指向目标点）
    double targetAngle = calculateTargetAngle(targetX, targetY, currentX, currentY);

    // 计算相对于当前朝向的转向角
    double relativeAngle = normalizeAngle(targetAngle - currentYaw);

    // 检查转向角是否在合理范围内
    if (fabs(relativeAngle) > MAX_APPROACH_ANGLE) {
        qDebug() << "接近角度过大：" << (relativeAngle * 180.0 / M_PI) << "度";
        relativeAngle = relativeAngle > 0 ? MAX_APPROACH_ANGLE : -MAX_APPROACH_ANGLE;
    }

    // 限制转向角度到物理极限
    double limitedAngle = limitSteeringAngle(relativeAngle);

    // 设置转向参数
    if(limitedAngle>0)
    {
        controlParams.theta_direction=0;
    }
    else
    {
        controlParams.theta_direction=1;
    }
    controlParams.theta_unsigned=fabs(limitedAngle*180/M_PI)/STEERING_RATION_PARAMS_NEW;

    // 计算接近速度（距离越远速度越快，但有上限）
    double approachVel = qMin(APPROACH_SPEED, distance * 0.1); // 距离(m) * 0.1 = 速度(m/s)
    approachVel = qMax(0.05, approachVel); // 最小速度

    // 根据转向角调整速度（转向角越大速度越慢）
    double speedFactor = 1.0 - fabs(limitedAngle) / MAXSTEERINGANGLE;
    approachVel *= qMax(0.3, speedFactor); // 最低保持30%速度

    if(approachVel>0)
    {
        controlParams.velParams_direction=0;
    }
    else
    {
        controlParams.velParams_direction=1;
    }
    controlParams.velParams_unsigned=fabs(approachVel)/VEL_RATION_PARAMS_NEW;

    qDebug() << "斜行接近：目标角度" << (targetAngle * 180.0 / M_PI)
             << "° 转向角" << (limitedAngle * 180.0 / M_PI)
             << "° 速度" << approachVel << "m/s";

    return controlParams;
}

// 姿态调整模式：原地旋转到目标姿态
VelControlParams CalIKControlModel::calOrientationParams_new(double targetYaw, double currentYaw)
{
    VelControlParams controlParams;
    controlParams.controlMode=2;
    // 计算角度误差
    double yawError = normalizeAngle(targetYaw - currentYaw);

    // 如果角度误差很小，不需要调整
    if (fabs(yawError) < 0.05) { // 约3度
        qDebug() << "角度误差很小，无需调整";
        controlParams.setZero();
        return controlParams;
    }
    if(yawError>0)
    {
        controlParams.theta_direction=0;
    }
    else
    {
        controlParams.theta_direction=1;
    }

    // 原地旋转速度控制
    double rotationSpeed = ORIENTATION_SPEED;

    // 根据角度误差调整旋转速度
    double speedFactor = qMin(1.0, fabs(yawError) / (M_PI/2)); // 误差越大速度越快
    rotationSpeed *= qMax(0.2, speedFactor); // 最低20%速度

    // 设置差速旋转（左右轮反向）
    if (yawError > 0) {
        // 逆时针旋转：线速度设为正
        controlParams.velParams_direction=0;
    } else {
        // 顺时针旋转：设为负
        controlParams.velParams_direction=1;
    }
    controlParams.velParams_unsigned=fabs(rotationSpeed)/VEL_RATION_PARAMS_NEW;
    controlParams.theta_unsigned=fabs(ORIENTATION_SPEED*180/M_PI)/STEERING_RATION_PARAMS_NEW;

    qDebug() << "姿态调整：角度误差" << (yawError * 180.0 / M_PI)
             //<< "° 转向角" << (steerAngle * 180.0 / M_PI)
             << "° 旋转速度" << rotationSpeed << "m/s";

    return controlParams;
}
