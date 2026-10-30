#ifndef CALIKCONTROLMODEL_H
#define CALIKCONTROLMODEL_H

#include <QObject>
#include<QtMath>
#include<QDebug>

//====8.6新增：更改报文，使用新的控制参数=====
struct VelControlParams
{
    quint8 controlMode; //0-无模式，1-AKM，2-原地旋转，3-横行，4-斜行
    quint8 velParams_unsigned;
    quint8 velParams_direction;
    quint8 theta_unsigned;
    quint8 theta_direction;
    VelControlParams():controlMode(0),velParams_unsigned(0),velParams_direction(0),theta_unsigned(0),theta_direction(0){} //默认初始化为无效报文内容
    void setZero()
    {
        velParams_unsigned=0;
        velParams_direction=0;
        theta_unsigned=0;
        theta_direction=0;
    }
};
//=================================

struct SteeringParams
{
    quint8 left_front_steering;
    quint8 right_front_steering;
    quint8 right_rear_steering;
    quint8 left_rear_steering;
    SteeringParams():left_front_steering(128),right_front_steering(128),right_rear_steering(128),left_rear_steering(128){}
};

struct MotorParams
{
    quint8 left_front_motor1;
    quint8 left_front_motor2;
    quint8 right_front_motor1;
    quint8 right_front_motor2;
    quint8 right_rear_motor1;
    quint8 right_rear_motor2;
    quint8 left_rear_motor1;
    quint8 left_rear_motor2;
    MotorParams():left_front_motor1(128),left_front_motor2(128),right_front_motor1(128),right_front_motor2(128),
        right_rear_motor1(128),right_rear_motor2(128),left_rear_motor1(128),left_rear_motor2(128){}
};

struct ControlSteeringSpeed
{
    double deltaa=0;
    double deltab=0;
    double delta=0;
    bool isInv=true;
};

struct ControlMotorSpeed
{
    double vlf1=0;
    double vlf2=0;
    double vrf1=0;
    double vrf2=0;
    double vrr1=0;
    double vrr2=0;
    double vlr1=0;
    double vlr2=0;
    void setInvData()
    {
        vlf1=-vlf1;
        vlf2=-vlf2;
        vrf1=-vrf1;
        vrf2=-vrf2;
        vlr1=-vlr1;
        vlr2=-vlr2;
        vrr1=-vrr1;
        vrr2=-vrr2;
    }
    void setSameData()
    {
        vlr1=vlf1;
        vlr2=vlf2;
        vrr1=vrf1;
        vrr2=vrf2;
    }
    /*
    void setRotData()
    {
        vlr1=-vlf1;
        vlr2=-vlf2;
        vrr1=-vrf1;
        vrr2=-vrf2;
    }*/
    void setZeroData()
    {
        vlf1=vlf2=vrf1=vrf2=vrr1=vrr2=vlr1=vlr2=0.0;
    }

    // ========== 新增乘除法工具函数 ==========

    // 乘法操作（就地修改）
    void multiplyAll(double factor)
    {
        vlf1 *= factor;
        vlf2 *= factor;
        vrf1 *= factor;
        vrf2 *= factor;
        vrr1 *= factor;
        vrr2 *= factor;
        vlr1 *= factor;
        vlr2 *= factor;
    }

    // 除法操作（就地修改）
    void divideAll(double divisor)
    {
        if (qFuzzyIsNull(divisor)) {
            qWarning() << "电机速度除法：除数接近零，操作取消";
            return;
        }

        vlf1 /= divisor;
        vlf2 /= divisor;
        vrf1 /= divisor;
        vrf2 /= divisor;
        vrr1 /= divisor;
        vrr2 /= divisor;
        vlr1 /= divisor;
        vlr2 /= divisor;
    }

    // 缩放操作（就地修改）
    void scaleAll(double scale)
    {
        multiplyAll(scale);
    }

    // 限制所有速度到指定范围
    void clampAll(double minVal, double maxVal)
    {
        vlf1 = qBound(minVal, vlf1, maxVal);
        vlf2 = qBound(minVal, vlf2, maxVal);
        vrf1 = qBound(minVal, vrf1, maxVal);
        vrf2 = qBound(minVal, vrf2, maxVal);
        vrr1 = qBound(minVal, vrr1, maxVal);
        vrr2 = qBound(minVal, vrr2, maxVal);
        vlr1 = qBound(minVal, vlr1, maxVal);
        vlr2 = qBound(minVal, vlr2, maxVal);
    }

    // 获取最大速度值
    double getMaxSpeed() const
    {
        double maxSpeed = qAbs(vlf1);
        maxSpeed = qMax(maxSpeed, qAbs(vlf2));
        maxSpeed = qMax(maxSpeed, qAbs(vrf1));
        maxSpeed = qMax(maxSpeed, qAbs(vrf2));
        maxSpeed = qMax(maxSpeed, qAbs(vrr1));
        maxSpeed = qMax(maxSpeed, qAbs(vrr2));
        maxSpeed = qMax(maxSpeed, qAbs(vlr1));
        maxSpeed = qMax(maxSpeed, qAbs(vlr2));
        return maxSpeed;
    }

    // 归一化所有速度到最大值
    void normalizeToMax(double maxSpeed)
    {
        double currentMax = getMaxSpeed();
        if (currentMax > maxSpeed && !qFuzzyIsNull(currentMax)) {
            double scaleFactor = maxSpeed / currentMax;
            multiplyAll(scaleFactor);
        }
    }

    // 运算符重载（返回新对象）
    ControlMotorSpeed operator*(double factor) const
    {
        ControlMotorSpeed result = *this;
        result.multiplyAll(factor);
        return result;
    }

    ControlMotorSpeed operator/(double divisor) const
    {
        ControlMotorSpeed result = *this;
        result.divideAll(divisor);
        return result;
    }

    // 复合赋值运算符
    ControlMotorSpeed& operator*=(double factor)
    {
        multiplyAll(factor);
        return *this;
    }

    ControlMotorSpeed& operator/=(double divisor)
    {
        divideAll(divisor);
        return *this;
    }
};

class CalIKControlModel : public QObject
{
    Q_OBJECT
public:
    explicit CalIKControlModel(QObject *parent = nullptr);
    bool calControlParams(double vx,double rz,int controlMode,ControlSteeringSpeed &mySteeringSpeed,ControlMotorSpeed &myMotorSpeed); //0-四轮阿卡曼,1-两轮阿卡曼,3-原地旋转
    bool calControlParams(double vx,double theta,int controlMode,ControlSteeringSpeed &mySteeringSpeed,ControlMotorSpeed &myMotorSpeed,bool isTheta); //遥控模式
    ControlMotorSpeed calActualHWSpeed(double vx=0,double rz=0,int controlMode=0,ControlSteeringSpeed mySpeed={0,0,0});
    SteeringParams convertToActualSteeringMsg(ControlSteeringSpeed mySteeringSpeed);
    SteeringParams convertToActualSteeringMsg_Mode_Spot(ControlSteeringSpeed mySteeringSpeed);
    MotorParams convertToActualMotorMsg(ControlMotorSpeed myMotorSpeed);
    void checkControlParams(ControlSteeringSpeed &mySteeringSpeed,ControlMotorSpeed &myMotorSpeed);
    // 斜行接近模式：直接指向目标点
    bool calApproachParams(double targetX, double targetY, double currentX,
                           double currentY, double currentYaw,
                           ControlSteeringSpeed &mySteeringSpeed,
                           ControlMotorSpeed &myMotorSpeed);
    // 姿态调整模式：原地旋转到目标姿态
    bool calOrientationParams(double targetYaw, double currentYaw,
                              ControlSteeringSpeed &mySteeringSpeed,
                              ControlMotorSpeed &myMotorSpeed);
    //====8.6新增：增加API，适用于新报文的参数计算======
    VelControlParams calAKMControlParams_new(double vx,double rz);
    VelControlParams calAKMThetaControlParams_new(double vx,double theta);
    VelControlParams calApproachParams_new(double targetX, double targetY, double currentX,
                                           double currentY, double currentYaw);
    // 姿态调整模式：原地旋转到目标姿态
    VelControlParams calOrientationParams_new(double targetYaw, double currentYaw);
    static constexpr double STEERING_RATION_PARAMS_NEW=0.1;  //转向角范围（-28度到+28度）
    //static constexpr double VEL_RATION_PARAMS_NEW=0.005; // 放大倍数不合适
    static constexpr double VEL_RATION_PARAMS_NEW=0.01;  //速度范围（0-1.25m/s）
    static constexpr double MAX_VEL_X=1.25;
    //================================================

    static constexpr double L=2.5; //车辆参数 前后轴距
    static constexpr double W=1.25; //车辆参数，转向机构间距
    static constexpr double W2=0.75; //车辆参数，同一转向机构中两轮轮间距
    static constexpr double MIN_CAL_VELX=0.01;
    static constexpr double MAXSTEERINGANGLE=25*M_PI/180;  // 最大转向角28度，对应最小转弯半径4.5m
    static constexpr double MAXSTEERINGANGLE_DEG=25;
    static constexpr double STEERING_RATION_PARAMS=0.2;
    static constexpr double STEERING_SPOT_RATION_PARAMS=0.5;
    static constexpr double MOTOR_RATION_PARAMS=12;
    static constexpr double REDUCTION_RATION=24.78; //轮边减速器减速比，轮边减速器半径
    static constexpr double WHEEL_RADIUS=0.226;
    const double minRadius = L / tan(MAXSTEERINGANGLE);
    const double wheelCircumference=2*M_PI*WHEEL_RADIUS;
    const double rotateR=sqrt(L*L/4+W*W/4);
    const double rotateThtea=atan(L/W);
    double vx=0; //前进方向速度
    double rz=0; //绕轴旋转速度
    double theta=0; //转向角

    double calculateDistance(double x1, double y1, double x2, double y2) const;
    double calculateTargetAngle(double targetX, double targetY, double currentX, double currentY) const;
    double normalizeAngle(double angle) const;
    double limitSteeringAngle(double angle) const;
private:
    // ========== 三阶段导航专用参数 ==========
    static constexpr double APPROACH_SPEED = 0.15;         // 斜行接近速度 (m/s)
    static constexpr double ORIENTATION_SPEED = 0.2;//0.05;      // 姿态调整角速度 (rad/s)
    static constexpr double MIN_APPROACH_DISTANCE = 0.2;   // 最小接近距离 (m)
    static constexpr double MAX_APPROACH_ANGLE = 25*M_PI/180;   // 最大接近角度 (25度)


signals:
};

#endif // CALIKCONTROLMODEL_H
