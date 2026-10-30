#ifndef POINTMANAGER_H
#define POINTMANAGER_H

#include <QObject>
#include <QPointF>
#include <QList>
#include <QColor>
#include <QDebug>
#include <QDataStream>
#include <QFile>
#include <QIODevice>

// 前向声明和运算符重载
struct NavigationPoint {
    QPointF position;
    QString name;
    bool isFixed;
    QColor color;
    int id;
};

// 为 NavigationPoint 添加 QDebug 运算符重载
inline QDebug operator<<(QDebug dbg, const NavigationPoint& point) {
    dbg << QString("NavigationPoint(%1, %2, %3)")
           .arg(point.name)
           .arg(point.position.x(), 0, 'f', 2)
           .arg(point.position.y(), 0, 'f', 2);
    return dbg;
}

// 为 QDataStream 添加序列化支持
inline QDataStream& operator<<(QDataStream& out, const NavigationPoint& point) {
    out << point.position << point.name << point.isFixed << point.color << point.id;
    return out;
}

inline QDataStream& operator>>(QDataStream& in, NavigationPoint& point) {
    in >> point.position >> point.name >> point.isFixed >> point.color >> point.id;
    return in;
}

class PointManager : public QObject {
    Q_OBJECT
public:
    explicit PointManager(QObject* parent = nullptr);
    
    // 点管理操作
    void addFixedPoint(const QPointF& position, const QString& name = "");
    void addCustomPoint(const QPointF& position, const QString& name = "");
    void removePoint(int id);
    void clearPoints();
    void clearCustomPoints();
    
    // 数据持久化
    bool saveToFile(const QString& filename);
    bool loadFromFile(const QString& filename);
    
    // 查询接口
    QList<NavigationPoint> getFixedPoints() const;
    QList<NavigationPoint> getCustomPoints() const;
    QList<NavigationPoint> getAllPoints() const;
    NavigationPoint getPoint(int id) const;
    
signals:
    void pointsChanged();
    void pointAdded(const NavigationPoint& point);
    void pointRemoved(int id);
    
private:
    QList<NavigationPoint> fixedPoints;
    QList<NavigationPoint> customPoints;
    int nextId = 1;
};

#endif // POINTMANAGER_H