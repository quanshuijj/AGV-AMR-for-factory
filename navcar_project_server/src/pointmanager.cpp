#include "navcar_project_server/pointmanager.h"
#include <QFile>
#include <QDataStream>
#include <QDebug>

PointManager::PointManager(QObject* parent) : QObject(parent) {}

void PointManager::addFixedPoint(const QPointF& position, const QString& name) {
    NavigationPoint point;
    point.id = nextId++;
    point.position = position;
    point.name = name.isEmpty() ? QString("固定点%1").arg(fixedPoints.size() + 1) : name;
    point.isFixed = true;
    point.color = Qt::red;
    
    fixedPoints.append(point);
    emit pointAdded(point);
    emit pointsChanged();
}

void PointManager::addCustomPoint(const QPointF& position, const QString& name) {
    NavigationPoint point;
    point.id = nextId++;
    point.position = position;
    point.name = name.isEmpty() ? QString("自定义点%1").arg(customPoints.size() + 1) : name;
    point.isFixed = false;
    point.color = Qt::blue;
    
    customPoints.append(point);
    emit pointAdded(point);
    emit pointsChanged();
}

void PointManager::removePoint(int id) {
    // 从固定点中移除
    for (int i = 0; i < fixedPoints.size(); ++i) {
        if (fixedPoints[i].id == id) {
            fixedPoints.removeAt(i);
            emit pointRemoved(id);
            emit pointsChanged();
            return;
        }
    }
    
    // 从自定义点中移除
    for (int i = 0; i < customPoints.size(); ++i) {
        if (customPoints[i].id == id) {
            customPoints.removeAt(i);
            emit pointRemoved(id);
            emit pointsChanged();
            return;
        }
    }
}

bool PointManager::saveToFile(const QString& filename) {
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "无法打开文件进行写入:" << filename;
        return false;
    }
    
    QDataStream out(&file);
    out << fixedPoints << customPoints;
    file.close();
    return true;
}

bool PointManager::loadFromFile(const QString& filename) {
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "无法打开文件进行读取:" << filename;
        return false;
    }
    
    QDataStream in(&file);
    in >> fixedPoints >> customPoints;
    file.close();
    
    // 更新nextId
    for (const auto& point : fixedPoints) {
        if (point.id >= nextId) nextId = point.id + 1;
    }
    for (const auto& point : customPoints) {
        if (point.id >= nextId) nextId = point.id + 1;
    }
    
    emit pointsChanged();
    return true;
}

QList<NavigationPoint> PointManager::getFixedPoints() const {
    return fixedPoints;
}

QList<NavigationPoint> PointManager::getCustomPoints() const {
    return customPoints;
}

QList<NavigationPoint> PointManager::getAllPoints() const {
    QList<NavigationPoint> allPoints;
    allPoints.append(fixedPoints);
    allPoints.append(customPoints);
    return allPoints;
}

NavigationPoint PointManager::getPoint(int id) const {
    for (const auto& point : fixedPoints) {
        if (point.id == id) return point;
    }
    for (const auto& point : customPoints) {
        if (point.id == id) return point;
    }
    return NavigationPoint();
}