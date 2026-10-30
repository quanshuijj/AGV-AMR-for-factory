#include "navcar_project_server/mapdisplaywidget.h"
#include <QWheelEvent>
#include <QMouseEvent>
#include <QDebug>
#include <yaml-cpp/yaml.h>
#include <QGraphicsPathItem>
#include <QFileInfo>

MapDisplayWidget::MapDisplayWidget(QWidget* parent)
    : QGraphicsView(parent) 
{
    scene = new QGraphicsScene(this);
    setScene(scene);
    setDragMode(ScrollHandDrag);
    setTransformationAnchor(AnchorUnderMouse);
}

bool MapDisplayWidget::loadMapFromFile(const QString& yamlFile) {
    try {
        // 读取YAML配置文件
        YAML::Node config = YAML::LoadFile(yamlFile.toStdString());
        
        // 检查必要字段是否存在
        if (!config["image"] || !config["resolution"] || !config["origin"]) {
            qWarning() <<"Invalid YAML config file: missing required fields";
            return false;
        }
        
        std::string imagePath = config["image"].as<std::string>();
        // 如果imagePath不是绝对路径，则补全为maps文件夹下
        QFileInfo yamlInfo(yamlFile);
        QString mapDir = yamlInfo.absolutePath();
        QString imageFullPath;
        if (QFileInfo(QString::fromStdString(imagePath)).isAbsolute()) {
            imageFullPath = QString::fromStdString(imagePath);
        } else {
            imageFullPath = mapDir + "/" + QString::fromStdString(imagePath);
        }
        double resolution = config["resolution"].as<double>();
        std::vector<double> origin = config["origin"].as<std::vector<double>>();
        
        // 检查origin数组大小
        if (origin.size() < 2) {
            qWarning() << "Invalid origin format in YAML file";
            return false;
        }
        
        // 加载PGM图像
        QImage mapImage(imageFullPath);
        if(mapImage.isNull()) {
            qWarning() << "Failed to load map image:" << QString::fromStdString(imagePath);
            return false;
        }
        
        // 创建地图显示
        QPixmap mapPixmap = QPixmap::fromImage(mapImage);
        scene->clear();
        mapItem = scene->addPixmap(mapPixmap);
        
        // 保存地图参数
        mapResolution = resolution;
        mapOrigin = QPointF(origin[0], origin[1]);
        
        // 设置场景范围
        scene->setSceneRect(0, 0, mapImage.width(), mapImage.height());
        fitInView(sceneRect(), Qt::KeepAspectRatio);
        
        mapLoaded = true;  // 记得设置地图加载状态
        return true;
        
    } catch (const YAML::Exception& e) {
        qWarning() << "YAML parsing error:" << e.what();
        return false;
    } catch (const std::exception& e) {
        qWarning() << "Error loading map:" << e.what();
        return false;
    }
}


void MapDisplayWidget::loadMap(const nav_msgs::OccupancyGrid& map) {
    // 实现地图加载和渲染
}

void MapDisplayWidget::wheelEvent(QWheelEvent* event) {
    // 实现缩放功能
    if (event->angleDelta().y() > 0) {
        scale(1.2, 1.2); // 放大
    } else {
        scale(1/1.2, 1/1.2); // 缩小
    }
    event->accept();
}


// MapDisplayWidget 中添加坐标转换方法（地图转换）
QPointF MapDisplayWidget::sceneToMap(const QPointF& scenePos) const {
    double mapX = scenePos.x() * mapResolution + mapOrigin.x();
    double mapY = (sceneRect().height() - scenePos.y()) * mapResolution + mapOrigin.y();
    return QPointF(mapX, mapY);
}

QPointF MapDisplayWidget::mapToScene(const QPointF& mapPos) const {
    double sceneX = (mapPos.x() - mapOrigin.x()) / mapResolution;
    double sceneY = sceneRect().height() - (mapPos.y() - mapOrigin.y()) / mapResolution;
    return QPointF(sceneX, sceneY);
}

void MapDisplayWidget::drawPoints(const QList<NavigationPoint>& points) {
    // 清除旧的点
    for (auto item : pointItems) {
        scene->removeItem(item);
        delete item;
    }
    pointItems.clear();
    
    // 绘制新点
    for (const auto& point : points) {
        QPointF scenePos = mapToScene(point.position);
        QColor color = point.color;
        int size = 10;
        QPen pen(Qt::black);

        if (point.id == m_startPointId) {
            color = Qt::green;
            size = 18;
            pen.setColor(Qt::darkGreen);
            pen.setWidth(4);
        } else if (point.id == m_targetPointId) {
            color = Qt::red;
            size = 20;
            pen.setColor(Qt::yellow);
            pen.setWidth(4);
        }

        QGraphicsEllipseItem* item = scene->addEllipse(
            scenePos.x() - size/2, scenePos.y() - size/2, size, size,
            pen, QBrush(color)
        );

        // Add text label for start/target
        if (point.id == m_startPointId) {
            item->setToolTip("Start Point");
        } else if (point.id == m_targetPointId) {
            item->setToolTip("Target Point");
        } else {
            item->setToolTip(point.name);
        }

        pointItems.append(item);
    }
}

void MapDisplayWidget::setStartPointId(int id) {
    m_startPointId = id;
    // We need to trigger redraw logic. 
    // Since we don't store points locally, we might need the caller to call drawPoints 
    // OR just rely on the next update.
    // However, usually drawPoints is called when points change. 
    // If we just change selection, we need to redraw current points.
    // Ideally MapDisplayWidget should store points, or we just rely on MainWindow to call drawPoints again.
    // Let's assume MainWindow will call drawPoints after setting ID.
}

void MapDisplayWidget::setTargetPointId(int id) {
    m_targetPointId = id;
}

bool MapDisplayWidget::isMapLoaded() const {
    return mapLoaded;
}

QPointF MapDisplayWidget::getLastClickedPosition() const {
    return lastClickedPosition;
}

void MapDisplayWidget::clear() {
    scene->clear();
    mapItem = nullptr;
    mapLoaded = false;
    clearPath();
    pointItems.clear();
}

double MapDisplayWidget::getResolution() const {
    return mapResolution;
}

void MapDisplayWidget::updatePath(const nav_msgs::Path& path) {
    // 清除旧路径
    clearPath();
    
    if (path.poses.empty()) {
        return;
    }
    
    QList<QPointF> pathPoints;
    for (const auto& pose : path.poses) {
        QPointF mapPos(pose.pose.position.x, pose.pose.position.y);
        QPointF scenePos = mapToScene(mapPos);
        pathPoints.append(scenePos);
    }
    
    // 绘制新路径
    drawPath(pathPoints, Qt::blue);
}

void MapDisplayWidget::clearPath() {
    for (auto item : pathItems) {
        scene->removeItem(item);
        delete item;
    }
    pathItems.clear();
}

void MapDisplayWidget::drawPath(const QList<QPointF>& pathPoints, const QColor& color) {
    if (pathPoints.size() < 2) {
        return;
    }
    
    QPainterPath painterPath;
    painterPath.moveTo(pathPoints.first());
    
    for (int i = 1; i < pathPoints.size(); ++i) {
        painterPath.lineTo(pathPoints[i]);
    }
    
    QGraphicsPathItem* pathItem = scene->addPath(painterPath, 
        QPen(color, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    pathItems.append(pathItem);
}

void MapDisplayWidget::updateRobotPosition(const geometry_msgs::PoseStamped& pose) {
    // 暂时不实现机器人位置显示
}

// 点击位置记录
void MapDisplayWidget::mousePressEvent(QMouseEvent* event) {
    QGraphicsView::mousePressEvent(event);
    
    if (event->button() == Qt::LeftButton) {
        QPointF scenePos = mapToScene(event->pos());
        lastClickedPosition = scenePos;
        emit pointClicked(sceneToMap(scenePos));
    }
}

void MapDisplayWidget::drawGlobalPath(const QList<QPointF>& pathPoints) {
    clearGlobalPath();
    if (pathPoints.size() < 2) return;
    QPainterPath painterPath;
    painterPath.moveTo(pathPoints.first());
    for (int i = 1; i < pathPoints.size(); ++i) {
        painterPath.lineTo(pathPoints[i]);
    }
    QGraphicsPathItem* item = scene->addPath(painterPath, QPen(Qt::blue, 2));
    globalPathItems.append(item);
}
void MapDisplayWidget::clearGlobalPath() {
    for (auto item : globalPathItems) { scene->removeItem(item); delete item; }
    globalPathItems.clear();
}
void MapDisplayWidget::drawTebGlobalPath(const QList<QPointF>& pathPoints) {
    clearTebGlobalPath();
    if (pathPoints.size() < 2) return;
    QPainterPath painterPath;
    painterPath.moveTo(pathPoints.first());
    for (int i = 1; i < pathPoints.size(); ++i) {
        painterPath.lineTo(pathPoints[i]);
    }
    QGraphicsPathItem* item = scene->addPath(painterPath, QPen(Qt::darkGreen, 2, Qt::DashLine));
    tebGlobalPathItems.append(item);
}
void MapDisplayWidget::clearTebGlobalPath() {
    for (auto item : tebGlobalPathItems) { scene->removeItem(item); delete item; }
    tebGlobalPathItems.clear();
}
void MapDisplayWidget::drawTebLocalPath(const QList<QPointF>& pathPoints) {
    clearTebLocalPath();
    if (pathPoints.size() < 2) return;
    QPainterPath painterPath;
    painterPath.moveTo(pathPoints.first());
    for (int i = 1; i < pathPoints.size(); ++i) {
        painterPath.lineTo(pathPoints[i]);
    }
    QGraphicsPathItem* item = scene->addPath(painterPath, QPen(Qt::red, 2, Qt::DotLine));
    tebLocalPathItems.append(item);
}
void MapDisplayWidget::clearTebLocalPath() {
    for (auto item : tebLocalPathItems) { scene->removeItem(item); delete item; }
    tebLocalPathItems.clear();
}
void MapDisplayWidget::drawActualOdomPath(const QList<QPointF>& pathPoints) {
    clearActualOdomPath();
    if (pathPoints.size() < 2) return;
    QPainterPath painterPath;
    painterPath.moveTo(pathPoints.first());
    for (int i = 1; i < pathPoints.size(); ++i) {
        painterPath.lineTo(pathPoints[i]);
    }
    QGraphicsPathItem* item = scene->addPath(painterPath, QPen(Qt::magenta, 2));
    actualOdomPathItems.append(item);
}
void MapDisplayWidget::clearActualOdomPath() {
    for (auto item : actualOdomPathItems) { scene->removeItem(item); delete item; }
    actualOdomPathItems.clear();
}


