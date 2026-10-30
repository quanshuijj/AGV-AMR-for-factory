#ifndef MAPDISPLAYWIDGET_H
#define MAPDISPLAYWIDGET_H

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "pointmanager.h"  

// 地图显示窗口类声明
class MapDisplayWidget : public QGraphicsView {
    Q_OBJECT
public:
    explicit MapDisplayWidget(QWidget* parent = nullptr);
    void loadMap(const nav_msgs::OccupancyGrid& map);
    void updateRobotPosition(const geometry_msgs::PoseStamped& pose);
    bool loadMapFromFile(const QString& yamlFile);
    void drawPoints(const QList<NavigationPoint>& points);
    
    void setStartPointId(int id);
    void setTargetPointId(int id);

     // 坐标转换（需要添加）
    QPointF sceneToMap(const QPointF& scenePos) const;
    QPointF mapToScene(const QPointF& mapPos) const;
    
    // 地图状态查询（需要添加）
    bool isMapLoaded() const;
    double getResolution() const;
    QPointF getOrigin() const;
    int getMapWidth() const;
    int getMapHeight() const;
    
    // 点击位置获取（需要添加）
    QPointF getLastClickedPosition() const;
    
    // 清除地图
    void clear();
    
    // 路径显示功能
    void updatePath(const nav_msgs::Path& path);
    void clearPath();
    void drawPath(const QList<QPointF>& pathPoints, const QColor& color = Qt::blue);
    
    // 多轨迹显示接口
    void drawGlobalPath(const QList<QPointF>& pathPoints);
    void drawTebGlobalPath(const QList<QPointF>& pathPoints);
    void drawTebLocalPath(const QList<QPointF>& pathPoints);
    void drawActualOdomPath(const QList<QPointF>& pathPoints);
    void clearGlobalPath();
    void clearTebGlobalPath();
    void clearTebLocalPath();
    void clearActualOdomPath();
signals:
    void pointClicked(QPointF position);  // 点击地图信号
    
protected:
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    
private:
    QGraphicsScene* scene;
    QGraphicsPixmapItem* mapItem;
    QGraphicsEllipseItem* robotItem;
    QList<QGraphicsEllipseItem*> pointItems;  // 存储绘制的点对象
    QList<QGraphicsPathItem*> pathItems;      // 存储绘制的路径对象
    
    QList<QGraphicsPathItem*> globalPathItems;
    QList<QGraphicsPathItem*> tebGlobalPathItems;
    QList<QGraphicsPathItem*> tebLocalPathItems;
    QList<QGraphicsPathItem*> actualOdomPathItems;
    
    int m_startPointId = -1;
    int m_targetPointId = -1;

    double mapResolution;
    QPointF mapOrigin;
    QPointF lastClickedPosition;
    bool mapLoaded = false;
};
#endif // MAPDISPLAYWIDGET_H