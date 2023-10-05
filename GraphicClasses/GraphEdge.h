#ifndef GRAPHEDGE_H
#define GRAPHEDGE_H

#include <QGraphicsItem>
#include <QObject>
#include "GraphNode.h"
#include "AbstractClasses/edge.h"

#include <QtWidgets>

class GraphNode;
class Edge;

class GraphEdge : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit GraphEdge(GraphNode* startNode,GraphNode* finishNode,Edge* parentEdge,int bandwidth=-1, QObject *parent = 0);
    ~GraphEdge();

    Edge *parentEdge() const;

    const QColor &color() const;
    void setColor(const QColor &newColor);

private:
        GraphNode* m_startNode,*m_finishNode;
        Edge* m_parentEdge;

        int m_weight;
        int m_bandwidth;
        int m_arrowSize;
        QColor m_color;

        QPointF m_startPoint,m_finishPoint;


        QPainterPath shape() const override;
        QRectF boundingRect() const override;
        void adjustCoord();
        void adjust();
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
        //void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
        //void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
        //void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

};


#endif // GRAPHEDGE_H
