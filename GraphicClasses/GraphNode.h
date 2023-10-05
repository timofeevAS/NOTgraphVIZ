#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include <QGraphicsItem>
#include <QObject>
#include "AbstractClasses/Node.h"
#include "GraphScene.h"

#include <QtWidgets>

class Node;
class GraphScene;

class GraphNode : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit GraphNode(Node* baseNode,GraphScene* graphScene, QObject *parent = 0);
    //explicit GraphNode(int numeric, QObject *parent = 0);

    ~GraphNode();
    void setColor(QColor new_color);
    QList<QColor> getColorList();
    QPointF getPoint();



    Node *baseNode() const;

    int radius() const;

private:
        Node* m_baseNode;
        GraphScene* m_graphScene;
        int m_numeric;
        int m_radius;
        QColor m_color;

        QPainterPath shape() const override;
        QRectF boundingRect() const override;
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
        void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
        void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;




};

#endif // GRAPHNODE_H
