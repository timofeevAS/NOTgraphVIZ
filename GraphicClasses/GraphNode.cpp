#include "GraphNode.h"


GraphNode::GraphNode(Node* baseNode,GraphScene* graphScene, QObject *parent)
{
    setFlag(ItemIsMovable);
    //setFlag(ItemSendsGeometryChanges);

    m_baseNode=baseNode;
    m_graphScene=graphScene;
    m_radius=30;
    m_color = this->getColorList().at(0);


    this->setParent(parent);
    //qDebug()<<baseNode->numeric();
}
/*
GraphNode::GraphNode(int numeric, QObject *parent)
{
    m_numeric=numeric;
    this->setParent(parent);
}
*/

GraphNode::~GraphNode()
{

}

QPainterPath GraphNode::shape() const
{
    QPainterPath tmp;
    tmp.addEllipse(boundingRect());
    return tmp;
}

QRectF GraphNode::boundingRect() const
{
    return QRectF(-m_radius/2,-m_radius/2,m_radius,m_radius);
}

void GraphNode::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setPen(QPen(Qt::black,2,Qt::SolidLine));
    painter->setBrush(QBrush(m_color));
    painter->drawEllipse(-m_radius/2,-m_radius/2,m_radius,m_radius);

    QString txt = "v"+QString().setNum(m_baseNode->numeric());
    QFont font("Arial",10);
    painter->setFont(font);
    QFontMetrics fm(font);
    painter->drawText(-fm.width(txt)/2,fm.height()/3,txt);
}

void GraphNode::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    Q_UNUSED(event);
    update();
    this->setPos(mapToScene(event->pos()));
    m_graphScene->update(0,0,1000,1000);
}

void GraphNode::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    Q_UNUSED(event);
    this->setCursor(QCursor(Qt::ClosedHandCursor));
    m_graphScene->updateSelectedNodes(this);
    //qDebug() << "Press on object";


}

void GraphNode::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    Q_UNUSED(event);
    this->setCursor(QCursor(Qt::ArrowCursor));
    m_graphScene->update(0,0,1000,1000);
}

void GraphNode::setColor(QColor new_color)
{
    m_color = new_color;
}

QList<QColor> GraphNode::getColorList()
{
    QList <QColor> colorList;
    colorList
              << QColor(229,204,255)
              << QColor(255,229,204)
              << QColor(204,255,204);
    return colorList;
}

QPointF GraphNode::getPoint()
{
    return this->pos().toPoint();
}

Node *GraphNode::baseNode() const
{
    return m_baseNode;
}

int GraphNode::radius() const
{
    return m_radius;
}
