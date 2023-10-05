
#include "GraphEdge.h"

GraphEdge::GraphEdge(GraphNode *startNode, GraphNode *finishNode,Edge* parentEdge, int bandwidth, QObject *parent)
{
    Q_UNUSED(parent);
    m_startNode=startNode;
    m_finishNode=finishNode;
    m_arrowSize = 10;
    m_startPoint=startNode->pos().toPoint();
    m_finishPoint=finishNode->pos().toPoint();
    m_parentEdge = parentEdge;
    m_bandwidth = bandwidth;
    m_color=QColor(Qt::black);
    //adjustCoord();
    adjust();
}

GraphEdge::~GraphEdge()
{

}

Edge *GraphEdge::parentEdge() const
{
    return m_parentEdge;
}

const QColor &GraphEdge::color() const
{
    return m_color;
}

void GraphEdge::setColor(const QColor &newColor)
{
    m_color = newColor;
    this->setZValue(100);
}

QPainterPath GraphEdge::shape() const
{
    QLineF line{m_startPoint, m_finishPoint};
    auto base = line.pointAt(1 - m_arrowSize / line.length());
    auto d = m_finishPoint - base;
    d = {d.y(), -d.x()};
    auto path = QPainterPath{};
    path.addPolygon(QVector<QPointF>{
                        m_startPoint + d, m_startPoint - d,
                        m_finishPoint - d, m_finishPoint + d
                    });
    return path;
}

QRectF GraphEdge::boundingRect() const
{
    if (!m_startNode || !m_finishNode)
        return QRectF();

    qreal penWidth = 1;
    qreal extra = (penWidth + m_arrowSize) / 2.0;

    return QRectF{ m_startPoint, m_finishPoint }
    .normalized()
    .marginsAdded({extra, extra, extra, extra});
}

void GraphEdge::adjustCoord()
{
    m_startPoint = m_startNode->getPoint();
    m_finishPoint = m_finishNode->getPoint();
}

void GraphEdge::adjust()
{
    if (!m_startNode || !m_finishNode)
        return;

    QLineF line(mapFromItem(m_startNode, 0, 0), mapFromItem(m_finishNode, 0, 0));
    qreal length = line.length();

    prepareGeometryChange();

    if (length > qreal(20.)) {
        QPointF edgeOffset((line.dx() * 10) / length, (line.dy() * 10) / length);
        m_startPoint = line.p1() + edgeOffset;
        m_finishPoint = line.p2() - edgeOffset;
    } else {
        m_startPoint = m_finishPoint = line.p1();
    }
}

void GraphEdge::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);
    adjust();
    //adjustCoord();
    if (!m_finishNode || !m_startNode)
        return;

    QLineF line(m_startPoint, m_finishPoint);
    if (qFuzzyCompare(line.length(), qreal(0.)))
        return;
    // Draw the line itself
    painter->setPen(QPen(m_color, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter->drawLine(line);

    // Draw the arrows if direction edge
    if(m_parentEdge->direction()){
        double angle = std::atan2(-line.dy(), line.dx());
     //   QPointF sourceArrowP1 = m_startPoint + QPointF(sin(angle + M_PI / 3) * m_arrowSize,
     //                                                 cos(angle + M_PI / 3) * m_arrowSize);
     //   QPointF sourceArrowP2 = m_startPoint + QPointF(sin(angle + M_PI - M_PI / 3) * m_arrowSize,
     //                                                 cos(angle + M_PI - M_PI / 3) * m_arrowSize);
        QPointF destArrowP1 = m_finishPoint + QPointF(sin(angle - M_PI / 3) * m_arrowSize,
                                                  cos(angle - M_PI / 3) * m_arrowSize);
        QPointF destArrowP2 = m_finishPoint + QPointF(sin(angle - M_PI + M_PI / 3) * m_arrowSize,
                                                  cos(angle - M_PI + M_PI / 3) * m_arrowSize);
        painter->setBrush(Qt::black);
        //painter->drawPolygon(QPolygonF() << line.p1() << sourceArrowP1 << sourceArrowP2);
        painter->drawPolygon(QPolygonF() << line.p2() << destArrowP1 << destArrowP2);
    }

    //Draw weight if weighted
    if(m_parentEdge->weight() != 1){
        QString txt = QString().setNum(m_parentEdge->weight());
        //if graph has matrix of bandwidth
        if(m_bandwidth!=-1){
            QString bandwidth_str = QString().setNum(m_bandwidth);

            txt.append(" ("+bandwidth_str+") ");
        }
        QFont font("Arial",10,2);
        painter->setFont(font);
        QFontMetrics fm(font);
        QPointF centerOfLine = line.center();
        font.setBold(true);
        painter->drawText(centerOfLine+QPointF(10,0),txt);
    }

}
