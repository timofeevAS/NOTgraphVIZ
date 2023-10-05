#ifndef GRAPHSCENE_H
#define GRAPHSCENE_H
#include <QGraphicsScene>
#include <QObject>
#include <QWidget>

#include "GraphNode.h"
#include "AbstractClasses/Graph.h"
#include "AbstractClasses/edge.h"
#include "GraphEdge.h"

class Graph;
class OrientedGraph;
class UnorientedGraph;
class GraphNode;
class GraphEdge;
class Edge;

class GraphScene : public QGraphicsScene
{
private:
    std::vector<GraphNode*> m_selectedNodes;
    std::vector<GraphNode*> m_createdNodes;
    std::vector<GraphEdge*> m_createdEdges;


public:
    void updateSelectedNodes(GraphNode* v1 = nullptr);
    explicit GraphScene(QObject *parent = nullptr);
    QPointF delta = {100,100}; //delta from 0,0 point idk how to fix w\o it, just place 0,0 and try to move nodes, and u will see

    void drawGraph(Graph* G);
    void recolorEdges(std::vector<Edge*> edges);
    void recolorEdges();
    GraphEdge* getGraphEdgeByParent(Edge* e);
    void removeNotRed(std::vector<Edge*> edges);

    const std::vector<GraphNode *> &selectedNodes() const;
};


#endif
