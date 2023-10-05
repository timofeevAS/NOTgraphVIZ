#ifndef GRAPHVIEW_H
#define GRAPHVIEW_H

#include <QGraphicsView>
#include <QObject>
#include <QWidget>
#include <QtWidgets>
#include <vector>
#include "AbstractClasses/Graph.h"

class Graph;


enum ActionNumber{
    CLEAR_LOG_OPTION,
    ERROR_ALGORITHM,
    GENERATE_RANDOM_ORIENTED_GRAPH,
    GENERATE_RANDOM_UNORIENETED_GRAPH,
    GENERATE_GRAPH,
    FIND_ALL_PATH_FROM_U_TO_V,
    FLOYD_WARSHALL_ALGORITHM,
    DIJKSTRA_ALGORITHM,
    BELLMAN_FORD_ALGORITHM,
    FORD_FALKERSON_ALGORITHM,
    MIN_COST_MAX_FLOW_ALGORITHM,
    PRIMA_ALGORITHM,
    KRUSKAL_ALGORITHM,
    DECODE_PRUFER,
    BORUVKA_ALGORITHM,
    CHECK_HAMILTONIAN,
    CHECK_EILER,
    BUILD_HAMILTIONIAN,
    BUILD_EILER,
    INPUT_GRAPH,
    TSP_SOLUTION,
};

class GraphView : public QGraphicsView
{
    Q_OBJECT

private:
    std::vector<QAction*> m_actions;
    Graph* m_G1;
    void prepareContextMenu();
public:
    GraphView(QGraphicsScene* scene,Graph* G);
    ~GraphView();
    QMenu* m_rightClickMenu;
    inline void setGraph(Graph*G){this->m_G1=G;};
    QMenu* rightClickMenu();
    void contextMenuEvent(QContextMenuEvent* event) override; //event by Right click
    ActionNumber getIndexOfAction(QString& action);


};

#endif // GRAPHVIEW_H
