#ifndef ADJENCYLIST_H
#define ADJENCYLIST_H

#include <vector>
#include <QList>

#include "edge.h"
#include "Node.h"


class AdjencyList{
private:
    std::vector<QList<Node*>>m_adjList;
    int m_vertexCount;
public:
    //CONSTRUCTOR
    AdjencyList(int vertexCount);

    //GETTERS
    QList<Node*> getVertexByNumeric(int num);

    void refill(std::vector<Edge*> E);
};

#endif // ADJENCYLIST_H
