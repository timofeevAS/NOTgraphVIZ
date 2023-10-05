#include "AdjencyList.h"


AdjencyList::AdjencyList(int vertexCount)
{
    m_vertexCount=vertexCount;
    m_adjList.resize(vertexCount);
}

QList<Node *> AdjencyList::getVertexByNumeric(int num)
{
    return m_adjList[num];
}

void AdjencyList::refill(std::vector<Edge *> E)
{
    m_adjList.clear();
    m_adjList.resize(m_vertexCount);
    for(Edge* e1 : E){
        m_adjList[e1->startVertex()->numeric()].append(e1->finishVertex());
    }
}

