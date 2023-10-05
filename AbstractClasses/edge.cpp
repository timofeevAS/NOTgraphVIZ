#include "edge.h"



bool Edge::direction() const
{
    return m_direction;
}

int Edge::weight() const
{
    return m_weight;
}

Node *Edge::startVertex() const
{
    return m_startVertex;
}

Node *Edge::finishVertex() const
{
    return m_finishVertex;
}

bool Edge::operator==(const Edge &right)
{
    if(this->startVertex()->numeric()==right.startVertex()->numeric() &&
       this->finishVertex()->numeric()==right.finishVertex()->numeric()){
        return true;
    }
    else
    {
        return false;
    }
}

bool Edge::operator()(const Edge &a, const Edge &b)
{
    return a.m_weight > b.m_weight;
}

Edge::Edge(Node *start, Node *end,bool direction,int weight)
{
    m_startVertex=start;
    m_finishVertex=end;
    m_direction=direction;
    m_weight=weight;
}
