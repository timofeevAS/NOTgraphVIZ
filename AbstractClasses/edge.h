#ifndef EDGE_H
#define EDGE_H

#include "Node.h"

class Edge

{
private:
    bool m_direction;//Oriented or not edge
    int m_weight; //Weighted or not edge
    Node* m_startVertex;
    Node* m_finishVertex;//start and finish node

public:
    //CONSTRUCTOR
    Edge(Node* start,Node* end,bool direction = false,int weight = 1);

    //GETTERS
    bool direction() const;
    int weight() const;
    Node *startVertex() const;
    Node *finishVertex() const;

    bool operator==(const Edge &right);
    bool operator()(const Edge& a,const Edge& b);
};

#endif // EDGE_H
