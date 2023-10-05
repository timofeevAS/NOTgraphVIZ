#include "Node.h"

int Node::numeric() const
{
    return m_numeric;
}

Node::Node(int numeric) : m_numeric(numeric)
{}

bool Node::operator==(const Node &right)
{
    if(this->numeric()==right.numeric()){
        return true;
    }
    else{
        return false;
    }
}
