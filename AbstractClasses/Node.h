#ifndef NODE_H
#define NODE_H
#include <vector>

class Node{

private:
    int m_numeric; //numeric parametr of node


public:
    //CONSTRUCTOR
    Node(int numeric);

    //GETTER

    bool operator==(const Node &right);


    int numeric() const;
};

#endif
