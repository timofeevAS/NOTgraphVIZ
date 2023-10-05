#ifndef ADJMATRIX_H
#define ADJMATRIX_H
#include <vector>
#include <map>

#include "edge.h"
#include "Node.h"

class AdjencyMatrix{

private:
    int m_vertexCount; // size of matrix
    std::vector<std::vector<int>> m_matrix; // matrix incapsuled into vector of vectors

public:
    AdjencyMatrix(int vertexCount);

    void refill(std::vector<Edge*> E);

    //GETTERS

    inline int getMaxSize() const {return m_matrix.size();};
    inline int getAtPos(int i,int j) const{ if(i < getMaxSize() && j<getMaxSize()){return m_matrix[i][j];}else{return 0;}};
    inline std::vector<std::vector<int>> getPureMatrixByVector() const {return m_matrix;};

    //SETTERS
    inline void setAtPos(int i,int j,int value){m_matrix[i][j]=value;};

    void toKirchhoff();

};

#endif
