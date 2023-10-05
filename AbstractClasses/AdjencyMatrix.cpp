#include "AdjencyMatrix.h"


AdjencyMatrix::AdjencyMatrix(int vertexCount) : m_vertexCount(vertexCount)
{
    m_vertexCount=vertexCount;
    m_matrix.resize(vertexCount,std::vector<int>(vertexCount,0));
}

void AdjencyMatrix::refill(std::vector<Edge*>E)
{
    m_matrix.clear();
    m_matrix.resize(m_vertexCount,std::vector<int>(m_vertexCount,0));
    for(Edge* e : E){
       m_matrix[e->startVertex()->numeric()][e->finishVertex()->numeric()]=e->weight();
    }
}


void AdjencyMatrix::toKirchhoff()
{
    int n = m_matrix.size();
    for(int i = 0; i < n;i++){
        int degreeOfRow=0;
        for(int j = 0; j < n;j++){
            if(m_matrix[i][j] != 0){
               degreeOfRow++;
               m_matrix[i][j] = -1;
            }
        }
        m_matrix[i][i]=degreeOfRow;
    }
}

