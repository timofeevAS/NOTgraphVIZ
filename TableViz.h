#ifndef TABLEVIZ_H
#define TABLEVIZ_H

#include <QTableWidget>
#include <QObject>
#include <vector>
#include <climits>

std::vector<std::vector<int>> matrixShimbellMultiply(const std::vector<std::vector<int>>& matrix1, const std::vector<std::vector<int>>& matrix2,int mode);
std::vector<std::vector<int>> matrixPower(const std::vector<std::vector<int>>& matrix, int power,int mode);


class TableViz : public QTableWidget
{
private:
    int m_maxsize;

    std::vector<std::vector<QTableWidgetItem*>>m_itemMatrix;


public:

    TableViz(int max_size);
    void shimbell(int degree,std::vector<int>numerics,int mode);
    void update(std::vector<std::vector<int>> matrix,std::vector<int>numerics);
    ~TableViz();
};




#endif // TABLEVIZ_H
