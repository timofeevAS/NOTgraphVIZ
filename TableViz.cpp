#include "TableViz.h"

void TableViz::update(std::vector<std::vector<int>>matrix,std::vector<int>numerics)
{

    if(m_maxsize != static_cast<int>(matrix.size()) || matrix.size()==0)return; //SIZES of matrix and maxsize is difference
    QStringList NumericLabels;

    for(int i = 0;i < m_maxsize;i++){
        NumericLabels.append(QString().setNum(i));
    }

    this->setHorizontalHeaderLabels(NumericLabels);
    this->setVerticalHeaderLabels(NumericLabels);

    for(int i = 0; i < m_maxsize;i++){
        this->setRowHidden(i,true);
        this->setColumnHidden(i,true);
    }

    for(int i = 0;i < m_maxsize;i++){
        for(int j = 0;j<m_maxsize;j++){
            this->m_itemMatrix[i][j]->setText(QString().setNum(matrix[i][j]));
            this->m_itemMatrix[i][j]->setFlags(m_itemMatrix[i][j]->flags() ^ Qt::ItemIsEditable);
        }
    }

    for(int num : numerics){
        this->setRowHidden(num,false);
        this->setColumnHidden(num,false);
    }




}

TableViz::~TableViz()
{
    for(int i = 0;i < m_maxsize;i++){
        for(int j = 0;j<m_maxsize;j++){
            delete m_itemMatrix[i][j];
        }
    }
}

TableViz::TableViz(int max_size)
{
    m_maxsize = max_size;
    QStringList NumericLabels;

    for(int i = 0;i < max_size;i++){
        NumericLabels.append(QString().setNum(i));
    }

    //CONFIGURE TABLE
    this->setHorizontalHeaderLabels(NumericLabels);
    this->setVerticalHeaderLabels(NumericLabels);
    this->setColumnCount(max_size);
    this->setRowCount(max_size);
    this->setFixedSize(590,670);
    this->setSortingEnabled(false);
    this->setSelectionMode(QAbstractItemView::SingleSelection);
    this->setDragEnabled(false);

    m_itemMatrix.resize(max_size,std::vector<QTableWidgetItem*>(max_size));

    for(int i = 0;i < m_maxsize;i++){
        for(int j = 0;j<m_maxsize;j++){
            m_itemMatrix[i][j]=new QTableWidgetItem(QString().setNum(0));
            this->setItem(i,j,m_itemMatrix[i][j]);
        }
    }
    for(int i = 0;i < max_size;i++){
        this->setColumnWidth(i,30);
    }
}

void TableViz::shimbell(int degree,std::vector<int>numerics,int mode)
{
    std::vector<std::vector<int>>tmp_matrix(m_itemMatrix.size(),std::vector<int>(m_itemMatrix.size(),0));
    std::vector<std::vector<int>>result(m_itemMatrix.size(),std::vector<int>(m_itemMatrix.size(),0));

    for(int i = 0;i < static_cast<int>(tmp_matrix.size());i++){
        for(int j = 0;j < static_cast<int>(tmp_matrix[i].size());j++){
            tmp_matrix[i][j] = m_itemMatrix[i][j]->text().toInt();
        }
    }
    tmp_matrix=matrixPower(tmp_matrix,degree,mode);
    this->update(tmp_matrix,numerics);


}

std::vector<std::vector<int> > matrixShimbellMultiply(const std::vector<std::vector<int> > &matrix1, const std::vector<std::vector<int> > &matrix2,int mode)
{
    int n = matrix1.size();
    int m = matrix2[0].size();

    int value_by_mode = mode == 0 ? INT_MAX : 0;

    std::vector<std::vector<int>> result(n, std::vector<int>(m, value_by_mode));
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < m; ++j) {
        for (int k = 0; k < static_cast<int>(matrix2.size()); ++k) {

            int result_num = (matrix1[i][k] == 0 || matrix2[k][j] == 0) && mode < 2 ? 0 : matrix1[i][k] + matrix2[k][j];
            if(result_num == 0 && mode == 0){continue;}
            if(mode == 0){result[i][j] = std::min(result_num,result[i][j]);}
            else if(mode == 1){result[i][j] = std::max(result_num,result[i][j]);}
            else if(mode == 2){result[i][j] = matrix1[i][k] * matrix2[k][j];}
        }
      }
    }

    for(int i = 0;i < n;i++){
        for(int j = 0;j<n;j++){
            if(result[i][j] == INT_MAX || result[i][j] == INT_MIN)
            { result[i][j] = 0; }
        }
    }

    return result;
}

std::vector<std::vector<int> > matrixPower(const std::vector<std::vector<int> > &matrix, int power,int mode)
{
    std::vector<std::vector<int>> result = matrix;
    for (int i = 1; i < power; ++i) {
      result = matrixShimbellMultiply(result, matrix,mode);
    }


    return result;
}
