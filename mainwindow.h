#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtWidgets>
#include <QDebug>

#include "TableViz.h"

#include "GraphicClasses/GraphScene.h"
#include "GraphicClasses/GraphView.h"
#include "GraphicClasses/GraphEdge.h"
#include "GraphicClasses/GraphNode.h"

#include "AbstractClasses/Graph.h"
#include "AbstractClasses/Node.h"
#include "AbstractClasses/edge.h"

#include "GenerateGraphDialog.h"



class MainWindow : public QMainWindow
{
    Q_OBJECT
private:
    Graph* G1; // GRAPH
    QPlainTextEdit *m_LogOutput; // LOGGING OF YOUR DOINGS
    TableViz *m_tableViz; // TABLE PRESENTATION OF WIDGET ADJENCY LIST
    TableViz *m_shimbellViz; // TABLE PRESENTATION OF SHIMBELL MATRIX
    TableViz *m_bandwidthViz; // TABLE OF BANDWIDTH FOR NETWORKS
    TableViz *m_kirchhoffViz; // TABLE OF KIRCHOFF MATRIX
    QTabWidget *m_selectorOfTable; //MENU TAB to select details table
    GraphScene *m_graphSceneVIZ;
    GraphView* m_graphViewVIZ;
    QWidget* mw;


    QWidget* m_shimbellMW;
    QVBoxLayout* m_shimbellLayout;
    QComboBox* m_shimbellSelector;
    QSpinBox* m_shimbellSpinBox;

    //GenerateGraphDialog m_generateGraphDialog;

    // LOGGING
    void updateCurrentLog(QString logData = "");
    void updateCurrentLog(std::vector<std::vector<int>> someMatrix);


    void updateAll(QString logData = "");

    void prepareGraphMemory();
    void drawGraph();



public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void s_actionsDoings(QAction* action);
    void s_shimbellDegree(int value);
    void s_shimbellSelector(const QString& text);


};
#endif // MAINWINDOW_H
