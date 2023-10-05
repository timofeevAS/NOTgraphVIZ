#include "mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{

    this->resize(800,700);

    mw= new QWidget();

    G1 = new Graph(false);

    m_selectorOfTable = new QTabWidget;

    m_graphSceneVIZ = new GraphScene;
    m_graphViewVIZ = new GraphView(m_graphSceneVIZ,G1);
    //m_graphViewVIZ->setFixedHeight();
    m_graphViewVIZ->setFixedSize(600,700);

    m_LogOutput = new QPlainTextEdit();
    //m_LogOutput->setFixedWidth(500);
    m_LogOutput->setReadOnly(true);
    m_LogOutput->setFixedSize(550,700);

    m_tableViz = new TableViz(100);
    m_shimbellViz = new TableViz(100);
    m_bandwidthViz = new TableViz(100);
    m_kirchhoffViz = new TableViz(100);

    m_shimbellLayout = new QVBoxLayout();
    m_shimbellSpinBox = new QSpinBox;
    m_shimbellSpinBox->setValue(1);
    m_shimbellSelector = new QComboBox;
    m_shimbellSelector->addItems(QStringList() <<"Min"<<"Max");
    m_shimbellLayout->addWidget(m_shimbellSpinBox);
    m_shimbellLayout->addWidget(m_shimbellSelector);
    m_shimbellLayout->addWidget(m_shimbellViz);
    m_shimbellMW = new QWidget();
    m_shimbellMW->setLayout(m_shimbellLayout);


    m_selectorOfTable->addTab(m_tableViz,"Adjency Matrix");
    m_selectorOfTable->addTab(m_shimbellMW,"Shimbell Matrix");
    m_selectorOfTable->addTab(m_bandwidthViz,"Bandwith Matrix");
    m_selectorOfTable->addTab(m_kirchhoffViz,"Kirchhoff Matrix");
    m_selectorOfTable->setFixedSize(600,700);


    QHBoxLayout * box = new QHBoxLayout(mw);



    this->setCentralWidget(mw);
    box->addWidget(m_graphViewVIZ);
    box->addWidget(m_selectorOfTable);
    box->addWidget(m_LogOutput);

    this->move(0,0);

    connect(m_graphViewVIZ->m_rightClickMenu,SIGNAL(triggered(QAction*)),this,SLOT(s_actionsDoings(QAction*)));
    connect(m_shimbellSelector,SIGNAL(currentTextChanged(const QString&)),this,SLOT(s_shimbellSelector(const QString&)));
    connect(m_shimbellSpinBox,SIGNAL(valueChanged(int)),this,SLOT(s_shimbellDegree(int)));

}

void MainWindow::updateCurrentLog(QString logData)
{
    if(logData == "__clear"){
        m_LogOutput->clear();
        return;
    }
    if(m_LogOutput->toPlainText().count("\n") > 500){
        updateCurrentLog("__clear");
        updateCurrentLog();
        updateCurrentLog(logData);
        return;
    }
    if(logData != ""){
       m_LogOutput->appendPlainText(logData);
       return;
    }
    m_LogOutput->clear();
    QString matr,lst;
    for(int i = 0;i<static_cast<int>(G1->nodes().size());i++){
           for(int j = 0;j<static_cast<int>(G1->nodes().size());j++){
               matr.push_back(QString().setNum(G1->adjMatrix().getAtPos(G1->nodes()[i]->numeric(),G1->nodes()[j]->numeric()) >= 1 ? 1 : 0));
           }
           matr.push_back(QChar('\n'));
       }

       for(int i = 0;i<static_cast<int>(G1->nodes().size());i++){
           lst.push_back(QString().setNum(G1->nodes()[i]->numeric()));
           lst.push_back(": ");
           for(int j = 0;j<G1->adjList().getVertexByNumeric(G1->nodes()[i]->numeric()).size();j++){
               lst.push_back(QString().setNum(G1->adjList().getVertexByNumeric(G1->nodes()[i]->numeric()).at(j)->numeric()));
               lst.push_back(',');
           }
           lst.push_back(QChar('\n'));
       }
       m_LogOutput->appendHtml("<HTML><B>OUTPUT</B><HTML>");
       m_LogOutput->appendPlainText("\n"+matr+"\n"+lst);

}

void MainWindow::updateCurrentLog(std::vector<std::vector<int> > someMatrix)
{
    QString output;
    for(Node* n1 : G1->nodes()){
        for(Node* n2 : G1->nodes()){
            if(someMatrix[n1->numeric()][n2->numeric()]==INT_FAST16_MAX){
                output.append("∞\t");
            }
            else{
                output.append(QString().setNum(someMatrix[n1->numeric()][n2->numeric()])+ "\t");
                }
            }
        output.append("\n");
    }
    updateCurrentLog(output);
}

void MainWindow::updateAll(QString logData){
    updateCurrentLog(logData);
    m_graphViewVIZ->setGraph(this->G1);
    m_tableViz->update(G1->adjMatrix().getPureMatrixByVector(),G1->getNumericsOfNodes());
    m_shimbellViz->update(G1->adjMatrix().getPureMatrixByVector(),G1->getNumericsOfNodes());
    m_bandwidthViz->update(G1->bandwidthMatrix().getPureMatrixByVector(),G1->getNumericsOfNodes());
    m_kirchhoffViz->update(G1->kirchhoffMatrix().getPureMatrixByVector(),G1->getNumericsOfNodes());
}

void MainWindow::prepareGraphMemory()
{
    if(!G1->nodes().empty()){ //not empty nodes set in graph
        delete G1;
        m_graphSceneVIZ->clear();
        m_graphSceneVIZ->updateSelectedNodes();
    }
}

void MainWindow::drawGraph()
{
    m_graphViewVIZ->setGraph(this->G1);
    m_graphSceneVIZ->drawGraph(this->G1);
}


MainWindow::~MainWindow()
{
    delete m_LogOutput;
    delete mw;
    delete G1;

}

void MainWindow::s_actionsDoings(QAction *action)
{
    qDebug()<<"CLICKED ON CONTEXT MENU: " << action->text();
    QString actionText(action->text());
    ActionNumber selectedOption = m_graphViewVIZ->getIndexOfAction(actionText);
    QString newToLog = "";
    std::pair <int,int> selected_vertex(-1,-1);

    if(m_graphSceneVIZ->selectedNodes().size() > 1){
        selected_vertex = std::make_pair(
                    m_graphSceneVIZ->selectedNodes()[m_graphSceneVIZ->selectedNodes().size()-2]->baseNode()->numeric(),
                    m_graphSceneVIZ->selectedNodes()[m_graphSceneVIZ->selectedNodes().size()-1]->baseNode()->numeric());
    }

    if(selectedOption == ActionNumber::GENERATE_GRAPH){

        prepareGraphMemory();

        GenerateGraphDialog tmp_graph_generate_dialog;
        //create QDialog to make preferences for generate users' graph

        if(tmp_graph_generate_dialog.exec()){

            bool tmp_weighted = tmp_graph_generate_dialog.flags().value("weighted");
            bool tmp_random = tmp_graph_generate_dialog.flags().value("random");
            int tmp_vertex_count = tmp_graph_generate_dialog.flags().value("vertex_count");
            bool tmp_oriented = tmp_graph_generate_dialog.flags().value("oriented");
            bool tmp_linked = tmp_graph_generate_dialog.flags().value("linked");
            bool tmp_acyclic = tmp_graph_generate_dialog.flags().value("acyclic");
            bool tmp_web_mode = tmp_graph_generate_dialog.flags().value("web");
            bool tmp_graph_mode = tmp_graph_generate_dialog.flags().value("graph");

            qDebug() << "CURRENT GENERATE FLAGS:"
            <<"\nWeighted flag: " <<tmp_weighted
            <<"\nRandom Flag: " << tmp_random
            <<"\nVertex Count: " << tmp_vertex_count
            <<"\nOriented Flag: " << tmp_oriented
            <<"\nLinked flag: " << tmp_linked
            <<"\nAcyclic flag: " << tmp_acyclic
            <<"\nWeb mode: " << tmp_web_mode
            <<"\nGraph mode: " << tmp_graph_mode;

            //if web_mode


            //if graph oriented
            if(tmp_oriented){
                G1 = new OrientedGraph(tmp_weighted);
            }
            else{
                G1 = new UnorientedGraph(tmp_weighted);
            }

            if(tmp_web_mode){
                G1->generateGraph(tmp_vertex_count,tmp_random,tmp_acyclic,tmp_linked,tmp_web_mode);
            }
            else{
            G1->generateGraph(tmp_vertex_count,tmp_random,tmp_acyclic,tmp_linked);
            }
            drawGraph();

            updateAll();
        }
    }
    else if(selectedOption == ActionNumber::INPUT_GRAPH){
        prepareGraphMemory();

        GenerateGraphInput tmp_graph_generate_dialog;
        //create QDialog to make preferences for generate users' graph

        if(tmp_graph_generate_dialog.exec()){
        qDebug()<<"INPUT MATRIX";
        G1 = new UnorientedGraph(true);
        G1->rebuildByAdjMatrix(tmp_graph_generate_dialog.adjMatrix);

        for(auto row : tmp_graph_generate_dialog.adjMatrix){
            for(auto j : row){
                qDebug()<<j;
            }
        }
        }
        drawGraph();
        updateAll();
    }
    else if(selectedOption == ActionNumber::FIND_ALL_PATH_FROM_U_TO_V){

        qDebug() << selected_vertex.first << selected_vertex.second;
        std::vector<std::vector<int>>all_path;
        std::vector<int>path;

        //update log that we start algo
        updateCurrentLog("Execute find all paths between: " +
                         QString().setNum(selected_vertex.first) + " " +
                         QString().setNum(selected_vertex.second) + "\n");

        G1->findAllPaths(selected_vertex.first,selected_vertex.second,path,all_path);
        //make log to output
        updateCurrentLog("Find out: " + QString().setNum(all_path.size()) + " paths\n");
        for(auto item : all_path){
            QString cur_path;
            for(auto v : item){
               cur_path.push_back(QString().setNum(v)+"->");
            }
            cur_path.remove(cur_path.size()-2,2);
            cur_path.append('\n');
            newToLog.append(cur_path);
        }
        if(all_path.empty()){
            newToLog.append("Doesn't exist any paths between: " +
                            QString().setNum(selected_vertex.first) + " " +
                            QString().setNum(selected_vertex.second));
        }
        updateCurrentLog(newToLog);
    }
    else if(selectedOption == ActionNumber::FLOYD_WARSHALL_ALGORITHM){
        qDebug() << "FLOYD_WARSHALL: " << selected_vertex.first << selected_vertex.second;
        std::vector<std::vector<int>>matrix_of_weight;
        std::vector<std::vector<int>>matrix_of_paths;
        std::vector<int> path_vector;
        path_vector = G1->floydWarshall(selected_vertex.first,selected_vertex.second,matrix_of_paths,matrix_of_weight);

        //update log that we start algo
        updateCurrentLog("Execute find path with Floyd Warshall between: " +
                         QString().setNum(selected_vertex.first) + " " +
                         QString().setNum(selected_vertex.second));
        updateCurrentLog("\nCount of iterations: " + QString().setNum(pow(G1->nodes().size(),3)) + "\n");
        updateCurrentLog("Matrix of weights: \n");
        updateCurrentLog(matrix_of_weight);
        updateCurrentLog("Matrix of paths: \n");
        updateCurrentLog(matrix_of_paths);
        if(path_vector.empty()){
            newToLog.append("Doesn't exist any paths between: " +
                            QString().setNum(selected_vertex.first) + " " +
                            QString().setNum(selected_vertex.second));
            updateCurrentLog(newToLog);
        }
        else{
            QString output_path = "Shortest path is:\n";
            for(auto v : path_vector){
                output_path.append(QString().setNum(v) + "->");
            }
            output_path.remove(output_path.size()-2,2);
            output_path.append("\n");
            updateCurrentLog(output_path);
        }
    }
    else if(selectedOption == ActionNumber::DIJKSTRA_ALGORITHM){
        qDebug() << "DIJKSTRA ALGO: " << selected_vertex.first << selected_vertex.second;
        std::vector<std::pair<int,bool>>vector_of_shortest_paths;
        std::vector<int> path_vector;
        path_vector = G1->dijkstraAlgorithm(selected_vertex.first,selected_vertex.second,vector_of_shortest_paths);

        //update log that we start algo
        updateCurrentLog("Execute find path with Dijkstra between: " +
                         QString().setNum(selected_vertex.first) + " " +
                         QString().setNum(selected_vertex.second));
        updateCurrentLog("\nCount of iterations: " + QString().setNum(path_vector.back()) + "\n");
        path_vector.pop_back();
        updateCurrentLog("Vector of shortest paths: \n");
        QString vector_sp_output;
        for(int i : G1->getNumericsOfNodes()){
            vector_sp_output.append(QString().setNum(i) + "\t");
        }
        vector_sp_output.append("\n");

        for(auto i : vector_of_shortest_paths){
            if(i.first == INT_FAST16_MAX){
                vector_sp_output.append("∞\t");
            }
            else{
                vector_sp_output.append(QString().setNum(i.first) + "\t");
            }
        }
        vector_sp_output.append("\n\n\n");

        updateCurrentLog(vector_sp_output);

        if(path_vector.empty()){
            newToLog.append("Doesn't exist any paths between: " +
                            QString().setNum(selected_vertex.first) + " " +
                            QString().setNum(selected_vertex.second));
            updateCurrentLog(newToLog);
        }
        else{
            QString output_path = "Shortest path is:\n";
            for(auto v : path_vector){
                output_path.append(QString().setNum(v) + "->");
            }
            output_path.remove(output_path.size()-2,2);
            output_path.append("\n");
            updateCurrentLog(output_path);

        }
    }
    else if(selectedOption==ActionNumber::BELLMAN_FORD_ALGORITHM){ qDebug() << "DIJKSTRA ALGO: " << selected_vertex.first << selected_vertex.second;
        std::vector<std::pair<int,bool>>vector_of_shortest_paths;
        std::vector<int> path_vector;
        path_vector = G1->bellmanFordAlgorithm(selected_vertex.first,selected_vertex.second,vector_of_shortest_paths);

        //update log that we start algo
        updateCurrentLog("Execute find path with Dijkstra between: " +
                         QString().setNum(selected_vertex.first) + " " +
                         QString().setNum(selected_vertex.second));
        updateCurrentLog("\nCount of iterations: " + QString().setNum(path_vector.back()) + "\n");
        path_vector.pop_back();
        updateCurrentLog("Vector of shortest paths: \n");
        QString vector_sp_output;
        for(int i : G1->getNumericsOfNodes()){
            vector_sp_output.append(QString().setNum(i) + "\t");
        }
        vector_sp_output.append("\n");

        for(auto i : vector_of_shortest_paths){
            if(i.first == INT_FAST16_MAX){
                vector_sp_output.append("∞\t");
            }
            else{
                vector_sp_output.append(QString().setNum(i.first) + "\t");
            }
        }
        vector_sp_output.append("\n");

        updateCurrentLog(vector_sp_output);

        if(path_vector.empty()){
            newToLog.append("Doesn't exist any paths between: " +
                            QString().setNum(selected_vertex.first) + " " +
                            QString().setNum(selected_vertex.second));
            updateCurrentLog(newToLog);
        }
        else{
            QString output_path = "Shortest path is:\n";
            for(auto v : path_vector){

                output_path.append(QString().setNum(v) + "->");
            }
            output_path.remove(output_path.size()-2,2);
            output_path.append("\n");
            updateCurrentLog(output_path);

        }

    }else if(selectedOption==ActionNumber::FORD_FALKERSON_ALGORITHM){
        /*
        if(G1->isWeb()==false){
            updateCurrentLog("You need to generate web");
        }
        */
        int source_numeric=0,
            sink_numeric=G1->nodes().size()-1;

        int maxFlow = G1->fordFalkersonAlgorithm(source_numeric,sink_numeric);
        QString log = "Execute Ford-falkerson algorithm between "+QString().setNum(source_numeric)+" "+QString().setNum(sink_numeric);
        updateCurrentLog(log+"\nMax flow is: " + QString().setNum(maxFlow));


    }else if(selectedOption==ActionNumber::MIN_COST_MAX_FLOW_ALGORITHM){
        int source_numeric=0,
            sink_numeric=G1->nodes().size()-1;



        int minCostMaxFlow = G1->twoThirdsMaxFlowAlgorithm(source_numeric,sink_numeric);
        int maxFlow = G1->fordFalkersonAlgorithm(source_numeric,sink_numeric);
        maxFlow *= (2/3.0);
        QString addition = QString("2/3 max flow is: ") + QString().setNum(maxFlow);
        QString log = "Execute Min-Cost-Max-Flow algorithm between "+QString().setNum(source_numeric)+" "+QString().setNum(sink_numeric);
        updateCurrentLog(addition+"\n"+log+"\nMin cost is: " + QString().setNum(minCostMaxFlow));
    }
    else if(selectedOption==ActionNumber::PRIMA_ALGORITHM || selectedOption==ActionNumber::KRUSKAL_ALGORITHM){

        if(selectedOption==ActionNumber::PRIMA_ALGORITHM && selected_vertex.first < 0){
            updateCurrentLog("ERROR!!!\nChoose vertex!");
            return;
        }

        std::vector<Edge*>result = selectedOption==ActionNumber::PRIMA_ALGORITHM ? G1->primaMinSpanTree(selected_vertex.first) :
                                                                                   G1->kruskalMinSpanTree(selected_vertex.first);
        //output

        QString algo_name = selectedOption==ActionNumber::PRIMA_ALGORITHM ? "Prima algorithm from " + QString().setNum(selected_vertex.first) :
                                                                            "Kruskala algorithm";

        QString log = "Execute " + algo_name + "\n";



        QString addition = "Minimal Span Tree is: \n";
        int summary_weight=0;
        for(Edge* e : result){
            addition += QString().setNum(e->startVertex()->numeric()) + "->" + QString().setNum(e->finishVertex()->numeric()) + "\t|" +
                    QString().setNum(e->weight()) + "\n";
            summary_weight+=e->weight();
        }
        addition+="Summary weight is: " + QString().setNum(summary_weight);
        addition+="\nIterations: " +QString().setNum(G1->lastIterations());


        if(G1->nodes().size() < 13){
            int countOfSpanTrees = G1->countOfSpanTrees();
            addition+="\nOverall count of span trees in graph is: " +QString().setNum(countOfSpanTrees);
        }else{
            addition+="\nWARNING!!!\nOverall count of span trees can not calculate due to size of matrix Kirchhoff bigger than 13\n";
        }
        updateCurrentLog(log+addition);

        //recoloring edges//recoloring edges
        m_graphSceneVIZ->recolorEdges();
        m_graphSceneVIZ->recolorEdges(result);

        //make prufer_code
        std::vector<int> prufer_code = G1->pruferCode(result);
        QString code_p ="";
        for(int num : prufer_code){
            //qDebug() << num;
            code_p+=QString().setNum(num) + " ";
        }
        updateCurrentLog("Prufer code: "+code_p+"\n");
        std::vector<std::vector<int>>decoded_prufer = G1->pruferDecode(prufer_code);

        updateCurrentLog(decoded_prufer);
    }
    else if(selectedOption==ActionNumber::DECODE_PRUFER){
        std::vector<Edge*>result = G1->kruskalMinSpanTree(selected_vertex.first);
        //make prufer_code
        std::vector<int> prufer_code = G1->pruferCode(result);
        QString code_p ="";
        for(int num : prufer_code){
            //qDebug() << num;
            code_p+=QString().setNum(num) + " ";
        }
        updateCurrentLog("Prufer code: "+code_p+"\n");
        std::vector<std::vector<int>>decoded_prufer = G1->pruferDecode(prufer_code);
        //G1->rebuildByAdjMatrix(decoded_prufer);
        //m_graphSceneVIZ->clear();
        //drawGraph();
        //recolor span tree
        m_graphSceneVIZ->recolorEdges();
        m_graphSceneVIZ->recolorEdges(result);
        m_graphSceneVIZ->removeNotRed(result);
        updateCurrentLog(decoded_prufer);
    }
    else if(selectedOption==ActionNumber::CLEAR_LOG_OPTION){
        updateCurrentLog();
    }
    else if(selectedOption==ActionNumber::CHECK_HAMILTONIAN){
        std::vector<Edge*>hamiltonian_path = G1->findHamiltonianCycle();
        if(!hamiltonian_path.empty()){
            //qDebug()<<"FIND HAMI";
            QString addition = "Hamiltonian cycle is: \n";


            for(Edge* e : hamiltonian_path){
                addition += QString().setNum(e->startVertex()->numeric()) + "->" + QString().setNum(e->finishVertex()->numeric()) + "\t|" +
                        QString().setNum(e->weight()) + "\n";
            }
            addition += "\n";
            updateCurrentLog(addition);
            m_graphSceneVIZ->recolorEdges();
            m_graphSceneVIZ->recolorEdges(hamiltonian_path);
            m_graphSceneVIZ->removeNotRed(hamiltonian_path);
            updateCurrentLog("Cycle is red");
        }
        else{
            updateCurrentLog("Graph hasnt hamiltonian cycle");
        }
    }
    else if(selectedOption==ActionNumber::BUILD_HAMILTIONIAN){
        qDebug()<<"Created Hami";
        G1->buildToHamiltonian();
        drawGraph();
        updateAll();
    }else if(selectedOption==ActionNumber::CHECK_EILER){
        std::vector<Edge*>euler_path = G1->findEulerianCycle();
        euler_path.pop_back();

        if(!G1->isEulerCriterium(G1->adjMatrixVector())){
            updateCurrentLog("Graph hasnt euler cycle (Criterium wrong)");
            return;
        }

        if(!euler_path.empty()){

            QString addition = "Euler cycle is: \n";


            for(Edge* e : euler_path){
                addition += QString().setNum(e->startVertex()->numeric()) + "->" + QString().setNum(e->finishVertex()->numeric()) + "\t|" +
                        QString().setNum(e->weight()) + "\n";
            }
            addition += "\n";
            updateCurrentLog(addition);
            m_graphSceneVIZ->recolorEdges();
            m_graphSceneVIZ->recolorEdges(euler_path);
            m_graphSceneVIZ->removeNotRed(euler_path);
            updateCurrentLog("Cycle is red");
        }
        else{
            updateCurrentLog("Graph hasnt euler cycle");
        }
    }else if(selectedOption==ActionNumber::BUILD_EILER){
        G1->buildToEuler();
        qDebug()<<"Created Euler";
        drawGraph();
        updateAll();
        //updateCurrentLog(G1->adjMatrixVector());
    }
    else if(selectedOption==ActionNumber::TSP_SOLUTION){
        std::vector<Edge*>tsp_path = G1->tspSolution();
        if(!tsp_path.empty()){
            int sum_w=0;
            QString addition = "TSP cycle is: \n";


            for(Edge* e : tsp_path){
                addition += QString().setNum(e->startVertex()->numeric()) + "->" + QString().setNum(e->finishVertex()->numeric()) + "\t|" +
                        QString().setNum(e->weight()) + "\n";
                sum_w+=e->weight();
            }
            addition += "\n";
            updateCurrentLog(addition);
            m_graphSceneVIZ->recolorEdges();
            m_graphSceneVIZ->recolorEdges(tsp_path);
            updateCurrentLog("Cycle is red");
            updateCurrentLog("Summary weight is: "+QString().setNum(sum_w));
        }
        else{
            updateCurrentLog("Graph hasnt tsp cycle");
        }
    }
}

void MainWindow::s_shimbellSelector(const QString &text)
{
    Q_UNUSED(text);
    s_shimbellDegree(m_shimbellSpinBox->value());
}

void MainWindow::s_shimbellDegree(int value)
{
    if(value==0){
        return;
    }

    if(value > static_cast<int>(G1->nodes().size())){
        m_shimbellSpinBox->setValue(static_cast<int>(G1->nodes().size()));
        return;
    }

    int mode = (m_shimbellSelector->currentText() == "Min") ? 0 : 1;

    m_shimbellViz->update(G1->adjMatrix().getPureMatrixByVector(),G1->getNumericsOfNodes());
    m_shimbellViz->shimbell(value,G1->getNumericsOfNodes(),mode);


}

