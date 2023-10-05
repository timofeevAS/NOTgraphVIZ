#include "GraphView.h"

ActionNumber GraphView::getIndexOfAction(QString& actionText){
    if(actionText == "Random oriented graph"){
        return ActionNumber::GENERATE_RANDOM_ORIENTED_GRAPH;
    }else if(actionText=="Random unoriented graph"){
        return ActionNumber::GENERATE_RANDOM_UNORIENETED_GRAPH;
    }else if(actionText=="Generate new graph"){
        return ActionNumber::GENERATE_GRAPH;
    }else if(actionText=="Find all paths"){
        return ActionNumber::FIND_ALL_PATH_FROM_U_TO_V;
    }else if(actionText=="Floyd Warshall"){
        return ActionNumber::FLOYD_WARSHALL_ALGORITHM;
    }else if(actionText=="Dijkstra's algorithm"){
        return ActionNumber::DIJKSTRA_ALGORITHM;
    }else if(actionText=="Bellman-Ford algorithm"){
        return ActionNumber::BELLMAN_FORD_ALGORITHM;
    }else if(actionText=="Ford-falkerson algorithm"){
        return ActionNumber::FORD_FALKERSON_ALGORITHM;
    }else if(actionText=="Min-Cost-Max-Flow algorithm"){
        return ActionNumber::MIN_COST_MAX_FLOW_ALGORITHM;
    }else if(actionText=="Prima algorithm"){
        return ActionNumber::PRIMA_ALGORITHM;
    }else if(actionText=="Kruskal algorithm"){
        return ActionNumber::KRUSKAL_ALGORITHM;
    }else if(actionText=="Boruvka algorithm"){
        return ActionNumber::BORUVKA_ALGORITHM;
    }else if(actionText=="Clear log output"){
        return ActionNumber::CLEAR_LOG_OPTION;
    }else if(actionText=="Is Hamiltonian"){
        return ActionNumber::CHECK_HAMILTONIAN;
    }else if(actionText=="Is Eiler"){
        return ActionNumber::CHECK_EILER;
    }else if(actionText=="Build to Hamiltonian"){
        return ActionNumber::BUILD_HAMILTIONIAN;
    }else if(actionText=="Build to Eiler"){
        return ActionNumber::BUILD_EILER;
    }else if(actionText=="Decode Prufer code"){
        return ActionNumber::DECODE_PRUFER;
    }else if(actionText=="Input adj matrix"){
        return ActionNumber::INPUT_GRAPH;
    }else if(actionText=="TSP solve"){
        return ActionNumber::TSP_SOLUTION;
    }

    return ActionNumber::ERROR_ALGORITHM;
}

void GraphView::prepareContextMenu()
{
    m_rightClickMenu = new QMenu();
    QMenu* menu_generateGraph = m_rightClickMenu->addMenu("Generate Graph");
    QMenu* menu_algorithms = m_rightClickMenu->addMenu("Algorithms");
    QMenu* menu_options = m_rightClickMenu->addMenu("Options");
    //QMenu* menu_generateGraph_selectOptions = menu_generateGraph->addMenu("Random Generate");

    menu_generateGraph->addAction("Generate new graph");
    menu_generateGraph->addAction("Input adj matrix");

    menu_algorithms->addAction(QString("Find all paths"));
    menu_algorithms->addAction(QString("Floyd Warshall"));
    menu_algorithms->addAction(QString("Dijkstra's algorithm"));
    menu_algorithms->addAction(QString("Bellman-Ford algorithm"));
    menu_algorithms->addAction(QString("Ford-falkerson algorithm"));
    menu_algorithms->addAction(QString("Min-Cost-Max-Flow algorithm"));

    QMenu *menu_algorithms_sub_menu0 = menu_algorithms->addMenu("Span trees");
    menu_algorithms_sub_menu0->addAction(QString("Prima algorithm"));
    menu_algorithms_sub_menu0->addAction(QString("Kruskal algorithm"));
    menu_algorithms_sub_menu0->addAction(QString("Decode Prufer code"));
    //menu_algorithms->addAction(QString("Boruvka algorithm (in develop)"));

    QMenu* menu_algorithms_sub_menu1 = menu_algorithms->addMenu("Cycles");
    menu_algorithms_sub_menu1->addAction("Is Hamiltonian");
    menu_algorithms_sub_menu1->addAction("Is Eiler");
    menu_algorithms_sub_menu1->addAction("Build to Hamiltonian");
    menu_algorithms_sub_menu1->addAction("Build to Eiler");
    menu_algorithms_sub_menu1->addAction("TSP solve");


    menu_options->addAction(QString("Clear log output"));

}


GraphView::GraphView(QGraphicsScene* scene,Graph* G):QGraphicsView(scene){
    this->m_G1=G;
    scene->clear();
    prepareContextMenu();
    this->setRenderHint(QPainter::Antialiasing);
}

GraphView::~GraphView()
{
    for(QAction* act : m_actions){
        delete act;
    }
}

QMenu *GraphView::rightClickMenu()
{
    return m_rightClickMenu;
}

void GraphView::contextMenuEvent(QContextMenuEvent *event)
{
    m_rightClickMenu->exec(event->globalPos());
}


