#include "GraphScene.h"


const std::vector<GraphNode *> &GraphScene::selectedNodes() const
{
    return m_selectedNodes;
}

void GraphScene::updateSelectedNodes(GraphNode *gv1)
{
    if(gv1 == nullptr){
        m_selectedNodes.resize(0);
        return;
    }

    m_selectedNodes.push_back(gv1);
    for(int i = 0; i < static_cast<int>(m_selectedNodes.size()-2);i++){
        m_selectedNodes[i]->setColor(gv1->getColorList().at(0));
    }

    if(m_selectedNodes.size() >= 2){
        m_selectedNodes[m_selectedNodes.size()-1]->setColor(gv1->getColorList().at(1));
        m_selectedNodes[m_selectedNodes.size()-2]->setColor(gv1->getColorList().at(2));
    }
    else{
        m_selectedNodes.back()->setColor(gv1->getColorList().at(1));
    }



    this->update(0,0,1000,1000);
}

GraphScene::GraphScene(QObject *parent)
{
    //  this->setBackgroundBrush(QBrush(QImage())
    this->setSceneRect(delta.x(),delta.y(),600,600);
    int MAX_CAPACITY = 101;
    //m_createdEdges.resize(MAX_CAPACITY);
    //yet idk what purpose to let it at code
    m_createdNodes.resize(MAX_CAPACITY,nullptr);


}

void GraphScene::drawGraph(Graph *G)
{
    //draw Nodes first of all
    m_selectedNodes.clear();
    this->items().clear();
    this->clear();
    m_createdNodes.clear();
    m_createdNodes.resize(100,nullptr);
    m_createdEdges.clear();
    for(Node* n1 : G->nodes()){
        GraphNode* graphic_node = new GraphNode(n1,this,nullptr);
        this->addItem(graphic_node);
        m_createdNodes[n1->numeric()] = graphic_node;
    }
    //after we drew all nodes, we need to replace them in circle
    QPointF centerOfScene = this->sceneRect().center();

    qreal radiusOfCircle = 100*(G->nodes().size()*50/this->sceneRect().size().width());
    double omega = M_PI/2;

    QPointF curNodePoint = {centerOfScene.x()+cos(omega)*radiusOfCircle,
                               centerOfScene.y()+sin(omega)*radiusOfCircle};//start point of nodes is pi/2 angle at circle with radius=radius
    int count = 0; //count of created nodes to switch omega angle

    for(int i = 0;i < static_cast<int>(m_createdNodes.size());i++){
        curNodePoint = {centerOfScene.x()+cos(omega)*(30+radiusOfCircle),
                                   centerOfScene.y()+sin(omega)*(30+radiusOfCircle)};
        if(m_createdNodes[i] != nullptr){
            m_createdNodes[i]->setPos(curNodePoint);
            count++;

            if(count % 2 != 0){
                omega += M_PI;
            }

            if(count % 2 == 0){
                omega += M_PI+M_PI/6;
            }

            if(count % 13 == 0){
                radiusOfCircle -= 100;
            }


        }
    }


    for(Edge* e1 : G->edges()){
        GraphNode* graphic_node1 = m_createdNodes[e1->startVertex()->numeric()];
        GraphNode* graphic_node2 = m_createdNodes[e1->finishVertex()->numeric()];

        int bandwidth=-1;
        if(G->isWeb()){
            bandwidth = G->bandwidthMatrix().getAtPos(e1->startVertex()->numeric(),e1->finishVertex()->numeric());
        }

        GraphEdge* graphic_edge = new GraphEdge(graphic_node1,
                                                graphic_node2,
                                                e1,bandwidth);
        this->addItem(graphic_edge);
        m_createdEdges.push_back(graphic_edge);
    }

}

void GraphScene::recolorEdges(std::vector<Edge *> edges)
{
    for(Edge* e : edges){
        GraphEdge* my_ge=getGraphEdgeByParent(e);
        if(!e->direction()){
            Node* tmp_node_s = new Node(e->startVertex()->numeric());
            Node* tmp_node_f = new Node(e->finishVertex()->numeric());

            Edge* tmp_edge = new Edge(tmp_node_f,tmp_node_s,e->direction(),e->weight());
            GraphEdge* my_tmp_ge = getGraphEdgeByParent(tmp_edge);
            if(my_tmp_ge != nullptr){
                my_tmp_ge->setColor(QColor(Qt::red));
            }
            delete tmp_edge;
            delete tmp_node_f;
            delete tmp_node_s;
        }


        if(my_ge != nullptr){
            my_ge->setColor(QColor(Qt::red));
        }
    }
    this->update();
}

void GraphScene::recolorEdges()
{
    for(GraphEdge* ge : m_createdEdges){
        ge->setColor(Qt::black);
        ge->setZValue(50);
    }
    this->update();
}

GraphEdge *GraphScene::getGraphEdgeByParent(Edge *e)
{
    for(GraphEdge* ge : m_createdEdges){


        if(ge->parentEdge()==e){
            return ge;
        }
    }
    return nullptr;
}

void GraphScene::removeNotRed(std::vector<Edge*> edges)
{
    for(GraphEdge* ge : m_createdEdges){
        bool inEdge = false;
        for(Edge* e :edges){
            if(ge->parentEdge()==e){
                inEdge=true;
                break;
            }
        }
        if(inEdge != true){
            ge->setColor(QColor(0,0,0,0));
        }
    }
}
