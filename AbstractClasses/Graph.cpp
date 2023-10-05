#include "Graph.h"
#include "settings.h"
#include <stack>
#include <queue>
#include <QDebug>


double bernoulli_distribution()
{
    srand(rand());
    double R = (double)(rand()) / ((double)(RAND_MAX));
    return R > 0.5 ? 0.6 : 0.4;
}


bool Graph::weighted() const
{
    return m_weighted;
}

const std::vector<Node *> &Graph::nodes() const
{
    return m_nodes;
}

int Graph::getDegreeNode(Node *v1)
{
    return adjList().getVertexByNumeric(v1->numeric()).size();
}

int Graph::getDegreeNode(int numeric_v1)
{
    return adjList().getVertexByNumeric(numeric_v1).size();
}

const std::vector<Edge *> &Graph::edges() const
{
    return m_edges;
}

std::vector<std::vector<int>>Graph::adjMatrixVector()
{
    std::vector<std::vector<int>> result;
    for(Node* n1 : m_nodes){
        std::vector<int>tmp;
        for(Node* n2 : m_nodes){
            tmp.push_back(m_adjMatrix.getAtPos(n1->numeric(),n2->numeric()));
        }
        result.push_back(tmp);
    }
    return result;
}


void Graph::addNode()
{
    int new_numeric=-1;
    if(edges().size()==100){
        return;
    }
    std::vector<bool>nodes_numerals(m_adjMatrix.getMaxSize(),false);

    for(auto i = 0; i < static_cast<int>(m_nodes.size());i++){
        nodes_numerals[m_nodes[i]->numeric()]=true;
    }

    for(auto i = 0; i < m_adjMatrix.getMaxSize();i++){
        if(nodes_numerals[i]==false){
           new_numeric=i;
           break;
        }
    }
    if(new_numeric==-1){
        //maximum size of vertexes
        return;
    }
    Node* v1 = new Node(new_numeric);
    m_nodes.push_back(v1);

    rebuildAbstractStructres();
}

void Graph::addEdge(Node *n1, Node *n2,int weight)
{
    Edge* e = new Edge(n1,n2,false,weight); //make edge
    m_edges.push_back(e); //add our edge
    rebuildAbstractStructres(); //rebuild matrix and adj list to fit Graph
}

void Graph::addEdge(int numeric_v1, int numeric_v2,int weight)
{
    Node* v1 = nullptr;
    Node* v2 = nullptr;

    for(Node* v : m_nodes){
        if(v->numeric()==numeric_v1){
            v1 = v;
        }
        if(v->numeric()==numeric_v2){
            v2 = v;
        }
    }

    if(v1 == nullptr || v2 == nullptr){
        return;
    }
    addEdge(v1,v2,weight);
}

void Graph::removeNode(int numeric)
{
    for(int i = 0; i<static_cast<int>(m_nodes.size());i++){
        if(m_nodes[i]->numeric()==numeric){
            removeEdgesByNode(m_nodes[i]);
            delete m_nodes[i];
            m_nodes.erase(m_nodes.begin()+i);
            rebuildAbstractStructres();
            return;
        }
    }


}

void Graph::removeEdge(Node *startV, Node *finishV)
{
    for(int i = 0;i < static_cast<int>(m_edges.size());i++){
        if(m_edges[i]->startVertex()==startV && m_edges[i]->finishVertex()==finishV){
            delete m_edges[i];
            m_edges.erase(m_edges.begin()+i);
            rebuildAbstractStructres();
            return;
        }
    }
}


void Graph::removeEdgesByNode(Node *v1)
{
    std::vector<Edge*>to_erase;
    for(int i = 0;i < static_cast<int>(m_edges.size());i++){
        if(m_edges[i]->startVertex() == v1 || m_edges[i]->finishVertex() == v1){
            to_erase.push_back(m_edges[i]);
        }
    }


    if(!to_erase.empty()){
        while(!to_erase.empty()){
            for(int i = 0; i < static_cast<int>(m_edges.size()); i++){
                if(m_edges[i] == to_erase.back()){
                    delete m_edges[i];
                    m_edges.erase(m_edges.begin()+i);
                    break;
                }
            }
            to_erase.pop_back();
        }
        rebuildAbstractStructres();
    }





}


std::vector<int> Graph::getNumericsOfNodes()
{
    std::vector<int>tmp;
    for(Node* v1 : m_nodes){
        tmp.push_back(v1->numeric());
    }
    return tmp;
}

AdjencyList Graph::adjList()
{
    return m_adjList;
}

const AdjencyMatrix &Graph::adjMatrix() const
{
    return m_adjMatrix;
}


void Graph::generateGraph(int vertexCount, bool random, bool acyclic, bool linked,bool webmode)
{

    while(!m_nodes.empty()){
        delete m_nodes.back();
        m_nodes.pop_back();
    }

    while(!m_edges.empty()){
        delete m_edges.back();
        m_edges.pop_back();
    }

    for(int i = 0;i < vertexCount;i++){
        addNode();
    }


    if(vertexCount==0){
        return;
    }

    //int base_degree = linked ? 1 : 0; // base degrees of vertex depend on linked option
    int base_degree = 0;

    std::vector<int>tmp_degreesOfVertex(vertexCount,base_degree);


    //int vertex_coefficient = acyclic ? vertexCount / 2 : vertexCount;
    if(random == false){
        rebuildAbstractStructres();
        return;
    }
    //generate degrees if random

    for(int i = 0;i<vertexCount;i++){
        int extra_val = acyclic ? i : 0;
        if(random || webmode){
            tmp_degreesOfVertex[i] += bernoulli_distribution() * (vertexCount - extra_val/2);
            if(!m_oriented){
                tmp_degreesOfVertex[i]/=2;
            }
            if(tmp_degreesOfVertex[i] == 0){
               tmp_degreesOfVertex[i]+=2;
            }
        }

    }


    //output degrees:
    for(int degree : tmp_degreesOfVertex){
        qDebug() << "degree: " << degree;
    }

    for(int i = 0; i <static_cast<int>( m_nodes.size());i++){
        int created_edges_current = 0;
        int t = acyclic ? i : 0;
        int j = t;
        while(tmp_degreesOfVertex[i] > 0){
            if(j != i && !isConnected(i,j) && bernoulli_distribution()> 0.5){
                int random_weight = weighted() ? static_cast<int>((double)(rand()) / ((double)(RAND_MAX))*100) : 1;
                addEdge(i,j,random_weight+1);
                tmp_degreesOfVertex[i]--;

                if(!m_oriented){
                    tmp_degreesOfVertex[j]--;
                }
            }
            if(j == static_cast<int>(m_nodes.size())){
                j = t;
                continue;
            }
            j++;
        }
    }

    /*
    std::vector<int>free_degrees=tmp_degreesOfVertex;

    for(int i =0;i<static_cast<int>(m_nodes.size());i++){
        int t = acyclic ? i : 0;
        int j = t;
        int safe=0;
        while(free_degrees[i]>0){
            if(j != i && !isConnected(i,j) && bernoulli_distribution()>0.5){
                int random_weight = weighted() ? static_cast<int>((double)(rand()) / ((double)(RAND_MAX))*100) : 1;
                if(!m_oriented && free_degrees[j]>0){
                    addEdge(i,j,random_weight+1);
                    free_degrees[i]--;
                    free_degrees[j]--;
                }
                else if(m_oriented){
                    addEdge(i,j,random_weight+1);
                    free_degrees[i]--;
                }
            }
            if(j == static_cast<int>(m_nodes.size())){
                j = t;
                continue;
            }
            j++;
            safe++;
            if(safe>10000){
                break;
            }
        }

    }
*/

    std::vector<std::vector<int>>tmp_matrix(100,std::vector<int>(100,0));
    std::vector<bool>visited(100,false);

    for(Edge* e1 : m_edges){
        tmp_matrix[e1->startVertex()->numeric()][e1->finishVertex()->numeric()] = 1;
        if(m_oriented==false){
            tmp_matrix[e1->finishVertex()->numeric()][e1->startVertex()->numeric()] = 1;
        }
            //qDebug() << e1->startVertex()->numeric() << "->" << e1->finishVertex()->numeric();
    }

    std::stack<int> s;
    s.push(m_nodes[0]->numeric());
    visited[s.top()]=true;
    while(!s.empty()){
        visited[s.top()] = true;
        bool newvertex = false;
        for(int i = 0;i < static_cast<int>(tmp_matrix[s.top()].size());i++){
            if(visited[i]==false && tmp_matrix[s.top()][i] == 1){
                s.push(i);
                newvertex = true;
                break;
            }
        }

        if(newvertex){continue;}

        s.pop();

    }

    bool pure_linked = true;

    for(Node* v1 : m_nodes){
        if(visited[v1->numeric()] == false){
            pure_linked = false;
            qDebug() << "NOT LINKED GRAPH WAS GENERATE ";
        }
    }

    if(!pure_linked){
        this->generateGraph(vertexCount,random,acyclic,linked,webmode);
        rebuildAbstractStructres();
        return;
    }

    if(webmode){
        for(int i = 1; i < static_cast<int>(m_nodes.size());i++){
            int zero_column = 0,zero_row=0;
            for(int j = 0;j<static_cast<int>(m_nodes.size());j++){
                //qDebug() << "tmp_matrix["<<j<<"]["<<i<<"]: " << tmp_matrix[j][i];
                if(tmp_matrix[j][i] == 0){
                    zero_column++;
                }
                if(tmp_matrix[i][j] == 0 && i != vertexCount-1){
                    zero_row++;
                }
            }
            qDebug() << "Count of zeros in column " << i << " :" << zero_column;
            qDebug() << "Count of zeros in row " << i << " :" << zero_row;
            if(zero_column == static_cast<int>(m_nodes.size()) ||
               zero_row == static_cast<int>(m_nodes.size())
                    ){
                int random_weight = weighted() ? static_cast<int>((double)(rand()) / ((double)(RAND_MAX))*100) : 1;

                // find minimal degree
                int index=-1;
                int minDegree=static_cast<int>(m_nodes.size())+1000;
                for(int k = 0;k<static_cast<int>(tmp_degreesOfVertex.size());k++){
                    if(tmp_degreesOfVertex[k] < minDegree && i != k){
                        minDegree=tmp_degreesOfVertex[k];
                        index = k;
                    }
                }

                //
                this->addEdge(i,index,random_weight);
                tmp_matrix[i][index]=random_weight;
                tmp_degreesOfVertex[index]++;
                qDebug() << "Make a WEB: " << i << " connect with " << index << "(weight is " << random_weight << ")";
            }
        }
    }


    //make matrix of bandwidth if web
    if(webmode){
        for(int i = 0;i < static_cast<int>(m_nodes.size());i++){
            for(int j = 0;j<static_cast<int>(m_nodes.size());j++){
                if(tmp_matrix[i][j] != 0){
                    int random_weight = weighted() ? static_cast<int>((double)(rand()) / ((double)(RAND_MAX))*100) : 1;
                    m_bandwidthMatrix.setAtPos(i,j,random_weight);
                }
            }
        }

        m_isWeb=true;
    }

    rebuildAbstractStructres();
}

bool Graph::isLinked(std::vector<std::vector<int> > graph)
{

    //check for in value
    if(graph.empty()){
        graph = adjMatrixVector();
    }

    int numVertices = graph.size();
    std::vector<bool> visited(numVertices, false);

    // Находим первую непосещенную вершину
    int startVertex = -1;
    for (int i = 0; i < numVertices; i++) {
        bool hasNeighbor = false;
        for (int j = 0; j < numVertices; j++) {
            if (graph[i][j]) {
                hasNeighbor = true;
                break;
            }
        }
        if (hasNeighbor) {
            startVertex = i;
            break;
        }
    }

    if (startVertex == -1) {
        // Граф пустой, считаем его связным
        return true;
    }

    // Выполняем глубинный обход для проверки связности
    dfs(startVertex, graph, visited);

    // Проверяем, что все вершины были посещены
    for (bool isVisited : visited) {
        if (!isVisited) {
            return false; // Граф не связный
        }
    }

    return true; // Граф связный
}

void Graph::dfs(int vertex, std::vector<std::vector<int> > &graph, std::vector<bool> &visited)
{
    std::stack<int> st;
    st.push(vertex);
    visited[vertex] = true;

    while (!st.empty()) {
        int currentVertex = st.top();
        st.pop();

        // Обрабатываем текущую вершину

        for (int neighbor = 0; neighbor < graph.size(); ++neighbor) {
            if (graph[currentVertex][neighbor] && !visited[neighbor]) {
                visited[neighbor] = true;
                st.push(neighbor);
            }
        }
    }
}

void Graph::findAllPaths(int u, int v, std::vector<int> &path, std::vector<std::vector<int> > &allPaths)
{
    // Добавляем вершину u в путь
       path.push_back(u);

       // Если u равно v, то добавляем текущий путь в список путей
       if (u == v)
       {
           allPaths.push_back(path);
       }
       else
       {
           // Рекурсивно вызываем функцию для каждого соседа вершины u
           for (int i = 0; i < m_adjList.getVertexByNumeric(u).size(); i++)
           {
               int neighbor = m_adjList.getVertexByNumeric(u)[i]->numeric();
               if (find(path.begin(), path.end(), neighbor) == path.end()) // Проверяем, не посещали ли мы уже эту вершину
               {
                   findAllPaths(neighbor, v, path, allPaths);
               }
           }
       }

       // Удаляем вершину u из пути, чтобы продолжить поиск других путей
       path.pop_back();
}


std::vector<int> Graph::floydWarshall(int u, int v, std::vector<std::vector<int>> &matrixOfPaths, std::vector<std::vector<int>> &matrixOfWeight)
{
    matrixOfWeight = m_adjMatrix.getPureMatrixByVector();
    for(int i = 0; i < static_cast<int>(matrixOfWeight.size());i++){
        for(int j = 0; j < static_cast<int>(matrixOfWeight[i].size());j++){
            matrixOfWeight[i][j] = matrixOfWeight[i][j] == 0 ? INT_FAST16_MAX : matrixOfWeight[i][j];
        }

    }

    matrixOfPaths.resize(matrixOfWeight.size(),std::vector<int>(matrixOfWeight.size(),0));

    for(Edge* e1 : m_edges){
        matrixOfPaths[e1->startVertex()->numeric()][e1->finishVertex()->numeric()] = e1->finishVertex()->numeric();
    }

    for(Node* n1 : m_nodes){
        matrixOfPaths[n1->numeric()][n1->numeric()] = n1->numeric();
    }


    for(int k = 0;k < static_cast<int>(matrixOfWeight.size());k++){
        for(int i = 0;i < static_cast<int>(matrixOfWeight.size());i++){
            for(int j = 0;j < static_cast<int>(matrixOfWeight.size());j++){
                int old_value = matrixOfWeight[i][j];
                matrixOfWeight[i][j] = std::min(matrixOfWeight[i][j],matrixOfWeight[i][k]+matrixOfWeight[k][j]);
                if(old_value != matrixOfWeight[i][j]){
                    matrixOfPaths[i][j] = matrixOfPaths[i][k];
                    }
           }
        }
    }

    //find Path from u to v
    std::vector<int> path;
    if(matrixOfPaths[u][v] == 0){
        return path;
    }

    path.push_back(u);
    while(u != v){
        u = matrixOfPaths[u][v];
        path.push_back(u);
    }
    return path;

}


std::vector<int> Graph::floydWarshallGetPath(int u, int v)
{
    //idk why its need
    Q_UNUSED(u);
    Q_UNUSED(v);
    std::vector<int>vec;
    return vec;
}

std::vector<int> Graph::dijkstraAlgorithm(int u, int v, std::vector<std::pair<int,bool>>& vectorShortestPath)
{
    //initialize vector
    int count = 0; //iterations count;
    vectorShortestPath.resize(m_nodes.size(),std::pair<int,bool>(INT_FAST16_MAX,false));
    vectorShortestPath[u].first = 0;
    std::vector<int> parents(m_nodes.size(),-1); // vector to find path

    std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>,std::greater<std::pair<int,int>>> pq;
    pq.push(std::make_pair(0,u));

    while(!pq.empty()){
        count++;
        int cur_vertex = pq.top().second;
        pq.pop();

        //if we have already visited that vertex
        if(vectorShortestPath[cur_vertex].second == true){
            continue;
        }
        vectorShortestPath[cur_vertex].second = true;

        for(int i = 0;i < static_cast<int>(m_adjMatrix.getPureMatrixByVector().size());i++){
           if(m_adjMatrix.getAtPos(cur_vertex,i) != 0){
               count++;
               int next_vertex = i;
               int weight = m_adjMatrix.getAtPos(cur_vertex,i);
               if(vectorShortestPath[next_vertex].first > vectorShortestPath[cur_vertex].first + weight){
                  vectorShortestPath[next_vertex].first = vectorShortestPath[cur_vertex].first + weight;
                  parents[next_vertex] = cur_vertex;
                  pq.push(std::make_pair(vectorShortestPath[next_vertex].first,next_vertex));
                  count++;
               }
           }
        }

    }

    std::vector<int>path;
    //find path to v;
    if(vectorShortestPath[v].first == INT_FAST16_MAX){
        path.push_back(count);
        return path; //no path
    }

    for (int i = v; i != u; i = parents[i]) {
        path.push_back(i);
    }
    path.push_back(u);
    reverse(path.begin(), path.end());
    path.push_back(count);
    return path;

}

std::vector<int> Graph::bellmanFordAlgorithm(int u, int v, std::vector<std::pair<int, bool> > &vectorShortestPath)
{
    int count = 0; //iterations count;
    vectorShortestPath.resize(m_nodes.size(),std::pair<int,bool>(INT_FAST16_MAX,false));
    vectorShortestPath[u].first = 0;
    std::vector<int> parents(m_nodes.size(),-1); // vector to find path

    std::queue<int>q;
    std::vector<bool>in_queue(m_adjMatrix.getMaxSize(),false);

    q.push(u);
    in_queue[u]=true;

    while(!q.empty()){
        int cur_vertex = q.front();
        q.pop();
        count++;
        in_queue[cur_vertex] = false;
        for(int i = 0;i < static_cast<int>(m_adjMatrix.getPureMatrixByVector().size());i++){
           if(m_adjMatrix.getAtPos(cur_vertex,i) != 0){
               count++;
               //int next_vertex = i;
               int weight = m_adjMatrix.getAtPos(cur_vertex,i);
               if(vectorShortestPath[cur_vertex].first != INT_FAST16_MAX &&
                       vectorShortestPath[cur_vertex].first + weight < vectorShortestPath[i].first){
                   vectorShortestPath[i].first = vectorShortestPath[cur_vertex].first + weight;
                   parents[i] = cur_vertex;

                   if(!in_queue[i]){
                       count++;
                       q.push(i);
                       in_queue[i] = true;
                   }
               }
           }
        }
    }

    std::vector<int>path;
    //find path to v;
    if(vectorShortestPath[v].first == INT_FAST16_MAX){
        path.push_back(count);
        return path; //no path
    }

    for (int i = v; i != u; i = parents[i]) {
        path.push_back(i);
    }
    path.push_back(u);
    reverse(path.begin(), path.end());
    path.push_back(count);
    return path;



}

int Graph::fordFalkersonAlgorithm(int source, int sink)
{
    int vertices = m_nodes.size();

    // Создаем остаточную сеть, инициализируем ее значениями исходного графа
    std::vector<std::vector<int>> residualGraph(vertices, std::vector<int>(vertices, 0));
    for (int i = 0; i < vertices; i++) {
        for (int j = 0; j < vertices; j++) {
            residualGraph[i][j] = bandwidthMatrix().getAtPos(i,j);
        }
    }

    std::vector<int> parent(vertices, -1);  // Содержит путь от источника к вершине

    int maxFlow = 0;  // Инициализация максимального потока

    // Находим увеличивающий путь и увеличиваем поток, пока они существуют
    while (fordFalkersonBFS(residualGraph, source, sink, parent)) {
        int pathFlow = INT_FAST16_MAX;  // Наименьшая пропускная способность на пути

        // Находим наименьшую пропускную способность ребер на пути
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            pathFlow = std::min(pathFlow, residualGraph[u][v]);
        }

        // Обновляем остаточные пропускные способности ребер и обратных ребер на пути
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            residualGraph[u][v] -= pathFlow;
            residualGraph[v][u] += pathFlow;
        }

        maxFlow += pathFlow;  // Увеличиваем максимальный поток
    }

    return maxFlow;
}

bool Graph::fordFalkersonBFS(std::vector<std::vector<int> > &residualGraph, int source, int sink, std::vector<int> &parent)
{
    int vertices = residualGraph.size();
    std::vector<bool> visited(vertices, false);
    std::queue<int> q;

    q.push(source);
    visited[source] = true;
    parent[source] = -1;

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int v = 0; v < vertices; v++) {
            if (!visited[v] && residualGraph[u][v] > 0) {
                q.push(v);
                parent[v] = u;
                visited[v] = true;
            }
        }
    }

    return visited[sink];
}

int Graph::minCostMaxFlowAlgorithm(std::vector<std::vector<int> > bandwidth, int source, int sink)
{
    int vertices = m_nodes.size();
    int flow = fordFalkersonAlgorithm(source,sink);
    flow *= (2/3.0);
    std::vector<std::vector<int>> residualGraph(vertices, std::vector<int>(vertices, 0));

    // Построение остаточного графа
    for (int i = 0; i < vertices; i++) {
        for (int j = 0; j < vertices; j++) {
            if (bandwidth[i][j] > 0) {
                residualGraph[i][j] = bandwidth[i][j];
                residualGraph[j][i] = -bandwidth[i][j];
            }
        }
    }

    int maxFlow = 0;
    int minCost = 0;

    while (true) {
        // Находим кратчайший путь с помощью алгоритма Дейкстры
        std::vector<int> parent = dijkstraMinCost(residualGraph, source);
        if (parent[sink] == -1)
            break;

        // Находим максимальный поток вдоль найденного пути
        int pathFlow = INT_FAST16_MAX;
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            pathFlow = std::min(pathFlow, residualGraph[u][v]);
        }

        if(maxFlow + pathFlow > flow){
            pathFlow = flow-maxFlow;
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                residualGraph[u][v] -= pathFlow;
                residualGraph[v][u] += pathFlow;
                minCost += pathFlow * adjMatrix().getAtPos(u,v);
                qDebug() << "mincost (editable): " << pathFlow << " " << adjMatrix().getAtPos(u,v) << "= " << pathFlow*adjMatrix().getAtPos(u,v);
            }
            break;
        }
        else{
            // if when we add pathflow smaller than flow(2/3)
            // Обновляем матрицу и стоимость
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                residualGraph[u][v] -= pathFlow;
                residualGraph[v][u] += pathFlow;
                minCost += pathFlow * adjMatrix().getAtPos(u,v);
                qDebug() << "mincost: " << pathFlow << " " << adjMatrix().getAtPos(u,v) << "= " << pathFlow*adjMatrix().getAtPos(u,v);
            }

            maxFlow += pathFlow;
        }

    }

    return minCost;
}

int Graph::twoThirdsMaxFlowAlgorithm(int source, int sink)
{
    // Находим максимальный поток
    int maxFlow = fordFalkersonAlgorithm(source, sink);

    // Вычисляем поток, равный 2/3 от максимального потока
    int desiredFlow = (2 * maxFlow) / 3;
    //qDebug()<<desiredFlow;
    // Обновляем пропускные способности для достижения желаемого потока
    std::vector<std::vector<int>> updatedBandwidth(m_nodes.size(),std::vector<int>(m_nodes.size(),0));
    int n = m_nodes.size();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            updatedBandwidth[i][j] = std::min(bandwidthMatrix().getAtPos(i,j), desiredFlow);
        }
    }

    // Вызываем функцию для поиска минимального стоимостного потока
    int minCost = minCostMaxFlowAlgorithm(updatedBandwidth, source, sink);

    return minCost;
}

std::vector<int> Graph::dijkstraMinCost(std::vector<std::vector<int> > &residualGraph, int source)
{
    int n = m_nodes.size();
    std::vector<int> dist(n, INT_FAST16_MAX);
    std::vector<int> parent(n, -1);
    std::vector<bool> visited(n, false);

    dist[source] = 0;

    for (int i = 0; i < n - 1; ++i) {
        int u = -1;
        for (int j = 0; j < n; ++j) {
            if (!visited[j] && (u == -1 || dist[j] < dist[u]))
                u = j;
        }

        visited[u] = true;

        for (int v = 0; v < n; ++v) {
            if (!visited[v] && residualGraph[u][v] && dist[u] != INT_FAST16_MAX &&
                dist[u] + adjMatrix().getAtPos(u,v) < dist[v] && residualGraph[u][v] > 0) {
                dist[v] = dist[u] + adjMatrix().getAtPos(u,v);
                parent[v] = u;
            }
        }
    }

    return parent;
}

std::vector<Edge *> Graph::primaMinSpanTree(int startVertex)
{

    m_lastIterations=0;
    std::vector<Edge*> mst;
    if (m_nodes.empty()) {
        return mst;
    }

    std::vector<bool> visited(Settings::MAX_COUNT_OF_NODES, false);
    std::priority_queue<Edge*, std::vector<Edge*>, EdgeCompare> minHeap;

    Node* start = this->getNodeByNumeric(startVertex);
    if(start == nullptr){
        return mst;
    }

    for (Edge* edge : getNeighboursEdges(start)) {
        minHeap.push(edge);
    }
    visited[start->numeric()] = true;

    while (!minHeap.empty()) {
        Edge* smallest = minHeap.top();
        minHeap.pop();
        m_lastIterations++;
        Node* node = visited[smallest->startVertex()->numeric()] ? smallest->finishVertex() : smallest->startVertex();

        if (!visited[node->numeric()]) {
            visited[node->numeric()] = true;
            mst.push_back(smallest);

            for (Edge* edge : getNeighboursEdges(node)) {
                if (!visited[edge->startVertex()->numeric()] || !visited[edge->finishVertex()->numeric()]) {
                    minHeap.push(edge);
                }
                m_lastIterations++;
            }
        }
    }

    return mst;
}

std::vector<Edge *> Graph::kruskalMinSpanTree(int startVertex)
{
    Q_UNUSED(startVertex);
    m_lastIterations=0;
    std::vector<Edge*> mst;
    UnionFind uf(m_nodes.size());


    std::priority_queue<Edge*, std::vector<Edge*>, EdgeCompare> minHeap;
    for (Edge* edge : m_edges) {
        minHeap.push(edge);
    }

    while (!minHeap.empty()) {
        Edge* edge = minHeap.top();
        minHeap.pop();
        m_lastIterations++;
        int nodeA = edge->startVertex()->numeric();
        int nodeB = edge->finishVertex()->numeric();
        if (uf.find(nodeA) != uf.find(nodeB)) {
            uf.merge(nodeA, nodeB);
            mst.push_back(edge);
        }
    }

    return mst;
}

int Graph::countOfSpanTrees()
{
    std::vector<std::vector<double>>matrix_k = this->toShortAdjMatrixDouble(m_kirchhoffMatrix);
    int n = matrix_k.size();

    /*
    for(int i = 0;i<n;i++){
        QString t = "";
        for(int j = 0;j<n;j++){
            t+=QString().setNum(matrix_k[i][j])+" ";
        }
        qDebug()<<t;
    }*/
    matrix_k=getMatrixWithoutRowAndCol(matrix_k,0,0);
    double ans = determinant(matrix_k,n);

    return ans;
}

std::vector<int> Graph::pruferCode(std::vector<Edge *> spanTree)
{
    int size = m_nodes.size();
    std::vector<std::vector<int>> adjacency_matrix(size,std::vector<int>(size,0));

    for(Edge* e : spanTree){
        adjacency_matrix[e->startVertex()->numeric()][e->finishVertex()->numeric()]=1;
        adjacency_matrix[e->finishVertex()->numeric()][e->startVertex()->numeric()]=1;
    }
    int n = adjacency_matrix.size(); // Количество вершин в дереве
    std::vector<int> code;

    std::vector<int> degrees(n, 0);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            degrees[i] += adjacency_matrix[i][j];
        }
    }

    std::vector<int>nweights;

    for (int k = 0; k < n - 2; k++) {
        int leaf = -1;

        for (int i = 0; i < n; i++) {
            if (degrees[i] == 1) {
                leaf = i;
                break;
            }
        }

        int neighbor = -1;
        //if leaf ==-1 bBUG
        for (int j = 0; j < n; j++) {
            if (adjacency_matrix[leaf][j] == 1) {
                neighbor = j;
                break;
            }
        }

        code.push_back(neighbor);
        nweights.push_back(this->adjMatrix().getAtPos(leaf,neighbor));
        degrees[leaf]--;
        degrees[neighbor]--;
        adjacency_matrix[leaf][neighbor] = 0;
        adjacency_matrix[neighbor][leaf] = 0;
    }

    Edge* laste;
    for(Edge* e : spanTree){
        if(adjacency_matrix[e->startVertex()->numeric()][e->finishVertex()->numeric()] != 0){
            laste=e;
            break;
        }
    }

    nweights.push_back(laste->weight());

    code.insert(code.end(),nweights.begin(),nweights.end());
    return code;
}

std::pair<int, int> Graph::pruferFindLast(std::vector<int> code)
{
    int n = code.size() + 2;
    std::vector<int> degree_counts(n + 2, 1); // Инициализация степеней вершин

    std::vector<std::vector<int>> adjacency_matrix(n + 2, std::vector<int>(n + 2, 0));

    for (int vertex : code) {
        degree_counts[vertex]++;
        adjacency_matrix[vertex][degree_counts[vertex]] = 1;
        adjacency_matrix[degree_counts[vertex]][vertex] = 1;
    }

    int leaf1 = -1, leaf2 = -1;
    for (int i = 1; i <= n + 1; i++) {
        if (degree_counts[i] == 1) {
            if (leaf1 == -1)
                leaf1 = i;
            else {
                leaf2 = i;
                break;
            }
        }
    }

    return std::make_pair(leaf1, leaf2);
}

std::vector<std::vector<int> > Graph::pruferDecode(std::vector<int> code)
{
    int n = nodes().size();  // Количество вершин в дереве
    std::vector<std::vector<int>> result_matrix(n,std::vector<int>(n, 0));

    std::vector<int> degrees(n, 1); // Инициализация степеней вершин

    for (int k = 0; k < n-2;k++) {
        degrees[code[k]]++;
    }

    int indexW = (this->nodes().size())-2;

    for (int k = 0; k < n-2;k++) {
        int vertex = code[k];
        int leaf = -1;

        for (int i = 0; i < n; i++) {
            if (degrees[i] == 1) {
                leaf = i;
                break;
            }
        }

        result_matrix[leaf][vertex] = code[indexW];
        result_matrix[vertex][leaf] = code[indexW];
        degrees[leaf]--;
        degrees[vertex]--;
        indexW++;
    }

    int leaf1 = -1, leaf2 = -1;

    for (int i = 0; i < n; i++) {
        if (degrees[i] == 1) {
            if (leaf1 == -1)
                leaf1 = i;
            else {
                leaf2 = i;
                break;
            }
        }
    }

    result_matrix[leaf1][leaf2] = code.back();
    result_matrix[leaf2][leaf1] = code.back();



    return result_matrix;

}

std::vector<Edge *> Graph::findHamiltonianCycle()
{
    int V = m_nodes.size();

    // Создаем вектор для хранения пути


    // Проверяем гамильтонов цикл для каждой начальной вершины
    std::vector<Edge*> cycle;
    for (int start = 0; start < V; start++) {
        std::vector<int> path(V, -1);
        // Помещаем начальную вершину в путь
        path[0] = start;

        // Проверяем, существует ли гамильтонов цикл, начиная с данной вершины
        if (hamiltonianCycleUtil(path, 1)) {
            // Создаем вектор ребер для представления гамильтонова цикла
            cycle.clear();
            for (int i = 0; i < V - 1; i++)
                cycle.push_back(
                    getEdgeByNodes(
                        getNodeByNumeric(path[i]),
                        getNodeByNumeric(path[i + 1])));

            cycle.push_back(
                getEdgeByNodes(
                    getNodeByNumeric(path[V - 1]),
                    getNodeByNumeric(path[0])));

            return cycle;
        }
    }

    // Если гамильтонов цикл не найден, возвращаем пустой вектор
    return {};
}

bool Graph::hamiltonianCycleUtil(std::vector<int> &path, int pos)
{
    int V = m_nodes.size();

    // Базовый случай: если все вершины включены в путь
    if (pos == V) {
        // Проверяем, есть ли ребро между последней вершиной и первой вершиной
        if (adjMatrix().getAtPos(path[pos - 1],path[0]) != 0)
            return true;
        else
            return false;
    }

    // Перебираем все вершины, чтобы добавить их в путь
    for (int v = 1; v < V; v++) {
        if (isSafe(v, path, pos)) {
            path[pos] = v;

            // Рекурсивно ищем гамильтонов цикл, начиная с вершины v
            if (hamiltonianCycleUtil(path, pos + 1))
                return true;

            // Если добавление вершины v не приводит к гамильтонову циклу, удаляем ее из пути
            path[pos] = -1;
        }
    }

    return false;
}

bool Graph::isSafe(int v, std::vector<int> &path, int pos)
{
    // Проверяем, есть ли ребро между последней вершиной пути и вершиной v
    if (adjMatrix().getAtPos(path[pos - 1],v) == 0)
        return false;

    // Проверяем, не посещалась ли вершина v ранее
    for (int i = 0; i < pos; i++) {
        if (path[i] == v)
            return false;
    }

    return true;
}

void Graph::buildToHamiltonian()
{
    std::random_device rd;
    // Создаем вектор всех возможных ребер
    std::vector<std::pair<int,int>> allEdges;
    int numVertices = nodes().size();
    for (int i = 0; i < numVertices; i++) {
        for (int j = i + 1; j < numVertices; j++) {
            allEdges.push_back({i,j});
            if(!m_oriented){
                //unoriented:
                allEdges.push_back({j,i});
            }
        }
    }


    std::mt19937 g(rd());
    shuffle(allEdges.begin(), allEdges.end(), g);
    int numEdgesToAdd = 1;
    // Добавляем случайные ребра до достижения необходимого количества
    bool flag = this->findHamiltonianCycle().empty();
    while(flag){
        int oldVal = numEdgesToAdd;
        for (std::pair<int,int> re : allEdges) {
            if (numEdgesToAdd == 0)
                break;

            // Проверяем, что добавляемое ребро не уже существует в графе
            bool edgeExists = false;
            if(adjMatrix().getAtPos(re.first,re.second) != 0){
                edgeExists = true;
            }
            int rw = 1;
            if (!edgeExists) {
                addEdge(re.first,re.second,rw);
                qDebug() << "HAMI GEN" << " "<< re.first << "-> " << re.second;
                numEdgesToAdd--;
                rebuildAbstractStructres();
            }
        }
        numEdgesToAdd = (oldVal + 1) % 5;
        flag = this->findHamiltonianCycle().empty();
    }
}

std::vector<Edge *> Graph::findEulerianCycle()
{
     std::vector<int> eulerCycle;
     int numVertices = nodes().size();
     std::vector<std::vector<int>>graph(numVertices);
     for(int i = 0;i < numVertices;i++){
         //qDebug() << i;
         for(Node* v : adjList().getVertexByNumeric(i)){
             graph[i].push_back(v->numeric());
             //qDebug()<<v->numeric();
         }
         //qDebug()<<"-------";
     }
     int start_vertex = -1;

    // Ищем вершину с нечетной степенью
    for (int i = 0; i < graph.size(); ++i) {
        if (graph[i].size() % 2 != 0) {
            if (start_vertex == -1) {
                start_vertex = i;
            } else {
                // Если мы нашли больше двух вершин с нечетной степенью, то граф не содержит Эйлеров цикл
                return {};
            }
        }
    }

    // Если все вершины имеют четную степень, мы можем начать с любой
    if (start_vertex == -1) {
        for (int i = 0; i < graph.size(); ++i) {
            if (!graph[i].empty()) {
                start_vertex = i;
                break;
            }
        }
    }

    std::vector<int> result;
    eulerUtilDFS(start_vertex, graph, result);

    // Проверяем, что мы прошли по всем ребрам
    for (const auto &edges : graph) {
        if (!edges.empty()) {
            return {}; // Если есть ребра, по которым мы не прошли, возвращаем пустой список
        }
    }

    // Разворачиваем результат, так как мы добавляли вершины в обратном порядке
    reverse(result.begin(), result.end());

     std::vector<Edge*>cycle;
     for(int i = 0;i < result.size()-1;i++){
         cycle.push_back(getEdgeByNodes(result[i],result[i+1]));
     }
     cycle.push_back(getEdgeByNodes(result[numVertices-1],result[0]));

     return cycle;
}

void Graph::eulerUtilDFS(int v, std::vector<std::vector<int> > &graph, std::vector<int> &result)
{
    while (!graph[v].empty()) {
        int next_vertex = graph[v].back();
        graph[v].pop_back();

        // Удаляем ребро из обоих концов
        graph[next_vertex].erase(remove(graph[next_vertex].begin(), graph[next_vertex].end(), v), graph[next_vertex].end());

        eulerUtilDFS(next_vertex, graph, result);
    }
    result.push_back(v);
}


void Graph::buildToEuler()
{
    std::function<int(const std::vector<std::vector<int>>&, int)> degreeAM = [](const std::vector<std::vector<int>>& adjacencyMatrix, int node) {
        int degree = 0;
        int numVertices = adjacencyMatrix.size();

        for (int i = 0; i < numVertices; i++) {
            if (adjacencyMatrix[node][i] != 0) {
                degree++;
            }
        }

        return degree;
    };
    std::vector<std::vector<int>>graph;
    std::vector<std::vector<int>>oldgraph;
    //make adj matrix
    if(isEulerCriterium(graph)){
        return;
    }
    graph=adjMatrixVector();
    oldgraph=graph;
    std::vector<std::pair<int,int>> allEdges;
    std::vector<int>oddV;
    int numVertices = nodes().size();

    std::vector<int>isolatedV;

    for(int i = 0;i < numVertices;i++){
        if(degreeAM(graph,i) % 2 != 0){
            oddV.push_back(i);
        }
        if(degreeAM(graph,i)==1){
            isolatedV.push_back(i);
        }
    }

    //сперва разберемся с висячие вершинами
    if(isolatedV.size()>1){
        for(int i = 0;i < isolatedV.size()-1;i+=2){
            if(i > isolatedV.size()) break;
            graph[isolatedV[i]][isolatedV[i+1]]=1;
            graph[isolatedV[i+1]][isolatedV[i]]=1;
            isolatedV.erase( find(isolatedV.begin(),isolatedV.end(),isolatedV[i]));
            isolatedV.erase( find(isolatedV.begin(),isolatedV.end(),isolatedV[i+1]));
        }
    }
    if(isolatedV.size()==1){
        int isolatedIndex = isolatedV.back();
        for(int i = 0;i <numVertices;i++){
           if(isolatedIndex != i && graph[isolatedIndex][i] == 0){
               graph[isolatedIndex][i] = 1;
               graph[i][isolatedIndex] = 1;
           }
        }
    }
    oddV.clear();
    isolatedV.clear();
    for(int i = 0;i < numVertices;i++){
        if(degreeAM(graph,i) % 2 != 0){
            oddV.push_back(i);
        }
        if(degreeAM(graph,i)==1){
            isolatedV.push_back(i);
        }
    }
    if(isolatedV.empty()){
        qDebug()<<"We removed isolated vertexes";
    }else{
        qDebug()<<"Some wrong with iso vert";
    }




    for (int i = 0; i < numVertices; i++) {
        for (int j = i + 1; j < numVertices; j++) {
            allEdges.push_back({i,j});
            if(!m_oriented){
                //unoriented:
                allEdges.push_back({j,i});
            }
        }
    }
    //connect
    for(auto pairV : allEdges){
        if(graph[pairV.first][pairV.second] == 0){
            bool inOddV1=false,inOddV2=false;

            for(int vindex : oddV){
                if(vindex==pairV.first){
                    inOddV1=true;
                }
                if(vindex==pairV.second){
                    inOddV2=true;
                }
            }

            //если оба значения есть в списке не четных
            if(inOddV1 && inOddV2){
                graph[pairV.first][pairV.second]=1;
                graph[pairV.second][pairV.first]=1;

                //удаляем из oddV
                oddV.erase( find(oddV.begin(),oddV.end(),pairV.first));
                oddV.erase( find(oddV.begin(),oddV.end(),pairV.second));
            }
        }
    }
    if(isEulerCriterium(graph)){
        qDebug()<<"Eulerated with adding";

        for(int i = 0; i < numVertices;i++){
            for(int j = 0;j<numVertices;j++){
                if(graph[i][j] - oldgraph[i][j] != 0){
                    qDebug() << (graph[i][j] - oldgraph[i][j] > 0 ? "We add " : "We remove")
                                                                   << i << " " << j;
                    graph[i][j] = weighted() ? static_cast<int>((double)(rand()) / ((double)(RAND_MAX))*100) : 1;
                }
            }
        }
        rebuildByAdjMatrix(graph);
        rebuildAbstractStructres();

        return;
    }
    oddV.clear();
    for(int i = 0;i < numVertices;i++){
        if(degreeAM(graph,i) % 2 != 0){
            oddV.push_back(i);
        }
    }
    //disconnect
    for(auto pairV : allEdges){
        if(graph[pairV.first][pairV.second] != 0){
            bool inOddV1=false,inOddV2=false;

            for(int vindex : oddV){
                if(vindex==pairV.first){
                    inOddV1=true;
                }
                if(vindex==pairV.second){
                    inOddV2=true;
                }
            }

            //если оба значения есть в списке не четных
            if(inOddV1 && inOddV2){
                graph[pairV.first][pairV.second]=0;
                graph[pairV.second][pairV.first]=0;

                //удаляем из oddV
                oddV.erase( find(oddV.begin(),oddV.end(),pairV.first));
                oddV.erase( find(oddV.begin(),oddV.end(),pairV.second));
            }
        }
    }

    qDebug() << "Full func";
    for(int i = 0; i < numVertices;i++){
        for(int j = 0;j<numVertices;j++){
            if(graph[i][j] - oldgraph[i][j] != 0){
                qDebug() << (graph[i][j] - oldgraph[i][j] > 0 ? "We add " : "We remove")
                                                               << i << " " << j;
            }
            if(graph[i][j] == 1 && oldgraph[i][j] != 1){
               graph[i][j] = weighted() ? static_cast<int>((double)(rand()) / ((double)(RAND_MAX))*100) : 1;
            }
        }
    }
    rebuildByAdjMatrix(graph);
    rebuildAbstractStructres();

}

bool Graph::isEulerCriterium(std::vector<std::vector<int> > graph)
{
    if(graph.empty()){
        graph=adjMatrixVector();
    }
    int count = 0;
    for (int i = 0; i < graph.size(); i++) {
        int degree = 0;
        for (int j = 0; j < graph[i].size(); j++) {
            degree += graph[i][j] == 0 ? 0 : 1;
        }
        if (degree % 2 != 0) count++;
    }
    if (count != 0 || !isLinked(graph)) return false;


    return true;

}

void Graph::hamiltonianDFS(int v, std::vector<std::vector<int> > &adj, std::vector<int> &path, std::vector<bool> &visited, std::vector<std::vector<int> > &cycles)
{
    visited[v] = true;
    path.push_back(v);

    if (path.size() == adj.size()) {
        // Если мы посетили все вершины, проверяем, есть ли ребро между текущей и начальной вершиной
        if (find(adj[v].begin(), adj[v].end(), path[0]) != adj[v].end()) {
            cycles.push_back(path);
        }
    } else {
        for (int u : adj[v]) {
            if (!visited[u]) {
                hamiltonianDFS(u, adj, path, visited, cycles);
            }
        }
    }

    // Если мы дошли до этой точки, уходим из текущей вершины и ищем другие пути
    visited[v] = false;
    path.pop_back();
}

std::vector<std::vector<int> > Graph::allHamiltonianCycles(std::vector<std::vector<int> > &adj)
{
    int n = adj.size();
    std::vector<bool> visited(n, false);
    std::vector<int> path;
    std::vector<std::vector<int>> cycles;

    for (int i = 0; i < n; ++i) {
        hamiltonianDFS(i, adj, path, visited, cycles);
    }

    return cycles;
}

std::vector<Edge *> Graph::tspSolution()
{
    if(findHamiltonianCycle().empty()){
        return {};
    }
    std::vector<std::vector<int>>adj = adjListByVector();
    auto cycles = allHamiltonianCycles(adj);
    std::vector<int>result;
    int minWeigth = INT_FAST16_MAX;


    std::ofstream file("cycles.txt");
    for (const auto &cycle : cycles) {
        int weight = cycleWeight(cycle);
        //qDebug() << "Hamiltonian cycle of weight " << weight << ": ";
        file << "Hamiltonian cycle of weight " << weight << ": ";
        QString tmp = "";

        for (int v : cycle) {
            tmp+= QString().setNum(v)+" ";
            file << v << " ";
        }
        if(weight < minWeigth){
            result=cycle;
            minWeigth=weight;
        }
        file << "\n";
        //qDebug()<<tmp;
    }

    file.close();
    std::vector<Edge*>cycle;
    int numVertices = nodes().size();
    for(int i = 0;i < result.size()-1;i++){
        cycle.push_back(getEdgeByNodes(result[i],result[i+1]));
    }
    cycle.push_back(getEdgeByNodes(result[numVertices-1],result[0]));

    return cycle;

}

int Graph::cycleWeight(const std::vector<int> &cycle)
{
    int weight = 0;
    auto graph=adjMatrixVector();
    for (int i = 0; i < cycle.size(); ++i) {
        weight += graph[cycle[i]][cycle[(i + 1) % cycle.size()]];
    }
    return weight;
}




const AdjencyMatrix &Graph::bandwidthMatrix() const
{
    return m_bandwidthMatrix;
}

bool Graph::isWeb() const
{
    return m_isWeb;
}

Node *Graph::getNodeByNumeric(int numeric)
{
    for (Node* u : m_nodes){
        if(u->numeric()==numeric){
            return u;
        }
    }
    return nullptr;
}

std::vector<Edge *> Graph::getNeighboursEdges(Node *u)
{
    std::vector<Edge*>result;
    QList<Node*>neighbours = adjList().getVertexByNumeric(u->numeric());

    for(Node* neighbour : neighbours){
        result.push_back(getEdgeByNodes(u,neighbour));
    }
    return result;
}

Edge *Graph::getEdgeByNodes(Node *u, Node *v)
{
    if(adjMatrix().getAtPos(u->numeric(),v->numeric())==0){
        return nullptr;
    }

    for(Edge* e : m_edges){
        if(e->startVertex()==u && e->finishVertex()==v){
            return e;
        }
    }

    return nullptr;
}

Edge *Graph::getEdgeByNodes(int numeric_v1, int numeric_v2)
{
    Node* v1 = getNodeByNumeric(numeric_v1);
    Node* v2 = getNodeByNumeric(numeric_v2);

    return getEdgeByNodes(v1,v2);

}

std::vector<std::vector<double> > Graph::toShortAdjMatrixDouble(AdjencyMatrix &matrix)
{
    std::vector<std::vector<double> > result(m_nodes.size(),std::vector<double>(m_nodes.size(),0));
    std::vector<std::vector<int> > pure_vector=matrix.getPureMatrixByVector();

    for(Edge* e : m_edges){
        result[e->startVertex()->numeric()][e->finishVertex()->numeric()]=pure_vector[e->startVertex()->numeric()][e->finishVertex()->numeric()];
        result[e->startVertex()->numeric()][e->startVertex()->numeric()]=pure_vector[e->startVertex()->numeric()][e->startVertex()->numeric()];
    }
    return result;
}

int Graph::lastIterations() const
{
    return m_lastIterations;
}

const AdjencyMatrix &Graph::kirchhoffMatrix() const
{
    return m_kirchhoffMatrix;
}

std::vector<std::vector<int> > Graph::adjListByVector()
{
    int numVertices = nodes().size();
    std::vector<std::vector<int> > graph(numVertices);
    for(int i = 0;i < numVertices;i++){
        //qDebug() << i;
        for(Node* v : adjList().getVertexByNumeric(i)){
            graph[i].push_back(v->numeric());
            //qDebug()<<v->numeric();
        }
        //qDebug()<<"-------";
    }
    return graph;
}

void Graph::rebuildByAdjMatrix(std::vector<std::vector<int> > adj_vector)
{
    clearNodesEdges();
    int n = adj_vector.size();
    for(int i = 0;i < n;i++){
        addNode();
    }

    for(int i = 0;i < n;i++){
        for(int j = 0;j < n;j++){
            if(adj_vector[i][j] != 0){
                addEdge(i,j,adj_vector[i][j]);
            }
            if(!this->m_oriented){
                adj_vector[j][i]=0;
            }
        }
    }
    rebuildAbstractStructres();

}

void Graph::clearNodesEdges()
{
    while(!m_nodes.empty()){
        delete m_nodes.back();
        m_nodes.pop_back();
    }

    while(!m_edges.empty()){
        delete m_edges.back();
        m_edges.pop_back();
    }
}

void Graph::rebuildAbstractStructres()
{
    m_adjList.refill(m_edges);
    m_adjMatrix.refill(m_edges);
    m_kirchhoffMatrix.refill(m_edges);
    m_kirchhoffMatrix.toKirchhoff();

}

double Graph::determinant(std::vector<std::vector<double> > &matrix, int n)
{
   int size = matrix.size();
   double det = 0;
   int degree = 1;
/*
   for(int i = 0;i<size;i++){
       QString t = "";
       for(int j = 0;j<size;j++){
           t+=QString().setNum(matrix[i][j])+" ";
       }
       qDebug()<<t;
   }
   */
  // qDebug()<<"\n";

   if(size == 1) {
       return matrix[0][0];
   }
   else if(size == 2) {
       //qDebug()<<"Result: " <<matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0];
       //qDebug()<<"_-------------------_------------_";
       return matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0];

   }
   else if(size == 3) {

       double det = matrix[0][0]*(matrix[1][1]*matrix[2][2] - matrix[1][2]*matrix[2][1])
           - matrix[0][1]*(matrix[1][0]*matrix[2][2] - matrix[1][2]*matrix[2][0])
           + matrix[0][2]*(matrix[1][0]*matrix[2][1] - matrix[1][1]*matrix[2][0]);
       //qDebug() << "Result size3: "<<det;
       return det;
   }
   else {
       for(int j = 0; j < size; j++) {
           std::vector<std::vector<double>> newMatrix = getMatrixWithoutRowAndCol(matrix, 0, j);
           //qDebug()<<"degree: " << degree << " matrix[0][" << j << "] :" << matrix[0][j];
           det += degree * matrix[0][j] * determinant(newMatrix,n);
           degree = -degree;
       }
   }

   return det;

}
//Default constructor
Graph::Graph(bool weighted) :
    m_weighted(weighted),
    m_adjMatrix(Settings::MAX_COUNT_OF_NODES),
    m_adjList(Settings::MAX_COUNT_OF_NODES),
    m_bandwidthMatrix(Settings::MAX_COUNT_OF_NODES),
    m_kirchhoffMatrix(Settings::MAX_COUNT_OF_NODES){m_oriented=false;m_isWeb=false;}

Graph::~Graph()
{
    //erase all Nodes
    for(Node *n : m_nodes){
        delete n;
    }
}



UnorientedGraph::UnorientedGraph(bool weighted):Graph(weighted){
m_oriented=false;
}

void UnorientedGraph::addEdge(Node *n1, Node *n2, int weight)
{
    Edge* e1 = new Edge(n2,n1,false,weight); //make edge
    Edge* e2 = new Edge(n1,n2,false,weight); //make edge
    m_edges.push_back(e1); //add our edge
    m_edges.push_back(e2);
    rebuildAbstractStructres(); //rebuild matrix and adj list to fit Graph
}

void UnorientedGraph::addEdge(int numeric_v1, int numeric_v2, int weight)
{
    Node* v1 = nullptr;
    Node* v2 = nullptr;

    for(Node* v : m_nodes){
        if(v->numeric()==numeric_v1){
            v1 = v;
        }
        if(v->numeric()==numeric_v2){
            v2 = v;
        }
    }

    if(v1 == nullptr || v2 == nullptr){
        return;
    }
    addEdge(v1,v2,weight);
}

void UnorientedGraph::removeEdge(Node *startV, Node *finishV)
{
    for(int i = 0;i < static_cast<int>(m_edges.size());i++){
        if(m_edges[i]->startVertex()==startV && m_edges[i]->finishVertex()==finishV){
            delete m_edges[i];
            m_edges.erase(m_edges.begin()+i);
        }
        if(m_edges[i]->finishVertex()==startV && m_edges[i]->startVertex()==finishV){
            delete m_edges[i];
            m_edges.erase(m_edges.begin()+i);
        }
    }
    rebuildAbstractStructres();

}


OrientedGraph::OrientedGraph(bool weighted):Graph(weighted)
{m_oriented=true;}

bool Graph::isConnected(int numeric_v1, int numeric_v2)
{
    Node* v1 = nullptr;
    Node* v2 = nullptr;

    for(Node* v : m_nodes){
        if(v->numeric()==numeric_v1){
            v1 = v;
        }
        if(v->numeric()==numeric_v2){
            v2 = v;
        }
    }

    if(v1 == nullptr || v2 == nullptr){
        return false;
    }
    return isConnected(v1,v2);

}

bool Graph::isConnected(Node *v1, Node *v2)
{
    for(Edge* e1 : m_edges){
        if(e1->startVertex() == v1 && e1->finishVertex() == v2){
            return true;
        }
    }
    return false;
}

void OrientedGraph::addEdge(Node *n1, Node *n2, int weight)
{
    Edge* e1 = new Edge(n1,n2,true,weight); //make edge
    m_edges.push_back(e1); //add our edge
    rebuildAbstractStructres(); //rebuild matrix and adj list to fit Graph
}

void OrientedGraph::addEdge(int numeric_v1, int numeric_v2, int weight)
{
    Node* v1 = nullptr;
    Node* v2 = nullptr;

    for(Node* v : m_nodes){
        if(v->numeric()==numeric_v1){
            v1 = v;
        }
        if(v->numeric()==numeric_v2){
            v2 = v;
        }
    }

    if(v1 == nullptr || v2 == nullptr){
        return;
    }
    addEdge(v1,v2,weight);

}

UnionFind::UnionFind(int n)
{
    rank = std::vector<int>(n, 0);
    parent = std::vector<int>(n);
    for (int i = 0; i < n; ++i) {
        parent[i] = i;
    }
}

int UnionFind::find(int x)
{
    if (x != parent[x]) {
        parent[x] = find(parent[x]);
    }
    return parent[x];
}

int UnionFind::merge(int x, int y)
{
    int rootX = find(x);
    int rootY = find(y);
    if (rootX != rootY) {
        if (rank[rootX] < rank[rootY]) {
            parent[rootX] = rootY;
        } else if (rank[rootX] > rank[rootY]) {
            parent[rootY] = rootX;
        } else {
            parent[rootY] = rootX;
            rank[rootX]++;
        }
    }
}

std::vector<std::vector<double> > getMatrixWithoutRowAndCol(std::vector<std::vector<double> > &mat, int p, int q)
{
    int n = mat.size();
    std::vector<std::vector<double>> result(n - 1, std::vector<double>(n - 1));

    int i = 0, j = 0;
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            if (row != p && col != q) {
                result[i][j++] = mat[row][col];
                if (j == n - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }

    return result;

}
