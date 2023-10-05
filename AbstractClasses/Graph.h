#pragma once

#include <vector>
#include <list>
#include "Node.h"
#include "edge.h"
#include "AdjencyMatrix.h"
#include "AdjencyList.h"

#include <cmath>
#include <climits>
#include <set>
#include <algorithm>
#include <unordered_map>
#include <random>
#include <fstream>

class UnionFind{
public:
    UnionFind(int n);
    int find(int x);
    int merge(int x,int y);

private:
    std::vector<int>parent;
    std::vector<int>rank;
};

struct EdgeCompare{
    bool operator()(const Edge* a,const Edge* b){
        return a->weight() > b->weight();
    }
};

std::vector<std::vector<double>> getMatrixWithoutRowAndCol(std::vector<std::vector<double>>& mat, int p, int q);

double bernoulli_distribution();

class Graph{

protected:
    const bool m_weighted;
    bool m_oriented;
    bool m_isWeb;
    int m_lastIterations=0;

    std::vector<Node*> m_nodes; // set of vertexes
    std::vector<Edge*> m_edges; // set of edges
    AdjencyList m_adjList; // adjency List
    AdjencyMatrix m_adjMatrix; // adjency Matrix
    AdjencyMatrix m_bandwidthMatrix;//bandwith matrix for web graph
    AdjencyMatrix m_kirchhoffMatrix;//kirchhoff matrix
    void rebuildAbstractStructres(); //method to rebuild matrix and list of adjency
    double determinant(std::vector<std::vector<double>>& matrix,int n);

    bool acyclic = false, linked = false;



public:
    //CONSTRUCTORS
    Graph(bool weighted); //constructor
    virtual ~Graph();


    //GETTERS

    bool isConnected(int numeric_v1, int numeric_v2);
    bool isConnected(Node* v1,Node* v2);
    std::vector<int> getNumericsOfNodes();
    AdjencyList adjList();
    const AdjencyMatrix &adjMatrix() const;
    bool weighted() const;
    const std::vector<Node *> &nodes() const;
    int getDegreeNode(Node* v1);
    int getDegreeNode(int numeric_v1);
    const std::vector<Edge *> &edges() const;
    std::vector<std::vector<int>> adjMatrixVector();
    const AdjencyMatrix &bandwidthMatrix() const;
    bool isWeb() const;
    Node* getNodeByNumeric(int numeric);
    std::vector<Edge*> getNeighboursEdges(Node* u);
    Edge* getEdgeByNodes(Node* u, Node* v);
    Edge* getEdgeByNodes(int numeric_v1,int numeric_v2);
    std::vector<std::vector<double>> toShortAdjMatrixDouble(AdjencyMatrix& matrix);
    int lastIterations() const;
    const AdjencyMatrix &kirchhoffMatrix() const;
    std::vector<std::vector<int>> adjListByVector();


    //SETTERS

    //METHODS
    void rebuildByAdjMatrix(std::vector<std::vector<int>>adj_vector);
    void clearNodesEdges();
    void addNode(); // add new Node
    virtual void addEdge(Node* n1,Node*n2, int weight = 1); //add new Edge
    virtual void addEdge(int numeric_v1,int numeric_v2,int weight = 1);

    void removeNode(int numeric);
    void removeEdge(Node* startV,Node* finishV);
    void removeEdgesByNode(Node* v1);

    virtual void generateGraph(int vertexCount, bool random = false,bool acyclic = false, bool linked = false,bool webmode=false);

    //ALGORITHMS
    //check for linked
    bool isLinked(std::vector<std::vector<int>>graph={});
    void dfs(int vertex, std::vector<std::vector<int>>& graph, std::vector<bool>& visited);

    //find all paths between two vertexes, using DFS algos
    void findAllPaths(int u,int v,std::vector<int>& path,std::vector<std::vector<int>>& allPaths);

    //floyd warshall modification with return the shortest path
    std::vector<int> floydWarshall(int u, int v, std::vector<std::vector<int>>& matrixOfPaths,std::vector<std::vector<int>>& matrixOfWeight);
    std::vector<int> floydWarshallGetPath(int u, int v);

    //classis dijkstra algorithm with find path between two vertex
    std::vector<int> dijkstraAlgorithm(int u, int v, std::vector<std::pair<int,bool>>& vectorShortestPath);

    //bellman ford algo with find path between two vertexes
    std::vector<int> bellmanFordAlgorithm(int u,int v,std::vector<std::pair<int,bool>>& vectorShortestPath);

    //Algo for Web (Ford-Fulkerson)
    int fordFalkersonAlgorithm(int source,int sink);
    bool fordFalkersonBFS(std::vector<std::vector<int>>& residualGraph, int source, int sink, std::vector<int>& parent);
    int minCostMaxFlowAlgorithm(std::vector<std::vector<int>> bandwidth,int source,int sink);
    int twoThirdsMaxFlowAlgorithm(int source, int sink);
    std::vector<int> dijkstraMinCost(std::vector<std::vector<int>>& residualGraph,int source);

    //Min spanning tree algos
    std::vector<Edge*> primaMinSpanTree(int startVertex);
    std::vector<Edge*> kruskalMinSpanTree(int startVertex);
    int countOfSpanTrees();
    //Code spanTree with Prufer
    std::vector<int> pruferCode(std::vector<Edge*> spanTree);
    std::pair<int,int>pruferFindLast(std::vector<int>code);
    std::vector<std::vector<int>> pruferDecode(std::vector<int> code);

    //HamiltonianPath
    std::vector<Edge*> findHamiltonianCycle();
    bool hamiltonianCycleUtil(std::vector<int>& path, int pos);
    bool isSafe(int v, std::vector<int>&path,int pos);
    void buildToHamiltonian();
    //EulerCycle
    std::vector<Edge*> findEulerianCycle();
    void eulerUtilDFS(int v, std::vector<std::vector<int>> &graph, std::vector<int> &result);
    void buildToEuler();
    bool isEulerCriterium(std::vector<std::vector<int>>graph={});
    //TSP
    void hamiltonianDFS(int v, std::vector<std::vector<int>> &adj, std::vector<int> &path, std::vector<bool> &visited, std::vector<std::vector<int>> &cycles);
    std::vector<std::vector<int>> allHamiltonianCycles(std::vector<std::vector<int>> &adj);
    std::vector<Edge*> tspSolution();
    int cycleWeight(const std::vector<int> &cycle);


};


class UnorientedGraph : public Graph
{

public:
    UnorientedGraph(bool weighted = false);

    //METHODS
    void addEdge(Node* n1,Node*n2, int weight = 1) override; //add new Edge
    void addEdge(int numeric_v1,int numeric_v2,int weight = 1) override;
    //void generateRandomGraph(int vertexCount) override;
    void removeEdge(Node* startV,Node* finishV);
};

class OrientedGraph : public Graph
{

public:
    OrientedGraph(bool weighted = false);

    //METHODS
    void addEdge(Node*n1,Node*n2,int weight = 1) override; //add new Edge
    void addEdge(int numeric_v1,int numeric_v2,int weight = 1) override;
    //void generateRandomGraph(int vertexCount);
};
