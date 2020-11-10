#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <stack>
#include <queue>
#include <time.h>


template<class T>
class Graph
{
public:
    virtual void AddEdge(const T& from, const T& to) = 0;

    virtual void RemoveEdge(const T& from, const T& to) = 0;

    bool VertexExists(const T& vertex)
    {
        return _adjacencyList.find(vertex) != _adjacencyList.end();
    }

    bool EdgeExists(const T& from, const T& to)
    {
        return _adjacencyList.find(from) != _adjacencyList.end() &&
        std::find(_adjacencyList[from].begin(), _adjacencyList[from].end(), to) != _adjacencyList[from].end();
    }

    size_t EdgeCount()
    {
        size_t edgeCount = 0;
        for(auto it : _adjacencyList)
        {
            edgeCount += it.second.size();
        }

        return edgeCount;
    }

    size_t VertexCount() const
    {
        return _adjacencyList.size();
    }

    virtual bool HasCycles() = 0;

    std::vector<std::vector<T>> FindConnectedComponents()
    {
        std::vector<std::vector<T>> foundComponents;
        std::unordered_map<T, VertexStatus> status;

        //Inicializar visitados
        for(auto key_value : _adjacencyList)
        {
            status[key_value.first] = VertexStatus::NotVisited;
        }

        for(auto key_value : _adjacencyList)
        {
            T vertex = key_value.first;

            if(status[vertex] == VertexStatus::NotVisited)
            {
                std::vector<T> currentComponent;

                FindConnectedComponents_DFS(vertex, status, currentComponent);

                foundComponents.push_back(currentComponent);
            }
        }

        return foundComponents;
    }

    size_t ConnectedComponentsCount()
    {
        return FindConnectedComponents().size();
    }

    bool IsConnected()
    {
        return ConnectedComponentsCount() < 2;
    }

    bool IsTree()
    {
        return !HasCycles() && IsConnected();
    }

    virtual bool IsBipartite()
    {
        return Two_Colorize_BFS().first;
    }

    std::pair<bool, std::unordered_map<T, size_t>> Two_Colorize_BFS()
    {
        std::unordered_map<T, size_t> colorMap;

        for(auto key_value : _adjacencyList)
        {
            colorMap[key_value.first] = Uncolored;
        }

        for(auto key_value : _adjacencyList)
        {
            T currentVertex = key_value.first;

            if(colorMap[currentVertex] == Uncolored)
            {
                colorMap[currentVertex] = rand() % 2 == 1 ? PS5_White : Xbox_Black;

                std::queue<T> vertexToVisit;
                vertexToVisit.push(currentVertex);

                while (!vertexToVisit.empty())
                {
                    T nextInQueue = vertexToVisit.front();
                    vertexToVisit.pop();

                    for (auto neighbor : _adjacencyList[nextInQueue])
                    {
                        if (colorMap[neighbor] == Uncolored)
                        {
                            colorMap[neighbor] = colorMap[nextInQueue] == Xbox_Black ? PS5_White : Xbox_Black;
                            vertexToVisit.push(neighbor);
                        }
                        else if (colorMap[neighbor] == colorMap[nextInQueue])
                        {
                            return std::make_pair(false, colorMap);
                        }
                    }
                }
            }
        }

        return std::make_pair(true, colorMap);
    }

protected:
    std::unordered_map<T, std::vector<T>> _adjacencyList;

    enum class VertexStatus { NotVisited, InStack, Visited };
    enum NodeColor {Xbox_Black = 0, PS5_White, Uncolored};

    void FindConnectedComponents_DFS(T currentVertex, std::unordered_map<T, VertexStatus>& status, std::vector<T>& currentComponent)
    {
        status[currentVertex] = VertexStatus::Visited;
        currentComponent.push_back(currentVertex);

        for(auto neighbor : _adjacencyList[currentVertex])
        {
            if(status[neighbor] == VertexStatus::NotVisited)
            {
                FindConnectedComponents_DFS(neighbor, status, currentComponent);
            }
        }
    }
};

template<class T>
class DirectedGraph : public Graph<T>
{
public:
    DirectedGraph(){}

    virtual void AddEdge(const T& from, const T& to)
    {
        if(!EdgeExists(from, to))
        {
            _adjacencyList[from].push_back(to);

            if(!VertexExists(to))
            {
                _adjacencyList[to];
            }
        }
    }

    virtual void RemoveEdge(const T& from, const T& to)
    {
        if(EdgeExists(from, to))
        {
            auto toIndex = std::find(_adjacencyList[from].begin(), _adjacencyList[from].end(), to);
            _adjacencyList[from].erase(toIndex);
        }
    }

    std::vector<std::vector<T>> FindCycles()
    {
        std::vector<std::vector<T>> cyclesFound;
        std::unordered_map<T, VertexStatus> status;

        for(auto key_value : _adjacencyList)
        {
            status[key_value.first] = VertexStatus::NotVisited;
        }

        for(auto key_value : _adjacencyList)
        {
            T vertex = key_value.first;

            if(status[vertex] == VertexStatus::NotVisited)
            {
                std::stack<T> currentPath;

                currentPath.push(vertex);
                status[vertex] = VertexStatus::InStack;

                FindCycles_DFS(status, currentPath, cyclesFound);
            }
        }

        return cyclesFound;
    }

    size_t CyclesCount()
    {
        auto cyclesFound = FindCycles();

        return cyclesFound.size();
    }

    virtual bool HasCycles()
    {
        return CyclesCount() > 0;
    }
protected:
    void FindCycles_DFS(std::unordered_map<T, VertexStatus>& status,
                        std::stack<T>& currentPath,
                        std::vector<std::vector<T>>& cyclesFound)
    {
        for(auto neighbor : _adjacencyList[currentPath.top()])
        {
            if(status[neighbor] == VertexStatus::InStack)
            {
                std::vector<T> cycleFound;
                auto pathCopy = currentPath;

                cycleFound.push_back(neighbor);
                while(!pathCopy.empty())
                {
                    cycleFound.push_back(pathCopy.top());
                    if(pathCopy.top() == neighbor)
                        break;
                    pathCopy.pop();
                }

                cyclesFound.push_back(cycleFound);
            }
            else if(status[neighbor] == VertexStatus::NotVisited)
            {
                currentPath.push(neighbor);
                status[neighbor] = VertexStatus::InStack;

                FindCycles_DFS(status, currentPath, cyclesFound);
            }
        }

        status[currentPath.top()] = VertexStatus::Visited;
        currentPath.pop();
    }
};

template<class T>
class UndirectedGraph : public Graph<T>
{
public:
    virtual void AddEdge(const T& from, const T& to)
    {
        if(!EdgeExists(from, to))
        {
            _adjacencyList[from].push_back(to);
            _adjacencyList[to].push_back(from);
        }
    }

    virtual void RemoveEdge(const T& from, const T& to)
    {
        if(EdgeExists(from, to))
        {
            auto toIndex = std::find(_adjacencyList[from].begin(), _adjacencyList[from].end(), to);
            _adjacencyList[from].erase(toIndex);

            auto fromIndex = std::find(_adjacencyList[to].begin(), _adjacencyList[to].end(), from);
            _adjacencyList[to].erase(fromIndex);
        }
    }

    virtual bool HasCycles()
    {
        std::unordered_map<T, VertexStatus> status;

        for(auto key_value : _adjacencyList)
        {
            status[key_value.first] = Graph<T>::VertexStatus::NotVisited;
        }

        for(auto key_value : _adjacencyList)
        {
            T vertex = key_value.first;

            if(status[vertex] == Graph<T>::VertexStatus::NotVisited)
            {
                if (HasCycles_DFS(vertex, nullptr, status))
                {
                    return true;
                }
            }
        }

        return false;
    }

    size_t EvenDegreeVertexCount()
    {
        size_t evenDegreeCount = 0;

        for(auto key_pair : _adjacencyList)
        {
            if(key_pair.second.size() % 2 == 0)
            {
                evenDegreeCount++;
            }
        }

        return evenDegreeCount;
    }

    size_t OddDegreeVertexCount()
    {
        size_t oddDegreeCount = 0;

        for(auto key_pair : _adjacencyList)
        {
            if(key_pair.second.size() % 2 != 0)
            {
                oddDegreeCount++;
            }
        }

        return oddDegreeCount;
    }

    bool HasEulerianCycle()
    {
        return EvenDegreeVertexCount() == this->VertexCount();
    }

    bool HasEulerianPath()
    {
        return HasEulerianCycle() || OddDegreeVertexCount() == 2;
    }

    std::vector<T> FindArticulationPoints()
    {
        std::vector<T> articulationPointsFound;

        std::unordered_map<T, Graph<T>::VertexStatus> status;
        std::unordered_map<T, size_t> discoveryTime;
        std::unordered_map<T, size_t> lowestDiscoveryTime;
        size_t timer = 0;

        for(auto key_value : _adjacencyList)
        {
            status[key_value.first] = Graph<T>::VertexStatus::NotVisited;
            discoveryTime[key_value.first] = -1;
            lowestDiscoveryTime[key_value.first] = -1;
        }

        for(auto key_value : _adjacencyList)
        {
            T vertex = key_value.first;

            if(status[vertex] == Graph<T>::VertexStatus::NotVisited)
            {
                FindArticulationPoints_DFS(vertex,
                                           timer,
                                           vertex,
                                           status,
                                           discoveryTime,
                                           lowestDiscoveryTime,
                                           articulationPointsFound);
            }
        }

        return articulationPointsFound;
    }

protected:
    bool HasCycles_DFS(T& vertex, T* parent, std::unordered_map<T, VertexStatus>& status)
    {
        status[vertex] = Graph<T>::VertexStatus::Visited;

        for(auto neighbor : _adjacencyList[vertex])
        {
            if(status[neighbor] == Graph<T>::VertexStatus::NotVisited)
            {
                if(HasCycles_DFS(neighbor, &vertex, status))
                {
                    return true;
                }
            }
            else if(parent != nullptr && *parent != neighbor)
            {
                return true;
            }
        }

        return false;
    }

    void FindArticulationPoints_DFS(T& vertex,
                                    size_t& timer,
                                    T& root,
                                    std::unordered_map<T, Graph<T>::VertexStatus>& status,
                                    std::unordered_map<T, size_t>& discoveryTime,
                                    std::unordered_map<T, size_t>& lowestDiscoveryTime,
                                    std::vector<T>& articulationsPointsFound)
    {
        status[vertex] = Graph<T>::VertexStatus::Visited;
        timer++;
        discoveryTime[vertex] = timer;
        lowestDiscoveryTime[vertex] = timer;
        size_t children = 0;

        for(auto neighbor : _adjacencyList[vertex])
        {
            if(status[neighbor] == Graph<T>::VertexStatus::NotVisited)
            {
                FindArticulationPoints_DFS(neighbor,
                                           timer,
                                           root,
                                           status,
                                           discoveryTime,
                                           lowestDiscoveryTime,
                                           articulationsPointsFound);

                lowestDiscoveryTime[vertex] = std::min(lowestDiscoveryTime[vertex], lowestDiscoveryTime[neighbor]);

                if (lowestDiscoveryTime[neighbor] >= discoveryTime[vertex] && vertex != root)
                {
                    articulationsPointsFound.push_back(vertex);
                }

                children++;
            }
            else
            {
                lowestDiscoveryTime[vertex] = std::min(lowestDiscoveryTime[vertex], discoveryTime[neighbor]);
            }

            if(vertex == root && children > 1)
            {
                articulationsPointsFound.push_back(vertex);
            }
        }
    }
};

template<class T>
class Tree : public UndirectedGraph<T>
{
public:
    Tree(){}

    virtual void AddEdge(const T& from, const T& to)
    {
        if(!this->EdgeExists(from, to))
        {
            UndirectedGraph<T>::AddEdge(from, to);

            if(!this->IsTree())
            {
                this->RemoveEdge(from, to);

                //throw "El árbol se marchitó. Me marcho yo. Yo sé perder.
            }
        }
    }

    virtual bool IsBipartite()
    {
        return true;
    }
};

int main()
{
    srand(time(NULL));

/*
    //DEMO CICLOS

    std::cout << "sinCiclos:" << std::endl;

    UndirectedGraph<size_t> sinCiclos;

    sinCiclos.AddEdge(1, 0);
    sinCiclos.AddEdge(1, 2);
    sinCiclos.AddEdge(0, 3);
    sinCiclos.AddEdge(3, 4);

    std::cout << (sinCiclos.HasCycles() ? "Ciclico" : "Aciclico") << std::endl << std::endl;

    //-----------------------------------------------

    std::cout << "conCiclos:" << std::endl;

    UndirectedGraph<size_t> conCiclos;

    conCiclos.AddEdge(1, 0);
    conCiclos.AddEdge(1, 2);
    conCiclos.AddEdge(2, 0);
    conCiclos.AddEdge(0, 3);
    conCiclos.AddEdge(3, 4);   

    std::cout << (conCiclos.HasCycles() ? "Ciclico" : "Aciclico") << std::endl << std::endl;

    //-----------------------------------------------

    std::cout << "conCiclos [Dirigido]:" << std::endl;

    DirectedGraph<size_t> dirigidoConCiclos;

    dirigidoConCiclos.AddEdge(2, 1);
    dirigidoConCiclos.AddEdge(2, 3);
    dirigidoConCiclos.AddEdge(3, 4);
    dirigidoConCiclos.AddEdge(4, 5);
    dirigidoConCiclos.AddEdge(5, 6);
    dirigidoConCiclos.AddEdge(6, 3);
    dirigidoConCiclos.AddEdge(1, 7);
    dirigidoConCiclos.AddEdge(7, 8);
    dirigidoConCiclos.AddEdge(8, 1);

    std::cout << (dirigidoConCiclos.HasCycles() ? "Ciclico" : "Aciclico") << std::endl;
    std::cout << "Cantidad de Ciclos: " << dirigidoConCiclos.CyclesCount() << std::endl;
    for(auto cycle : dirigidoConCiclos.FindCycles())
    {
        std::cout << "Ciclo: ";
        for(auto vertex : cycle)
        {
            std::cout << vertex << " ";
        }
        std::cout << std::endl;
    }
*/

/*
    //DEMO COMPONENTES CONECTADOS
    std::cout << "connected:" << std::endl;

    UndirectedGraph<std::string> connected;

    connected.AddEdge("A", "B");
    connected.AddEdge("B", "C");
    connected.AddEdge("C", "D");
    connected.AddEdge("D", "E");
    connected.AddEdge("D", "F");
    connected.AddEdge("D", "G");

    std::cout << (connected.IsConnected() ? "Conectado" : "No Conectado") << std::endl;
    std::cout << "Cantidad de componentes conectados: " << connected.ConnectedComponentsCount() << std::endl;

    //-----------------------------------------------

    std::cout << "disconnected:" << std::endl;

    UndirectedGraph<std::string> disconnected;

    disconnected.AddEdge("A", "B");
    disconnected.AddEdge("C", "D");
    disconnected.AddEdge("E", "F");
    disconnected.AddEdge("E", "G");

    std::cout << (disconnected.IsConnected() ? "Conectado" : "No Conectado") << std::endl;
    std::cout << "Cantidad de componentes conectados: " << disconnected.ConnectedComponentsCount() << std::endl;
    for(auto connectedComponent : disconnected.FindConnectedComponents())
    {
        std::cout << "Componente conectado: ";

        for(auto vertex : connectedComponent)
        {
            std::cout << vertex << " ";
        }

        std::cout << std::endl;
    }
*/

/*
    //DEMO: ARBOLES

    std::cout << "elTri: " << std::endl;

    Tree<long long> elTri;

    elTri.AddEdge(1, 2);
    elTri.AddEdge(2, 3);
    elTri.AddEdge(2, 4);
    elTri.AddEdge(1, 5);
    elTri.AddEdge(5, 6);
    elTri.AddEdge(5, 7);

    std::cout << (elTri.HasCycles() ? "Ciclico" : "Aciclico") << std::endl;
    std::cout << (elTri.IsConnected() ? "Conectado" : "No Conectado") << std::endl;
    std::cout << (elTri.IsTree() ? "Arbol" : "No Arbol") << std::endl;
*/

/*
    //DEMO: GRAFOS BIPARTITAS - 2-COLOREO
    std::cout << "bipartite:" << std::endl;

    UndirectedGraph<char> bipartite;

    bipartite.AddEdge('a', 'b');
    bipartite.AddEdge('b', 'c');
    bipartite.AddEdge('b', 'd');
    bipartite.AddEdge('c', 'e');
    bipartite.AddEdge('c', 'f');
    bipartite.AddEdge('d', 'g');
    bipartite.AddEdge('d', 'h');

    std::cout << (bipartite.IsBipartite() ? "Bipartita" : "No Bipartita") << std::endl;
    auto colorizeResult = bipartite.Two_Colorize_BFS();
    for(auto key_value : colorizeResult.second)
    {
        std::cout << key_value.first << " " << (key_value.second == 0 ? "Negro" : "Blanco") << std::endl;
    }

    //-----------------------------------------------

    std::cout << "noBipartite:" << std::endl;

    UndirectedGraph<char> noBipartite;

    noBipartite.AddEdge('a', 'b');
    noBipartite.AddEdge('b', 'c');
    noBipartite.AddEdge('c', 'd');
    noBipartite.AddEdge('d', 'e');
    noBipartite.AddEdge('e', 'a');

    std::cout << (noBipartite.IsBipartite() ? "Bipartita" : "No Bipartita") << std::endl;
*/

/*
    //DEMO: PUNTOS DE ARTICULACION (VERTICES DE CORTE)
    UndirectedGraph<std::string> cutVertexGraph;

    cutVertexGraph.AddEdge("A", "B");
    cutVertexGraph.AddEdge("A", "C");
    cutVertexGraph.AddEdge("B", "D");
    cutVertexGraph.AddEdge("C", "D");
    cutVertexGraph.AddEdge("C", "E");
    cutVertexGraph.AddEdge("D", "E");
    cutVertexGraph.AddEdge("E", "F");
    cutVertexGraph.AddEdge("E", "G");
    cutVertexGraph.AddEdge("F", "G");

    std::cout << "Puntos de articulacion: " << std::endl;
    for(auto aP : cutVertexGraph.FindArticulationPoints())
    {
        std::cout << aP << std::endl;
    }
*/

/*
    //DEMO: EXISTENCIA DE CAMINOS/CICLOS EULERIANOS

    std::cout << "Lugares con camino y ciclo euleriano: " << std::endl;

    UndirectedGraph<std::string> locations;

    std::string zero_bbb("Bob-omb Battlefield");
    std::string one_wf("Whomps Fortress");
    std::string two_jrb("Jolly Roger Bay");
    std::string three_ccm("Cool, Cool Mountain");
    std::string four_bbh("Big Boo's Haunt");

    locations.AddEdge(zero_bbb, one_wf);
    locations.AddEdge(zero_bbb, two_jrb);
    locations.AddEdge(zero_bbb, three_ccm);
    locations.AddEdge(zero_bbb, four_bbh);
    locations.AddEdge(one_wf, two_jrb);
    locations.AddEdge(three_ccm, four_bbh);

    std::cout << "Vertices con grado par: " << locations.EvenDegreeVertexCount() << "/" << locations.VertexCount() << std::endl;
    std::cout << "Vertices con grado impar: " << locations.OddDegreeVertexCount() << "/" << locations.VertexCount() << std::endl;
    std::cout << (locations.HasEulerianPath() ? "Camino Euleriano" : "Sin Camino Euleriano") << std::endl;
    std::cout << (locations.HasEulerianCycle() ? "Ciclo Euleriano" : "Sin Ciclo Euleriano") << std::endl << std::endl;

    //-----------------------------------------------

    std::cout << "Lugares sin camino ni ciclo euleriano: " << std::endl;

    UndirectedGraph<std::string> locationsNoEuler;

    locationsNoEuler.AddEdge(zero_bbb, one_wf);
    locationsNoEuler.AddEdge(zero_bbb, two_jrb);
    locationsNoEuler.AddEdge(zero_bbb, three_ccm);
    locationsNoEuler.AddEdge(one_wf, three_ccm);
    locationsNoEuler.AddEdge(one_wf, two_jrb);
    locationsNoEuler.AddEdge(three_ccm, four_bbh);

    std::cout << "Vertices con grado par: " << locationsNoEuler.EvenDegreeVertexCount() << "/" << locationsNoEuler.VertexCount() << std::endl;
    std::cout << "Vertices con grado impar: " << locationsNoEuler.OddDegreeVertexCount() << "/" << locationsNoEuler.VertexCount() << std::endl;
    std::cout << (locationsNoEuler.HasEulerianPath() ? "Camino Euleriano" : "Sin Camino Euleriano") << std::endl;
    std::cout << (locationsNoEuler.HasEulerianCycle() ? "Ciclo Euleriano" : "Sin Ciclo Euleriano") << std::endl;   
*/


    return 0;
}
