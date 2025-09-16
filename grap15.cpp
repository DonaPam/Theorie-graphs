#include <iostream>
#include <random>
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <set>
#include <iomanip>
#include <ctime>
#include <stack>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <utility>

using namespace std;
int INF = 1e9;
// Fonction qui génère un nombre aléatoire de Bernoulli avec probabilité p
int bernoulli_custom(double p) {
    // Générer un nombre aléatoire entre 0 et 1
    return (static_cast<double>(rand()) / RAND_MAX) < p ? 1 : 0;
}

vector<vector<int>> generateConnectedDAG(int n, double p, vector<int>& degrees) {
    vector<vector<int>> M(n, vector<int>(n, 0));
    random_device rd;
    mt19937 gen(rd());
    bernoulli_distribution dist(p);

    degrees.assign(n, 0);
    // Étape 1: Générer un ordre topologique aléatoire
    vector<int> order(n);
    for (int i = 0; i < n; ++i) order[i] = i;
    shuffle(order.begin(), order.end(), gen);

    // Étape 2: Construire un arbre couvrant orienté (DAG connecté minimal)
    for (int j = 1; j < n; ++j) {
        int parent = uniform_int_distribution<int>(0, j - 1)(gen);
        M[order[parent]][order[j]] = 1;  // Arc parent → enfant
        degrees[order[parent]]++;
    }

    // Étape 3: Ajouter des arcs aléatoires i → j (i < j dans l'ordre)
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (dist(gen))
            {// Vérifier que l'arc n'existe pas déjà
                if (M[order[i]][order[j]] == 0) {
                    M[order[i]][order[j]] = 1;
                    degrees[order[i]]++;
                }
            }
        }
    }

    return M;
}



void print_adjency_Matrix(const vector<vector<int>>& M) {
    for (const auto& row : M) {
        for (int val : row) {
            cout << val << " ";
        }
        cout << endl;
    }
}

// Zhimbel method implementation

vector<vector<int>> Zhimbel(vector<vector<int>> weight_matrix, vector<vector<int>> matrix, const int num_vertex, const int parametr) {
    vector<vector<int>> new_matrix(num_vertex, vector<int>(num_vertex, 0));
    for (int i = 0; i < num_vertex; i++) {
        for (int j = 0; j < num_vertex; j++) {
            vector<int> vect;
            for (int k = 0; k < num_vertex; k++) {
                if (matrix[i][k] == 0 || weight_matrix[k][j] == 0) {
                    vect.push_back(0);
                }
                else {
                    vect.push_back(matrix[i][k] + weight_matrix[k][j]);
                }
            }
            sort(vect.begin(), vect.end());
            if (vect.back() == vect.front() && vect.back() == 0) {
                new_matrix[i][j] = 0;
            }
            else {
                if (parametr == 1) {
                    new_matrix[i][j] = vect.back(); // max
                }
                else if (parametr == 2) {
                    vect.erase(remove(vect.begin(), vect.end(), 0), vect.end());
                    if (!vect.empty()) {
                        new_matrix[i][j] = vect.front(); // min
                    }
                }
            }
        }
    }
    return new_matrix;
}

// Generate reachability matrix
vector<vector<int>> Generate_reachability_matrix(const vector<vector<int>>& adjacency_matrix, const vector<vector<int>>& matrix, const int num_vertex) {
    vector<vector<int>> new_matrix(num_vertex, vector<int>(num_vertex, 0));
    for (int i = 0; i < num_vertex; i++) {
        for (int j = 0; j < num_vertex; j++) {
            for (int k = 0; k < num_vertex; k++) {
                new_matrix[i][j] += (matrix[i][k] * adjacency_matrix[k][j]);
            }
        }
    }
    return new_matrix;
}

// Sum of matrices
vector<vector<int>> Addition_matrix(const vector<vector<int>>& matrix1, const vector<vector<int>>& matrix2, const int num_vertex) {
    vector<vector<int>> new_matrix(num_vertex, vector<int>(num_vertex, 0));
    for (int i = 0; i < num_vertex; i++) {
        for (int j = 0; j < num_vertex; j++) {
            new_matrix[i][j] = (matrix1[i][j] + matrix2[i][j]);
        }
    }
    return new_matrix;
}

// Print matrix
void PrintMatrix(const vector<vector<int>>& matrix) {
    for (const auto& row : matrix) {
        for (int val : row) {
            cout << setw(5) << val;
        }
        cout << "\n";
    }
    cout << "\n";
}

// Get path based on 'from' vector
vector<int> GetPath(const vector<int>& from, int finish) {
    vector<int> path;
    for (int v = finish; v != -1; v = from[v]) {
        path.push_back(v);
    }
    reverse(path.begin(), path.end());
    return path;
}

// Input wrapper
int intinput() {
    int val;
    while (true) {
        std::cin >> val;
        if (std::cin.fail() || std::cin.peek() != '\n') {
            std::cin.clear();
            std::cin.ignore(std::cin.rdbuf()->in_avail());
            printf("\nThe entry is incorrect.Please try again.\n");
        }
        else break;
    }
    return val;
}

// Get start and finish vertices
void GetStartFinishVertex(int& start, int& finish, int num_vertex) {
    cout << "Enter the number of the vertex from which you want to build a route from 0 to " << num_vertex - 1 << ":" << endl;
    start = intinput();
    while (start < 0 || start > num_vertex - 1) {
        cout << "Incorrect input. Enter the number of the vertex from which you want to build a route from 0 to " << num_vertex - 1 << ":" << endl;
        start = intinput();
    }

    cout << "Enter the number of the vertex to which you want to build a route from 0 to " << num_vertex - 1 << ":" << endl;
    finish = intinput();
    while (finish < 0 || finish > num_vertex - 1) {
        cout << "Invalid input. Enter the number of the vertex you want to build a route to from 0 to " << num_vertex - 1 << ":" << endl;
        finish = intinput();
    }
}
void DepthFirstSearch(const vector<vector<int>>& adjacency_matrix, int start, int& iter_DFS) {
    int num_vertex = adjacency_matrix.size();
    vector<bool> visited(num_vertex, false);
    stack<int> s;
    s.push(start);

    // Génération de la matrice de bande passante
    vector<vector<int>> bandwidth_matrix(num_vertex, vector<int>(num_vertex, 0));
    srand(time(0));
    for (int i = 0; i < num_vertex; i++) {
        for (int j = 0; j < num_vertex; j++) {
            if (adjacency_matrix[i][j] != 0) {
                bandwidth_matrix[i][j] = rand() % (10 - 1 + 1) + 1;
            }
        }
    }

    vector<int> dfs_order;

    while (!s.empty()) {
        int v = s.top();
        s.pop();

        if (!visited[v]) {
            dfs_order.push_back(v);
            visited[v] = true;

            // Empiler les voisins non visités (de droite à gauche)
            for (int i = num_vertex - 1; i >= 0; --i) {
                iter_DFS++;
                if (adjacency_matrix[v][i] && !visited[i]) {
                    s.push(i);
                }
            }
        }
    }

    // Affichage formaté sans flèche finale
    cout << "DFS Order: ";
    for (size_t i = 0; i < dfs_order.size(); ++i) {
        cout << dfs_order[i];
        if (i != dfs_order.size() - 1) {
            cout << " -> ";
        }
    }
    cout << endl;
}

// Fonction pour trouver le plus court chemin dans un DAG depuis une source donnée
vector<int> DAGSSS(const vector<vector<int>>& adjMatrix, const vector<vector<int>>& weightMatrix, int source, int num_vertices, int& iteration) {
    vector<int> dist(num_vertices, INF);
    vector<int> top_order;

    // Mettre la distance du sommet source à 0
    dist[source] = 0;

    // Étape 1: Calculer un ordre topologique
    vector<bool> visited(num_vertices, false);
    stack<int> stack;

    function<void(int)> topological_sort = [&](int v) {
        visited[v] = true;
        iteration++;
        for (int i = 0; i < num_vertices; ++i) {
            if (adjMatrix[v][i] && !visited[i]) {
                topological_sort(i);
            }
        }
        stack.push(v);
        };

    for (int i = 0; i < num_vertices; ++i) {
        if (!visited[i]) {
            topological_sort(i);
        }
    }

    // Étape 2: Mettre à jour les distances basées sur l'ordre topologique
    while (!stack.empty()) {
        int u = stack.top();
        stack.pop();

        for (int v = 0; v < num_vertices; ++v) {
            iteration++;
            if (adjMatrix[u][v] > 0 && dist[u] != INF && dist[u] + weightMatrix[u][v] < dist[v]) {
                dist[v] = dist[u] + weightMatrix[u][v];
            }
        }
    }

    return dist;
}
vector<vector<int>> FloydWarshall(vector<vector<int>>& weight_matrix, int num_vertex, vector<vector<int>>& path, int& iteration) {
    vector<vector<int>> dist(num_vertex, vector<int>(num_vertex));

    for (int i = 0; i < num_vertex; i++) {
        for (int j = 0; j < num_vertex; j++) {
            if (i == j) {
                dist[i][j] = 0;
            }
            else if (weight_matrix[i][j] != 0) {
                dist[i][j] = weight_matrix[i][j];
                path[i][j] = i;
            }
            else {
                dist[i][j] = INF;
                path[i][j] = -1;
            }
        }
    }
    for (int k = 0; k < num_vertex; k++) {
        for (int i = 0; i < num_vertex; i++) {
            for (int j = 0; j < num_vertex; j++) {
                iteration++;
                if (dist[i][k] != INF && dist[k][j] != INF && dist[i][j] > dist[i][k] + dist[k][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    path[i][j] = path[k][j];
                }
            }
        }
    }

    for (int i = 0; i < num_vertex; i++) {
        if (dist[i][i] < 0) {
            cout << "The graph contains a cycle of negative length." << endl;
            return {};
        }
    }

    return dist;
}
vector<int> Getdist(int from, int to, const vector<vector<int>>& path) {
    vector<int> result;
    if (path[from][to] == -1) return result;

    while (to != from) {
        result.push_back(to);
        to = path[from][to];
    }
    result.push_back(from);
    reverse(result.begin(), result.end());
    return result;
}
void PrintWayInfo(int num_vertex, const vector<vector<int>>& dist, int from, int where_, const vector<vector<int>>& path) {
    cout << "Matrice of distance:\n";
    for (int i = 0; i < num_vertex; i++) {
        for (int j = 0; j < num_vertex; j++) {
            if (dist[i][j] != INF && dist[i][j] != -INF) {
                cout << dist[i][j] << "\t";
            }
            else {
                cout << "0" << "\t";
            }
        }
        cout << endl;
    }

    if (dist[from][where_] != INF && dist[from][where_] != -INF) {
        cout << "\nshort distance from " << from << " to " << where_ << ": " << dist[from][where_] << endl;
        cout << "way: ";
        vector<int> route = Getdist(from, where_, path);
        for (size_t i = 0; i < route.size(); i++) {
            if (i == route.size() - 1)
                cout << route[i] << ".";
            else
                cout << route[i] << " -> ";
        }
        cout << endl;
    }
    else {
        cout << "\nCannot be build." << endl;
    }
}
vector<int> reconstructPath(int s, int t, vector<vector<int>>& path) {
    vector<int> route;
    if (path[s][t] == -1) return route; // pas de chemin

    int current = t;
    while (current != s) {
        route.push_back(current);
        current = path[s][current];
    }
    route.push_back(s);
    reverse(route.begin(), route.end());
    return route;
}

bool DFS(const vector<vector<int>>& rGraph, int s, int t, vector<int>& parent) {
    int num_vertex = rGraph.size();
    vector<bool> visited(num_vertex, false);
    stack<int> stack;
    stack.push(s);
    visited[s] = true;

    while (!stack.empty()) {
        int u = stack.top();
        stack.pop();

        for (int v = 0; v < num_vertex; v++) {
            if (!visited[v] && rGraph[u][v] > 0) { // Si le sommet n'est pas visité et qu'il y a une capacité résiduelle
                stack.push(v);
                visited[v] = true;
                parent[v] = u; // Enregistrer le parent pour reconstruire le chemin

                if (v == t) {
                    return true; // Si on atteint le sommet cible
                }
            }
        }
    }
    return false; // Aucun chemin trouvé
}
int Ford_Fulkerson(const vector<vector<int>>& capacity, int s, int t, int num_vertex) {
    vector<vector<int>> rGraph = capacity; // Graphe de capacité résiduelle
    vector<int> parent(num_vertex); // Pour stocker le chemin augmentant
    int max_flow = 0; // Flux maximal

    // Tant qu'il existe un chemin augmentant
    while (DFS(rGraph, s, t, parent)) {
        // Trouver la capacité minimale du chemin trouvé
        int path_flow = INF;
        for (int v = t; v != s; v = parent[v]) {
            int u = parent[v];
            path_flow = min(path_flow, rGraph[u][v]);
        }

        // Mettre à jour les capacités résiduelles des arêtes et des arêtes inverses
        for (int v = t; v != s; v = parent[v]) {
            int u = parent[v];
            rGraph[u][v] -= path_flow; // Réduire la capacité de l'arête
            rGraph[v][u] += path_flow; // Augmenter la capacité de l'arête inverse
        }

        max_flow += path_flow; // Ajouter le flux du chemin trouvé au flux maximal
    }

    return max_flow;
}

void modyFloydWarshall(const vector<vector<int>>& cost_matrix, int num_vertex, int s, int t, vector<int>& parent) {
    vector<int> dist(num_vertex, INF); // Distance minimale trouvée
    vector<bool> visited(num_vertex, false); // Sommets déjà traités

    dist[s] = 0;
    parent[s] = -1;

    for (int i = 0; i < num_vertex; i++) {
        int u = -1;
        int min_dist = INF;

        // Trouver le sommet non visité avec la plus petite distance
        for (int v = 0; v < num_vertex; v++) {
            if (!visited[v] && dist[v] < min_dist) {
                min_dist = dist[v];
                u = v;
            }
        }

        if (u == -1) break; // Aucun sommet atteignable

        visited[u] = true;

        // Relaxation des voisins
        for (int v = 0; v < num_vertex; v++) {
            if (!visited[v] && cost_matrix[u][v] < INF) { // Arc existe
                if (dist[u] + cost_matrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + cost_matrix[u][v];
                    parent[v] = u;
                }
            }
        }
    }
}


int mincost_maxflow_fw(vector<vector<int>> cost_matrix,
    vector<vector<int>> bandwidth_matrix,
    int s, int t, int num_vertex, int need_flow) {
    int flow = 0;
    int cost = 0;
    vector<vector<int>> flow_matrix(num_vertex, vector<int>(num_vertex, 0));

    while (flow < need_flow) {
        int iteration = 0;
        vector<vector<int>> path(num_vertex, vector<int>(num_vertex, -1));
        auto dist = FloydWarshall(cost_matrix, num_vertex, path, iteration);

        // Pas de chemin disponible
        if (dist[s][t] == INF) break;

        auto route = reconstructPath(s, t, path);
        if (route.empty()) break;

        // Trouver le débit maximal sur ce chemin
        int delta = INF;
        for (int i = 0; i < route.size() - 1; i++) {
            delta = min(delta, bandwidth_matrix[route[i]][route[i + 1]]);
        }
        delta = min(delta, need_flow - flow);

        // Appliquer le débit
        for (int i = 0; i < route.size() - 1; i++) {
            int u = route[i], v = route[i + 1];
            bandwidth_matrix[u][v] -= delta;
            bandwidth_matrix[v][u] += delta;

            // Ajustement pour graphe résiduel
            cost_matrix[v][u] = -cost_matrix[u][v];

            flow_matrix[u][v] += delta;
            cost += delta * cost_matrix[u][v];
        }

        flow += delta;
    }

    // Affichage des flux
    for (int i = 0; i < num_vertex; i++) {
        for (int j = 0; j < num_vertex; j++) {
            if (flow_matrix[i][j] > 0) {
                cout << "Flux from " << i << " -> " << j
                    << " = " << flow_matrix[i][j]
                    << ", cost = " << cost_matrix[i][j] << endl;
            }
        }
    }

    return cost;
}

//determinant
int determinant(int num_vertex, vector<vector<int>> kirhgof_matrix) {
    int det = 0;
    vector<vector<int>> submatrix(num_vertex, vector<int>(num_vertex, 0));

    if (num_vertex == 2) {
        return ((kirhgof_matrix[0][0] * kirhgof_matrix[1][1]) - (kirhgof_matrix[1][0] * kirhgof_matrix[0][1]));
    }
    else {
        for (int x = 0; x < num_vertex; x++) {
            int subi = 0;
            for (int i = 1; i < num_vertex; i++) {
                int subj = 0;
                for (int j = 0; j < num_vertex; j++) {
                    if (j == x) {
                        continue;
                    }
                    submatrix[subi][subj] = kirhgof_matrix[i][j];
                    subj++;
                }
                subi++;
            }
            det = det + (pow(-1, x) * kirhgof_matrix[0][x] * determinant(num_vertex - 1, submatrix));
        }
    }
    return det;
}

struct edge {
    int cost;
    int from;
    int where_;
};

struct compare {
    bool operator() (edge const& a, edge const& b) const {
        return a.cost > b.cost;
    }
};

struct comparenum {
    bool operator() (edge const& a, edge const& b) const {
        return a.from < b.from;
    }
};


int find_root(int k, vector<int>parent) {
    if (parent[k] == k) {
        return k;
    }
    return find_root(parent[k], parent);
}

void union_(int f_p, int s_p, vector<int>& parent) {
    int x = find_root(f_p, parent);
    int y = find_root(s_p, parent);
    parent[x] = y;
}

vector<edge> Kruskal(vector<vector<int>> weight_matrix, int num_vertex, int& arb_weight, int& iter_kruskal) {
    vector<edge> edges;

    for (int i = 0; i < num_vertex; i++) {
        for (int j = 0; j < num_vertex; j++) {
            if (weight_matrix[i][j] != 0) {
                edges.push_back({ weight_matrix[i][j],i,j });
            }
        }
    }

    sort(edges.begin(), edges.end(), comparenum());

    vector<int> parent(num_vertex);
    for (int i = 0; i < num_vertex; i++) {
        parent[i] = i;
    }
    vector<edge> arb;
    while (arb.size() != num_vertex - 1) {
        edge next_edge = edges.back();
        edges.pop_back();
        int f_p = find_root(next_edge.from, parent);
        int s_p = find_root(next_edge.where_, parent);
        if (f_p != s_p) {
            arb.push_back(next_edge);
            arb_weight += next_edge.cost;
            union_(f_p, s_p, parent);
        }
        iter_kruskal++;
    }
    return arb;
}
vector<pair<int, int>> codePrufer(vector<edge> arb, int num_vertex) {
    vector<vector<int>> arb_matrix(num_vertex, vector<int>(num_vertex, 0));
    for (int k = 0; k < arb.size(); k++) {
        int i = arb[k].from;
        int j = arb[k].where_;
        int w = arb[k].cost;
        arb_matrix[i][j] = w;
        arb_matrix[j][i] = w;
    }
    vector<pair<int, int>> prufer;
    vector<int> degree_vert(num_vertex);
    for (int i = 0; i < num_vertex; i++) {
        for (int j = 0; j < num_vertex; j++) {
            if (arb_matrix[i][j] != 0) {
                degree_vert[i]++;
            }
        }
    }

    int r = 0;
    for (int i = 0; i < num_vertex; i++) {
        r += degree_vert[i];
    }

    while (r != 0) {
        for (int i = 0; i < num_vertex; i++) {
            if (degree_vert[i] == 1) {
                for (int j = 0; j < num_vertex; j++) {
                    if (arb_matrix[i][j] != 0) {
                        prufer.push_back(make_pair(arb_matrix[i][j], j));
                        degree_vert[i]--;
                        degree_vert[j]--;
                        arb_matrix[i][j] = 0;
                        arb_matrix[j][i] = 0;
                        r -= 2;
                    }
                }
            }
        }
    }

    return prufer;
}

vector<vector<int>> decodePrufer(vector<pair<int, int>> prufer) {
    int num_vertex = prufer.size() + 1;
    vector<vector<int>> ans_matrix(num_vertex, vector<int>(num_vertex, 0));
    vector<int> vertexes(num_vertex, 0);
    for (int i = 0; i < prufer.size(); i++) {
        vertexes[prufer[i].second] += 1;
    }

    int j = 0;
    int num = 0;
    for (int i = 0; i < num_vertex - 1; i++) {
        for (j = 0; j < num_vertex; j++) {
            if (i == num_vertex - 2) {
                if (vertexes[j] == 0) {
                    vertexes[j] = -1;
                    ans_matrix[j][prufer[i].second] = prufer[i].first;
                    ans_matrix[prufer[i].second][j] = prufer[i].first;
                    vertexes[prufer[i].second]--;
                    num = j;
                    break;
                }
            }
            else {
                if (vertexes[j] == 0 && num <= j) {
                    vertexes[j] = -1;
                    ans_matrix[j][prufer[i].second] = prufer[i].first;
                    ans_matrix[prufer[i].second][j] = prufer[i].first;
                    vertexes[prufer[i].second]--;
                    num = j;
                    break;
                }
            }

        }
    }
    return ans_matrix;
}
bool compare(const edge& e1, const edge& e2) {
    return e1.cost < e2.cost;
}
std::vector<edge> minimal_edge_cover(const std::vector<std::vector<int>>& weight_matrix, int num_vertex) {
    std::vector<edge> all_edges;

    // Collecter toutes les arêtes non nulles
    for (int i = 0; i < num_vertex; ++i) {
        for (int j = i + 1; j < num_vertex; ++j) {
            if (weight_matrix[i][j] != 0) {
                all_edges.push_back({ weight_matrix[i][j], i, j });
            }
        }
    }

    // Trier par poids croissan
    std::sort(all_edges.begin(), all_edges.end(), comparenum());

    std::vector<bool> covered(num_vertex, false);
    int covered_count = 0;
    std::vector<edge> cover_edges;

    // Parcourir les arêtes triées
    for (const auto& e : all_edges) {
        // Si au moins un sommet n’est pas encore couvert
        if (!covered[e.from] || !covered[e.where_]) {
            cover_edges.push_back(e);
            if (!covered[e.from]) { covered[e.from] = true; ++covered_count; }
            if (!covered[e.where_]) { covered[e.where_] = true; ++covered_count; }
        }
        if (covered_count == num_vertex) break; // tous couverts
    }

    return cover_edges;
}

//------------------------------------------------------------------------------------------------


vector<vector<int>> MakeEulerGraph(int num_vertex, vector<vector<int>> undirected_weight_matrix, vector<int> undirected_degrees) {
    vector<vector<int>> eulergraph(undirected_weight_matrix);
    int countevenvert = 0;
    bool flagEuler = false;
    //выводим матрицу стоимости
    cout << "\nweight matrix: " << endl;
    PrintMatrix(eulergraph);
    //вывод всех степеней вершин
    for (int i = 0; i < num_vertex; i++) {
        cout << "Vertex Degree No." << i << " = " << undirected_degrees[i] << endl;
    }
    //проверяем, что все степени вершин графа четные => эйлеров граф
    for (int i = 0; i < num_vertex; i++) {
        if (undirected_degrees[i] % 2) {
            countevenvert++;
        }
    }
    if (countevenvert == num_vertex) {
        cout << "The graph is Eulerian.\n" << endl;
        flagEuler = true;
    }
    //если не все степени вершин графа четные => надо модифицировать граф - сделать эйлеровым
    else {
        cout << "\nThe graph is not Eulerian. We modify the graph.\n" << endl;

        while (!flagEuler) {
            bool flag = 0;
            for (int i = 0; i < num_vertex; i++) {
                for (int j = 0; j < num_vertex; j++) {
                    //добавляем новое ребро, если у вершины нечетная степень и такого ребра еще нет
                    if (undirected_degrees[i] % 2 != 0 and undirected_degrees[j] % 2 != 0 and eulergraph[i][j] == 0 and i != j) {
                        flag = true;
                        int r = rand() % (10 - (1) + 1) + 1;
                        eulergraph[i][j] = r;
                        eulergraph[j][i] = r;
                        undirected_degrees[i]++;
                        undirected_degrees[j]++;
                        cout << "Added a rib: " << i << "<->" << j << " with weight: " << r << endl;
                        break;
                    }
                }
                //если добавили ребро, то можно проверить степени вершин на четность и выйти из цикла добавления вершины
                if (flag) {
                    break;
                }
            }
            //если не получилось добавить ребро, то пробуем удалить
            if (!flag) {
                for (int i = 0; i < num_vertex; i++) {
                    for (int j = 0; j < num_vertex; j++) {
                        if (undirected_degrees[i] % 2 != 0 and undirected_degrees[j] % 2 != 0 and eulergraph[i][j] != 0 and i != j) {
                            flag = true;
                            eulergraph[i][j] = 0;
                            eulergraph[j][i] = 0;
                            undirected_degrees[i]--;
                            undirected_degrees[j]--;
                            cout << "The rib was removed: " << i << "<->" << j << endl;
                            break;
                        }
                    }
                    //если удалили ребро, то можно проверить степени вершин на четность и выйти из цикла удаления вершины
                    if (flag) {
                        break;
                    }
                }
            }
            //проверяем получилось ли сделать граф с четными степенями вершин
            int countevenvert1 = 0;
            for (int i = 0; i < num_vertex; i++) {
                if (undirected_degrees[i] % 2 == 0) {
                    countevenvert1++;
                }
            }
            //если получилось, то поднимаем флаг и выходим из цикла, если нет - продолжаем добавлять/удалять ребро
            if (countevenvert1 == num_vertex) {
                flagEuler = true;
            }
        }
        cout << "Now the graph is Eulerian.\n" << endl;
        //выводим матрицу стоимости
        cout << "Weight matrix: " << endl;
        PrintMatrix(eulergraph);
        //вывод всех степеней вершин
        for (int i = 0; i < num_vertex; i++) {
            cout << "Vertex Degree No." << i << " = " << undirected_degrees[i] << endl;
        }
    }
    return eulergraph;
}

void FindEulerCycle(int num_vertex, vector<vector<int>> eulergraph) {
    stack<int> vertexes;
    vector<int> cycle;
    int vert = 0;
    //ищем первую вершину, из которой есть ребро
    for (vert = 0; vert < num_vertex; vert++) {
        for (int j = 0; j < num_vertex; j++) {
            if (eulergraph[vert][j] != 0) {
                break;
            }
        }
        if (vert < num_vertex) {
            break;
        }
    }
    //добавляем эту вершину в стек
    vertexes.push(vert);
    //пока стек не пустой
    while (!vertexes.empty()) {
        //берем верхнюю вершину
        vert = vertexes.top();
        int i;
        //находим первую смежную с ней
        for (i = 0; i < num_vertex; i++) {
            if (eulergraph[vert][i] != 0) {
                break;
            }
        }
        //если смежной не нашлось, то добавляем эту вершину в цикл, удаляем из стека
        if (i == num_vertex) {
            cycle.push_back(vert);
            vertexes.pop();
        }
        //если нашли смежную, то добавляем в стек и удаляем ребро между смежными вершинами в матрице
        else {
            vertexes.push(i);
            eulergraph[vert][i] = 0;
            eulergraph[i][vert] = 0;
        }
    }
    cout << "\nEuler cycle:" << endl;
    for (int i = 0; i < cycle.size(); i++) {
        cout << cycle[i];
        if (i != cycle.size() - 1) {
            cout << " -> ";
        }
    }
}


//вспомогательная функция для проверки возможности добавления вершины в путь
bool CanGoNext(int v, vector<vector<int>>& gamiltongraph, vector<int>& path, int position) {
    //проверяем, есть ли ребро между последней вершиной path[position - 1] и вершиной v в матрице смежности
    //если нет, то возвращаем false => вершина не мб добавлена в путь
    if (gamiltongraph[path[position - 1]][v] == 0) {
        return false;
    }
    //затем проверяем не была ли вершина уже посещена
    //если была, то возвращаем false => вершина не мб добавлена в путь
    for (int i = 0; i < position; i++) {
        if (path[i] == v) {
            return false;
        }
    }
    return true;
}


//функция для поиска гамильтонова цикла (рекурсивный)
bool FindGamiltonCycle(vector<vector<int>>& gamiltongraph, vector<int>& path, int position) {
    //если все вершины посещены и последняя соединена с первой, то возвращается true => есть гамильтонов цикл в графе
    if (position == gamiltongraph.size()) {
        return gamiltongraph[path[position - 1]][path[0]] != 0;
    }
    //иначе ищем следующую вершину для посещения
    for (int v = 1; v < gamiltongraph.size(); v++) {
        //в которую есть возможность дойти (есть исходящее ребро + не была посещена смежная вершина)
        if (CanGoNext(v, gamiltongraph, path, position)) {
            //если найлена, то добавляем эту вершину в путь
            path[position] = v;

            if (FindGamiltonCycle(gamiltongraph, path, position + 1)) {
                return true;
            }
            //если гамильтонов цикл не найден, то удаляем вершину v из пути
            path[position] = -1;
        }
    }
    return false;
}


//функция для проверки наличия гамильтонова цикла (рекурсивный)
bool ExistGamiltonCycle(vector<vector<int>>& gamiltongraph) {
    vector<int> path(gamiltongraph.size(), -1);
    path[0] = 0;
    if (!FindGamiltonCycle(gamiltongraph, path, 1)) {
        return false;
    }
    return true;
}


//гамильтонов граф: проверка, модификация
vector<vector<int>> MakeGamiltonGraph(int num_vertex, vector<vector<int>> undirected_weight_matrix) {
    vector<vector<int>> gamiltongraph(undirected_weight_matrix);
    //выводим матрицу стоимости
    cout << "\nMatrice de poids: " << endl;
    PrintMatrix(gamiltongraph);
    if (ExistGamiltonCycle(gamiltongraph)) {
        cout << "The graph is Halmitonian\n" << endl;
    }
    else {
        cout << "\nThe graph is not Halmitonian. Transform the graph " << endl;
        vector<int> vect(num_vertex, -1);

        for (int i = 0; i < num_vertex; i++) {
            int r = rand() % ((num_vertex - 1) - 0 + 1) + 0;
            if (find(vect.begin(), vect.end(), r) == vect.end()) {
                vect[i] = r;
            }
            else {
                i--;
            }
        }
        for (int i = 0; i < num_vertex; i++) {
            if (gamiltongraph[vect[i]][vect[(i + 1) % num_vertex]] == 0) {
                int r = rand() % (10 - (1) + 1) + 1;
                gamiltongraph[vect[i]][vect[(i + 1) % num_vertex]] = r;
                gamiltongraph[vect[(i + 1) % num_vertex]][vect[i]] = r;
                cout << "Added a rib: " << vect[i] << "<->" << vect[(i + 1) % num_vertex] << " с весом: " << r << endl;
            }
        }
        if (ExistGamiltonCycle(gamiltongraph)) {
            cout << "Now the Graph is Galmitonian." << endl;

            cout << "Matrix of weight: " << endl;
            PrintMatrix(gamiltongraph);
        }
        else {
            cout << "The graph cannot be transform and is not galmitonian\n" << endl;
        }
    }
    return gamiltongraph;
}

void findGamiltonCycleforTSP(vector<vector<int>>& gamiltongraph, vector<bool>& visited, vector<int>& path, int vert, int num_vertex, int count, int cost, int& min_cost, vector<int>& min_path, ofstream& fout) {

    path.push_back(vert);

    if (count == num_vertex and gamiltongraph[vert][0]) {
        for (int i = 0; i < path.size(); i++) {
            fout << path[i] << " -> ";

        }
        fout << path[0];
        fout << "\tWeight cycle: " << gamiltongraph[vert][0] + cost << endl;
        if (cost + gamiltongraph[vert][0] < min_cost) {
            min_cost = cost + gamiltongraph[vert][0];
            min_path = path;
        }
        path.pop_back();
        return;
    }
    for (int i = 0; i < num_vertex; i++) {
        if (!visited[i] and gamiltongraph[vert][i] != 0) {
            visited[i] = true;
            findGamiltonCycleforTSP(gamiltongraph, visited, path, i, num_vertex, count + 1, cost + gamiltongraph[vert][i], min_cost, min_path, fout);
            visited[i] = false;
        }
    }
    path.pop_back();
}

void TransportSalesmanProblem(vector<vector<int>>& gamiltongraph, int num_vertex, ofstream& fout) {
    vector<bool> visited(num_vertex);
    vector<int> path;
    visited[0] = true;


    int min_cost = INF;
    vector<int> min_path;
    findGamiltonCycleforTSP(gamiltongraph, visited, path, 0, num_vertex, 1, 0, min_cost, min_path, fout);
    cout << "The Traveling Salesman Cycle: " << endl;
    for (int i = 0; i < min_path.size(); i++) {
        cout << min_path[i] << " -> ";
    }
    cout << min_path[0];
    cout << "\nWeight: " << min_cost << endl;
    fout.close();
}

std::ostream& operator<<(std::ostream& os, const edge& e) {
    os << e.from << " <-> " << e.where_;
    return os;
}
pair<int, int> find_source_and_sink(const vector<vector<int>>& capacity) {
    int n = capacity.size();
    vector<int> in_degree(n, 0);
    vector<int> out_degree(n, 0);

    // Compter les degrés entrants et sortants
    for (int u = 0; u < n; ++u) {
        for (int v = 0; v < n; ++v) {
            if (capacity[u][v] > 0) {
                out_degree[u]++;
                in_degree[v]++;
            }
        }
    }

    // Trouver une source (zéro entrée) et un puits (zéro sortie)
    int source = -1, sink = -1;
    for (int i = 0; i < n; ++i) {
        if (in_degree[i] == 0 && source == -1) {
            source = i;
        }
        if (out_degree[i] == 0 && sink == -1) {
            sink = i;
        }
    }

    return { source, sink };
}

void printmenu() {
    printf("------------------------------------MENU----------------------------------\n");
    printf("\nEnter a number from 0 to 8:\n");
    printf("0 - End the program\n");
    printf("1 - Generate a new graph\n");
    printf("\nLaboratory 1\n");
    printf("2 - Display the adjacency matrix of vertices\n");
    printf("3 - Display the weight matrix\n");
    printf("4 - Zhimbel method\n");
    printf("5 - Find the number of routes from one vertex to another\n");
    printf("\nLaboratory 2\n");
    printf("6 - Perform Depth-First Search (DFS)\n");
    printf("7 - Algorithm Floyd-Warshall\n");
    printf("8 - Compare algorithms \n");
    printf("\nLaboratory 3\n");
    printf("9 - Print the capacity matrix\n");
    printf("10 - Print the cost matrix\n");
    printf("11 - Find the maximum flow using the Ford-Fulkerson algorithm\n");
    printf("12 - Find the given minimum cost flow\n");
    printf("\nLaboratory 4\n");
    printf("13 - Print Kirchhoff Matrix\n");
    printf("14 - Find the number of spanning trees\n");
    printf("15 - Build the minimum weight spanning tree using Kruskal's algorithm\n");
    printf("16 - Encode and decode using the Prüfer code\n");
    printf("17 - Minimum rib cover\n");
    printf("\nLaboratory 5\n");
    printf("18 - Check: is the graph Eulerian. If not, modify. Build an Euler cycle.\n");
    printf("19 - Check: is the graph Hamiltonian. If not, modify. Build Hamiltonian cycles, write to file. Traveling salesman problem.\n\n");
    printf("---------------------------------------------------------------------------\n");
}

// Menu function
void menu(bool& exitflag) {
    while (exitflag) {
        cout << "Enter the number of vertices in the graph:" << endl;
        int num_vertex = intinput();
        while (num_vertex < 2 || num_vertex > 100) {
            printf("\nThe number of vertices in the graph can be from 2 to 100. Please try again.\n");
            num_vertex = intinput();
        }
        vector<int> degrees(num_vertex);
        //double p = 0.3;
        double p;
        p = 0.5;
        /* do {
         printf("\nEnter the probability of bernoulli.\n");

         cin >> p;
         } while (p < 0 || p>1);*/
        vector<vector<int>> adjMatrix = generateConnectedDAG(num_vertex, p, degrees);

        // Output degrees of vertices
        for (int i = 0; i < num_vertex; i++) {
            cout << "Degree of vertex " << i << " = " << degrees[i] << endl;
        }


        // Weight matrix choice
        cout << "\nChoose: 1 - graph with positive weights, 2 - graph with positive and negative weights:" << endl;
        int sign = intinput();
        while (sign < 1 || sign > 2) {
            cout << "Incorrect input. Choose: 1 - graph with positive weights, 2 - graph with positive and negative weights:" << endl;
            sign = intinput();
        }

        vector<vector<int>> weight_matrix(num_vertex, vector<int>(num_vertex, 0));
        srand(static_cast<unsigned>(time(0))); // Seed once
        for (int i = 0; i < num_vertex; i++) {
            for (int j = 0; j < num_vertex; j++) {
                if (adjMatrix[i][j] != 0) {
                    weight_matrix[i][j] = (sign == 1) ? rand() % 10 + 1 : rand() % 21 - 10;
                }
            }
        }

        // Identity matrix
        vector<vector<int>> ed_matrix(num_vertex, vector<int>(num_vertex, 0));
        for (int i = 0; i < num_vertex; i++) {
            ed_matrix[i][i] = 1;
        }

        // Generate the reachability matrix
        vector<vector<int>> reachability_matrix = adjMatrix;
        vector<vector<int>> res_reachability_matrix(num_vertex, vector<int>(num_vertex, 0));
        res_reachability_matrix = Addition_matrix(ed_matrix, adjMatrix, num_vertex);
        for (int i = 0; i < num_vertex; i++) {
            reachability_matrix = Generate_reachability_matrix(adjMatrix, reachability_matrix, num_vertex);
            res_reachability_matrix = Addition_matrix(res_reachability_matrix, reachability_matrix, num_vertex);
        }

        vector<vector<int>> bandwidth_matrix(num_vertex, vector<int>(num_vertex, 0));
        srand(time(0));
        for (int i = 0; i < num_vertex; i++) {
            for (int j = 0; j < num_vertex; j++) {
                if (adjMatrix[i][j] != 0) {
                    //rand()%(end-start+1)+start
                    bandwidth_matrix[i][j] = rand() % (10 - (1) + 1) + 1;
                }
            }
        }
        vector<vector<int>> cost_matrix(num_vertex, vector<int>(num_vertex, 0));
        srand(time(0) + 1);
        for (int i = 0; i < num_vertex; i++) {
            for (int j = 0; j < num_vertex; j++) {
                if (adjMatrix[i][j] != 0) {
                    //rand()%(end-start+1)+start
                    cost_matrix[i][j] = rand() % (10 - (1) + 1) + 1;


                }
            }
        }

        //Matrice non orienté 
        vector<vector<int>> undirected_adjacency_matrix(num_vertex, vector<int>(num_vertex, 0));
        for (int i = 0; i < num_vertex; i++) {
            for (int j = 0; j < num_vertex; j++) {
                if (adjMatrix[i][j] != 0) {
                    undirected_adjacency_matrix[j][i] = adjMatrix[i][j];
                    undirected_adjacency_matrix[i][j] = adjMatrix[i][j];
                }
            }
        }

        //matrice des degrées
        vector<vector<int>> degree_matrix(num_vertex, vector<int>(num_vertex, 0));
        for (int i = 0; i < num_vertex; i++) {
            //int d = 0;
            for (int j = 0; j < num_vertex; j++) {
                if (undirected_adjacency_matrix[i][j] != 0) {
                    degree_matrix[i][i] += 1;
                }
            }

        }

        vector<int> undirected_degrees(num_vertex, 0);
        for (int i = 0; i < num_vertex; i++) {
            undirected_degrees[i] = degree_matrix[i][i];
        }



        vector<vector<int>> kirhgof_matrix(num_vertex, vector<int>(num_vertex, 0));
        for (int i = 0; i < num_vertex; i++) {
            for (int j = 0; j < num_vertex; j++) {
                kirhgof_matrix[i][j] = degree_matrix[i][j] - undirected_adjacency_matrix[i][j];
            }
        }


        vector<vector<int>> undirected_weight_matrix(num_vertex, vector<int>(num_vertex, 0));
        for (int i = 0; i < num_vertex; i++) {
            for (int j = 0; j < num_vertex; j++) {
                if (cost_matrix[i][j] != 0) {
                    undirected_weight_matrix[j][i] = weight_matrix[i][j];
                    undirected_weight_matrix[i][j] = weight_matrix[i][j];
                }
            }
        }






        printmenu();
        int input = 0;
        int iter = 0;
        int iteration = 0;
        float iter_div = 0;
        ofstream fout;
        fout.open("GamiltonCycles.txt");
        bool flag = true;
        while (flag) {
            input = intinput();
            switch (input) {
            case 0: {
                printf("\nYou selected function 0 - end the program\n");
                flag = false;
                exitflag = false;
                break;
            }
            case 1: {
                printf("\n function 1 - Generate a new graph\n");
                flag = false;
                break;
            }
            case 2: {
                printf("\n function 2 - Display the adjacency matrix of vertices\n");
                PrintMatrix(adjMatrix);
                cout << "\n";
                printmenu();
                break;
            }
            case 3: {
                printf("\nfunction 3 - Display the weight matrix\n");
                PrintMatrix(weight_matrix);
                cout << "\n";
                printmenu();
                break;
            }
            case 4: {
                printf("\n function 4 - Zhimbel method\n");
                if (num_vertex <= 1) {
                    cout << "The Zhimbel method cannot be used with 1 vertex." << endl;
                }
                else {
                    cout << "Enter the length of the route (the length of the route must be from 1 to " << num_vertex - 1 << " edges): " << endl;
                    int length = intinput();
                    while (length < 1 || length > num_vertex - 1) {
                        cout << "The route length must be from 1 to " << num_vertex - 1 << " edges. Please try again." << endl;
                        length = intinput();
                    }

                    cout << "Choose: 1 - find the maximum length route; 2 - find the minimum length route" << endl;
                    int parametr = intinput();
                    while (parametr < 1 || parametr > 2) {
                        cout << "Incorrect input. Choose: 1 - find the maximum length route; 2 - find the minimum length route." << endl;
                        parametr = intinput();
                    }
                    vector<vector<int>> zhimbel_matrix(weight_matrix);
                    for (int i = 0; i < length - 1; i++) {
                        zhimbel_matrix = Zhimbel(weight_matrix, zhimbel_matrix, num_vertex, parametr);
                    }
                    PrintMatrix(zhimbel_matrix);
                }
                cout << "\n";
                printmenu();
                break;
            }
            case 5: {
                printf("\n function 5 - Find the number of routes from one vertex to another.\n");
                int from, where_;
                GetStartFinishVertex(from, where_, num_vertex);

                if (res_reachability_matrix[from][where_] != 0) {
                    cout << "\nThe number of routes from vertex " << from << " to vertex " << where_ << ": " << res_reachability_matrix[from][where_] << endl;
                }
                else {
                    cout << "\nSuch a route cannot be built." << endl;
                }
                cout << "\n";
                printmenu();
                break;
            }
            case 6:
            {
                printf("\n 6 -DepthFirstSearch Algorithm\n");
                int start;
                cout << "Enter start vertex for DFS: ";
                cin >> start;
                DepthFirstSearch(adjMatrix, start, iteration);
                cout << "\nNumber of iterations " << iteration << endl;
                printmenu();
                break;
            }
            case 7:
            {
                printf("\n 7 - Floyd-Warshall Algorithm\n");

                vector<vector<int>> H(num_vertex, vector<int>(num_vertex));
                vector<vector<int>> shortest_paths = FloydWarshall(weight_matrix, num_vertex, H, iter);

                int start, end;
                cout << "\nEnter two vertices : ";
                cin >> start >> end;

                if (shortest_paths[start][end] == 0) {
                    cout << "No path found between " << start << " and " << end << ".\n";
                }
                else {
                    PrintWayInfo(num_vertex, shortest_paths, start, end, H);
                    cout << "\nNumber of iterations " << iter << endl;
                }
                cout << "\n";
                printmenu();
                break;
            }
            case 8:
            {
                iter_div = float(iter) / float(iteration);
                cout << "\nThe depth-first search algorithm is " << iter_div << " time faster than the FloydWarshall algorithm" << endl;

            }
            case 9:
            {
                printf("\nOutput the capacity matrix\n");
                PrintMatrix(bandwidth_matrix);
                cout << "\n";
                printmenu();
                break;
            }
            case 10:
            {
                printf("\n The cost matrix\n");
                PrintMatrix(cost_matrix);
                cout << "\n";
                printmenu();
                break;
            }
            case 11: {

                printf("\nFind the maximum flux by the Ford-Fulkerson algorithm\n");
                // Trouver source et puits automatiquement
                pair<int, int> endpoints = find_source_and_sink(bandwidth_matrix);
                int from = endpoints.first;
                int where_ = endpoints.second;

                if (from == -1 || where_ == -1) {
                    cout << "Erreur : Impossible de déterminer une source ou un puits valide.\n";
                    break;
                }
                int max_flow = Ford_Fulkerson(bandwidth_matrix, from, where_, num_vertex);
                cout << from << endl;
                cout << where_ << endl;
                cout << "Maximum flow: " << max_flow << endl;
                cout << "\n";
                printmenu();
                break;
            }
            case 12: {
                printf("\n Find the minimum cost\n");
                pair<int, int> endpoints = find_source_and_sink(bandwidth_matrix);
                int from = endpoints.first;
                int where_ = endpoints.second;

                if (from == -1 || where_ == -1) {
                    cout << "Erreur : Impossible de déterminer une source ou un puits valide.\n";
                    break;
                }
                int max_flow = Ford_Fulkerson(bandwidth_matrix, from, where_, num_vertex);

                int need_flow = (max_flow * 2) / 3;
                if (need_flow == 0) {
                    cout << "The flux is 0, the minimum cost cannot be given" << endl;
                }
                else {
                    cout << "The maximum flux: " << max_flow << endl;
                    cout << "The given flux [2/3*max]: " << need_flow << endl;
                    int cost = mincost_maxflow_fw(cost_matrix, bandwidth_matrix, from, where_, num_vertex, need_flow);
                    cout << "The minimum cost for the flux : " << cost << endl;
                }

                cout << "\n";
                printmenu();
                break;
            }
            case 13: {
                printf("\nPrint Kirchhoff matrix\n");
                PrintMatrix(undirected_adjacency_matrix);
                PrintMatrix(degree_matrix);
                // PrintMatrix(undirected_degrees);
                PrintMatrix(kirhgof_matrix);
                cout << "\n";
                printmenu();
                break;
            }
            case 14: {
                printf("\n Find the number of spanning trees\n");
                vector<vector<int>> new_kirhgof_matrix(num_vertex - 1, vector<int>(num_vertex - 1, 0));
                for (int i = 0; i < num_vertex - 1; i++) {
                    for (int j = 0; j < num_vertex - 1; j++) {
                        new_kirhgof_matrix[i][j] = kirhgof_matrix[i + 1][j + 1];
                    }
                }
                if (num_vertex == 2) {
                    cout << "Number of spanning trees: " << 1 << "\n";
                }
                else {
                    cout << "Number of spanning trees: " << determinant(num_vertex - 1, new_kirhgof_matrix) << "\n";
                }
                cout << "\n";
                printmenu();
                break;
            }
            case 15: {
                printf("\n Build a minimum weight spanning tree using Kruskal's algorithm\n");
                int arb_weight = 0;
                int iter_kruskal = 0;
                vector<edge> kr = Kruskal(undirected_weight_matrix, num_vertex, arb_weight, iter_kruskal);
                cout << "The weight matrix of an undirected graph: " << endl;
                PrintMatrix(undirected_weight_matrix);
                cout << "\nThe number of algorithm iterations: " << iter_kruskal << endl;
                cout << "The minimum weight spanning tree: " << arb_weight << endl;
                for (int i = 0; i < kr.size(); i++) {
                    cout << kr[i].from << " <-> " << kr[i].where_ << " weight: " << "" << kr[i].cost << endl;
                }
                cout << "\n";
                printmenu();
                break;
            }
            case 16: {
                printf("\n Encode and decode a skeleton using Prüfer code\n");
                int arb_weight = 0;
                int iter_kruskal = 0;
                vector<edge> kr = Kruskal(undirected_weight_matrix, num_vertex, arb_weight, iter_kruskal);
                vector<pair<int, int>> pr = codePrufer(kr, num_vertex);
                cout << "Prufer code: " << endl;
                for (int i = 0; i < pr.size(); i++) {
                    cout << "vertex number: " << pr[i].second << " edge weight: " << pr[i].first << endl;
                }
                vector<vector<int>> dec_pr = decodePrufer(pr);
                cout << "\nDecoded Prüfer code: " << endl;
                PrintMatrix(dec_pr);

                cout << "\n";
                printmenu();
                break;
            }

            case 17: {
                string user_choice;
                cout << "Choose 1 - for the original graph or 2 - for the resulting skeleton : ";
                cin >> user_choice;

                vector<edge> edge_cover;

                if (user_choice == "1") {

                    edge_cover = minimal_edge_cover(undirected_adjacency_matrix, num_vertex);
                }
                else if (user_choice == "2") {
                    // On régénère le squelette (arbre couvrant) avec Kruskal
                    int arb_weight = 0;
                    int iter_kruskal = 0;
                    vector<edge> kr = Kruskal(undirected_weight_matrix, num_vertex, arb_weight, iter_kruskal);

                    // Convertir les arêtes du MST en matrice d'adjacence
                    vector<vector<int>> mst_matrix(num_vertex, vector<int>(num_vertex, 0));
                    for (const auto& e : kr) {
                        mst_matrix[e.from][e.where_] = e.cost;
                        mst_matrix[e.where_][e.from] = e.cost;
                    }

                    edge_cover = minimal_edge_cover(mst_matrix, num_vertex);
                }
                else {
                    cout << "Invalid choice.Please enter '1' or '2'." << endl;
                    break;
                }

                int total_weight = 0;
                // Affichage du résultat
                cout << "Minimum rib cover : " << endl;
                for (const auto& e : edge_cover) {
                    cout << e << " " << endl;
                    // std::cout << e.from << " <-> " << e.where_ << " poids: " << e.cost << "\n";
                    total_weight += e.cost;
                }
                cout << endl;

                printmenu();
                break;
            }
            case 18: {
                printf("\n Check: is the graph Eulerian. If not, modify. Build Eulerian cycles, write to file. Traveling salesman problem.\n");
                vector<vector<int>> eulergraph = MakeEulerGraph(num_vertex, undirected_weight_matrix, undirected_degrees);
                if (num_vertex <= 2) {
                    cout << "The graph cannot be Eulerian since it contains only 2 vertices." << endl;
                }
                else {
                    FindEulerCycle(num_vertex, eulergraph);  // Affiche le cycle d’Euler
                }
                cout << "\n";
                printmenu();
                break;
            }
            case 19: {
                printf("\n Check: is the graph Hamiltonian. If not, modify. Build Hamiltonian cycles, write to file. Traveling salesman problem.\n");
                vector<vector<int>> gamiltongraph(num_vertex, vector<int>(num_vertex, 0));

                cout << "\nDAG initial:\n";
                PrintMatrix(weight_matrix);;

                if (num_vertex <= 2) {
                    cout << "The graph cannot be Hamiltonian, since it consists of 2 vertices" << endl;
                }
                else {
                    gamiltongraph = MakeGamiltonGraph(num_vertex, undirected_weight_matrix);
                    TransportSalesmanProblem(gamiltongraph, num_vertex, fout);
                }

                cout << "\n";
                printmenu();
                break;
            }
            default:
                printf("\nIncorrect input. Please try again.\n");
                break;
            }
        }
    }
}

int main() {
    setlocale(LC_ALL, "Russian");
    srand(static_cast<unsigned>(time(0)));
    bool exitflag = true;
    menu(exitflag);
    return 0;
}