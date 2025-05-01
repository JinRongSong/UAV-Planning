#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <tuple>
#include <limits>
#include <functional>
#include <set>

using namespace std;

const int radius = 1;  // UAV Radius
static int dim = 10; // Dimensions to check for RRT - Connect

struct position {
    int x, y, z;
    bool operator==(const position& other) const { return x == other.x && y == other.y && z == other.z; }
    bool operator!=(const position& other) const { return !(*this == other); }
    bool operator<(const position& other) const { return tie(x, y, z) < tie(other.x, other.y, other.z); }
};

namespace std {
    template <>
    struct hash<position> {
        size_t operator()(const position& v) const {
            return ((v.x * 12321) ^ (v.y * 1232342) ^ (v.z * 2137));
        }
    };
}


struct Obstacle {
    position pos;
    int radius;
    Obstacle(position p, int r) : pos(p), radius(r) {}
};



unordered_map<int, vector<Obstacle>> dynamic_obstacles;

struct node {
    position pos;
    int time;
    double gCost, hCost;
    node* parent;


    node(position p, int t) : pos(p), time(t), gCost(0), hCost(0), parent(nullptr) {}


    double fCost() const { return gCost + hCost; }
};

struct CompareNode {
    bool operator()(const node* a, const node* b) {
        return a->fCost() > b->fCost();
    }
};

bool isOccupied(position pos, int time) {
    if (dynamic_obstacles.count(time)) {
        for (const auto& obstacle : dynamic_obstacles[time]) {
            double dist = sqrt(pow(pos.x - obstacle.pos.x, 2) + 
                               pow(pos.y - obstacle.pos.y, 2) + 
                               pow(pos.z - obstacle.pos.z, 2));
            if (dist <= obstacle.radius + radius) {
                return true;  // Occupied if UAV would intersect with the obstacle
            }
        }
    }
    return false;
}

double euclidean(position a, position b) {
    return sqrt((a.x - b.x)*(a.x - b.x) +
                (a.y - b.y)*(a.y - b.y) +
                (a.z - b.z)*(a.z - b.z));
}

vector<position> get_neighbors(position p) {
    vector<position> neighbors;
    for (int dx = -1; dx <= 1; dx++){
        for (int dy = -1; dy <= 1; dy++){
            for (int dz = -1; dz <= 1; dz++) {
                if (dx == 0 && dy == 0 && dz == 0){
                    continue;
                }

                neighbors.push_back({p.x + dx, p.y + dy, p.z + dz});
            }
        }
    }
    return neighbors;
}

vector<tuple<int, position>> reconstruct_path(node* end) {
    vector<tuple<int, position>> path;

    while (end != nullptr) {
        path.push_back({end->time, end->pos});
        end = end->parent;
    }

    reverse(path.begin(), path.end());
    return path;
}

vector<tuple<int, position>> astar(position start, position goal) {
    priority_queue<node*, vector<node*>, CompareNode> open;

    unordered_map<pair<position, int>, double, 
        function<size_t(const pair<position, int>&)>> cost_so_far(0, 
        [](const pair<position, int>& p) {
            return hash<position>()(p.first) ^ hash<int>()(p.second);
        });

    unordered_map<pair<position, int>, node*, 
        function<size_t(const pair<position, int>&)>> all_nodes(0,
        [](const pair<position, int>& p) {
            return hash<position>()(p.first) ^ hash<int>()(p.second);
        });

    node* startNode = new node(start, 0);

    startNode->hCost = euclidean(start, goal);

    open.push(startNode);
    cost_so_far[{start, 0}] = 0;
    all_nodes[{start, 0}] = startNode;

    while (!open.empty()) {
        node* current = open.top();
        open.pop();

        if (current->pos == goal) {
            return reconstruct_path(current);
        }

        for (position neighbor : get_neighbors(current->pos)) {
            int new_time = current->time + 1;
            if (isOccupied(neighbor, new_time)){
                continue;
            }

            double new_cost = current->gCost + 1;

            auto key = make_pair(neighbor, new_time);

            if (!cost_so_far.count(key) || new_cost < cost_so_far[key]) {
                cost_so_far[key] = new_cost;
                node* nextNode = new node(neighbor, new_time);
                nextNode->gCost = new_cost;
                nextNode->hCost = euclidean(neighbor, goal);
                nextNode->parent = current;
                open.push(nextNode);
                all_nodes[key] = nextNode;
            }
        }
    }

    return {};
}





void export_path_rrt(const vector<position>& path, const string& filename) {
    ofstream file(filename);
    int t = 0;
    for (auto& p : path) {
        file << t++ << " " << p.x << " " << p.y << " " << p.z << "\n";
    }
    file.close();
}

void export_path_astar(const vector<tuple<int, position>>& path, const string& filename) {
    ofstream file(filename);
    for (auto& [t, p] : path) {
        file << t << " " << p.x << " " << p.y << " " << p.z << "\n";
    }
    file.close();
}

void export_dynamic_obstacles(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing.\n";
        return;
    }

    for (const auto& [t, obstacles] : dynamic_obstacles) {
        for (const auto& obs : obstacles) {
            file << t << " " << obs.pos.x << " " << obs.pos.y << " " << obs.pos.z << " " << obs.radius << "\n";
        }
    }

    file.close();
}





struct RRTNode {
    position pos;
    RRTNode* parent;
    RRTNode(position p) : pos(p), parent(nullptr) {}
};


bool isFree(position pos) {
    for (const auto& [t, obs_at_t] : dynamic_obstacles) {
        for (const auto& obstacle : obs_at_t) {
            double dist = sqrt(pow(pos.x - obstacle.pos.x, 2) + 
                               pow(pos.y - obstacle.pos.y, 2) + 
                               pow(pos.z - obstacle.pos.z, 2));
            if (dist <= obstacle.radius + radius) {
                return false; 
            }
        }
    }
    return true;
}


position randomSample(int xmin, int xmax, int ymin, int ymax, int zmin, int zmax) {
    return {rand() % (xmax - xmin + 1) + xmin,
            rand() % (ymax - ymin + 1) + ymin,
            rand() % (zmax - zmin + 1) + zmin};
}

position steer(position from, position to, int stepSize = 1) {
    position direction = {to.x - from.x, to.y - from.y, to.z - from.z};

    double length = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);

    if (length == 0) {
      return from;  
    }

    direction = {static_cast<int>(round(from.x + stepSize * direction.x / length)), static_cast<int>(round(from.y + stepSize * direction.y / length)), static_cast<int>(round(from.z + stepSize * direction.z / length))};
    return direction;
}

RRTNode* nearest(const vector<RRTNode*>& tree, position sample) {
    RRTNode* closest = nullptr;
    double minDist = numeric_limits<double>::infinity();
    for (auto* node : tree) {
        double dist = euclidean(node->pos, sample);
        if (dist < minDist) {
            minDist = dist;
            closest = node;
        }
    }
    return closest;
}

bool connect(vector<RRTNode*>& tree, position target, RRTNode*& new_node, int maxStep = 1000) {
    RRTNode* nearest_node = nearest(tree, target);
    position current = nearest_node->pos;
    for (int i = 0; i < maxStep; ++i) {
        position next = steer(current, target);
        if (!isFree(next)) return false;
        RRTNode* node = new RRTNode(next);
        node->parent = nearest_node;
        tree.push_back(node);
        nearest_node = node;
        current = next;
        if (current == target) {
            new_node = node;
            return true;
        }
    }
    new_node = nearest_node;
    return false;
}

vector<position> reconstruct_rrt_path(RRTNode* node) {
    vector<position> path;
    while (node) {
        path.push_back(node->pos);
        node = node->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}


vector<position> rrt_connect(position start, position goal, int iterations = 5000) {
    vector<RRTNode*> treeStart{new RRTNode(start)};

    vector<RRTNode*> treeGoal{new RRTNode(goal)};

    int xmin = 0, ymin = 0, zmin = 0, xmax = dim, ymax = dim, zmax = dim;

    for (int i = 0; i < iterations; ++i) {

        position randSample = randomSample(xmin, xmax, ymin, ymax, zmin, zmax);
        RRTNode* newStartNode = nullptr;

        if (!connect(treeStart, randSample, newStartNode)){
            continue;
        }

        RRTNode* newGoalNode = nullptr;
        if (connect(treeGoal, newStartNode->pos, newGoalNode)) {
            vector<position> path1 = reconstruct_rrt_path(newStartNode);
            vector<position> path2 = reconstruct_rrt_path(newGoalNode);


            reverse(path2.begin(), path2.end());
            
            path1.insert(path1.end(), path2.begin(), path2.end());

            for (auto* node : treeStart){
                delete node;
            }
            for (auto* node : treeGoal){
                delete node;
            }

            return path1;
        }

        swap(treeStart, treeGoal);
    }

    for (auto* node : treeStart){
        delete node;
    } 
    for (auto* node : treeGoal){
        delete node;
    }
    return {};
}


const static int totTime = 1000;

int main() {
    // Static Obstacles 10x10
    for (int t = 0; t <= totTime; ++t) {
        dynamic_obstacles[t].push_back(Obstacle({4, 5, 5}, 2));  // Obstacle with radius 2
        dynamic_obstacles[t].push_back(Obstacle({10, 3, 5}, 2));  // Obstacle with radius 3
        dynamic_obstacles[t].push_back(Obstacle({6, 6, 2}, 1));  // Obstacle with radius 1
        dynamic_obstacles[t].push_back(Obstacle({5, 5, 5}, 2));  // Obstacle with radius 2
    }

    // // Dynamic Obstacles 10x10
    // for (int t = 0; t <= totTime; ++t) {
    //     dynamic_obstacles[t].push_back(Obstacle({4 + t, 5, 5}, 2));
    //     dynamic_obstacles[t].push_back(Obstacle({10, t, 5}, 2));
    //     dynamic_obstacles[t].push_back(Obstacle({6, 6, t}, 1));
    //     dynamic_obstacles[t].push_back(Obstacle({10 - t, 10 - t, 10 - t}, 2));
    // }

    // // Static Obstacles 50x50x50
    // for (int t = 0; t <= totTime; ++t) {
    //     dynamic_obstacles[t].push_back(Obstacle({5, 5, 5}, 3));
    //     dynamic_obstacles[t].push_back(Obstacle({25, 25, 25}, 4));
    //     dynamic_obstacles[t].push_back(Obstacle({40, 10, 10}, 2));
    //     dynamic_obstacles[t].push_back(Obstacle({10, 40, 40}, 3));
    //     dynamic_obstacles[t].push_back(Obstacle({30, 30, 5}, 5));
    //     dynamic_obstacles[t].push_back(Obstacle({5, 30, 30}, 2));
    // }


    // // Dynamic Obstacles 50x50x50
    // for (int t = 0; t <= totTime; ++t) {
    //     dynamic_obstacles[t].push_back(Obstacle({5 + t, 5 + t/5, 5}, 3));
    //     dynamic_obstacles[t].push_back(Obstacle({25, 25 + t/10, 25}, 4));
    //     dynamic_obstacles[t].push_back(Obstacle({40, t/20-5, 10}, 2));
    //     dynamic_obstacles[t].push_back(Obstacle({10, 40, t/3}, 3));
    //     dynamic_obstacles[t].push_back(Obstacle({10, 40-t/50, 5}, 5));
    //     dynamic_obstacles[t].push_back(Obstacle({5 + t/2, 30, 30}, 2));
    // }

    // 10x10x10
    position start = {0, 0, 0};
    position goal = {10, 10, 10};


    // // 50x50x50
    // position start = {2, 2, 2};
    // position goal = {47, 47, 47};


    auto start_time = chrono::high_resolution_clock::now();

    // A*
    auto path_astar = astar(start, goal);
    if (!path_astar.empty()) {
        cout << "Path found with " << path_astar.size() << " steps.\n";
        export_path_astar(path_astar, "path.txt");
    } else {
        cout << "No path found.\n";
    }




    // // RRT-Connect
    // auto path = rrt_connect(start, goal);
    // if (!path.empty()) {
    //     cout << "Path found with " << path.size() << " steps.\n";
    //     export_path_rrt(path, "path.txt");
    // } else {
    //     cout << "No path found.\n";
    // }



    auto end_time = chrono::high_resolution_clock::now();

	auto elapsed_time = chrono::duration<double, milli>(end_time - start_time);

	cout << "Runtime: " << elapsed_time.count() << " ms" << endl;

    export_dynamic_obstacles("obstacles_dynamic.txt");

    return 0;
}