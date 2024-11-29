#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>

const int dx[] = {-1, 1, 0, 0, -1, 1, -1, 1};
const int dy[] = {0, 0, -1, 1, -1, -1, 1, 1}; //좌,우, 위, 아래, 대각선2

struct Node {
    int x, y;
    int g_cost, h_cost;
    Node* parent;

    Node(int x, int y, int g_cost, int h_cost, Node* parent = nullptr) 
        : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(parent) {}

    int f_cost() const {
        return g_cost + h_cost;
    }

    bool operator>(const Node& other) const {
        return f_cost() > other.f_cost();
    }
};


Eigen::MatrixXi readCSV(const std::string& filename, int& rows, int& cols) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        throw std::runtime_error("File opening failed");
    }

    std::string line;
    std::vector<std::vector<int>> grid;
    int max_cols = 0;

    rows = 0;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        std::stringstream ss(line);
        std::string value;
        std::vector<int> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(std::stoi(value));
        }

        
        max_cols = std::max(max_cols, static_cast<int>(row.size()));
        grid.push_back(row);
        rows++;
    }

    
    cols = max_cols;
    Eigen::MatrixXi mat(rows, cols);
    mat.setZero();

    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < grid[i].size(); ++j) {
            mat(i, j) = grid[i][j];
        }
    }

    return mat;
}



// Manhattan distance
int heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

bool isValid(int x, int y, int rows, int cols, const Eigen::MatrixXi& grid) {
    return x >= 0 && y >= 0 && x < rows && y < cols && grid(x, y) == 0;
}

//-----------------------------------A*-----------------------------------------//
std::vector<std::pair<int, int>> AStar(int startX, int startY, int goalX, int goalY, const Eigen::MatrixXi& grid, int rows, int cols) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::unordered_map<int, std::unordered_map<int, bool>> closedList;

    Node* startNode = new Node(startX, startY, 0, heuristic(startX, startY, goalX, goalY));
    openList.push(*startNode);

    Eigen::MatrixXi nodeParentData(0, 4);
    // int i=0;


    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();
        

        // 행렬에 새로운 데이터 삽입 (node와 parent 정보)
        if (current.parent != nullptr) {
            // i++;
            nodeParentData.conservativeResize(nodeParentData.rows() + 1, Eigen::NoChange);
            nodeParentData(nodeParentData.rows() - 1, 0) = current.x;      // node.x
            nodeParentData(nodeParentData.rows() - 1, 1) = current.y;      // node.y
            nodeParentData(nodeParentData.rows() - 1, 2) = current.parent->x;  // parent.x
            nodeParentData(nodeParentData.rows() - 1, 3) = current.parent->y;  // parent.y
            // std::cout << nodeParentData(i,1) <<"\n";
        }


        if (current.x == goalX && current.y == goalY) {
            std::vector<std::pair<int, int>> path;
            path.push_back({goalX, goalY});
            // std::cout << "path rows : " << nodeParentData.rows() << ", path cols : " << nodeParentData.cols() <<"\n";
            for(int i = nodeParentData.rows();i>0;i--)
            {
                path.push_back({nodeParentData(i-1,2), nodeParentData(i-1,3)});
                // std::cout << "path x : " << nodeParentData(i-1,0) << ", y : " << nodeParentData(i-1,1) <<"\n";
            }


            std::reverse(path.begin(), path.end());
            return path;
        }


        int newX = 0;
        int newY = 0;

        for (int i = 0; i < 8; i++) {
            newX = current.x + dx[i];
            newY = current.y + dy[i];

            if (isValid(newX, newY, rows, cols, grid) && !closedList[newX][newY]) {
                int new_g_cost = current.g_cost + 1;
                int new_h_cost = heuristic(newX, newY, goalX, goalY);
                openList.push(Node(newX, newY, new_g_cost, new_h_cost, new Node(current.x, current.y, current.g_cost, current.h_cost)));
                closedList[newX][newY] = true;
            }
        }
        Node topNode = openList.top();
        std::cout << "Top node: (" << topNode.x << ", " << topNode.y << ") with f_cost: " << topNode.f_cost() << std::endl;
        
    }

    return {};
}

int main() {

    int rows, cols;
    std::string filename = "/home/q922/Desktop/simulation_ws/src/astar_b9/maps/map.csv";
    
    Eigen::MatrixXi grid = readCSV(filename, rows, cols);
    std::cout << "rows : " << rows << ", cols : " << cols << "\n";

    int startX = 140;
    int startY = 209;
    int goalX = 116;
    int goalY = 262;

    std::vector<std::pair<int, int>> path = AStar(startX, startY, goalX, goalY, grid, rows, cols);

    if (!path.empty()) {
        std::cout << "Path : \n";
        for (const auto& p : path) {
            std::cout << "[" << p.first << ", " << p.second << "]\n";
        }
    } else {
        std::cout << "No path\n";
    }

    return 0;
}
