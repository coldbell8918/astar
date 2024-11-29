#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

const int dx[] = {-1, 1, 0, 0, -1, 1, -1, 1};
const int dy[] = {0, 0, -1, 1, -1, -1, 1, 1}; // 좌, 우, 위, 아래, 대각선2

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

// Manhattan distance
int heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

bool isValid(int x, int y, int rows, int cols, const Eigen::MatrixXi& grid) {
    return x >= 0 && y >= 0 && x < rows && y < cols && grid(x, y) == 0;
}

std::vector<std::pair<int, int>> AStar(int startX, int startY, int goalX, int goalY, const Eigen::MatrixXi& grid, int rows, int cols) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::unordered_map<int, std::unordered_map<int, bool>> closedList;

    Node* startNode = new Node(startX, startY, 0, heuristic(startX, startY, goalX, goalY));
    openList.push(*startNode);

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        if (current.x == goalX && current.y == goalY) {
            std::vector<std::pair<int, int>> path;
            path.push_back({goalX, goalY});
            while (current.parent != nullptr) {
                current = *current.parent;
                path.push_back({current.x, current.y});
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 8; i++) {
            int newX = current.x + dx[i];
            int newY = current.y + dy[i];

            if (isValid(newX, newY, rows, cols, grid) && !closedList[newX][newY]) {
                int new_g_cost = current.g_cost + 1;
                int new_h_cost = heuristic(newX, newY, goalX, goalY);
                openList.push(Node(newX, newY, new_g_cost, new_h_cost, new Node(current.x, current.y, current.g_cost, current.h_cost)));
                closedList[newX][newY] = true;
            }
        }
    }
    return {};
}

class AStarNode : public rclcpp::Node {
public:
    AStarNode() : Node("astar_node") {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AStarNode::mapCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/astar_goal", 10, std::bind(&AStarNode::goalCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr goal_sub_;
    Eigen::MatrixXi grid_;
    int rows_ = 0, cols_ = 0;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        rows_ = msg->info.height;
        cols_ = msg->info.width;
        grid_ = Eigen::MatrixXi(rows_, cols_);

        for (int y = 0; y < rows_; ++y) {
            for (int x = 0; x < cols_; ++x) {
                int value = msg->data[y * cols_ + x];
                // -1(알 수 없는 영역)과 0(이동 가능)을 모두 0으로 처리
                if (value == -1 || value == 0) {
                    grid_(y, x) = 0; // 이동 가능
                } else {
                    grid_(y, x) = 1; // 장애물
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Map received with size: %d x %d", rows_, cols_);
    }

    void goalCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Goal message must contain 4 elements: startX, startY, goalX, goalY");
            return;
        }

        int startX = msg->data[0];
        int startY = msg->data[1];
        int goalX = msg->data[2];
        int goalY = msg->data[3];

        if (grid_.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "Map data not received yet");
            return;
        }

        std::vector<std::pair<int, int>> path = AStar(startX, startY, goalX, goalY, grid_, rows_, cols_);
        if (!path.empty()) {
            RCLCPP_INFO(this->get_logger(), "Path found:");
            std::string path_str = "Path: ";
            for (const auto& p : path) {
                path_str += "[" + std::to_string(p.first) + ", " + std::to_string(p.second) + "] ";
            }
            RCLCPP_INFO(this->get_logger(), path_str.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No path found");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarNode>());
    rclcpp::shutdown();
    return 0;
}
