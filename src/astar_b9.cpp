#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int32.hpp> 

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

    Eigen::MatrixXi nodeParentData(0, 4);
    // int i=0;


    while (!openList.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("AStar"), "OpenList size: %zu", openList.size());
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
        // std::cout << "Top node: (" << topNode.x << ", " << topNode.y << ") with f_cost: " << topNode.f_cost() << std::endl;
        
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

        path_length_pub_ = this->create_publisher<std_msgs::msg::Int32>("/path_length", 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr goal_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr path_length_pub_;
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
    }

    void goalCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {

        RCLCPP_INFO(this->get_logger(), "GoalCallback called with data size: %zu", msg->data.size());

        if (msg->data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Goal message is empty");
            return;
        }

        // 첫 번째 값: 목표 좌표의 개수
        int goal_count = msg->data[0];

        // 데이터 크기 검증 (goal_count * 2 + 2: 시작 좌표 2개 + 목표 좌표 2개씩)
        if (msg->data.size() != static_cast<size_t>(goal_count * 2 + 3)) {
            RCLCPP_ERROR(this->get_logger(), "Goal message size mismatch. Expected %d goal coordinates.", goal_count);
            return;
        }

        // 시작 좌표
        int startX = msg->data[1];
        int startY = msg->data[2];

        // 목표 지점 좌표 추출
        std::vector<std::pair<int, int>> goals;
        for (int i = 0; i < goal_count; ++i) {
            int goalX = msg->data[3 + i * 2];
            int goalY = msg->data[4 + i * 2];
            goals.emplace_back(goalX, goalY);
        }

        if (grid_.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "Map data not received yet");
            return;
        }

        // 각 목표에 대해 경로 계산 및 최단 경로 선택
        int shortest_index = -1;
        size_t shortest_length = std::numeric_limits<size_t>::max();

        for (size_t i = 0; i < goals.size(); ++i) {
            int goalX = goals[i].first;
            int goalY = goals[i].second;

            std::vector<std::pair<int, int>> path = AStar(startX, startY, goalX, goalY, grid_, rows_, cols_);
            if (!path.empty()) {
                RCLCPP_INFO(this->get_logger(), "Path %zu length: %zu", i + 1, path.size());
                if (path.size() < shortest_length) {
                    shortest_length = path.size();
                    shortest_index = i;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "No path found for goal %zu (%d, %d)", i + 1, goalX, goalY);
            }
        }

        // 최단 경로 인덱스 발행
        if (shortest_index != -1) {
            auto path_length_msg = std_msgs::msg::Int32();
            path_length_msg.data = static_cast<int>(shortest_index);
            path_length_pub_->publish(path_length_msg);
            RCLCPP_INFO(this->get_logger(), "Shortest path is to goal %d with length %zu", shortest_index, shortest_length);
        } else {
            RCLCPP_WARN(this->get_logger(), "No valid paths found for any goals");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarNode>();
    rclcpp::spin(node);  // 여기서 SIGINT(CTRL+C)를 감지합니다.
    rclcpp::shutdown();  // ROS2 시스템을 안전하게 종료합니다.
    return 0;
}
