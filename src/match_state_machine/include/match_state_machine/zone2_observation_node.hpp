#ifndef ZONE2_OBSERVATION_NODE_HPP_
#define ZONE2_OBSERVATION_NODE_HPP_

#include <cmath>
#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <optional>
#include <queue>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct KFSInfo {
    int id = -1;
    int node = -1;

    double x = 0.0;
    double y = 0.0;
    double h = 0.0;

    double rel_x = 0.0;
    double rel_y = 0.0;

    rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};

    bool done = false;
    std::string type = "UNKNOWN";
    std::string slot = "";
};

class ForestGraphPlanner {
public:
    struct Node {
        int id;
        double x;
        double y;
        double h;
        std::vector<int> neighbors;
    };

    // 每一个 Step 表示一条边：from_id -> to_id
    struct Step {
        int from_id = -1;
        int to_id = -1;

        double target_x = 0.0;
        double target_y = 0.0;

        // 机器人车头应该保持的方向
        double yaw = 0.0;

        // from 到 to 的距离
        double length = 0.0;

        double from_h = 0.0;
        double to_h = 0.0;
        double dh = 0.0;

        bool lift_up = false;
        bool lift_down = false;
        bool backward = false;
    };

    struct CostContext {
        std::set<int> blocked_nodes;
        std::set<int> fake_nodes;
        std::set<int> r1_nodes;
        std::set<int> r2_nodes;
        std::set<int> visited_nodes;

        double base_move_cost = 1.0;

        double up_step_extra_cost = 0.8;
        double down_step_extra_cost = 0.8;
        double height_cost_weight = 1.0;

        double fake_cost = 10000.0;
        double blocked_cost = 10000.0;

        // R1 KFS：不一定禁止通行，但是需要人工清理/等待
        double r1_wait_cost = 4.0;

        // R2 KFS：收益，负数，但最终 edgeCost 会保证为正
        double r2_reward = -2.0;

        double visited_cost = 0.3;
    };

    ForestGraphPlanner() {
        initGraph();
    }

    const Node* getNode(int id) const {
        auto it = nodes_.find(id);
        return it == nodes_.end() ? nullptr : &it->second;
    }

    bool areAdjacent(int a, int b) const {
        auto* n = getNode(a);
        if (!n) return false;
        return std::find(n->neighbors.begin(), n->neighbors.end(), b) != n->neighbors.end();
    }

    double edgeCost(int from_id, int to_id, const CostContext& ctx) const {
        const auto* from = getNode(from_id);
        const auto* to = getNode(to_id);

        if (!from || !to) {
            return std::numeric_limits<double>::infinity();
        }

        if (ctx.blocked_nodes.count(to_id)) {
            return ctx.blocked_cost;
        }

        if (ctx.fake_nodes.count(to_id)) {
            return ctx.fake_cost;
        }

        double cost = ctx.base_move_cost;

        const double dist = std::hypot(to->x - from->x, to->y - from->y);
        cost += dist;

        const double dh = to->h - from->h;
        cost += std::abs(dh) * ctx.height_cost_weight;

        if (dh > 0.15) {
            // 低 -> 高：正着上台阶，额外代价
            cost += ctx.up_step_extra_cost;
        } else if (dh < -0.15) {
            // 高 -> 低：倒着下台阶，额外代价
            cost += ctx.down_step_extra_cost;
        }

        if (ctx.r1_nodes.count(to_id)) {
            cost += ctx.r1_wait_cost;
        }

        if (ctx.r2_nodes.count(to_id)) {
            cost += ctx.r2_reward;
        }

        if (ctx.visited_nodes.count(to_id)) {
            cost += ctx.visited_cost;
        }

        // Dijkstra 不允许负边权，保证最小正代价
        return std::max(0.001, cost);
    }

    std::optional<std::vector<int>> dijkstra(
        int start,
        int goal,
        const CostContext& ctx) const
    {
        if (!nodes_.count(start) || !nodes_.count(goal)) {
            return {};
        }

        if (ctx.blocked_nodes.count(goal) || ctx.fake_nodes.count(goal)) {
            return {};
        }

        struct QNode {
            int id;
            double cost;

            bool operator>(const QNode& other) const {
                return cost > other.cost;
            }
        };

        std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> pq;
        std::unordered_map<int, double> dist;
        std::unordered_map<int, int> parent;

        for (const auto& kv : nodes_) {
            dist[kv.first] = std::numeric_limits<double>::infinity();
        }

        dist[start] = 0.0;
        parent[start] = std::numeric_limits<int>::min();
        pq.push({start, 0.0});

        while (!pq.empty()) {
            QNode cur = pq.top();
            pq.pop();

            if (cur.cost > dist[cur.id]) {
                continue;
            }

            if (cur.id == goal) {
                break;
            }

            const auto& node = nodes_.at(cur.id);

            for (int nb : node.neighbors) {
                if (!nodes_.count(nb)) {
                    continue;
                }

                const double ec = edgeCost(cur.id, nb, ctx);

                if (!std::isfinite(ec) || ec >= ctx.blocked_cost) {
                    continue;
                }

                const double nd = cur.cost + ec;

                if (nd < dist[nb]) {
                    dist[nb] = nd;
                    parent[nb] = cur.id;
                    pq.push({nb, nd});
                }
            }
        }

        if (!parent.count(goal)) {
            return {};
        }

        std::vector<int> path;
        for (int c = goal; c != std::numeric_limits<int>::min(); c = parent[c]) {
            path.push_back(c);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    double pathCost(
        const std::vector<int>& path,
        const CostContext& ctx) const
    {
        if (path.size() < 2) {
            return 0.0;
        }

        double total = 0.0;

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            total += edgeCost(path[i], path[i + 1], ctx);
        }

        return total;
    }

    std::vector<Step> pathToSteps(const std::vector<int>& path) const {
        std::vector<Step> steps;

        if (path.size() < 2) {
            return steps;
        }

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const auto& from = nodes_.at(path[i]);
            const auto& to = nodes_.at(path[i + 1]);

            Step s;
            s.from_id = from.id;
            s.to_id = to.id;

            s.target_x = to.x;
            s.target_y = to.y;

            const double dx = to.x - from.x;
            const double dy = to.y - from.y;

            s.from_h = from.h;
            s.to_h = to.h;
            s.dh = to.h - from.h;

            const double move_yaw = std::atan2(dy, dx);
            s.length = std::hypot(dx, dy);

            if (s.dh > 0.15) {
                // 低 -> 高：正着上
                s.lift_up = true;
                s.lift_down = false;
                s.backward = false;
                s.yaw = move_yaw;
            } else if (s.dh < -0.15) {
                // 高 -> 低：倒着下
                s.lift_up = false;
                s.lift_down = true;
                s.backward = true;
                s.yaw = std::remainder(move_yaw + M_PI, 2.0 * M_PI);
            } else {
                // 同高度：正着走
                s.lift_up = false;
                s.lift_down = false;
                s.backward = false;
                s.yaw = move_yaw;
            }

            steps.push_back(s);
        }

        return steps;
    }

private:
    std::unordered_map<int, Node> nodes_;

    double nodeHeight(int id) const {
            switch (id) {
                // 16号启动区、17号一区最后点、0号新观察点
                case 16:
                case 17:
                case 0:
                    return 0.00;

                // 根据最新确认的 zone2_observation.yaml 高度配置
                case 1:
                    return 0.40;
                case 2:
                    return 0.20;
                case 3:
                    return 0.40;

                case 4:
                    return 0.60;
                case 5:
                    return 0.40;
                case 6:
                    return 0.20;

                case 7:
                    return 0.40;
                case 8:
                    return 0.60;
                case 9:
                    return 0.40;

                case 10:
                    return 0.20;
                case 11:
                    return 0.40;
                case 12:
                    return 0.20;

                // 下台阶后的落点
                case 13:
                case 14:
                case 15:
                    return 0.00;

                default:
                    return 0.0;
            }
        }

    void add(int id, double x, double y, std::vector<int> ns, double h) {
        nodes_[id] = Node{id, x, y, h, ns};
    }

    void initGraph() {
            // ============================================================
            // Zone2 实际区块中心点坐标
            //
            // 区块布局：
            //
            //   3    2    1
            //   6    5    4
            //   9    8    7
            //   12   11   10
            //   15   14   13
            //
            // 实际坐标：
            //   1:  (3.360, -0.330)
            //   2:  (3.360, -1.530)
            //   3:  (3.360, -2.730)
            //   4:  (4.560, -0.330)
            //   5:  (4.560, -1.530)
            //   6:  (4.560, -2.730)
            //   7:  (5.760, -0.330)
            //   8:  (5.760, -1.530)
            //   9:  (5.760, -2.730)
            //   10: (6.960, -0.330)
            //   11: (6.960, -1.530)
            //   12: (6.960, -2.730)
            //   13: (8.260, -0.100)
            //   14: (8.260, -1.100)
            //   15: (8.260, -2.300)
            //
            // 注意：
            // - 0 号是启动后先到达的真实过渡点。
            // - 13 / 15 是下台阶出口点。
            // - 14 作为真实中心点保留在图中，但默认不连接，避免误走 11 -> 14。
            // ============================================================

            constexpr double node16_x = 0.000;
                constexpr double node16_y = 0.000;

                constexpr double node17_x = 0.840;
                constexpr double node17_y = -1.540;

                constexpr double node0_x = 2.100;
                constexpr double node0_y = -1.540;

            constexpr double node1_x = 3.360;
            constexpr double node1_y = -0.330;
            constexpr double node2_x = 3.360;
            constexpr double node2_y = -1.530;
            constexpr double node3_x = 3.360;
            constexpr double node3_y = -2.730;

            constexpr double node4_x = 4.560;
            constexpr double node4_y = -0.330;
            constexpr double node5_x = 4.560;
            constexpr double node5_y = -1.530;
            constexpr double node6_x = 4.560;
            constexpr double node6_y = -2.730;

            constexpr double node7_x = 5.760;
            constexpr double node7_y = -0.330;
            constexpr double node8_x = 5.760;
            constexpr double node8_y = -1.530;
            constexpr double node9_x = 5.760;
            constexpr double node9_y = -2.730;

            constexpr double node10_x = 6.960;
            constexpr double node10_y = -0.330;
            constexpr double node11_x = 6.960;
            constexpr double node11_y = -1.530;
            constexpr double node12_x = 6.960;
            constexpr double node12_y = -2.730;

            constexpr double node13_x = 8.260;
            constexpr double node13_y = -0.100;
            constexpr double node14_x = 8.260;
            constexpr double node14_y = -1.100;
            constexpr double node15_x = 8.260;
            constexpr double node15_y = -2.300;

            // 16号点：启动区
                add(16, node16_x, node16_y, {17}, nodeHeight(16));

                // 17号点：一区最后点，也就是原来的0号过渡点
                add(17, node17_x, node17_y, {16, 0}, nodeHeight(17));

                // 0号点：新观察点，用于观察第一排 3/2/1
                add(0, node0_x, node0_y, {17, 2}, nodeHeight(0));

            // 第一列 x = 3.360
            add(1, node1_x, node1_y, {2, 4}, nodeHeight(1));
            add(2, node2_x, node2_y, {0, 1, 3, 5}, nodeHeight(2));
            add(3, node3_x, node3_y, {2, 6}, nodeHeight(3));

            // 第二列 x = 4.560
            add(4, node4_x, node4_y, {1, 5, 7}, nodeHeight(4));
            add(5, node5_x, node5_y, {2, 4, 6, 8}, nodeHeight(5));
            add(6, node6_x, node6_y, {3, 5, 9}, nodeHeight(6));

            // 第三列 x = 5.760
            add(7, node7_x, node7_y, {4, 8, 10}, nodeHeight(7));
            add(8, node8_x, node8_y, {5, 7, 9, 11}, nodeHeight(8));
            add(9, node9_x, node9_y, {6, 8, 12}, nodeHeight(9));

            // 第四列 x = 6.960
            add(10, node10_x, node10_y, {7, 11, 13}, nodeHeight(10));
            add(11, node11_x, node11_y, {8, 10, 12}, nodeHeight(11));
            add(12, node12_x, node12_y, {9, 11, 15}, nodeHeight(12));

            // 第五列 / 下台阶出口点 x = 8.260
            add(13, node13_x, node13_y, {10}, nodeHeight(13));

            // 14号点保留真实坐标，但默认不连接。
            // 如果后面确认 11 可以下到 14，再改为：
            // add(14, node14_x, node14_y, {11}, nodeHeight(14));
            // 同时把 11 的邻居改成 {8, 10, 12, 14}
            add(14, node14_x, node14_y, {}, nodeHeight(14));

            add(15, node15_x, node15_y, {12}, nodeHeight(15));
        }
};

#endif  // ZONE2_OBSERVATION_NODE_