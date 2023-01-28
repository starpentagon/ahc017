#include <iostream>
#include <queue>

#include "Graph.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
// clang-format on

using ll = long long;

Graph::Graph(int N)
    : N_(N), total_dist_(0) {
   node_sum_dist_.resize(N + 1, 0);
   adj_list_.resize(N + 1);

   node_coord_.resize(N + 1);
}

void Graph::AddEdge(int u, int v, ll w) {
   adj_list_[u].emplace_back(v, w);
   adj_list_[v].emplace_back(u, w);

   edge_list_.emplace_back(u, v, w);
}

// ダイクストラ法で単一始点最短路を求める
// @pre 各エッジの重みが非負であること
// 計算量: O(E + N log N)
// 非連結成分には numeric_limits<long long>::max() が設定される
vector<long long> ShortestPathDijkstra(const vector<vector<pair<Node, ll>>>& adj_list, const int start, const vector<vector<bool>>& del_edge_flg) {
   // 重みリストの初期化
   int L = (int)adj_list.size();
   constexpr long long INF = DIST_INF;
   vector<long long> min_weight_list(L, INF);

   // 重み最小のノードを管理
   using WeightNode = pair<long long, int>;  // (startからの最小重み, ノード番号)
   priority_queue<WeightNode, vector<WeightNode>, greater<WeightNode>> node_queue;

   min_weight_list[start] = 0;
   node_queue.emplace(0, start);

   while (!node_queue.empty()) {
      const auto [min_weight, min_node] = node_queue.top();
      node_queue.pop();

      // すでに更新済みの場合はskip
      // - skipしないとO(N^2)となるケースが存在
      // see: https://snuke.hatenablog.com/entry/2021/02/22/102734
      if (min_weight_list[min_node] < min_weight) {
         continue;
      }

      // 重み最小のノードに隣接するノードを更新できるかチェック
      for (const auto& [node_to, weight] : adj_list[min_node]) {
         if (del_edge_flg[min_node][node_to]) continue;

         if (min_weight_list[node_to] > min_weight + weight) {
            min_weight_list[node_to] = min_weight + weight;
            node_queue.emplace(min_weight_list[node_to], node_to);
         }
      }
   }

   return min_weight_list;
}

void Graph::CalcAllDist() {
   vector<vector<bool>> del_edge_flg(N_ + 1, vector<bool>(N_ + 1, false));

   for (Node s = 1; s <= N_; s++) {
      auto min_dist = ShortestPathDijkstra(adj_list_, s, del_edge_flg);

      ll node_dist = 0;

      for (Node t = 1; t <= N_; t++) {
         node_dist += min_dist[t];
      }

      node_sum_dist_[s] = node_dist;
   }
}

ll Graph::CalcCost(const std::vector<int>& del_edge_list) const {
   vector<vector<bool>> del_edge_flg(N_ + 1, vector<bool>(N_ + 1, false));

   for (auto e : del_edge_list) {
      const auto [u, v, w] = edge_list_[e];

      del_edge_flg[u][v] = true;
      del_edge_flg[v][u] = true;
   }

   ll cost = 0;

   for (Node s = 1; s <= N_; s++) {
      auto min_dist = ShortestPathDijkstra(adj_list_, s, del_edge_flg);

      ll node_dist = 0;

      for (Node t = 1; t <= N_; t++) {
         node_dist += min_dist[t];
      }

      cost += node_dist - node_sum_dist_[s];
   }

   return 1000 * cost / (N_ * (N_ - 1));
}

void Graph::SetNodeCoord(int u, int x, int y) {
   node_coord_[u] = Coord(x, y);
}

long long Graph::CalcNodeSqDist(Node u, Node v) const {
   auto [x_u, y_u] = node_coord_[u];
   auto [x_v, y_v] = node_coord_[v];

   ll sq_dist = (x_u - x_v) * (x_u - x_v);
   sq_dist += (y_u - y_v) * (y_u - y_v);

   return sq_dist;
}

long long Graph::CalcEdgeSqDist(const Edge& edge_1, const Edge& edge_2) const {
   auto [u_1, v_1, w_1] = edge_1;
   auto [u_2, v_2, w_2] = edge_2;

   ll edge_dist = CalcNodeSqDist(u_1, u_2);

   chmin(edge_dist, CalcNodeSqDist(u_1, v_2));
   chmin(edge_dist, CalcNodeSqDist(v_1, u_2));
   chmin(edge_dist, CalcNodeSqDist(v_1, v_2));

   return edge_dist;
}