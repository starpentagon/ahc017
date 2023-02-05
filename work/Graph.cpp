#include <cmath>
#include <algorithm>
#include <cassert>
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
   node_shortest_tree_.resize(N + 1);

   adj_list_.resize(N + 1);

   node_coord_.resize(N + 1);
}

void Graph::AddEdge(int u, int v, ll w) {
   int i = (int)edge_list_.size();
   adj_list_[u].emplace_back(i, v, w);
   adj_list_[v].emplace_back(i, u, w);

   edge_list_.emplace_back(u, v, w);
}

// BFSで単一始点最短路を求める
// 計算量: O(N+E)
vector<long long> ShortestPathBFS(const vector<vector<Adj>>& adj_list, const int start, const EdgeBit& del_edge_flg, const int end = -1) {
   // 重みリストの初期化
   constexpr long long INF = DIST_INF;
   int L = (int)adj_list.size();
   vector<long long> min_weight_list(L, INF);

   min_weight_list[start] = 0;

   // 最短路が求まったノードを管理する
   using WeightNode = pair<long long, int>;  // (startからの最小重み, ノード番号)
   queue<WeightNode> node_queue;
   node_queue.emplace(0, start);

   while (!node_queue.empty()) {
      const auto [min_weight, min_node] = node_queue.front();
      node_queue.pop();

      if (min_weight_list[min_node] < min_weight) continue;

      if (min_node == end) {
         break;
      }

      // 隣接するノードうち未訪問のものを更新する
      for (const auto [edge_index, node_to, weight] : adj_list[min_node]) {
         if (del_edge_flg[edge_index]) continue;

         if (min_weight_list[node_to] > min_weight + weight) {
            min_weight_list[node_to] = min_weight_list[min_node] + weight;
            node_queue.emplace(min_weight_list[node_to], node_to);
         }
      }
   }

   return min_weight_list;
}

// ノード間の最短経路を求める(重み付き用)
// @param start, node: 最短経路を求めるノード
// @param min_weight_list: startから各ノードの最短距離が格納されたテーブル
// @retval node_path: startノードからendノードまでの最短経路順に並べたノードリスト
// @note 計算量: O(E)
// Unverified
EdgeBit FindShortestPath(const int start, const int end, const vector<vector<Adj>>& adj_list, const vector<long long>& min_weight_list, const EdgeBit& del_edge_flg) {
   int node = end;
   EdgeBit path_bit;

   while (node != start) {
      for (auto [edge_index, to, weight] : adj_list[node]) {
         if (del_edge_flg[edge_index]) continue;

         if (min_weight_list[node] >= min_weight_list[to] + weight) {
            node = to;
            path_bit.set(edge_index);
            break;
         }
      }
   }

   return path_bit;
}

void Graph::Prep() {
   EdgeBit del_edge_flg;
   edge_bypass_.resize(edge_list_.size());
   edge_betweenness_.resize(edge_list_.size(), 0);

   for (Node s = 1; s <= N_; s++) {
      auto min_dist = ShortestPathBFS(adj_list_, s, del_edge_flg);

      ll node_dist = 0;

      for (Node t = 1; t <= N_; t++) {
         node_dist += min_dist[t];
      }

      node_sum_dist_[s] = node_dist;

      SetShortestTree(s, s, -1, min_dist);
   }

   // 辺eを削除した場合の迂回路を求める
   rep(e, edge_list_.size()) {
      auto [u, v, w] = edge_list_[e];

      del_edge_flg[e] = 1;

      auto min_dist = ShortestPathBFS(adj_list_, u, del_edge_flg, v);
      edge_bypass_[e] = FindShortestPath(u, v, adj_list_, min_dist, del_edge_flg);

      del_edge_flg[e] = 0;
   }
}

pair<ll, int> Graph::CalcCost(const std::vector<int>& del_edge_list) const {
   EdgeBit del_edge_flg;

   for (auto e : del_edge_list) {
      del_edge_flg.set(e);
   }

   int disconnected_count = 0;

   ll cost = 0;

   for (Node s = 1; s <= N_; s++) {
      EdgeBit shortest_change_bit = del_edge_flg & node_shortest_tree_[s];

      if (shortest_change_bit.none()) continue;

      auto min_dist = ShortestPathBFS(adj_list_, s, del_edge_flg);

      ll node_dist = 0;

      for (Node t = 1; t <= N_; t++) {
         node_dist += min_dist[t];

         if (min_dist[t] == DIST_INF) {
            // debug
            /*
            cerr << "Node:" << s << ' ' << t << endl;
            */
            //--debug
            disconnected_count++;
         }
      }

      cost += node_dist - node_sum_dist_[s];
   }

   return {1000LL * cost / (N_ * (N_ - 1)), disconnected_count};
}

pair<long long, int> Graph::CalcScheduleCost(int D, const std::vector<int>& schedule) const {
   ll cost = 0;
   int discon_cnt = 0;

   for (int d = 1; d <= D; d++) {
      vector<int> edge_list;

      rep(e, edge_list_.size()) {
         if (schedule[e] == d) {
            edge_list.emplace_back(e);
         }
      }

      auto [day_cost, day_discon] = CalcCost(edge_list);

      // debug
      /*
      if (day_discon > 0) {
         cerr << d << endl;
         for (auto e : edge_list) {
            cerr << e << ' ';
         }
         cerr << endl;
         exit(0);
      }
      */
      //--debug

      cost += day_cost;
      discon_cnt += day_discon;
   }

   cost = (ll)round(1.0 * cost / D);

   return {cost, discon_cnt};
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

int Graph::SetShortestTree(Node start, Node node, Node p, const std::vector<long long>& min_dist) {
   int total_child_cnt = 0;

   for (auto [edge_index, n_node, w] : adj_list_[node]) {
      if (n_node == p) continue;

      if (min_dist[n_node] != min_dist[node] + w) continue;
      node_shortest_tree_[start].set(edge_index);

      int child_cnt = SetShortestTree(start, n_node, node, min_dist);
      total_child_cnt += child_cnt;
   }

   total_child_cnt++;  // 自分を含める

   if (p != -1) {
      auto e = GetEdgeIndex(node, p);

      // 自身と親の辺はtotal_child_cnt回通る
      edge_betweenness_[e] += total_child_cnt;
   }

   return total_child_cnt;
}
const int Graph::GetEdgeIndex(Node u, Node v) const {
   for (auto [edge_index, to, w] : adj_list_[u]) {
      if (to == v) return edge_index;
   }

   assert(false);
   return -1;
}
