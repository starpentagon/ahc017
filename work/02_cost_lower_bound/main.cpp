#include <iostream>
#include <vector>
#include <tuple>
#include <set>
#include <queue>
#include <algorithm>

#include "../debug.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

using ll = long long;
static constexpr long long DIST_INF = 1000000000;
using Node = int;

vector<long long> ShortestPathBFS(const vector<vector<pair<Node, ll>>>& adj_list, const int start, pair<int, int> e) {
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

      // 隣接するノードうち未訪問のものを更新する
      for (const auto [node_to, weight] : adj_list[min_node]) {
         if (min_node == e.first && node_to == e.second) continue;
         if (min_node == e.second && node_to == e.first) continue;

         if (min_weight_list[node_to] > min_weight + weight) {
            min_weight_list[node_to] = min_weight_list[min_node] + weight;
            node_queue.emplace(min_weight_list[node_to], node_to);
         }
      }
   }

   return min_weight_list;
}

int main() {
   int N, M, D, K;
   cin >> N >> M >> D >> K;

   // N: nodes, M: edges
   using Edge = pair<int, long long>;  // to, weight
   vector<vector<Edge>> adj_list(N + 1, vector<Edge>());

   using EdgeInfo = tuple<int, int, long long>;
   vector<EdgeInfo> edge_list;

   for (int i = 0; i < M; i++) {
      int u, v;
      cin >> u >> v;

      long long w;
      cin >> w;

      adj_list[u].emplace_back(v, w);
      adj_list[v].emplace_back(u, w);

      edge_list.emplace_back(u, v, w);
   }

   vector<set<pair<int, int>>> node_shortest_path(N + 1);
   vector<long long> node_sum_dist(N + 1, 0);

   auto dfs = [&](auto dfs, int root, int node, int p, const vector<long long>& min_dist) -> void {
      if (node_shortest_path[root].count({p, node})) return;

      node_shortest_path[root].insert({p, node});
      node_shortest_path[root].insert({node, p});

      for (auto [n_node, w] : adj_list[node]) {
         if (n_node == p) continue;

         if (min_dist[n_node] != min_dist[node] + w) continue;

         dfs(dfs, root, n_node, node, min_dist);
      }
   };

   for (int s = 1; s <= N; s++) {
      auto min_dist = ShortestPathBFS(adj_list, s, {-1, -1});

      for (int t = 1; t <= N; t++) {
         if (s == t) continue;

         node_sum_dist[s] += min_dist[t];
      }
      dfs(dfs, s, s, -1, min_dist);
   }

   long long cost = 0;

   for (auto [u, v, w] : edge_list) {
      for (int s = 1; s <= N; s++) {
         bool changed = false;

         if (node_shortest_path[s].count({u, v})) {
            changed = true;
         }

         if (!changed) continue;

         auto min_dist = ShortestPathBFS(adj_list, s, {u, v});
         long long changed_dist = 0;

         for (int t = 1; t <= N; t++) {
            changed_dist += min_dist[t];
         }

         cost += changed_dist - node_sum_dist[s];
      }
   }

   cost *= 1000;
   cost /= N * (N - 1);
   cost /= D;
   cerr << cost << endl;

   return 0;
}
