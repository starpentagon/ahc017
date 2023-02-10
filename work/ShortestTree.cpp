#include "ShortestTree.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }

template<class T> istream& operator>>(istream& is, vector<T>& vec){ rep(i, vec.size()) is >> vec[i]; return is;}
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

// ダイクストラ法で単一始点最短路を求める
// @pre 各エッジの重みが非負であること
// 計算量: O(E + N log N)
// 非連結成分には numeric_limits<long long>::max() が設定される
vector<long long> ShortestPathDijkstra(const vector<vector<Adj>>& adj_list, const int start) {
   // 重みリストの初期化
   int L = (int)adj_list.size();
   constexpr long long INF = numeric_limits<long long>::max();
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
      for (const auto& [edge_index, node_to, weight] : adj_list[min_node]) {
         if (min_weight_list[node_to] > min_weight + weight) {
            min_weight_list[node_to] = min_weight + weight;
            node_queue.emplace(min_weight_list[node_to], node_to);
         }
      }
   }

   return min_weight_list;
}

ShortestTree::ShortestTree(int N, int node)
    : Graph(N), node_(node) {
   auto min_dist = ShortestPathDijkstra(adj_list_, node);

   auto dfs = [&](auto dfs, int node, int p) -> void {
      if (p != -1) {
         min_dist_tree_[node] = NodeInfo(min_dist[node], p);
      }

      for (auto [e, to, w] : adj_list_[node]) {
         if (to == p) continue;

         dfs(dfs, to, node);
      }
   };

   dfs(dfs, node, -1);
}

void ShortestTree::UpdateMinDistTree(int e) {
   auto [u, v, w] = edge_list_[e];

   // 重みリストの初期化
   int L = (int)adj_list.size();
   constexpr long long INF = numeric_limits<long long>::max();

   // 重み最小のノードを管理
   using WeightNode = pair<long long, int>;  // (startからの最小重み, ノード番号)
   priority_queue<WeightNode, vector<WeightNode>, greater<WeightNode>> node_queue;

   if (min_dist_tree_[u].second != -1)
      node_queue.emplace(min_dist_tree_[u], u);

   if (min_dist_tree_[v].second != -1)
      node_queue.emplace(min_dist_tree_[v], v);

   while (!node_queue.empty()) {
      const auto [min_weight, min_node] = node_queue.top();
      node_queue.pop();

      // すでに更新済みの場合はskip
      // - skipしないとO(N^2)となるケースが存在
      // see: https://snuke.hatenablog.com/entry/2021/02/22/102734
      if (min_dist_tree_[min_node] < min_weight) {
         continue;
      }

      // 重み最小のノードに隣接するノードを更新できるかチェック
      for (const auto& [edge_index, node_to, weight] : adj_list[min_node]) {
         if (del_edge_[edge_index]) continue;

         if (min_dist_tree_[node_to] > min_weight + weight) {
            min_dist_tree_[node_to] = min_weight + weight;
            node_queue.emplace(min_dist_tree_[node_to], node_to);
         }
      }
   }
}

void ShortestTree::AddEdge(int e) {
   if (!del_edge_[e]) return;

   del_edge_[e] = 0;

   auto [u, v, w] = edge_list_[e];

   auto du = min_dist_tree_[u].first;
   auto dv = min_dist_tree_[v].first;

   if (chmin(min_dist_tree_[u], dv + w)) {
      min_dist_tree_[u].second = v;
   }

   if (chmin(min_dist_tree_[v], du + w)) {
      min_dist_tree_[v].second = u;
   }

   UpdateMinDistTree(e);
}

void ShortestTree::DelEdge(int e) {
   if (del_edge_[e]) return;

   del_edge_[e] = 1;

   auto [u, v, w] = edge_list_[e];
   int parent = u, child = v;

   if (min_dist_tree_[u].second == v) {
      parent = v;
      child = u;
   }

   // 子の最短路木をクリアする
   auto dfs = [&](auto dfs, int node, int p) -> void {
      min_dist_tree_[node] = NodeInfo(DIST_INF, -1);

      for (auto [e, to, w] : adj_list_[node]) {
         if (to == p) continue;
         if (del_edge_[e]) continue;

         dfs(dfs, to, node);
      }
   };

   dfs(dfs, child, parent);
   UpdateMinDistTree(e);
}

long long ShortestTree::CalcTotalDist() const {
   long long dist = 0;

   for (int i = 1; i <= N; i++) {
      dist += dist_[i];
   }

   return dist;
}
