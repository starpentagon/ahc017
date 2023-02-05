#include <queue>
#include <map>
#include <algorithm>
#include "CycleSplit.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }

template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

CycleSplit::CycleSplit(int N, int D)
    : Graph(N) {
}

std::vector<std::vector<int>> CycleSplit::CalcSplit() {
   Prep();

   int E = edge_list_.size();

   // 最短路木でeを通った距離の総和
   std::vector<long long> edge_dist_sum_(E, 0);

   for (int n = 1; n <= N_; n++) {
      auto shortest_tree = node_shortest_tree_[n];

      rep(e, E) {
         if (!shortest_tree[e]) continue;
         auto [u, v, w] = edge_list_[e];
         edge_dist_sum_[e] += w;
      }
   }

   using WeightIndex = pair<long long, int>;
   vector<WeightIndex> wl;
   wl.reserve(E);

   rep(e, E) {
      wl.emplace_back(edge_dist_sum_[e], e);
   }

   sort(wl.rbegin(), wl.rend());
}

vector<vector<int>> CycleSplit::FindLoop(const int e, const int L) const {
   auto [u, v, w] = edge_list_[e];

   vector<vector<int>> loop_list;
   using Info = tuple<int, int, int>;  // edge index, parent edge, dist

   queue<Info> que;
   map<int, Info> info_map;
   EdgeBit visited;

   que.emplace(e, -1, 1);
   info_map[e] = Info(e, -1, 1);

   auto get_loop_index = [&](int e) {
      vector<int> loop;
      while (get<1>(info_map[e]) != -1) {
         loop.emplace_back(e);
         e = get<1>(info_map[e]);
      }

      reverse(loop.begin(), loop.end());
      return loop;
   };

   while (!que.empty()) {
      auto [edge, p_edge, dist] = que.front();
      que.pop();
      visited[edge] = 1;

      auto [e_u, e_v, w] = edge_list_[edge];

      if (e_v == u) {
         auto loop = get_loop_index(edge);
         loop_list.emplace_back(loop);
         continue;
      }

      if (dist >= L) continue;

      for (auto [n_index, to, weight] : adj_list_[e_v]) {
         if (visited[n_index]) continue;
         que.emplace(n_index, edge, dist + 1);
         info_map[e] = Info(n_index, edge, dist + 1);
      }
   }
}
