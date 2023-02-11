#include <iostream>
#include "../Graph.hpp"
#include "../debug.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }

template<class T> istream& operator>>(istream& is, vector<T>& vec){ rep(i, vec.size()) is >> vec[i]; return is;}
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

using P = pair<int, int>;
using Edge = tuple<int, int, long long>;  // index, to, weight
using ll = long long;

int main() {
   int N, M, D, K;
   cin >> N >> M >> D >> K;

   vector<vector<Edge>> adj_list(N + 1);
   vector<P> pos_list(N + 1);

   Graph graph(N);

   rep(e, M) {
      int u, v, w;
      cin >> u >> v >> w;

      adj_list[u].emplace_back(e, v, w);
      adj_list[v].emplace_back(e, u, w);

      graph.AddEdge(u, v, w);
   }

   graph.Prep();

   rep(i, N) {
      int x, y;
      cin >> x >> y;

      pos_list[i + 1] = P(x, y);
   }

   vector<ll> single_edge_cost(M);

   rep(e, M) {
      vector<int> del_edge_list;
      del_edge_list.emplace_back(e);

      single_edge_cost[e] = graph.CalcCost(del_edge_list).first;
   }

   ll m = -1;
   rep(e, M) {
      if (chmax(m, single_edge_cost[e])) {
         debug(e, m);
      }
   }

   vector<vector<ll>> pair_edge_cost_diff(M, vector<ll>(M, 0));

   rep(e1, M) {
      cerr << e1 << endl;
      rep(e2, M) {
         if (e1 == e2) continue;

         if (e1 > e2) {
            pair_edge_cost_diff[e1][e2] = pair_edge_cost_diff[e2][e1];
            continue;
         }

         vector<int> del_edge_list;
         del_edge_list.emplace_back(e1);
         del_edge_list.emplace_back(e2);

         ll cost = graph.CalcCost(del_edge_list).first;
         pair_edge_cost_diff[e1][e2] = cost - single_edge_cost[e1] - single_edge_cost[e2];

         cout << e1 << ',' << e2 << ',' << pair_edge_cost_diff[e1][e2] << endl;
      }
   }
   return 0;
}
