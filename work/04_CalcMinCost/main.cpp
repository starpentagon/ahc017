#include <iostream>
#include <vector>
#include <tuple>
#include <set>
#include <queue>
#include <algorithm>

#include "../debug.hpp"
#include "../Graph.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

using ll = long long;
using Node = int;

static constexpr int N = 4;
static constexpr int M = 5;

vector<int> edge_schedule(M);
vector<int> best_edge_schedule(M);
ll best_cost = numeric_limits<ll>::max();

ll CalcMinCost(Graph& graph, int D, int e) {
   if (e == M) {
      ll cost = 0;

      rep(d, D) {
         vector<int> edge_index_list;

         rep(i, M) {
            if (edge_schedule[i] == d) {
               edge_index_list.emplace_back(i);
            }
         }

         cost += graph.CalcCost(edge_index_list).first;
      }

      debug(cost, edge_schedule);

      if (chmin(best_cost, cost)) {
         best_edge_schedule = edge_schedule;
      }

      return 0;
   }

   rep(d, D) {
      edge_schedule[e] = d;
      CalcMinCost(graph, D, e + 1);
   }

   return 0;
}

int main() {
   // N: nodes, M: edges
   using EdgeInfo = tuple<int, int, long long>;
   vector<EdgeInfo> edge_list{{1, 2, 1}, {1, 3, 4}, {1, 4, 3}, {2, 4, 2}, {3, 4, 5}};

   Graph graph(N);

   for (auto [u, v, w] : edge_list) {
      graph.AddEdge(u, v, w);
   }

   graph.Prep();

   CalcMinCost(graph, 3, 0);

   cout << "Best cost: " << best_cost << endl;
   cout << "Best Schedule: " << best_edge_schedule << endl;

   return 0;
}
