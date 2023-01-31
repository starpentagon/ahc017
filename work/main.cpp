#include <map>
#include <iostream>
#include "UnionFind.hpp"
#include "XorShift.hpp"
#include "Graph.hpp"
#include "ConnectionSet.hpp"
#include "SqDistScheduler.hpp"
#include "BypassSetScheduler.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

int CalcOverK(int K, const vector<int>& schedule) {
   map<int, int> day_edge;
   for (auto d : schedule) {
      day_edge[d]++;
   }

   int over_k = 0;

   for (auto [d, cnt] : day_edge) {
      if (cnt > K) over_k++;
   }

   return over_k;
}

int main() {
   int N, M, D, K;
   cin >> N >> M >> D >> K;

   ConnectionSet connector(N, D);
   // SqDistScheduler scheduler(N, D, K);

   rep(i, M) {
      int u, v, w;
      cin >> u >> v >> w;

      connector.AddEdge(u, v, w);
   }

   /*
   rep(i, N) {
      int x, y;
      cin >> x >> y;

      scheduler.SetNodeCoord(i + 1, x, y);
   }
   */

   auto day_avail_edge_bit = connector.CalcAvailEdgeSet();
   BypassSetScheduler scheduler(D, K, connector, day_avail_edge_bit);

   auto schedule = scheduler.MakeSchedule();
   cout << schedule << endl;

#ifdef LOCAL
   auto [sche_cost, sche_discon_cnt] = connector.CalcScheduleCost(schedule);
   cerr << "Cost=" << sche_cost << ' ';
   cerr << "DisconCnt=" << sche_discon_cnt << ' ';
   cerr << "OverK=" << CalcOverK(K, schedule) << ' ';
   cerr << "InBypass=" << scheduler.GetBypassSet().InBypassEdgeCount() << ' ';
   cerr << endl;
#endif
   return 0;
}
