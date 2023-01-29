#include <iostream>
#include "UnionFind.hpp"
#include "XorShift.hpp"
#include "Graph.hpp"
#include "ConnectionSet.hpp"
#include "SqDistScheduler.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

int main() {
   int N, M, D, K;
   cin >> N >> M >> D >> K;

   ConnectionSet connector(N, D);
   SqDistScheduler scheduler(N, D, K);

   rep(i, M) {
      int u, v, w;
      cin >> u >> v >> w;

      connector.AddEdge(u, v, w);
      scheduler.AddEdge(u, v, w);
   }

   rep(i, N) {
      int x, y;
      cin >> x >> y;

      scheduler.SetNodeCoord(i + 1, x, y);
   }

   auto day_avail_edge_bit = connector.CalcAvailEdgeSet();
   scheduler.MakeSchedule(day_avail_edge_bit);

   auto schedule = scheduler.GetSchedule();
   cout << schedule << endl;

#ifdef LOCAL
   cerr << "Cost=" << scheduler.CalcScheduleCost() << ' ';
   cerr << "DisconCnt=" << scheduler.CalcScheduleDisconCount() << ' ';
   cerr << endl;
#endif
   return 0;
}
