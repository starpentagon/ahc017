#include <map>
#include <iostream>
#include "UnionFind.hpp"
#include "XorShift.hpp"
#include "Graph.hpp"
#include "ConnectionSet.hpp"
#include "ShortestTree.hpp"
#include "BypassSet.hpp"
#include "BypassSetScheduler.hpp"
#include "FaceGroup.hpp"
#include "FaceGroupSchedulerExp.hpp"
#include "DualGraph.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

int main() {
   int N, M, D, K;
   cin >> N >> M >> D >> K;

   int min_K = (M + D - 1) / D;
   bool tight = (K - min_K) <= 3;

   FaceGroup face_group(N, D);
   ConnectionSet connector(N, D);
   vector<Edge> edge_list;

   rep(i, M) {
      int u, v, w;
      cin >> u >> v >> w;

      face_group.AddEdge(u, v, w);
      edge_list.emplace_back(u, v, w);
      connector.AddEdge(u, v, w);
   }

   vector<Coord> coord_list;

   rep(i, N) {
      int x, y;
      cin >> x >> y;

      coord_list.emplace_back(x, y);
      face_group.SetNodeCoord(i + 1, x, y);
   }

   vector<int> schedule;

   if (tight) {
      auto day_avail_edge_bit = connector.CalcAvailEdgeSet();
      BypassSetScheduler scheduler(D, K, connector, day_avail_edge_bit);

      schedule = scheduler.MakeSchedule();
      cout << schedule << endl;
#ifdef LOCAL
      auto [sche_cost, sche_discon_cnt] = connector.CalcScheduleCost(D, schedule);
      cerr << "Cost=" << sche_cost << ' ';
      cerr << "DisconCnt=" << sche_discon_cnt << ' ';
      cerr << "OverK=" << CalcOverK(K, schedule) << ' ';
      cerr << "InBypass=-1" << ' ';
      cerr << "Iter=-1" << ' ';
      cerr << endl;
#endif
   } else {
      auto face_group_list = face_group.MakeGroup();

      // FaceGroupScheduler scheduler(M, D, K, connector, day_avail_edge_bit, face_group_list);
      FaceGroupSchedulerExp scheduler(M, D, K, face_group, face_group_list);
      schedule = scheduler.MakeSchedule(10000);
      cout << schedule << endl;

#ifdef LOCAL
      auto [sche_cost, sche_discon_cnt] = face_group.CalcScheduleCost(D, schedule);
      // int sche_cost = 0, sche_discon_cnt = 0;
      cerr
          << "Cost=" << sche_cost << ' ';
      cerr << "DisconCnt=" << sche_discon_cnt << ' ';
      cerr << "OverK=" << CalcOverK(K, schedule) << ' ';
      cerr << "InBypass=-1" << ' ';
      cerr << "Iter=" << scheduler.GetIterCount() << ' ';
      cerr << endl;
#endif
   }

   return 0;
}
