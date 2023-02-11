#include <map>
#include <iostream>
#include "UnionFind.hpp"
#include "XorShift.hpp"
#include "Graph.hpp"
#include "ShortestTree.hpp"
#include "FaceGroup.hpp"
#include "FaceGroupSchedulerExp.hpp"
#include "DualGraph.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

int main() {
   cin.tie(nullptr);
   ios::sync_with_stdio(false);

   int N, M, D, K;
   cin >> N >> M >> D >> K;

   FaceGroup face_group(N, D);

   rep(i, M) {
      int u, v, w;
      cin >> u >> v >> w;

      face_group.AddEdge(u, v, w);
   }

   rep(i, N) {
      int x, y;
      cin >> x >> y;

      face_group.SetNodeCoord(i + 1, x, y);
   }

   vector<int> schedule;

   auto face_group_list = face_group.MakeGroup();

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

   return 0;
}
