#include <iostream>
#include "SqDistScheduler.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

int main() {
   int N, M, D, K;
   cin >> N >> M >> D >> K;

   SqDistScheduler scheduler(N, D, K);

   rep(i, M) {
      int u, v, w;
      cin >> u >> v >> w;

      scheduler.AddEdge(u, v, w);
   }

   rep(i, N) {
      int x, y;
      cin >> x >> y;

      scheduler.SetNodeCoord(i + 1, x, y);
   }

   scheduler.MakeSchedule();

   auto schedule = scheduler.GetSchedule();
   cout << schedule << endl;

   cerr << scheduler.CalcScheduleCost() << endl;

   return 0;
}
