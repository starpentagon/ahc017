#include <iostream>
#include <vector>
#include <tuple>
#include <map>

#include "../ConnectionSet.hpp"
#include "../UnionFind.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

using ll = long long;
int N, M, D, K;
vector<Edge> edge_list;

void output(const ConnectionSet& connector) {
   cerr << "DisConDay=" << connector.DisconnectedDayCount() << " ";
   cerr << "Avail0=" << connector.AvailCountEdge(0) << " ";
   cerr << "Avail1=" << connector.AvailCountEdge(1) << " ";
   cerr << "Avail2=" << connector.AvailCountEdge(2) << " ";
   cerr << "Avail3=" << connector.AvailCountEdge(3) << " ";
   cerr << "ScheduleRoom=" << connector.ScheduleRoom() << " ";
   cerr << "Avail1InBypass=" << connector.AvailOneEdgeInBypassCount() << " ";
   cerr << endl;
}

int main() {
   cin >> N >> M >> D >> K;

   ConnectionSet connector(N, D);

   rep(i, M) {
      int u, v, w;
      cin >> u >> v >> w;

      connector.AddEdge(u, v, w);
      edge_list.emplace_back(u, v, w);
   }

   rep(i, N) {
      int x, y;
      cin >> x >> y;

      connector.SetNodeCoord(i + 1, x, y);
   }

   auto day_avail_edge_bit = connector.CalcAvailEdgeSet();

   /*

   rep(d, D) {
      cerr << d << ": ";
      rep(e, M) if (day_avail_edge_bit[d][e]) cerr << e << ' ';
      cerr << endl;
   }
   */

   output(connector);
}
