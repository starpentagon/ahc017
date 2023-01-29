#include <iostream>
#include <vector>
#include <tuple>
#include <map>

#include "../ConnectionSet.hpp"
#include "../UnionFind.hpp"
#include "../debug.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

using ll = long long;
int N, M, D, K;
vector<Edge> edge_list;

void output(vector<EdgeBit>& day_avail_edge_bit) {
   // 連結性のチェック
   int discon_day = 0;

   rep(d, D) {
      UnionFind uf(N);

      rep(e, M) {
         if (day_avail_edge_bit[d][e] == 0) {
            auto [u, v, w] = edge_list[e];
            uf.Unite(u, v);
         }
      }

      if ((int)uf.size(1) != N) {
         discon_day++;
      }
   }

   // 辺の工事可能な日数
   vector<int> avail_cnt(M, 0);

   rep(d, D) {
      rep(e, M) {
         if (day_avail_edge_bit[d][e] == 1) avail_cnt[e]++;
      }
   }

   map<int, int> cnt_map;

   rep(e, M) {
      int cnt = avail_cnt[e];
      cnt_map[cnt]++;
   }

   // 日ごとの工事可能日数の余裕度
   int average_edge = (M + D - 1) / D;
   double min_schedule_room = D;

   rep(d, D) {
      int avail_edge_cnt = 0;

      rep(e, M) if (day_avail_edge_bit[d][e]) avail_edge_cnt++;

      double schedule_room = 1.0 * avail_edge_cnt / average_edge;
      chmin(min_schedule_room, schedule_room);
   }

   cerr << "DisConDay=" << discon_day << " ";
   cerr << "Avail0=" << cnt_map[0] << " ";
   cerr << "Avail1=" << cnt_map[1] << " ";
   cerr << "Avail2=" << cnt_map[2] << " ";
   cerr << "Avail3=" << cnt_map[3] << " ";
   cerr << "ScheduleRoom=" << min_schedule_room << " ";
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

   output(day_avail_edge_bit);
}
