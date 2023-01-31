#include "UnionFind.hpp"
#include "XorShift.hpp"
#include "BypassSet.hpp"
#include "ConnectionSet.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
// clang-format on

using ll = long long;

ConnectionSet::ConnectionSet(int N, int D)
    : Graph(N), D_(D), mt_(1234), max_temp_(kDefaultMaxTemp), min_temp_(kDefaultMinTemp) {
   day_connection_set_.resize(D);
}

int ConnectionSet::CalcCost(const vector<EdgeBit> &day_connection_set) const {
   int cost = 0;

   // 辺ごとの工事可能な日数に対するコスト
   rep(e, edge_list_.size()) {
      int avail_count = 0;

      rep(d, D_) {
         if (day_connection_set[d][e] == 0) avail_count++;
      }

      if (avail_count == 0) {
         cost += kCostNonAvailable;
      } else if (avail_count == 1) {
         cost += kCostOneAvailable;
      } else if (avail_count == 2) {
         cost += kCostTwoAvailable;
      }
   }

   // 固定する連結成分の大きさに対するコスト
   rep(d, D_) {
      cost += kCostConnect * day_connection_set[d].count();
   }

   return cost;
}

void ConnectionSet::InitDayConnectionSet() {
   int E = (int)edge_list_.size();

   rep(d, D_) {
      // todo: UnionFindをresetして使いまわすと高速化？
      UnionFind uf(N_);
      int cc_edge_cnt = 0;

      while (cc_edge_cnt < N_ - 1) {
         int e = XorShift() % E;

         auto [u, v, w] = edge_list_[e];

         if (uf.IsSameGroup(u, v)) continue;

         uf.Unite(u, v);
         cc_edge_cnt++;
         day_connection_set_[d][e] = 1;
      }
   }
}

Trans ConnectionSet::GenerateTransition() {
   int E = (int)edge_list_.size();
   EdgeBit non_avail_edge_bit;

   rep(e, E) non_avail_edge_bit[e] = 1;

   rep(d, D_) {
      rep(e, E) {
         if (day_connection_set_[d][e] == 0) non_avail_edge_bit[e] = 0;
      }
   }

   auto rnd = XorShift() % 100;

   auto edge_select = [&](int d, const EdgeBit &selectable_edge_bit) -> Trans {
      vector<int> edge_list;

      rep(e, E) {
         if (!selectable_edge_bit[e]) continue;
         edge_list.emplace_back(e);
      }

      int ind = XorShift() % edge_list.size();
      int e = edge_list[ind];

      // 追加する辺集合
      auto add_edge_bit = edge_bypass_[e];
      auto contained_edge_bit = day_connection_set_[d] & add_edge_bit;
      add_edge_bit ^= contained_edge_bit;  // すでに登録されている辺は除く

      return {d, e, add_edge_bit};
   };

   if (non_avail_edge_bit.any() && rnd < kDefaultSelectNotAvailable) {
      int d = XorShift() % D_;
      auto trans = edge_select(d, non_avail_edge_bit);

      return trans;
   } else {
      int d = XorShift() % D_;
      auto trans = edge_select(d, day_connection_set_[d]);

      return trans;
   }
}

vector<EdgeBit> ConnectionSet::CalcAvailEdgeSet() {
   uniform_real_distribution<> uniform_dist(0.0, 1.0);
   static constexpr int kMaxCount = 500;

   // 初期化
   Prep();
   InitDayConnectionSet();

   int cur_cost = CalcCost(day_connection_set_);

   int best_cost = cur_cost;
   auto best_day_connection_set = day_connection_set_;

   rep(i, kMaxCount) {
      const double progress = min(1.0 * i / kMaxCount, 1.0);
      const double temp = max_temp_ + (min_temp_ - max_temp_) * progress;

      // 遷移
      auto trans = GenerateTransition();

      auto trans_day_connection_set = day_connection_set_;

      {
         auto [d, e, add_edge_bit] = trans;
         trans_day_connection_set[d][e] = 0;
         trans_day_connection_set[d] |= add_edge_bit;
      }

      auto trans_cost = CalcCost(trans_day_connection_set);
      auto delta_improve = cur_cost - trans_cost;

      bool search_update = false;

      if (delta_improve >= 0) {
         // 更新できる場合は必ず遷移する
         search_update = true;
      } else {
         // 悪化している場合も一定確率で更新する
         double prob = exp(delta_improve / temp);
         double rnd = uniform_dist(mt_);

         if (prob > rnd) {
            search_update = true;
         }
      }

      if (!search_update) {
         continue;
      }

      // debug(i, cur_cost, delta_improve);

      // 遷移する
      swap(trans_day_connection_set, day_connection_set_);
      cur_cost -= delta_improve;

      if (chmin(best_cost, cur_cost)) {
         best_day_connection_set = day_connection_set_;
      }
   }

   // 工事可能な日がない辺は強制的に工事可能な日を設定する
   vector<EdgeBit> day_avail_edge_bit = best_day_connection_set;
   EdgeBit avail_bit;

   rep(d, D_) {
      day_avail_edge_bit[d].flip();
      avail_bit |= day_avail_edge_bit[d];
   }

   if (!avail_bit.all()) {
      rep(e, edge_list_.size()) {
         if (avail_bit[e] == 1) continue;
         int d = XorShift() % D_;
         day_avail_edge_bit[d][e] = 1;
      }
   }

   return day_avail_edge_bit;
}

int ConnectionSet::DisconnectedDayCount() const {
   int discon_day = 0;
   int E = (int)edge_list_.size();

   rep(d, D_) {
      UnionFind uf(N_);

      rep(e, E) {
         if (day_connection_set_[d][e] == 1) {
            auto [u, v, w] = edge_list_[e];
            uf.Unite(u, v);
         }
      }

      if ((int)uf.size(1) != N_) {
         discon_day++;
      }
   }

   return discon_day;
}

int ConnectionSet::AvailCountEdge(int count) const {
   // 辺の工事可能な日数
   int edge_count = 0;
   int E = (int)edge_list_.size();

   rep(e, E) {
      int avail_count = 0;

      rep(d, D_) {
         if (day_connection_set_[d][e] == 0) avail_count++;
      }

      if (avail_count == count) edge_count++;
   }

   return edge_count;
}

double ConnectionSet::ScheduleRoom() const {
   int E = (int)edge_list_.size();
   int average_edge = (E + D_ - 1) / D_;
   double min_schedule_room = D_;

   rep(d, D_) {
      int avail_edge_cnt = 0;

      rep(e, E) if (day_connection_set_[d][e] == 0) avail_edge_cnt++;

      double schedule_room = 1.0 * avail_edge_cnt / average_edge;
      chmin(min_schedule_room, schedule_room);
   }

   return min_schedule_room;
}

int ConnectionSet::AvailOneEdgeInBypassCount() const {
   int dummy_K = 3000;
   BypassSet bypass_set(D_, dummy_K, *this);

   int E = (int)edge_list_.size();

   rep(e, E) {
      int avail_count = 0;
      int avail_day = -1;

      rep(d, D_) {
         if (day_connection_set_[d][e] == 0) {
            avail_day = d;
            avail_count++;
         }
      }

      if (avail_count == 1) {
         bypass_set.AddEdge(avail_day, e);
      }
   }

   return bypass_set.InBypassEdgeCount();
}
