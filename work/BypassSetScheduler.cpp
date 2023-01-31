#include <cassert>
#include "XorShift.hpp"
#include "BypassSetScheduler.hpp"

// #include "debug.hpp"

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
// clang-format on

using namespace std;

BypassSetScheduler::BypassSetScheduler(int D, int K, const Graph &graph, const vector<EdgeBit> &day_avail_edge_bit)
    : graph_(graph), D_(D), K_(K), mt_(1234), max_temp_(kBySetSA_DefaultMaxTemp), min_temp_(kBySetSA_DefaultMinTemp), day_avail_edge_bit_(day_avail_edge_bit), bypass_set_(D, K, graph) {
}

void BypassSetScheduler::Initialize() {
   int E = graph_.GetEdgeList().size();
   vector<int> avail_cnt(E, 0);

   rep(d, D_) {
      rep(e, E) {
         if (day_avail_edge_bit_[d][e]) avail_cnt[e]++;
      }
   }

   rep(e, E) {
      if (avail_cnt[e] == 1) avail_one_edge_[e] = 1;
   }

   // 初期集合を作る
   rep(e, E) {
      vector<int> day_list;

      rep(d, D_) {
         if (day_avail_edge_bit_[d][e]) day_list.emplace_back(d);
      }

      int ind = 0;

      if (day_list.size() > 1) {
         ind = XorShift() % day_list.size();
      }

      assert(!day_list.empty());
      int d = day_list[ind];
      bypass_set_.AddEdge(d, e);
   }
}

vector<int> BypassSetScheduler::MakeSchedule() {
   uniform_real_distribution<> uniform_dist(0.0, 1.0);
   static constexpr int kMaxCount = 100 * 1000;

   Initialize();

   int cur_cost = CalcCost();
   int best_cost = cur_cost;
   BypassSet best_bypass_set = bypass_set_;

   rep(i, kMaxCount) {
      if (best_cost == 0) {
         break;
      }

      const double progress = min(1.0 * i / kMaxCount, 1.0);
      const double temp = max_temp_ + (min_temp_ - max_temp_) * progress;

      // 遷移
      auto [trans_e, from_d, to_d] = GenerateTransition();

      if (from_d == to_d) {
         continue;
      }

      if (trans_e == -1) {
         // 迂回路集合を変えることができない
         break;
      }

      bypass_set_.DelEdge(from_d, trans_e);
      bypass_set_.AddEdge(to_d, trans_e);

      // debug(trans_e, from_d, to_d);
      auto trans_cost = CalcCost();

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
         // 元に戻す
         bypass_set_.DelEdge(to_d, trans_e);
         bypass_set_.AddEdge(from_d, trans_e);
         continue;
      }

      cur_cost -= delta_improve;

      if (chmin(best_cost, cur_cost)) {
         best_bypass_set = bypass_set_;
         // debug(i, best_cost, cur_cost, delta_improve, best_bypass_set.InBypassEdgeCount());
      }
   }

   int E = graph_.GetEdgeList().size();
   vector<int> schedule(E, -1);

   rep(e, E) {
      schedule[e] = best_bypass_set.GetDay(e) + 1;
   }

   bypass_set_ = best_bypass_set;

   // debug
   InBypassTest();
   // -- debug

   return schedule;
}

int BypassSetScheduler::CalcCost() const {
   int cost = 0;

   cost += kBySetSA_EdgeInBypass * bypass_set_.InBypassEdgeCount();

   rep(d, D_) {
      if (bypass_set_.GetDayEdgeCount(d) > K_) cost += (bypass_set_.GetDayEdgeCount(d) - K_) * kBySetSA_OverK;
   }

   return cost;
}

BySetSA_Trans BypassSetScheduler::GenerateTransition() {
   // 工事辺数が超過している場合は解消する遷移を生成する
   rep(d, D_) {
      if (bypass_set_.GetDayEdgeCount(d) > K_) {
         int E = graph_.GetEdgeList().size();

         vector<int> avail_day_list;

         rep(next_d, D_) {
            if (bypass_set_.GetDayEdgeCount(next_d) >= K_) continue;
            avail_day_list.emplace_back(next_d);
         }

         int ind = 0;
         if (avail_day_list.size() > 1) ind = XorShift() % avail_day_list.size();
         int next_d = avail_day_list[ind];

         rep(e, E) {
            if (bypass_set_.GetDay(e) == d && day_avail_edge_bit_[next_d][e]) {
               return {e, d, next_d};
            }
         }
      }
   }

   int rnd = XorShift() % 100;
   int e = -1;
   if (rnd < kBySetSA_DefaultSelectInBypass) {
      e = bypass_set_.SelectInBypassEdge(avail_one_edge_);
   } else {
      e = bypass_set_.SelectBypassGeneratorEdge(avail_one_edge_);
   }

   if (e == -1) {
      return {-1, -1, -1};
   }

   int cur_d = bypass_set_.GetDay(e);

   vector<int> avail_day_list;

   rep(d, D_) {
      if (d == cur_d) continue;
      if (!day_avail_edge_bit_[d][e]) continue;
      if (bypass_set_.GetDayEdgeCount(d) >= K_) continue;

      avail_day_list.emplace_back(d);
   }

   if (avail_day_list.empty()) {
      return {e, cur_d, cur_d};
   }

   int ind = 0;
   if (avail_day_list.size() > 1) ind = XorShift() % avail_day_list.size();
   int next_d = avail_day_list[ind];

   return {e, cur_d, next_d};
}

void BypassSetScheduler::InBypassTest() {
   auto day_edge_bit = bypass_set_.GetDayEdgeBit();
   int E = graph_.GetEdgeList().size();
   int in_bypass_cnt = 0;

   rep(d, D_) {
      EdgeBit bypass_bit;

      rep(e, E) {
         if (!day_edge_bit[d][e]) continue;

         bypass_bit |= graph_.GetBypassBit(e);
      }

      assert(bypass_set_.day_bypass_bit_[d] == bypass_bit);

      rep(e, E) {
         if (!day_edge_bit[d][e]) continue;
         if (bypass_bit[e]) {
            assert(!bypass_set_.bypass_generator_list_[e].empty());
            in_bypass_cnt++;
         } else {
            assert(bypass_set_.bypass_generator_list_[e].empty());
         }
      }
   }

   assert(in_bypass_cnt == bypass_set_.InBypassEdgeCount());
}
