#include <numeric>
#include "SqDistScheduler.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
// clang-format on

using ll = long long;

SqDistScheduler::SqDistScheduler(int N, int D, int K)
    : Graph(N), D_(D), K_(K) {
}

void SqDistScheduler::MakeSchedule() {
   CalcAllDist();

   int M = edge_list_.size();
   schedule_.resize(M);
   daily_cost_.resize(D_);
   daily_discon_count_.resize(D_);

   int E = (M + D_ - 1) / D_;  // 1日あたりの工事件数

   vector<bool> constructed(M, false);

   auto get_edge_index_list = [&]() {
      vector<int> e_list;
      e_list.reserve(M);

      rep(e, M) {
         if (constructed[e]) continue;
         e_list.emplace_back(e);
      }
      return e_list;
   };

   vector<vector<ll>> dist_tbl(M, vector<ll>(M, 0));

   rep(i, M) rep(j, M) {
      auto edge_1 = edge_list_[i];
      auto edge_2 = edge_list_[j];

      dist_tbl[i][j] = CalcEdgeSqDist(edge_1, edge_2);
   }

   // 未工事の辺
   auto calc_construct_edge = [&]() {
      auto edge_index_list = get_edge_index_list();

      vector<int> construct_edge_index;

      if ((int)edge_index_list.size() <= E) {
         construct_edge_index = edge_index_list;

         for (auto e : construct_edge_index) {
            constructed[e] = true;
         }

         return construct_edge_index;
      }

      construct_edge_index.reserve(E);

      {
         // 工事日の順序性はないので未工事の辺の先頭を工事するとしてよい
         int e = edge_index_list[0];
         construct_edge_index.emplace_back(e);
         constructed[e] = true;
      }

      while ((int)construct_edge_index.size() < E) {
         int ne = -1;
         ll max_min_dist = -1;

         rep(e, M) {
            if (constructed[e]) continue;

            ll min_dist = DIST_INF;

            for (auto ce : construct_edge_index) {
               chmin(min_dist, dist_tbl[e][ce]);
            }

            if (chmax(max_min_dist, min_dist)) {
               ne = e;
            }
         }

         construct_edge_index.emplace_back(ne);
         constructed[ne] = true;
      }

      return construct_edge_index;
   };

   rep(d, D_) {
      // 未工事の枝を1つ決める
      auto construct_edge_index = calc_construct_edge();

      for (auto e : construct_edge_index) {
         schedule_[e] = d + 1;
      }

      auto [cost, discon_count] = CalcCost(construct_edge_index);

      daily_cost_[d] = cost;
      daily_discon_count_[d] = discon_count;
   }
}

long long SqDistScheduler::CalcScheduleCost() const {
   ll sum_cost = accumulate(daily_cost_.begin(), daily_cost_.end(), 0LL);
   sum_cost /= D_;

   return sum_cost;
}

int SqDistScheduler::CalcScheduleDisconCount() const {
   ll sum_count = accumulate(daily_discon_count_.begin(), daily_discon_count_.end(), 0);
   return sum_count;
}
