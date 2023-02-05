#include <queue>
#include <algorithm>
#include <iostream>
#include "BypassSetScheduler.hpp"
#include "FaceGroupScheduler.hpp"
#include "XorShift.hpp"

#include "debug.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
// clang-format on

FaceGroupScheduler::FaceGroupScheduler(int M, int D, int K, const Graph &graph, const vector<EdgeBit> &day_avail_edge_bit, const vector<Faces> &face_group_list)
    : graph_(graph), M_(M), D_(D), K_(K), face_group_list_(face_group_list), mt_(1234), max_temp_(kFaceGroupSA_DefaultMaxTemp), min_temp_(kFaceGroupSA_DefaultMinTemp), edge_day_(M, -1), day_construction_count_(D, 0), day_avail_edge_bit_(day_avail_edge_bit), bypass_set_(D, K, graph) {
}

void FaceGroupScheduler::InitializeRandom() {
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

      edge_day_[e] = d;
      day_construction_count_[d]++;

      bypass_set_.AddEdge(d, e);
   }
}

void FaceGroupScheduler::Initialize() {
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
   EdgeBit scheduled;

   auto edge_avail_day = [&](int e) {
      vector<int> day_list;
      rep(d, D_) {
         if (day_avail_edge_bit_[d][e]) day_list.emplace_back(d);
      }
      return day_list;
   };

   auto calc_day_between = [&](int d, const vector<vector<EdgeBetween>> &face_edge_list) -> pair<long long, vector<int>> {
      int L = face_edge_list.size();
      priority_queue<EdgeBetween> que;

      rep(f, L) {
         for (auto [b, e] : face_edge_list[f]) {
            if (scheduled[e]) continue;
            if (!day_avail_edge_bit_[d][e]) continue;

            que.emplace(b, e);

            break;
         }
      }

      int day_cnt = day_construction_count_[d];
      long long between = 0;
      vector<int> edge_list;

      while (day_cnt < K_ && !que.empty()) {
         auto [b, e] = que.top();
         que.pop();
         day_cnt++;

         between += b;
         edge_list.emplace_back(e);
      }

      return {between, edge_list};
   };

   for (const auto &faces : face_group_list_) {
      priority_queue<EdgeBetween> que;
      int L = faces.size();
      vector<vector<EdgeBetween>> face_edge_list(L);
      EdgeBit edge_bit;  // Face内での共有辺はindexが小さいface(辺数が少ないface)に割り当てる

      rep(f, L) {
         for (auto [b, e] : faces[f].GetEdgeList()) {
            if (scheduled[e] || edge_bit[e]) continue;
            edge_bit[e] = 1;

            face_edge_list[f].emplace_back(b, e);
            que.emplace(b, e);
         }
      }

      bitset<31> day_bit;

      while (!que.empty()) {
         auto [b, e] = que.top();
         que.pop();
         if (scheduled[e]) continue;

         auto day_list = edge_avail_day(e);

         long long best_bet = -1, best_d = -1;
         vector<int> best_edge_list;

         for (auto d : day_list) {
            if (day_bit[d]) continue;

            auto [bet, edge_list] = calc_day_between(d, face_edge_list);

            if (chmax(best_bet, bet)) {
               best_d = d;
               best_edge_list = edge_list;
            }
         }

         if (best_d == -1) continue;
         debug(best_d, best_edge_list);

         for (auto be : best_edge_list) {
            edge_day_[be] = best_d;
            day_construction_count_[best_d]++;
            scheduled[be] = true;

            day_bit[best_d] = 1;
            bypass_set_.AddEdge(best_d, e);
         }
      }
   }

   // 工事日が未割り当ての辺(工事辺数を越えているケース)はランダムに割り当てる
   rep(e, E) {
      if (edge_day_[e] == -1) {
         auto day_list = edge_avail_day(e);
         int ind = XorShift() % day_list.size();
         int d = day_list[ind];

         edge_day_[e] = d;
         day_construction_count_[d]++;
         scheduled[e] = true;

         bypass_set_.AddEdge(d, e);
      }
   }

   // debug
   // cerr << "Init" << endl;
   // OutputInfo();
}

long long FaceGroupScheduler::CalcCost(bool log) const {
   long long bypass_cost = 0;

   bypass_cost += kBySetSA_EdgeInBypass * bypass_set_.InBypassEdgeCount();

   rep(d, D_) {
      if (bypass_set_.GetDayEdgeCount(d) > K_) bypass_cost += (bypass_set_.GetDayEdgeCount(d) - K_) * kBySetSA_OverK;
   }

   if (log) {
      cerr << "bypass_cost: " << bypass_cost << endl;
   }
   // debug
   static bool f_out = false;
   if (!f_out && bypass_cost == 0) {
      cerr << "Bypass clear:" << endl;
      OutputInfo();
      f_out = true;
   }

   //   return bypass_cost;
   //--debug

   long long cost = 1000 * bypass_cost;
   int E = graph_.GetEdgeList().size();

   // 日別の工事辺数
   vector<int> day_edge_count(D_, 0);

   rep(e, E) {
      assert(edge_day_[e] >= 0);
      day_edge_count[edge_day_[e]]++;
   }

   // 面集合制約に関するコスト
   EdgeBit scheduled;

   auto can_assign = [](auto can_assign, int i, const vector<int> &edge_list, const map<int, vector<int>> &edge_to_face_index, vector<bool> &assign) -> bool {
      if (i == (int)edge_list.size()) return true;

      int e = edge_list[i];

      auto it = edge_to_face_index.find(e);
      assert(it != edge_to_face_index.end());

      for (auto f : it->second) {
         if (assign[f]) continue;

         assign[f] = true;
         bool check = can_assign(can_assign, i + 1, edge_list, edge_to_face_index, assign);
         assign[f] = false;

         if (check) return true;
      }

      return false;
   };

   for (auto faces : face_group_list_) {
      int L = faces.size();

      vector<set<int>> day_edge_set(D_);

      rep(f, L) {
         for (auto [b, e] : faces[f].GetEdgeList()) {
            int d = edge_day_[e];
            day_edge_set[d].insert(e);
         }
      }

      // 辺 -> face index
      map<int, vector<int>> edge_to_face_index;

      rep(f, L) {
         for (auto [b, e] : faces[f].GetEdgeList()) {
            edge_to_face_index[e].emplace_back(f);
         }
      }

      /*
      rep(d, D_) {
         // 未決定の辺を同じ日に同じ面で工事していないか
         {
            vector<int> unselected_edge_list;

            for (auto e : day_edge_set[d]) {
               if (scheduled[e]) continue;
               unselected_edge_list.emplace_back(e);
            }

            // todo: bool値ではなくassign出来なかった最小の辺数を求めたほうが良さそう
            vector<bool> unselect_assign(L, false);
            bool unselect_check = can_assign(can_assign, 0, unselected_edge_list, edge_to_face_index, unselect_assign);

            if (!unselect_check) {
               // debug -- うまく行っていない。スケジュールが悪いか？BypassSet cost=0なら考慮しなくても大丈夫なので外してみる
               // cost += kFaceGroupSA_SameUnselectedEdge;
               // -- debug

               if (log) {
                  cerr << "kFaceGroupSA_SameUnselectedEdge: " << kFaceGroupSA_SameUnselectedEdge << endl;
                  debug(unselected_edge_list);
                  debug(edge_to_face_index);
               }
            }
         }

         // 決定済の辺を含めて同じに工事をすると迂回路が長くなるので可能であれば避ける
         {
            vector<int> selected_edge_list;

            for (auto e : day_edge_set[d]) {
               selected_edge_list.emplace_back(e);
            }

            // todo: bool値ではなくassign出来なかった最小の辺数を求めたほうが良さそう
            vector<bool> select_assign(L, false);
            bool select_check = can_assign(can_assign, 0, selected_edge_list, edge_to_face_index, select_assign);

            if (!select_check) {
               // debug -- うまく行っていない。スケジュールが悪いか？BypassSet cost=0なら考慮しなくても大丈夫なので外してみる
               // cost += kFaceGroupSA_SameSalectedEdge;
               // -- debug

               if (log) {
                  cerr << "kFaceGroupSA_SameSalectedEdge: " << kFaceGroupSA_SameSalectedEdge << endl;
               }
            }
         }
      }
      */
      // schedule update
      rep(f, L) {
         for (auto [b, e] : faces[f].GetEdgeList()) {
            scheduled[e] = 1;
         }
      }
   }

   // 面集合のmax betweennessに関するコスト
   for (auto faces : face_group_list_) {
      int L = faces.size();
      vector<vector<long long>> day_betweenness(D_);

      rep(f, L) {
         for (auto [b, e] : faces[f].GetEdgeList()) {
            int d = edge_day_[e];
            day_betweenness[d].emplace_back(b);
         }
      }

      rep(d, D_) {
         if (day_betweenness[d].empty()) continue;

         auto max_day_bet = *max_element(day_betweenness[d].begin(), day_betweenness[d].end());

         if (log) {
            // cerr << "day : " << d << ", " << max_day_bet / 1000 << endl;
         }

         cost += max_day_bet / 1000;
      }
   }

   return cost;
}

FaceGroupSA_Trans FaceGroupScheduler::GenerateTransition() {
   int E = graph_.GetEdgeList().size();

   // 工事辺数が超過している場合は解消する遷移を生成する
   rep(d, D_) {
      if (day_construction_count_[d] > K_) {
         vector<int> avail_day_list;

         rep(next_d, D_) {
            if (day_construction_count_[next_d] >= K_) continue;
            avail_day_list.emplace_back(next_d);
         }

         int ind = 0;
         if (avail_day_list.size() > 1) ind = XorShift() % avail_day_list.size();
         int next_d = avail_day_list[ind];

         rep(e, E) {
            if (edge_day_[e] == d && day_avail_edge_bit_[next_d][e]) {
               return {e, d, next_d};
            }
         }
      }
   }

   int e = -1, cnt = 0;

   if (bypass_set_.InBypassEdgeCount() > 0) {
      int rnd = XorShift() % 100;
      if (rnd < kBySetSA_DefaultSelectInBypass) {
         e = bypass_set_.SelectInBypassEdge(avail_one_edge_);
      } else {
         e = bypass_set_.SelectBypassGeneratorEdge(avail_one_edge_);
      }
   } else {
      while (true) {
         int rand_e = XorShift() % E;
         cnt++;

         if (cnt >= 100) break;
         if (avail_one_edge_[rand_e]) continue;

         e = rand_e;
         break;
      }
   }

   if (e == -1) {
      return {-1, -1, -1};
   }

   int cur_d = edge_day_[e];

   vector<int> avail_day_list;

   rep(d, D_) {
      if (d == cur_d) continue;
      if (!day_avail_edge_bit_[d][e]) continue;
      if (day_construction_count_[d] >= K_) continue;

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

int FaceGroupScheduler::MinConstrucitonDay() const {
   int min_count = numeric_limits<int>::max();
   int min_d = -1;

   rep(d, D_) {
      if (chmin(min_count, day_construction_count_[d])) {
         min_d = d;
      }
   }

   assert(min_d != -1);
   return min_d;
}

pair<int, EdgeBit> FaceGroupScheduler::CalcMaxPriorityDay(const EdgeBit &scheduled, vector<priority_queue<EdgePriority>> &que_list) const {
   // 未スケジュールの辺が先頭に来るようにする
   int L = que_list.size();

   rep(f, L) {
      while (!que_list[f].empty()) {
         int e = que_list[f].top().second;
         if (scheduled[e]) {
            que_list[f].pop();
         } else {
            break;
         }
      }
   }

   long long max_priority = -1;
   EdgeBit max_edge_bit;
   int max_d = -1;

   rep(d, D_) {
      long long priority = 0;
      int cur_k = day_construction_count_[d];

      EdgeBit day_edge_bit;

      rep(f, L) {
         // ToDo: 前から順にみているが高い順にみた方が良い
         if (cur_k >= K_) break;
         if (que_list[f].empty()) continue;

         vector<EdgeBetween> skipped_edge;
         int top_edge = -1, top_b = -1;

         while (!que_list[f].empty()) {
            auto [b, e] = que_list[f].top();

            if (day_avail_edge_bit_[d][e]) {
               top_edge = e;
               top_b = b;
               break;
            }

            que_list[f].pop();
            skipped_edge.emplace_back(b, e);
         }

         // 元に戻す
         for (auto [b, e] : skipped_edge) {
            que_list[f].emplace(b, e);
         }

         priority += top_b;
         day_edge_bit[top_edge] = 1;
         cur_k++;
      }

      if (chmax(max_priority, priority)) {
         max_d = d;
         max_edge_bit = day_edge_bit;
      }
   }

   return {max_d, max_edge_bit};
}

vector<int> FaceGroupScheduler::MakeSchedule() {
   uniform_real_distribution<> uniform_dist(0.0, 1.0);
   static constexpr int kMaxCount = 10 * 1000;

   InitializeRandom();

   int cur_cost = CalcCost();
   int best_cost = cur_cost;
   BypassSet best_bypass_set = bypass_set_;

   vector<int> best_edge_day = edge_day_;

   auto MakeTrans = [&](int e, int from_d, int to_d) {
      edge_day_[e] = to_d;
      day_construction_count_[from_d]--;
      day_construction_count_[to_d]++;
   };

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
         // 変更加えることができない
         break;
      }

      MakeTrans(trans_e, from_d, to_d);
      bypass_set_.DelEdge(from_d, trans_e);
      bypass_set_.AddEdge(to_d, trans_e);

      // debug(trans_e, from_d, to_d);
      auto trans_cost = CalcCost();

      auto delta_improve = cur_cost - trans_cost;
      // debug(i, cur_cost, delta_improve);
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
         MakeTrans(trans_e, to_d, from_d);
         bypass_set_.DelEdge(from_d, trans_e);
         bypass_set_.AddEdge(to_d, trans_e);
         continue;
      }

      cur_cost -= delta_improve;

      if (chmin(best_cost, cur_cost)) {
         best_edge_day = edge_day_;
         best_bypass_set = bypass_set_;
         // debug(i, best_cost, cur_cost, delta_improve);
      }
   }

   edge_day_ = best_edge_day;
   bypass_set_ = best_bypass_set;

   int E = graph_.GetEdgeList().size();
   vector<int> schedule(E, -1);

   // debug
   /*
   debug(edge_day_[554], edge_day_[867], CalcCost(false));
   int trans_e = 554;
   int from_d = 4, to_d = 18;
   MakeTrans(trans_e, from_d, to_d);
   bypass_set_.DelEdge(from_d, trans_e);
   bypass_set_.AddEdge(to_d, trans_e);
   debug(edge_day_[554], edge_day_[867], CalcCost(false));
   */

   //--debug

   rep(e, E) {
      schedule[e] = edge_day_[e] + 1;
   }

   return schedule;
}

vector<int> FaceGroupScheduler::MakeScheduleOld(const vector<Faces> &face_group_list) {
   EdgeBit scheduled;

   auto top_edge = [&, this](int d, priority_queue<EdgePriority> &que) {
      while (!que.empty()) {
         int e = que.top().second;

         if (!day_avail_edge_bit_[d][e]) continue;

         if (scheduled[e]) {
            que.pop();
            continue;
         }

         return e;
      }

      return -1;
   };

   for (const auto &faces : face_group_list) {
      int L = faces.size();
      vector<priority_queue<EdgePriority>> que_list(L);

      // 面(辺集合)ごとに重要度の高い辺のqueを作る
      rep(f, L) {
         for (auto [b, e] : faces[f].GetEdgeList()) {
            if (scheduled[e]) continue;
            que_list[f].emplace(b, e);
         }
      }

      while (true) {
         auto [d, edge_bit] = CalcMaxPriorityDay(scheduled, que_list);

         bool constructed = false;

         rep(f, L) {
            if (day_construction_count_[d] >= K_) {
               d = CalcMaxPriorityDay(scheduled, que_list).first;
            }

            int e = top_edge(d, que_list[f]);

            if (e == -1) continue;

            SetSchedule(e, d, scheduled);
            constructed = true;
         }

         if (!constructed) {
            break;
         }
      }
   }

   return edge_day_;
}

void FaceGroupScheduler::SetSchedule(int e, int d, EdgeBit &scheduled) {
   scheduled[e] = 1;
   edge_day_[e] = d + 1;
   day_construction_count_[d]++;
}

int CalcOverK(int K, const std::vector<int> &schedule) {
   std::map<int, int> day_edge;
   for (auto d : schedule) {
      day_edge[d]++;
   }

   int over_k = 0;

   for (auto [d, cnt] : day_edge) {
      if (cnt > K) over_k++;
   }

   return over_k;
}

void FaceGroupScheduler::OutputInfo() const {
   int E = graph_.GetEdgeList().size();
   vector<int> s(E);
   rep(i, E) s[i] = edge_day_[i] + 1;
   auto [sche_cost, sche_discon_cnt] = graph_.CalcScheduleCost(D_, s);

   cerr << "Cost=" << sche_cost << ' ';
   cerr << "DisconCnt=" << sche_discon_cnt << ' ';
   cerr << "OverK=" << CalcOverK(K_, s) << ' ';
   cerr << "InBypass=" << bypass_set_.InBypassEdgeCount() << ' ';
   cerr << endl;
}
