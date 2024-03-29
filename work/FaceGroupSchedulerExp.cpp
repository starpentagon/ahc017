#include <cassert>
#include <queue>
#include <algorithm>
#include <iostream>
#include "BypassSetScheduler.hpp"
#include "FaceGroupSchedulerExp.hpp"
#include "UnionFind.hpp"
#include "XorShift.hpp"
#include "ShortestTree.hpp"

#include "debug.hpp"

using namespace std;

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
// clang-format on

FaceGroupSchedulerExp::FaceGroupSchedulerExp(int M, int D, int K, const Graph &graph, const vector<Faces> &face_group_list)
    : graph_(graph), M_(M), D_(D), K_(K), face_group_list_(face_group_list), mt_(1234), max_temp_(kFaceGroupExpSA_DefaultMaxTemp), min_temp_(kFaceGroupExpSA_DefaultMinTemp), edge_day_(M, -1), day_construction_count_(D, 0), day_cost_(D, 0), iter_count_(0) {
   start_time_ = chrono::system_clock::now();

   auto n1 = graph_.GetCoordNode(0, 0);
   auto n2 = graph_.GetCoordNode(500, 0);
   auto n3 = graph_.GetCoordNode(1000, 0);
   auto n4 = graph_.GetCoordNode(0, 500);
   auto n5 = graph_.GetCoordNode(500, 500);
   auto n6 = graph_.GetCoordNode(1000, 500);
   auto n7 = graph_.GetCoordNode(0, 1000);
   auto n8 = graph_.GetCoordNode(500, 1000);
   auto n9 = graph_.GetCoordNode(1000, 1000);

   rep_point_list_.emplace_back(n1);
   rep_point_list_.emplace_back(n2);
   rep_point_list_.emplace_back(n3);
   rep_point_list_.emplace_back(n4);
   rep_point_list_.emplace_back(n5);
   rep_point_list_.emplace_back(n6);
   rep_point_list_.emplace_back(n7);
   rep_point_list_.emplace_back(n8);
   rep_point_list_.emplace_back(n9);

   int N = graph_.GetNodeSize();
   min_dist_tree_.resize(D_, vector<ShortestTree>(9, ShortestTree(N)));
}

int FaceGroupSchedulerExp::ElapsedTime() {
   auto e_time = chrono::system_clock::now() - start_time_;
   int e_time_ms = chrono::duration_cast<chrono::milliseconds>(e_time).count();
   return e_time_ms;
}

void FaceGroupSchedulerExp::Initialize() {
   // 初期集合を作る
   EdgeBit scheduled;

   auto edge_avail_day = [&](int e) {
      vector<int> day_list;
      rep(d, D_) {
         day_list.emplace_back(d);
      }
      return day_list;
   };

   auto calc_day_between = [&](int d, const vector<vector<EdgeBetween>> &face_edge_list) -> pair<long long, vector<int>> {
      int L = face_edge_list.size();
      priority_queue<EdgeBetween> que;

      rep(f, L) {
         for (auto [b, e] : face_edge_list[f]) {
            if (scheduled[e]) continue;

            que.emplace(b, e);

            break;
         }
      }

      int day_cnt = day_construction_count_[d];
      long long between = 0;
      vector<int> edge_list;

      // while (day_cnt < K_ && !que.empty()) {
      while (!que.empty()) {
         auto [b, e] = que.top();
         que.pop();
         day_cnt++;

         between += b;
         edge_list.emplace_back(e);
      }

      return {between, edge_list};
   };

   int faces_cnt = 0;

   for (const auto &faces : face_group_list_) {
      faces_cnt++;
      if (faces_cnt > sche_face_group_) break;

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
         // shuffle(day_list.begin(), day_list.end(), mt_);

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
         for (auto be : best_edge_list) {
            SetDayEdge(-1, best_d, be);

            scheduled[be] = true;
            day_bit[best_d] = 1;
         }
      }
   }

   // 工事辺数の上限overの解消
   // OutputInfo();

   rep(from_d, D_) {
      if (day_construction_count_[from_d] <= K_) continue;

      rep(to_d, D_) {
         if (to_d == from_d) continue;
         if (day_construction_count_[to_d] >= K_) continue;

         AdjustMaxConst(from_d, to_d);

         if (day_construction_count_[from_d] <= K_) break;
      }
   }

   rep(from_d, D_) {
      if (day_construction_count_[from_d] <= K_) continue;

      rep(to_d, D_) {
         if (to_d == from_d) continue;
         if (day_construction_count_[to_d] >= K_) continue;

         int cnt = 0;
         while (day_construction_count_[from_d] > K_) {
            cnt++;
            // cerr << "random" << endl;
            AdjustMaxConst(from_d, to_d, true);
            // debug(from_d, to_d, day_construction_count_[from_d], day_construction_count_[to_d]);
            if (cnt > 10) break;
         }

         if (day_construction_count_[from_d] <= K_) break;
      }
   }

   rep(from_d, D_) {
      if (day_construction_count_[from_d] <= K_) continue;

      rep(to_d, D_) {
         if (to_d == from_d) continue;
         if (day_construction_count_[to_d] >= K_) continue;

         int cnt = 0;
         while (day_construction_count_[from_d] > K_) {
            cnt++;
            // cerr << "force" << endl;
            AdjustMaxConst(from_d, to_d, true, true);
            // debug(from_d, to_d, day_construction_count_[from_d], day_construction_count_[to_d]);

            if (cnt > 1000) break;
         }

         if (day_construction_count_[from_d] <= K_) break;
      }
   }

   auto graph_edge_list = graph_.GetEdgeList();

   rep(d, D_) {
      rep(i, 9) {
         for (auto [u, v, w] : graph_edge_list) {
            min_dist_tree_[d][i].AddEdge(u, v, w);
         }

         auto p = rep_point_list_[i];
         min_dist_tree_[d][i].Init(p);
      }
   }

   int E = graph_edge_list.size();

   rep(e, E) {
      int d = edge_day_[e];

      rep(i, 9) {
         min_dist_tree_[d][i].DelEdge(e);
      }
   }

   // cerr << "Init" << endl;
   //  debug(day_construction_count_);
   //   OutputInfo();
}

long long FaceGroupSchedulerExp::CalcEstimCost(int target_e, int from_d, int to_d) {
   int E = graph_.GetEdgeList().size();

   // from_d, to_dの変更前のコスト
   long long before_cost = 0;

   {
      vector<int> from_edge_list, to_edge_list;
      rep(e, E) {
         if (edge_day_[e] == from_d) from_edge_list.emplace_back(e);
         if (edge_day_[e] == to_d) to_edge_list.emplace_back(e);
      }

      before_cost += graph_.CalcCost(target_e, from_edge_list).first;
      before_cost += graph_.CalcCost(target_e, to_edge_list).first;
   }

   long long after_cost = 0;

   {
      vector<int> from_edge_list, to_edge_list;
      rep(e, E) {
         if (e == target_e) {
            to_edge_list.emplace_back(e);
         } else {
            if (edge_day_[e] == from_d) from_edge_list.emplace_back(e);
            if (edge_day_[e] == to_d) to_edge_list.emplace_back(e);
         }
      }

      after_cost += graph_.CalcCost(target_e, from_edge_list).first;
      after_cost += graph_.CalcCost(target_e, to_edge_list).first;
   }

   return before_cost - after_cost;
}

void FaceGroupSchedulerExp::MinDistCheck() {
   int E = graph_.GetEdgeList().size();

   rep(d, D_) {
      vector<int> del_edge_list;
      rep(e, E) {
         if (edge_day_[e] == d) {
            del_edge_list.emplace_back(e);
         }
      }

      rep(i, 9) {
         auto p = rep_point_list_[i];
         auto dist1 = graph_.CalcCostNode(p, del_edge_list).first;
         auto dist2 = min_dist_tree_[d][i].CalcTotalDist();

         if (abs(dist1 - dist2) >= 2) {
            // debug(d, i, dist1, dist2);
         }
      }
   }
}

long long FaceGroupSchedulerExp::CalcEstimCostByPoints(int target_e, int from_d, int to_d) {
   // from_d, to_dの変更前のコスト
   long long before_cost = 0;
   long long after_cost = 0;

   vector<int> cost_to_list(9, 0);

   rep(i, 9) {
      auto cost_from = min_dist_tree_[from_d][i].CalcTotalDist();

      cost_to_list[i] = min_dist_tree_[to_d][i].CalcTotalDist();

      before_cost += cost_from;
      before_cost += cost_to_list[i];

      after_cost += cost_to_list[i];
   }

   rep(i, 9) {
      min_dist_tree_[from_d][i].AddEdge(target_e);
      auto cost_from = min_dist_tree_[from_d][i].CalcTotalDist();
      after_cost += cost_from;
   }

   rep(i, 9) {
      min_dist_tree_[to_d][i].DelEdge(target_e);
      auto cost_to = min_dist_tree_[to_d][i].CalcTotalDist();

      after_cost += cost_to - cost_to_list[i];

      if (before_cost <= after_cost) {
         break;
      }
   }

   if (before_cost <= after_cost) {
      rep(i, 9) {
         min_dist_tree_[from_d][i].DelEdge(target_e);
         min_dist_tree_[to_d][i].AddEdge(target_e);
      }
   }

   return before_cost - after_cost;
}

void FaceGroupSchedulerExp::AdjustMaxConst(int from_d, int to_d, bool randomize_tree, bool force_e) {
   int E = graph_.GetEdgeList().size();
   int N = graph_.GetNodeSize();
   // from_dから削除する
   vector<EdgeBetween> from_d_edge;
   rep(e, E) {
      if (edge_day_[e] == from_d) {
         int b = graph_.GetEdgeBetweenness(e);
         from_d_edge.emplace_back(b, e);
      }
   }

   // edge bvetweennessの小さいものを移す
   sort(from_d_edge.begin(), from_d_edge.end());

   UnionFind uf(N);
   // to_dで全域木を求め、削除可能な辺を求める
   vector<EdgeBetween> edge_list;
   rep(e, E) {
      if (edge_day_[e] == to_d) continue;  // もともと削除予定
      auto b = graph_.GetEdgeBetweenness(e);
      edge_list.emplace_back(b, e);
   }

   if (randomize_tree) {
      shuffle(edge_list.begin(), edge_list.end(), mt_);
   } else {
      sort(edge_list.rbegin(), edge_list.rend());
   }
   const auto &edge_info = graph_.GetEdgeList();

   set<int> can_del_edge;

   rep(i, (int)edge_list.size()) {
      auto [b, e] = edge_list[i];

      if (force_e && e == from_d_edge[0].second) {
         can_del_edge.insert(e);
         continue;
      }

      auto [u, v, w] = edge_info[e];

      if (uf.IsSameGroup(u, v)) continue;

      uf.Unite(u, v);

      if ((int)uf.size(1) == N) {
         for (int j = i + 1; j < (int)edge_list.size(); j++) {
            can_del_edge.insert(edge_list[j].second);
         }
         break;
      }
   }

   for (auto [b, e] : from_d_edge) {
      if (day_construction_count_[to_d] >= K_) break;
      if (day_construction_count_[from_d] <= K_) break;
      if (!can_del_edge.count(e)) continue;

      SetDayEdge(from_d, to_d, e);
   }
}

pair<long long, long long> FaceGroupSchedulerExp::CalcCost(bool log) {
   int E = graph_.GetEdgeList().size();

   rep(d, D_) {
      vector<int> edge_list;
      rep(i, E) {
         if (edge_day_[i] == d) edge_list.emplace_back(i);
      }

      day_cost_[d] = graph_.CalcCost(edge_list).first;
   }

   long long cost = 0;
   rep(d, D_) {
      cost += day_cost_[d];
   }
   cost = (long long)round(1.0 * cost / D_);

   // 面集合のmax betweennessに関するコスト
   long long bet_cost = 0;

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

         bet_cost += max_day_bet / 100;
      }
   }

   return {cost, bet_cost};
}

pair<long long, long long> FaceGroupSchedulerExp::CalcCost(int d1, int d2) {
   int E = graph_.GetEdgeList().size();

   rep(d, D_) {
      if (d != d1 && d != d2) continue;
      vector<int> edge_list;
      rep(i, E) {
         if (edge_day_[i] == d) edge_list.emplace_back(i);
      }

      day_cost_[d] = graph_.CalcCost(edge_list).first;
   }

   long long cost = 0;
   rep(d, D_) {
      cost += day_cost_[d];
   }
   cost = (long long)round(1.0 * cost / D_);

   // 面集合のmax betweennessに関するコスト
   long long bet_cost = 0;

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

         bet_cost += max_day_bet / 10;
      }
   }

   return {cost, bet_cost};
}

void FaceGroupSchedulerExp::SetDayEdge(int from_d, int to_d, int e) {
   edge_day_[e] = to_d;

   if (from_d != -1) day_construction_count_[from_d]--;
   day_construction_count_[to_d]++;
}

FaceGroupSA_Trans FaceGroupSchedulerExp::GenerateTransition(int iter) {
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
            if (edge_day_[e] == d) {
               return {e, d, next_d};
            }
         }
      }
   }

   int e = -1;
   int progress = min(100, 100 * iter / 2000);
   int select_rnd = XorShift() % 100;

   if (select_rnd < progress) {
      // 全体からランダムに選択
      int rand_g = XorShift() % sche_face_group_;
      int rand_f = XorShift() % face_group_list_[rand_g].size();
      int rand_E = face_group_list_[rand_g][rand_f].EdgeCount();
      int ind = XorShift() % rand_E;
      int rand_e = face_group_list_[rand_g][rand_f].GetEdgeList()[ind].second;

      e = rand_e;
   } else {
      // 上位20Face Groupから選択
      int select_group_size = min(50, sche_face_group_);

      int rand_g = XorShift() % select_group_size;
      int rand_f = XorShift() % face_group_list_[rand_g].size();
      int rand_E = face_group_list_[rand_g][rand_f].EdgeCount();
      int ind = XorShift() % rand_E;
      int rand_e = face_group_list_[rand_g][rand_f].GetEdgeList()[ind].second;

      e = rand_e;
   }

   if (e == -1) {
      return {-1, -1, -1};
   }

   int cur_d = edge_day_[e];

   vector<int> avail_day_list;

   rep(d, D_) {
      if (d == cur_d) continue;
      if (day_construction_count_[d] >= K_) continue;

      avail_day_list.emplace_back(d);
   }

   if (avail_day_list.empty()) {
      return {e, cur_d, cur_d};
   }

   set<int> adj_day_set;
   auto [u, v, w] = graph_.edge_list_[e];

   for (auto [edge_index, to, w] : graph_.adj_list_[u]) {
      int d = edge_day_[edge_index];

      if (d == cur_d) continue;
      if (day_construction_count_[d] >= K_) continue;

      adj_day_set.insert(d);
   }

   for (auto [edge_index, to, w] : graph_.adj_list_[v]) {
      int d = edge_day_[edge_index];

      if (d == cur_d) continue;
      if (day_construction_count_[d] >= K_) continue;

      adj_day_set.insert(d);
   }

   int day_select_rand = XorShift() % 100;
   int next_d = -1;

   if (!adj_day_set.empty() && day_select_rand < 75) {
      vector<int> next_day_list;
      for (auto d : adj_day_set) {
         next_day_list.emplace_back(d);
      }

      int ind = 0;
      if (next_day_list.size() > 1) ind = XorShift() % next_day_list.size();
      next_d = next_day_list[ind];
   } else {
      int ind = 0;
      if (avail_day_list.size() > 1) ind = XorShift() % avail_day_list.size();
      next_d = avail_day_list[ind];
   }

   return {e, cur_d, next_d};
}

int FaceGroupSchedulerExp::MinConstrucitonDay() const {
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

pair<int, EdgeBit> FaceGroupSchedulerExp::CalcMaxPriorityDay(const EdgeBit &scheduled, vector<priority_queue<EdgePriority>> &que_list) const {
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

            top_edge = e;
            top_b = b;
            break;

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

vector<int> FaceGroupSchedulerExp::MakeSchedule(int sche_face_group) {
   int E = graph_.GetEdgeList().size();
   sche_face_group_ = min(sche_face_group, (int)face_group_list_.size());
   uniform_real_distribution<> uniform_dist(0.0, 1.0);
   static constexpr int kMaxCount = 1000 * 1000;

   int kMaxTime = 6 * 1000 - 250;

   if (E < 1500) kMaxTime = 6 * 1000 - 200;

   Initialize();
   long long cur_cost = 100000000;
   long long best_cost = cur_cost;

   vector<int> best_edge_day = edge_day_;

   auto MakeTrans = [&](int e, int from_d, int to_d) {
      edge_day_[e] = to_d;
      day_construction_count_[from_d]--;
      day_construction_count_[to_d]++;
   };

   iter_count_ = kMaxCount;
   int skip_cnt = 0;

   rep(i, kMaxCount) {
      if (best_cost == 0) {
         break;
      }

      if (ElapsedTime() > kMaxTime) {
         iter_count_ = i;
         break;
      }

      // 遷移
      auto [trans_e, from_d, to_d] = GenerateTransition(i);

      if (from_d == to_d) {
         continue;
      }

      if (trans_e == -1) {
         // 変更加えることができない
         break;
      }

      // auto estim_delta = CalcEstimCost(trans_e, from_d, to_d);
      auto estim_delta = CalcEstimCostByPoints(trans_e, from_d, to_d);
      bool search_update = false;

      if (estim_delta > 0) {
         // 更新できる場合は必ず遷移する
         search_update = true;
      }

      // debug(i, skip_cnt, estim_delta);

      if (!search_update) {
         // 元に戻す
         skip_cnt++;
         continue;
      }

      MakeTrans(trans_e, from_d, to_d);

      // debug(i, delta_improve, estim_delta);

      // cur_cost -= delta_improve;
      cur_cost -= estim_delta;
      // cur_bet -= delta_improve_bet;

      if (chmin(best_cost, cur_cost)) {
         best_edge_day = edge_day_;
         // debug(i, best_cost, cur_cost);
      }
   }

   edge_day_ = best_edge_day;

   vector<int> schedule(E, -1);

   rep(e, E) {
      schedule[e] = edge_day_[e] + 1;
   }

   // debug(day_construction_count_);
   // OutputInfo();

   return schedule;
}

void FaceGroupSchedulerExp::SetSchedule(int e, int d, EdgeBit &scheduled) {
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
      if (d == 0) continue;
      if (cnt > K) over_k++;
   }

   return over_k;
}

bool FaceGroupSchedulerExp::OutputInfo() const {
   int E = graph_.GetEdgeList().size();
   vector<int> s(E);
   rep(i, E) s[i] = edge_day_[i] + 1;

   map<int, vector<int>> day_edge_list;
   rep(i, E) {
      int d = edge_day_[i];
      if (d != -1) {
         day_edge_list[d].emplace_back(i);
      }
   }

   auto [sche_cost, sche_discon_cnt] = graph_.CalcScheduleCost(D_, s);

   cerr << "Cost=" << sche_cost << ' ';
   cerr << "DisconCnt=" << sche_discon_cnt << ' ';
   cerr << "OverK=" << CalcOverK(K_, s) << ' ';
   cerr << "InBypass=-1" << ' ';
   cerr << endl;

   return sche_discon_cnt == 0;
}
