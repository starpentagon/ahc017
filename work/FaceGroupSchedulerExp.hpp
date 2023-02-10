#pragma once

#include <vector>
#include <queue>
#include <random>
#include <chrono>

#include "ShortestTree.hpp"
#include "FaceGroup.hpp"
#include "BypassSet.hpp"
using EdgePriority = std::pair<long long, int>;

int CalcOverK(int K, const std::vector<int>& schedule);

// コスト関数
static constexpr long kFaceGroupExpSA_OverK = 1000;              // 工事辺数の超過
static constexpr long kFaceGroupExpSA_SameUnselectedEdge = 500;  // 同一の面集合内の未決定の辺が同日に工事
static constexpr long kFaceGroupExpSA_SameSalectedEdge = 200;    // 同一の面集合内の決定済の辺が同日に工事

// 焼きなまし法のパラメタ
static constexpr int kFaceGroupExpSA_DefaultMaxTemp = 0;
static constexpr int kFaceGroupExpSA_DefaultMinTemp = 0;

using FaceGroupSA_Trans = std::tuple<int, int, int>;  // 遷移情報(遷移を辺, 元の工事日, 遷移先の工事日)

class FaceGroupSchedulerExp {
  public:
   FaceGroupSchedulerExp(int M, int D, int K, const Graph& graph, const std::vector<Faces>& face_group_list);

   // 辺ごとの工事日を決める
   std::vector<int> MakeSchedule(int sche_face_group);
   std::vector<int> MakeScheduleOld(const std::vector<Faces>& face_group_list);

   const BypassSet& GetBypassSet() const {
      return bypass_set_;
   }

   int GetIterCount() const {
      return iter_count_;
   }

  protected:
   // 工事予定日を初期化する
   void Initialize();
   void InitializeRandom();

   void AdjustMaxConst(int from_d, int to_d, bool randomize_tree = false, bool force_e = false);

   void SetDayEdge(int from_d, int to_d, int e);

   // 工事件数が少ない日を返す
   int MinConstrucitonDay() const;

   // 辺の優先度の総和が最大になる日と工事する辺を求める
   std::pair<int, EdgeBit> CalcMaxPriorityDay(const EdgeBit& scheduled, std::vector<std::priority_queue<EdgePriority>>& que_list) const;

   FaceGroupSA_Trans GenerateTransition(int iter);

   void SetSchedule(int e, int d, EdgeBit& scheduled);

   int ElapsedTime();
   // スケジュールのコストを計算する
   std::pair<long long, long long> CalcCost(bool log = false);
   std::pair<long long, long long> CalcCost(int d1, int d2);

   long long CalcEstimCost(int e, int from_d, int to_d);
   long long CalcEstimCostByPoints(int e, int from_d, int to_d);

   void MinDistCheck();

   bool OutputInfo() const;

   const Graph& graph_;

   int M_;  // 辺の数
   int D_;  // スケジュール日数
   int K_;  // 工事可能な辺数

   int sche_face_group_;  // スケジューリングするFaceGroupの数

   // 面集合のリスト
   std::vector<Faces> face_group_list_;

   std::mt19937_64 mt_;

   // 温度パラメタ
   int max_temp_;  // 最大温度
   int min_temp_;  // 最小温度

   std::vector<int> edge_day_;                // edge_day_[e]: eの工事日
   std::vector<int> day_construction_count_;  //  day_construction_count_[d]: d日目の工事件数

   std::vector<long long> day_cost_;  // 日別のコスト

   std::chrono::system_clock::time_point start_time_;
   BypassSet bypass_set_;

   int iter_count_;

   std::vector<Node> rep_point_list_;                      // 代表点
   std::vector<std::vector<ShortestTree>> min_dist_tree_;  // 日別代表点別の最短路木
};