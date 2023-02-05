#pragma once

#include <vector>
#include <queue>
#include <random>

#include "FaceGroup.hpp"
#include "BypassSet.hpp"

using EdgePriority = std::pair<long long, int>;

int CalcOverK(int K, const std::vector<int>& schedule);

// コスト関数
static constexpr long kFaceGroupSA_OverK = 1000;              // 工事辺数の超過
static constexpr long kFaceGroupSA_SameUnselectedEdge = 500;  // 同一の面集合内の未決定の辺が同日に工事
static constexpr long kFaceGroupSA_SameSalectedEdge = 200;    // 同一の面集合内の決定済の辺が同日に工事

// 焼きなまし法のパラメタ
static constexpr int kFaceGroupSA_DefaultMaxTemp = 5011;
static constexpr int kFaceGroupSA_DefaultMinTemp = 298;

using FaceGroupSA_Trans = std::tuple<int, int, int>;  // 遷移情報(遷移を辺, 元の工事日, 遷移先の工事日)

class FaceGroupScheduler {
  public:
   FaceGroupScheduler(int M, int D, int K, const Graph& graph, const std::vector<EdgeBit>& day_avail_edge_bit, const std::vector<Faces>& face_group_list);

   // 辺ごとの工事日を決める
   std::vector<int> MakeSchedule();
   std::vector<int> MakeScheduleOld(const std::vector<Faces>& face_group_list);

   const BypassSet& GetBypassSet() const {
      return bypass_set_;
   }

  protected:
   // 工事予定日を初期化する
   void Initialize();
   void InitializeRandom();

   // 工事件数が少ない日を返す
   int MinConstrucitonDay() const;

   // 辺の優先度の総和が最大になる日と工事する辺を求める
   std::pair<int, EdgeBit> CalcMaxPriorityDay(const EdgeBit& scheduled, std::vector<std::priority_queue<EdgePriority>>& que_list) const;

   FaceGroupSA_Trans GenerateTransition();

   void SetSchedule(int e, int d, EdgeBit& scheduled);

   // スケジュールのコストを計算する
   long long CalcCost(bool log = false) const;

   void OutputInfo() const;

   const Graph& graph_;

   int M_;  // 辺の数
   int D_;  // スケジュール日数
   int K_;  // 工事可能な辺数

   // 面集合のリスト
   std::vector<Faces> face_group_list_;

   std::mt19937_64 mt_;

   // 温度パラメタ
   int max_temp_;  // 最大温度
   int min_temp_;  // 最小温度

   std::vector<int> edge_day_;                // edge_day_[e]: eの工事日
   std::vector<int> day_construction_count_;  //  day_construction_count_[d]: d日目の工事件数

   std::vector<EdgeBit> day_avail_edge_bit_;  // 日別の工事可能な辺集合
   EdgeBit avail_one_edge_;                   // 工事可能日が1日のみの辺

   BypassSet bypass_set_;
};