#pragma once

#include <vector>
#include <random>
#include "Graph.hpp"
#include "BypassSet.hpp"

// 遷移の種類
enum BySetSA_TransType {
   kSelectInBypass,         // 迂回路集合中の工事予定辺を選ぶ
   kSelectBypassGenerator,  // 迂回路を生成している事予定辺を選ぶ
   kBySetSA_TransNum,
};

// 遷移確率
static constexpr int kBySetSA_DefaultSelectInBypass = 50;
static constexpr int kBySetSA_DefaultSelectBypassGenerator = 100 - kBySetSA_DefaultSelectInBypass;

// コスト関数
static constexpr int kBySetSA_EdgeInBypass = 1;  // 迂回路集合中の工事予定辺数
static constexpr int kBySetSA_OverK = 1000;      // 工事辺数の超過

// 焼きなまし法のパラメタ
static constexpr int kBySetSA_DefaultMaxTemp = 5011;
static constexpr int kBySetSA_DefaultMinTemp = 298;

using BySetSA_Trans = std::tuple<int, int, int>;  // 遷移情報(遷移を辺, 元の工事日, 遷移先の工事日)

// 迂回路集合が工事辺を含まないようにスケジューリングする
class BypassSetScheduler {
  public:
   BypassSetScheduler(int D, int K, const Graph& graph, const std::vector<EdgeBit>& day_avail_edge_bit);

   // 辺ごとの工事日を決める
   std::vector<int> MakeSchedule();

   const BypassSet& GetBypassSet() const {
      return bypass_set_;
   }
   void InBypassTest();

  protected:
   // 工事予定日を初期化する
   void Initialize();

   // BypassSetのコストを算出する
   int CalcCost() const;

   // 遷移先を生成する
   BySetSA_Trans GenerateTransition();

   const Graph& graph_;

   int D_;  // スケジュール日数
   int K_;  // 工事可能な辺数

   std::mt19937_64 mt_;

   // 温度パラメタ
   int max_temp_;  // 最大温度
   int min_temp_;  // 最小温度

   std::vector<EdgeBit> day_avail_edge_bit_;  // 日別の工事可能な辺集合
   EdgeBit avail_one_edge_;                   // 工事可能日が1日のみの辺

   BypassSet bypass_set_;
};