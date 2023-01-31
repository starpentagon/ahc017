#pragma once

#include <vector>
#include <random>
#include "Graph.hpp"

// 遷移の種類
enum TransType {
   kSelectNotAvailable,  // 工事可能な集合に含まれていない辺を選ぶ
   kSelectAvailable,     // 工事可能な集合に含まれている辺を選ぶ
   kTransNum,
};

// 遷移確率
static constexpr int kDefaultSelectNotAvailable = 95;
static constexpr int kDefaultSelectAvailable = 5;

// コスト関数
static constexpr int kCostNonAvailable = 3000;  // 工事可能な回数が0回の辺のコスト
static constexpr int kCostOneAvailable = 30;    // 工事可能な回数が1回の辺のコスト
static constexpr int kCostTwoAvailable = 5;     // 工事可能な回数が2回の辺のコスト
static constexpr int kCostConnect = 1;          // 連結集合のサイズのコスト(距離最小化の自由度を残すため連結集合のサイズは小さい方が良い)

// 焼きなまし法のパラメタ
static constexpr int kDefaultMaxTemp = 2011;
static constexpr int kDefaultMinTemp = 298;

using Trans = std::tuple<int, int, EdgeBit>;  // 遷移情報(遷移を行う日, 削除するedge index, 追加するedge index bit)

// 日ごとに連結になる辺集合を固定して工事しないようにする
class ConnectionSet
    : public Graph {
  public:
   ConnectionSet(int N, int D);

   // 日別の工事可能な辺集合を求める
   std::vector<EdgeBit> CalcAvailEdgeSet();

   // 非連結な日数を返す
   int DisconnectedDayCount() const;

   // 工事可能な回数がcount回の辺数を返す
   int AvailCountEdge(int count) const;

   // 工事可能な回数が1回の辺のうち迂回路集合に入る辺の数を返す
   int AvailOneEdgeInBypassCount() const;

   // スケジュールの余裕度を返す
   double ScheduleRoom() const;

  protected:
   // 連結な辺集合を初期化する
   void InitDayConnectionSet();

   // 遷移先を生成する
   Trans GenerateTransition();

   // 与えられた日別の連結な辺集合に対するコストを計算する
   int CalcCost(const std::vector<EdgeBit>& day_connection_set) const;

   int D_;  // スケジュール日数

   std::vector<EdgeBit> day_connection_set_;  // 日別の連結な辺集合

   std::mt19937_64 mt_;

   // 温度パラメタ
   int max_temp_;  // 最大温度
   int min_temp_;  // 最小温度
};