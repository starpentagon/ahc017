#pragma once

#include <vector>
#include <set>
#include "Graph.hpp"

class BypassSetScheduler;

// 迂回路集合の管理
class BypassSet {
  public:
   BypassSet(int D, int K, const Graph& graph);

   // d日目に辺eを追加する
   void AddEdge(int d, int e);

   // d日目から辺eを削除する
   void DelEdge(int d, int e);

   // 迂回路集合中の辺の数を返す
   int InBypassEdgeCount() const;

   // 迂回路集合中の(day, edge)をランダムに返す
   // 迂回路集合中に辺がない場合はassert and -1を返す
   int SelectInBypassEdge(const EdgeBit& avail_one_edge) const;

   // 迂回路集合に辺を持つ(day, edge)をランダムに返す
   // 迂回路集合に辺を持つ辺がない場合はassert and -1を返す
   int SelectBypassGeneratorEdge(const EdgeBit& avail_one_edge) const;

   // 工事日に含まれる辺の数を返す
   int GetDayEdgeCount(int d) const {
      return day_edge_bit_[d].count();
   }

   int GetDay(int e) const;

   const std::vector<EdgeBit>& GetDayEdgeBit() const {
      return day_edge_bit_;
   }

   BypassSet& operator=(const BypassSet& bypass_set);

  protected:
   const Graph& graph_;
   int D_;  // スケジュール日数
   int K_;  // 工事可能な辺数

   std::vector<EdgeBit> day_edge_bit_;                 // day_edge_bit_[d]: 工事日dの辺集合
   std::vector<EdgeBit> day_bypass_bit_;               // day_bypass_bit_[d]: 工事日dの迂回路集合
   std::vector<std::set<int>> in_bypass_edge_list_;    // in_bypass_edge_list_[e]: eの迂回路中の工事予定辺のリスト
   std::vector<std::set<int>> bypass_generator_list_;  // bypass_generator_list_[e]: eを含む迂回路の生成元リスト

   friend BypassSetScheduler;
};
