#pragma once

#include "Graph.hpp"

// nodeを始点とする最短路木を管理する
using NodeInfo = std::pair<long long, int>;  // dist, parent

class ShortestTree
    : public Graph {
  public:
   ShortestTree(int N, int node);

   long long CalcTotalDist() const;

  protected:
   void AddEdge(int e);
   void DelEdge(int e);
   void UpdateMinDistTree(int e);

   int node_;

   EdgeBit del_edge_;                     // 削除した辺フラグ
   std::vector<NodeInfo> min_dist_tree_;  // node_を始点とする最短路(node_からの距離, 親ノード)を記録する
};