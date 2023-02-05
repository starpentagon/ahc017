#pragma once
#include "Graph.hpp"

// 閉路分割を求める
class CycleSplit
    : public Graph {
  public:
   CycleSplit(int N, int D);

   std::vector<std::vector<int>> CalcSplit();

  protected:
   std::vector<std::vector<int>> FindLoop(const int e, const int L) const;
   std::vector<std::vector<int>> edge_split_;
};