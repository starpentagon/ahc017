#pragma once

#include <vector>
#include <tuple>

using Node = int;
using Edge = std::tuple<Node, Node, long long>;  // from, to, dist
using Coord = std::pair<int, int>;

static constexpr long long DIST_INF = 1000000000;

class Graph {
  public:
   Graph(int N);

   void AddEdge(int u, int v, long long w);

   // ノードの座標を取得する
   Coord GetNodeCoord(Node u) const {
      return node_coord_[u];
   }

   // ノードの座標を設定する
   void SetNodeCoord(Node u, int x, int y);

   // 2ノード間距離の総和を求める
   void CalcAllDist();

   // 辺を削除した時の不満度を求める
   long long CalcCost(const std::vector<int>& del_edge_index_list) const;

   // ノード間の平方距離
   long long CalcNodeSqDist(Node u, Node v) const;
   long long CalcEdgeSqDist(const Edge& edge_1, const Edge& edge_2) const;

  protected:
   int N_;

   long long total_dist_;                  // ノード間距離の総和
   std::vector<long long> node_sum_dist_;  // node_sum_dist_[n]: ノードnからの距離の総和

   std::vector<Edge> edge_list_;                                    // 辺リスト
   std::vector<std::vector<std::pair<Node, long long>>> adj_list_;  // 隣接リスト

   std::vector<Coord> node_coord_;  // ノードの座標
};