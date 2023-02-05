#pragma once

#include <vector>
#include <tuple>
#include <set>
#include <bitset>

using Node = int;
using Edge = std::tuple<Node, Node, long long>;  // from, to, dist
using Adj = std::tuple<int, Node, long long>;    // edge index, to, dist
using Coord = std::pair<int, int>;

static constexpr int MaxEdge = 3000;
using EdgeBit = std::bitset<MaxEdge + 1>;

static constexpr long long DIST_INF = 1000000000;

class Graph {
  public:
   Graph(int N);

   void AddEdge(int u, int v, long long w);

   // ノードの座標を取得する
   Coord GetNodeCoord(Node u) const {
      return node_coord_[u];
   }

   const std::vector<Coord>& GetCoordList() const {
      return node_coord_;
   }

   // ノードの座標を設定する
   void SetNodeCoord(Node u, int x, int y);

   // 前処理を行う
   // - 2ノード間距離の総和を求める
   // - 最短路木を求める
   // - Edge betweennessを求める
   void Prep();

   // 辺を削除した時の不満度と非連結なノードペア数を求める
   std::pair<long long, int> CalcCost(const std::vector<int>& del_edge_index_list) const;
   std::pair<long long, int> CalcScheduleCost(int D, const std::vector<int>& schedule) const;

   // ノード間の平方距離
   long long CalcNodeSqDist(Node u, Node v) const;
   long long CalcEdgeSqDist(const Edge& edge_1, const Edge& edge_2) const;

   const std::vector<Edge>& GetEdgeList() const {
      return edge_list_;
   }

   const EdgeBit& GetBypassBit(const int e) const {
      return edge_bypass_[e];
   }

   const int GetEdgeIndex(Node u, Node v) const;

   int GetEdgeBetweenness(int e) const {
      return edge_betweenness_[e];
   }

   int GetNodeSize() const {
      return N_;
   }

  protected:
   int N_;

   // 最短路木を構築する
   int SetShortestTree(Node start, Node node, Node p, const std::vector<long long>& min_dist);

   long long total_dist_;                     // ノード間距離の総和
   std::vector<long long> node_sum_dist_;     // node_sum_dist_[n]: ノードnからの距離の総和
   std::vector<EdgeBit> node_shortest_tree_;  // node_shortest_tree_[n]: ノードnの最短路木

   std::vector<EdgeBit> edge_bypass_;   // edge_bypass_[e]: 辺eを削除した際の迂回路(edge indexの集合)
   std::vector<int> edge_betweenness_;  // edge_betweenness_[e]: 辺eのedge betweenness

   std::vector<Edge> edge_list_;             // 辺リスト
   std::vector<std::vector<Adj>> adj_list_;  // 隣接リスト

   std::vector<Coord> node_coord_;  // ノードの座標
};