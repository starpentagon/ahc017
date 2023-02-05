#pragma once

#include <set>
#include <map>
#include <bitset>
#include <cassert>
#include <set>

#include "Graph.hpp"

// 面
using EdgeBetween = std::pair<int, int>;      // edge betweenness, edge_index
using Rect = std::tuple<int, int, int, int>;  // [min_x, min_y, max_x, max_y]

class Face {
  public:
   Face(const Graph& graph, const std::vector<Node>& node_list);

   int EdgeCount() const {
      return edge_list_.size();
   }

   int UnselectedCount() const {
      return unselected_.size();
   }

   // 未選択の辺の最大edge betweennessを返す
   int MaxBetweenness() const;

   void Select(const int e);

   bool IsAdjacent(const Face& face) const;

   const std::vector<EdgeBetween>& GetEdgeList() const {
      return edge_list_;
   }

   // 未選択の辺リストを返す
   std::vector<EdgeBetween> GetUnselectedEdgeList() const;

   // 矩形を返す
   const Rect& GetRect() const {
      return rect_;
   }

   // 矩形の大きさ(幅+高さ)を返す
   int RectSize() const;

  protected:
   const Graph& graph_;

   Rect rect_;  // 面を含む矩形領域

   std::set<int> selected_;
   std::set<int> unselected_;

   std::vector<EdgeBetween> edge_list_;  // 辺集合, edge betweennessの降順にソート
};

using Faces = std::vector<Face>;
using FaceBit = std::bitset<2510>;  // F = 2 + M - N <= 2 + 3000 - 500

class FaceGroup
    : public Graph {
  public:
   FaceGroup(int N, int D);
   std::vector<Faces> MakeGroup();

   // Faceを連結できるか判定する
   bool CanFaceGroup(const std::vector<int>& face_index, int adj_face_index) const;

  protected:
   // GreedyにFace集合を拡大する
   // Face集合は辺数順に昇順ソートされる
   std::vector<int> Greedy();
   void GenerateFaces();

   int D_;

   std::vector<std::vector<int>> face_adj_list_;  // 面の隣接リスト

   std::map<int, std::set<int>> face_max_bet_map_;  // max_bet -> face index set

   FaceBit face_bit_;  // 選択済みのFace bit

   Faces face_list_;  // グラフの面リスト
   std::vector<std::vector<int>> face_group_list_;
};
