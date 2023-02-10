#include <algorithm>

#include "DualGraph.hpp"
#include "FaceGroup.hpp"

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }

//template<class T> ostream& operator<<(ostream& os, vector<T>& vec){ rep(i, vec.size()) os << vec[i] << (i+1==(int)vec.size() ? "" : " "); return os;}
// clang-format on

using namespace std;

Face::Face(const Graph& graph, const vector<Node>& node_list)
    : graph_(graph), rect_{1001, 1001, -1, -1} {
   int L = node_list.size();
   assert(L >= 3);

   for (int i = 1; i <= L; i++) {
      int u = node_list[i - 1];
      int v = node_list[i % L];

      int e = graph.GetEdgeIndex(u, v);
      int b = graph.GetEdgeBetweenness(e);

      edge_list_.emplace_back(b, e);
      unselected_.insert(e);
   }

   sort(edge_list_.rbegin(), edge_list_.rend());

   int x_min = 1001, y_min = 1001, x_max = -1, y_max = -1;

   for (auto u : node_list) {
      auto [x, y] = graph.GetNodeCoord(u);

      chmin(x_min, x);
      chmin(y_min, y);
      chmax(x_max, x);
      chmax(y_max, y);
   }

   rect_ = make_tuple(x_min, y_min, x_max, y_max);
}

int Face::MaxBetweenness() const {
   for (auto [b, e] : edge_list_) {
      if (selected_.count(e)) continue;
      return b;
   }

   return 0;
}

void Face::Select(const int e) {
   auto it = unselected_.find(e);
   assert(it != unselected_.end());

   unselected_.erase(it);
   selected_.insert(e);
}

bool Face::IsAdjacent(const Face& face) const {
   const auto& edge_list = face.GetEdgeList();

   for (auto [b, e] : edge_list) {
      if (unselected_.count(e) || selected_.count(e)) return true;
   }

   return false;
}

int Face::RectSize() const {
   auto [x_min, y_min, x_max, y_max] = rect_;
   int dx = x_max - x_min;
   int dy = y_max - y_min;

   return dx + dy;
}

vector<EdgeBetween> Face::GetUnselectedEdgeList() const {
   vector<EdgeBetween> edge_list;

   for (auto [b, e] : edge_list_) {
      if (unselected_.count(e)) {
         edge_list.emplace_back(b, e);
      }
   }

   return edge_list;
}

FaceGroup::FaceGroup(int N, int D)
    : Graph(N), D_(D) {
}

vector<Faces> FaceGroup::MakeGroup() {
   Prep(false);
   GenerateFaces();  // 面の生成

   int E = edge_list_.size();
   int F = face_list_.size();

   // 辺 -> 面indexの対応
   vector<vector<int>> edge_to_face_index(E);

   rep(i, F) {
      const auto& face = face_list_[i];
      const auto face_edge_list = face.GetEdgeList();

      for (auto [b, e] : face_edge_list) {
         edge_to_face_index[e].emplace_back(i);
      }
   }

   // Max betweennessの算出
   rep(i, F) {
      const auto& face = face_list_[i];
      auto max_bet = face.MaxBetweenness();

      face_max_bet_map_[max_bet].insert(i);
   }

   // 面の隣接リストの作成
   face_adj_list_.resize(F);

   rep(i, F) rep(j, i) {
      if (face_list_[i].IsAdjacent(face_list_[j])) {
         face_adj_list_[i].emplace_back(j);
         face_adj_list_[j].emplace_back(i);
      }
   }

   // 面集合の生成
   while (true) {
      auto face_group = Greedy();

      if (face_group.empty()) break;
      face_group_list_.emplace_back(face_group);

      // 辺を選択
      map<int, int> update_face_index;  // max betweennessの更新が必要なface index

      for (auto face_index : face_group) {
         auto edge_list = face_list_[face_index].GetUnselectedEdgeList();

         for (auto [b, e] : edge_list) {
            for (auto i : edge_to_face_index[e]) {
               if (!update_face_index.count(i)) {
                  update_face_index[i] = face_list_[i].MaxBetweenness();
               }

               face_list_[i].Select(e);
            }
         }
      }

      // Max betweennessを更新
      for (auto [face_index, prev_max] : update_face_index) {
         // 更新前のmax betweennessを削除
         auto prev_max_it = face_max_bet_map_.find(prev_max);
         assert(prev_max_it != face_max_bet_map_.end());

         auto prev_it = prev_max_it->second.find(face_index);
         assert(prev_it != prev_max_it->second.end());
         prev_max_it->second.erase(prev_it);

         // 更新
         int max_bet = face_list_[face_index].MaxBetweenness();
         face_max_bet_map_[max_bet].insert(face_index);
      }
   }

   vector<Faces> face_group_list(face_group_list_.size());

   rep(i, face_group_list_.size()) {
      int L = face_group_list_[i].size();
      face_group_list[i].reserve(L);

      for (auto f : face_group_list_[i]) {
         face_group_list[i].emplace_back(face_list_[f]);
      }
   }

   return face_group_list;

   // debug
   /*
      rep(i, face_group_list_.size()) {
         auto face_group = face_group_list_[i];
         set<int> edge_set;
         for (auto f : face_group) {
            for (auto [b, e] : face_list_[f].GetEdgeList()) {
               edge_set.insert(e);
            }
         }

         for (auto e : edge_set) {
            cerr << e << ", ";
         }
         cerr << endl;
      }
      // --debug
   */
}

bool FaceGroup::CanFaceGroup(const std::vector<int>& face_index_list, int adj_face_index) const {
   // 迂回路が大きくなり過ぎないように連結成分の「大きさ」でも制限をかける
   auto [x_min, y_min, x_max, y_max] = face_list_[adj_face_index].GetRect();

   for (auto i : face_index_list) {
      auto [x1, y1, x2, y2] = face_list_[i].GetRect();

      chmin(x_min, x1);
      chmin(y_min, y1);
      chmax(x_max, x2);
      chmax(y_max, y2);
   }

   // ToDo: tuning
   int dx = x_max - x_min, dy = y_max - y_min;

   if (dx + dy > 300) {
      // cerr << "size over" << endl;
      return false;
   }
   using FaceSize = pair<int, int>;  // 辺の数, face_index
   vector<FaceSize> face_size_list;
   face_size_list.reserve(face_index_list.size() + 1);

   face_size_list.emplace_back(
       face_list_[adj_face_index].UnselectedCount(), adj_face_index);

   for (auto i : face_index_list) {
      int d = face_list_[i].UnselectedCount();
      face_size_list.emplace_back(d, i);
   }

   // FaceGroupの中で決定する部分サイクルの短いものから順に作っていく
   sort(face_size_list.begin(), face_size_list.end());
   int max_d = -1;
   EdgeBit selected;

   for (auto [d, i] : face_size_list) {
      auto edge_list = face_list_[i].GetUnselectedEdgeList();

      int face_d = 0;

      for (auto [b, e] : edge_list) {
         if (selected[e]) continue;

         selected[e] = true;
         face_d++;
      }

      // 部分サイクルの作成にD日より日数がかかる場合は結合しない
      if (chmax(max_d, face_d)) {
         if (max_d > D_) {
            return false;
         }
      }
   }

   return true;
}

void FaceGroup::GenerateFaces() {
   auto node_coord_list = GetCoordList();
   vector<Coord> coord_list(N_);

   rep(i, N_) {
      coord_list[i] = node_coord_list[i + 1];
   }

   DualGraph dual_graph(coord_list);

   for (auto [u, v, w] : edge_list_) {
      dual_graph.add_edge(u - 1, v - 1);
   }

   auto face_node_list = dual_graph.build();
   face_list_.reserve(face_node_list.size());

   for (const auto& node_list : face_node_list) {
      face_list_.emplace_back(*this, node_list);
   }
}

vector<int> FaceGroup::Greedy() {
   int face_index = -1;

   // Edge Betweennessの大きいものから考慮する
   for (auto it = face_max_bet_map_.rbegin(); it != face_max_bet_map_.rend(); ++it) {
      auto [max_b, face_set] = *it;
      face_index = -1;
      int min_rect_size = numeric_limits<int>::max();

      for (auto i : face_set) {
         // 未選択の辺がD本以下(スケジュール内で連結性を保つため)のものを選択
         // 矩形サイズが小さいものから選択
         if (face_bit_[i]) continue;
         if (face_list_[i].UnselectedCount() > D_) continue;

         int rect_size = face_list_[i].RectSize();

         if (chmin(min_rect_size, rect_size)) {
            face_index = i;
         }
      }

      if (face_index != -1) {
         // debug(face_index, max_b, face_list_[face_index].GetEdgeList());
         break;
      }
   }

   vector<int> face_index_list;  // face_indexのリスト

   if (face_index == -1) {
      return face_index_list;
   }

   using FaceEdgeCnt = pair<int, int>;
   vector<FaceEdgeCnt> face_edge_cnt_list;

   auto add_index = [&](int index) {
      face_edge_cnt_list.emplace_back(face_list_[index].UnselectedCount(), index);
      face_index_list.emplace_back(index);
      face_bit_[index] = 1;
   };

   add_index(face_index);

   using MaxBetIndex = pair<int, int>;  // max betweenness, adj_face_index
   vector<MaxBetIndex> max_bet_face_list;

   for (auto adj_face_index : face_adj_list_[face_index]) {
      if (face_bit_[adj_face_index]) continue;
      // debug(face_index, adj_face_index, CanFaceGroup(face_index_list, adj_face_index));
      int max_bet = face_list_[adj_face_index].MaxBetweenness();
      max_bet_face_list.emplace_back(max_bet, adj_face_index);
   }

   sort(max_bet_face_list.rbegin(), max_bet_face_list.rend());

   for (auto [max_b, adj_face_index] : max_bet_face_list) {
      // debug(max_b, adj_face_index, CanFaceGroup(face_index_list, adj_face_index));

      if (CanFaceGroup(face_index_list, adj_face_index)) {
         add_index(adj_face_index);
      }
   }

   // 辺数の昇順にソート
   sort(face_edge_cnt_list.begin(), face_edge_cnt_list.end());
   assert(face_edge_cnt_list.size() == face_index_list.size());

   rep(i, face_edge_cnt_list.size()) {
      face_index_list[i] = face_edge_cnt_list[i].second;
   }

   return face_index_list;
}
