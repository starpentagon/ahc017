#include <cassert>
#include "UnionFind.hpp"

using namespace std;

UnionFind::UnionFind(const size_t N)
    : N_(N), parent_node_id_(N + 1), tree_size_(N + 1, 1) {
   // 全ノードをrootで初期化する
   for (size_t i = 0; i <= N; i++) {
      parent_node_id_[i] = i;
   }
}

size_t UnionFind::root(size_t node) const {
   assert(1 <= node && node <= N_);

   // ルートノード以外のノードを記録し直接、親ノードをルートノードにつなぎ変える(経路圧縮)
   vector<size_t> internal_nodes;

   while (parent_node_id_[node] != node) {
      internal_nodes.push_back(node);
      node = parent_node_id_[node];
   }

   for (auto n : internal_nodes) {
      parent_node_id_[n] = node;
   }

   return node;
}

bool UnionFind::IsSameGroup(size_t node_1, size_t node_2) const {
   auto parent_1 = root(node_1);
   auto parent_2 = root(node_2);

   return parent_1 == parent_2;
}

void UnionFind::Unite(size_t node_1, size_t node_2) {
   if (IsSameGroup(node_1, node_2)) {
      // すでに同じ木の場合は何もしない
      return;
   }

   // サイズの小さい方を大きいにつなぐ
   auto size_1 = size(node_1);
   auto size_2 = size(node_2);

   size_t union_from = node_1, union_to = node_2;

   if (size_1 > size_2) {
      union_from = node_2;
      union_to = node_1;
   }
   auto parent_from = root(union_from);
   auto parent_to = root(union_to);

   parent_node_id_[parent_from] = parent_to;
   tree_size_[parent_to] = tree_size_[parent_to] + tree_size_[parent_from];
}

size_t UnionFind::size(size_t node) const {
   auto parent = root(node);
   return tree_size_[parent];
}

map<size_t, vector<size_t>> UnionFind::EnumGroup() const {
   map<size_t, vector<size_t>> group;

   for (size_t i = 1; i <= N_; i++) {
      auto parent = root(i);
      group[parent].push_back(i);
   }

   return group;
}
