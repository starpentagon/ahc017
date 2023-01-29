#pragma once

#include <vector>
#include <map>

namespace std {
// UnionFindで素集合を管理するクラス
// ノードIDは1-indexed
class UnionFind {
  public:
   UnionFind(const size_t N);

   // node_1とnode_2が所属する集合を結合する(Union by size)
   // 計算量: O(almost 1)
   void Unite(size_t node_1, size_t node_2);

   // node_1とnode_2が同じ集合に入っているか
   // 計算量: O(almost 1)
   bool IsSameGroup(size_t node_1, size_t node_2) const;

   // nodeが所属する集合のサイズを返す
   // 計算量: O(almost 1)
   size_t size(size_t node) const;

   // 各グループを返す(親ノードの番号 -> 子のリスト)
   map<size_t, vector<size_t>> EnumGroup() const;

   // nodeの親のnode_idを返す
   size_t root(size_t node) const;

  private:
   // ノードの数
   size_t N_;

   // parent_node_id_[i]: 親ノードのノードID, rootの場合はparent_node_id_[i] ==
   // iになる
   mutable vector<size_t> parent_node_id_;

   // tree_size_[i]: ルートノードiに含まれる集合のサイズ
   vector<size_t> tree_size_;
};
}  // namespace std
