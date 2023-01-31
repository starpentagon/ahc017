#include <cassert>

#include "XorShift.hpp"
#include "BypassSet.hpp"

// clang-format off
#define rep(i, n) for (int i = 0; (i) < (int)(n); (i)++)

template<class T> bool chmax(T &a, const T &b) {if(a<b) {a=b; return true;} return false; }
template<class T> bool chmin(T &a, const T &b) {if(a>b) {a=b; return true;} return false; }
// clang-format on

using namespace std;

BypassSet::BypassSet(int D, int K, const Graph &graph)
    : graph_(graph), D_(D), K_(K) {
   int E = graph.GetEdgeList().size();

   day_edge_bit_.resize(D);
   day_bypass_bit_.resize(D);
   in_bypass_edge_list_.resize(E);
   bypass_generator_list_.resize(E);
}

void BypassSet::AddEdge(int d, int e) {
   int E = graph_.GetEdgeList().size();

   day_edge_bit_[d][e] = 1;

   if (day_bypass_bit_[d][e]) {
      // eが迂回路集合に含まれる
      rep(p, E) {
         if (!day_edge_bit_[d][p]) continue;
         auto bypass_bit = graph_.GetBypassBit(p);

         if (bypass_bit[e]) {
            // pの迂回路集合にeが含まれる
            in_bypass_edge_list_[p].insert(e);
            bypass_generator_list_[e].insert(p);
         }
      }
   }

   auto bypass_bit = graph_.GetBypassBit(e);
   day_bypass_bit_[d] |= bypass_bit;

   if ((day_edge_bit_[d] & bypass_bit).any()) {
      // eの迂回路集合に工事予定の辺pが含まれる
      rep(p, E) {
         if (!day_edge_bit_[d][p]) continue;
         if (!bypass_bit[p]) continue;

         in_bypass_edge_list_[e].insert(p);
         bypass_generator_list_[p].insert(e);
      }
   }
}

void BypassSet::DelEdge(int d, int e) {
   int E = graph_.GetEdgeList().size();

   day_edge_bit_[d][e] = 0;

   // 迂回路集合を再構築
   day_bypass_bit_[d].reset();

   rep(p, E) {
      if (!day_edge_bit_[d][p]) continue;
      day_bypass_bit_[d] |= graph_.GetBypassBit(p);
   }

   // eを生成元とするbypass_generator_list_の更新
   for (auto q : in_bypass_edge_list_[e]) {
      auto it = bypass_generator_list_[q].find(e);
      assert(it != bypass_generator_list_[q].end());
      bypass_generator_list_[q].erase(it);
   }

   in_bypass_edge_list_[e].clear();

   for (auto p : bypass_generator_list_[e]) {
      auto it = in_bypass_edge_list_[p].find(e);
      assert(it != in_bypass_edge_list_[p].end());
      in_bypass_edge_list_[p].erase(it);
   }

   bypass_generator_list_[e].clear();
}

int BypassSet::InBypassEdgeCount() const {
   int count = 0;
   int E = graph_.GetEdgeList().size();

   rep(e, E) {
      if (!bypass_generator_list_[e].empty()) count++;
   }

   return count;
}

int BypassSet::SelectInBypassEdge(const EdgeBit &avail_one_edge) const {
   int E = graph_.GetEdgeList().size();
   vector<int> edge_list;

   rep(e, E) {
      if (avail_one_edge[e]) continue;
      if (!bypass_generator_list_[e].empty()) edge_list.emplace_back(e);
   }

   if (edge_list.empty()) {
      return -1;
   }

   assert(!edge_list.empty());

   int ind = XorShift() % edge_list.size();
   return edge_list[ind];
}

int BypassSet::GetDay(int e) const {
   rep(d, D_) if (day_edge_bit_[d][e]) return d;
   return -1;
}

BypassSet &BypassSet::operator=(const BypassSet &bypass_set) {
   D_ = bypass_set.D_;
   K_ = bypass_set.K_;

   day_edge_bit_ = bypass_set.day_edge_bit_;
   day_bypass_bit_ = bypass_set.day_bypass_bit_;
   in_bypass_edge_list_ = bypass_set.in_bypass_edge_list_;
   bypass_generator_list_ = bypass_set.bypass_generator_list_;

   return *this;
}
