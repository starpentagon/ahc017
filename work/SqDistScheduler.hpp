// 平方距離に基づく辺選択を行う
#include "Graph.hpp"

class SqDistScheduler
    : public Graph {
  public:
   SqDistScheduler(int N, int D, int K);

   void MakeSchedule();
   std::vector<int> GetSchedule() const {
      return schedule_;
   }

   long long CalcScheduleCost() const;
   int CalcScheduleDisconCount() const;

  protected:
   int D_;  // スケジュール日数
   int K_;  // 最大の変数

   std::vector<int> schedule_;            // schedule_[i]: i番目の辺の工事日
   std::vector<long long> daily_cost_;    // 日別の不満度
   std::vector<int> daily_discon_count_;  // 日別の非連結なノードペア数
};
