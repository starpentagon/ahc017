#include "XorShift.hpp"

unsigned int XorShift() {
   // ref: https://qiita.com/drken/items/7c6ff2aa4d8fce1c9361#9-xorshift
   static unsigned int tx = 123456789, ty = 362436069, tz = 521288629, tw = 88675123;

   unsigned int tt = (tx ^ (tx << 11));

   tx = ty;
   ty = tz;
   tz = tw;

   return (tw = (tw ^ (tw >> 19)) ^ (tt ^ (tt >> 8)));
}