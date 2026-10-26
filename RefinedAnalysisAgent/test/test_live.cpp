// Validation of the HL-93 search against hand calculations (kip-ft units)
#include "../fem/RfaLive.h"
#include <cstdio>
#include <cmath>
#include <vector>
using namespace rfa;
static int g_fail = 0, g_pass = 0;
static void check(const char* name, double got, double expect, double reltol)
{
   double err = std::fabs(got - expect) / std::max(std::fabs(expect), 1e-30);
   bool ok = err <= reltol;
   std::printf("%-60s got %12.5g  expect %12.5g  err %7.3f%%  %s\n", name, got, expect, 100 * err, ok ? "ok" : "FAIL");
   ok ? g_pass++ : g_fail++;
}
static HL93 KipFt()
{
   HL93 h;
   h.truckFront = 8; h.truckRear = 32; h.truckSpacing1 = 14; h.rearMin = 14; h.rearMax = 30;
   h.tandem = 25; h.tandemSpacing = 4; h.laneLoad = 0.64; h.laneLoadWidth = 10; h.laneWidth = 12;
   h.wheelEdge = 2; h.gauge = 6; h.twoTruckGap = 50;
   return h;
}
// moment influence line at x=a of a simple span L
static double ILM(double x, double a, double L) { if (x < 0 || x > L) return 0; return x <= a ? x * (L - a) / L : a * (L - x) / L; }

int main()
{
   HL93 ll = KipFt();
   const double ds = 0.5;
   // 1. 100 ft simple span, midspan moment: truck 1520, lane 800, tandem 1200 -> 1.33*1520+800 = 2821.6
   {
      const double L = 100; int n = (int)(L / ds) + 1;
      std::vector<double> IL(n); for (int i = 0; i < n; i++) IL[i] = ILM(i * ds, 50, L);
      auto r = SearchLine(IL, 0, ds, ll, +1, false);
      check("SS 100ft M(mid): truck+lane, 1 lane", r.value, 1.33 * 1520 + 800, 1e-9);
      check("SS 100ft M(mid): vehicle part (truck 1520*1.33)", r.placement[0].vehicleEffect, 1.33 * 1520, 1e-9);
      check("SS 100ft M(mid): rear spacing 14 ft", r.placement[0].rearSpacing, 14, 1e-9);
   }
   // 2. 30 ft span: tandem governs at midspan: 25*7.5 + 25*5.5 = 325 ; truck: 32*7.5+32*0.5+8*0.5 = 260 ; lane 0.64*900/8=72
   {
      const double L = 30; int n = (int)(L / ds) + 1;
      std::vector<double> IL(n); for (int i = 0; i < n; i++) IL[i] = ILM(i * ds, 15, L);
      auto r = SearchLine(IL, 0, ds, ll, +1, false);
      check("SS 30ft M(mid): tandem governs 1.33*325+72", r.value, 1.33 * 325 + 72, 1e-9);
      check("SS 30ft vehicle is tandem", r.placement[0].vehicle == Vehicle::Tandem ? 1 : 0, 1, 0);
   }
   // 3. Reaction influence (end shear), 100 ft: truck at support: 32*1 + 32*0.86 + 8*0.72 = 65.28; lane 32 -> 1.33*65.28+32
   {
      const double L = 100; int n = (int)(L / ds) + 1;
      std::vector<double> IL(n); for (int i = 0; i < n; i++) IL[i] = 1 - i * ds / L;
      auto r = SearchLine(IL, 0, ds, ll, +1, false);
      check("SS 100ft end reaction 1.33*65.28+32", r.value, 1.33 * 65.28 + 32, 1e-9);
   }
   // 4. Two-span continuous 2x100 ft, reaction influence line at the middle support (analytic):
   //    R(x) = x(3L^2 - x^2)/(2L^3) for 0<=x<=L, symmetric.  With 2 trucks the 90% case should govern.
   {
      const double L = 100; int n = (int)(2 * L / ds) + 1;
      std::vector<double> IL(n);
      for (int i = 0; i < n; i++) { double x = i * ds; double xx = x <= L ? x : 2 * L - x; IL[i] = xx * (3 * L * L - xx * xx) / (2 * L * L * L); }
      auto r1 = SearchLine(IL, 0, ds, ll, +1, false);
      auto r2 = SearchLine(IL, 0, ds, ll, +1, true);
      // brute-force reference for the two-truck case
      auto at = [&](double x) { if (x < 0 || x > 2 * L) return 0.0; double xx = x <= L ? x : 2 * L - x; return xx * (3 * L * L - xx * xx) / (2 * L * L * L); };
      double best2 = 0;
      for (double a = -28; a <= 2 * L; a += ds)
         for (double b = a + 28 + 50; b <= 2 * L + 28; b += ds)
         {
            double t1 = 8 * at(a) + 32 * at(a + 14) + 32 * at(a + 28);
            double t2 = 8 * at(b) + 32 * at(b + 14) + 32 * at(b + 28);
            double t1r = 8 * at(a + 28) + 32 * at(a + 14) + 32 * at(a);
            double t2r = 8 * at(b + 28) + 32 * at(b + 14) + 32 * at(b);
            best2 = std::max({ best2, t1 + t2, t1r + t2r });
         }
      double lane = 0.64 * 2 * (L * 0.625); // integral of R over each span = 5L/8
      check("2-span pier reaction: 2 trucks (brute force) + lane, 90%", r2.value, 0.9 * (1.33 * best2 + lane), 1e-4);
      check("2-span pier reaction: two-truck case governs", r2.value > r1.value ? 1 : 0, 1, 0);
   }
   // 5. Surface uniform across 40 ft roadway = beam line IL -> 3 lanes, 0.85*3 per-lane effect
   {
      const double L = 100;
      Grid2D g; g.s0 = 0; g.ds = ds; g.ns = (int)(L / ds) + 1; g.o0 = 0; g.dO = 0.5; g.no = 81; // 40 ft
      g.v.resize((size_t)g.ns * g.no);
      for (int k = 0; k < g.ns; k++) for (int j = 0; j < g.no; j++) g.v[(size_t)k * g.no + j] = ILM(k * ds, 50, L);
      SearchOptions o; o.sign = +1; o.roadLeft = 0; o.roadRight = 40;
      auto r = SearchSurface(g, ll, o);
      // per lane: vehicle = 2 wheel lines x half axle = full axle -> 1.33*1520; lane load 10 ft strip x 0.064 ksf = 0.64 klf -> 800
      double lane1 = 1.33 * 1520 + 800;
      check("uniform surface 40 ft: 3 lanes x 0.85", r.value, 3 * 0.85 * lane1, 1e-9);
      check("uniform surface: 3 lanes loaded", r.lanes, 3, 0);
      check("uniform surface: 1 lane x 1.2", r.byLanes[0], 1.2 * lane1, 1e-9);
      check("uniform surface: 2 lanes x 1.0", r.byLanes[1], 2 * lane1, 1e-9);
      // negative search on a positive surface: nothing to load
      o.sign = -1;
      auto rn = SearchSurface(g, ll, o);
      check("uniform positive surface, min search = 0", rn.value + 1.0, 1.0, 1e-12);
   }
   // 6. Surface that only exists over a 4 ft strip at o = 20..24 ft (girder line): best is one truck
   //    whose wheel sits on it; lane load strip covers it -> 1.2 * (0.5*1.33*1520 + 0.064*4*800/... )
   {
      const double L = 100;
      Grid2D g; g.s0 = 0; g.ds = ds; g.ns = (int)(L / ds) + 1; g.o0 = 0; g.dO = 0.5; g.no = 121; // 60 ft
      g.v.assign((size_t)g.ns * g.no, 0.0);
      for (int k = 0; k < g.ns; k++) for (int j = 40; j <= 48; j++) g.v[(size_t)k * g.no + j] = ILM(k * ds, 50, L);
      SearchOptions o; o.sign = +1; o.roadLeft = 0; o.roadRight = 60;
      auto r = SearchSurface(g, ll, o);
      // Two adjacent lanes put one wheel each on the strip (wheels 4 ft apart across the lane line):
      // 2 lanes x MPF 1.0 x (0.5 * 1.33*1520) + lane load over the strip (trapezoid footprint of
      // the sampled step = 4.5 ft) -> beats one lane x 1.2
      double laneE = (0.64 / 10) * 4.5 * (0.5 * L * 25);
      check("4 ft strip surface: 2 lanes, one wheel each + lane", r.value, 1.0 * (2 * 0.5 * 1.33 * 1520 + laneE), 1e-9);
      check("4 ft strip surface: 2 lanes loaded", r.lanes, 2, 0);
      check("4 ft strip surface: 1-lane value = 1.2 x (1 wheel + lane)", r.byLanes[0], 1.2 * (0.5 * 1.33 * 1520 + laneE), 1e-9);
   }
   std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
   return g_fail ? 1 : 0;
}
