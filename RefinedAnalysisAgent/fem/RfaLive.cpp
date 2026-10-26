///////////////////////////////////////////////////////////////////////
// RfaLive - HL-93 placement on influence surfaces (see RfaLive.h)
///////////////////////////////////////////////////////////////////////
#include "RfaLive.h"
#include <cmath>
#include <limits>

namespace rfa
{
   const char* VehicleName(Vehicle v)
   {
      switch (v)
      {
      case Vehicle::Truck: return "Design truck";
      case Vehicle::Tandem: return "Design tandem";
      case Vehicle::TwoTrucks: return "90% two trucks";
      default: return "none";
      }
   }

   namespace
   {
      const double NEG = -std::numeric_limits<double>::infinity();

      struct LongBest
      {
         double val = 0; // 0 = vehicle entirely off the structure
         Vehicle veh = Vehicle::None;
         int k = 0, dir = 1, hsteps = 0;
      };

      // Longitudinal placement on a 1D line w (already multiplied by the search sign).
      // `scale` multiplies axle loads (0.5 for one wheel line of a surface, 1 for a line girder).
      class LongSearch
      {
      public:
         LongSearch(const HL93& ll, double ds) : m_ll(ll)
         {
            a1 = std::max(1, (int)std::lround(ll.truckSpacing1 / ds));
            hmin = std::max(1, (int)std::lround(ll.rearMin / ds));
            hmax = std::max(hmin, (int)std::lround(ll.rearMax / ds));
            hstep = std::max(1, (int)std::lround(0.3048 / ds)); // 1 ft increments
            tand = std::max(1, (int)std::lround(ll.tandemSpacing / ds));
            gap = std::max(1, (int)std::lround(ll.twoTruckGap / ds));
         }

         static double At(const std::vector<double>& w, int k) { return (k >= 0 && k < (int)w.size()) ? w[k] : 0.0; }

         LongBest Truck(const std::vector<double>& w, double scale) const
         {
            LongBest b;
            int n = (int)w.size(), ext = a1 + hmax;
            for (int k = -ext; k < n + ext; k++)
               for (int dir = -1; dir <= 1; dir += 2)
               {
                  double f = m_ll.truckFront * At(w, k) + m_ll.truckRear * At(w, k + dir * a1);
                  for (int h = hmin; h <= hmax; h += hstep)
                  {
                     double v = scale * (f + m_ll.truckRear * At(w, k + dir * (a1 + h)));
                     if (v > b.val) { b.val = v; b.veh = Vehicle::Truck; b.k = k; b.dir = dir; b.hsteps = h; }
                  }
                  if (hmax % hstep != hmin % hstep) // make sure the 30 ft limit itself is tried
                  {
                     double v = scale * (f + m_ll.truckRear * At(w, k + dir * (a1 + hmax)));
                     if (v > b.val) { b.val = v; b.veh = Vehicle::Truck; b.k = k; b.dir = dir; b.hsteps = hmax; }
                  }
               }
            return b;
         }

         LongBest Tandem(const std::vector<double>& w, double scale) const
         {
            LongBest b;
            int n = (int)w.size();
            for (int k = -tand; k < n + tand; k++)
            {
               double v = scale * m_ll.tandem * (At(w, k) + At(w, k + tand));
               if (v > b.val) { b.val = v; b.veh = Vehicle::Tandem; b.k = k; b.dir = 1; b.hsteps = 0; }
            }
            return b;
         }

         // Two design trucks (14 ft rear spacing), same direction, >= gap between them (100%, caller applies 0.9)
         LongBest TwoTrucks(const std::vector<double>& w, double scale) const
         {
            LongBest b;
            int n = (int)w.size(), ext = 2 * a1;
            int lo = -ext, hi = n + ext; // reference index range
            int m = hi - lo;
            std::vector<double> t(m);
            for (int dir = -1; dir <= 1; dir += 2)
            {
               for (int i = 0; i < m; i++)
               {
                  int k = lo + i;
                  t[i] = scale * (m_ll.truckFront * At(w, k) + m_ll.truckRear * At(w, k + dir * a1) + m_ll.truckRear * At(w, k + dir * 2 * a1));
               }
               // suffix max of t for the second truck
               std::vector<double> suf(m + 1, NEG); std::vector<int> arg(m + 1, -1);
               for (int i = m - 1; i >= 0; i--) { suf[i] = suf[i + 1]; arg[i] = arg[i + 1]; if (t[i] > suf[i]) { suf[i] = t[i]; arg[i] = i; } }
               int sep = 2 * a1 + gap;
               for (int i = 0; i + sep < m; i++)
               {
                  double v = t[i] + suf[i + sep];
                  if (v > b.val) { b.val = v; b.veh = Vehicle::TwoTrucks; b.k = lo + i; b.dir = dir; b.hsteps = arg[i + sep] + lo; /* second truck ref */ }
               }
            }
            return b;
         }

         int a1, hmin, hmax, hstep, tand, gap;
      private:
         const HL93& m_ll;
      };
   }

   ////////////////////////////////////////////////////////////////////
   LiveResult SearchLine(const std::vector<double>& IL, double s0, double ds, const HL93& ll, int sign, bool twoTrucks)
   {
      std::vector<double> w(IL.size());
      for (size_t i = 0; i < IL.size(); i++) w[i] = sign * IL[i];
      LongSearch L(ll, ds);
      LongBest tr = L.Truck(w, 1.0), ta = L.Tandem(w, 1.0);
      LongBest veh = (ta.val > tr.val) ? ta : tr;
      double lane = 0;
      for (size_t i = 0; i < w.size(); i++)
      {
         // trapezoid weights on the positive part
         double wt = (i == 0 || i + 1 == w.size()) ? 0.5 : 1.0;
         lane += wt * std::max(w[i], 0.0);
      }
      lane *= ll.laneLoad * ds;

      LiveResult r;
      LanePlacement p;
      p.vehicle = veh.veh;
      p.vehicleS = s0 + veh.k * ds;
      p.direction = veh.dir;
      p.rearSpacing = veh.veh == Vehicle::Truck ? veh.hsteps * ds : 0;
      p.vehicleEffect = (1 + ll.IM) * veh.val;
      p.laneEffect = lane;
      double std_ = p.vehicleEffect + p.laneEffect;
      if (twoTrucks)
      {
         LongBest tt = L.TwoTrucks(w, 1.0);
         double v2 = 0.9 * ((1 + ll.IM) * tt.val + lane);
         if (v2 > std_)
         {
            p.vehicle = Vehicle::TwoTrucks; p.vehicleS = s0 + tt.k * ds; p.direction = tt.dir; p.rearSpacing = ll.truckSpacing1;
            p.secondS = s0 + tt.hsteps * ds;
            p.vehicleEffect = 0.9 * (1 + ll.IM) * tt.val; p.laneEffect = 0.9 * lane;
            std_ = v2;
         }
      }
      r.value = sign * std_;
      r.lanes = 1; r.mpf = 1.0;
      r.placement.push_back(p);
      r.byLanes.push_back(r.value);
      return r;
   }

   ////////////////////////////////////////////////////////////////////
   LiveResult SearchSurface(const Grid2D& g, const HL93& ll, const SearchOptions& opt)
   {
      LiveResult res;
      const int ns = g.ns, no = g.no;
      const int sg = opt.sign >= 0 ? 1 : -1;
      LongSearch Ls(ll, g.ds);

      const int gaugeSteps = (int)std::lround(ll.gauge / g.dO);
      const int half = gaugeSteps / 2;
      const int laneSteps = (int)std::lround(ll.laneWidth / g.dO);
      const int edgeSteps = (int)std::lround(ll.wheelEdge / g.dO);
      const int lw = (int)std::lround(ll.laneLoadWidth / g.dO);

      // --- vehicles per transverse centre position c ---
      std::vector<LongBest> veh(no), two(no);
      std::vector<double> w(ns);
      for (int c = half; c + half < no; c++)
      {
         for (int k = 0; k < ns; k++) w[k] = sg * (g.at(k, c - half) + g.at(k, c + (gaugeSteps - half)));
         LongBest tr = Ls.Truck(w, 0.5), ta = Ls.Tandem(w, 0.5);
         veh[c] = (ta.val > tr.val) ? ta : tr;
         if (opt.twoTrucks) two[c] = Ls.TwoTrucks(w, 0.5);
      }

      // --- lane load strips by left edge a ---
      const double q = ll.laneLoad / ll.laneLoadWidth; // N/m^2
      std::vector<double> laneE(no, 0.0);
      {
         std::vector<double> pre(no + 1);
         for (int k = 0; k < ns; k++)
         {
            pre[0] = 0;
            for (int j = 0; j < no; j++) pre[j + 1] = pre[j] + sg * g.at(k, j);
            double wk = (k == 0 || k + 1 == ns) ? 0.5 : 1.0;
            for (int a = 0; a + lw < no; a++)
            {
               // trapezoid across the strip
               double strip = (pre[a + lw + 1] - pre[a]) - 0.5 * (sg * g.at(k, a) + sg * g.at(k, a + lw));
               strip *= g.dO;
               if (strip > 0) laneE[a] += wk * strip;
            }
         }
         for (auto& e : laneE) e *= q * g.ds;
      }

      // --- per-lane best value by lane left-edge index L ---
      int Lmin = (int)std::ceil((opt.roadLeft - g.o0) / g.dO - 1e-9);
      int Lmax = (int)std::floor((opt.roadRight - g.o0) / g.dO + 1e-9) - laneSteps;
      Lmin = std::max(Lmin, 0);
      Lmax = std::min(Lmax, no - 1 - laneSteps);
      int Nlanes = opt.maxLanes >= 0 ? opt.maxLanes : (int)std::floor((opt.roadRight - opt.roadLeft) / ll.laneWidth + 1e-9);
      if (Lmax < Lmin || Nlanes <= 0) return res;

      struct LaneBest { double v; LanePlacement p; };
      std::vector<LaneBest> lb(Lmax - Lmin + 1);
      for (int L = Lmin; L <= Lmax; L++)
      {
         LaneBest best{ NEG, {} };
         int c0 = L + edgeSteps + half, c1 = L + laneSteps - edgeSteps - (gaugeSteps - half);
         int bc = -1; double bv = 0;
         int btc = -1; double btv = 0;
         for (int c = c0; c <= c1; c++)
         {
            if (c < half || c + half >= no) continue;
            if (bc < 0 || veh[c].val > bv) { bc = c; bv = veh[c].val; }
            if (opt.twoTrucks && (btc < 0 || two[c].val > btv)) { btc = c; btv = two[c].val; }
         }
         int ba = L; double bl = 0;
         for (int a = L; a <= L + laneSteps - lw; a++) if (a + lw < no && laneE[a] > bl) { bl = laneE[a]; ba = a; }

         LanePlacement p;
         p.laneLeft = g.o0 + L * g.dO;
         p.laneStripLeft = g.o0 + ba * g.dO;
         if (bc >= 0)
         {
            const LongBest& vb = veh[bc];
            p.vehicle = vb.veh; p.vehicleCentre = g.o0 + bc * g.dO; p.vehicleS = g.s0 + vb.k * g.ds;
            p.direction = vb.dir; p.rearSpacing = vb.veh == Vehicle::Truck ? vb.hsteps * g.ds : 0;
            p.vehicleEffect = (1 + ll.IM) * vb.val;
         }
         p.laneEffect = bl;
         best.v = p.vehicleEffect + p.laneEffect; best.p = p;
         if (opt.twoTrucks && btc >= 0)
         {
            double v2 = 0.9 * ((1 + ll.IM) * btv + bl);
            if (v2 > best.v)
            {
               const LongBest& tb = two[btc];
               LanePlacement p2 = p;
               p2.vehicle = Vehicle::TwoTrucks; p2.vehicleCentre = g.o0 + btc * g.dO; p2.vehicleS = g.s0 + tb.k * g.ds;
               p2.direction = tb.dir; p2.rearSpacing = ll.truckSpacing1;
               p2.secondS = g.s0 + tb.hsteps * g.ds;
               p2.vehicleEffect = 0.9 * (1 + ll.IM) * btv; p2.laneEffect = 0.9 * bl;
               best.v = v2; best.p = p2;
            }
         }
         lb[L - Lmin] = best;
      }

      // --- DP over lane positions: F[n][L] = best sum of n lanes with left edges >= L ---
      int nL = Lmax - Lmin + 1;
      std::vector<std::vector<double>> F(Nlanes + 1, std::vector<double>(nL + 1, NEG));
      std::vector<std::vector<char>> take(Nlanes + 1, std::vector<char>(nL + 1, 0));
      for (int i = 0; i <= nL; i++) F[0][i] = 0;
      for (int n = 1; n <= Nlanes; n++)
         for (int i = nL - 1; i >= 0; i--)
         {
            double skip = F[n][i + 1];
            int nxt = i + laneSteps;
            double prev = nxt <= nL ? F[n - 1][std::min(nxt, nL)] : (n - 1 == 0 ? 0.0 : NEG);
            double tk = (prev == NEG) ? NEG : lb[i].v + prev;
            if (tk > skip) { F[n][i] = tk; take[n][i] = 1; }
            else F[n][i] = skip;
         }

      int bestN = 0; double bestV = NEG;
      for (int n = 1; n <= Nlanes; n++)
      {
         double v = F[n][0] == NEG ? NEG : ll.MPF(n) * F[n][0];
         res.byLanes.push_back(v == NEG ? 0.0 : sg * v);
         if (v > bestV) { bestV = v; bestN = n; }
      }
      if (bestN == 0) return res;
      res.value = sg * bestV;
      res.lanes = bestN;
      res.mpf = ll.MPF(bestN);
      // reconstruct
      int i = 0, n = bestN;
      while (n > 0 && i < nL)
      {
         if (take[n][i]) { res.placement.push_back(lb[i].p); i += laneSteps; n--; }
         else i++;
      }
      return res;
   }
}
