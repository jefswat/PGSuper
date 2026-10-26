///////////////////////////////////////////////////////////////////////
// RfaLive - AASHTO LRFD HL-93 placement on influence surfaces
//
// The surface is sampled on a regular grid in roadway coordinates:
//   s = distance along the alignment (lane direction)
//   o = transverse offset (increasing to the right)
// so lanes are straight rows of the grid, which also handles curved
// bridges (the caller maps (s,o) to plan coordinates when sampling).
//
// Rules implemented (AASHTO LRFD 9th Ed.):
//   3.6.1.1.1  design lanes: N = INT(w / 12 ft), 12 ft wide, placed anywhere
//              across the roadway (lanes may not overlap)
//   3.6.1.1.2  multiple presence factors 1.20 / 1.00 / 0.85 / 0.65
//   3.6.1.2    design truck (8-32-32 k, 14 ft, 14-30 ft), tandem (2x25 k @ 4 ft),
//              lane 0.64 klf over 10 ft width, applied where it increases the effect
//   3.6.1.3.1  wheel centre >= 2 ft from lane edge; 90% of two trucks
//              (>= 50 ft apart) + 90% lane for negative moment / pier reactions
//   3.6.2.1    IM = 33% on truck and tandem, not on lane load
// Every lane chooses its own vehicle and longitudinal position.
///////////////////////////////////////////////////////////////////////
#pragma once
#include <vector>
#include <algorithm>
#include <string>

namespace rfa
{
   struct HL93
   {
      // SI defaults (N, m)
      double truckFront = 35585.5, truckRear = 142342.0;  // 8 k, 32 k
      double truckSpacing1 = 4.2672;                      // 14 ft
      double rearMin = 4.2672, rearMax = 9.144;           // 14..30 ft
      double tandem = 111206.0, tandemSpacing = 1.2192;   // 25 k @ 4 ft
      double laneLoad = 9339.9;                           // 0.64 klf (N/m along lane)
      double laneLoadWidth = 3.048;                       // 10 ft
      double laneWidth = 3.6576;                          // 12 ft
      double wheelEdge = 0.6096;                          // 2 ft
      double gauge = 1.8288;                              // 6 ft
      double twoTruckGap = 15.24;                         // 50 ft
      double IM = 0.33;
      double mpf[4] = { 1.20, 1.00, 0.85, 0.65 };          // 4+ lanes use the last
      double MPF(int n) const { return n <= 0 ? 0.0 : mpf[(std::min)(n, 4) - 1]; }
   };

   struct Grid2D
   {
      double s0 = 0, ds = 0.1524; int ns = 0;  // s_k = s0 + k*ds
      double o0 = 0, dO = 0.1524; int no = 0;  // o_j = o0 + j*dO
      std::vector<double> v;                    // v[k*no + j]
      double at(int k, int j) const { return v[(size_t)k * no + j]; }
   };

   enum class Vehicle { None, Truck, Tandem, TwoTrucks };

   struct LanePlacement
   {
      double laneLeft = 0;      // o of the lane's left edge
      Vehicle vehicle = Vehicle::None;
      double vehicleCentre = 0; // o of the vehicle centreline
      double vehicleS = 0;      // s of the lead axle (as positioned)
      double rearSpacing = 0;   // truck variable spacing used
      int direction = 1;        // +1 axles trail toward +s from lead axle, -1 opposite
      double laneStripLeft = 0; // o of the 10 ft lane load strip's left edge
      double secondS = 0;       // two trucks: s of the second truck's lead axle
      double vehicleEffect = 0; // incl. IM (and 0.9 for two trucks)
      double laneEffect = 0;    // incl. 0.9 for two trucks
   };

   struct LiveResult
   {
      double value = 0;     // governing effect incl. IM and MPF (sign of the search)
      int lanes = 0;        // number of loaded lanes
      double mpf = 0;
      std::vector<LanePlacement> placement;
      std::vector<double> byLanes; // governing value (incl. MPF) for 1..N loaded lanes
   };

   struct SearchOptions
   {
      int sign = +1;            // +1 maximise, -1 minimise (result value has this sign)
      bool twoTrucks = false;   // include 90% two-truck + 90% lane case
      int maxLanes = -1;        // -1: INT(roadway / 12 ft)
      double roadLeft = 0, roadRight = 0; // curb-to-curb limits in o
   };

   // Influence surface search (multi-lane, MPF)
   LiveResult SearchSurface(const Grid2D& g, const HL93& ll, const SearchOptions& opt);

   // One design lane on a line girder (no MPF): influence line IL sampled at s0 + k*ds
   LiveResult SearchLine(const std::vector<double>& IL, double s0, double ds, const HL93& ll, int sign, bool twoTrucks);

   const char* VehicleName(Vehicle v);
}
