///////////////////////////////////////////////////////////////////////
// RfaBridge - builds the plate + eccentric beam model of a girder bridge
// from a neutral description, computes influence surfaces for girder
// actions, places HL-93 on them and derives live load distribution
// factors (refined analysis per AASHTO LRFD 4.6.3).
//
// Independent of PGSuper: the agent's adapter fills BridgeInput.
// Plan coordinates (x, y) are any right-handed Cartesian system; the
// roadway system is (station s, offset o) with o increasing to the right
// when looking ahead on station.
///////////////////////////////////////////////////////////////////////
#pragma once
#include "RfaFem.h"
#include "RfaLive.h"
#include <functional>
#include <memory>

namespace rfa
{
   struct P2 { double x = 0, y = 0; };

   // Girder section properties at a point along the girder
   struct GirderSection
   {
      double E = 0, A = 0, Iv = 0, Il = 0, J = 0;
      double ecc = 0;   // deck mid-surface down to girder centroid
      double yb = 0;    // girder centroid to bottom of girder
      double height = 0;
   };

   struct GirderLine
   {
      // plan points along the girder (PGSuper GetSegmentEndPoints)
      P2 pier1, end1, brg1, brg2, end2, pier2;
      // section at fraction f along brg1->brg2 (f may be < 0 or > 1 for the girder ends)
      std::function<GirderSection(double f)> section;
   };

   struct Diaphragm
   {
      double f = 0;          // fraction along brg1->brg2 (same for all girders of the span)
      int gFrom = 0, gTo = 0;// girders spanned (gTo = gFrom + 1 for one bay)
      double E = 0, width = 0, height = 0; // rectangular section (width along girder)
      double ecc = 0;        // deck mid-surface down to diaphragm centroid
   };

   struct SpanInput
   {
      std::vector<GirderLine> girders;   // left to right
      double deckE = 0, deckNu = 0.2, deckT = 0;
      std::vector<Diaphragm> diaphragms; // intermediate
      double endDiaphragmW[2] = { 0,0 }, endDiaphragmH[2] = { 0,0 }, endDiaphragmE = 0; // at start/end supports
   };

   enum class SupportKind { Abutment, SimplePier, ContinuousPier };

   struct PierInput
   {
      SupportKind kind = SupportKind::Abutment;
      bool integral[2] = { false,false }; // rotation fixed: [back side, ahead side]
      bool deckContinuous = false;        // deck nodes shared across a simple pier (link slab)
   };

   struct Alignment
   {
      // point on the alignment at station s and the unit normal toward +offset
      std::function<void(double s, P2& pt, P2& normal)> eval;
      // station/offset of a plan point
      std::function<void(const P2& p, double& s, double& o)> inverse;
   };

   struct BridgeInput
   {
      std::vector<SpanInput> spans;
      std::vector<PierInput> piers;  // spans.size() + 1
      Alignment alignment;
      // deck edges and curb lines (offsets) as functions of station
      std::function<double(double s)> leftDeckEdge, rightDeckEdge, leftCurb, rightCurb;
   };

   struct MeshOptions
   {
      double transverseSize = 0.45;  // target element size across the deck (m)
      double longitudinalSize = 0.75;// target element size along the girders (m)
      int minPerBay = 4;
      double gridStep = 0.1524;      // influence surface sampling (0.5 ft)
      bool includePlateInGirderActions = true; // tributary deck strip in girder M and V
      double bearingSpringFactor = 100.0;      // horizontal bearing stiffness = factor * EA/L
   };

   // One response location on a girder
   enum class Action { MomentPos, MomentNeg, Shear, Reaction };

   struct StationResult
   {
      int span = 0, girder = 0;
      double f = 0;              // fraction along brg1->brg2 (pier: < 0 or > 1)
      std::string label;         // "0.5L", "Pier 2", "Shear (start)", ...
      Action action = Action::MomentPos;
      // FE governing (HL-93, MPF, IM), line-girder one lane (IM), ratio
      double fe = 0, line = 0, g = 0;
      double feOneLaneNoMpf = 0, gFatigue = 0; // one lane, MPF removed
      LiveResult governing;                    // placement details
      int surfaceId = -1;                      // index into Results::surfaces (if kept)
   };

   struct GirderLLDF
   {
      int span = 0, girder = 0;
      double gMpos = 0, gMneg = 0, gV = 0;     // Strength
      double gMposF = 0, gMnegF = 0, gVF = 0;  // Fatigue (one lane, no MPF)
   };
   struct PierLLDF { int pier = 0, girder = 0; double gMneg = 0, gMnegF = 0; };

   struct Results
   {
      bool ok = false;
      std::string error;
      std::vector<std::string> notes;
      int nodes = 0, shells = 0, beams = 0, equations = 0;
      double roadLeft = 0, roadRight = 0; int designLanes = 0;
      std::vector<StationResult> stations;
      std::vector<GirderLLDF> girders;
      std::vector<PierLLDF> piers;
      // kept surfaces (for plots): grid + label
      struct Surface { std::string label; Grid2D grid; };
      std::vector<Surface> surfaces;
      // mesh for plots (plan)
      std::vector<P2> meshNodes;
      std::vector<std::array<int, 4>> meshShells;
      std::vector<std::array<int, 2>> meshBeams;
      // girder lines in roadway coordinates (s, o): [span][girder] -> brg1, brg2
      struct GirderSO { int span = 0, girder = 0; double s1 = 0, o1 = 0, s2 = 0, o2 = 0; };
      std::vector<GirderSO> girderLines;
      double seconds = 0;
   };

   struct AnalysisOptions
   {
      MeshOptions mesh;
      HL93 live;
      bool keepSurfaces = true;     // keep the governing positive-moment surface per girder/span
      bool stationRatio = false;    // LLDF = largest station ratio instead of ratio of envelope maxima
      std::function<void(const std::string&)> progress;
   };

   Results RunRefinedAnalysis(const BridgeInput& in, const AnalysisOptions& opt);
}
