// Refined live load analysis for PGSuper: adapter, cache, JSON, LLDF application and report.
#include "stdafx.h"
#include "RefinedLL.h"
#include "ExportUtil.h"

#include "..\RefinedAnalysisAgent\fem\RfaBridge.h"

#include <IFace\Bridge.h>
#include <IFace\Alignment.h>
#include <IFace\Intervals.h>
#include <IFace\PointOfInterest.h>
#include <IFace\Project.h>
#include <IFace\DistributionFactors.h>
#include <PsgLib\BridgeDescription2.h>
#include <PsgLib\PierData2.h>
#include <PsgLib\SpanData2.h>
#include <EAF\EAFDisplayUnits.h>
#include <EAF\EAFReportManager.h>
#include <EAF\EAFUtilities.h>
#include <Reporting\BrokerReportSpecification.h>
#include <Reporting\BrokerReportSpecificationBuilder.h>
#include <Reporting\PGSuperTitlePageBuilder.h>
#include <Reporter\Reporter.h>

#include <atlimage.h>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <map>

#undef min
#undef max

namespace
{
   rfa::P2 P(IPoint2d* p) { Float64 x, y; p->get_X(&x); p->get_Y(&y); return { x, y }; }

   std::string GirderName(int g) { return std::string(1, (char)('A' + g)); }

   struct Cache
   {
      std::mutex lock;
      std::string key;
      std::shared_ptr<const rfa::Results> results;
   } g_Cache;

   // Cheap fingerprint of the analysis inputs (positions, sections, deck, roadway, piers)
   std::string Fingerprint(const rfa::BridgeInput& in)
   {
      std::ostringstream os;
      os.precision(9);
      for (auto& sp : in.spans)
      {
         os << "S" << sp.deckE << "," << sp.deckT << "," << sp.diaphragms.size() << "," << sp.endDiaphragmW[0] << sp.endDiaphragmH[0] << sp.endDiaphragmW[1] << sp.endDiaphragmH[1] << ";";
         for (auto& g : sp.girders)
         {
            os << g.end1.x << "," << g.end1.y << "," << g.brg1.x << "," << g.brg1.y << "," << g.brg2.x << "," << g.brg2.y << "," << g.end2.x << "," << g.end2.y << ",";
            for (double f : { 0.0, 0.5, 1.0 })
            {
               rfa::GirderSection s = g.section(f);
               os << s.E << "," << s.A << "," << s.Iv << "," << s.Il << "," << s.ecc << ",";
            }
         }
      }
      for (auto& p : in.piers) os << (int)p.kind << p.integral[0] << p.integral[1] << ";";
      for (double s : { 0.0, 50.0, 100.0, 200.0 }) os << in.leftCurb(s) << "," << in.rightCurb(s) << "," << in.leftDeckEdge(s) << "," << in.rightDeckEdge(s) << ";";
      return os.str();
   }
}

////////////////////////////////////////////////////////////////////////
// Adapter
////////////////////////////////////////////////////////////////////////
bool RefinedLL_BuildInput(std::shared_ptr<WBFL::EAF::Broker> pBroker, rfa::BridgeInput& in, std::vector<std::string>& notes, std::string& error)
{
   GET_IFACE2(pBroker, IBridge, pBridge);
   GET_IFACE2(pBroker, IGirder, pGirder);
   GET_IFACE2(pBroker, ISectionProperties, pSectProp);
   GET_IFACE2(pBroker, IMaterials, pMaterials);
   GET_IFACE2(pBroker, IIntervals, pIntervals);
   GET_IFACE2(pBroker, IPointOfInterest, pPoi);
   GET_IFACE2(pBroker, IRoadway, pRoadway);

   pgsTypes::SupportedDeckType deck = pBridge->GetDeckType();
   if (deck != pgsTypes::sdtCompositeCIP && deck != pgsTypes::sdtCompositeSIP)
   {
      error = "The refined analysis models a composite structural deck slab (cast-in-place or stay-in-place panels). "
              "This bridge has no structural deck slab (adjacent members / overlay), which needs a different model (shear-key hinged plates); not supported.";
      return false;
   }
   SpanIndexType nSpans = pBridge->GetSpanCount();
   if (pBridge->GetGirderGroupCount() != nSpans)
   {
      error = "Girder groups do not match spans (spliced girder layout); the refined analysis supports PGSuper precast girder bridges.";
      return false;
   }

   IntervalIndexType llInterval = pIntervals->GetLiveLoadInterval();
   pgsTypes::HaunchAnalysisSectionPropertiesType haunchType = pSectProp->GetHaunchAnalysisSectionPropertiesType();
   const Float64 Ed = pMaterials->GetDeckEc(0, llInterval);

   in.spans.clear();
   in.piers.clear();
   for (SpanIndexType spanIdx = 0; spanIdx < nSpans; spanIdx++)
   {
      rfa::SpanInput sp;
      GroupIndexType grpIdx = (GroupIndexType)spanIdx;
      GirderIndexType nG = pBridge->GetGirderCount(grpIdx);
      sp.deckE = Ed;
      sp.deckNu = 0.2;
      {
         CSegmentKey key(grpIdx, nG / 2, 0);
         Float64 Ls = pBridge->GetSegmentLength(key);
         pgsPointOfInterest poi = pPoi->GetPointOfInterest(key, 0.5 * Ls);
         sp.deckT = pBridge->GetStructuralSlabDepth(poi);
      }
      for (GirderIndexType g = 0; g < nG; g++)
      {
         CSegmentKey segKey(grpIdx, g, 0);
         if (pBridge->GetSegmentCount(CGirderKey(grpIdx, g)) != 1)
         {
            error = "Girder " + GirderName((int)g) + " of span " + std::to_string(spanIdx + 1) + " has more than one segment; not supported.";
            return false;
         }
         CComPtr<IPoint2d> p1, e1, b1, b2, e2, p2;
         pGirder->GetSegmentEndPoints(segKey, pgsTypes::pcGlobal, &p1, &e1, &b1, &b2, &e2, &p2);
         rfa::GirderLine gl;
         gl.pier1 = P(p1); gl.end1 = P(e1); gl.brg1 = P(b1); gl.brg2 = P(b2); gl.end2 = P(e2); gl.pier2 = P(p2);

         Float64 Lseg = pBridge->GetSegmentLength(segKey);
         Float64 ed1 = pBridge->GetSegmentStartEndDistance(segKey);
         Float64 ed2 = pBridge->GetSegmentEndEndDistance(segKey);
         Float64 Eg = pMaterials->GetSegmentEc(segKey, llInterval);
         Float64 ts = sp.deckT;
         // sample the section at 21 points and interpolate (sections are queried per mesh row)
         struct Sample { double f; rfa::GirderSection s; };
         auto samples = std::make_shared<std::vector<Sample>>();
         for (int k = 0; k <= 20; k++)
         {
            double f = k / 20.0;
            Float64 X = std::clamp(ed1 + f * (Lseg - ed1 - ed2), 0.0, Lseg);
            pgsPointOfInterest poi = pPoi->GetPointOfInterest(segKey, X);
            rfa::GirderSection s;
            s.E = Eg;
            s.A = pSectProp->GetAg(pgsTypes::sptGrossNoncomposite, llInterval, poi);
            s.Iv = pSectProp->GetIxx(pgsTypes::sptGrossNoncomposite, llInterval, poi);
            s.Il = pSectProp->GetIyy(pgsTypes::sptGrossNoncomposite, llInterval, poi);
            s.J = s.A * s.A * s.A * s.A / (40.0 * (s.Iv + s.Il)); // AASHTO Eq. 4.6.2.2.1-3 (open sections)
            s.height = pSectProp->GetHg(pgsTypes::sptGrossNoncomposite, llInterval, poi);
            Float64 yt = fabs(pSectProp->GetY(pgsTypes::sptGrossNoncomposite, llInterval, poi, pgsTypes::TopGirder));
            s.yb = s.height - yt;
            Float64 haunch = pSectProp->GetStructuralHaunchDepth(poi, haunchType);
            s.ecc = ts / 2 + haunch + yt;
            samples->push_back({ f, s });
         }
         gl.section = [samples](double f)
         {
            const auto& v = *samples;
            if (f <= v.front().f) return v.front().s;
            if (f >= v.back().f) return v.back().s;
            size_t k = std::min((size_t)(f * 20), v.size() - 2);
            double t = (f - v[k].f) / (v[k + 1].f - v[k].f);
            const rfa::GirderSection& a = v[k].s; const rfa::GirderSection& b = v[k + 1].s;
            rfa::GirderSection s;
            s.E = a.E + t * (b.E - a.E); s.A = a.A + t * (b.A - a.A); s.Iv = a.Iv + t * (b.Iv - a.Iv); s.Il = a.Il + t * (b.Il - a.Il);
            s.J = a.J + t * (b.J - a.J); s.ecc = a.ecc + t * (b.ecc - a.ecc); s.yb = a.yb + t * (b.yb - a.yb); s.height = a.height + t * (b.height - a.height);
            return s;
         };
         sp.girders.push_back(gl);

         // intermediate diaphragms between this girder and the next
         if (g + 1 < nG)
         {
            Float64 Lbrg = Lseg - ed1 - ed2;
            rfa::GirderSection mid = gl.section(0.5);
            for (const auto& d : pBridge->GetPrecastDiaphragms(segKey))
            {
               if (d.H <= 0 || d.T <= 0) { notes.push_back("Span " + std::to_string(spanIdx + 1) + ": a precast diaphragm defined by weight only is not included in the stiffness"); continue; }
               rfa::Diaphragm dd;
               dd.f = (d.Location - ed1) / Lbrg; dd.gFrom = (int)g; dd.gTo = (int)g + 1;
               dd.E = Eg; dd.width = d.T; dd.height = d.H;
               dd.ecc = mid.ecc - (mid.height - mid.yb) + d.H / 2; // top of diaphragm at top of girder
               sp.diaphragms.push_back(dd);
            }
            for (const auto& d : pBridge->GetCastInPlaceDiaphragms(CSpanKey(spanIdx, g)))
            {
               if (d.H <= 0 || d.T <= 0) { notes.push_back("Span " + std::to_string(spanIdx + 1) + ": a cast-in-place diaphragm defined by weight only is not included in the stiffness"); continue; }
               rfa::Diaphragm dd;
               // location from the left end of the span (CL pier) -> fraction along the bearings
               Float64 dPierBrg = std::hypot(gl.brg1.x - gl.pier1.x, gl.brg1.y - gl.pier1.y);
               dd.f = (d.Location - dPierBrg) / Lbrg; dd.gFrom = (int)g; dd.gTo = (int)g + 1;
               dd.E = Ed; dd.width = d.T; dd.height = d.H;
               dd.ecc = ts / 2 + d.H / 2; // hangs from the deck
               sp.diaphragms.push_back(dd);
            }
         }
      }
      // end (pier) diaphragms
      for (int end = 0; end < 2; end++)
      {
         Float64 W = 0, H = 0;
         pBridge->GetPierDiaphragmSize((PierIndexType)(spanIdx + end), end == 0 ? pgsTypes::Ahead : pgsTypes::Back, &W, &H);
         sp.endDiaphragmW[end] = W; sp.endDiaphragmH[end] = H;
      }
      sp.endDiaphragmE = Ed;
      in.spans.push_back(std::move(sp));
   }

   PierIndexType nPiers = pBridge->GetPierCount();
   for (PierIndexType p = 0; p < nPiers; p++)
   {
      rfa::PierInput pi;
      pgsTypes::BoundaryConditionType bc = pBridge->GetBoundaryConditionType(p);
      bool integralBack = false, integralAhead = false;
      switch (bc)
      {
      case pgsTypes::bctIntegralAfterDeck: case pgsTypes::bctIntegralBeforeDeck: integralBack = integralAhead = true; break;
      case pgsTypes::bctIntegralAfterDeckHingeBack: case pgsTypes::bctIntegralBeforeDeckHingeBack: integralAhead = true; break;
      case pgsTypes::bctIntegralAfterDeckHingeAhead: case pgsTypes::bctIntegralBeforeDeckHingeAhead: integralBack = true; break;
      default: break;
      }
      if (p == 0 || p == nPiers - 1)
         pi.kind = rfa::SupportKind::Abutment;
      else
      {
         // girders continuous through the pier for live load (continuity diaphragm, or integral
         // on both sides) - PGSuper's IsContinuousBoundaryCondition back and ahead
         bool bBack = IsContinuousBoundaryConditionBackSide(bc) && !(bc == pgsTypes::bctIntegralAfterDeckHingeAhead || bc == pgsTypes::bctIntegralBeforeDeckHingeAhead);
         bool bAhead = IsContinuousBoundaryConditionAheadSide(bc) && !(bc == pgsTypes::bctIntegralAfterDeckHingeBack || bc == pgsTypes::bctIntegralBeforeDeckHingeBack);
         pi.kind = (bBack && bAhead) ? rfa::SupportKind::ContinuousPier : rfa::SupportKind::SimplePier;
      }
      pi.integral[0] = integralBack;
      pi.integral[1] = integralAhead;
      pi.deckContinuous = false; // PGSuper analyses simple spans as simple for live load
      in.piers.push_back(pi);
   }

   // Alignment (station, offset) <-> plan. The offset direction is checked against the
   // inverse so that +offset matches PGSuper's offsets (curb and deck edge offsets).
   std::shared_ptr<IRoadway> road = pRoadway;
   in.alignment.eval = [road](double s, rfa::P2& pt, rfa::P2& n)
   {
      CComPtr<IDirection> normal;
      road->GetBearingNormal(s, &normal);
      CComPtr<IPoint2d> a, b;
      road->GetPoint(s, 0.0, normal, pgsTypes::pcGlobal, &a);
      road->GetPoint(s, 1.0, normal, pgsTypes::pcGlobal, &b);
      pt = P(a);
      rfa::P2 q = P(b);
      Float64 ss, oo;
      road->GetStationAndOffset(pgsTypes::pcGlobal, b, &ss, &oo);
      double sg = oo >= 0 ? 1.0 : -1.0;
      n = { sg * (q.x - pt.x), sg * (q.y - pt.y) };
   };
   in.alignment.inverse = [road](const rfa::P2& p, double& s, double& o)
   {
      CComPtr<IPoint2d> pt;
      pt.CoCreateInstance(CLSID_Point2d);
      pt->Move(p.x, p.y);
      road->GetStationAndOffset(pgsTypes::pcGlobal, pt, &s, &o);
   };
   std::shared_ptr<IBridge> bridge = pBridge;
   std::shared_ptr<IPointOfInterest> poiIf = pPoi;
   auto Xb = [poiIf](double s) { return poiIf->ConvertRouteToBridgeLineCoordinate(s); };
   in.leftDeckEdge = [bridge, Xb](double s) { return bridge->GetLeftSlabEdgeOffset(Xb(s)); };
   in.rightDeckEdge = [bridge, Xb](double s) { return bridge->GetRightSlabEdgeOffset(Xb(s)); };
   in.leftCurb = [bridge, Xb](double s) { return bridge->GetLeftCurbOffset(Xb(s)); };
   in.rightCurb = [bridge, Xb](double s) { return bridge->GetRightCurbOffset(Xb(s)); };
   return true;
}

////////////////////////////////////////////////////////////////////////
// Run (cached)
////////////////////////////////////////////////////////////////////////
std::shared_ptr<const rfa::Results> RefinedLL_Run(std::shared_ptr<WBFL::EAF::Broker> pBroker, bool bForce)
{
   rfa::BridgeInput in;
   std::vector<std::string> notes;
   std::string error;
   auto r = std::make_shared<rfa::Results>();
   try
   {
      if (!RefinedLL_BuildInput(pBroker, in, notes, error))
      {
         r->error = error;
         return r;
      }
      std::string key = Fingerprint(in);
      {
         std::lock_guard<std::mutex> lk(g_Cache.lock);
         if (!bForce && g_Cache.results && g_Cache.key == key) return g_Cache.results;
      }
      rfa::AnalysisOptions opt;
      opt.progress = [](const std::string& s) { Trace(CA2W(("refined LL: " + s).c_str()).m_psz); };
      *r = rfa::RunRefinedAnalysis(in, opt);
      r->notes.insert(r->notes.begin(), notes.begin(), notes.end());
      std::lock_guard<std::mutex> lk(g_Cache.lock);
      g_Cache.key = key;
      g_Cache.results = r;
   }
   catch (const WBFL::System::XBase& e)
   {
      r->ok = false;
      r->error = std::string(CW2A(Describe(e).c_str()));
   }
   catch (const std::exception& e)
   {
      r->ok = false;
      r->error = e.what();
   }
   return r;
}

////////////////////////////////////////////////////////////////////////
// JSON
////////////////////////////////////////////////////////////////////////
namespace
{
   const char* ActionName(rfa::Action a)
   {
      switch (a)
      {
      case rfa::Action::MomentPos: return "+M";
      case rfa::Action::MomentNeg: return "-M";
      case rfa::Action::Shear: return "V";
      default: return "R";
      }
   }
   std::string J(double v) { std::ostringstream os; os.precision(9); os << v; return os.str(); }
   std::string Q(const std::string& s) { return JsonString(std::wstring(CA2W(s.c_str()).m_psz)); }
}

std::string RefinedLL_Json(std::shared_ptr<WBFL::EAF::Broker> pBroker, const rfa::Results& r)
{
   std::ostringstream os;
   os << "{\n  \"ok\": " << (r.ok ? "true" : "false");
   if (!r.ok) { os << ",\n  \"error\": " << Q(r.error) << "\n}\n"; return os.str(); }
   os << ",\n  \"units\": \"SI (N, m)\"";
   os << ",\n  \"model\": {\"nodes\": " << r.nodes << ", \"shells\": " << r.shells << ", \"beams\": " << r.beams << ", \"equations\": " << r.equations
      << ", \"design_lanes\": " << r.designLanes << ", \"roadway_left_m\": " << J(r.roadLeft) << ", \"roadway_right_m\": " << J(r.roadRight) << ", \"seconds\": " << J(r.seconds) << "}";
   os << ",\n  \"notes\": [";
   for (size_t i = 0; i < r.notes.size(); i++) os << (i ? ", " : "") << Q(r.notes[i]);
   os << "]";

   // current (PGSuper) factors for comparison
   GET_IFACE2(pBroker, ILiveLoadDistributionFactors, pLLDF);
   os << ",\n  \"girders\": [";
   for (size_t i = 0; i < r.girders.size(); i++)
   {
      const auto& g = r.girders[i];
      CSpanKey key((SpanIndexType)g.span, (GirderIndexType)g.girder);
      double cM = 0, cN = 0, cV = 0;
      try
      {
         cM = pLLDF->GetMomentDistFactor(key, pgsTypes::StrengthI);
         cN = pLLDF->GetNegMomentDistFactor(key, pgsTypes::StrengthI);
         cV = pLLDF->GetShearDistFactor(key, pgsTypes::StrengthI);
      }
      catch (...) {}
      os << (i ? ",\n    " : "\n    ") << "{\"span\": " << g.span + 1 << ", \"girder\": \"" << GirderName(g.girder) << "\""
         << ", \"gM_pos\": " << J(g.gMpos) << ", \"gM_neg\": " << J(g.gMneg) << ", \"gV\": " << J(g.gV)
         << ", \"fatigue_gM_pos\": " << J(g.gMposF) << ", \"fatigue_gM_neg\": " << J(g.gMnegF) << ", \"fatigue_gV\": " << J(g.gVF)
         << ", \"current_gM_pos\": " << J(cM) << ", \"current_gM_neg\": " << J(cN) << ", \"current_gV\": " << J(cV) << "}";
   }
   os << "\n  ]";
   os << ",\n  \"piers\": [";
   for (size_t i = 0; i < r.piers.size(); i++)
   {
      const auto& p = r.piers[i];
      os << (i ? ", " : "") << "{\"pier\": " << p.pier + 1 << ", \"girder\": \"" << GirderName(p.girder) << "\", \"gM_neg\": " << J(p.gMneg) << ", \"fatigue_gM_neg\": " << J(p.gMnegF) << "}";
   }
   os << "]";
   os << ",\n  \"stations\": [";
   for (size_t i = 0; i < r.stations.size(); i++)
   {
      const auto& s = r.stations[i];
      os << (i ? ",\n    " : "\n    ") << "{\"span\": " << s.span + 1 << ", \"girder\": \"" << GirderName(s.girder) << "\", \"location\": " << Q(s.label)
         << ", \"f\": " << J(s.f) << ", \"action\": \"" << ActionName(s.action) << "\", \"fe\": " << J(s.fe) << ", \"line_girder\": " << J(s.line)
         << ", \"g\": " << J(s.g) << ", \"fe_one_lane_no_mpf\": " << J(s.feOneLaneNoMpf) << ", \"g_fatigue\": " << J(s.gFatigue)
         << ", \"lanes\": " << s.governing.lanes << ", \"placement\": [";
      for (size_t k = 0; k < s.governing.placement.size(); k++)
      {
         const auto& p = s.governing.placement[k];
         os << (k ? ", " : "") << "{\"lane_left_m\": " << J(p.laneLeft) << ", \"vehicle\": " << Q(rfa::VehicleName(p.vehicle))
            << ", \"vehicle_offset_m\": " << J(p.vehicleCentre) << ", \"vehicle_station_m\": " << J(p.vehicleS) << ", \"rear_spacing_m\": " << J(p.rearSpacing)
            << ", \"direction\": " << p.direction << ", \"lane_strip_left_m\": " << J(p.laneStripLeft) << ", \"vehicle_effect\": " << J(p.vehicleEffect) << ", \"lane_effect\": " << J(p.laneEffect) << "}";
      }
      os << "]}";
   }
   os << "\n  ]\n}\n";
   return os.str();
}

////////////////////////////////////////////////////////////////////////
// Apply as directly input LLDFs
////////////////////////////////////////////////////////////////////////
bool RefinedLL_Apply(std::shared_ptr<WBFL::EAF::Broker> pBroker, const rfa::Results& r, std::wstring& message)
{
   if (!r.ok) { message = L"refined analysis failed: " + std::wstring(CA2W(r.error.c_str()).m_psz); return false; }
   GET_IFACE2(pBroker, IBridgeDescription, pBD);
   CBridgeDescription2 desc(*pBD->GetBridgeDescription());
   desc.SetDistributionFactorMethod(pgsTypes::DirectlyInput);
   for (const auto& g : r.girders)
   {
      CSpanData2* pSpan = desc.GetSpan((SpanIndexType)g.span);
      pSpan->SetLLDFPosMoment((GirderIndexType)g.girder, pgsTypes::StrengthI, g.gMpos);
      pSpan->SetLLDFPosMoment((GirderIndexType)g.girder, pgsTypes::FatigueI, g.gMposF);
      pSpan->SetLLDFNegMoment((GirderIndexType)g.girder, pgsTypes::StrengthI, g.gMneg);
      pSpan->SetLLDFNegMoment((GirderIndexType)g.girder, pgsTypes::FatigueI, g.gMnegF);
      pSpan->SetLLDFShear((GirderIndexType)g.girder, pgsTypes::StrengthI, g.gV);
      pSpan->SetLLDFShear((GirderIndexType)g.girder, pgsTypes::FatigueI, g.gVF);
   }
   for (const auto& p : r.piers)
   {
      CPierData2* pPier = desc.GetPier((PierIndexType)p.pier);
      pPier->SetLLDFNegMoment((GirderIndexType)p.girder, pgsTypes::StrengthI, p.gMneg);
      pPier->SetLLDFNegMoment((GirderIndexType)p.girder, pgsTypes::FatigueI, p.gMnegF);
   }
   pBD->SetBridgeDescription(desc);
   std::wostringstream os;
   os << L"live load distribution factors set to Directly Input from the refined analysis (" << r.girders.size() << L" girders, " << r.piers.size() << L" pier/girder values)";
   message = os.str();
   return true;
}

////////////////////////////////////////////////////////////////////////
// Report
////////////////////////////////////////////////////////////////////////
namespace
{
   // Influence surface image in roadway coordinates with the governing placement drawn on top
   std::wstring DrawSurface(const rfa::Results& r, const rfa::Results::Surface& srf, const rfa::StationResult* st, const std::wstring& file)
   {
      const rfa::Grid2D& g = srf.grid;
      const int W = 1000;
      double sLen = (g.ns - 1) * g.ds, oLen = (g.no - 1) * g.dO;
      int H = std::clamp((int)(W * oLen / std::max(sLen, 1e-9)), 150, 340);
      const int top = 22, left = 6, plotH = H, plotW = W - 2 * left;
      CImage img;
      if (!img.Create(W, H + top + 26, 24)) return L"";
      HDC dc = img.GetDC();
      ::SetBkMode(dc, TRANSPARENT);
      RECT all = { 0, 0, W, H + top + 26 };
      ::FillRect(dc, &all, (HBRUSH)::GetStockObject(WHITE_BRUSH));
      double vmax = 0;
      for (double v : g.v) vmax = std::max(vmax, fabs(v));
      if (vmax <= 0) vmax = 1;
      auto sx = [&](double s) { return left + (int)std::lround((s - g.s0) / std::max(sLen, 1e-9) * (plotW - 1)); };
      auto oy = [&](double o) { return top + (int)std::lround((o - g.o0) / std::max(oLen, 1e-9) * (plotH - 1)); };
      for (int py = 0; py < plotH; py++)
      {
         int j = std::clamp((int)std::lround((double)py / (plotH - 1) * (g.no - 1)), 0, g.no - 1);
         for (int px = 0; px < plotW; px++)
         {
            int k = std::clamp((int)std::lround((double)px / (plotW - 1) * (g.ns - 1)), 0, g.ns - 1);
            double t = std::clamp(g.at(k, j) / vmax, -1.0, 1.0);
            // diverging: blue (-) / white / red (+)
            int R, G, B;
            if (t >= 0) { R = 255 - (int)(t * (255 - 178)); G = 255 - (int)(t * (255 - 24)); B = 255 - (int)(t * (255 - 43)); }
            else { double u = -t; R = 255 - (int)(u * (255 - 33)); G = 255 - (int)(u * (255 - 102)); B = 255 - (int)(u * (255 - 172)); }
            img.SetPixelRGB(left + px, top + py, (BYTE)R, (BYTE)G, (BYTE)B);
         }
      }
      // girder lines
      HPEN gpen = ::CreatePen(PS_SOLID, 1, RGB(90, 90, 90));
      HGDIOBJ old = ::SelectObject(dc, gpen);
      for (auto& gl : r.girderLines)
      {
         ::MoveToEx(dc, sx(gl.s1), oy(gl.o1), nullptr);
         ::LineTo(dc, sx(gl.s2), oy(gl.o2));
      }
      ::SelectObject(dc, old); ::DeleteObject(gpen);
      // governing placement
      if (st)
      {
         HPEN lpen = ::CreatePen(PS_DOT, 1, RGB(0, 0, 0));
         HBRUSH wheel = ::CreateSolidBrush(RGB(0, 0, 0));
         old = ::SelectObject(dc, lpen);
         const double ft = 0.3048;
         for (auto& p : st->governing.placement)
         {
            // lane outline
            ::SelectObject(dc, ::GetStockObject(NULL_BRUSH));
            ::Rectangle(dc, sx(g.s0), oy(p.laneLeft), sx(g.s0 + sLen) + 1, oy(p.laneLeft + 12 * ft) + 1);
            // wheels
            std::vector<double> axles;
            auto truck = [&](double s0, double rear) { axles.push_back(s0); axles.push_back(s0 + p.direction * 14 * ft); axles.push_back(s0 + p.direction * (14 * ft + rear)); };
            if (p.vehicle == rfa::Vehicle::Truck) truck(p.vehicleS, p.rearSpacing);
            else if (p.vehicle == rfa::Vehicle::Tandem) { axles.push_back(p.vehicleS); axles.push_back(p.vehicleS + 4 * ft); }
            else if (p.vehicle == rfa::Vehicle::TwoTrucks) { truck(p.vehicleS, 14 * ft); truck(p.secondS, 14 * ft); }
            HGDIOBJ ob = ::SelectObject(dc, wheel);
            for (double s : axles)
               for (double w : { -3 * ft, 3 * ft })
               {
                  int x = sx(s), y = oy(p.vehicleCentre + w);
                  ::Ellipse(dc, x - 4, y - 4, x + 5, y + 5);
               }
            ::SelectObject(dc, ob);
         }
         ::SelectObject(dc, old);
         ::DeleteObject(lpen); ::DeleteObject(wheel);
      }
      // titles
      HFONT font = ::CreateFontW(15, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");
      HGDIOBJ of = ::SelectObject(dc, font);
      std::wstring title = std::wstring(CA2W(srf.label.c_str()).m_psz) + L"   (influence surface; red = increases the effect, blue = decreases; dots = governing wheels, dotted = design lanes)";
      ::TextOutW(dc, left, 3, title.c_str(), (int)title.size());
      std::wstring foot = L"station along alignment ->   |   offset increases downward (curb to curb)";
      ::TextOutW(dc, left, top + plotH + 5, foot.c_str(), (int)foot.size());
      ::SelectObject(dc, of); ::DeleteObject(font);
      img.ReleaseDC();
      if (FAILED(img.Save(file.c_str(), Gdiplus::ImageFormatPNG))) return L"";
      return file;
   }

   std::wstring W(const std::string& s) { return std::wstring(CA2W(s.c_str()).m_psz); }
   std::wstring F3(double v) { std::wostringstream os; os.setf(std::ios::fixed); os.precision(3); os << v; return os.str(); }
   std::wstring F1(double v) { std::wostringstream os; os.setf(std::ios::fixed); os.precision(1); os << v; return os.str(); }
}

rptChapter* CRefinedLLChapterBuilder::Build(const std::shared_ptr<const WBFL::Reporting::ReportSpecification>& pRptSpec, Uint16 level) const
{
   rptChapter* pChapter = CPGSuperChapterBuilder::Build(pRptSpec, level);
   auto pBrokerSpec = std::dynamic_pointer_cast<const CBrokerReportSpecification>(pRptSpec);
   auto pBroker = pBrokerSpec->GetBroker();
   GET_IFACE2(pBroker, IEAFDisplayUnits, pDisplayUnits);
   GET_IFACE2(pBroker, ILiveLoadDistributionFactors, pLLDF);

   rptParagraph* p = new rptParagraph(rptStyleManager::GetHeadingStyle());
   *pChapter << p;
   *p << _T("Refined Live Load Analysis (AASHTO LRFD 4.6.3): plate and eccentric beam model with influence surfaces") << rptNewLine;

   auto r = RefinedLL_Run(pBroker);
   p = new rptParagraph;
   *pChapter << p;
   if (!r->ok)
   {
      *p << color(Red) << _T("The refined analysis could not be performed: ") << W(r->error).c_str() << color(Black) << rptNewLine;
      return pChapter;
   }

   *p << _T("The deck is modelled with four-node shell elements (membrane with incompatible modes + MITC4 Mindlin plate) at the mid-depth of the structural slab. ")
      << _T("Each girder is a beam element at its centroid, connected to the deck by rigid links (eccentricity = half the structural slab + structural haunch + girder top to centroid), so composite action comes from the model. ")
      << _T("Intermediate and end diaphragms are beam elements. Girders are supported vertically at their bearings (at the pier centreline where the girders are continuous); horizontal restraint is statically determinate, matching the pin/roller line-girder idealisation.") << rptNewLine << rptNewLine;
   *p << _T("For every girder, the influence surface of each action (moment at tenth points, shear at the critical sections, reactions, negative moment over continuous piers) is computed with one adjoint solution and searched for the governing HL-93 placement: ")
      << _T("design lanes anywhere across the roadway (number per 3.6.1.1.1), design truck with 14 to 30 ft rear spacing or design tandem anywhere in each lane (wheels at least 2 ft from the lane edge), the 10 ft lane load where it increases the effect, IM = 33% on truck and tandem, ")
      << _T("multiple presence factors (3.6.1.1.2), and 90% of two trucks plus 90% lane for negative moment and pier reactions (3.6.1.3.1). ")
      << _T("Girder actions are the resultants on the tributary section (girder + deck strip to the mid-bay lines) from element nodal forces, so they are in exact equilibrium with the load.") << rptNewLine << rptNewLine;
   *p << _T("Distribution factor = (governing refined effect) / (one design lane on the line girder, same supports and continuity), taken as the ratio of envelope maxima over the span for moments, at the critical section for shear. ")
      << _T("Fatigue factors use one loaded lane with the multiple presence factor removed (3.6.1.4.3b), computed with HL-93.") << rptNewLine;

   // model summary
   rptRcTable* pTable = rptStyleManager::CreateDefaultTable(2, _T("Model"));
   *p << pTable << rptNewLine;
   int row = 0;
   auto kv = [&](const std::wstring& k, const std::wstring& v) { (*pTable)(row, 0) << k.c_str(); (*pTable)(row, 1) << v.c_str(); row++; };
   (*pTable)(row, 0) << _T("Item"); (*pTable)(row, 1) << _T("Value"); row++;
   kv(L"Nodes / shells / beams", std::to_wstring(r->nodes) + L" / " + std::to_wstring(r->shells) + L" / " + std::to_wstring(r->beams));
   kv(L"Equations", std::to_wstring(r->equations));
   const auto& lenUnit = pDisplayUnits->GetSpanLengthUnit().UnitOfMeasure;
   kv(L"Curb-to-curb width used for lanes", F1(WBFL::Units::ConvertFromSysUnits(r->roadRight - r->roadLeft, lenUnit)) + L" " + lenUnit.UnitTag());
   kv(L"Design lanes", std::to_wstring(r->designLanes));
   kv(L"Run time", F1(r->seconds) + L" s");
   for (auto& n : r->notes) kv(L"Note", W(n));

   // distribution factors
   p = new rptParagraph(rptStyleManager::GetHeadingStyle());
   *pChapter << p;
   *p << _T("Live Load Distribution Factors") << rptNewLine;
   p = new rptParagraph;
   *pChapter << p;
   pTable = rptStyleManager::CreateDefaultTable(12, _T("Refined vs. current (PGSuper) distribution factors"));
   *p << pTable << rptNewLine;
   pTable->SetNumberOfHeaderRows(2);
   const TCHAR* h1[] = { _T("Span"), _T("Girder") };
   for (int c = 0; c < 2; c++) { pTable->SetRowSpan(0, c, 2); (*pTable)(0, c) << h1[c]; }
   pTable->SetColumnSpan(0, 2, 3); (*pTable)(0, 2) << _T("Refined: Strength/Service");
   pTable->SetColumnSpan(0, 5, 3); (*pTable)(0, 5) << _T("Refined: Fatigue");
   pTable->SetColumnSpan(0, 8, 3); (*pTable)(0, 8) << _T("Current: Strength/Service");
   pTable->SetRowSpan(0, 11, 2); (*pTable)(0, 11) << _T("Refined / current +M");
   const TCHAR* h2[] = { _T("+M"), _T("-M"), _T("V") };
   for (int c = 0; c < 9; c++) (*pTable)(1, 2 + c) << h2[c % 3];
   row = 2;
   for (const auto& g : r->girders)
   {
      CSpanKey key((SpanIndexType)g.span, (GirderIndexType)g.girder);
      double cM = 0, cN = 0, cV = 0;
      try { cM = pLLDF->GetMomentDistFactor(key, pgsTypes::StrengthI); cN = pLLDF->GetNegMomentDistFactor(key, pgsTypes::StrengthI); cV = pLLDF->GetShearDistFactor(key, pgsTypes::StrengthI); }
      catch (...) {}
      int c = 0;
      (*pTable)(row, c++) << std::to_wstring(g.span + 1).c_str();
      (*pTable)(row, c++) << W(GirderName(g.girder)).c_str();
      for (double v : { g.gMpos, g.gMneg, g.gV, g.gMposF, g.gMnegF, g.gVF, cM, cN, cV }) (*pTable)(row, c++) << F3(v).c_str();
      (*pTable)(row, c++) << (cM > 0 ? F3(g.gMpos / cM).c_str() : _T("-"));
      row++;
   }
   bool anyPier = false;
   for (auto& pr : r->piers) if (pr.pier > 0 && pr.pier + 1 < (int)(r->piers.back().pier + 1)) anyPier = true;
   if (anyPier)
   {
      pTable = rptStyleManager::CreateDefaultTable(4, _T("Negative moment at piers (refined)"));
      *p << pTable << rptNewLine;
      (*pTable)(0, 0) << _T("Pier"); (*pTable)(0, 1) << _T("Girder"); (*pTable)(0, 2) << _T("-M Strength/Service"); (*pTable)(0, 3) << _T("-M Fatigue");
      row = 1;
      int last = r->piers.back().pier;
      for (auto& pr : r->piers)
      {
         if (pr.pier == 0 || pr.pier == last) continue;
         (*pTable)(row, 0) << std::to_wstring(pr.pier + 1).c_str(); (*pTable)(row, 1) << W(GirderName(pr.girder)).c_str();
         (*pTable)(row, 2) << F3(pr.gMneg).c_str(); (*pTable)(row, 3) << F3(pr.gMnegF).c_str();
         row++;
      }
   }
   *p << _T("To use these factors in PGSuper's analysis, enter them as Directly Input distribution factors (Bridge Description > Live Load Distribution Factors), or apply them with the headless apply_refined_lldf edit.") << rptNewLine;

   // station detail
   const auto& momUnit = pDisplayUnits->GetMomentUnit().UnitOfMeasure;
   const auto& shrUnit = pDisplayUnits->GetShearUnit().UnitOfMeasure;
   p = new rptParagraph(rptStyleManager::GetHeadingStyle());
   *pChapter << p;
   *p << _T("Governing Live Load Effects") << rptNewLine;
   p = new rptParagraph;
   *pChapter << p;
   *p << _T("Effects include IM and multiple presence factors. Line girder = one design lane on a single girder line.") << rptNewLine;
   int curSpan = -1;
   for (size_t k = 0; k < r->stations.size(); k++)
   {
      const auto& s = r->stations[k];
      if (s.span != curSpan)
      {
         curSpan = s.span;
         std::wstring t = L"Span " + std::to_wstring(curSpan + 1);
         pTable = rptStyleManager::CreateDefaultTable(9, t.c_str());
         *p << pTable << rptNewLine;
         const TCHAR* hd[] = { _T("Girder"), _T("Location"), _T("Action"), _T("Refined"), _T("Line girder"), _T("Ratio"), _T("Lanes loaded"), _T("Vehicles"), _T("Units") };
         for (int c = 0; c < 9; c++) (*pTable)(0, c) << hd[c];
         row = 1;
      }
      bool isM = s.action == rfa::Action::MomentPos || s.action == rfa::Action::MomentNeg;
      double feV = isM ? WBFL::Units::ConvertFromSysUnits(s.fe, momUnit) : WBFL::Units::ConvertFromSysUnits(s.fe, shrUnit);
      double lnV = isM ? WBFL::Units::ConvertFromSysUnits(s.line, momUnit) : WBFL::Units::ConvertFromSysUnits(s.line, shrUnit);
      std::wstring tag = isM ? momUnit.UnitTag() : shrUnit.UnitTag();
      (*pTable)(row, 0) << W(GirderName(s.girder)).c_str();
      (*pTable)(row, 1) << W(s.label).c_str();
      (*pTable)(row, 2) << W(ActionName(s.action)).c_str();
      (*pTable)(row, 3) << F1(feV).c_str();
      (*pTable)(row, 4) << F1(lnV).c_str();
      (*pTable)(row, 5) << F3(s.g).c_str();
      (*pTable)(row, 6) << std::to_wstring(s.governing.lanes).c_str();
      std::wstring veh;
      for (auto& pl : s.governing.placement) { if (!veh.empty()) veh += L", "; veh += W(rfa::VehicleName(pl.vehicle)); }
      (*pTable)(row, 7) << veh.c_str();
      (*pTable)(row, 8) << tag.c_str();
      row++;
   }

   // influence surfaces
   p = new rptParagraph(rptStyleManager::GetHeadingStyle());
   *pChapter << p;
   *p << _T("Influence Surfaces and Governing Placements") << rptNewLine;
   p = new rptParagraph;
   *pChapter << p;
   namespace fs = std::filesystem;
   wchar_t tmp[MAX_PATH]; ::GetTempPathW(MAX_PATH, tmp);
   fs::path dir = fs::path(tmp) / L"PGSuperRefinedLL";
   std::error_code ec;
   fs::create_directories(dir, ec);
   static int s_run = 0;
   s_run++;
   for (size_t k = 0; k < r->stations.size(); k++)
   {
      const auto& s = r->stations[k];
      if (s.surfaceId < 0) continue;
      fs::path file = dir / (L"surface_" + std::to_wstring(s_run) + L"_" + std::to_wstring(s.surfaceId) + L".png");
      std::wstring path = DrawSurface(*r, r->surfaces[s.surfaceId], &s, file.wstring());
      if (!path.empty()) *p << rptRcImage(path) << rptNewLine << rptNewLine;
   }
   return pChapter;
}

void RefinedLL_RegisterReport(std::shared_ptr<WBFL::EAF::Broker> pBroker)
{
   GET_IFACE2(pBroker, IEAFReportManager, pRptMgr);
   std::shared_ptr<WBFL::Reporting::ReportSpecificationBuilder> pSpecBuilder(std::make_shared<CBrokerReportSpecificationBuilder>(pBroker));
   auto pRptBuilder = std::make_shared<WBFL::Reporting::ReportBuilder>(_T("Refined Live Load Analysis Report"));
   pRptBuilder->AddTitlePageBuilder(std::shared_ptr<WBFL::Reporting::TitlePageBuilder>(std::make_shared<CPGSuperTitlePageBuilder>(pBroker, pRptBuilder->GetName(), false)));
   pRptBuilder->SetReportSpecificationBuilder(pSpecBuilder);
   pRptBuilder->AddChapterBuilder(std::shared_ptr<WBFL::Reporting::ChapterBuilder>(std::make_shared<CRefinedLLChapterBuilder>()));
   pRptMgr->AddReportBuilder(pRptBuilder);
}
