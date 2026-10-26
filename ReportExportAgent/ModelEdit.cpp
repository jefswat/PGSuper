#include "stdafx.h"
#include "ReportExportAgent.h"
#include "ExportUtil.h"

#include <EAF\EAFDocument.h>
#include <EAF\EAFUtilities.h>
#include <EAF\EAFStatusCenter.h>
#include <IFace\Project.h>
#include <IFace\Bridge.h>
#include <IFace\DistributionFactors.h>
#include <PsgLib\BridgeDescription2.h>
#include <PsgLib\PierData2.h>
#include <PsgLib\SpanData2.h>
#include <PsgLib\GirderGroupData.h>
#include <PsgLib\SplicedGirderData.h>
#include <PsgLib\PrecastSegmentData.h>
#include <PsgLib\GirderSpacing2.h>
#include <PsgLib\ShearData.h>
#include <IFace\Artifact.h>
#include <PgsExt\GirderDesignArtifact.h>
#include <LRFD\RebarPool.h>

#include <algorithm>
#include "RefinedLL.h"
#include "..\RefinedAnalysisAgent\fem\RfaBridge.h"

// Model editing: applies a list of edits to the open model through PGSuper's own
// IBridgeDescription interface (the one its edit dialogs use, so PGSuper keeps dependent
// data consistent), then saves the result as a new .pgs file.
//
// Edits file (written by pgs_modify.py, UTF-8), one edit per line, SI units:
//    <op> TAB key=value TAB key=value ...
// Ops: set_girder_count  count [span]
//      move_pier         pier station option(move_bridge|adjust_prev_span|adjust_next_span|adjust_adjacent_spans)
//      set_span_length   span length
//      set_girder_spacing spacing [pier]
//      set_girder_type   name [span [girder]]
//      set_strands       [span [girder]] straight harped total temporary (any of them)
//      set_concrete      [span [girder]] fc fci (either)
//      set_slab_thickness thickness            (deck gross depth; a constant-depth overhang follows)
//      set_deck_concrete fc
//      set_overhang      overhang | left right  keep(deck_width|spacing)   (span 1 mid-span)
//      set_skew          angle(deg) direction(left|right) [pier]
//      apply_refined_lldf  (no arguments) refined plate/eccentric-beam analysis -> Directly Input LLDFs
//      set_slab_offset   slab_offset            ("A" dimension, whole bridge)
//      design            [span] [girders=1,2,..] flexure(1|0) shear(none|layout|retain)
//                        concrete(min_strength|preserve) haunch(preserve|design)
//                        PGSuper's girder design, applied as Project > Design Girder does
// Spans, piers and girders are 1-based as PGSuper labels them (pier 1 = Abutment 1).
// All edits must succeed or nothing is saved. With no output file nothing is saved: the result
// only describes the model (inspection).

namespace
{
   struct EditError
   {
      std::wstring msg;
   };

   using Args = std::map<std::wstring, std::wstring>;
   struct Edit
   {
      int line;
      std::wstring op;
      Args args;
   };

   std::wstring FromUtf8(const std::string& s)
   {
      if (s.empty()) return std::wstring();
      int n = ::MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), nullptr, 0);
      std::wstring out(n, L'\0');
      ::MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), out.data(), n);
      return out;
   }

   std::vector<Edit> ReadEdits(const std::filesystem::path& file)
   {
      std::ifstream f(file, std::ios::binary);
      if (!f) throw EditError{ L"cannot read the edits file " + file.wstring() };
      std::vector<Edit> edits;
      std::string raw;
      int line = 0;
      while (std::getline(f, raw))
      {
         line++;
         if (line == 1 && raw.rfind("\xEF\xBB\xBF", 0) == 0) raw.erase(0, 3);
         if (!raw.empty() && raw.back() == '\r') raw.pop_back();
         if (raw.empty() || raw[0] == '#') continue;
         std::wstringstream ss(FromUtf8(raw));
         Edit e{ line };
         std::wstring field;
         std::getline(ss, e.op, L'\t');
         while (std::getline(ss, field, L'\t'))
         {
            auto eq = field.find(L'=');
            if (eq == std::wstring::npos) throw EditError{ L"line " + std::to_wstring(line) + L": expected key=value, got " + field };
            e.args[field.substr(0, eq)] = field.substr(eq + 1);
         }
         edits.push_back(e);
      }
      return edits;
   }

   bool Has(const Args& a, const wchar_t* k) { return a.find(k) != a.end(); }

   const std::wstring& Need(const Args& a, const wchar_t* k)
   {
      auto it = a.find(k);
      if (it == a.end()) throw EditError{ std::wstring(L"missing '") + k + L"'" };
      return it->second;
   }

   double Num(const Args& a, const wchar_t* k)
   {
      try { return std::stod(Need(a, k)); }
      catch (const std::logic_error&) { throw EditError{ std::wstring(L"'") + k + L"' is not a number" }; }
   }

   long Int(const Args& a, const wchar_t* k)
   {
      try { return std::stol(Need(a, k)); }
      catch (const std::logic_error&) { throw EditError{ std::wstring(L"'") + k + L"' is not an integer" }; }
   }

   std::string Num(double v)
   {
      char b[40];
      sprintf_s(b, "%.10g", v);
      return b;
   }

   std::wstring PierLabel(PierIndexType idx, PierIndexType nPiers)
   {
      return (idx == 0 || idx == nPiers - 1 ? L"Abutment " : L"Pier ") + std::to_wstring(idx + 1);
   }

   const char* SpacingTypeName(pgsTypes::SupportedBeamSpacing sbs)
   {
      switch (sbs)
      {
      case pgsTypes::sbsUniform: return "uniform";
      case pgsTypes::sbsGeneral: return "general";
      case pgsTypes::sbsUniformAdjacent: return "uniform adjacent";
      case pgsTypes::sbsGeneralAdjacent: return "general adjacent";
      case pgsTypes::sbsConstantAdjacent: return "constant adjacent";
      case pgsTypes::sbsUniformAdjacentWithTopWidth: return "uniform adjacent with top width";
      case pgsTypes::sbsGeneralAdjacentWithTopWidth: return "general adjacent with top width";
      default: return "other";
      }
   }

   const char* DeckTypeName(pgsTypes::SupportedDeckType dt)
   {
      switch (dt)
      {
      case pgsTypes::sdtCompositeCIP: return "cast-in-place slab";
      case pgsTypes::sdtCompositeSIP: return "cast-in-place slab on stay-in-place panels";
      case pgsTypes::sdtCompositeOverlay: return "composite structural overlay";
      case pgsTypes::sdtNonstructuralOverlay: return "nonstructural overlay";
      case pgsTypes::sdtNone: return "none";
      default: return "other";
      }
   }

   // bridge layout as PGSuper holds it (SI: m, Pa), for the before/after comparison
   std::string Summary(IBridgeDescription* pBD, IBridge* pGeo)
   {
      const CBridgeDescription2* pBridge = pBD->GetBridgeDescription();
      std::ostringstream os;
      PierIndexType nPiers = pBridge->GetPierCount();
      os << "{\"piers\": [";
      for (PierIndexType i = 0; i < nPiers; i++)
      {
         const CPierData2* pPier = pBridge->GetPier(i);
         os << (i ? ", " : "") << "{\"label\": " << JsonString(PierLabel(i, nPiers))
            << ", \"station_m\": " << Num(pPier->GetStation())
            << ", \"orientation\": " << JsonString(pPier->GetOrientation());
         Float64 skew = 0;
         bool bSkew = false;
         try { bSkew = pGeo->GetSkewAngle(pPier->GetStation(), pPier->GetOrientation(), &skew); }
         catch (...) {}
         if (bSkew)
            os << ", \"skew_deg\": " << Num(skew * 180.0 / 3.14159265358979323846);
         os << "}";
      }
      os << "], \"spans\": [";
      SpanIndexType nSpans = pBridge->GetSpanCount();
      for (SpanIndexType s = 0; s < nSpans; s++)
      {
         const CGirderGroupData* pGroup = pBridge->GetGirderGroup((GroupIndexType)s);
         os << (s ? ", " : "") << "{\"span\": " << (s + 1)
            << ", \"length_m\": " << Num(pBridge->GetSpan(s)->GetSpanLength());
         try   // slab overhangs at mid-span (girder CL to deck edge), as PGSuper computes them
         {
            Float64 mid = 0.5 * pGeo->GetSpanLength(s);
            Float64 left = pGeo->GetLeftSlabOverhang(s, mid), right = pGeo->GetRightSlabOverhang(s, mid);
            os << ", \"left_overhang_m\": " << Num(left) << ", \"right_overhang_m\": " << Num(right);
         }
         catch (...) {}
         if (pGroup)
         {
            GirderIndexType nGdr = pGroup->GetGirderCount();
            os << ", \"girders\": " << nGdr << ", \"girder_types\": [";
            for (GirderIndexType g = 0; g < nGdr; g++)
               os << (g ? ", " : "") << JsonString(pGroup->GetGirderName(g));
            os << "], \"girder_data\": [";
            for (GirderIndexType g = 0; g < nGdr; g++)
            {
               const CPrecastSegmentData* pSeg = pBD->GetPrecastSegmentData(CSegmentKey((GroupIndexType)s, g, 0));
               os << (g ? ", " : "") << "{\"girder\": \"" << (char)('A' + g) << "\"";
               if (pSeg)
               {
                  const auto& st = pSeg->Strands;
                  os << ", \"strand_input\": \"" << (st.GetStrandDefinitionType() == pgsTypes::sdtTotal ? "total"
                        : st.GetStrandDefinitionType() == pgsTypes::sdtStraightHarped ? "straight+harped" : "direct") << "\""
                     << ", \"straight\": " << st.GetStrandCount(pgsTypes::Straight)
                     << ", \"harped\": " << st.GetStrandCount(pgsTypes::Harped)
                     << ", \"temporary\": " << st.GetStrandCount(pgsTypes::Temporary)
                     << ", \"fc_pa\": " << Num(pSeg->Material.Concrete.Fc)
                     << ", \"fci_pa\": " << Num(pSeg->Material.Concrete.Fci);
               }
               os << "}";
            }
            os << "]";
         }
         os << "}";
      }
      auto sbs = pBridge->GetGirderSpacingType();
      os << "], \"spacing_type\": \"" << SpacingTypeName(sbs) << "\"";
      if (sbs == pgsTypes::sbsUniform)
         os << ", \"spacing_m\": " << Num(pBridge->GetGirderSpacing());
      const CDeckDescription2* pDeck = pBridge->GetDeckDescription();
      auto dt = pDeck->GetDeckType();
      os << ", \"deck\": {\"type\": \"" << DeckTypeName(dt) << "\"";
      if (dt != pgsTypes::sdtNone)
         os << ", \"thickness_m\": " << Num(pDeck->GrossDepth)
            << ", \"overhang_edge_depth_m\": " << Num(pDeck->OverhangEdgeDepth[0])
            << ", \"fc_pa\": " << Num(pDeck->Concrete.Fc);
      auto sot = pBridge->GetSlabOffsetType();
      os << ", \"slab_offset_type\": \"" << (sot == pgsTypes::sotBridge ? "bridge" : sot == pgsTypes::sotBearingLine ? "per pier" : "per girder") << "\"";
      if (sot == pgsTypes::sotBridge)
         os << ", \"slab_offset_m\": " << Num(pBridge->GetSlabOffset());
      os << "}}";
      return os.str();
   }

   // girders an edit applies to: [span [girder]] or the whole bridge
   std::vector<CGirderKey> Targets(IBridgeDescription* pBD, const Args& a)
   {
      const CBridgeDescription2* pBridge = pBD->GetBridgeDescription();
      std::vector<CGirderKey> keys;
      GroupIndexType first = 0, last = pBridge->GetGirderGroupCount() - 1;
      if (Has(a, L"span"))
      {
         long span = Int(a, L"span");
         if (span < 1 || (GroupIndexType)span > pBridge->GetGirderGroupCount())
            throw EditError{ L"span " + std::to_wstring(span) + L" does not exist (the bridge has "
                             + std::to_wstring(pBridge->GetGirderGroupCount()) + L")" };
         first = last = (GroupIndexType)(span - 1);
      }
      else if (Has(a, L"girder"))
         throw EditError{ L"'girder' needs 'span'" };
      for (GroupIndexType grp = first; grp <= last; grp++)
      {
         GirderIndexType nGdr = pBridge->GetGirderGroup(grp)->GetGirderCount();
         if (Has(a, L"girder"))
         {
            long g = Int(a, L"girder");
            if (g < 1 || (GirderIndexType)g > nGdr)
               throw EditError{ L"girder " + std::to_wstring(g) + L" does not exist in span "
                                + std::to_wstring(grp + 1) + L" (it has " + std::to_wstring(nGdr) + L")" };
            keys.emplace_back(grp, (GirderIndexType)(g - 1));
         }
         else
            for (GirderIndexType g = 0; g < nGdr; g++)
               keys.emplace_back(grp, g);
      }
      return keys;
   }

   // left minus right slab overhang at mid-span of span 1 (where the girder group sits on the deck)
   double OverhangDiff(IBridge* pGeo)
   {
      Float64 mid = 0.5 * pGeo->GetSpanLength(0);
      return pGeo->GetLeftSlabOverhang(0, mid) - pGeo->GetRightSlabOverhang(0, mid);
   }

   // Shift the reference girder offset until left - right overhang (span 1 mid-span) equals
   // target. The offset doesn't move the girders one-for-one on the deck (it is measured along its
   // own datum), so solve for it: secant steps from a trial step, verified. On failure the
   // original offset is restored and false returned.
   bool SolveOverhangDiff(IBridgeDescription* pBD, IBridge* pGeo, double target)
   {
      double dA = OverhangDiff(pGeo);
      if (fabs(dA - target) <= 0.001) return true;
      CBridgeDescription2 desc(*pBD->GetBridgeDescription());
      double off0 = desc.GetRefGirderOffset();
      double offA = off0, offB = off0 + 0.3;    // trial step (m)
      desc.SetRefGirderOffset(offB);
      pBD->SetBridgeDescription(desc);
      double dB = OverhangDiff(pGeo);
      for (int it = 0; it < 6 && 0.001 < fabs(dB - target) && 1e-9 < fabs(dB - dA); it++)
      {
         double offC = offB + (target - dB) * (offB - offA) / (dB - dA);
         offA = offB; dA = dB;
         offB = offC;
         desc.SetRefGirderOffset(offB);
         pBD->SetBridgeDescription(desc);
         dB = OverhangDiff(pGeo);
      }
      if (fabs(dB - target) <= 0.001) return true;
      desc.SetRefGirderOffset(off0);
      pBD->SetBridgeDescription(desc);
      return false;
   }

   std::wstring Ft(double m)
   {
      wchar_t b[40];
      swprintf_s(b, L"%.3f ft", m / 0.3048);
      return b;
   }

   void Apply(IBridgeDescription* pBD, ILibraryNames* pLibNames, IBridge* pGeo, const Edit& e, std::wstring& note)
   {
      const CBridgeDescription2* pBridge = pBD->GetBridgeDescription();
      const Args& a = e.args;
      if (e.op == L"set_girder_count")
      {
         long n = Int(a, L"count");
         if (n < 1 || 100 < n) throw EditError{ L"count must be 1..100" };
         if (Has(a, L"span"))
         {
            long span = Int(a, L"span");
            if (span < 1 || (GroupIndexType)span > pBridge->GetGirderGroupCount())
               throw EditError{ L"span " + std::to_wstring(span) + L" does not exist" };
            if (pBD->UseSameNumberOfGirdersInAllGroups())
               pBD->UseSameNumberOfGirdersInAllGroups(false);   // one span only: counts may now differ by span
            pBD->SetGirderCount((GroupIndexType)(span - 1), (GirderIndexType)n);
         }
         else if (pBD->UseSameNumberOfGirdersInAllGroups())
            pBD->SetGirderCount((GirderIndexType)n);
         else
            for (GroupIndexType g = 0; g < pBridge->GetGirderGroupCount(); g++)
               pBD->SetGirderCount(g, (GirderIndexType)n);
      }
      else if (e.op == L"move_pier")
      {
         long pier = Int(a, L"pier");
         if (pier < 1 || (PierIndexType)pier > pBridge->GetPierCount())
            throw EditError{ L"pier " + std::to_wstring(pier) + L" does not exist (the bridge has "
                             + std::to_wstring(pBridge->GetPierCount()) + L" piers and abutments)" };
         static const std::map<std::wstring, pgsTypes::MovePierOption> options{
            {L"move_bridge", pgsTypes::MoveBridge}, {L"adjust_prev_span", pgsTypes::AdjustPrevSpan},
            {L"adjust_next_span", pgsTypes::AdjustNextSpan}, {L"adjust_adjacent_spans", pgsTypes::AdjustAdjacentSpans} };
         std::wstring opt = Has(a, L"option") ? Need(a, L"option") : std::wstring(L"adjust_adjacent_spans");
         auto it = options.find(opt);
         if (it == options.end()) throw EditError{ L"unknown option " + opt };
         PierIndexType idx = (PierIndexType)(pier - 1);
         if ((idx == 0 && it->second == pgsTypes::AdjustPrevSpan) ||
             (idx == pBridge->GetPierCount() - 1 && it->second == pgsTypes::AdjustNextSpan))
            throw EditError{ L"option " + opt + L" needs a span on that side of the pier" };
         double station = Num(a, L"station");
         // the move must keep the piers in order (no negative or zero-length spans)
         if (it->second != pgsTypes::MoveBridge)
         {
            double lo = idx > 0 ? pBridge->GetPier(idx - 1)->GetStation() : -1e30;
            double hi = idx + 1 < pBridge->GetPierCount() ? pBridge->GetPier(idx + 1)->GetStation() : 1e30;
            if (it->second == pgsTypes::AdjustAdjacentSpans && !(lo < station && station < hi))
               throw EditError{ L"the new station must lie between the neighbouring piers" };
         }
         pBD->MovePier(idx, station, it->second);
      }
      else if (e.op == L"set_span_length")
      {
         long span = Int(a, L"span");
         if (span < 1 || (SpanIndexType)span > pBridge->GetSpanCount())
            throw EditError{ L"span " + std::to_wstring(span) + L" does not exist" };
         double L = Num(a, L"length");
         if (L <= 0) throw EditError{ L"length must be positive" };
         pBD->SetSpanLength((SpanIndexType)(span - 1), L);
      }
      else if (e.op == L"set_girder_spacing")
      {
         double s = Num(a, L"spacing");
         if (s <= 0) throw EditError{ L"spacing must be positive" };
         auto sbs = pBridge->GetGirderSpacingType();
         if (!IsSpreadSpacing(sbs))
            throw EditError{ L"the bridge uses adjacent girders: spacing follows the girder width and joints" };
         std::wstring anchor = Has(a, L"anchor") ? Need(a, L"anchor") : std::wstring(L"center");
         if (anchor != L"center" && anchor != L"reference")
            throw EditError{ L"anchor must be 'center' or 'reference'" };
         if (sbs == pgsTypes::sbsUniform)
         {
            if (Has(a, L"pier")) throw EditError{ L"spacing is uniform for the whole bridge: omit 'pier'" };
            double d0 = OverhangDiff(pGeo);
            pBD->SetGirderSpacing(s);
            // PGSuper keeps the reference girder where it is, so the change lands on one side:
            // move the group back to where it sat on the deck
            if (anchor == L"center" && 0.001 < fabs(OverhangDiff(pGeo) - d0))
               note = SolveOverhangDiff(pBD, pGeo, d0)
                  ? L"girders re-centred: reference girder offset adjusted so the overhangs keep their balance"
                  : L"could not re-centre the girders; PGSuper's reference girder placement kept - check the overhangs";
         }
         else
         {
            PierIndexType nPiers = pBridge->GetPierCount();
            PierIndexType first = 0, last = nPiers - 1;
            if (Has(a, L"pier"))
            {
               long pier = Int(a, L"pier");
               if (pier < 1 || (PierIndexType)pier > nPiers) throw EditError{ L"pier does not exist" };
               first = last = (PierIndexType)(pier - 1);
            }
            for (PierIndexType p = first; p <= last; p++)
               for (auto face : { pgsTypes::Back, pgsTypes::Ahead })
               {
                  if ((face == pgsTypes::Back && p == 0) || (face == pgsTypes::Ahead && p == nPiers - 1))
                     continue;   // no span on that side
                  const CGirderSpacing2* pSpacing = pBridge->GetPier(p)->GetGirderSpacing(face);
                  if (!pSpacing) continue;
                  CGirderSpacing2 spacing(*pSpacing);
                  for (GroupIndexType g = 0; g < spacing.GetSpacingGroupCount(); g++)
                     spacing.SetGirderSpacing(g, s);
                  pBD->SetGirderSpacing(p, face, spacing);
               }
            if (anchor == L"center")
               note = L"general (pier-by-pier) spacing: girders were not re-centred; check the overhangs";
         }
      }
      else if (e.op == L"set_girder_type")
      {
         std::wstring name = Need(a, L"name");
         std::vector<std::_tstring> names;
         pLibNames->EnumGirderNames(&names);
         if (std::find(names.begin(), names.end(), name) == names.end())
            throw EditError{ L"girder type '" + name + L"' is not in the girder library" };
         if (!Has(a, L"span") && !Has(a, L"girder") && pBD->UseSameGirderForEntireBridge())
            pBD->SetGirderName(name.c_str());
         else
         {
            if (pBD->UseSameGirderForEntireBridge())
               pBD->UseSameGirderForEntireBridge(false);   // part of the bridge only
            for (const auto& key : Targets(pBD, a))
               pBD->SetGirderName(key, name.c_str());
         }
      }
      else if (e.op == L"set_strands")
      {
         for (const auto& key : Targets(pBD, a))
         {
            CSegmentKey segKey(key, 0);
            CPrecastSegmentData seg(*pBD->GetPrecastSegmentData(segKey));
            auto type = seg.Strands.GetStrandDefinitionType();
            if (Has(a, L"straight") || Has(a, L"harped"))
            {
               if (type != pgsTypes::sdtStraightHarped)
                  throw EditError{ L"girder uses " + std::wstring(type == pgsTypes::sdtTotal ? L"total-strand input: give 'total'"
                                   : L"direct strand input (fill/rows/individual strands), which isn't supported") };
               if (Has(a, L"straight")) seg.Strands.SetStrandCount(pgsTypes::Straight, (StrandIndexType)Int(a, L"straight"));
               if (Has(a, L"harped")) seg.Strands.SetStrandCount(pgsTypes::Harped, (StrandIndexType)Int(a, L"harped"));
            }
            if (Has(a, L"total"))
            {
               if (type != pgsTypes::sdtTotal)
                  throw EditError{ L"girder uses separate straight/harped input: give 'straight' and 'harped'" };
               seg.Strands.SetStrandCount(pgsTypes::Permanent, (StrandIndexType)Int(a, L"total"));
            }
            if (Has(a, L"temporary"))
               seg.Strands.SetStrandCount(pgsTypes::Temporary, (StrandIndexType)Int(a, L"temporary"));
            pBD->SetPrecastSegmentData(segKey, seg);
         }
      }
      else if (e.op == L"set_concrete")
      {
         if (!Has(a, L"fc") && !Has(a, L"fci")) throw EditError{ L"give 'fc' and/or 'fci'" };
         for (const auto& key : Targets(pBD, a))
         {
            CSegmentKey segKey(key, 0);
            CPrecastSegmentData seg(*pBD->GetPrecastSegmentData(segKey));
            if (Has(a, L"fc")) seg.Material.Concrete.Fc = Num(a, L"fc");
            if (Has(a, L"fci")) seg.Material.Concrete.Fci = Num(a, L"fci");
            if (seg.Material.Concrete.Fci > seg.Material.Concrete.Fc)
               throw EditError{ L"f'ci would exceed f'c" };
            pBD->SetPrecastSegmentData(segKey, seg);
         }
      }
      else if (e.op == L"set_slab_thickness")
      {
         double th = Num(a, L"thickness");
         if (th <= 0) throw EditError{ L"thickness must be positive" };
         CDeckDescription2 deck(*pBridge->GetDeckDescription());
         if (deck.GetDeckType() == pgsTypes::sdtNone) throw EditError{ L"the bridge has no deck" };
         double old = deck.GrossDepth;
         deck.GrossDepth = th;
         for (auto& d : deck.OverhangEdgeDepth)   // a constant-depth overhang stays constant
            if (fabs(d - old) < 1e-6) d = th;
         pBD->SetDeckDescription(deck);
      }
      else if (e.op == L"set_deck_concrete")
      {
         double fc = Num(a, L"fc");
         if (fc <= 0) throw EditError{ L"fc must be positive" };
         CDeckDescription2 deck(*pBridge->GetDeckDescription());
         if (deck.GetDeckType() == pgsTypes::sdtNone) throw EditError{ L"the bridge has no deck" };
         deck.Concrete.Fc = fc;
         pBD->SetDeckDescription(deck);
      }
      else if (e.op == L"set_slab_offset")
      {
         double A = Num(a, L"slab_offset");
         if (A <= 0) throw EditError{ L"slab_offset must be positive" };
         if (pGeo->GetHaunchInputDepthType() != pgsTypes::hidACamber)
            throw EditError{ L"this model defines the haunch by its depth, not by the \"A\" dimension" };
         pBD->SetSlabOffsetType(pgsTypes::sotBridge);
         pBD->SetSlabOffset(A);
      }
      else if (e.op == L"set_skew")
      {
         double ang = Num(a, L"angle");
         if (ang < 0 || 89 < ang) throw EditError{ L"skew angle must be 0 to 89 degrees (give the side with 'direction')" };
         std::wstring dir = Has(a, L"direction") ? Need(a, L"direction") : std::wstring(L"right");
         if (dir != L"left" && dir != L"right") throw EditError{ L"direction must be left or right" };
         wchar_t buf[64];
         swprintf_s(buf, L"%.10g %s", ang, dir == L"left" ? L"L" : L"R");
         std::wstring orient = ang == 0 ? std::wstring(L"Normal") : std::wstring(buf);
         CBridgeDescription2 desc(*pBridge);
         PierIndexType nPiers = desc.GetPierCount(), first = 0, last = nPiers - 1;
         if (Has(a, L"pier"))
         {
            long pier = Int(a, L"pier");
            if (pier < 1 || (PierIndexType)pier > nPiers) throw EditError{ L"pier " + std::to_wstring(pier) + L" does not exist" };
            first = last = (PierIndexType)(pier - 1);
         }
         for (PierIndexType p = first; p <= last; p++)
         {
            Float64 skew;
            if (!pGeo->GetSkewAngle(desc.GetPier(p)->GetStation(), orient.c_str(), &skew))
               throw EditError{ L"PGSuper did not accept the pier orientation '" + orient + L"'" };
            desc.GetPier(p)->SetOrientation(orient.c_str());
         }
         pBD->SetBridgeDescription(desc);
         note = L"pier orientation set to '" + orient + L"'";
      }
      else if (e.op == L"set_overhang")
      {
         const CDeckDescription2* pDeck = pBridge->GetDeckDescription();
         if (IsConstantWidthDeck(pDeck->GetDeckType()) || pDeck->DeckEdgePoints.empty())
            throw EditError{ L"this deck type has no overhang input: the deck width follows the girders" };
         Float64 mid = 0.5 * pGeo->GetSpanLength(0);
         auto left = [&] { return pGeo->GetLeftSlabOverhang(0, mid); };
         auto right = [&] { return pGeo->GetRightSlabOverhang(0, mid); };
         double L, R;
         if (Has(a, L"overhang"))
            L = R = Num(a, L"overhang");
         else if (Has(a, L"left") || Has(a, L"right"))
         {
            L = Has(a, L"left") ? Num(a, L"left") : left();
            R = Has(a, L"right") ? Num(a, L"right") : right();
         }
         else
            throw EditError{ L"give 'overhang', or 'left' and/or 'right'" };
         if (L < 0 || R < 0) throw EditError{ L"overhangs must not be negative" };
         std::wstring keep = Has(a, L"keep") ? Need(a, L"keep") : std::wstring(L"deck_width");
         if (keep == L"spacing")
         {
            // move the deck edges; the girders stay where they are
            for (int it = 0; it < 5; it++)
            {
               double dl = L - left(), dr = R - right();
               if (fabs(dl) < 0.0005 && fabs(dr) < 0.0005) break;
               CBridgeDescription2 desc(*pBD->GetBridgeDescription());
               for (auto& pt : desc.GetDeckDescription()->DeckEdgePoints)
               {
                  pt.LeftEdge += dl;
                  pt.RightEdge += dr;
               }
               pBD->SetBridgeDescription(desc);
            }
            if (0.001 < fabs(L - left()) || 0.001 < fabs(R - right()))
               throw EditError{ L"could not reach these overhangs by moving the deck edges" };
            note = L"deck edges moved, girder spacing unchanged: the deck width is now " + Ft(L + R) + L" of overhang plus the girder group";
         }
         else if (keep == L"deck_width")
         {
            // same deck: solve for the uniform spacing (girders centred as asked) that gives L + R
            if (pBridge->GetGirderSpacingType() != pgsTypes::sbsUniform)
               throw EditError{ L"keeping the deck width needs uniform girder spacing: use keep='spacing', or set the spacing first" };
            GirderIndexType n = pBridge->GetGirderGroup((GroupIndexType)0)->GetGirderCount();
            if (n < 2) throw EditError{ L"one girder: use keep='spacing'" };
            double target = L + R;
            auto trial = [&](double s)
            {
               pBD->SetGirderSpacing(s);
               SolveOverhangDiff(pBD, pGeo, L - R);
               return left() + right() - target;
            };
            double sA = pBD->GetBridgeDescription()->GetGirderSpacing();
            double gA = left() + right() - target;
            double sB = sA + gA / (n - 1);
            if (sB <= 0) throw EditError{ L"the overhangs leave no room for the girders on this deck" };
            double gB = trial(sB);
            for (int it = 0; it < 8 && 0.001 < fabs(gB) && 1e-9 < fabs(gB - gA); it++)
            {
               double sC = sB - gB * (sB - sA) / (gB - gA);
               if (sC <= 0) throw EditError{ L"the overhangs leave no room for the girders on this deck" };
               sA = sB; gA = gB;
               sB = sC;
               gB = trial(sB);
            }
            if (0.001 < fabs(gB) || 0.001 < fabs(L - left()))
               throw EditError{ L"could not find a girder spacing that gives these overhangs on this deck" };
            note = L"deck width kept: girder spacing set to " + Ft(sB);
         }
         else
            throw EditError{ L"keep must be deck_width or spacing" };
      }
      else
         throw EditError{ L"unknown edit '" + e.op + L"'" };
   }
}

namespace
{
   // ---------------------------------------------------------------- girder design
   // The design op runs PGSuper's own girder design (IArtifact::CreateDesignArtifact, what
   // Project > Design Girder uses) for the selected girders, then copies each design into the
   // bridge the way the GUI's design transaction (txnDesignGirder) does: the designed segment
   // data (strands, concrete, handling, stirrups when shear is designed) and, when the haunch
   // is designed, the "A" dimension of each designed girder. All girders are designed first
   // and then applied together, as the GUI does.

   struct OutcomeInfo
   {
      const char* name;
      const char* text;
   };

   OutcomeInfo DescribeOutcome(pgsSegmentDesignArtifact::Outcome o)
   {
      using A = pgsSegmentDesignArtifact;
      switch (o)
      {
      case A::Success: return { "success", "design succeeded" };
      case A::SuccessButLongitudinalBarsNeeded4FlexuralTensionCy: return { "success_bars_needed_release", "design succeeded; longitudinal bars are needed for tension at release (casting yard)" };
      case A::SuccessButLongitudinalBarsNeeded4FlexuralTensionLifting: return { "success_bars_needed_lifting", "design succeeded; longitudinal bars are needed for tension during lifting" };
      case A::SuccessButLongitudinalBarsNeeded4FlexuralTensionHauling: return { "success_bars_needed_hauling", "design succeeded; longitudinal bars are needed for tension during hauling" };
      case A::TooManyStrandsReqd: return { "too_many_strands", "more strands are required than the girder can hold" };
      case A::ReleaseStrength: return { "release_strength", "the required release strength f'ci exceeds the maximum allowed" };
      case A::OverReinforced: return { "over_reinforced", "the section is over-reinforced" };
      case A::UnderReinforced: return { "under_reinforced", "the section is under-reinforced (minimum reinforcement not met)" };
      case A::UltimateMomentCapacity: return { "moment_capacity", "the ultimate moment capacity is not sufficient" };
      case A::StrandSlopeOutOfRange: return { "strand_slope", "the harped strand slope is outside the allowed range" };
      case A::ExceededMaxHoldDownForce: return { "hold_down_force", "the maximum hold-down force is exceeded" };
      case A::ShearExceedsMaxConcreteStrength: return { "shear_concrete_strength", "shear needs a concrete strength above the maximum allowed" };
      case A::TooManyStirrupsReqd: return { "too_many_stirrups", "too many stirrups are required for shear" };
      case A::TooManyStirrupsReqdForHorizontalInterfaceShear: return { "too_many_stirrups_interface", "too many stirrups are required for horizontal interface shear" };
      case A::TooManyStirrupsReqdForSplitting: return { "too_many_stirrups_splitting", "too many stirrups are required for splitting resistance" };
      case A::ConflictWithLongReinforcementShearSpec: return { "long_reinf_conflict", "longitudinal reinforcement for shear conflicts with the project criteria" };
      case A::TooManyBarsForLongReinfShear: return { "long_reinf_bars", "too many bars are needed for longitudinal reinforcement for shear" };
      case A::TooMuchStrandsForLongReinfShear: return { "long_reinf_strands", "too many strands are needed for longitudinal reinforcement for shear" };
      case A::StrandsReqdForLongReinfShearAndFlexureTurnedOff: return { "long_reinf_needs_flexure", "strands are needed for longitudinal reinforcement for shear, but flexural design is off" };
      case A::NoDevelopmentLengthForLongReinfShear: return { "long_reinf_development", "bars cannot develop for longitudinal reinforcement for shear" };
      case A::NoStrandDevelopmentLengthForLongReinfShear: return { "long_reinf_strand_development", "strands cannot develop for longitudinal reinforcement for shear" };
      case A::RebarForceExceedsPretensionForceForLongReinfShear: return { "long_reinf_rebar_force", "rebar force exceeds the pretension force for longitudinal reinforcement for shear" };
      case A::MaxIterExceeded: return { "not_converged", "the design did not converge (maximum iterations exceeded)" };
      case A::GirderLiftingConcreteStrength: return { "lifting_strength", "lifting needs a concrete strength above the maximum allowed" };
      case A::GirderLiftingStability: return { "lifting_stability", "the girder is not stable during lifting" };
      case A::GirderShippingConcreteStrength: return { "hauling_strength", "hauling needs a concrete strength above the maximum allowed" };
      case A::GirderShippingStability: return { "hauling_stability", "the girder is not stable during hauling" };
      case A::GirderShippingConfiguration: return { "hauling_configuration", "no hauling configuration (support locations, truck) works" };
      case A::StressExceedsConcreteStrength: return { "stress_exceeds_strength", "stresses exceed what the maximum concrete strength allows" };
      case A::DebondDesignFailed: return { "debond_failed", "the debonding design failed" };
      case A::DesignCancelled: return { "cancelled", "the design was cancelled" };
      case A::NoDesignRequested: return { "no_design_requested", "no design was requested" };
      case A::LldfRangeOfApplicabilityError: return { "lldf_range", "live load distribution factors are outside the LRFD range of applicability" };
      case A::DesignNotSupported_Losses: return { "not_supported_losses", "design is not available with time-step losses" };
      case A::DesignNotSupported_Strands: return { "not_supported_strands", "design is not available for this strand input type" };
      case A::DesignNotSupported_Material: return { "not_supported_material", "design is not available for this girder material (UHPC)" };
      default: return { "other", "unrecognised design outcome" };
      }
   }

   bool IsSuccess(pgsSegmentDesignArtifact::Outcome o)
   {
      using A = pgsSegmentDesignArtifact;
      return o == A::Success || o == A::SuccessButLongitudinalBarsNeeded4FlexuralTensionCy ||
             o == A::SuccessButLongitudinalBarsNeeded4FlexuralTensionLifting ||
             o == A::SuccessButLongitudinalBarsNeeded4FlexuralTensionHauling;
   }

   bool IsNotApplicable(pgsSegmentDesignArtifact::Outcome o)
   {
      using A = pgsSegmentDesignArtifact;
      return o == A::DesignCancelled || o == A::NoDesignRequested || o == A::DesignNotSupported_Losses ||
             o == A::DesignNotSupported_Strands || o == A::DesignNotSupported_Material;
   }

   const char* DescribeNote(pgsSegmentDesignArtifact::DesignNote n)
   {
      using A = pgsSegmentDesignArtifact;
      switch (n)
      {
      case A::dnExistingShearDesignPassedSpecCheck: return "the existing stirrup layout passed the shear check and was kept";
      case A::dnShearRequiresStrutAndTie: return "shear near the supports requires a strut-and-tie analysis";
      case A::dnStrandsAddedForLongReinfShear: return "strands were added for longitudinal reinforcement for shear";
      case A::dnStirrupsTightendedForLongReinfShear: return "stirrups were tightened for longitudinal reinforcement for shear";
      case A::dnLongitudinalBarsNeeded4FlexuralTensionCy: return "longitudinal bars are needed for tension at release";
      case A::dnLongitudinalBarsNeeded4FlexuralTensionLifting: return "longitudinal bars are needed for tension during lifting";
      case A::dnLongitudinalBarsNeeded4FlexuralTensionHauling: return "longitudinal bars are needed for tension during hauling";
      case A::dnConcreteStrengthIncreasedForShearStress: return "concrete strength was increased for shear stress";
      case A::dnTransformedSectionsSetToGross: return "designed with gross section properties (the criteria use transformed sections)";
      case A::dnParabolicHaunchSetToConstant: return "designed with a constant haunch (the criteria use a parabolic haunch)";
      default: return "design note";
      }
   }

   const char* FlexureDesignName(arFlexuralDesignType t)
   {
      switch (t)
      {
      case dtNoDesign: return "none";
      case dtDesignForHarping: return "harped strands";
      case dtDesignForDebonding: return "debonded straight strands";
      case dtDesignFullyBonded: return "fully bonded straight strands";
      case dtDesignFullyBondedRaised: return "fully bonded raised straight strands";
      case dtDesignForDebondingRaised: return "debonded raised straight strands";
      default: return "other";
      }
   }

   // girders to design: [span] and [girders=1,2,...] (1-based); default every girder of every span
   std::vector<CGirderKey> DesignTargets(IBridgeDescription* pBD, const Args& a)
   {
      const CBridgeDescription2* pBridge = pBD->GetBridgeDescription();
      GroupIndexType nGroups = pBridge->GetGirderGroupCount();
      GroupIndexType first = 0, last = nGroups - 1;
      if (Has(a, L"span"))
      {
         long span = Int(a, L"span");
         if (span < 1 || (GroupIndexType)span > nGroups)
            throw EditError{ L"span " + std::to_wstring(span) + L" does not exist (the bridge has " + std::to_wstring(nGroups) + L")" };
         first = last = (GroupIndexType)(span - 1);
      }
      std::vector<long> girders;
      if (Has(a, L"girders"))
      {
         std::wstringstream ss(Need(a, L"girders"));
         std::wstring item;
         while (std::getline(ss, item, L','))
         {
            try { girders.push_back(std::stol(item)); }
            catch (const std::logic_error&) { throw EditError{ L"'girders' must be a comma-separated list of girder numbers" }; }
         }
         if (girders.empty()) throw EditError{ L"'girders' is empty" };
      }
      std::vector<CGirderKey> keys;
      for (GroupIndexType grp = first; grp <= last; grp++)
      {
         GirderIndexType nGdr = pBridge->GetGirderGroup(grp)->GetGirderCount();
         if (girders.empty())
         {
            for (GirderIndexType g = 0; g < nGdr; g++)
               keys.emplace_back(grp, g);
            continue;
         }
         for (long g : girders)
         {
            if (g < 1 || (GirderIndexType)g > nGdr)
               throw EditError{ L"girder " + std::to_wstring(g) + L" does not exist in span " + std::to_wstring(grp + 1)
                                + L" (it has " + std::to_wstring(nGdr) + L")" };
            CGirderKey key(grp, (GirderIndexType)(g - 1));
            if (std::find(keys.begin(), keys.end(), key) == keys.end())
               keys.push_back(key);
         }
      }
      return keys;
   }

   std::string StrengthState(const pgsSegmentDesignArtifact::ConcreteStrengthDesignState& s)
   {
      try { return JsonString(s.AsString()); }
      catch (...) { return "null"; }
   }

   // one girder's design as JSON (SI: m, Pa)
   std::string DesignJson(const pgsSegmentDesignArtifact* pSeg, bool bApplied, bool bHaunchApplied)
   {
      const CSegmentKey& key = pSeg->GetSegmentKey();
      auto outcome = pSeg->GetOutcome();
      OutcomeInfo info = DescribeOutcome(outcome);
      const auto& opt = pSeg->GetDesignOptions();
      std::ostringstream os;
      os << "{\"span\": " << (key.groupIndex + 1) << ", \"girder\": \"" << (char)('A' + key.girderIndex) << "\""
         << ", \"outcome\": \"" << info.name << "\", \"success\": " << (IsSuccess(outcome) ? "true" : "false")
         << ", \"message\": " << JsonString(CA2W(info.text).m_psz)
         << ", \"applied\": " << (bApplied ? "true" : "false")
         << ", \"notes\": [";
      auto notes = pSeg->GetDesignNotes();
      for (size_t i = 0; i < notes.size(); i++)
         os << (i ? ", " : "") << JsonString(CA2W(DescribeNote(notes[i])).m_psz);
      os << "]";
      if (IsNotApplicable(outcome))
         return os.str() + "}";

      os << ", \"flexure_design\": \"" << FlexureDesignName(pSeg->GetDoDesignFlexure()) << "\"";
      if (opt.doDesignForFlexure != dtNoDesign)
      {
         CPrecastSegmentData data = pSeg->GetSegmentData();
         const WBFL::Materials::PsStrand* pStrand = data.Strands.GetStrandMaterial(pgsTypes::Straight);
         StrandIndexType ns = pSeg->GetNumStraightStrands(), nh = pSeg->GetNumHarpedStrands();
         os << ", \"straight\": " << ns << ", \"harped\": " << nh
            << ", \"temporary\": " << pSeg->GetNumTempStrands()
            << ", \"total_permanent\": " << (ns + nh)
            << ", \"debonded\": " << pSeg->GetStraightStrandDebondInfo().size()
            << ", \"strand\": " << (pStrand ? JsonString(pStrand->GetName()) : std::string("null"))
            << ", \"harp_offset_end_m\": " << Num(pSeg->GetHarpStrandOffsetEnd(pgsTypes::metStart))
            << ", \"harp_offset_hp_m\": " << Num(pSeg->GetHarpStrandOffsetHp(pgsTypes::metStart))
            << ", \"fc_pa\": " << Num(pSeg->GetConcreteStrength())
            << ", \"fci_pa\": " << Num(pSeg->GetReleaseStrength())
            << ", \"fc_controlled_by\": " << StrengthState(pSeg->GetFinalDesignState())
            << ", \"fci_controlled_by\": " << StrengthState(pSeg->GetReleaseDesignState())
            << ", \"slab_offset_start_m\": " << Num(pSeg->GetSlabOffset(pgsTypes::metStart))
            << ", \"slab_offset_end_m\": " << Num(pSeg->GetSlabOffset(pgsTypes::metEnd))
            << ", \"haunch_applied\": " << (bHaunchApplied ? "true" : "false");
         if (opt.doDesignLifting)
            os << ", \"lift_point_left_m\": " << Num(pSeg->GetLeftLiftingLocation())
               << ", \"lift_point_right_m\": " << Num(pSeg->GetRightLiftingLocation());
         if (opt.doDesignHauling)
            os << ", \"haul_leading_overhang_m\": " << Num(pSeg->GetLeadingOverhang())
               << ", \"haul_trailing_overhang_m\": " << Num(pSeg->GetTrailingOverhang())
               << ", \"haul_truck\": " << JsonString(pSeg->GetHaulTruck() ? pSeg->GetHaulTruck() : L"");
      }
      os << ", \"shear_designed\": " << (pSeg->GetDoDesignShear() ? "true" : "false");
      if (pSeg->GetDoDesignShear() && pSeg->GetShearData())
      {
         const CShearData2* pShear = pSeg->GetShearData();
         os << ", \"stirrup_zones_designed\": " << pSeg->GetNumberOfStirrupZonesDesigned()
            << ", \"stirrups_symmetric\": " << (pShear->bAreZonesSymmetrical ? "true" : "false")
            << ", \"stirrup_zones\": [";
         for (size_t i = 0; i < pShear->ShearZones.size(); i++)
         {
            const auto& z = pShear->ShearZones[i];
            os << (i ? ", " : "") << "{\"zone\": " << (i + 1)
               << ", \"length_m\": " << Num(z.ZoneLength)
               << ", \"bar\": " << JsonString(WBFL::LRFD::RebarPool::GetBarSize(z.VertBarSize))
               << ", \"spacing_m\": " << Num(z.BarSpacing)
               << ", \"legs\": " << Num(z.nVertBars)
               << ", \"interface_legs\": " << Num(z.nHorzInterfaceBars) << "}";
         }
         os << "]";
      }
      return os.str() + "}";
   }

   // design op: flexure=1|0, shear=none|layout|retain, concrete=min_strength|preserve,
   // haunch=preserve|design, [span], [girders=1,2]. Returns the JSON array of girder designs.
   std::string Design(std::shared_ptr<WBFL::EAF::Broker> pBroker, IBridgeDescription* pBD, const Args& a, std::wstring& note)
   {
      bool bFlexure = !Has(a, L"flexure") || Need(a, L"flexure") != L"0";
      std::wstring shear = Has(a, L"shear") ? Need(a, L"shear") : std::wstring(L"none");
      arShearDesignType shearType;
      if (shear == L"none") shearType = sdtNoDesign;
      else if (shear == L"layout") shearType = sdtLayoutStirrups;
      else if (shear == L"retain") shearType = sdtRetainExistingLayout;
      else throw EditError{ L"shear must be none, layout or retain" };
      std::wstring conc = Has(a, L"concrete") ? Need(a, L"concrete") : std::wstring(L"min_strength");
      if (conc != L"min_strength" && conc != L"preserve") throw EditError{ L"concrete must be min_strength or preserve" };
      arConcreteDesignType concType = conc == L"preserve" ? cdPreserveStrength : cdDesignForMinStrength;
      std::wstring haunch = Has(a, L"haunch") ? Need(a, L"haunch") : std::wstring(L"preserve");
      if (haunch != L"preserve" && haunch != L"design") throw EditError{ L"haunch must be preserve or design" };
      arSlabOffsetDesignType haunchType = haunch == L"design" ? sodDesignHaunch : sodPreserveHaunch;
      if (!bFlexure && shearType == sdtNoDesign) throw EditError{ L"nothing to design: flexure and shear are both off" };

      // the same preconditions the GUI checks before it designs
      GET_IFACE2(pBroker, ILossParameters, pLoss);
      if (pLoss->GetLossMethod() == PrestressLossCriteria::LossMethodType::TIME_STEP)
         throw EditError{ L"prestress losses use the time-step method; PGSuper's girder design is not available for it" };
      GET_IFACE2(pBroker, IEAFStatusCenter, pStatus);
      if (pStatus->GetSeverity() == WBFL::EAF::StatusSeverityType::Error)
      {
         std::wstring msgs;
         for (IndexType i = 0; i < pStatus->Count(); i++)
         {
            auto item = pStatus->GetByIndex(i);
            if (pStatus->GetSeverity(item) == WBFL::EAF::StatusSeverityType::Error)
               msgs += (msgs.empty() ? L"" : L"; ") + std::wstring(item->GetDescription());
         }
         throw EditError{ L"PGSuper reports errors that must be fixed before girders can be designed: " + msgs };
      }
      GET_IFACE2(pBroker, ISpecification, pSpec);
      if (haunchType == sodDesignHaunch)
      {
         GET_IFACE2(pBroker, IBridge, pBridge);
         if (!pSpec->DesignSlabHaunch())
         {
            haunchType = sodPreserveHaunch;
            note = L"the project criteria do not allow haunch design: the haunch was kept";
         }
         else if (pBridge->GetHaunchInputDepthType() != pgsTypes::hidACamber)
         {
            haunchType = sodPreserveHaunch;
            note = L"haunch design needs the \"A\" dimension haunch input: the haunch was kept";
         }
      }

      std::vector<CGirderKey> keys = DesignTargets(pBD, a);
      GET_IFACE2(pBroker, IArtifact, pIArtifact);
      std::vector<pgsGirderDesignArtifact> artifacts;   // copies: applying a design invalidates PGSuper's cached artifacts
      for (const auto& key : keys)
      {
         std::wstring label = L"span " + std::to_wstring(key.groupIndex + 1) + L" girder " + std::wstring(1, (wchar_t)(L'A' + key.girderIndex));
         Trace(L"design " + label);
         const pgsGirderDesignArtifact* pArtifact = pIArtifact->CreateDesignArtifact(key, bFlexure, haunchType, concType, shearType);
         if (!pArtifact) throw EditError{ L"the design of " + label + L" was cancelled" };
         if (pArtifact->GetSegmentDesignArtifact(0)->GetOutcome() == pgsSegmentDesignArtifact::DesignNotSupported_Strands)
            throw EditError{ label + L": PGSuper's design does not support this girder's strand input type" };
         artifacts.push_back(*pArtifact);
      }

      std::vector<std::pair<bool, bool>> applied;   // (segment data, haunch) per girder
      {
         GET_IFACE2(pBroker, IEvents, pEvents);
         CIEventsHolder hold(pEvents);
         bool bSegmentOffsets = false, bGirderCamber = false;
         for (const auto& art : artifacts)
         {
            const pgsSegmentDesignArtifact* pSeg = art.GetSegmentDesignArtifact(0);
            const CSegmentKey& segKey = pSeg->GetSegmentKey();
            const arDesignOptions& opt = pSeg->GetDesignOptions();
            bool bData = false, bHaunch = false;
            if (!IsNotApplicable(pSeg->GetOutcome()))
            {
               if (opt.doDesignForFlexure != dtNoDesign || opt.doDesignForShear != sdtNoDesign)
               {
                  pBD->SetPrecastSegmentData(segKey, pSeg->GetSegmentData());
                  bData = true;
               }
               // haunch: per designed girder, as the GUI's "all selected girders" choice
               if (haunchType == sodDesignHaunch && opt.doDesignForFlexure != dtNoDesign && opt.doDesignSlabOffset != sodPreserveHaunch)
               {
                  if (!bSegmentOffsets) { pBD->SetSlabOffsetType(pgsTypes::sotSegment); bSegmentOffsets = true; }
                  pBD->SetSlabOffset(segKey, pSeg->GetSlabOffset(pgsTypes::metStart), pSeg->GetSlabOffset(pgsTypes::metEnd));
                  if (opt.doDesignSlabOffset == sodDesignHaunch && pSpec->IsAssumedExcessCamberForLoad())
                  {
                     if (!bGirderCamber) { pBD->SetAssumedExcessCamberType(pgsTypes::aecGirder); bGirderCamber = true; }
                     pBD->SetAssumedExcessCamber(segKey.groupIndex, segKey.girderIndex, pSeg->GetAssumedExcessCamber());
                  }
                  bHaunch = true;
               }
            }
            applied.emplace_back(bData, bHaunch);
         }
      }

      std::ostringstream os;
      os << "[";
      for (size_t i = 0; i < artifacts.size(); i++)
         os << (i ? ", " : "") << DesignJson(artifacts[i].GetSegmentDesignArtifact(0), applied[i].first, applied[i].second);
      os << "]";
      return os.str();
   }
}

namespace
{
   // Quick checks that the edited model can be analysed: slab overhangs at mid-span and the
   // live load distribution factors of every girder (exterior girders fail here when the deck
   // no longer suits the girder layout). Returns JSON items; bReady = no check failed.
   std::string Validate(std::shared_ptr<WBFL::EAF::Broker> pBroker, bool& bReady)
   {
      bReady = true;
      std::ostringstream os;
      bool first = true;
      auto item = [&](const std::string& fields) { os << (first ? "" : ", ") << "{" << fields << "}"; first = false; };
      GET_IFACE2(pBroker, IBridge, pBridge);
      GET_IFACE2(pBroker, ILiveLoadDistributionFactors, pLLDF);
      SpanIndexType nSpans = pBridge->GetSpanCount();
      for (SpanIndexType sp = 0; sp < nSpans; sp++)
      {
         std::string where = "\"span\": " + std::to_string(sp + 1);
         try
         {
            Float64 mid = 0.5 * pBridge->GetSpanLength(sp);
            Float64 left = pBridge->GetLeftSlabOverhang(sp, mid), right = pBridge->GetRightSlabOverhang(sp, mid);
            bool ok = 0 <= left && 0 <= right;
            if (!ok) bReady = false;
            item(where + ", \"check\": \"slab overhang at mid-span\", \"status\": \"" + (ok ? "ok" : "error")
                 + "\", \"left_m\": " + Num(left) + ", \"right_m\": " + Num(right)
                 + (ok ? "" : ", \"message\": \"a girder lies outside the deck (negative overhang)\""));
         }
         catch (const WBFL::System::XBase& e)
         {
            bReady = false;
            item(where + ", \"check\": \"slab overhang at mid-span\", \"status\": \"error\", \"message\": " + JsonString(Describe(e)));
         }
         GirderIndexType nGdr = pBridge->GetGirderCount((GroupIndexType)sp);
         for (GirderIndexType g = 0; g < nGdr; g++)
         {
            std::string gw = where + ", \"girder\": \"" + std::string(1, (char)('A' + g)) + "\"";
            try
            {
               CSpanKey key(sp, g);
               Float64 dfm = pLLDF->GetMomentDistFactor(key, pgsTypes::StrengthI);
               Float64 dfv = pLLDF->GetShearDistFactor(key, pgsTypes::StrengthI);
               item(gw + ", \"check\": \"live load distribution factors\", \"status\": \"ok\", \"moment\": "
                    + Num(dfm) + ", \"shear\": " + Num(dfv));
            }
            catch (const WBFL::System::XBase& e)
            {
               bReady = false;
               item(gw + ", \"check\": \"live load distribution factors\", \"status\": \"error\", \"message\": " + JsonString(Describe(e)));
            }
            catch (CException* pEx)
            {
               bReady = false;
               TCHAR msg[512] = {};
               pEx->GetErrorMessage(msg, 512);
               std::wstring text = msg;
               if (text.empty())
                  text = L"PGSuper could not compute the distribution factors (" + std::wstring(CA2W(pEx->GetRuntimeClass()->m_lpszClassName))
                       + L"). Usually the girder spacing or deck overhang is outside the LRFD 4.6.2.2 range of applicability "
                         L"(e.g. the exterior web to curb distance de must be -1.0 to 5.5 ft).";
               pEx->Delete();
               item(gw + ", \"check\": \"live load distribution factors\", \"status\": \"error\", \"message\": " + JsonString(text));
            }
            catch (const std::exception& ex)
            {
               bReady = false;
               item(gw + ", \"check\": \"live load distribution factors\", \"status\": \"error\", \"message\": "
                    + JsonString(std::wstring(CA2W(ex.what())) + L" [" + std::wstring(CA2W(typeid(ex).name())) + L"]"));
            }
            catch (...)
            {
               bReady = false;
               item(gw + ", \"check\": \"live load distribution factors\", \"status\": \"error\", \"message\": "
                    "\"PGSuper could not compute the distribution factors (usually: girder layout or deck overhang outside the "
                    "LRFD range of applicability)\"");
            }
         }
      }
      return "[" + os.str() + "]";
   }
}

void CReportExportAgent::ModifyModel(const CString& strEdits, const CString& strOutput, const CString& strResult)
{
   AFX_MANAGE_STATE(AfxGetAppModuleState());
   namespace fs = std::filesystem;

   std::ofstream result(fs::path((LPCTSTR)strResult), std::ios::binary);
   std::vector<std::string> opLog;
   std::vector<std::string> designs;
   bool bOK = true;
   std::wstring failure;
   std::string before, after;
   DialogAutoAnswer dlg;
   try
   {
      GET_IFACE(IBridgeDescription, pBD);
      GET_IFACE(ILibraryNames, pLibNames);
      GET_IFACE(IBridge, pGeo);
      before = Summary(pBD.get(), pGeo.get());
      auto edits = ReadEdits(fs::path((LPCTSTR)strEdits));
      for (const auto& e : edits)
      {
         std::ostringstream os;
         os << "{\"line\": " << e.line << ", \"op\": " << JsonString(e.op);
         try
         {
            std::wstring note;
            if (e.op == L"design")
               designs.push_back(Design(m_pBroker, pBD.get(), e.args, note));
            else if (e.op == L"apply_refined_lldf")
            {
               // refined analysis of the model as edited so far -> Directly Input LLDFs
               auto r = RefinedLL_Run(m_pBroker, true);
               if (!RefinedLL_Apply(m_pBroker, *r, note)) throw EditError{ note };
            }
            else
               Apply(pBD.get(), pLibNames.get(), pGeo.get(), e, note);
            os << ", \"status\": \"ok\"";
            if (!note.empty()) os << ", \"note\": " << JsonString(note);
            os << "}";
         }
         catch (const EditError& err)
         {
            os << ", \"status\": \"error\", \"message\": " << JsonString(err.msg) << "}";
            bOK = false;
         }
         catch (const WBFL::System::XBase& err)
         {
            os << ", \"status\": \"error\", \"message\": " << JsonString(Describe(err)) << "}";
            bOK = false;
         }
         catch (CException* pEx)
         {
            TCHAR msg[512];
            pEx->GetErrorMessage(msg, 512);
            pEx->Delete();
            os << ", \"status\": \"error\", \"message\": " << JsonString(msg) << "}";
            bOK = false;
         }
         catch (const std::exception& err)
         {
            os << ", \"status\": \"error\", \"message\": " << JsonString(CA2W(err.what()).m_psz) << "}";
            bOK = false;
         }
         catch (...)
         {
            os << ", \"status\": \"error\", \"message\": \"unexpected exception in PGSuper\"}";
            bOK = false;
         }
         opLog.push_back(os.str());
         Trace(L"edit   " + e.op + (bOK ? L" ok" : L" FAILED"));
         if (!bOK) break;   // all or nothing: stop at the first failure, nothing is saved
      }
      after = Summary(pBD.get(), pGeo.get());
   }
   catch (const EditError& err) { bOK = false; failure = err.msg; }
   catch (const WBFL::System::XBase& err) { bOK = false; failure = Describe(err); }
   catch (...) { bOK = false; failure = L"unexpected exception while editing"; }

   std::string validation = "null";
   bool bReady = false;
   if (bOK)
   {
      try { validation = Validate(m_pBroker, bReady); }
      catch (...) { validation = "[{\"check\": \"validation\", \"status\": \"error\", \"message\": \"validation could not run\"}]"; }
      Trace(bReady ? L"validate ok" : L"validate: problems found");
   }

   bool bSaved = false;
   if (bOK && !strOutput.IsEmpty())   // no output file: inspection only, nothing saved
   {
      CEAFDocument* pDoc = EAFGetDocument();
      bSaved = pDoc && pDoc->OnSaveDocument(strOutput);
      if (bSaved)
         pDoc->SetModifiedFlag(FALSE);
      else
         failure = L"PGSuper could not save " + std::wstring((LPCTSTR)strOutput);
   }

   // PGSuper's own messages about the model (status center), e.g. inconsistent input
   std::ostringstream status;
   status << "[";
   try
   {
      GET_IFACE(IEAFStatusCenter, pStatus);
      IndexType n = pStatus->Count();
      for (IndexType i = 0; i < n; i++)
      {
         auto item = pStatus->GetByIndex(i);
         auto sev = pStatus->GetSeverity(item);
         status << (i ? ", " : "") << "{\"severity\": \""
                << (sev == WBFL::EAF::StatusSeverityType::Error ? "error" : sev == WBFL::EAF::StatusSeverityType::Warning ? "warning" : "information")
                << "\", \"message\": " << JsonString(item->GetDescription()) << "}";
      }
   }
   catch (...) {}
   status << "]";

   result << "{\"saved\": " << (bSaved ? "true" : "false")
          << ", \"analysis_ready\": " << (bOK && bReady ? "true" : "false")
          << ", \"validation\": " << validation
          << ", \"output\": " << JsonString((LPCTSTR)strOutput)
          << ", \"error\": " << JsonString(failure)
          << ", \"edits\": [";
   for (size_t i = 0; i < opLog.size(); i++)
      result << (i ? ", " : "") << opLog[i];
   result << "], \"dialogs\": " << JsonString(dlg.Note())
          << ", \"status_items\": " << status.str()
          << ", \"before\": " << (before.empty() ? "null" : before)
          << ", \"after\": " << (after.empty() ? "null" : after) << ", \"designs\": [";
   for (size_t i = 0; i < designs.size(); i++)
      result << (i ? ", " : "") << designs[i].substr(1, designs[i].size() - 2);   // flatten the per-op arrays
   result << "]}\n";
}
