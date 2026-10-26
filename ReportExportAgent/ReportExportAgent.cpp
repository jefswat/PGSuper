#include "stdafx.h"
#include "ReportExportAgent.h"
#include "ReportExportCommandLineInfo.h"

#include <EAF\EAFApp.h>
#include <EAF\EAFMainFrame.h>
#include <EAF\EAFDocument.h>
#include <EAF\EAFUtilities.h>
#include <EAF\EAFReportManager.h>
#include <IFace\Bridge.h>
#include <EAF\EAFProgress.h>
#include <EAF\AutoProgress.h>
#include <Reporting\SpanGirderReportSpecification.h>
#include <Reporting\SpanGirderBearingReportSpecification.h>
#include <Reporting\ReactionInterfaceAdapters.h>
#include <Reporting\SectionPropertiesReportSpecification.h>
#include <Reporting\MomentCapacityReportSpecification.h>
#include <Reporting\CrackedSectionReportSpecification.h>
#include <IFace\AnalysisResults.h>
#include <IFace\Intervals.h>
#include <IFace\PointOfInterest.h>
#include <Reporter\Reporter.h>
#include <Reporter\HtmlReportVisitor.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <functional>
#include <limits>
#include <map>
#include <cmath>
#include <set>
#include <sstream>
#include <vector>

#include "ExportUtil.h"
#include "RefinedLL.h"
#include "..\RefinedAnalysisAgent\fem\RfaBridge.h"

bool CReportExportAgent::RegisterInterfaces()
{
   EAF_AGENT_REGISTER_INTERFACES;
   return true;
}

bool CReportExportAgent::Init()
{
   EAF_AGENT_INIT;
   Trace(L"agent loaded (Init)");
   return true;
}

bool CReportExportAgent::Reset()
{
   EAF_AGENT_RESET;
   return true;
}

bool CReportExportAgent::ShutDown()
{
   EAF_AGENT_SHUTDOWN;
   return true;
}

CLSID CReportExportAgent::GetCLSID() const
{
   return CLSID_ReportExportAgent;
}

bool CReportExportAgent::IntegrateWithReporting(bool bIntegrate)
{
   if (bIntegrate)
   {
      RefinedLL_RegisterReport(m_pBroker);
      Trace(L"Refined Live Load Analysis report registered");
   }
   return true;
}

void CReportExportAgent::ExportRefinedLL(const CString& strFolder)
{
   namespace fs = std::filesystem;
   fs::path root((LPCTSTR)strFolder);
   std::error_code ec;
   fs::create_directories(root, ec);
   auto r = RefinedLL_Run(m_pBroker, true);
   {
      std::ofstream js(root / L"refined_lldf.json", std::ios::binary);
      js << RefinedLL_Json(m_pBroker, *r);
   }
   // kept influence surfaces as CSV: s_m, o_m, value (SI)
   for (size_t k = 0; k < r->surfaces.size(); k++)
   {
      const auto& srf = r->surfaces[k];
      std::ofstream cs(root / (L"surface_" + std::to_wstring(k) + L".csv"), std::ios::binary);
      cs << "# " << srf.label << "\n# station_m,offset_m,value\n";
      for (int i = 0; i < srf.grid.ns; i += 2)
         for (int j = 0; j < srf.grid.no; j += 2)
            cs << srf.grid.s0 + i * srf.grid.ds << "," << srf.grid.o0 + j * srf.grid.dO << "," << srf.grid.at(i, j) << "\n";
   }
   Trace(r->ok ? L"refined LL exported" : L"refined LL failed");
}

namespace
{
   // Graph views can only be created once BridgeLink has finished starting up, so the
   // graph export runs from a one-shot timer in the normal message loop (like a user
   // opening graphs from the menu) and then closes BridgeLink.
   CReportExportAgent* s_pGraphAgent = nullptr;
   CString s_GraphFolder;
   // answers startup dialogs (Tip of the Day) until the graph export starts
   DialogAutoAnswer* s_pStartupDialogs = nullptr;

   void CALLBACK GraphTimerProc(HWND, UINT, UINT_PTR idEvent, DWORD)
   {
      ::KillTimer(nullptr, idEvent);
      CEAFMainFrame* pFrame = EAFGetMainFrame();
      if (pFrame && !pFrame->IsWindowEnabled())
      {
         // a modal dialog is up (e.g. a startup tip): try again shortly
         ::SetTimer(nullptr, 0, 1000, &GraphTimerProc);
         return;
      }
      if (s_pStartupDialogs)
      {
         delete s_pStartupDialogs;   // before ExportGraphs installs its own hooks
         s_pStartupDialogs = nullptr;
      }
      if (s_pGraphAgent)
      {
         s_pGraphAgent->ExportGraphs(s_GraphFolder);
         Trace(L"graph export finished");
         s_pGraphAgent = nullptr;
      }
      if (CEAFDocument* pDoc = EAFGetDocument())
         pDoc->SetModifiedFlag(FALSE);   // nothing to save: close without prompting
      if (pFrame)
         pFrame->PostMessage(WM_CLOSE);
   }
}

BOOL CReportExportAgent::ProcessCommandLineOptions(CEAFCommandLineInfo& cmdInfo)
{
   // Re-parse the command line with our own parser; ignore anything that isn't ours
   // Two ways to ask for an export:
   //  1. environment variable PGSUPER_EXPORT_REPORTS=<folder> with "BridgeLink.exe <file.pgs>".
   //     This is the reliable route: with a single file parameter the other PGSuper
   //     extensions (e.g. XBeamRate) decline the command line, so it reaches this agent.
   //  2. /ExportReports=<folder> <file.pgs> - only works when this agent is asked before
   //     XBeamRate, which rejects any unrecognized multi-parameter command line.
   CString strFolder;
   wchar_t env[4096];
   DWORD n = ::GetEnvironmentVariableW(L"PGSUPER_EXPORT_REPORTS", env, _countof(env));
   if (0 < n && n < _countof(env))
   {
      strFolder = env;
   }

   CReportExportCommandLineInfo info;
   EAFGetApp()->ParseCommandLine(info);
   if (info.m_bExportReports)
   {
      strFolder = info.m_OutputFolder;
   }

   // PGSUPER_MODIFY_EDITS=<edits file> + PGSUPER_MODIFY_OUTPUT=<new .pgs> + PGSUPER_MODIFY_RESULT=<json>:
   // edit the model through PGSuper and save it as a new file (ModelEdit.cpp); nothing else runs
   auto envVar = [](const wchar_t* name)
   {
      wchar_t buf[4096];
      DWORD len = ::GetEnvironmentVariableW(name, buf, _countof(buf));
      return (0 < len && len < _countof(buf)) ? CString(buf) : CString();
   };
   CString strEdits = envVar(L"PGSUPER_MODIFY_EDITS");
   if (!strEdits.IsEmpty())
   {
      Trace(L"modify model: edits=[" + std::wstring((LPCTSTR)strEdits) + L"]");
      ModifyModel(strEdits, envVar(L"PGSUPER_MODIFY_OUTPUT"), envVar(L"PGSUPER_MODIFY_RESULT"));
      Trace(L"modify model finished");
      cmdInfo.m_bCommandLineMode = TRUE;   // close BridgeLink when done
      return TRUE;
   }

   // PGSUPER_REFINED_LL=<folder>: refined live load analysis -> refined_lldf.json + surface CSVs
   CString strRefined = envVar(L"PGSUPER_REFINED_LL");
   if (!strRefined.IsEmpty())
   {
      Trace(L"refined LL: folder=[" + std::wstring((LPCTSTR)strRefined) + L"]");
      ExportRefinedLL(strRefined);
      if (strFolder.IsEmpty() && envVar(L"PGSUPER_EXPORT_GRAPHS").IsEmpty())
      {
         cmdInfo.m_bCommandLineMode = TRUE;
         return TRUE;
      }
   }

   // PGSUPER_EXPORT_GRAPHS=<folder>: every graph as PNG + CSV (GraphExport.cpp)
   CString strGraphs;
   n = ::GetEnvironmentVariableW(L"PGSUPER_EXPORT_GRAPHS", env, _countof(env));
   if (0 < n && n < _countof(env))
   {
      strGraphs = env;
   }

   Trace(std::wstring(L"ProcessCommandLineOptions: cmdline=[") + EAFGetApp()->m_lpCmdLine +
         L"] folder=[" + (LPCTSTR)strFolder + L"] graphs=[" + (LPCTSTR)strGraphs + L"]");
   if (strFolder.IsEmpty() && strGraphs.IsEmpty())
      return FALSE;   // not ours

   if (!strFolder.IsEmpty())
   {
      ExportReports(strFolder);
      Trace(L"export finished");
   }
   if (!strGraphs.IsEmpty())
   {
      s_pGraphAgent = this;
      s_GraphFolder = strGraphs;
      s_pStartupDialogs = new DialogAutoAnswer;
      ::SetTimer(nullptr, 0, 500, &GraphTimerProc);
      Trace(L"graph export scheduled");
      return TRUE;   // stay open until the timer has run the graph export
   }

   cmdInfo.m_bCommandLineMode = TRUE;   // close BridgeLink when done
   return TRUE;
}

void CReportExportAgent::ExportReports(const CString& strFolder)
{
   namespace fs = std::filesystem;
   fs::path root((LPCTSTR)strFolder);
   std::error_code ec;
   fs::create_directories(root, ec);

   std::ofstream index(root / L"index.json", std::ios::binary);
   index << "[\n";
   bool first = true;
   double curX = std::numeric_limits<double>::quiet_NaN();   // x_mm of the current POI target
   auto log = [&](const std::wstring& report, const std::wstring& target, const std::wstring& spec,
                  const std::wstring& file, const std::wstring& status, const std::wstring& error,
                  const std::wstring& note = std::wstring())
   {
      index << (first ? "" : ",\n") << "  {\"report\": " << JsonString(report)
            << ", \"target\": " << JsonString(target) << ", \"spec\": " << JsonString(spec)
            << ", \"file\": " << JsonString(file) << ", \"status\": " << JsonString(status)
            << ", \"error\": " << JsonString(error);
      if (!note.empty())
         index << ", \"note\": " << JsonString(note);
      if (!std::isnan(curX))
      {
         char b[32]; sprintf_s(b, "%.3f", curX);
         index << ", \"x_mm\": " << b;
      }
      index << "}";
      index.flush();
      first = false;
   };

   GET_IFACE(IEAFReportManager, pRptMgr);
   GET_IFACE(IBridge, pBridge);

   // every girder / segment / span / girder line in the bridge
   std::vector<CGirderKey> girders;
   std::vector<CSegmentKey> segments;
   GirderIndexType maxGirders = 0;
   GroupIndexType nGroups = pBridge->GetGirderGroupCount();
   for (GroupIndexType grp = 0; grp < nGroups; grp++)
   {
      GirderIndexType nGdr = pBridge->GetGirderCount(grp);
      maxGirders = Max(maxGirders, nGdr);
      for (GirderIndexType gdr = 0; gdr < nGdr; gdr++)
      {
         CGirderKey key(grp, gdr);
         girders.push_back(key);
         SegmentIndexType nSeg = pBridge->GetSegmentCount(key);
         for (SegmentIndexType seg = 0; seg < nSeg; seg++)
            segments.emplace_back(grp, gdr, seg);
      }
   }
   SpanIndexType nSpans = pBridge->GetSpanCount();

   // points of interest for the POI-based reports (Section Properties, Moment Capacity
   // Details, Cracked Section Details). Default: span tenth points of every girder.
   // PGSUPER_EXPORT_REPORTS_POIS=all -> every POI the report dialogs offer.
   bool bAllPois = false;
   {
      wchar_t f[32];
      DWORD n = ::GetEnvironmentVariableW(L"PGSUPER_EXPORT_REPORTS_POIS", f, _countof(f));
      bAllPois = (0 < n && n < _countof(f) && _wcsicmp(f, L"all") == 0);
   }
   // negative moment only matters where girders can be continuous
   std::vector<bool> moments = (1 < nSpans) ? std::vector<bool>{true, false} : std::vector<bool>{true};
   std::vector<std::pair<std::wstring, pgsPointOfInterest>> pois;
   auto poiTargets = [&]() -> const std::vector<std::pair<std::wstring, pgsPointOfInterest>>&
   {
      if (!pois.empty())
         return pois;
      GET_IFACE(IPointOfInterest, pPoi);
      for (const auto& k : girders)
      {
         PoiList v;
         if (bAllPois)
            pPoi->GetPointsOfInterest(CSegmentKey(k.groupIndex, k.girderIndex, ALL_SEGMENTS), &v);
         else
            pPoi->GetPointsOfInterest(CSpanKey(k.groupIndex, k.girderIndex), POI_SPAN | POI_TENTH_POINTS, &v);
         int i = 0;
         for (const pgsPointOfInterest& poi : v)
         {
            std::wostringstream t;
            t << GirderLabel(k);
            Uint16 tenth = poi.IsTenthPoint(POI_SPAN);
            if (!bAllPois && 0 < tenth)
               t << L" " << (tenth - 1) / 10 << L"." << (tenth - 1) % 10 << L"L";
            else
               t << L" POI " << std::setw(3) << std::setfill(L'0') << i << L" x=" << std::fixed
                 << std::setprecision(3) << WBFL::Units::ConvertFromSysUnits(poi.GetDistFromStart(), WBFL::Units::Measure::Feet) << L" ft";
            pois.emplace_back(t.str(), poi);
            i++;
         }
      }
      return pois;
   };

   // optional filter for testing: PGSUPER_EXPORT_REPORTS_ONLY=part of name;other part
   std::vector<std::wstring> only;
   {
      wchar_t f[2048];
      DWORD n = ::GetEnvironmentVariableW(L"PGSUPER_EXPORT_REPORTS_ONLY", f, _countof(f));
      if (0 < n && n < _countof(f))
      {
         std::wstringstream ss(f);
         std::wstring item;
         while (std::getline(ss, item, L';'))
            if (!item.empty()) only.push_back(item);
      }
   }

   GET_IFACE(IEAFProgress, pProgress);
   WBFL::EAF::AutoProgress ap(pProgress);
   auto names = pRptMgr->GetReportNames(false);
   Trace(L"reports registered: " + std::to_wstring(names.size()));

   for (const auto& name : names)
   {
      if (!only.empty() && std::none_of(only.begin(), only.end(),
            [&name](const std::wstring& o) { return name.find(o) != std::wstring::npos; }))
         continue;
      std::wstring specType;
      curX = std::numeric_limits<double>::quiet_NaN();
      try
      {
         auto desc = pRptMgr->GetReportDescription(name);
         auto pSpecBuilder = pRptMgr->GetReportSpecificationBuilder(desc);
         auto pRptBuilder = pRptMgr->GetReportBuilder(name);
         std::shared_ptr<WBFL::Reporting::ReportSpecification> pSpec;
         std::wstring specNote;
         {
            DialogAutoAnswer dlg;
            pSpec = pSpecBuilder ? pSpecBuilder->CreateDefaultReportSpec(desc) : nullptr;
            specNote = dlg.Note();
         }
         if (!pSpec || !pRptBuilder)
         {
            log(name, L"", L"", L"", L"skipped", L"no default report specification", specNote);
            continue;
         }

         // retarget the specification to every applicable part of the bridge
         std::vector<std::pair<std::wstring, std::function<void()>>> targets;
         std::map<std::wstring, double> xOf;   // POI targets: distance from segment start, mm
         if (auto s = std::dynamic_pointer_cast<CSegmentReportSpecification>(pSpec))
         {
            specType = L"segment";
            for (const auto& k : segments)
            {
               std::wostringstream t;
               t << GirderLabel(k) << L" Segment " << (k.segmentIndex + 1);
               targets.emplace_back(t.str(), [s, k] { s->SetSegmentKey(k); });
            }
         }
         else if (auto s = std::dynamic_pointer_cast<CGirderReportSpecification>(pSpec))
         {
            specType = L"girder";
            for (const auto& k : girders)
               targets.emplace_back(GirderLabel(k), [s, k] { s->SetGirderKey(k); });
         }
         else if (auto s = std::dynamic_pointer_cast<CGirderLineReportSpecification>(pSpec))
         {
            specType = L"girder line";
            for (GirderIndexType g = 0; g < maxGirders; g++)
               targets.emplace_back(std::wstring(L"Girder Line ") + (wchar_t)(L'A' + g), [s, g] { s->SetGirderIndex(g); });
         }
         else if (auto s = std::dynamic_pointer_cast<CSpanReportSpecification>(pSpec))
         {
            specType = L"span";
            for (SpanIndexType sp = 0; sp < nSpans; sp++)
               targets.emplace_back(L"Span " + std::to_wstring(sp + 1), [s, sp] { s->SetSpan(sp); });
         }
         else if (auto s = std::dynamic_pointer_cast<CMultiGirderReportSpecification>(pSpec))
         {
            specType = L"multi-girder";
            targets.emplace_back(L"All Girders", [s, girders] { s->SetGirderKeys(girders); });
         }
         else if (auto s = std::dynamic_pointer_cast<CMultiViewSpanGirderReportSpecification>(pSpec))
         {
            specType = L"multi-view";
            for (const auto& k : girders)
               targets.emplace_back(GirderLabel(k), [s, k] { s->SetGirderKeys(std::vector<CGirderKey>{k}); });
         }
         else if (auto s = std::dynamic_pointer_cast<CBearingReportSpecification>(pSpec))
         {
            // every bearing of every girder (the dialog lists the same locations)
            specType = L"bearing";
            GET_IFACE(IBearingDesign, pBearingDesign);
            GET_IFACE(IIntervals, pIntervals);
            GET_IFACE(IPointOfInterest, pPoi);
            IntervalIndexType intervalIdx = pIntervals->GetLastCompositeDeckInterval();
            for (const auto& k : girders)
            {
               BearingDesignProductReactionAdapter adapter(pBearingDesign, intervalIdx, k);
               ReactionLocationIter iter = adapter.GetReactionLocations(pBridge, pPoi);
               for (iter.First(); !iter.IsDone(); iter.Next())
               {
                  ReactionLocation loc = iter.CurrentItem();
                  // PierLabel already names the face, e.g. "Abutment 1 - Ahead"
                  targets.emplace_back(GirderLabel(k) + L" - " + loc.PierLabel,
                                       [s, loc] { s->SetReactionLocation(loc); });
               }
            }
         }
         else if (auto s = std::dynamic_pointer_cast<CSectionPropertiesReportSpecification>(pSpec))
         {
            specType = L"poi";
            IntervalIndexType intervalIdx = s->GetInterval();   // PGSuper's default interval
            for (const auto& [label, poi] : poiTargets())
            {
               targets.emplace_back(label, [s, poi, intervalIdx] { s->SetOptions(poi, intervalIdx); });
               xOf[label] = WBFL::Units::ConvertFromSysUnits(poi.GetDistFromStart(), WBFL::Units::Measure::Millimeter);
            }
         }
         else if (auto s = std::dynamic_pointer_cast<CMomentCapacityReportSpecification>(pSpec))
         {
            specType = L"poi";
            for (const auto& [label, poi] : poiTargets())
               for (bool bPos : moments)
               {
                  targets.emplace_back(label + (bPos ? L" +M" : L" -M"), [s, poi, bPos] { s->SetOptions(poi, bPos); });
                  xOf[targets.back().first] = WBFL::Units::ConvertFromSysUnits(poi.GetDistFromStart(), WBFL::Units::Measure::Millimeter);
               }
         }
         else if (auto s = std::dynamic_pointer_cast<CCrackedSectionReportSpecification>(pSpec))
         {
            specType = L"poi";
            for (const auto& [label, poi] : poiTargets())
               for (bool bPos : moments)
               {
                  targets.emplace_back(label + (bPos ? L" +M" : L" -M"), [s, poi, bPos] { s->SetOptions(poi, bPos); });
                  xOf[targets.back().first] = WBFL::Units::ConvertFromSysUnits(poi.GetDistFromStart(), WBFL::Units::Measure::Millimeter);
               }
         }
         else
         {
            specType = L"default";
            targets.emplace_back(L"Default", [] {});
         }

         fs::path dir = root / SafeName(name);
         fs::create_directories(dir, ec);
         for (auto& [label, apply] : targets)
         {
            fs::path file = dir / (SafeName(label) + L".html");
            auto xi = xOf.find(label);
            curX = (xi == xOf.end()) ? std::numeric_limits<double>::quiet_NaN() : xi->second;
            Trace(L"start  " + name + L" / " + label);
            pProgress->UpdateMessage((name + L" - " + label).c_str());
            DWORD t0 = ::GetTickCount();
            DialogAutoAnswer dlg;
            auto note = [&] { std::wstring n = dlg.Note(); return specNote.empty() ? n : n.empty() ? specNote : specNote + L"; " + n; };
            try
            {
               apply();
               if (!pSpec->IsValid())
               {
                  log(name, label, specType, L"", L"skipped", L"report specification not valid for this target", note());
                  continue;
               }
               auto pReport = pRptBuilder->CreateReport(pSpec);
               std::wostringstream html;
               rptHtmlReportVisitor visitor(&html, 96, 96, rptHtmlHelper::BrowserType::Edge);
               visitor.VisitReport(pReport.get());
               std::string bytes = ToUtf8(html.str());
               std::ofstream out(file, std::ios::binary);
               out << "\xEF\xBB\xBF" << bytes;   // UTF-8 BOM so browsers read symbols correctly
               log(name, label, specType, file.lexically_relative(root).wstring(), L"ok", L"", note());
               Trace(L"done   " + std::to_wstring((::GetTickCount() - t0) / 1000.0) + L" s");
            }
            catch (const WBFL::System::XBase& e)
            {
               log(name, label, specType, L"", L"error", Describe(e), note());
            }
            catch (CException* pEx)
            {
               TCHAR msg[512];
               pEx->GetErrorMessage(msg, 512);
               pEx->Delete();
               log(name, label, specType, L"", L"error", msg, note());
            }
            catch (const std::exception& e)
            {
               log(name, label, specType, L"", L"error", CA2W(e.what()).m_psz, note());
            }
            catch (...)
            {
               log(name, label, specType, L"", L"error", L"unknown exception", note());
            }
         }
      }
      catch (...)
      {
         log(name, L"", specType, L"", L"error", L"could not create the report specification");
      }
   }

   index << "\n]\n";
}
