#include "stdafx.h"
#include "ReportExportAgent.h"
#include "ExportUtil.h"

#include <EAF\EAFGraphManager.h>
#include <EAF\EAFGraphChildFrame.h>
#include <EAF\EAFGraphView.h>
#include <EAF\EAFAutoCalcGraphBuilder.h>
#include <EAF\EAFViewController.h>
#include <EAF\EAFProgress.h>
#include <EAF\AutoProgress.h>
#include <IFace\Views.h>
#include <IFace\Bridge.h>
#include <IFace\PointOfInterest.h>
#include <Graphing\GraphXY.h>

#include <Graphs\GirderGraphBuilderBase.h>
#include <Graphs\SegmentGraphBuilderBase.h>
#include <Graphs\AnalysisResultsGraphBuilder.h>
#include <Graphs\SegmentAnalysisResultsGraphBuilder.h>
#include <Graphs\GirderPropertiesGraphBuilder.h>
#include <Graphs\EffectivePrestressGraphBuilder.h>
#include <Graphs\FinishedElevationGraphBuilder.h>
#include <Graphs\ConcretePropertyGraphBuilder.h>
#include <Graphs\DeflectionHistoryGraphBuilder.h>
#include <Graphs\StabilityGraphBuilder.h>
#include <Graphs\StressHistoryGraphBuilder.h>

#include <atlimage.h>
#include <algorithm>
#include <functional>
#include <numeric>

namespace
{
   // Read-only access to the protected GraphXY that each PGSuper graph builder draws.
   // Forming &Derived::member is allowed for a protected member; no object is created.
   template <class T>
   struct GraphAccess : public T
   {
      static const WBFL::Graphing::GraphXY* Get(WBFL::Graphing::GraphBuilder& builder)
      {
         T* p = dynamic_cast<T*>(&builder);
         if (!p)
            return nullptr;
         WBFL::Graphing::GraphXY T::* pm = &GraphAccess::m_Graph;
         return &(p->*pm);
      }
   };

   const WBFL::Graphing::GraphXY* FindGraphXY(WBFL::Graphing::GraphBuilder& b)
   {
      if (auto g = GraphAccess<CGirderGraphBuilderBase>::Get(b)) return g;
      if (auto g = GraphAccess<CSegmentGraphBuilderBase>::Get(b)) return g;
      if (auto g = GraphAccess<CConcretePropertyGraphBuilder>::Get(b)) return g;
      if (auto g = GraphAccess<CDeflectionHistoryGraphBuilder>::Get(b)) return g;
      if (auto g = GraphAccess<CStabilityGraphBuilder>::Get(b)) return g;
      if (auto g = GraphAccess<CStressHistoryGraphBuilder>::Get(b)) return g;
      return nullptr;
   }

   std::string Csv(const std::wstring& s)
   {
      std::string u = ToUtf8(s), out = "\"";
      for (char c : u)
         out += (c == '"') ? std::string("\"\"") : std::string(1, c);
      return out + "\"";
   }

   std::wstring Text(LPCTSTR s) { return s ? std::wstring(s) : std::wstring(); }

   // series,x,y (long format) with the graph and axis titles as '#' comment lines
   void WriteCsv(const WBFL::Graphing::GraphXY& g, const std::filesystem::path& file, size_t& nSeries, size_t& nPoints)
   {
      std::ofstream f(file, std::ios::binary);
      f << "\xEF\xBB\xBF";
      f << "# title: " << ToUtf8(Text(g.GetTitle())) << "\n";
      f << "# subtitle: " << ToUtf8(Text(g.GetSubtitle())) << "\n";
      f << "# x: " << ToUtf8(Text(g.GetXAxisTitle())) << "\n";
      f << "# y: " << ToUtf8(Text(g.GetYAxisTitle())) << "\n";
      f << "series,x,y\n";
      nSeries = nPoints = 0;
      char b[64];
      for (IndexType cookie : g.GetCookies())
      {
         std::_tstring label; int pen = 0, width = 0; COLORREF color = 0;
         g.GetDataSeriesData(cookie, &label, &pen, &width, &color);
         WBFL::Graphing::GraphXY::DataSeries pts;
         g.GetDataSeriesPoints(cookie, &pts);
         std::string name = Csv(label);
         for (auto& p : pts)
         {
            sprintf_s(b, ",%.10g,%.10g\n", p.X(), p.Y());
            f << name << b;
         }
         nSeries++;
         nPoints += pts.size();
      }
   }

   // Draw the graph off screen at the view's size and save it as PNG
   bool RenderPng(CEAFGraphView* pView, WBFL::Graphing::GraphBuilder& builder, const std::filesystem::path& file)
   {
      CRect rc;
      pView->GetClientRect(&rc);
      if (rc.Width() <= 0 || rc.Height() <= 0)
         return false;
      CClientDC viewDC(pView);
      CDC mem;
      mem.CreateCompatibleDC(&viewDC);
      CBitmap bmp;
      bmp.CreateCompatibleBitmap(&viewDC, rc.Width(), rc.Height());
      CBitmap* pOld = mem.SelectObject(&bmp);
      mem.FillSolidRect(rc, RGB(255, 255, 255));
      builder.DrawGraph(pView, &mem);
      mem.SelectObject(pOld);
      CImage img;
      img.Attach((HBITMAP)bmp.Detach());
      return SUCCEEDED(img.Save(file.c_str(), Gdiplus::ImageFormatPNG));
   }

   CEAFGraphChildFrame* ActiveGraphFrame()
   {
      CMDIFrameWnd* pMain = dynamic_cast<CMDIFrameWnd*>(EAFGetApp()->m_pMainWnd);
      if (!pMain)
         return nullptr;
      return dynamic_cast<CEAFGraphChildFrame*>(pMain->MDIGetActive());
   }


   // Control IDs of PGSuper's graph control bars (PGSuper\Graphs\resource.h, version_9.0.4.1)
   enum GraphControlID
   {
      IDC_EVENT = 1096, IDC_PRECAST_SEGMENT = 3009, IDC_CLOSURE_JOINT = 3010, IDC_DECK = 3011,
      IDC_GROUP = 3012, IDC_GIRDER = 3013, IDC_SEGMENT = 3014, IDC_GRAPH_TYPE = 3017, IDC_ACTION = 3020,
      IDC_POI = 3023, IDC_TOPDECK = 3024, IDC_BOTTOMDECK = 3025, IDC_TOPGIRDER = 3026, IDC_BOTTOMGIRDER = 3027,
      IDC_PROPERTY = 3030, IDC_INTERVALS = 3045, IDC_FC = 3034, IDC_EC = 3035, IDC_SH = 3036, IDC_CR = 3037,
      IDC_STRESS = 3046, IDC_FORCE = 3047, IDC_MODE = 3048, IDC_DROP_LIST = 3050, IDC_SELECT_LIST = 3051,
   };

   // Drives a graph's control bar the way a user does: set a combo box / radio button /
   // list box, then send the notification the control itself would send. (PGSuper's graph
   // view controllers can't be used: PGSuperGraphing.dll has no ATL module, so creating
   // one dereferences a null _pAtlModule.)
   class ControlBar
   {
   public:
      explicit ControlBar(CWnd* pBar) : m_hBar(pBar ? pBar->GetSafeHwnd() : nullptr) {}
      bool IsValid() const { return m_hBar != nullptr; }
      HWND Item(int id) const { return m_hBar ? ::GetDlgItem(m_hBar, id) : nullptr; }
      bool Has(int id) const { return Item(id) != nullptr; }
      int Count(int id) const { HWND h = Item(id); return h ? (int)::SendMessage(h, CB_GETCOUNT, 0, 0) : 0; }
      std::wstring Text(int id, int i) const
      {
         HWND h = Item(id);
         int n = h ? (int)::SendMessage(h, CB_GETLBTEXTLEN, i, 0) : -1;
         if (n < 0) return std::wstring();
         std::wstring t(n + 1, L'\0');
         ::SendMessage(h, CB_GETLBTEXT, i, (LPARAM)t.data());
         t.resize(n);
         return t;
      }
      int Find(int id, const wchar_t* part) const
      {
         for (int i = 0; i < Count(id); i++)
            if (Text(id, i).find(part) != std::wstring::npos) return i;
         return -1;
      }
      void Select(int id, int i)
      {
         HWND h = Item(id);
         if (!h || i < 0 || Count(id) <= i) throw std::runtime_error("graph control not available");
         ::SendMessage(h, CB_SETCURSEL, i, 0);
         Notify(id, CBN_SELCHANGE, h);
      }
      void Click(int id, std::initializer_list<int> group)
      {
         HWND h = Item(id);
         if (!h || !::IsWindowEnabled(h)) throw std::runtime_error("graph option not available");
         for (int g : group)
            if (HWND hg = Item(g)) ::SendMessage(hg, BM_SETCHECK, BST_UNCHECKED, 0);
         ::SendMessage(h, BM_SETCHECK, BST_CHECKED, 0);
         Notify(id, BN_CLICKED, h);
      }
      void SelectAll(int id)
      {
         HWND h = Item(id);
         if (!h) return;
         LONG style = ::GetWindowLong(h, GWL_STYLE);
         if (style & (LBS_MULTIPLESEL | LBS_EXTENDEDSEL))
            ::SendMessage(h, LB_SETSEL, TRUE, -1);
         else if (0 < ::SendMessage(h, LB_GETCOUNT, 0, 0))
            ::SendMessage(h, LB_SETCURSEL, 0, 0);
         Notify(id, LBN_SELCHANGE, h);
      }
      // tick a check box if the graph offers it (no error when it doesn't)
      void Check(int id)
      {
         HWND h = Item(id);
         if (!h || !::IsWindowEnabled(h)) return;
         ::SendMessage(h, BM_SETCHECK, BST_CHECKED, 0);
         Notify(id, BN_CLICKED, h);
      }
      // multi-select list box with nothing selected (e.g. intervals): select everything
      void SelectAllIfNone(int id)
      {
         HWND h = Item(id);
         wchar_t cls[32] = {};
         if (!h || !::GetClassNameW(h, cls, _countof(cls)) || _wcsicmp(cls, L"ListBox") != 0) return;
         if ((::GetWindowLong(h, GWL_STYLE) & (LBS_MULTIPLESEL | LBS_EXTENDEDSEL)) && ::SendMessage(h, LB_GETSELCOUNT, 0, 0) == 0)
            SelectAll(id);
      }
      // span/girder(/segment) combos; group before girder because it refills the girder list
      void SelectSegment(const CSegmentKey& k)
      {
         if (Has(IDC_GROUP)) Select(IDC_GROUP, (int)k.groupIndex);
         if (Has(IDC_GIRDER)) Select(IDC_GIRDER, (int)k.girderIndex);
         if (Has(IDC_SEGMENT) && k.segmentIndex != INVALID_INDEX && k.segmentIndex < (SegmentIndexType)Count(IDC_SEGMENT))
            Select(IDC_SEGMENT, (int)k.segmentIndex);
      }
   private:
      void Notify(int id, int code, HWND h) { ::SendMessage(m_hBar, WM_COMMAND, MAKEWPARAM(id, code), (LPARAM)h); }
      HWND m_hBar;
   };

   const wchar_t* PropertyName(int p)
   {
      static const wchar_t* names[] = { L"Height", L"Area", L"Moment of Inertia", L"Centroid", L"Section Modulus",
         L"Area of Prestress", L"Kern Point", L"Strand Eccentricity", L"Tendon Eccentricity", L"Tendon Profile",
         L"Effective Flange Width", L"fc", L"Ec" };
      return (0 <= p && p < (int)_countof(names)) ? names[p] : L"Property";
   }

   // Diagnostics: log access violations (module + offset and the call stack) to the trace
   std::wstring ModuleOffset(void* addr)
   {
      HMODULE hMod = nullptr;
      wchar_t path[MAX_PATH] = L"?";
      if (::GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                               (LPCWSTR)addr, &hMod) && hMod)
         ::GetModuleFileNameW(hMod, path, MAX_PATH);
      wchar_t b[64];
      swprintf_s(b, L"+0x%llx", (unsigned long long)((BYTE*)addr - (BYTE*)hMod));
      return std::filesystem::path(path).filename().wstring() + b;
   }

   LONG CALLBACK LogAccessViolation(EXCEPTION_POINTERS* p)
   {
      static int count = 0;
      if (p->ExceptionRecord->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && count++ < 3)
      {
         wchar_t b[128];
         swprintf_s(b, L"ACCESS VIOLATION %s address 0x%llx at ", p->ExceptionRecord->ExceptionInformation[0] ? L"writing" : L"reading",
                    (unsigned long long)p->ExceptionRecord->ExceptionInformation[1]);
         Trace(b + ModuleOffset(p->ExceptionRecord->ExceptionAddress));
         void* frames[40];
         USHORT n = ::RtlCaptureStackBackTrace(0, 40, frames, nullptr);
         for (USHORT i = 0; i < n; i++)
            Trace(L"   " + ModuleOffset(frames[i]));
      }
      return EXCEPTION_CONTINUE_SEARCH;
   }
}

void CReportExportAgent::ExportGraphs(const CString& strFolder)
{
   // Views, frames and menus belong to the application: create them in its module state,
   // as the GUI does when a graph is opened from the menu (our DLL state has no main window).
   AFX_MANAGE_STATE(AfxGetAppModuleState());

   PVOID hVeh = ::AddVectoredExceptionHandler(1, &LogAccessViolation);
   namespace fs = std::filesystem;
   fs::path root((LPCTSTR)strFolder);
   std::error_code ec;
   fs::create_directories(root, ec);

   std::ofstream index(root / L"index.json", std::ios::binary);
   index << "[\n";
   index.flush();
   bool first = true;
   auto log = [&](const std::wstring& graph, const std::wstring& target, const std::wstring& png, const std::wstring& csv,
                  size_t nSeries, size_t nPoints, const std::wstring& status, const std::wstring& error, const std::wstring& note)
   {
      index << (first ? "" : ",\n") << "  {\"graph\": " << JsonString(graph) << ", \"target\": " << JsonString(target)
            << ", \"png\": " << JsonString(png) << ", \"csv\": " << JsonString(csv)
            << ", \"series\": " << nSeries << ", \"points\": " << nPoints
            << ", \"status\": " << JsonString(status) << ", \"error\": " << JsonString(error);
      if (!note.empty())
         index << ", \"note\": " << JsonString(note);
      index << "}";
      index.flush();
      first = false;
   };

   GET_IFACE(IEAFGraphManager, pGraphMgr);
   GET_IFACE(IViews, pViews);
   GET_IFACE(IBridge, pBridge);
   GET_IFACE(IPointOfInterest, pPoi);

   std::vector<CGirderKey> girders;
   std::vector<CSegmentKey> segments;
   GroupIndexType nGroups = pBridge->GetGirderGroupCount();
   for (GroupIndexType grp = 0; grp < nGroups; grp++)
   {
      GirderIndexType nGdr = pBridge->GetGirderCount(grp);
      for (GirderIndexType gdr = 0; gdr < nGdr; gdr++)
      {
         CGirderKey key(grp, gdr);
         girders.push_back(key);
         SegmentIndexType nSeg = pBridge->GetSegmentCount(key);
         for (SegmentIndexType seg = 0; seg < nSeg; seg++)
            segments.emplace_back(grp, gdr, seg);
      }
   }
   auto segLabel = [&](const CSegmentKey& k)
   {
      std::wstring s = GirderLabel(k);
      if (1 < pBridge->GetSegmentCount(k))
         s += L" Segment " + std::to_wstring(k.segmentIndex + 1);
      return s;
   };

   std::vector<std::wstring> only;
   {
      wchar_t f[2048];
      DWORD n = ::GetEnvironmentVariableW(L"PGSUPER_EXPORT_GRAPHS_ONLY", f, _countof(f));
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
   auto names = pGraphMgr->GetGraphNames();
   Trace(L"graphs registered: " + std::to_wstring(names.size()));

   for (const auto& name : names)
   {
      if (!only.empty() && std::none_of(only.begin(), only.end(),
            [&name](const std::wstring& o) { return name.find(o) != std::wstring::npos; }))
         continue;

      Trace(L"open   " + name);
      std::wstring openNote;
      try
      {
         DialogAutoAnswer dlg;
         pViews->CreateGraphView(name.c_str(), nullptr);
         openNote = dlg.Note();
      }
      catch (...)
      {
         log(name, L"", L"", L"", 0, 0, L"error", L"could not open the graph view", openNote);
         continue;
      }
      CEAFGraphChildFrame* pFrame = ActiveGraphFrame();
      CEAFGraphView* pView = pFrame ? pFrame->GetGraphView() : nullptr;
      CEAFGraphBuilderBase* pGraphBuilder = pView ? dynamic_cast<CEAFGraphBuilderBase*>(pView->GetGraphBuilder().get()) : nullptr;
      if (!pView || !pView->GetGraphBuilder())
      {
         log(name, L"", L"", L"", 0, 0, L"error", L"graph view did not open", openNote);
         if (pFrame) pFrame->SendMessage(WM_CLOSE);
         continue;
      }
      ControlBar ui(pGraphBuilder ? pGraphBuilder->GetGraphControlWindow() : nullptr);
      WBFL::Graphing::GraphBuilder* pB = pView->GetGraphBuilder().get();

      // a fixed, large drawing area: 1600 x 1000 frame (the graph controls take part of it)
      pFrame->MDIRestore();
      pFrame->SetWindowPos(nullptr, 0, 0, 1600, 1000, SWP_NOZORDER | SWP_NOMOVE | SWP_NOACTIVATE);
      pFrame->RecalcLayout();

      // settings for each exported graph, applied through the graph's control bar
      std::vector<std::pair<std::wstring, std::function<void()>>> targets;
      auto actions = [&](const std::wstring& where, const CSegmentKey& k)
      {
         // every loading in the last interval, one graph per action (moment, shear, ...).
         // "Plot by Loading": the drop list picks the interval, the list holds the loadings
         // ("Plot by Interval" would instead draw one loading with a series per interval).
         ui.SelectSegment(k);
         for (int a = 0; a < ui.Count(IDC_ACTION); a++)
            targets.emplace_back(where + L" - " + ui.Text(IDC_ACTION, a), [&ui, k, a]
            {
               ui.SelectSegment(k);
               int mode = ui.Find(IDC_MODE, L"Loading");
               if (0 <= mode) ui.Select(IDC_MODE, mode);
               ui.Select(IDC_ACTION, a);
               if (0 < ui.Count(IDC_DROP_LIST)) ui.Select(IDC_DROP_LIST, ui.Count(IDC_DROP_LIST) - 1);
               ui.SelectAll(IDC_SELECT_LIST);
            });
      };
      auto midspan = [&](const std::wstring& where, const CSegmentKey& k)
      {
         targets.emplace_back(where + L" 0.5L", [&ui, k]
         {
            ui.SelectSegment(k);
            int i = ui.Find(IDC_POI, L"0.5L");
            ui.Select(IDC_POI, 0 <= i ? i : ui.Count(IDC_POI) / 2);
         });
      };
      auto combo = [&](const std::wstring& where, const CSegmentKey& k, int id)
      {
         ui.SelectSegment(k);
         for (int i = 0; i < ui.Count(id); i++)
            targets.emplace_back(where + L" - " + ui.Text(id, i), [&ui, k, id, i]
               { ui.SelectSegment(k); ui.Select(id, i); ui.SelectAllIfNone(IDC_INTERVALS); });
      };

      if (!ui.IsValid())
         targets.emplace_back(L"Default", [] {});
      else if (dynamic_cast<CAnalysisResultsGraphBuilder*>(pB))
         for (const auto& k : girders) actions(GirderLabel(k), CSegmentKey(k, INVALID_INDEX));
      else if (dynamic_cast<CSegmentAnalysisResultsGraphBuilder*>(pB))
         for (const auto& k : segments) actions(segLabel(k), k);
      else if (dynamic_cast<CGirderPropertiesGraphBuilder*>(pB))
         for (const auto& k : girders) combo(GirderLabel(k), CSegmentKey(k, INVALID_INDEX), IDC_PROPERTY);
      else if (dynamic_cast<CFinishedElevationGraphBuilder*>(pB))
      {
         for (const auto& k : girders) combo(GirderLabel(k), CSegmentKey(k, INVALID_INDEX), IDC_GRAPH_TYPE);
         // the differential graph starts with nothing ticked: plot finished deck and top of girder
         const int IDC_FINISHED_DECK = 3023, IDC_FINISHED_GIRDER_TOP = 3025;
         for (auto& t : targets)
            if (t.first.find(L"ifferential") != std::wstring::npos)
               t.second = [&ui, f = t.second, IDC_FINISHED_DECK, IDC_FINISHED_GIRDER_TOP]
                  { f(); ui.Check(IDC_FINISHED_DECK); ui.Check(IDC_FINISHED_GIRDER_TOP); };
      }
      else if (dynamic_cast<CStabilityGraphBuilder*>(pB))
         for (const auto& k : segments) combo(segLabel(k), k, IDC_EVENT);
      else if (dynamic_cast<CStressHistoryGraphBuilder*>(pB))
      {
         for (const auto& k : girders) midspan(GirderLabel(k), CSegmentKey(k, INVALID_INDEX));
         for (auto& t : targets)   // plot every stress location the graph offers
            t.second = [&ui, f = t.second] { f(); for (int id : { IDC_TOPGIRDER, IDC_BOTTOMGIRDER, IDC_TOPDECK, IDC_BOTTOMDECK }) ui.Check(id); };
      }
      else if (dynamic_cast<CDeflectionHistoryGraphBuilder*>(pB))
         for (const auto& k : girders) midspan(GirderLabel(k), CSegmentKey(k, INVALID_INDEX));
      else if (dynamic_cast<CEffectivePrestressGraphBuilder*>(pB))
      {
         for (const auto& k : girders)
            for (auto [id, what] : { std::pair{ (int)IDC_STRESS, L"Stress" }, std::pair{ (int)IDC_FORCE, L"Force" } })
               targets.emplace_back(GirderLabel(k) + L" - " + what, [&ui, k, id = id]
               {
                  ui.SelectSegment(CSegmentKey(k, INVALID_INDEX));
                  ui.Click(id, { IDC_STRESS, IDC_FORCE });
                  ui.SelectAllIfNone(IDC_INTERVALS);
               });
      }
      else if (dynamic_cast<CConcretePropertyGraphBuilder*>(pB))
      {
         const std::pair<int, const wchar_t*> types[] = { {IDC_FC, L"fc"}, {IDC_EC, L"Ec"}, {IDC_SH, L"Shrinkage"}, {IDC_CR, L"Creep"} };
         for (const auto& k : segments)
            for (auto [id, what] : types)
               targets.emplace_back(segLabel(k) + L" - " + what, [&ui, k, id = id]
               {
                  ui.Click(IDC_PRECAST_SEGMENT, { IDC_PRECAST_SEGMENT, IDC_CLOSURE_JOINT, IDC_DECK });
                  ui.SelectSegment(k);
                  ui.Click(id, { IDC_FC, IDC_EC, IDC_SH, IDC_CR });
               });
         for (auto [id, what] : types)
            targets.emplace_back(std::wstring(L"Deck - ") + what, [&ui, id = id]
            {
               ui.Click(IDC_DECK, { IDC_PRECAST_SEGMENT, IDC_CLOSURE_JOINT, IDC_DECK });
               ui.Click(id, { IDC_FC, IDC_EC, IDC_SH, IDC_CR });
            });
      }
      else
         targets.emplace_back(L"Default", [] {});
      fs::path dir = root / SafeName(name);
      fs::create_directories(dir, ec);
      for (auto& [label, apply] : targets)
      {
         Trace(L"graph  " + name + L" / " + label);
         pProgress->UpdateMessage((name + L" - " + label).c_str());
         DialogAutoAnswer dlg;
         std::wstring note = openNote;
         auto withDialogs = [&] { std::wstring n = dlg.Note(); return note.empty() ? n : n.empty() ? note : note + L"; " + n; };
         try
         {
            apply();
            auto& pBuilder = pView->GetGraphBuilder();
            if (auto pAuto = dynamic_cast<CEAFAutoCalcGraphBuilder*>(pBuilder.get()))
               pAuto->ForceUpdate();
            else if (auto pBase = dynamic_cast<CEAFGraphBuilderBase*>(pBuilder.get()))
               pBase->Update();

            fs::path base = dir / SafeName(label);
            fs::path png = base; png += L".png";
            fs::path csv = base; csv += L".csv";
            bool bValid = pBuilder->CanPrint() ? true : false;
            // the legend grows with the number of series (about 2 per row): make the image
            // taller so the plot area keeps its size (e.g. ~60 loadings on an analysis graph)
            const WBFL::Graphing::GraphXY* pXY = FindGraphXY(*pBuilder);
            IndexType nLegend = pXY ? pXY->GetDataSeriesCount() : 0;
            int height = 1000 + (16 < nLegend ? (int)std::min<IndexType>((nLegend + 1) / 2 * 20, 1400) : 0);
            pFrame->SetWindowPos(nullptr, 0, 0, 1600, height, SWP_NOZORDER | SWP_NOMOVE | SWP_NOACTIVATE);
            pFrame->RecalcLayout();
            bool bPng = RenderPng(pView, *pBuilder, png);
            size_t nSeries = 0, nPoints = 0;
            bool bCsv = false;
            if (const WBFL::Graphing::GraphXY* pGraph = FindGraphXY(*pBuilder))
            {
               WriteCsv(*pGraph, csv, nSeries, nPoints);
               bCsv = true;
            }
            std::wstring status = !bValid ? L"no data" : (nSeries == 0 && bCsv) ? L"empty" : L"ok";
            log(name, label, bPng ? png.lexically_relative(root).wstring() : L"",
                bCsv ? csv.lexically_relative(root).wstring() : L"", nSeries, nPoints, status,
                bPng ? L"" : L"PNG not written", withDialogs());
         }
         catch (const WBFL::System::XBase& e)
         {
            log(name, label, L"", L"", 0, 0, L"error", Describe(e), withDialogs());
         }
         catch (CException* pEx)
         {
            TCHAR msg[512];
            pEx->GetErrorMessage(msg, 512);
            pEx->Delete();
            log(name, label, L"", L"", 0, 0, L"error", msg, withDialogs());
         }
         catch (const std::exception& e)
         {
            log(name, label, L"", L"", 0, 0, L"skipped", CA2W(e.what()).m_psz, withDialogs());
         }
         catch (...)
         {
            log(name, label, L"", L"", 0, 0, L"error", L"unknown exception", withDialogs());
         }
      }
      pFrame->SendMessage(WM_CLOSE);
   }
   index << "\n]\n";
   ::RemoveVectoredExceptionHandler(hVeh);
}
