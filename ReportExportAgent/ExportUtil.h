#pragma once

// Helpers shared by the report and graph export (one private copy per source file).
#include <EAF\EAFApp.h>
#include <PsgLib\Keys.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace
{
   std::string ToUtf8(const std::wstring& s)
   {
      if (s.empty())
         return std::string();
      int n = ::WideCharToMultiByte(CP_UTF8, 0, s.data(), (int)s.size(), nullptr, 0, nullptr, nullptr);
      std::string out(n, '\0');
      ::WideCharToMultiByte(CP_UTF8, 0, s.data(), (int)s.size(), out.data(), n, nullptr, nullptr);
      return out;
   }

   std::string JsonString(const std::wstring& s)
   {
      std::string u = ToUtf8(s), out = "\"";
      for (char c : u)
      {
         switch (c)
         {
         case '"':  out += "\\\""; break;
         case '\\': out += "\\\\"; break;
         case '\n': out += "\\n"; break;
         case '\r': out += "\\r"; break;
         case '\t': out += "\\t"; break;
         default:
            if ((unsigned char)c < 0x20) { char b[8]; sprintf_s(b, "\\u%04x", c); out += b; }
            else out += c;
         }
      }
      return out + "\"";
   }

   // file-system safe name
   std::wstring SafeName(const std::wstring& s)
   {
      std::wstring out;
      for (wchar_t c : s)
         out += (wcschr(L"<>:\"/\\|?*", c) || c < 32) ? L'_' : c;
      while (!out.empty() && (out.back() == L' ' || out.back() == L'.'))
         out.pop_back();
      return out.empty() ? std::wstring(L"report") : out;
   }

   std::wstring GirderLabel(const CGirderKey& key)
   {
      std::wostringstream os;
      os << L"Span " << (key.groupIndex + 1) << L" Girder " << (wchar_t)(L'A' + key.girderIndex);
      return os.str();
   }

   // exception text; many WBFL/PGSuper exceptions carry no message, so add the type and reason code
   std::wstring Describe(const WBFL::System::XBase& e)
   {
      std::wstring msg = e.GetErrorMessage();
      std::wstring type = (LPCWSTR)CA2W(typeid(e).name());
      if (type.rfind(L"class ", 0) == 0) type.erase(0, 6);
      std::wstring detail = type + L" (reason " + std::to_wstring(e.GetReason()) + L")";
      return msg.empty() ? detail : msg + L" [" + detail + L"]";
   }

   // Diagnostic trace: %TEMP%\ReportExportAgent.log (load, command line, results)
   void Trace(const std::wstring& msg)
   {
      wchar_t tmp[MAX_PATH];
      ::GetTempPathW(MAX_PATH, tmp);
      std::ofstream f(std::filesystem::path(tmp) / L"ReportExportAgent.log", std::ios::app | std::ios::binary);
      SYSTEMTIME st; ::GetLocalTime(&st);
      char ts[32]; sprintf_s(ts, "%02d:%02d:%02d ", st.wHour, st.wMinute, st.wSecond);
      f << ts << ToUtf8(msg) << "\n";
   }

   // Answers any dialog box that PGSuper opens while a report is being built
   // (e.g. the Report Options dialog of the Bearing Spec Check Report). Without this
   // the modal dialog sits hidden behind BridgeLink's busy window and the export hangs.
   // IDOK accepts PGSuper's default selection; a dialog that keeps coming back (failed
   // validation) is cancelled instead. Titles are collected for index.json.
   // Only dialogs created while the hook is installed are touched, so PGSuper's
   // modeless Progress window (created before the export loop) is left alone.
   class DialogAutoAnswer
   {
   public:
      DialogAutoAnswer()
      {
         s_pThis = this;
         m_hHook = ::SetWindowsHookExW(WH_CBT, &DialogAutoAnswer::CbtProc, nullptr, ::GetCurrentThreadId());
      }
      ~DialogAutoAnswer()
      {
         if (m_hHook)
            ::UnhookWindowsHookEx(m_hHook);
         s_pThis = nullptr;
      }
      DialogAutoAnswer(const DialogAutoAnswer&) = delete;
      DialogAutoAnswer& operator=(const DialogAutoAnswer&) = delete;

      std::wstring Note() const
      {
         std::wstring note;
         for (const auto& t : m_Answered)
            note += (note.empty() ? L"auto-answered dialog: " : L"; ") + t;
         return note;
      }

   private:
      static LRESULT CALLBACK CbtProc(int nCode, WPARAM wParam, LPARAM lParam)
      {
         if (s_pThis && nCode == HCBT_CREATEWND && IsDialog((HWND)wParam))
            s_pThis->m_Created.insert((HWND)wParam);
         else if (s_pThis && nCode == HCBT_ACTIVATE && s_pThis->m_Created.count((HWND)wParam))
            s_pThis->Answer((HWND)wParam);
         else if (s_pThis && nCode == HCBT_DESTROYWND)
            s_pThis->m_Created.erase((HWND)wParam);
         return ::CallNextHookEx(nullptr, nCode, wParam, lParam);
      }

      static bool IsDialog(HWND hWnd)
      {
         wchar_t cls[64] = {};
         ::GetClassNameW(hWnd, cls, _countof(cls));
         return wcscmp(cls, L"#32770") == 0;
      }

      static BOOL CALLBACK FindProgressBar(HWND hChild, LPARAM lParam)
      {
         wchar_t cls[64] = {};
         ::GetClassNameW(hChild, cls, _countof(cls));
         if (_wcsicmp(cls, L"msctls_progress32") == 0)
         {
            *(bool*)lParam = true;
            return FALSE;
         }
         return TRUE;
      }

      void Answer(HWND hWnd)
      {
         wchar_t title[256] = {};
         ::GetWindowTextW(hWnd, title, _countof(title));

         // never touch a progress window: "cancelling" PGSuper's progress dialog asks
         // "Are you sure you want to cancel?" and the two would loop forever
         bool bProgress = false;
         ::EnumChildWindows(hWnd, &DialogAutoAnswer::FindProgressBar, (LPARAM)&bProgress);
         if (bProgress)
            return;

         // loop breaker: a run should never need this many answers; stop and leave the
         // dialog up (the Python harness records it and closes BridgeLink)
         if (kMaxAnswers <= m_Answered.size())
         {
            if (kMaxAnswers == m_Answered.size())
            {
               Trace(L"dialog  too many dialogs answered; leaving the rest for the harness");
               m_Answered.push_back(L"(stopped answering after " + std::to_wstring(kMaxAnswers) + L" dialogs)");
            }
            return;
         }

         int& count = m_Seen[hWnd];
         count++;
         auto usable = [hWnd](int id) { HWND h = ::GetDlgItem(hWnd, id); return h && ::IsWindowEnabled(h); };
         int id = 0;
         if (count <= 2 && usable(IDOK))  id = IDOK;
         else if (usable(IDCANCEL))       id = IDCANCEL;
         else if (usable(IDNO))           id = IDNO;
         else if (usable(IDOK))           id = IDOK;
         else                             id = IDCANCEL;   // dialogs close on IDCANCEL even without a button

         ::PostMessageW(hWnd, WM_COMMAND, MAKEWPARAM(id, BN_CLICKED), (LPARAM)::GetDlgItem(hWnd, id));
         std::wstring what = std::wstring(*title ? title : L"(untitled)") +
            (id == IDOK ? L" [OK]" : id == IDNO ? L" [No]" : L" [Cancel]");
         m_Answered.push_back(what);
         Trace(L"dialog  " + what);
      }

      static inline DialogAutoAnswer* s_pThis = nullptr;
      HHOOK m_hHook = nullptr;
      std::map<HWND, int> m_Seen;
      std::set<HWND> m_Created;
      std::vector<std::wstring> m_Answered;
      static constexpr size_t kMaxAnswers = 20;
   };
}
