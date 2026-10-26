#include "stdafx.h"
#include "ReportExportCommandLineInfo.h"

CReportExportCommandLineInfo::CReportExportCommandLineInfo() :
   CPGSBaseCommandLineInfo()
{
}

void CReportExportCommandLineInfo::ParseParam(LPCTSTR lpszParam, BOOL bFlag, BOOL bLast)
{
   bool bMyParameter = false;
   CString strParam(lpszParam);

   if (bFlag)
   {
      // /ExportReports=<folder>: the folder rides on the flag because PGSuper's core test
      // agent parses the command line first and rejects any extra positional parameter
      if (m_Count == 0 && strParam.Left(14).CompareNoCase(_T("ExportReports=")) == 0)
      {
         m_bExportReports = true;
         m_OutputFolder = strParam.Mid(14);
         m_OutputFolder.Trim(_T("\" "));   // strip surrounding quotes/spaces
         m_bCommandLineMode = true;
         m_bShowSplash = FALSE;
         bMyParameter = true;
      }
   }

   if (!bMyParameter)
   {
      CPGSBaseCommandLineInfo::ParseParam(lpszParam, bFlag, bLast);
   }

   m_Count++;
}

CString CReportExportCommandLineInfo::GetUsageMessage()
{
   return CString(_T("/ExportReports=outputfolder filename.pgs"));
}

LPCTSTR CReportExportCommandLineInfo::GetAppName() const
{
   return _T("PGSuper");
}
