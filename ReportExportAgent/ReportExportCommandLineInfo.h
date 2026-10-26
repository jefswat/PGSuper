#pragma once

#include <PgsExt\BaseCommandLineInfo.h>

/// Parses   /ExportReports=<output folder> <file.pgs>
/// Anything else is left to the base class and to other agents.
class CReportExportCommandLineInfo : public CPGSBaseCommandLineInfo
{
public:
   CReportExportCommandLineInfo();

   virtual void ParseParam(LPCTSTR lpszParam, BOOL bFlag, BOOL bLast) override;
   virtual CString GetUsageMessage() override;

   bool m_bExportReports = false;
   CString m_OutputFolder;

private:
   Uint32 m_Count = 0; // parameter number
   virtual LPCTSTR GetAppName() const override;

   CReportExportCommandLineInfo(const CReportExportCommandLineInfo&) = delete;
   CReportExportCommandLineInfo& operator=(const CReportExportCommandLineInfo&) = delete;
};
