#pragma once

#include <EAF\Agent.h>
#include <EAF\EAFUIIntegration.h>
#include "CLSID.h"

/// PGSuper extension agent that renders every registered report to HTML files.
///
///    BridgeLink.exe /ExportReports=<output folder> <file.pgs>
///
/// For each report the default report specification is created (no dialogs) and,
/// when the specification is girder/segment/span/girder-line based, it is retargeted
/// to every girder/segment/span/girder line in the bridge. Output:
///    <folder>\<report>\<target>.html   and   <folder>\index.json
class CReportExportAgent : public WBFL::EAF::Agent,
   public IEAFProcessCommandLine,
   public WBFL::EAF::IAgentReportingIntegration
{
public:
   CReportExportAgent() = default;

   // Agent
   std::_tstring GetName() const override { return _T("ReportExportAgent"); }
   bool RegisterInterfaces() override;
   bool Init() override;
   bool Reset() override;
   bool ShutDown() override;
   CLSID GetCLSID() const override;

   // IEAFProcessCommandLine
   BOOL ProcessCommandLineOptions(CEAFCommandLineInfo& cmdInfo) override;

   // IAgentReportingIntegration: registers the Refined Live Load Analysis report (RefinedLL.cpp)
   bool IntegrateWithReporting(bool bIntegrate) override;

   // refined live load analysis results as JSON (+ optional influence surface CSVs)
   void ExportRefinedLL(const CString& strFolder);

   // every graph as PNG + CSV (GraphExport.cpp); runs once BridgeLink has started up
   void ExportGraphs(const CString& strFolder);

   // apply an edits file through IBridgeDescription and save as a new .pgs (ModelEdit.cpp)
   void ModifyModel(const CString& strEdits, const CString& strOutput, const CString& strResult);

private:
   EAF_DECLARE_AGENT_DATA;

   void ExportReports(const CString& strFolder);
};
