// DLL entry point: registers the agent's class factory with the EAF component system.
// PGSuper finds the DLL through ReportExport.Manifest.PGSuper (CATID_PGSuperExtensionAgent).

// Not using stdafx.h here: the interface headers below must be seen for the first time
// after <initguid.h> so their IIDs are defined in this module.
#include <targetver.h>
#include <PGSuperAll.h>
#include <afxwin.h>
#include <afxdisp.h>

// define the interface IDs used by this module
#include <initguid.h>
#include "CLSID.h"
#include "PGSuperCatCom.h"
#include <EAF\EAFUIIntegration.h>
#include <EAF\EAFStatusCenter.h>
#include <EAF\EAFReportManager.h>
#include <EAF\EAFProgress.h>
#include <IFace\Bridge.h>
#include <IFace\AnalysisResults.h>
#include <IFace\Intervals.h>
#include <IFace\PointOfInterest.h>
#include <IFace\Views.h>
#include <IFace\Project.h>
#include <IFace\DistributionFactors.h>
#include <IFace\Artifact.h>
#include <EAF\EAFGraphManager.h>
#include <IFace\Alignment.h>
#include <EAF\EAFDisplayUnits.h>
#include <WBFLGeometry_i.c>   // CLSID_Point2d (refined LL analysis)

#include "ReportExportAgent.h"
#include <EAF\ComponentModule.h>

WBFL::EAF::ComponentModule Module_;

EAF_BEGIN_OBJECT_MAP(ObjectMap)
   EAF_OBJECT_ENTRY(CLSID_ReportExportAgent, CReportExportAgent)
EAF_END_OBJECT_MAP()

class CReportExportAgentApp : public CWinApp
{
public:
   virtual BOOL InitInstance() override;
   virtual int ExitInstance() override;

   DECLARE_MESSAGE_MAP()
};

BEGIN_MESSAGE_MAP(CReportExportAgentApp, CWinApp)
END_MESSAGE_MAP()

CReportExportAgentApp theApp;

BOOL CReportExportAgentApp::InitInstance()
{
   Module_.Init(ObjectMap);
   return CWinApp::InitInstance();
}

int CReportExportAgentApp::ExitInstance()
{
   Module_.Term();
   return CWinApp::ExitInstance();
}
