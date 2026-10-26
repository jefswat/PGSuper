///////////////////////////////////////////////////////////////////////
// ReportExportAgent - PGSuper extension agent that writes every report to HTML
// from the command line:
//
//    BridgeLink.exe /ExportReports=<output folder> <file.pgs>
///////////////////////////////////////////////////////////////////////
#pragma once

#include <guiddef.h>

// {880751F9-0928-4062-824A-D70D12C49CBF}
DEFINE_GUID(CLSID_ReportExportAgent,
   0x880751F9, 0x0928, 0x4062, 0x82, 0x4A, 0xD7, 0x0D, 0x12, 0xC4, 0x9C, 0xBF);
