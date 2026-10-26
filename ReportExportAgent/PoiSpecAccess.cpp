#include "stdafx.h"

// PGSuperReporting.dll does not export CCrackedSectionReportSpecification or its base
// CPoiReportSpecification, so their setters are defined here from the same sources
// (PGSuper\Reporting\PoiReportSpecification.cpp, CrackedSectionReportSpecification.cpp,
// tag version_9.0.4.1). The object layout is the one built into the installed DLL.
#include <Reporting\CrackedSectionReportSpecification.h>

void CPoiReportSpecification::SetPOI(const pgsPointOfInterest& poi)
{
   m_Poi = poi;
}

void CCrackedSectionReportSpecification::SetOptions(const pgsPointOfInterest& poi, bool bPositiveMoment)
{
   SetPOI(poi);
   m_bPositiveMoment = bPositiveMoment;
}
