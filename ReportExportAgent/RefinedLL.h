#pragma once
// Refined live load analysis (plate + eccentric beam, influence surfaces, HL-93) for PGSuper.
// Engine: ..\RefinedAnalysisAgent\fem (RfaFem / RfaLive / RfaBridge).

#include <Reporting\PGSuperChapterBuilder.h>
#include <memory>
#include <string>

namespace WBFL { namespace EAF { class Broker; } }
namespace rfa { struct Results; struct BridgeInput; }

// Build the neutral bridge description from the open PGSuper model.
bool RefinedLL_BuildInput(std::shared_ptr<WBFL::EAF::Broker> pBroker, rfa::BridgeInput& in, std::vector<std::string>& notes, std::string& error);

// Run the analysis (cached for the current model state; bForce re-runs)
std::shared_ptr<const rfa::Results> RefinedLL_Run(std::shared_ptr<WBFL::EAF::Broker> pBroker, bool bForce = false);

// Results as JSON (SI units)
std::string RefinedLL_Json(std::shared_ptr<WBFL::EAF::Broker> pBroker, const rfa::Results& r);

// Write the refined LLDFs into the bridge description as "Directly Input" factors.
// Returns false (and a message) if the results are not usable.
bool RefinedLL_Apply(std::shared_ptr<WBFL::EAF::Broker> pBroker, const rfa::Results& r, std::wstring& message);

// Register the "Refined Live Load Analysis" report
void RefinedLL_RegisterReport(std::shared_ptr<WBFL::EAF::Broker> pBroker);

class CRefinedLLChapterBuilder : public CPGSuperChapterBuilder
{
public:
   CRefinedLLChapterBuilder(bool bSelect = true) : CPGSuperChapterBuilder(bSelect) {}
   LPCTSTR GetName() const override { return TEXT("Refined Live Load Analysis"); }
   rptChapter* Build(const std::shared_ptr<const WBFL::Reporting::ReportSpecification>& pRptSpec, Uint16 level) const override;
};
