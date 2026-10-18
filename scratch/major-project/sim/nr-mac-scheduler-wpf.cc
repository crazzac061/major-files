/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include "nr-mac-scheduler-wpf.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/nr-mac-scheduler-ue-info-pf.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerWeightedPF");

NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerWeightedPF);

TypeId
NrMacSchedulerWeightedPF::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrMacSchedulerWeightedPF")
            .SetParent<NrMacSchedulerOfdmaPF>()
            .SetGroupName("Nr")
            .AddConstructor<NrMacSchedulerWeightedPF>()
            .AddAttribute("Weight",
                          "PF metric multiplier applied to the UEs of this scheduler",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrMacSchedulerWeightedPF::SetWeight,
                                             &NrMacSchedulerWeightedPF::GetWeight),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("SliceType",
                          "Slice served by this scheduler (0 uRLLC, 1 eMBB, 2 mMTC)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrMacSchedulerWeightedPF::SetSliceType,
                                               &NrMacSchedulerWeightedPF::GetSliceType),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("ResourceShare",
                          "Fraction of the DL symbols of every slot this scheduler may use",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NrMacSchedulerWeightedPF::SetResourceShare,
                                             &NrMacSchedulerWeightedPF::GetResourceShare),
                          MakeDoubleChecker<double>(0.0, 1.0));
    return tid;
}

NrMacSchedulerWeightedPF::NrMacSchedulerWeightedPF()
    : m_weight(1.0),
      m_sliceType(0),
      m_resourceShare(1.0)
{
    NS_LOG_FUNCTION(this);
}

NrMacSchedulerWeightedPF::~NrMacSchedulerWeightedPF()
{
    NS_LOG_FUNCTION(this);
}

void
NrMacSchedulerWeightedPF::SetWeight(double w)
{
    NS_LOG_FUNCTION(this << w);
    m_weight = (std::isfinite(w) && w >= 0.0) ? w : 1.0;
}

double
NrMacSchedulerWeightedPF::GetWeight() const
{
    return m_weight;
}

void
NrMacSchedulerWeightedPF::SetSliceType(uint8_t s)
{
    m_sliceType = s;
}

uint8_t
NrMacSchedulerWeightedPF::GetSliceType() const
{
    return m_sliceType;
}

void
NrMacSchedulerWeightedPF::SetResourceShare(double share)
{
    NS_LOG_FUNCTION(this << share);
    if (!std::isfinite(share))
    {
        share = 1.0;
    }
    m_resourceShare = std::min(1.0, std::max(0.01, share));
}

double
NrMacSchedulerWeightedPF::GetResourceShare() const
{
    return m_resourceShare;
}

void
NrMacSchedulerWeightedPF::SetUeWeight(uint16_t rnti, double w)
{
    m_ueWeight[rnti] = (std::isfinite(w) && w >= 0.0) ? w : 1.0;
}

void
NrMacSchedulerWeightedPF::ClearUeWeights()
{
    m_ueWeight.clear();
}

double
NrMacSchedulerWeightedPF::GetEffectiveWeight(uint16_t rnti) const
{
    auto it = m_ueWeight.find(rnti);
    return it != m_ueWeight.end() ? it->second : m_weight;
}

#if WPF_ENABLE_WEIGHTED_COMPARE
// ---------------------------------------------------------------------------
// !! VERIFY AGAINST YOUR 5G-LENA VERSION !!
// The PF compare function of 5G-LENA ranks UEs by  potentialTput / avgTput.
// This override reproduces that ranking and multiplies it by the weight.
// Check contrib/nr/model/nr-mac-scheduler-ue-info-pf.{h,cc}:
//   - field names  m_potentialTputDl, m_avgTputDl, m_rnti
//   - how NrMacSchedulerUeInfoPF::CompareUeWeightsDl computes its ratio
// If they differ, adapt the three lines marked (*) below.
// ---------------------------------------------------------------------------
std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerWeightedPF::GetUeCompareDlFn() const
{
    return [this](const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                  const NrMacSchedulerNs3::UePtrAndBufferReq& rhs) -> bool {
        auto l = dynamic_cast<NrMacSchedulerUeInfoPF*>(lhs.first.get());
        auto r = dynamic_cast<NrMacSchedulerUeInfoPF*>(rhs.first.get());
        NS_ABORT_MSG_IF(l == nullptr || r == nullptr,
                        "NrMacSchedulerWeightedPF needs NrMacSchedulerUeInfoPF UE objects");

        const double lm = GetEffectiveWeight(l->m_rnti) *                             // (*)
                          std::max(1e-9, static_cast<double>(l->m_potentialTputDl)) / // (*)
                          std::max(1e-9, static_cast<double>(l->m_avgTputDl));        // (*)
        const double rm = GetEffectiveWeight(r->m_rnti) *
                          std::max(1e-9, static_cast<double>(r->m_potentialTputDl)) /
                          std::max(1e-9, static_cast<double>(r->m_avgTputDl));
        if (lm != rm)
        {
            return lm > rm;
        }
        return l->m_rnti < r->m_rnti; // deterministic tie-break
    };
}
#endif

#if WPF_ENABLE_RESOURCE_SHARE
// ---------------------------------------------------------------------------
// !! VERIFY AGAINST YOUR 5G-LENA VERSION !!
// Signature must match NrMacSchedulerOfdma::AssignDLRBG in
// contrib/nr/model/nr-mac-scheduler-ofdma.h, and the base method must be
// reachable (protected). The override simply hands the base class FEWER symbols,
// so the slot keeps (1 - share) of its DL symbols unused for this BWP.
// ---------------------------------------------------------------------------
NrMacSchedulerNs3::BeamSymbolMap
NrMacSchedulerWeightedPF::AssignDLRBG(uint32_t symAvail,
                                      const NrMacSchedulerNs3::ActiveUeMap& activeDl) const
{
    uint32_t capped = symAvail;
    if (m_resourceShare < 0.999 && symAvail > 0)
    {
        capped = static_cast<uint32_t>(std::floor(symAvail * m_resourceShare + 0.5));
        // every active beam needs at least one symbol, otherwise the base class starves it
        const uint32_t minSym = std::max<uint32_t>(1, static_cast<uint32_t>(activeDl.size()));
        capped = std::min(symAvail, std::max(capped, minSym));
    }
    return NrMacSchedulerOfdmaPF::AssignDLRBG(capped, activeDl);
}
#endif

} // namespace ns3