/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
//
// nr-mac-scheduler-wpf.h   (Phase 1d)
//
// Weighted Proportional-Fair OFDMA scheduler with RUN-TIME controllable knobs.
// One scheduler instance exists per (gNB, BWP). Because each BWP carries exactly
// one slice in this project, a pure PF-metric weight cannot change anything
// (every UE in the BWP gets the same multiplier). Therefore two knobs exist:
//
//   Weight         multiplies the PF metric of the UEs of this scheduler.
//                  Only has an effect between UEs with DIFFERENT weights
//                  (see SetUeWeight) i.e. when several slices/flows share a BWP.
//   ResourceShare  fraction (0,1] of the DL symbols this scheduler may use
//                  in each slot. This is what makes "slice weights" observable
//                  while every slice sits in its own BWP.
//
// Both can be changed at any time (attributes or setters); the new values apply
// from the next scheduling slot.
//
// COMPILE SWITCHES (set with -D or edit): if one of the two NR-internal overrides
// does not match your 5G-LENA version, set it to 0 to get the rest compiling.
//   WPF_ENABLE_WEIGHTED_COMPARE  override GetUeCompareDlFn()
//   WPF_ENABLE_RESOURCE_SHARE    override AssignDLRBG()
//
#ifndef NR_MAC_SCHEDULER_WPF_H
#define NR_MAC_SCHEDULER_WPF_H

#include "ns3/nr-mac-scheduler-ofdma-pf.h"

#include <functional>
#include <map>

#ifndef WPF_ENABLE_WEIGHTED_COMPARE
#define WPF_ENABLE_WEIGHTED_COMPARE 1
#endif
#ifndef WPF_ENABLE_RESOURCE_SHARE
#define WPF_ENABLE_RESOURCE_SHARE 1
#endif

namespace ns3
{

/**
 * \ingroup scheduler
 * \brief Weighted Proportional Fair scheduler with run-time weight / resource-share control
 */
class NrMacSchedulerWeightedPF : public NrMacSchedulerOfdmaPF
{
  public:
    static TypeId GetTypeId();

    NrMacSchedulerWeightedPF();
    ~NrMacSchedulerWeightedPF() override;

    /// PF-metric multiplier for all UEs of this scheduler (default 1.0, must be >= 0)
    void SetWeight(double w);
    double GetWeight() const;

    /// Slice id served by this scheduler (0 uRLLC, 1 eMBB, 2 mMTC) - informational
    void SetSliceType(uint8_t s);
    uint8_t GetSliceType() const;

    /// Fraction of DL symbols usable per slot, clamped to [0.01, 1]
    void SetResourceShare(double share);
    double GetResourceShare() const;

    /// Per-UE override of the PF multiplier (replaces Weight for that RNTI)
    void SetUeWeight(uint16_t rnti, double w);
    void ClearUeWeights();

    /// Weight that is applied to a given RNTI right now
    double GetEffectiveWeight(uint16_t rnti) const;

  protected:
#if WPF_ENABLE_WEIGHTED_COMPARE
    std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                       const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)> GetUeCompareDlFn()
        const override;
#endif

#if WPF_ENABLE_RESOURCE_SHARE
    NrMacSchedulerNs3::BeamSymbolMap AssignDLRBG(
        uint32_t symAvail,
        const NrMacSchedulerNs3::ActiveUeMap& activeDl) const override;
#endif

  private:
    double m_weight;
    uint8_t m_sliceType;
    double m_resourceShare;
    std::map<uint16_t, double> m_ueWeight;
};

} // namespace ns3

#endif /* NR_MAC_SCHEDULER_WPF_H */