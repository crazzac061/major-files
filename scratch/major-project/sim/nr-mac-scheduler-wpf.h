/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#ifndef NR_MAC_SCHEDULER_WPF_H
#define NR_MAC_SCHEDULER_WPF_H

#include "ns3/nr-mac-scheduler-ofdma-pf.h"

namespace ns3 {

/**
 * \ingroup scheduler
 * \brief Weighted Proportional Fair Scheduler (Stub)
 */
class NrMacSchedulerWeightedPF : public NrMacSchedulerOfdmaPF
{
public:
  /**
   * \brief GetTypeId
   * \return The TypeId of the class
   */
  static TypeId GetTypeId ();

  /**
   * \brief Constructor
   */
  NrMacSchedulerWeightedPF ();

  /**
   * \brief Destructor
   */
  ~NrMacSchedulerWeightedPF () override;

  /**
   * \brief Set weight
   * \param w Weight
   */
  void SetWeight (double w);

  /**
   * \brief Get weight
   * \return Weight
   */
  double GetWeight () const;

  /**
   * \brief Set slice type
   * \param s Slice type
   */
  void SetSliceType (uint8_t s);

  /**
   * \brief Get slice type
   * \return Slice type
   */
  uint8_t GetSliceType () const;

private:
  double m_weight;
  uint8_t m_sliceType;
};

} // namespace ns3

#endif /* NR_MAC_SCHEDULER_WPF_H */
