/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include "nr-mac-scheduler-wpf.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NrMacSchedulerWeightedPF");

NS_OBJECT_ENSURE_REGISTERED (NrMacSchedulerWeightedPF);

TypeId
NrMacSchedulerWeightedPF::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::NrMacSchedulerWeightedPF")
    .SetParent<NrMacSchedulerOfdmaPF> ()
    .SetGroupName ("Nr")
    .AddConstructor<NrMacSchedulerWeightedPF> ()
  ;
  return tid;
}

NrMacSchedulerWeightedPF::NrMacSchedulerWeightedPF ()
  : m_weight (1.0),
    m_sliceType (0)
{
  NS_LOG_FUNCTION (this);
}

NrMacSchedulerWeightedPF::~NrMacSchedulerWeightedPF ()
{
  NS_LOG_FUNCTION (this);
}

void
NrMacSchedulerWeightedPF::SetWeight (double w)
{
  m_weight = w;
}

double
NrMacSchedulerWeightedPF::GetWeight () const
{
  return m_weight;
}

void
NrMacSchedulerWeightedPF::SetSliceType (uint8_t s)
{
  m_sliceType = s;
}

uint8_t
NrMacSchedulerWeightedPF::GetSliceType () const
{
  return m_sliceType;
}

} // namespace ns3
