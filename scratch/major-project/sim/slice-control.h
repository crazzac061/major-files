/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
//
// slice-control.h   (Phase 1d)
//
// Run-time control hooks used by the RL action:  w_BWP[3]  and  p_tx.
//
//   SetSliceWeights(gnb, w)   w = raw agent output, any non-negative numbers.
//        1. sanitised (NaN/negative -> 0), clipped to [0,1]
//        2. normalised to sum 1, then a floor is enforced (no slice is switched off)
//        3. pushed to the WeightedPF scheduler of BWP s on that gNB:
//              Weight        = w_s              (PF metric multiplier)
//              ResourceShare = w_s / max_j w_j  (fraction of DL symbols; best slice = 100 %)
//
//   SetTxPowerFraction(gnb, p)   p in [0,1] -> [minDbm, maxDbm], applied to every BWP PHY.
//
// IMPORTANT: call these only from an event (Simulator::Schedule...) at a step boundary,
// never from inside a scheduler/PHY callback.
//
#ifndef SLICE_CONTROL_H
#define SLICE_CONTROL_H

#include "nr-mac-scheduler-wpf.h"
#include "slice-kpi-collector.h"

#include "ns3/nr-module.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace slicing
{

using namespace ns3;

class SliceControl
{
  public:
    SliceControl(Ptr<NrHelper> nrHelper,
                 NetDeviceContainer gnbDevs,
                 SliceKpiCollector* kpi,
                 double weightFloor = 0.05,
                 double txMinDbm = 30.0,
                 double txMaxDbm = 43.0)
        : m_nr(nrHelper),
          m_gnb(gnbDevs),
          m_kpi(kpi),
          m_floor(weightFloor),
          m_pMin(txMinDbm),
          m_pMax(txMaxDbm),
          m_w(gnbDevs.GetN(), {{1.0 / 3, 1.0 / 3, 1.0 / 3}}),
          m_p(gnbDevs.GetN(), 1.0)
    {
    }

    /// Normalise raw agent output: sanitise, normalise to sum 1, enforce floor.
    std::array<double, NUM_SLICES> Normalise(std::array<double, NUM_SLICES> w) const
    {
        double sum = 0;
        for (auto& x : w)
        {
            x = (std::isfinite(x) && x > 0.0) ? std::min(1.0, x) : 0.0;
            sum += x;
        }
        if (sum <= 1e-12)
        {
            return {{1.0 / 3, 1.0 / 3, 1.0 / 3}};
        }
        for (auto& x : w)
        {
            x = std::max(x / sum, m_floor);
        }
        sum = w[0] + w[1] + w[2];
        for (auto& x : w)
        {
            x /= sum;
        }
        return w;
    }

    /// Apply slice weights on one gNB. Returns the weights that were actually applied.
    std::array<double, NUM_SLICES> SetSliceWeights(uint32_t gnb,
                                                   const std::array<double, NUM_SLICES>& raw)
    {
        NS_ABORT_MSG_IF(gnb >= m_gnb.GetN(), "SetSliceWeights: bad gNB index");
        const auto w = Normalise(raw);
        const double wmax = *std::max_element(w.begin(), w.end());
        for (uint32_t s = 0; s < NUM_SLICES; ++s)
        {
            Ptr<NrMacScheduler> base = m_nr->GetScheduler(m_gnb.Get(gnb), s);
            Ptr<NrMacSchedulerWeightedPF> wpf = DynamicCast<NrMacSchedulerWeightedPF>(base);
            NS_ABORT_MSG_IF(
                !wpf,
                "BWP " << s
                       << " scheduler is not NrMacSchedulerWeightedPF - call "
                          "nrHelper->SetSchedulerTypeId(NrMacSchedulerWeightedPF::GetTypeId()) "
                          "BEFORE InstallGnbDevice");
            wpf->SetSliceType(static_cast<uint8_t>(s));
            wpf->SetWeight(w[s]);
            wpf->SetResourceShare(w[s] / wmax);
        }
        m_w[gnb] = w;
        PublishControl(gnb);
        return w;
    }

    /// Apply the same raw weights on all gNBs.
    void SetSliceWeightsAll(const std::array<double, NUM_SLICES>& raw)
    {
        for (uint32_t g = 0; g < m_gnb.GetN(); ++g)
        {
            SetSliceWeights(g, raw);
        }
    }

    /// p in [0,1] -> dBm range, on every BWP of the gNB.
    double SetTxPowerFraction(uint32_t gnb, double p)
    {
        NS_ABORT_MSG_IF(gnb >= m_gnb.GetN(), "SetTxPowerFraction: bad gNB index");
        p = std::isfinite(p) ? std::min(1.0, std::max(0.0, p)) : 1.0;
        const double dbm = m_pMin + p * (m_pMax - m_pMin);
        for (uint32_t s = 0; s < NUM_SLICES; ++s)
        {
            m_nr->GetGnbPhy(m_gnb.Get(gnb), s)->SetAttribute("TxPower", DoubleValue(dbm));
        }
        m_p[gnb] = p;
        PublishControl(gnb);
        return dbm;
    }

    const std::array<double, NUM_SLICES>& GetWeights(uint32_t gnb) const
    {
        return m_w.at(gnb);
    }

    double GetTxPowerFraction(uint32_t gnb) const
    {
        return m_p.at(gnb);
    }

  private:
    void PublishControl(uint32_t gnb)
    {
        if (m_kpi)
        {
            m_kpi->SetAppliedControl(gnb, m_w[gnb], m_p[gnb]);
        }
    }

    Ptr<NrHelper> m_nr;
    NetDeviceContainer m_gnb;
    SliceKpiCollector* m_kpi;
    double m_floor, m_pMin, m_pMax;
    std::vector<std::array<double, NUM_SLICES>> m_w;
    std::vector<double> m_p;
};

} // namespace slicing

#endif // SLICE_CONTROL_H