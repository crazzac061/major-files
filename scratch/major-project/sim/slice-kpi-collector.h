/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
//
// slice-kpi-collector.h   (Phase 1b)
//
// Per-step (default 100 ms) KPI extraction for the RL environment.
//
//   * Throughput / delay / loss come from application-level traces
//     (UdpClient "Tx", UdpServer "Rx"). Delay is measured from the SeqTsHeader
//     that UdpClient puts in every packet, so every flow (uRLLC, eMBB, mMTC)
//     must be a UdpClient/UdpServer pair.
//   * Samples are attributed to the gNB that serves the UE at the END of the window.
//   * PRB utilisation is fed by NotifyPrb() (PHY slot-stats trace). If it is
//     never fed, a throughput-based PROXY is reported instead (see SetPrbProxyPeakMbps).
//   * Drop rate is reported with a ONE-STEP LAG: packets still in flight when a
//     window closes would otherwise look "lost". Packets are binned by their
//     TRANSMIT time and a bin is finalised one window after it closed.
//
// Observation order for the RL contract (GnbStepKpi::ToVector, 12 floats):
//   for slice in {uRLLC, eMBB, mMTC}: [throughput_Mbps, mean_delay_ms, prb_util, drop_rate]
//
#ifndef SLICE_KPI_COLLECTOR_H
#define SLICE_KPI_COLLECTOR_H

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/seq-ts-header.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace slicing
{

using namespace ns3;

enum SliceId : uint32_t
{
    SLICE_URLLC = 0,
    SLICE_EMBB = 1,
    SLICE_MMTC = 2,
    NUM_SLICES = 3
};

inline const char*
SliceName(uint32_t s)
{
    static const char* n[] = {"uRLLC", "eMBB", "mMTC"};
    return n[s < NUM_SLICES ? s : 0];
}

struct SliceKpi
{
    double tputMbps{0};
    double meanDelayMs{0};
    double p99DelayMs{0};
    double prbUtil{0};
    double dropRate{0}; // lagged by one step, see header comment
    uint32_t txPkts{0}; // packets sent in this window by UEs served by this gNB
    uint32_t rxPkts{0}; // packets received in this window
    uint32_t nUe{0};
};

struct GnbStepKpi
{
    uint32_t gnb{0};
    std::array<SliceKpi, NUM_SLICES> slice;
    uint32_t nUe{0};
    bool hoFlag{false};
    std::array<double, NUM_SLICES> appliedWeights{{1.0 / 3, 1.0 / 3, 1.0 / 3}};
    double txPowerFrac{1.0};

    /// 12-float k_t vector of the RL contract (slice-major).
    std::array<double, 12> ToVector() const
    {
        std::array<double, 12> v{};
        for (uint32_t s = 0; s < NUM_SLICES; ++s)
        {
            v[4 * s + 0] = slice[s].tputMbps;
            v[4 * s + 1] = slice[s].meanDelayMs;
            v[4 * s + 2] = slice[s].prbUtil;
            v[4 * s + 3] = slice[s].dropRate;
        }
        return v;
    }
};

struct UeStepKpi
{
    uint32_t ue{0};
    int servingGnb{-1};
    double x{0}, y{0}, speed{0};
    std::array<double, NUM_SLICES> tputMbps{{0, 0, 0}};
    std::array<double, NUM_SLICES> meanDelayMs{{0, 0, 0}};
    std::array<uint32_t, NUM_SLICES> rxPkts{{0, 0, 0}};
};

struct StepKpi
{
    uint32_t step{0}; // 1-based
    double t{0};      // sim time at the END of the window [s]
    int phase{-1};    // traffic phase index (-1 if unknown)
    std::vector<GnbStepKpi> gnb;
    std::vector<UeStepKpi> ue;
};

class SliceKpiCollector
{
  public:
    SliceKpiCollector(uint32_t nGnb, uint32_t nUe, Time window, const std::string& prefix)
        : m_nGnb(nGnb),
          m_nUe(nUe),
          m_window(window),
          m_windowNs(window.GetNanoSeconds()),
          m_windowSec(window.GetSeconds()),
          m_flows(nUe * NUM_SLICES),
          m_prbUsed(nGnb * NUM_SLICES, 0.0),
          m_prbAvail(nGnb * NUM_SLICES, 0.0),
          m_hoFlag(nGnb, false),
          m_appliedW(nGnb, {{1.0 / 3, 1.0 / 3, 1.0 / 3}}),
          m_appliedP(nGnb, 1.0)
    {
        m_fStep.open(prefix + "-step-kpi.csv");
        m_fStep << "Step,Time_s,Phase,gNB,Slice,Throughput_Mbps,MeanDelay_ms,P99Delay_ms,"
                   "PrbUtil,DropRate_lagged,TxPkts,RxPkts,nUE\n";
        m_fGnb.open(prefix + "-gnb-step.csv");
        m_fGnb << "Step,Time_s,Phase,gNB,nUE,W_uRLLC,W_eMBB,W_mMTC,TxPowerFrac,HO_Flag\n";
        m_fUe.open(prefix + "-ue-step.csv");
        m_fUe << "Step,Time_s,UE,ServingGnb,X_m,Y_m,Speed_mps\n";
        m_fFlow.open(prefix + "-ue-flow-step.csv");
        m_fFlow << "Step,Time_s,UE,Slice,ServingGnb,Throughput_Mbps,MeanDelay_ms,RxPkts\n";
        m_peakMbps = {{1500.0, 2500.0, 50.0}};
    }

    // ---------------- configuration ----------------

    /// Hook the Tx/Rx traces of one flow. Call once per (ue, slice).
    void RegisterFlow(uint32_t ue, uint32_t slice, Ptr<UdpClient> tx, Ptr<UdpServer> rx)
    {
        NS_ABORT_MSG_IF(ue >= m_nUe || slice >= NUM_SLICES, "RegisterFlow: bad index");
        const uint32_t f = ue * NUM_SLICES + slice;
        tx->TraceConnectWithoutContext("Tx",
                                       MakeBoundCallback(&SliceKpiCollector::TxTramp, this, f));
        rx->TraceConnectWithoutContext("Rx",
                                       MakeBoundCallback(&SliceKpiCollector::RxTramp, this, f));
    }

    void SetUeNodes(const NodeContainer& ueNodes)
    {
        m_ueNodes = ueNodes;
    }

    /// Function returning the serving gNB INDEX (0..nGnb-1) of a UE, or -1 if unknown.
    void SetServingGnbFn(std::function<int(uint32_t)> fn)
    {
        m_servingFn = std::move(fn);
    }

    /// Optional: returns the current traffic phase index (for logging).
    void SetPhaseFn(std::function<int()> fn)
    {
        m_phaseFn = std::move(fn);
    }

    /// Called at the end of every step with the full record (used later by the gym bridge).
    void SetStepCallback(std::function<void(const StepKpi&)> cb)
    {
        m_stepCb = std::move(cb);
    }

    /// Proxy PRB utilisation = throughput / peak. Used only while NotifyPrb() was never called.
    void SetPrbProxyPeakMbps(const std::array<double, NUM_SLICES>& p)
    {
        m_peakMbps = p;
    }

    // ---------------- event inputs ----------------

    /// Feed from the gNB PHY slot-statistics trace. used/avail in any consistent unit (REG).
    void NotifyPrb(uint32_t gnb, uint32_t slice, double used, double avail)
    {
        if (gnb >= m_nGnb || slice >= NUM_SLICES)
        {
            return;
        }
        m_prbUsed[gnb * NUM_SLICES + slice] += used;
        m_prbAvail[gnb * NUM_SLICES + slice] += avail;
        m_prbFed = true;
    }

    /// Mark both gNBs of a handover (-1 entries are ignored). Cleared after the next step.
    void NotifyHandover(int srcGnb, int tgtGnb)
    {
        if (srcGnb >= 0 && srcGnb < static_cast<int>(m_nGnb))
        {
            m_hoFlag[srcGnb] = true;
        }
        if (tgtGnb >= 0 && tgtGnb < static_cast<int>(m_nGnb))
        {
            m_hoFlag[tgtGnb] = true;
        }
    }

    /// Record the control values that are in force (written by SliceControl).
    void SetAppliedControl(uint32_t gnb, const std::array<double, NUM_SLICES>& w, double pFrac)
    {
        if (gnb < m_nGnb)
        {
            m_appliedW[gnb] = w;
            m_appliedP[gnb] = pFrac;
        }
    }

    // ---------------- run control ----------------

    /// First window starts NOW (call at episode start). Runs nSteps steps.
    void Start(uint32_t nSteps)
    {
        m_maxSteps = nSteps;
        m_step = 0;
        m_t0Bin = Simulator::Now().GetNanoSeconds() / m_windowNs;
        Simulator::Schedule(m_window, &SliceKpiCollector::Step, this);
    }

    void Finish()
    {
        m_fStep.close();
        m_fGnb.close();
        m_fUe.close();
        m_fFlow.close();
        std::cout << "[KPI] steps logged: " << m_step << "  (window " << m_windowSec * 1e3
                  << " ms)\n";
        for (uint32_t s = 0; s < NUM_SLICES; ++s)
        {
            std::cout << "[KPI]   " << SliceName(s) << ": tx=" << m_totTx[s] << " rx=" << m_totRx[s]
                      << "  (cumulative, includes packets after the last full window)\n";
        }
        std::cout << "[KPI] PRB source: " << (m_prbFed ? "PHY trace" : "throughput PROXY") << "\n";
    }

    const StepKpi& Last() const
    {
        return m_last;
    }

    uint32_t StepCount() const
    {
        return m_step;
    }

  private:
    struct FlowWin
    {
        uint32_t txPkts{0};
        uint32_t rxPkts{0};
        uint64_t rxBytes{0};
        double delaySumMs{0};
        std::vector<double> delaysMs;
    };

    // ---- trace trampolines (MakeBoundCallback needs plain function pointers) ----
    static void TxTramp(SliceKpiCollector* self, uint32_t f, Ptr<const Packet> p)
    {
        self->OnTx(f);
    }

    static void RxTramp(SliceKpiCollector* self, uint32_t f, Ptr<const Packet> p)
    {
        self->OnRx(f, p);
    }

    void OnTx(uint32_t f)
    {
        m_flows[f].txPkts++;
        const int64_t bin = Simulator::Now().GetNanoSeconds() / m_windowNs;
        m_bins[f][bin][0]++;
        m_totTx[f % NUM_SLICES]++;
    }

    void OnRx(uint32_t f, Ptr<const Packet> p)
    {
        // UdpServer fires "Rx" BEFORE it strips the SeqTsHeader, so we can read the Tx timestamp.
        Ptr<Packet> c = p->Copy();
        SeqTsHeader h;
        if (c->PeekHeader(h) == 0)
        {
            return;
        }
        const Time ts = h.GetTs();
        const double dMs = (Simulator::Now() - ts).GetSeconds() * 1e3;
        FlowWin& w = m_flows[f];
        w.rxPkts++;
        w.rxBytes += p->GetSize();
        w.delaySumMs += dMs;
        w.delaysMs.push_back(dMs);
        m_bins[f][ts.GetNanoSeconds() / m_windowNs][1]++;
        m_totRx[f % NUM_SLICES]++;
    }

    static double Percentile(std::vector<double>& v, double q)
    {
        if (v.empty())
        {
            return 0.0;
        }
        std::sort(v.begin(), v.end());
        size_t idx = static_cast<size_t>(std::ceil(q * v.size()));
        idx = idx == 0 ? 0 : std::min(idx - 1, v.size() - 1);
        return v[idx];
    }

    void Step()
    {
        ++m_step;
        const Time now = Simulator::Now();
        const int64_t endedBin = now.GetNanoSeconds() / m_windowNs - 1; // window that just closed
        const int64_t lagBin = endedBin - 1;                            // bin finalised for drops

        StepKpi rec;
        rec.step = m_step;
        rec.t = now.GetSeconds();
        rec.phase = m_phaseFn ? m_phaseFn() : -1;
        rec.gnb.resize(m_nGnb);
        for (uint32_t g = 0; g < m_nGnb; ++g)
        {
            rec.gnb[g].gnb = g;
        }
        rec.ue.resize(m_nUe);

        // aggregation scratch
        struct Agg
        {
            uint64_t rxBytes{0};
            uint32_t tx{0}, rx{0};
            double delaySum{0};
            std::vector<double> delays;
            uint32_t lagTx{0}, lagRx{0};
            uint32_t nUe{0};
        };

        std::vector<Agg> agg(m_nGnb * NUM_SLICES);
        std::vector<int> serving(m_nUe, -1);

        for (uint32_t u = 0; u < m_nUe; ++u)
        {
            serving[u] = m_servingFn ? m_servingFn(u) : -1;
            UeStepKpi& ur = rec.ue[u];
            ur.ue = u;
            ur.servingGnb = serving[u];
            if (u < m_ueNodes.GetN())
            {
                Ptr<MobilityModel> mob = m_ueNodes.Get(u)->GetObject<MobilityModel>();
                if (mob)
                {
                    const Vector pos = mob->GetPosition();
                    ur.x = pos.x;
                    ur.y = pos.y;
                    ur.speed = mob->GetVelocity().GetLength();
                }
            }
            for (uint32_t s = 0; s < NUM_SLICES; ++s)
            {
                const uint32_t f = u * NUM_SLICES + s;
                FlowWin& fw = m_flows[f];
                ur.tputMbps[s] = fw.rxBytes * 8.0 / m_windowSec / 1e6;
                ur.meanDelayMs[s] = fw.rxPkts ? fw.delaySumMs / fw.rxPkts : 0.0;
                ur.rxPkts[s] = fw.rxPkts;

                // lagged loss bookkeeping
                uint32_t lagTx = 0, lagRx = 0;
                auto& bins = m_bins[f];
                auto it = bins.find(lagBin);
                if (it != bins.end())
                {
                    lagTx = it->second[0];
                    lagRx = it->second[1];
                }
                bins.erase(bins.begin(), bins.upper_bound(lagBin)); // drop finalised bins

                const int g = serving[u];
                if (g >= 0 && g < static_cast<int>(m_nGnb))
                {
                    Agg& a = agg[g * NUM_SLICES + s];
                    a.rxBytes += fw.rxBytes;
                    a.tx += fw.txPkts;
                    a.rx += fw.rxPkts;
                    a.delaySum += fw.delaySumMs;
                    a.delays.insert(a.delays.end(), fw.delaysMs.begin(), fw.delaysMs.end());
                    a.lagTx += lagTx;
                    a.lagRx += lagRx;
                    a.nUe++;
                }
                fw = FlowWin(); // reset window
            }
        }

        for (uint32_t g = 0; g < m_nGnb; ++g)
        {
            GnbStepKpi& gr = rec.gnb[g];
            gr.hoFlag = m_hoFlag[g];
            gr.appliedWeights = m_appliedW[g];
            gr.txPowerFrac = m_appliedP[g];
            uint32_t ueCount = 0;
            for (uint32_t s = 0; s < NUM_SLICES; ++s)
            {
                Agg& a = agg[g * NUM_SLICES + s];
                SliceKpi& k = gr.slice[s];
                k.tputMbps = a.rxBytes * 8.0 / m_windowSec / 1e6;
                k.meanDelayMs = a.rx ? a.delaySum / a.rx : 0.0;
                k.p99DelayMs = Percentile(a.delays, 0.99);
                k.txPkts = a.tx;
                k.rxPkts = a.rx;
                k.nUe = a.nUe;
                k.dropRate =
                    a.lagTx ? std::max(0.0, 1.0 - static_cast<double>(a.lagRx) / a.lagTx) : 0.0;
                const double avail = m_prbAvail[g * NUM_SLICES + s];
                if (m_prbFed)
                {
                    k.prbUtil =
                        avail > 0 ? std::min(1.0, m_prbUsed[g * NUM_SLICES + s] / avail) : 0.0;
                }
                else
                {
                    k.prbUtil = std::min(1.0, k.tputMbps / m_peakMbps[s]);
                }
                m_prbUsed[g * NUM_SLICES + s] = 0.0;
                m_prbAvail[g * NUM_SLICES + s] = 0.0;
                ueCount = std::max(ueCount, a.nUe);
            }
            gr.nUe = ueCount;
        }
        std::fill(m_hoFlag.begin(), m_hoFlag.end(), false);

        WriteCsv(rec);
        m_last = rec;
        if (m_stepCb)
        {
            m_stepCb(m_last);
        }
        if (m_step < m_maxSteps)
        {
            Simulator::Schedule(m_window, &SliceKpiCollector::Step, this);
        }
    }

    void WriteCsv(const StepKpi& r)
    {
        m_fStep << std::fixed << std::setprecision(4);
        m_fGnb << std::fixed << std::setprecision(4);
        m_fUe << std::fixed << std::setprecision(3);
        m_fFlow << std::fixed << std::setprecision(4);
        for (const auto& g : r.gnb)
        {
            for (uint32_t s = 0; s < NUM_SLICES; ++s)
            {
                const SliceKpi& k = g.slice[s];
                m_fStep << r.step << "," << r.t << "," << r.phase << "," << g.gnb << ","
                        << SliceName(s) << "," << k.tputMbps << "," << k.meanDelayMs << ","
                        << k.p99DelayMs << "," << k.prbUtil << "," << k.dropRate << "," << k.txPkts
                        << "," << k.rxPkts << "," << k.nUe << "\n";
            }
            m_fGnb << r.step << "," << r.t << "," << r.phase << "," << g.gnb << "," << g.nUe << ","
                   << g.appliedWeights[0] << "," << g.appliedWeights[1] << ","
                   << g.appliedWeights[2] << "," << g.txPowerFrac << "," << (g.hoFlag ? 1 : 0)
                   << "\n";
        }
        for (const auto& u : r.ue)
        {
            m_fUe << r.step << "," << r.t << "," << u.ue << "," << u.servingGnb << "," << u.x << ","
                  << u.y << "," << u.speed << "\n";
            for (uint32_t s = 0; s < NUM_SLICES; ++s)
            {
                m_fFlow << r.step << "," << r.t << "," << u.ue << "," << SliceName(s) << ","
                        << u.servingGnb << "," << u.tputMbps[s] << "," << u.meanDelayMs[s] << ","
                        << u.rxPkts[s] << "\n";
            }
        }
    }

    // ---- state ----
    uint32_t m_nGnb;
    uint32_t m_nUe;
    Time m_window;
    int64_t m_windowNs;
    double m_windowSec;
    NodeContainer m_ueNodes;
    std::vector<FlowWin> m_flows;
    std::map<uint32_t, std::map<int64_t, std::array<uint32_t, 2>>>
        m_bins; // flow -> bin -> {tx, rx}
    std::vector<double> m_prbUsed, m_prbAvail;
    bool m_prbFed{false};
    std::array<double, NUM_SLICES> m_peakMbps;
    std::vector<bool> m_hoFlag;
    std::vector<std::array<double, NUM_SLICES>> m_appliedW;
    std::vector<double> m_appliedP;
    std::array<uint64_t, NUM_SLICES> m_totTx{{0, 0, 0}};
    std::array<uint64_t, NUM_SLICES> m_totRx{{0, 0, 0}};
    std::function<int(uint32_t)> m_servingFn;
    std::function<int()> m_phaseFn;
    std::function<void(const StepKpi&)> m_stepCb;
    uint32_t m_step{0};
    uint32_t m_maxSteps{0};
    int64_t m_t0Bin{0};
    StepKpi m_last;
    std::ofstream m_fStep, m_fGnb, m_fUe, m_fFlow;
};

} // namespace slicing

#endif // SLICE_KPI_COLLECTOR_H