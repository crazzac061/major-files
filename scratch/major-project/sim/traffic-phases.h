/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
//
// traffic-phases.h   (Phase 1c)
//
// Four-phase, 50 s episode (proposal Table 5.3). All times are relative to the
// episode start (= application start time).
//
//   Phase  Time      uRLLC          eMBB       mMTC
//   0 Baseline   0-15   512 B / 1 ms    50 Mbps    64 B / 100 ms
//   1 uRLLC burst 15-30 512 B / 0.5 ms  20 Mbps    64 B / 100 ms
//   2 eMBB surge 30-45  512 B / 1 ms   100 Mbps    64 B / 100 ms
//   3 mMTC flood 45-50  512 B / 1 ms    50 Mbps    64 B / 10 ms
//
// All flows must be UdpClient applications: the rate is changed at run time by
// rewriting the "Interval" attribute (UdpClient reads it each time it schedules
// the next packet). The eMBB interval is derived from the target rate:
//     interval = payloadBytes * 8 / rate
//
#ifndef TRAFFIC_PHASES_H
#define TRAFFIC_PHASES_H

#include "slice-kpi-collector.h" // SliceId

#include "ns3/applications-module.h"
#include "ns3/core-module.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace slicing
{

using namespace ns3;

struct TrafficPhase
{
    const char* name;
    double startS; // relative to episode start
    double urllcIntervalMs;
    double embbMbps;
    double mmtcIntervalMs;
};

class TrafficPhaseController
{
  public:
    static std::vector<TrafficPhase> DefaultPhases()
    {
        return {{"baseline", 0.0, 1.0, 50.0, 100.0},
                {"urllc-burst", 15.0, 0.5, 20.0, 100.0},
                {"embb-surge", 30.0, 1.0, 100.0, 100.0},
                {"mmtc-flood", 45.0, 1.0, 50.0, 10.0}};
    }

    /// payload sizes must match the PacketSize given to the UdpClients
    TrafficPhaseController(std::vector<TrafficPhase> phases,
                           uint32_t embbPayloadBytes,
                           double loadScale = 1.0)
        : m_phases(std::move(phases)),
          m_embbBytes(embbPayloadBytes),
          m_loadScale(loadScale)
    {
    }

    void AddFlow(uint32_t ue, uint32_t slice, Ptr<UdpClient> client)
    {
        m_flows.push_back({ue, slice, client});
    }

    /// Initial Interval for a slice at phase 0 (use it when creating the UdpClient).
    Time InitialInterval(uint32_t slice) const
    {
        return IntervalFor(m_phases.front(), slice);
    }

    /// Schedule every phase change. Call once, at episode start (Now == episode start).
    void Schedule()
    {
        for (size_t i = 0; i < m_phases.size(); ++i)
        {
            Simulator::Schedule(Seconds(m_phases[i].startS),
                                &TrafficPhaseController::Apply,
                                this,
                                i);
        }
    }

    int CurrentPhase() const
    {
        return m_current;
    }

    const std::vector<TrafficPhase>& Phases() const
    {
        return m_phases;
    }

  private:
    struct Flow
    {
        uint32_t ue;
        uint32_t slice;
        Ptr<UdpClient> client;
    };

    Time IntervalFor(const TrafficPhase& ph, uint32_t slice) const
    {
        double sec = 1.0;
        switch (slice)
        {
        case SLICE_URLLC:
            sec = ph.urllcIntervalMs * 1e-3 / m_loadScale;
            break;
        case SLICE_EMBB:
            sec = m_embbBytes * 8.0 / (ph.embbMbps * 1e6 * m_loadScale);
            break;
        default:
            sec = ph.mmtcIntervalMs * 1e-3 / m_loadScale;
            break;
        }
        return NanoSeconds(static_cast<int64_t>(std::llround(sec * 1e9)));
    }

    void Apply(size_t i)
    {
        m_current = static_cast<int>(i);
        const TrafficPhase& ph = m_phases[i];
        for (auto& f : m_flows)
        {
            f.client->SetAttribute("Interval", TimeValue(IntervalFor(ph, f.slice)));
        }
        std::cout << "[PHASE] t=" << Simulator::Now().GetSeconds() << "s -> " << i << " ("
                  << ph.name << ")  uRLLC " << ph.urllcIntervalMs << " ms, eMBB " << ph.embbMbps
                  << " Mbps, mMTC " << ph.mmtcIntervalMs << " ms\n";
    }

    std::vector<TrafficPhase> m_phases;
    uint32_t m_embbBytes;
    double m_loadScale;
    std::vector<Flow> m_flows;
    int m_current{-1};
};

} // namespace slicing

#endif // TRAFFIC_PHASES_H