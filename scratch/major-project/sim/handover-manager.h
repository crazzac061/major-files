/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
//
// handover-manager.h   (Phase 1a)
//
// Why the old code logged zero handovers (most likely causes, in order):
//   1. NOTHING TRIGGERED THEM. The A3RsrpHandoverAlgorithm attributes it set belong to the
//      LTE module; no handover algorithm was installed on the gNBs, and 5G-LENA (to our
//      knowledge) has no automatic A3 algorithm, so no UE ever asked for a handover.
//   2. TRACE PATH. It connected "/NodeList/N/DeviceList/0/LteUeRrc/HandoverStart" through
//      Config. If the attribute name does not match, nothing is connected (and nothing says so).
//
// This class fixes both:
//   * traces are hooked DIRECTLY on each UE's RRC object (no Config path to get wrong);
//   * handovers are triggered by a scripted A3-style rule (neighbour better than serving
//     by `hysteresisDb` for `ttt`), evaluated every `period`, using an RSRP PROXY from
//     the UE-gNB distance (log-distance with exponent `plExp`);
//   * every event is logged with serving/target CELL and gNB index;
//   * PrintSummary() states clearly whether handovers were requested / started / completed.
//
// 5G-LENA ASSUMPTIONS TO VERIFY (grep in contrib/nr):
//   (NrHelper::HandoverRequest does NOT exist in ns-3.40 + this 5G-LENA; we call
//    LteEnbRrc::SendHandoverRequest(rnti, targetCellId) on the source gNB RRC instead.)
//   NrUeNetDevice::GetRrc()  -> Ptr<LteUeRrc>
//   NrGnbNetDevice::GetRrc() -> Ptr<LteEnbRrc>
//
#ifndef HANDOVER_MANAGER_H
#define HANDOVER_MANAGER_H

#include "slice-kpi-collector.h"

#include "ns3/lte-enb-rrc.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/nr-module.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace slicing
{

using namespace ns3;

struct HandoverEvent
{
    double t;
    uint32_t ue;
    std::string kind; // REQUEST | START | OK | ERROR
    int srcCell, tgtCell;
    int srcGnb, tgtGnb;
    double x, y;
};

class HandoverManager
{
  public:
    HandoverManager(Ptr<NrHelper> nr,
                    NetDeviceContainer gnbDevs,
                    NetDeviceContainer ueDevs,
                    NodeContainer gnbNodes,
                    NodeContainer ueNodes)
        : m_nr(nr),
          m_gnb(gnbDevs),
          m_ue(ueDevs),
          m_gnbNodes(gnbNodes),
          m_ueNodes(ueNodes),
          m_lastServing(ueDevs.GetN(), 0),
          m_inHo(ueDevs.GetN(), false),
          m_reqTime(ueDevs.GetN(), -1e9),
          m_lastHoEnd(ueDevs.GetN(), -1e9),
          m_cand(ueDevs.GetN(), -1),
          m_candCount(ueDevs.GetN(), 0)
    {
        for (uint32_t u = 0; u < m_ue.GetN(); ++u)
        {
            m_lastServing[u] = ClosestGnb(u); // until the RRC reports its cell
        }
    }

    /// Hook HandoverStart / HandoverEndOk / HandoverEndError on every UE RRC.
    void ConnectTraces(SliceKpiCollector* kpi = nullptr)
    {
        m_kpi = kpi;
        for (uint32_t u = 0; u < m_ue.GetN(); ++u)
        {
            Ptr<LteUeRrc> rrc = DynamicCast<NrUeNetDevice>(m_ue.Get(u))->GetRrc();
            NS_ABORT_MSG_IF(!rrc, "UE " << u << " has no RRC - call after AttachToClosestEnb");
            rrc->TraceConnectWithoutContext(
                "HandoverStart",
                MakeBoundCallback(&HandoverManager::StartTramp, this, u));
            rrc->TraceConnectWithoutContext("HandoverEndOk",
                                            MakeBoundCallback(&HandoverManager::OkTramp, this, u));
            rrc->TraceConnectWithoutContext("HandoverEndError",
                                            MakeBoundCallback(&HandoverManager::ErrTramp, this, u));
        }
    }

    /// Serving gNB index of a UE (from the RRC cell id; falls back to the last known value).
    int ServingGnb(uint32_t u)
    {
        Ptr<LteUeRrc> rrc = DynamicCast<NrUeNetDevice>(m_ue.Get(u))->GetRrc();
        if (rrc && rrc->GetState() == LteUeRrc::CONNECTED_NORMALLY)
        {
            const int g = GnbOfCell(rrc->GetCellId());
            if (g >= 0)
            {
                m_lastServing[u] = g;
            }
        }
        return m_lastServing[u];
    }

    /// Start the scripted A3-style trigger. Call at episode start.
    void StartScripted(Time period = MilliSeconds(100),
                       double hysteresisDb = 3.0,
                       Time ttt = MilliSeconds(100),
                       Time minDwell = Seconds(1.0),
                       double plExp = 3.5)
    {
        m_period = period;
        m_hys = hysteresisDb;
        m_tttSteps =
            std::max<int>(1, static_cast<int>(std::ceil(ttt.GetSeconds() / period.GetSeconds())));
        m_dwell = minDwell.GetSeconds();
        m_plExp = plExp;
        m_scripted = true;
        Simulator::Schedule(m_period, &HandoverManager::Check, this);
    }

    /// Print RRC state of every UE (call a little after attach, e.g. at t = 0.5 s).
    void PrintRrcStates(const std::string& tag)
    {
        std::cout << "[HO-DIAG] " << tag << " t=" << Simulator::Now().GetSeconds() << "s\n";
        for (uint32_t u = 0; u < m_ue.GetN(); ++u)
        {
            Ptr<LteUeRrc> rrc = DynamicCast<NrUeNetDevice>(m_ue.Get(u))->GetRrc();
            std::cout << "  UE-" << u << " state=" << (rrc ? StateName(rrc->GetState()) : "none")
                      << " cell=" << (rrc ? rrc->GetCellId() : 0) << " gnb=" << ServingGnb(u)
                      << "\n";
        }
    }

    void PrintSummary() const
    {
        std::cout << "[HO] requested=" << m_nReq << " started=" << m_nStart << " ok=" << m_nOk
                  << " error=" << m_nErr << "\n";
        if (m_nReq > 0 && m_nStart == 0)
        {
            std::cout
                << "[HO] WARNING: handovers were requested but none started. Check that X2 "
                   "links exist before attach, that LteEnbRrc::SendHandoverRequest works for NR "
                   "gNBs in this 5G-LENA version, and the UE RRC states printed by "
                   "PrintRrcStates().\n";
        }
        if (m_nReq == 0)
        {
            std::cout
                << "[HO] No handover was requested: UEs never met the A3-style condition. Use "
                   "faster/longer mobility, a lower --hoHyst, or more UEs near cell edges.\n";
        }
    }

    void WriteCsv(const std::string& file) const
    {
        std::ofstream f(file);
        f << "Time_s,UE_ID,Event,Source_Cell,Target_Cell,Source_gNB,Target_gNB,UE_X,UE_Y\n";
        for (const auto& e : m_log)
        {
            f << std::fixed << std::setprecision(3) << e.t << "," << e.ue << "," << e.kind << ","
              << e.srcCell << "," << e.tgtCell << "," << e.srcGnb << "," << e.tgtGnb << ","
              << std::setprecision(2) << e.x << "," << e.y << "\n";
        }
    }

    uint32_t NumStarted() const
    {
        return m_nStart;
    }

    uint32_t NumOk() const
    {
        return m_nOk;
    }

    const std::vector<HandoverEvent>& Log() const
    {
        return m_log;
    }

    /// Lowest cell id owned by gNB `g` (its primary cell), -1 if none found.
    int PrimaryCellOfGnb(int g) const
    {
        Ptr<LteEnbRrc> rrc = DynamicCast<NrGnbNetDevice>(m_gnb.Get(g))->GetRrc();
        for (uint16_t c = 1; rrc && c < 512; ++c)
        {
            if (rrc->HasCellId(c))
            {
                return c;
            }
        }
        return -1;
    }

    /// gNB index that owns `cellId` (-1 if none). Works whether cells are numbered per gNB or per
    /// BWP.
    int GnbOfCell(uint16_t cellId) const
    {
        for (uint32_t g = 0; g < m_gnb.GetN(); ++g)
        {
            Ptr<LteEnbRrc> rrc = DynamicCast<NrGnbNetDevice>(m_gnb.Get(g))->GetRrc();
            if (rrc && rrc->HasCellId(cellId))
            {
                return static_cast<int>(g);
            }
        }
        return -1;
    }

  private:
    // ---------- trace trampolines ----------
    static void StartTramp(HandoverManager* s,
                           uint32_t u,
                           uint64_t,
                           uint16_t cell,
                           uint16_t,
                           uint16_t tgt)
    {
        s->OnStart(u, cell, tgt);
    }

    static void OkTramp(HandoverManager* s, uint32_t u, uint64_t, uint16_t cell, uint16_t)
    {
        s->OnEnd(u, cell, true);
    }

    static void ErrTramp(HandoverManager* s, uint32_t u, uint64_t, uint16_t cell, uint16_t)
    {
        s->OnEnd(u, cell, false);
    }

    void Record(uint32_t u, const char* kind, int srcCell, int tgtCell)
    {
        HandoverEvent e;
        e.t = Simulator::Now().GetSeconds();
        e.ue = u;
        e.kind = kind;
        e.srcCell = srcCell;
        e.tgtCell = tgtCell;
        e.srcGnb = srcCell >= 0 ? GnbOfCell(srcCell) : -1;
        e.tgtGnb = tgtCell >= 0 ? GnbOfCell(tgtCell) : -1;
        Ptr<MobilityModel> mob = m_ueNodes.Get(u)->GetObject<MobilityModel>();
        const Vector p = mob ? mob->GetPosition() : Vector(0, 0, 0);
        e.x = p.x;
        e.y = p.y;
        m_log.push_back(e);
        std::cout << "[HO-" << kind << "] t=" << e.t << "s UE-" << u << " cell " << srcCell
                  << " -> " << tgtCell << " (gNB " << e.srcGnb << " -> " << e.tgtGnb << ") pos=("
                  << p.x << "," << p.y << ")\n";
    }

    void OnStart(uint32_t u, uint16_t cell, uint16_t tgt)
    {
        ++m_nStart;
        m_inHo[u] = true;
        Record(u, "START", cell, tgt);
        if (m_kpi)
        {
            m_kpi->NotifyHandover(GnbOfCell(cell), GnbOfCell(tgt));
        }
    }

    void OnEnd(uint32_t u, uint16_t cell, bool ok)
    {
        ok ? ++m_nOk : ++m_nErr;
        m_inHo[u] = false;
        m_lastHoEnd[u] = Simulator::Now().GetSeconds();
        const int g = GnbOfCell(cell);
        if (ok && g >= 0)
        {
            m_lastServing[u] = g;
        }
        Record(u, ok ? "OK" : "ERROR", cell, cell);
        if (m_kpi)
        {
            m_kpi->NotifyHandover(g, g);
        }
    }

    // ---------- scripted A3-style trigger ----------
    double RsrpProxy(double d) const
    {
        return -10.0 * m_plExp * std::log10(std::max(d, 1.0));
    }

    int ClosestGnb(uint32_t u) const
    {
        Ptr<MobilityModel> um = m_ueNodes.Get(u)->GetObject<MobilityModel>();
        int best = 0;
        double bd = 1e18;
        for (uint32_t g = 0; g < m_gnbNodes.GetN(); ++g)
        {
            const double d = um->GetDistanceFrom(m_gnbNodes.Get(g)->GetObject<MobilityModel>());
            if (d < bd)
            {
                bd = d;
                best = static_cast<int>(g);
            }
        }
        return best;
    }

    void Check()
    {
        const double now = Simulator::Now().GetSeconds();
        for (uint32_t u = 0; u < m_ue.GetN(); ++u)
        {
            Ptr<LteUeRrc> rrc = DynamicCast<NrUeNetDevice>(m_ue.Get(u))->GetRrc();
            if (!rrc || rrc->GetState() != LteUeRrc::CONNECTED_NORMALLY)
            {
                continue;
            }
            if (m_inHo[u] && now - m_reqTime[u] > 2.0)
            {
                m_inHo[u] = false; // request was swallowed (no START trace) - unblock the UE
            }
            if (m_inHo[u] || now - m_lastHoEnd[u] < m_dwell)
            {
                m_candCount[u] = 0;
                continue;
            }
            const int srv = ServingGnb(u);
            Ptr<MobilityModel> um = m_ueNodes.Get(u)->GetObject<MobilityModel>();
            const double srvRsrp =
                RsrpProxy(um->GetDistanceFrom(m_gnbNodes.Get(srv)->GetObject<MobilityModel>()));
            int best = -1;
            double bestRsrp = -1e18;
            for (uint32_t g = 0; g < m_gnbNodes.GetN(); ++g)
            {
                if (static_cast<int>(g) == srv)
                {
                    continue;
                }
                const double r =
                    RsrpProxy(um->GetDistanceFrom(m_gnbNodes.Get(g)->GetObject<MobilityModel>()));
                if (r > bestRsrp)
                {
                    bestRsrp = r;
                    best = static_cast<int>(g);
                }
            }
            if (best >= 0 && bestRsrp - srvRsrp > m_hys)
            {
                m_candCount[u] = (m_cand[u] == best) ? m_candCount[u] + 1 : 1;
                m_cand[u] = best;
                if (m_candCount[u] >= m_tttSteps)
                {
                    ++m_nReq;
                    m_inHo[u] = true;
                    m_reqTime[u] = now;
                    m_candCount[u] = 0;
                    Record(u, "REQUEST", static_cast<int>(rrc->GetCellId()), best);
                    // NrHelper has no HandoverRequest() in this 5G-LENA version, so do what
                    // LteHelper::DoHandoverRequest does: ask the SOURCE gNB RRC to start an
                    // X2 handover for this UE's RNTI towards the target cell.
                    Ptr<LteEnbRrc> srcRrc = DynamicCast<NrGnbNetDevice>(m_gnb.Get(srv))->GetRrc();
                    const int tgtCell = PrimaryCellOfGnb(best);
                    if (!srcRrc || tgtCell < 0)
                    {
                        std::cout << "[HO] cannot request handover: srcRrc/targetCell not found\n";
                        m_inHo[u] = false;
                    }
                    else
                    {
                        srcRrc->SendHandoverRequest(rrc->GetRnti(), static_cast<uint16_t>(tgtCell));
                    }
                }
            }
            else
            {
                m_candCount[u] = 0;
                m_cand[u] = -1;
            }
        }
        if (m_scripted)
        {
            Simulator::Schedule(m_period, &HandoverManager::Check, this);
        }
    }

    static const char* StateName(LteUeRrc::State s)
    {
        switch (s)
        {
        case LteUeRrc::IDLE_START:
            return "IDLE_START";
        case LteUeRrc::IDLE_CELL_SEARCH:
            return "IDLE_CELL_SEARCH";
        case LteUeRrc::IDLE_WAIT_MIB_SIB1:
            return "IDLE_WAIT_MIB_SIB1";
        case LteUeRrc::IDLE_WAIT_MIB:
            return "IDLE_WAIT_MIB";
        case LteUeRrc::IDLE_WAIT_SIB1:
            return "IDLE_WAIT_SIB1";
        case LteUeRrc::IDLE_CAMPED_NORMALLY:
            return "IDLE_CAMPED_NORMALLY";
        case LteUeRrc::IDLE_WAIT_SIB2:
            return "IDLE_WAIT_SIB2";
        case LteUeRrc::IDLE_RANDOM_ACCESS:
            return "IDLE_RANDOM_ACCESS";
        case LteUeRrc::IDLE_CONNECTING:
            return "IDLE_CONNECTING";
        case LteUeRrc::CONNECTED_NORMALLY:
            return "CONNECTED_NORMALLY";
        case LteUeRrc::CONNECTED_HANDOVER:
            return "CONNECTED_HANDOVER";
        case LteUeRrc::CONNECTED_PHY_PROBLEM:
            return "CONNECTED_PHY_PROBLEM";
        case LteUeRrc::CONNECTED_REESTABLISHING:
            return "CONNECTED_REESTABLISHING";
        default:
            return "?";
        }
    }

    Ptr<NrHelper> m_nr;
    NetDeviceContainer m_gnb, m_ue;
    NodeContainer m_gnbNodes, m_ueNodes;
    SliceKpiCollector* m_kpi{nullptr};
    std::vector<int> m_lastServing;
    std::vector<bool> m_inHo;
    std::vector<double> m_reqTime, m_lastHoEnd;
    std::vector<int> m_cand, m_candCount;
    std::vector<HandoverEvent> m_log;
    Time m_period{MilliSeconds(100)};
    double m_hys{3.0};
    int m_tttSteps{1};
    double m_dwell{1.0};
    double m_plExp{3.5};
    bool m_scripted{false};
    uint32_t m_nReq{0}, m_nStart{0}, m_nOk{0}, m_nErr{0};
};

} // namespace slicing

#endif // HANDOVER_MANAGER_H