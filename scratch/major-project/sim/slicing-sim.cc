/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
//
// slicing-sim.cc  -  3-gNB 5G NR network-slicing data plane (Phase 1 of the RL project)
//
//   1a  handover      -> handover-manager.h   (direct RRC traces + scripted A3-style trigger)
//   1b  KPI hooks     -> slice-kpi-collector.h (100 ms windows, per slice / gNB / UE)
//   1c  traffic phases-> traffic-phases.h      (4-phase 50 s episode, Table 5.3)
//   1d  WPF scheduler -> nr-mac-scheduler-wpf.* + slice-control.h (weights at run time)
//
// Episode timeline:  [0, appStart)  attach + warm-up
//                    [appStart, appStart+episodeLen)  the 50 s RL episode (500 steps of 100 ms)
//
#include "handover-manager.h"
#include "nr-mac-scheduler-wpf.h"
#include "slice-control.h"
#include "slice-kpi-collector.h"
#include "traffic-phases.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-module.h"

#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <vector>

#ifdef SLICING_PRB_TRACE
#include "ns3/sfnsf.h"
#endif

using namespace ns3;
using namespace slicing;

NS_LOG_COMPONENT_DEFINE("SlicingSimulation");

// ============================================================
// NETWORK BANDWIDTH ANALYSIS (documented here for reference)
// ============================================================
// Band 0 - uRLLC: 6.0 GHz, 100 MHz, mu=3 (120 kHz SCS)   peak ~1.5 Gbps / gNB
// Band 1 - eMBB : 3.5 GHz, 100 MHz, mu=1 ( 30 kHz SCS)   peak ~2.5 Gbps / gNB
// Band 2 - mMTC : 700 MHz,  10 MHz, mu=0 ( 15 kHz SCS)   peak ~50 Mbps  / gNB
// Backhaul 100 Gb/s (not a bottleneck); X2 10 Gbps / 0.5 ms.
// ============================================================

#ifdef SLICING_PRB_TRACE
// ----------------------------------------------------------------------------
// PRB utilisation from the gNB PHY slot statistics.
// !! The trace signature below is the 5G-LENA "SlotDataStats" one as we remember it.
// !! Verify with:  grep -n "SlotDataStats" -A6 contrib/nr/model/nr-gnb-phy.cc
// ----------------------------------------------------------------------------
static void
PrbTrace(SliceKpiCollector* kpi,
         uint32_t gnb,
         uint32_t slice,
         const SfnSf& sfn,
         uint32_t scheduledUe,
         uint32_t usedReg,
         uint32_t usedSym,
         uint32_t availableRb,
         uint32_t availableSym,
         uint16_t bwpId,
         uint16_t cellId)
{
    kpi->NotifyPrb(gnb, slice, usedReg, static_cast<double>(availableRb) * availableSym);
}
#endif

int
main(int argc, char* argv[])
{
    // ---------------- command line ----------------
    uint32_t nUe = 18;               // total UEs, shared across the 3 gNBs
    double episodeLen = 50.0;        // RL episode length [s]
    double appStart = 1.0;           // traffic + episode start [s]
    uint32_t windowMs = 100;         // KPI step / agent decision interval [ms]
    std::string hoMode = "scripted"; // scripted | off
    double hoHyst = 3.0;             // dB  (A3-style hysteresis on the RSRP proxy)
    double hoTttMs = 100.0;          // ms  (time-to-trigger)
    double hoDwell = 1.0;            // s   (min time between two handovers of one UE)
    double hoPlExp = 3.5;            // path-loss exponent of the RSRP proxy
    bool trafficPhases = true;       // 4-phase schedule (Table 5.3) vs. constant baseline
    double loadScale = 1.0;          // multiplies every traffic rate
    bool weightTest = false;  // eMBB share 25 % between +10 s and +20 s (checks that weights work)
    bool flowmon = false;     // legacy FlowMonitor end-of-run CSVs (slow)
    uint32_t printEvery = 50; // console KPI line every N steps (0 = off)
    bool verbose = false;
    bool pcap = false;
    uint32_t seed = 42;
    uint32_t run = 1;
    std::string prefix = "major";

    CommandLine cmd(__FILE__);
    cmd.AddValue("nUe", "Total UEs (shared across 3 gNBs)", nUe);
    cmd.AddValue("episodeLen", "RL episode length in seconds", episodeLen);
    cmd.AddValue("appStart", "Traffic/episode start time in seconds", appStart);
    cmd.AddValue("windowMs", "KPI window = agent step in ms", windowMs);
    cmd.AddValue("hoMode", "Handover trigger: scripted | off", hoMode);
    cmd.AddValue("hoHyst", "A3-style hysteresis in dB", hoHyst);
    cmd.AddValue("hoTttMs", "A3-style time-to-trigger in ms", hoTttMs);
    cmd.AddValue("hoDwell", "Minimum seconds between two handovers of a UE", hoDwell);
    cmd.AddValue("hoPlExp", "Path-loss exponent of the RSRP proxy", hoPlExp);
    cmd.AddValue("trafficPhases", "Enable the 4-phase traffic schedule", trafficPhases);
    cmd.AddValue("loadScale", "Global traffic rate multiplier", loadScale);
    cmd.AddValue("weightTest", "Throttle eMBB share between +10 s and +20 s", weightTest);
    cmd.AddValue("flowmon", "Also write legacy FlowMonitor CSVs", flowmon);
    cmd.AddValue("printEvery", "Print a KPI line every N steps", printEvery);
    cmd.AddValue("verbose", "Enable NS_LOG", verbose);
    cmd.AddValue("pcap", "Enable PCAP traces on P2P links", pcap);
    cmd.AddValue("seed", "RngSeedManager seed", seed);
    cmd.AddValue("run", "RngSeedManager run number (independent streams)", run);
    cmd.AddValue("prefix", "Output file prefix", prefix);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run);

    const double simTime = appStart + episodeLen;
    const uint32_t nSteps = static_cast<uint32_t>(std::lround(episodeLen * 1000.0 / windowMs));
    const uint32_t nGnb = 3;

    if (verbose)
    {
        LogComponentEnable("SlicingSimulation", LOG_LEVEL_ALL);
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
    }

    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    // NOTE: the former ns3::A3RsrpHandoverAlgorithm defaults were removed on purpose:
    // they configure an LTE-only algorithm that is not installed on NR gNBs. The A3-style
    // trigger is now implemented in handover-manager.h and configured with --hoHyst/--hoTttMs.

    // ==========================================
    // Node creation
    // ==========================================
    NodeContainer gnbNodes;
    gnbNodes.Create(nGnb);
    NodeContainer ueNodes;
    ueNodes.Create(nUe);
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // ==========================================
    // Topology (3-gNB Urban Macro)
    // ==========================================
    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    gnbPositionAlloc->Add(Vector(100.0, 75.0, 25.0));  // gNB-0
    gnbPositionAlloc->Add(Vector(400.0, 75.0, 25.0));  // gNB-1
    gnbPositionAlloc->Add(Vector(250.0, 333.0, 25.0)); // gNB-2

    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);

    Ptr<RandomBoxPositionAllocator> uePositionAlloc = CreateObject<RandomBoxPositionAllocator>();
    uePositionAlloc->SetAttribute("X",
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
    uePositionAlloc->SetAttribute("Y",
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
    uePositionAlloc->SetAttribute("Z", StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                "Speed",
                                StringValue("ns3::UniformRandomVariable[Min=1.0|Max=5.0]"),
                                "Pause",
                                StringValue("ns3::UniformRandomVariable[Min=1.0|Max=5.0]"),
                                "PositionAllocator",
                                PointerValue(uePositionAlloc));
    ueMobility.SetPositionAllocator(uePositionAlloc);
    ueMobility.Install(ueNodes);

    Ptr<ListPositionAllocator> corePositionAlloc = CreateObject<ListPositionAllocator>();
    corePositionAlloc->Add(Vector(250.0, -50.0, 0.0));
    MobilityHelper coreMobility;
    coreMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    coreMobility.SetPositionAllocator(corePositionAlloc);
    coreMobility.Install(remoteHostContainer);

    // ==========================================
    // Internet / EPC
    // ==========================================
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    // X2 attributes must be set BEFORE AddX2Interface() to have any effect.
    epcHelper->SetAttribute("X2LinkDataRate", DataRateValue(DataRate("10Gbps")));
    epcHelper->SetAttribute("X2LinkDelay",
                            TimeValue(MicroSeconds(500))); // MilliSeconds(0.5) == 0 !

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<ConstantPositionMobilityModel> pgwMobility =
        pgw->GetObject<ConstantPositionMobilityModel>();
    if (!pgwMobility)
    {
        pgwMobility = CreateObject<ConstantPositionMobilityModel>();
        pgw->AggregateObject(pgwMobility);
    }
    pgwMobility->SetPosition(Vector(250.0, -30.0, 0.0));

    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    if (pcap)
    {
        p2ph.EnablePcapAll(prefix + "-slicing");
    }
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // ==========================================
    // NR helper and operation bands
    // ==========================================
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf0(6.0e9, 100e6, 1, BandwidthPartInfo::UMa);
    OperationBandInfo band0 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf0);
    CcBwpCreator::SimpleOperationBandConf bandConf1(3.5e9, 100e6, 1, BandwidthPartInfo::UMa);
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);
    CcBwpCreator::SimpleOperationBandConf bandConf2(700e6, 10e6, 1, BandwidthPartInfo::RMa);
    OperationBandInfo band2 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf2);

    nrHelper->InitializeOperationBand(&band0, NrHelper::INIT_PROPAGATION | NrHelper::INIT_CHANNEL);
    nrHelper->InitializeOperationBand(&band1, NrHelper::INIT_PROPAGATION | NrHelper::INIT_CHANNEL);
    nrHelper->InitializeOperationBand(&band2, NrHelper::INIT_PROPAGATION | NrHelper::INIT_CHANNEL);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band0, band1, band2});

    // ==========================================
    // Antennas
    // ==========================================
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // ==========================================
    // BWP manager: QCI -> BWP (= slice)
    // ==========================================
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(1));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(2));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(1));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(2));

    // (1d) the weighted PF scheduler is now the one actually installed.
    nrHelper->SetSchedulerTypeId(NrMacSchedulerWeightedPF::GetTypeId());

    // ==========================================
    // Install devices
    // ==========================================
    NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    const double txPower = 43.0; // dBm
    const std::string pattern = "F|F|F|F|F|F|F|F|F|F|";
    const uint32_t numerology[3] = {3, 1, 0};
    for (auto it = enbNetDev.Begin(); it != enbNetDev.End(); ++it)
    {
        Ptr<NetDevice> dev = *it;
        for (uint32_t b = 0; b < 3; ++b)
        {
            nrHelper->GetGnbPhy(dev, b)->SetAttribute("Numerology", UintegerValue(numerology[b]));
            nrHelper->GetGnbPhy(dev, b)->SetAttribute("TxPower", DoubleValue(txPower));
            nrHelper->GetGnbPhy(dev, b)->SetAttribute("Pattern", StringValue(pattern));
        }
        DynamicCast<NrGnbNetDevice>(dev)->UpdateConfig();
    }
    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // ==========================================
    // X2 between gNBs (links configured above)
    // ==========================================
    epcHelper->AddX2Interface(gnbNodes.Get(0), gnbNodes.Get(1));
    epcHelper->AddX2Interface(gnbNodes.Get(1), gnbNodes.Get(2));
    epcHelper->AddX2Interface(gnbNodes.Get(0), gnbNodes.Get(2));

    // ==========================================
    // IP on UEs
    // ==========================================
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // ==========================================
    // Attach
    // ==========================================
    nrHelper->AttachToClosestEnb(ueNetDev, enbNetDev);

    // ==========================================
    // Phase 1 objects (all owned here, destroyed after Simulator::Destroy)
    // ==========================================
    auto kpi = std::make_unique<SliceKpiCollector>(nGnb, nUe, MilliSeconds(windowMs), prefix);
    kpi->SetUeNodes(ueNodes);

    // (1a) handover: traces on the RRC objects (must be after attach so the RRCs exist)
    auto ho = std::make_unique<HandoverManager>(nrHelper, enbNetDev, ueNetDev, gnbNodes, ueNodes);
    ho->ConnectTraces(kpi.get());
    kpi->SetServingGnbFn([&ho](uint32_t u) { return ho->ServingGnb(u); });
    Simulator::Schedule(Seconds(std::min(0.5, appStart * 0.5)),
                        &HandoverManager::PrintRrcStates,
                        ho.get(),
                        std::string("after-attach"));

    // (1d) run-time control hooks
    auto ctl = std::make_unique<SliceControl>(nrHelper, enbNetDev, kpi.get());

#ifdef SLICING_PRB_TRACE
    for (uint32_t g = 0; g < nGnb; ++g)
    {
        for (uint32_t s = 0; s < NUM_SLICES; ++s)
        {
            nrHelper->GetGnbPhy(enbNetDev.Get(g), s)
                ->TraceConnectWithoutContext("SlotDataStats",
                                             MakeBoundCallback(&PrbTrace, kpi.get(), g, s));
        }
    }
#endif

    // ==========================================
    // Bearers + traffic (all three flows are UdpClient/UdpServer pairs)
    // ==========================================
    const uint32_t urllcBytes = 512;
    const uint32_t embbBytes =
        1400; // payload incl. 12 B SeqTs header; < MTU so no IP fragmentation
    const uint32_t mmtcBytes = 64;

    // (1c) phase controller (needs the payload size of the eMBB flow)
    std::vector<TrafficPhase> phaseTable = TrafficPhaseController::DefaultPhases();
    if (!trafficPhases)
    {
        phaseTable.resize(1); // constant baseline
    }
    auto phases = std::make_unique<TrafficPhaseController>(phaseTable, embbBytes, loadScale);
    kpi->SetPhaseFn([&phases]() { return phases->CurrentPhase(); });

    const uint16_t dlPortURLLC = 3000;
    const uint16_t dlPortEMBB = 4000;
    const uint16_t dlPortMMTC = 5000;
    std::map<uint16_t, std::pair<uint32_t, std::string>> portToUeSlice;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;

    auto makeBearer = [&](Ptr<NetDevice> ueDev, EpsBearer::Qci qci, uint8_t tos) {
        EpsBearer bearer(qci);
        Ptr<EpcTft> tft = Create<EpcTft>();
        EpcTft::PacketFilter pf;
        pf.localPortStart = 0;
        pf.localPortEnd = 65535;
        pf.typeOfService = tos;
        pf.typeOfServiceMask = 0xFF;
        tft->Add(pf);
        nrHelper->ActivateDedicatedEpsBearer(ueDev, bearer, tft);
    };

    auto makeFlow = [&](uint32_t u,
                        uint32_t slice,
                        uint16_t port,
                        uint8_t tos,
                        uint32_t bytes,
                        Ipv4Address ueIp) {
        UdpServerHelper server(port);
        ApplicationContainer sApp = server.Install(ueNodes.Get(u));
        serverApps.Add(sApp);

        InetSocketAddress dest(ueIp, port);
        dest.SetTos(tos);
        UdpClientHelper client(dest);
        client.SetAttribute("PacketSize", UintegerValue(bytes));
        client.SetAttribute("Interval", TimeValue(phases->InitialInterval(slice)));
        client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        ApplicationContainer cApp = client.Install(remoteHost);
        clientApps.Add(cApp);

        Ptr<UdpClient> c = DynamicCast<UdpClient>(cApp.Get(0));
        Ptr<UdpServer> s = DynamicCast<UdpServer>(sApp.Get(0));
        kpi->RegisterFlow(u, slice, c, s); // (1b)
        phases->AddFlow(u, slice, c);      // (1c)
        portToUeSlice[port] = {u, SliceName(slice)};
    };

    for (uint32_t u = 0; u < nUe; ++u)
    {
        Ptr<NetDevice> ueDevice = ueNetDev.Get(u);
        const Ipv4Address ueIpv4 = Ipv4Address::ConvertFrom(ueIpIface.GetAddress(u));

        // one PDU session, three QoS flows classified by DSCP (ToS = DSCP << 2)
        makeBearer(ueDevice, EpsBearer::GBR_CONV_VOICE, 0xB8);         // EF   -> BWP-0 uRLLC
        makeBearer(ueDevice, EpsBearer::NGBR_LOW_LAT_EMBB, 0x28);      // AF11 -> BWP-1 eMBB
        makeBearer(ueDevice, EpsBearer::NGBR_VIDEO_TCP_DEFAULT, 0x00); // CS0  -> BWP-2 mMTC

        makeFlow(u, SLICE_URLLC, dlPortURLLC + u, 0xB8, urllcBytes, ueIpv4);
        makeFlow(u, SLICE_EMBB, dlPortEMBB + u, 0x28, embbBytes, ueIpv4);
        makeFlow(u, SLICE_MMTC, dlPortMMTC + u, 0x00, mmtcBytes, ueIpv4);
    }

    serverApps.Start(Seconds(appStart));
    clientApps.Start(Seconds(appStart));
    serverApps.Stop(Seconds(simTime + 0.2));
    clientApps.Stop(Seconds(simTime));

    // ==========================================
    // Episode start: everything that must begin together at t = appStart
    // ==========================================
    std::vector<int> initialGnb(nUe, -1);
    Simulator::Schedule(Seconds(appStart), [&]() {
        for (uint32_t u = 0; u < nUe; ++u)
        {
            initialGnb[u] = ho->ServingGnb(u);
        }
        phases->Schedule();                         // (1c) phase changes
        kpi->Start(nSteps);                         // (1b) first window starts now
        ctl->SetSliceWeightsAll({{1.0, 1.0, 1.0}}); // (1d) equal weights => all shares 100 %
        for (uint32_t g = 0; g < nGnb; ++g)
        {
            ctl->SetTxPowerFraction(g, 1.0);
        }
        if (hoMode == "scripted")
        {
            ho->StartScripted(MilliSeconds(windowMs),
                              hoHyst,
                              MilliSeconds(static_cast<int64_t>(hoTttMs)),
                              Seconds(hoDwell),
                              hoPlExp);
        }
    });

    if (weightTest)
    {
        // Proof that the weights act: eMBB throughput must drop in [+10 s, +20 s).
        Simulator::Schedule(Seconds(appStart + 10.0),
                            [&]() { ctl->SetSliceWeightsAll({{1.0, 0.25, 1.0}}); });
        Simulator::Schedule(Seconds(appStart + 20.0),
                            [&]() { ctl->SetSliceWeightsAll({{1.0, 1.0, 1.0}}); });
    }

    if (printEvery > 0)
    {
        kpi->SetStepCallback([printEvery, nGnb](const StepKpi& r) {
            if (r.step % printEvery != 0)
            {
                return;
            }
            std::cout << "[STEP " << r.step << "] t=" << r.t << "s phase=" << r.phase;
            for (uint32_t g = 0; g < nGnb; ++g)
            {
                const auto& k = r.gnb[g];
                std::cout << " | gNB" << g << " nUE=" << k.nUe << " eMBB=" << std::fixed
                          << std::setprecision(1) << k.slice[SLICE_EMBB].tputMbps
                          << "Mbps uRLLC-p99=" << std::setprecision(2)
                          << k.slice[SLICE_URLLC].p99DelayMs << "ms";
            }
            std::cout << "\n";
        });
    }

    // ==========================================
    // Flow Monitor (legacy end-of-run statistics; off by default)
    // ==========================================
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor;
    if (flowmon)
    {
        monitor = flowmonHelper.InstallAll();
    }

    Simulator::Stop(Seconds(simTime + 0.5));
    Simulator::Run();

    // ==========================================
    // Post-run
    // ==========================================
    kpi->Finish();
    ho->PrintSummary();
    ho->WriteCsv(prefix + "-handover-events.csv");
    std::cout << "[OUTPUT] " << prefix << "-step-kpi.csv, -gnb-step.csv, -ue-step.csv, "
              << "-ue-flow-step.csv, -handover-events.csv\n";

    // Initial vs. final serving gNB per UE (the old code wrote the same value twice)
    {
        std::ofstream f(prefix + "-ue-serving.csv");
        f << "UE_ID,Initial_gNB,Final_gNB,Changed\n";
        for (uint32_t u = 0; u < nUe; ++u)
        {
            const int fin = ho->ServingGnb(u);
            f << u << "," << initialGnb[u] << "," << fin << "," << (fin != initialGnb[u] ? 1 : 0)
              << "\n";
        }
    }

    if (flowmon)
    {
        monitor->CheckForLostPackets();
        monitor->SerializeToXmlFile(prefix + "-flow-stats.xml", true, true);
        Ptr<Ipv4FlowClassifier> classifier =
            DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
        FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

        std::ofstream ueSliceCsv(prefix + "-ue-slice-stats.csv");
        ueSliceCsv << "UE_ID,Slice,Dest_Port,Tx_Packets,Rx_Packets,Throughput_Mbps,Mean_Delay_ms,"
                      "Loss_Rate\n";
        for (auto i = stats.begin(); i != stats.end(); ++i)
        {
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
            auto m = portToUeSlice.find(t.destinationPort);
            if (m == portToUeSlice.end())
            {
                continue; // control-plane / non-slice flow
            }
            const auto& fs = i->second;
            const double tput = fs.rxBytes * 8.0 / episodeLen / 1e6;
            const double delay =
                fs.rxPackets > 0 ? fs.delaySum.GetSeconds() * 1e3 / fs.rxPackets : 0.0;
            const double loss =
                fs.txPackets > 0 ? double(fs.txPackets - fs.rxPackets) / fs.txPackets : 0.0;
            ueSliceCsv << m->second.first << "," << m->second.second << "," << t.destinationPort
                       << "," << fs.txPackets << "," << fs.rxPackets << "," << std::fixed
                       << std::setprecision(4) << tput << "," << delay << ","
                       << std::setprecision(6) << loss << "\n";
        }
    }

    Simulator::Destroy();
    return 0;
}