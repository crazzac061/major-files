/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include "nr-mac-scheduler-wpf.h"

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
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SlicingSimulation");

// ============================================================
// NETWORK BANDWIDTH ANALYSIS (documented here for reference)
// ============================================================
//
// Band 0 – uRLLC: 6.0 GHz carrier, 100 MHz BW, Numerology μ=3 (120 kHz SCS)
//   Peak DL per gNB  ≈  2 × 4 (UE MIMO) × 100 MHz × log2(1+SNR) ≈ ~1.5 Gbps
//
// Band 1 – eMBB: 3.5 GHz carrier, 100 MHz BW, Numerology μ=1 (30 kHz SCS)
//   Peak DL per gNB  ≈  2 × 4 (UE MIMO) × 100 MHz × log2(1+SNR) ≈ ~2.5 Gbps
//
// Band 2 – mMTC: 700 MHz carrier, 10 MHz BW, Numerology μ=0 (15 kHz SCS)
//   Peak DL per gNB  ≈  10 MHz × ~1.5 ≈ ~50 Mbps
//
// Total aggregate capacity across 3 gNBs:
//   uRLLC:  3 × 1.5  Gbps =  4.5 Gbps
//   eMBB:   3 × 2.5  Gbps =  7.5 Gbps
//   mMTC:   3 × 0.05 Gbps =  0.15 Gbps
//   -----------------------------------------
//   TOTAL   ≈  12.15 Gbps theoretical peak
//
// Backhaul: 100 Gb/s P2P – NOT a bottleneck.
// X2 links: 10 Gbps, 0.5 ms – adequate for handover signaling.
//
// UE SCALING:
//   With OFDMA-PF scheduler on 100 MHz eMBB band, you can support 50–100 UEs
//   per gNB before per-UE throughput degrades below useful levels.
//   Default increased to 18 UEs (6 per gNB) – change with --nUe=<N>.
// ============================================================

// ============================================================
// Global structures for per-UE tracking
// ============================================================

struct UeRecord
{
    uint32_t ueIdx;
    Ipv4Address ueIp;
    std::string slice; // "uRLLC", "eMBB", "mMTC"
    uint16_t port;
};

// Handover event log entries
struct HandoverEvent
{
    double time;
    uint32_t ueIdx;
    uint64_t srcGnbImsi;
    uint64_t tgtGnbImsi;
    double ueX;
    double ueY;
};

static std::vector<HandoverEvent> g_handoverLog;

// Serving cell log (UE idx → current gNB IMSI)
static std::map<uint32_t, uint64_t> g_servingCell;

// UE mobility snapshots
struct MobilitySnapshot
{
    double time;
    uint32_t ueIdx;
    double x;
    double y;
    uint64_t servingGnb;
};

static std::vector<MobilitySnapshot> g_mobilityLog;

// NodeContainer accessible to callbacks
static NodeContainer g_ueNodes;

// ============================================================
// Trace callbacks
// ============================================================

void
HandoverStartCallback(uint32_t ueIdx,
                      std::string context,
                      uint64_t imsi,
                      uint16_t cellId,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    Ptr<MobilityModel> mob = g_ueNodes.Get(ueIdx)->GetObject<MobilityModel>();
    Vector pos = mob ? mob->GetPosition() : Vector(0, 0, 0);

    HandoverEvent ev;
    ev.time = Simulator::Now().GetSeconds();
    ev.ueIdx = ueIdx;
    ev.srcGnbImsi = imsi; // LTE stack uses IMSI for cell identity here
    ev.tgtGnbImsi = targetCellId;
    ev.ueX = pos.x;
    ev.ueY = pos.y;
    g_handoverLog.push_back(ev);

    NS_LOG_INFO("[HO-START] t=" << ev.time << "s  UE-" << ueIdx << "  cell " << cellId << " -> "
                                << targetCellId << "  pos=(" << pos.x << "," << pos.y << ")");
}

void
HandoverEndOkCallback(uint32_t ueIdx,
                      std::string context,
                      uint64_t imsi,
                      uint16_t cellId,
                      uint16_t rnti)
{
    g_servingCell[ueIdx] = cellId;
    NS_LOG_INFO("[HO-OK]    t=" << Simulator::Now().GetSeconds() << "s  UE-" << ueIdx
                                << "  now at cell " << cellId);
}

// ============================================================
// Periodic UE mobility snapshot (called every 1 s)
// ============================================================

void
LogMobilitySnapshot(uint32_t nUe, double simTime)
{
    double now = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < nUe; ++i)
    {
        Ptr<MobilityModel> mob = g_ueNodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mob ? mob->GetPosition() : Vector(0, 0, 0);
        MobilitySnapshot snap;
        snap.time = now;
        snap.ueIdx = i;
        snap.x = pos.x;
        snap.y = pos.y;
        snap.servingGnb = g_servingCell.count(i) ? g_servingCell[i] : 0;
        g_mobilityLog.push_back(snap);
    }

    // Reschedule only if still within simulation time
    if (now + 1.0 < simTime)
        Simulator::Schedule(Seconds(1.0), &LogMobilitySnapshot, nUe, simTime);
}

// ============================================================
// Main
// ============================================================

int
main(int argc, char* argv[])
{
    uint32_t nUe = 18; // increased default: 6 per gNB
    double simTime = 30.0;
    bool verbose = false;
    bool pcap = false;
    uint32_t seed = 42;

    CommandLine cmd(__FILE__);
    cmd.AddValue("nUe", "Total UEs (shared across 3 gNBs)", nUe);
    cmd.AddValue("simTime", "Simulation duration in seconds", simTime);
    cmd.AddValue("verbose", "Enable NS_LOG", verbose);
    cmd.AddValue("pcap", "Enable PCAP traces on P2P links", pcap);
    cmd.AddValue("seed", "RngSeedManager seed", seed);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seed);

    if (verbose)
    {
        LogComponentEnable("SlicingSimulation", LOG_LEVEL_ALL);
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("LtePdcp", LOG_LEVEL_INFO);
    }

    // A3 Handover Configuration
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::A3RsrpHandoverAlgorithm::Hysteresis", DoubleValue(3.0));
    Config::SetDefault("ns3::A3RsrpHandoverAlgorithm::TimeToTrigger", TimeValue(MilliSeconds(100)));

    // ==========================================
    // 2A. Node Creation
    // ==========================================
    NodeContainer gnbNodes;
    gnbNodes.Create(3);

    NodeContainer ueNodes;
    ueNodes.Create(nUe);
    g_ueNodes = ueNodes; // make accessible to callbacks

    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // ==========================================
    // 2B. Topology (3-gNB Urban Macro)
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
    // 2C. Internet / EPC Topology
    // ==========================================
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

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
        p2ph.EnablePcapAll("major-slicing");

    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // ==========================================
    // 2D. NR Helper and Operation Bands
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
    // 2E. Antennas and Beamforming
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
    // 2F. gNB BWP Manager QCI Routing
    // ==========================================
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(1));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(2));

    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(1));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(2));

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerOfdmaPF"));

    // ==========================================
    // 2G. Install gNB and UE Devices
    // ==========================================
    NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    double txPower = 43.0; // dBm
    std::string pattern = "F|F|F|F|F|F|F|F|F|F|";

    for (auto it = enbNetDev.Begin(); it != enbNetDev.End(); ++it)
    {
        Ptr<NetDevice> dev = *it;
        nrHelper->GetGnbPhy(dev, 0)->SetAttribute("Numerology", UintegerValue(3));
        nrHelper->GetGnbPhy(dev, 0)->SetAttribute("TxPower", DoubleValue(txPower));
        nrHelper->GetGnbPhy(dev, 0)->SetAttribute("Pattern", StringValue(pattern));

        nrHelper->GetGnbPhy(dev, 1)->SetAttribute("Numerology", UintegerValue(1));
        nrHelper->GetGnbPhy(dev, 1)->SetAttribute("TxPower", DoubleValue(txPower));
        nrHelper->GetGnbPhy(dev, 1)->SetAttribute("Pattern", StringValue(pattern));

        nrHelper->GetGnbPhy(dev, 2)->SetAttribute("Numerology", UintegerValue(0));
        nrHelper->GetGnbPhy(dev, 2)->SetAttribute("TxPower", DoubleValue(txPower));
        nrHelper->GetGnbPhy(dev, 2)->SetAttribute("Pattern", StringValue(pattern));

        DynamicCast<NrGnbNetDevice>(dev)->UpdateConfig();
    }

    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();

    // ==========================================
    // 2H. Xn Interface Between gNBs
    // ==========================================
    epcHelper->AddX2Interface(gnbNodes.Get(0), gnbNodes.Get(1));
    epcHelper->AddX2Interface(gnbNodes.Get(1), gnbNodes.Get(2));
    epcHelper->AddX2Interface(gnbNodes.Get(0), gnbNodes.Get(2));

    epcHelper->SetAttribute("X2LinkDataRate", DataRateValue(DataRate("10Gbps")));
    epcHelper->SetAttribute("X2LinkDelay", TimeValue(MilliSeconds(0.5)));

    // ==========================================
    // 2I. Internet Stack and IP Addressing on UEs
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
    // 2J. Attach UEs to Nearest gNB
    // ==========================================
    nrHelper->AttachToClosestEnb(ueNetDev, enbNetDev);

    // ==========================================
    // 2K-HO. Connect Handover Trace Callbacks
    //   Must be done AFTER AttachToClosestEnb so RRC instances exist.
    //   Traces fire on UE-side RRC: HandoverStarted / HandoverEndOk
    // ==========================================
    for (uint32_t u = 0; u < nUe; ++u)
    {
        // Initial serving cell (closest gNB at t=0)
        Ptr<MobilityModel> ueMob = ueNodes.Get(u)->GetObject<MobilityModel>();
        Ptr<MobilityModel> gnb0Mob = gnbNodes.Get(0)->GetObject<MobilityModel>();
        Ptr<MobilityModel> gnb1Mob = gnbNodes.Get(1)->GetObject<MobilityModel>();
        Ptr<MobilityModel> gnb2Mob = gnbNodes.Get(2)->GetObject<MobilityModel>();

        double d0 = ueMob->GetDistanceFrom(gnb0Mob);
        double d1 = ueMob->GetDistanceFrom(gnb1Mob);
        double d2 = ueMob->GetDistanceFrom(gnb2Mob);

        uint64_t initCellId = 0;
        if (d0 <= d1 && d0 <= d2)
            initCellId = 1; // gNB cell IDs start at 1
        else if (d1 <= d0 && d1 <= d2)
            initCellId = 2;
        else
            initCellId = 3;
        g_servingCell[u] = initCellId;

        // HandoverStart trace: /NodeList/<ueNode>/DeviceList/0/LteUeRrc/HandoverStart
        std::ostringstream pathStart;
        pathStart << "/NodeList/" << ueNodes.Get(u)->GetId()
                  << "/DeviceList/0/LteUeRrc/HandoverStart";
        Config::Connect(pathStart.str(), MakeBoundCallback(&HandoverStartCallback, u));

        // HandoverEndOk trace
        std::ostringstream pathEnd;
        pathEnd << "/NodeList/" << ueNodes.Get(u)->GetId()
                << "/DeviceList/0/LteUeRrc/HandoverEndOk";
        Config::Connect(pathEnd.str(), MakeBoundCallback(&HandoverEndOkCallback, u));
    }

    // Start periodic mobility snapshot at t=1s (after apps start)
    Simulator::Schedule(Seconds(1.0), &LogMobilitySnapshot, nUe, simTime);

    // ==========================================
    // 2K. EPS Bearers (3 per UE) + Applications
    // ==========================================
    uint16_t dlPortURLLC = 3000;
    uint16_t dlPortEMBB = 4000;
    uint16_t dlPortMMTC = 5000;

    ApplicationContainer serverApps;
    ApplicationContainer clientApps;

    // Per-flow identification: flowKey → (ueIdx, slice)
    // Flow key = destination port, since each UE has unique port per slice
    std::map<uint16_t, std::pair<uint32_t, std::string>> portToUeSlice;

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<NetDevice> ueDevice = ueNetDev.Get(u);
        Address ueAddress = ueIpIface.GetAddress(u);
        Ipv4Address ueIpv4 = Ipv4Address::ConvertFrom(ueAddress);

        // ---- Bearer 1: uRLLC ----
        EpsBearer urllcBearer(EpsBearer::GBR_CONV_VOICE);
        Ptr<EpcTft> urllcTft = Create<EpcTft>();
        EpcTft::PacketFilter urllcPf;
        urllcPf.localPortStart = 0;
        urllcPf.localPortEnd = 65535;
        urllcPf.typeOfService = 0xB8;
        urllcPf.typeOfServiceMask = 0xFF;
        urllcTft->Add(urllcPf);
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, urllcBearer, urllcTft);

        // ---- Bearer 2: eMBB ----
        EpsBearer embbBearer(EpsBearer::NGBR_LOW_LAT_EMBB);
        Ptr<EpcTft> embbTft = Create<EpcTft>();
        EpcTft::PacketFilter embbPf;
        embbPf.localPortStart = 0;
        embbPf.localPortEnd = 65535;
        embbPf.typeOfService = 0x28;
        embbPf.typeOfServiceMask = 0xFF;
        embbTft->Add(embbPf);
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, embbBearer, embbTft);

        // ---- Bearer 3: mMTC ----
        EpsBearer mmtcBearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
        Ptr<EpcTft> mmtcTft = Create<EpcTft>();
        EpcTft::PacketFilter mmtcPf;
        mmtcPf.localPortStart = 0;
        mmtcPf.localPortEnd = 65535;
        mmtcPf.typeOfService = 0x00;
        mmtcPf.typeOfServiceMask = 0xFF;
        mmtcTft->Add(mmtcPf);
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, mmtcBearer, mmtcTft);

        // ---- uRLLC Traffic: CBR 512B @ 1ms, ToS=0xB8 ----
        uint16_t portURLLC = dlPortURLLC + u;
        portToUeSlice[portURLLC] = {u, "uRLLC"};
        UdpServerHelper urllcServer(portURLLC);
        serverApps.Add(urllcServer.Install(ueNodes.Get(u)));

        InetSocketAddress urllcDest(ueIpv4, portURLLC);
        urllcDest.SetTos(0xB8);
        UdpClientHelper urllcClient(urllcDest);
        urllcClient.SetAttribute("PacketSize", UintegerValue(512));
        urllcClient.SetAttribute("Interval", TimeValue(MilliSeconds(1)));
        urllcClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        clientApps.Add(urllcClient.Install(remoteHost));

        // ---- eMBB Traffic: OnOff 50 Mbps 1500B, ToS=0x28 ----
        uint16_t portEMBB = dlPortEMBB + u;
        portToUeSlice[portEMBB] = {u, "eMBB"};
        PacketSinkHelper embbServer("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), portEMBB));
        serverApps.Add(embbServer.Install(ueNodes.Get(u)));

        InetSocketAddress embbDest(ueIpv4, portEMBB);
        embbDest.SetTos(0x28);
        OnOffHelper embbClient("ns3::UdpSocketFactory", embbDest);
        embbClient.SetAttribute("PacketSize", UintegerValue(1500));
        embbClient.SetAttribute("DataRate", DataRateValue(DataRate("50Mbps")));
        embbClient.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        embbClient.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        clientApps.Add(embbClient.Install(remoteHost));

        // ---- mMTC Traffic: CBR 64B @ 100ms, ToS=0x00 ----
        uint16_t portMMTC = dlPortMMTC + u;
        portToUeSlice[portMMTC] = {u, "mMTC"};
        UdpServerHelper mmtcServer(portMMTC);
        serverApps.Add(mmtcServer.Install(ueNodes.Get(u)));

        InetSocketAddress mmtcDest(ueIpv4, portMMTC);
        mmtcDest.SetTos(0x00);
        UdpClientHelper mmtcClient(mmtcDest);
        mmtcClient.SetAttribute("PacketSize", UintegerValue(64));
        mmtcClient.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
        mmtcClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        clientApps.Add(mmtcClient.Install(remoteHost));
    }

    serverApps.Start(Seconds(1.0));
    clientApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(simTime));
    clientApps.Stop(Seconds(simTime));

    // ==========================================
    // Flow Monitor
    // ==========================================
    FlowMonitorHelper flowmonHelper;
    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // ==========================================
    // Post-run: Write output files
    // ==========================================

    monitor->SerializeToXmlFile("major-slicing-flow-stats.xml", true, true);
    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    // ---- (1) Legacy per-flow CSV (kept for backwards compatibility) ----
    {
        std::ofstream csvFile("major-slicing-flow-stats.csv");
        csvFile << "Flow ID,Source IP,Destination IP,Tx Packets,Rx Packets,"
                   "Throughput (Mbps),Mean Delay (ms),Loss Rate\n";

        for (auto i = stats.begin(); i != stats.end(); ++i)
        {
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
            double throughput = i->second.rxBytes * 8.0 / (simTime - 1.0) / 1e6;
            double meanDelay =
                i->second.rxPackets > 0
                    ? (i->second.delaySum.GetSeconds() * 1000.0) / i->second.rxPackets
                    : 0;
            double lossRate =
                (double)(i->second.txPackets - i->second.rxPackets) / i->second.txPackets;

            std::cout << "Flow " << i->first << " Src=" << t.sourceAddress
                      << " Dst=" << t.destinationAddress << " Tx=" << i->second.txPackets
                      << " Rx=" << i->second.rxPackets << " Tput=" << throughput << " Mbps"
                      << " Delay=" << meanDelay << " ms"
                      << " Loss=" << lossRate << "\n";

            csvFile << i->first << "," << t.sourceAddress << "," << t.destinationAddress << ","
                    << i->second.txPackets << "," << i->second.rxPackets << "," << throughput << ","
                    << meanDelay << "," << lossRate << "\n";
        }
        csvFile.close();
    }

    // ---- (2) Per-UE per-slice CSV ----
    {
        std::ofstream ueSliceCsv("major-ue-slice-stats.csv");
        ueSliceCsv << "UE_ID,Slice,Dest_Port,Dest_IP,"
                      "Tx_Packets,Rx_Packets,Throughput_Mbps,Mean_Delay_ms,Loss_Rate,"
                      "Initial_ServingCell,Final_ServingCell\n";

        for (auto i = stats.begin(); i != stats.end(); ++i)
        {
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);

            // Only classify downlink flows (remoteHost → UE); skip control flows
            uint16_t dstPort = t.destinationPort;
            if (portToUeSlice.find(dstPort) == portToUeSlice.end())
                continue;

            auto [ueIdx, slice] = portToUeSlice[dstPort];

            double throughput = i->second.rxBytes * 8.0 / (simTime - 1.0) / 1e6;
            double meanDelay =
                i->second.rxPackets > 0
                    ? (i->second.delaySum.GetSeconds() * 1000.0) / i->second.rxPackets
                    : 0;
            double lossRate =
                (double)(i->second.txPackets - i->second.rxPackets) / i->second.txPackets;

            // Initial cell from our map (set at t=0)
            uint64_t finalCell = g_servingCell.count(ueIdx) ? g_servingCell[ueIdx] : 0;

            ueSliceCsv << ueIdx << "," << slice << "," << dstPort << "," << t.destinationAddress
                       << "," << i->second.txPackets << "," << i->second.rxPackets << ","
                       << std::fixed << std::setprecision(4) << throughput << "," << std::fixed
                       << std::setprecision(4) << meanDelay << "," << std::fixed
                       << std::setprecision(6) << lossRate << ","
                       << (g_servingCell.count(ueIdx) ? g_servingCell[ueIdx] : (uint64_t)0) << ","
                       << finalCell << "\n";
        }
        ueSliceCsv.close();
        std::cout << "\n[OUTPUT] Per-UE per-slice stats  ->  major-ue-slice-stats.csv\n";
    }

    // ---- (3) Handover event log CSV ----
    {
        std::ofstream hoCsv("major-handover-events.csv");
        hoCsv << "Time_s,UE_ID,Source_CellID,Target_CellID,UE_X,UE_Y\n";
        for (const auto& ev : g_handoverLog)
        {
            hoCsv << std::fixed << std::setprecision(3) << ev.time << "," << ev.ueIdx << ","
                  << ev.srcGnbImsi << "," << ev.tgtGnbImsi << "," << std::setprecision(2) << ev.ueX
                  << "," << ev.ueY << "\n";
        }
        hoCsv.close();
        std::cout << "[OUTPUT] Handover events         ->  major-handover-events.csv"
                  << "  (" << g_handoverLog.size() << " events)\n";
    }

    // ---- (4) UE Mobility + Serving Cell timeline CSV ----
    {
        std::ofstream mobCsv("major-ue-mobility.csv");
        mobCsv << "Time_s,UE_ID,X_m,Y_m,Serving_CellID\n";
        for (const auto& snap : g_mobilityLog)
        {
            mobCsv << std::fixed << std::setprecision(1) << snap.time << "," << snap.ueIdx << ","
                   << std::setprecision(2) << snap.x << "," << snap.y << "," << snap.servingGnb
                   << "\n";
        }
        mobCsv.close();
        std::cout << "[OUTPUT] UE mobility timeline    ->  major-ue-mobility.csv"
                  << "  (" << g_mobilityLog.size() << " snapshots)\n";
    }

    Simulator::Destroy();
    return 0;
}
