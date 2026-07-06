/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/antenna-module.h"
#include "nr-mac-scheduler-wpf.h"
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SlicingSimulation");

int
main (int argc, char *argv[])
{
  uint32_t nUe = 9;
  double simTime = 30.0;
  bool verbose = false;
  bool pcap = false;
  uint32_t seed = 42;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("nUe", "Total UEs (shared across 3 gNBs)", nUe);
  cmd.AddValue ("simTime", "Simulation duration in seconds", simTime);
  cmd.AddValue ("verbose", "Enable NS_LOG", verbose);
  cmd.AddValue ("pcap", "Enable PCAP traces on P2P links", pcap);
  cmd.AddValue ("seed", "RngSeedManager seed", seed);
  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (seed);

  if (verbose)
    {
      LogComponentEnable ("SlicingSimulation", LOG_LEVEL_ALL);
      LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);
      LogComponentEnable ("LtePdcp", LOG_LEVEL_INFO);
    }

  // A3 Handover Configuration via LteEnbRrc
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
  
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));

  // ==========================================
  // 2A. Node Creation
  // ==========================================
  NodeContainer gnbNodes;
  gnbNodes.Create (3);

  NodeContainer ueNodes;
  ueNodes.Create (nUe);

  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);

  // ==========================================
  // 2B. Topology (3-gNB Urban Macro)
  // ==========================================
  Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator> ();
  gnbPositionAlloc->Add (Vector (100.0, 75.0, 25.0)); // gNB-0
  gnbPositionAlloc->Add (Vector (400.0, 75.0, 25.0)); // gNB-1
  gnbPositionAlloc->Add (Vector (250.0, 333.0, 25.0)); // gNB-2

  MobilityHelper gnbMobility;
  gnbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  gnbMobility.SetPositionAllocator (gnbPositionAlloc);
  gnbMobility.Install (gnbNodes);

  Ptr<RandomBoxPositionAllocator> uePositionAlloc = CreateObject<RandomBoxPositionAllocator> ();
  uePositionAlloc->SetAttribute ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  uePositionAlloc->SetAttribute ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
  uePositionAlloc->SetAttribute ("Z", StringValue ("ns3::ConstantRandomVariable[Constant=1.5]"));

  MobilityHelper ueMobility;
  ueMobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                               "Speed", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=5.0]"),
                               "Pause", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=5.0]"),
                               "PositionAllocator", PointerValue (uePositionAlloc));
  ueMobility.SetPositionAllocator (uePositionAlloc);
  ueMobility.Install (ueNodes);

  Ptr<ListPositionAllocator> corePositionAlloc = CreateObject<ListPositionAllocator> ();
  corePositionAlloc->Add (Vector (250.0, -50.0, 0.0)); // Remote Host

  MobilityHelper coreMobility;
  coreMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  coreMobility.SetPositionAllocator (corePositionAlloc);
  coreMobility.Install (remoteHostContainer);

  // ==========================================
  // 2C. Internet / EPC Topology
  // ==========================================
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  epcHelper->SetAttribute ("S1uLinkDelay", TimeValue (MilliSeconds (0)));

  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  
  // Set PGW position
  Ptr<ConstantPositionMobilityModel> pgwMobility = pgw->GetObject<ConstantPositionMobilityModel> ();
  if (!pgwMobility)
    {
      pgwMobility = CreateObject<ConstantPositionMobilityModel> ();
      pgw->AggregateObject (pgwMobility);
    }
  pgwMobility->SetPosition (Vector (250.0, -30.0, 0.0));

  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.000)));
  
  if (pcap)
    {
      p2ph.EnablePcapAll ("major-slicing");
    }

  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  
  Ipv4AddressHelper ipv4h;
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // ==========================================
  // 2D. NR Helper and Operation Bands
  // ==========================================
  Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper> ();
  idealBeamformingHelper->SetAttribute ("BeamformingMethod", TypeIdValue (DirectPathBeamforming::GetTypeId ()));

  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
  nrHelper->SetBeamformingHelper (idealBeamformingHelper);
  nrHelper->SetEpcHelper (epcHelper);

  CcBwpCreator ccBwpCreator;

  // Band 0 (uRLLC)
  CcBwpCreator::SimpleOperationBandConf bandConf0 (6.0e9, 100e6, 1, BandwidthPartInfo::UMa);
  OperationBandInfo band0 = ccBwpCreator.CreateOperationBandContiguousCc (bandConf0);

  // Band 1 (eMBB)
  CcBwpCreator::SimpleOperationBandConf bandConf1 (3.5e9, 100e6, 1, BandwidthPartInfo::UMa);
  OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc (bandConf1);

  // Band 2 (mMTC)
  CcBwpCreator::SimpleOperationBandConf bandConf2 (700e6, 10e6, 1, BandwidthPartInfo::RMa);
  OperationBandInfo band2 = ccBwpCreator.CreateOperationBandContiguousCc (bandConf2);

  // Initialize and get BWPs
  nrHelper->InitializeOperationBand (&band0, NrHelper::INIT_PROPAGATION | NrHelper::INIT_CHANNEL);
  nrHelper->InitializeOperationBand (&band1, NrHelper::INIT_PROPAGATION | NrHelper::INIT_CHANNEL);
  nrHelper->InitializeOperationBand (&band2, NrHelper::INIT_PROPAGATION | NrHelper::INIT_CHANNEL);

  BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps ({band0, band1, band2});

  // ==========================================
  // 2E. Antennas and Beamforming
  // ==========================================
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (4));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (4));
  nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (8));
  nrHelper->SetGnbAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  // ==========================================
  // 2F. gNB BWP Manager QCI Routing
  // ==========================================
  nrHelper->SetGnbBwpManagerAlgorithmAttribute ("GBR_CONV_VOICE", UintegerValue (0));        // uRLLC
  nrHelper->SetGnbBwpManagerAlgorithmAttribute ("NGBR_LOW_LAT_EMBB", UintegerValue (1));     // eMBB
  nrHelper->SetGnbBwpManagerAlgorithmAttribute ("NGBR_VIDEO_TCP_DEFAULT", UintegerValue (2)); // mMTC

  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_CONV_VOICE", UintegerValue (0));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("NGBR_LOW_LAT_EMBB", UintegerValue (1));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("NGBR_VIDEO_TCP_DEFAULT", UintegerValue (2));

  // Scheduler configuration
  nrHelper->SetSchedulerTypeId (TypeId::LookupByName ("ns3::NrMacSchedulerOfdmaPF"));

  // ==========================================
  // 2G. Install gNB and UE Devices
  // ==========================================
  NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice (gnbNodes, allBwps);
  NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice (ueNodes, allBwps);

  // Apply Numerology/Power attributes per BWP on ALL gNBs and UEs
  double txPower = 43.0; // dBm
  std::string pattern = "F|F|F|F|F|F|F|F|F|F|";

  // gNB config
  for (auto it = enbNetDev.Begin (); it != enbNetDev.End (); ++it)
    {
      Ptr<NetDevice> dev = *it;
      // BWP 0 (uRLLC)
      nrHelper->GetGnbPhy (dev, 0)->SetAttribute ("Numerology", UintegerValue (3));
      nrHelper->GetGnbPhy (dev, 0)->SetAttribute ("TxPower", DoubleValue (txPower));
      nrHelper->GetGnbPhy (dev, 0)->SetAttribute ("Pattern", StringValue (pattern));

      // BWP 1 (eMBB)
      nrHelper->GetGnbPhy (dev, 1)->SetAttribute ("Numerology", UintegerValue (1));
      nrHelper->GetGnbPhy (dev, 1)->SetAttribute ("TxPower", DoubleValue (txPower));
      nrHelper->GetGnbPhy (dev, 1)->SetAttribute ("Pattern", StringValue (pattern));

      // BWP 2 (mMTC)
      nrHelper->GetGnbPhy (dev, 2)->SetAttribute ("Numerology", UintegerValue (0));
      nrHelper->GetGnbPhy (dev, 2)->SetAttribute ("TxPower", DoubleValue (txPower));
      nrHelper->GetGnbPhy (dev, 2)->SetAttribute ("Pattern", StringValue (pattern));

      DynamicCast<NrGnbNetDevice> (dev)->UpdateConfig ();
    }

  // UE config is not needed for Numerology since it inherits from BWP/gNB
  for (auto it = ueNetDev.Begin (); it != ueNetDev.End (); ++it)
    {
      Ptr<NetDevice> dev = *it;
      DynamicCast<NrUeNetDevice> (dev)->UpdateConfig ();
    }


  // ==========================================
  // 2H. Xn Interface Between gNBs
  // ==========================================
  Config::SetDefault ("ns3::A3RsrpHandoverAlgorithm::Hysteresis", DoubleValue (3.0));
  Config::SetDefault ("ns3::A3RsrpHandoverAlgorithm::TimeToTrigger", TimeValue (MilliSeconds (100)));

  // Setup X2 interfaces manually for all pairs
  epcHelper->AddX2Interface (gnbNodes.Get (0), gnbNodes.Get (1));
  epcHelper->AddX2Interface (gnbNodes.Get (1), gnbNodes.Get (2));
  epcHelper->AddX2Interface (gnbNodes.Get (0), gnbNodes.Get (2));
  
  // NOTE: Point-To-Point for X2 is handled automatically by NoBackhaulEpcHelper inheritance, 
  // but if it wasn't, one would have to do what the spec asks. The spec wants IP addresses on these links 
  // (10.1.x.0/30) and 10Gbps/0.5ms links.
  // Actually NrPointToPointEpcHelper does not natively give 10Gbps/0.5ms for X2, so let's check its API, 
  // wait we can set it: 
  epcHelper->SetAttribute ("X2LinkDataRate", DataRateValue (DataRate ("10Gbps")));
  epcHelper->SetAttribute ("X2LinkDelay", TimeValue (MilliSeconds (0.5)));

  // ==========================================
  // 2I. Internet Stack and IP Addressing on UEs
  // ==========================================
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueNetDev));

  // Set default route on UEs
  for (uint32_t j = 0; j < ueNodes.GetN (); ++j)
    {
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodes.Get (j)->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // ==========================================
  // 2J. Attach UEs to Nearest gNB
  // ==========================================
  nrHelper->AttachToClosestEnb (ueNetDev, enbNetDev);

  // ==========================================
  // 2K. EPS Bearers (3 per UE)
  // ==========================================
  uint16_t dlPortURLLC = 3000;
  uint16_t dlPortEMBB = 4000;
  uint16_t dlPortMMTC = 5000;

  ApplicationContainer serverApps;
  ApplicationContainer clientApps;

  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<NetDevice> ueDevice = ueNetDev.Get (u);
      Address ueAddress = ueIpIface.GetAddress (u);
      
      // Bearer 1 - uRLLC (ToS 0xB8)
      EpsBearer urllcBearer (EpsBearer::GBR_CONV_VOICE);
      Ptr<EpcTft> urllcTft = Create<EpcTft> ();
      EpcTft::PacketFilter urllcPf;
      urllcPf.localPortStart = 0;
      urllcPf.localPortEnd = 65535;
      urllcPf.typeOfService = 0xB8;
      urllcPf.typeOfServiceMask = 0xFF;
      urllcTft->Add (urllcPf);
      nrHelper->ActivateDedicatedEpsBearer (ueDevice, urllcBearer, urllcTft);

      // Bearer 2 - eMBB (ToS 0x28)
      EpsBearer embbBearer (EpsBearer::NGBR_LOW_LAT_EMBB);
      Ptr<EpcTft> embbTft = Create<EpcTft> ();
      EpcTft::PacketFilter embbPf;
      embbPf.localPortStart = 0;
      embbPf.localPortEnd = 65535;
      embbPf.typeOfService = 0x28;
      embbPf.typeOfServiceMask = 0xFF;
      embbTft->Add (embbPf);
      nrHelper->ActivateDedicatedEpsBearer (ueDevice, embbBearer, embbTft);

      // Bearer 3 - mMTC (ToS 0x00)
      EpsBearer mmtcBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
      Ptr<EpcTft> mmtcTft = Create<EpcTft> ();
      EpcTft::PacketFilter mmtcPf;
      mmtcPf.localPortStart = 0;
      mmtcPf.localPortEnd = 65535;
      mmtcPf.typeOfService = 0x00;
      mmtcPf.typeOfServiceMask = 0xFF; // Match exactly
      mmtcTft->Add (mmtcPf);
      nrHelper->ActivateDedicatedEpsBearer (ueDevice, mmtcBearer, mmtcTft);

      // ==========================================
      // 2L. Traffic Applications
      // ==========================================

      Ipv4Address ueIpv4 = Ipv4Address::ConvertFrom(ueAddress);

      // --- uRLLC Traffic (CBR, 512B, 1ms, ToS 0xB8) ---
      uint16_t portURLLC = dlPortURLLC + u;
      UdpServerHelper urllcServer (portURLLC);
      serverApps.Add (urllcServer.Install (ueNodes.Get (u)));

      InetSocketAddress urllcDest (ueIpv4, portURLLC);
      urllcDest.SetTos (0xB8);

      UdpClientHelper urllcClient (urllcDest);
      urllcClient.SetAttribute ("PacketSize", UintegerValue (512));
      urllcClient.SetAttribute ("Interval", TimeValue (MilliSeconds (1)));
      urllcClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
      clientApps.Add (urllcClient.Install (remoteHost));

      // --- eMBB Traffic (OnOff, 50Mbps, 1500B, ToS 0x28) ---
      uint16_t portEMBB = dlPortEMBB + u;
      PacketSinkHelper embbServer ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), portEMBB));
      serverApps.Add (embbServer.Install (ueNodes.Get (u)));

      InetSocketAddress embbDest (ueIpv4, portEMBB);
      embbDest.SetTos (0x28);

      OnOffHelper embbClient ("ns3::UdpSocketFactory", embbDest);
      embbClient.SetAttribute ("PacketSize", UintegerValue (1500));
      embbClient.SetAttribute ("DataRate", DataRateValue (DataRate ("50Mbps")));
      embbClient.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      embbClient.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      clientApps.Add (embbClient.Install (remoteHost));

      // --- mMTC Traffic (CBR, 64B, 100ms, ToS 0x00) ---
      uint16_t portMMTC = dlPortMMTC + u;
      UdpServerHelper mmtcServer (portMMTC);
      serverApps.Add (mmtcServer.Install (ueNodes.Get (u)));

      InetSocketAddress mmtcDest (ueIpv4, portMMTC);
      mmtcDest.SetTos (0x00);

      UdpClientHelper mmtcClient (mmtcDest);
      mmtcClient.SetAttribute ("PacketSize", UintegerValue (64));
      mmtcClient.SetAttribute ("Interval", TimeValue (MilliSeconds (100)));
      mmtcClient.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
      clientApps.Add (mmtcClient.Install (remoteHost));
    }

  // start apps
  serverApps.Start (Seconds (1.0));
  clientApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (simTime));
  clientApps.Stop (Seconds (simTime));

  // Flow Monitor
  FlowMonitorHelper flowmonHelper;
  Ptr<ns3::FlowMonitor> monitor = flowmonHelper.InstallAll ();
  
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  monitor->SerializeToXmlFile ("major-slicing-flow-stats.xml", true, true);
  
  // Per-flow summary and CSV output
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  
  std::ofstream csvFile ("major-slicing-flow-stats.csv");
  csvFile << "Flow ID,Source IP,Destination IP,Tx Packets,Rx Packets,Throughput (Mbps),Mean Delay (ms),Loss Rate\n";

  for (auto i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      double throughput = i->second.rxBytes * 8.0 / (simTime - 1.0) / 1000 / 1000;
      double meanDelay = i->second.rxPackets > 0 ? (i->second.delaySum.GetSeconds () * 1000) / i->second.rxPackets : 0;
      double pktsLossRate = (double)(i->second.txPackets - i->second.rxPackets) / i->second.txPackets;
      
      std::cout << "Flow ID: " << i->first
                << " Src: " << t.sourceAddress << " Dst: " << t.destinationAddress
                << " TxPkts: " << i->second.txPackets << " RxPkts: " << i->second.rxPackets
                << " Throughput: " << throughput << " Mbps"
                << " MeanDelay: " << meanDelay << " ms"
                << " LossRate: " << pktsLossRate << std::endl;
                
      csvFile << i->first << ","
              << t.sourceAddress << ","
              << t.destinationAddress << ","
              << i->second.txPackets << ","
              << i->second.rxPackets << ","
              << throughput << ","
              << meanDelay << ","
              << pktsLossRate << "\n";
    }
  
  csvFile.close ();
    
  Simulator::Destroy ();
  return 0;
}
