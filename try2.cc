#include "ns3/callback.h"
#include "ns3/basic-energy-source.h"
#include "ns3/command-line.h"
#include "ns3/wifi-helper.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/wifi-mode.h"
#include "ns3/wifi-phy.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/dsdv-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/config.h"
#include "ns3/wifi-utils.h"
#include "ns3/callback.h"
#include "ns3/mobility-model.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/wifi-net-device.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiTry2");

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

void RssiGet(std::string context,Ptr<const Packet> packet, uint16_t channelFreq, WifiTxVector tx,MpduInfo mpdu,SignalNoiseDbm snr)
{
  std::cout<<"signal:"<<snr.signal<<std::endl;


}

double GetDistance(Ptr<Node> A,Ptr<Node> B){    //计算节点A和B之间的距离
  Ptr<MobilityModel> mobility_A = A->GetObject<MobilityModel>();
  double distance = B->GetObject<MobilityModel>()->GetDistanceFrom(mobility_A);

  return distance;
}

template <int node>
void RemainingEnergyTrace (double oldValue, double newValue)
{
  std::stringstream ss;
  ss << "energy_" << node << ".log";

  static std::fstream f (ss.str ().c_str (), std::ios::out);

  f << Simulator::Now ().GetSeconds () << "    remaining energy=" << newValue << std::endl;
}



int main (int argc,char* argv[])
{
    std::string phyMode ("DsssRate1Mbps");
    uint16_t nums = 20;
    uint16_t nPackets = 1;
    uint16_t PacketSize = 1000;
    double interval = 1.0;
    double txCurrent = 0.0102;
    double txPowerStart = -1;
    double idleCurrent = 0.000729;
    double voltage = 3.7;

    CommandLine cmd(__FILE__);
    cmd.Parse (argc,argv);
    LogComponentEnable("WifiTry2",LOG_LEVEL_FUNCTION);

    Time interPacketInterval = Seconds(interval);

    NodeContainer wNode;
    wNode.Create (nums);

    WifiHelper wifi;
    
    wifi.SetStandard(WIFI_STANDARD_80211b);

    YansWifiChannelHelper wifiChannel;
    wifiChannel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel",
                                    "Frequency",DoubleValue(2.412e+09),
                                    "MinDistance",DoubleValue(1.0),
                                    "HeightAboveZ",DoubleValue(1.0));
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    

    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode",StringValue(phyMode),"ControlMode",StringValue(phyMode));
    //通过wifiPhy.Set来设置物理层参数,大概要实现在能量、功率或者距离上的限制
    //自组织网络
    wifiPhy.Set("TxGain",DoubleValue(-10));
    wifiPhy.Set("TxPowerStart",DoubleValue(-1));
    wifiPhy.Set("TxPowerEnd",DoubleValue(-1));
    //问题：如何实现多跳？

    NetDeviceContainer wifiDevice = wifi.Install(wifiPhy,wifiMac,wNode);  //device上可以添加error model实现丢包
    

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator","X",StringValue("ns3::UniformRandomVariable[Min=100.0|Max=200.0]"),
                                                                          "Y",StringValue("ns3::UniformRandomVariable[Min=100.0|Max=200.0]"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wNode);
    NS_LOG_INFO("Topology built");
    /*  以上代码经过waf编译无错误*/

    DsdvHelper dsdv;   //想办法实现自己的helper类
    InternetStackHelper internet;   
    internet.SetRoutingHelper(dsdv);
    internet.Install(wNode);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0","255.255.255.0");
    Ipv4InterfaceContainer ipc = ipv4.Assign(wifiDevice);

  
    std::cout<<ipc.GetAddress(0,0)<<std::endl;
    double dist;
    dist = GetDistance(wNode.Get(0),wNode.Get(1));
    std::cout<<dist<<std::endl;


    EnergySourceContainer sourceContainer;
    BasicEnergySourceHelper basicEnergy;
    WifiRadioEnergyModelHelper wifiEnergy;
    
   
    //下面配置电池和用电模型
    basicEnergy.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(226440));
    basicEnergy.Set("BasicEnergySupplyVoltageV",DoubleValue(3.7));

    wifiEnergy.Set("TxCurrentA",DoubleValue(0.0102));
    wifiEnergy.Set("RxCurrentA",DoubleValue(0.00729));
    wifiEnergy.Set("IdleCurrentA",DoubleValue(0.000729));
    wifiEnergy.Set("SleepCurrentA",DoubleValue(4.38e-7));

    double eta = DbmToW (txPowerStart) / ((txCurrent - idleCurrent) * voltage);

    wifiEnergy.SetTxCurrentModel("ns3::LinearWifiTxCurrentModel",
                                  "Voltage",DoubleValue(voltage),
                                  "IdleCurrent",DoubleValue(idleCurrent),
                                  "Eta",DoubleValue(eta));

  for(NodeContainer::Iterator i = wNode.Begin();i!=wNode.End();i++)   //给安装电源和用电模型
  {
    sourceContainer.Add(basicEnergy.Install(*i));
    Ptr<WifiNetDevice> wifinetdevice;

    for(uint32_t j = 0;j!=(*i)->GetNDevices();j++)
    {
      wifinetdevice = (*i)->GetDevice(j)->GetObject<WifiNetDevice>();
      if(wifinetdevice!=0)
      {
        wifiEnergy.Install(wifinetdevice,sourceContainer.Get(sourceContainer.GetN()-1));

      }

    }

  }

  sourceContainer.Get(6)->TraceConnectWithoutContext("TotalEnergyConsumption", MakeCallback(&RemainingEnergyTrace<0>));  //跟踪能量消耗

    //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",MakeCallback(&RssiGet));  //跟踪RSSI
    //怎样跟踪包交付呢？


    /* 配置节点实现发包*/


    ApplicationContainer app;
  
    OnOffHelper onoff ("ns3::UdpSocketFactory",InetSocketAddress(ipc.GetAddress (6,0),80));  //发送地址
    onoff.SetAttribute("DataRate",DataRateValue (DataRate ("1Mbps")));
    onoff.SetAttribute("PacketSize",UintegerValue(PacketSize));
    onoff.SetAttribute("OnTime",StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime",StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    AddressValue remote (InetSocketAddress(ipc.GetAddress (10,0),80));   //远端目的地址
    onoff.SetAttribute("Remote",remote);

    app = onoff.Install(wNode.Get(6));
    app.Start(Seconds(0.01));
    app.Stop(Seconds(30.0));

    PacketSinkHelper packetsink ("ns3::UdpSocketFactory",InetSocketAddress(Ipv4Address::GetAny(),80));

    app = packetsink.Install(wNode.Get(10));

    app.Start(Seconds(0.01));
    app.Stop(Seconds(30.0));
    NS_LOG_INFO("App Set");

    
    Simulator::Stop(Seconds(32.0));
    Simulator::Run();
    Simulator::Destroy();


    



}