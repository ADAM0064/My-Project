#include <iostream>
#include <algorithm>
#include <utility>
#include "ns3/basic-energy-source.h"
#include "ns3/csma-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/qlearning.h"
#include "ns3/string.h"
#include "ns3/command-line.h"
#include "ns3/qlearning.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/callback.h"
#include "ns3/csma-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/wifi-utils.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/csma-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-net-device.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/random-variable-stream.h"


using namespace ns3;


/**
 * 需要完成的事情：
 * 1、寻路代码的验证和完善
 * 2、路由转发的端口转发
 * 3、reward的获取（latency和lifetime的测量）
 * 4、PRN、吞吐量等信息的获取和统计
 * 
 * 存在的问题：
 * 1、转发可能不如预期
 * 2、电池在初始化配置时存在问题，用wifinetdevice可能有bug
 * 3、应用层发包不知道怎么配置
 */

//上行节点信息（预期）
struct NodeInfo{
    uint32_t index;     //node container内的下标，相当于节点id，IP地址交给ipv4address container储存，应该与下标对应
    uint32_t hop;       //跳数
    bool battery;   //是否是电池供电,true代表是

};

//邻接表内信息
struct EdgeInfo{
    uint32_t index;
    bool battery;
    double distance;            //可以考虑套用模型计算损耗，发射功率相同损耗越小越好
    bool inUplink = false;      //记录是否在上行图内
    uint32_t hop_cache = 0;     //暂存的跳数

    EdgeInfo(uint32_t id,bool p,double d)
    {
        this->index = id;
        this->battery = p;
        this->distance = d;
    }

};

struct SuccessorInfo{
    uint32_t index;
    uint32_t hop;
    bool battery;
    double distance;
    double cost = 0;

    SuccessorInfo(uint32_t id, uint32_t h,uint16_t p,double d)
        {
            this->index = id;
            this->hop = h;
            this->battery = p;
            this->distance = d;
        }

    bool operator < (SuccessorInfo s)       //调用sort需要重载
    {
        return this->cost < s.cost;
    }
    bool operator > (SuccessorInfo s)
    {
        return this->cost > s.cost;
    }
};
 bool CompareCost(const SuccessorInfo &A,const SuccessorInfo&B)
{
  return A.cost < B.cost;
}

class TryRouting:public Qlearning
{
  public:
  //开关
    bool printRoutingTable = false;//打印路由表

  //函数
    //A、B之间的距离
    double GetNodeDistance(Ptr<Node> A,Ptr<Node> B); 
    //初始化拓扑图
    void InitializeGraph(NodeContainer container);
    //向拓扑图中添加信息
    void UpdateGraph(NodeContainer nodeContainer,double range);

    //初始化信息
    void InitializeUpList(NodeContainer nodeContainer);   
    //寻找和上行节点有多条边连接的候选节点
    bool FindNode2Up();     

    void ChooseUpNode(double wh,double wp,double ws,double dist);

    bool FindNode1UP();

    void Choose1WayUP();

    void InitializeRouting(double range);

    void Run_epoch(double wh,double wp,double ws,double dist);

    void PrintBaseGraph();

    void PrintRemainNodeList();

    void PrintUplinkNodeList();

    void Run_EpochWithQvalue(double reward , double distance);
    //

    void SetUpNodes();

    void SetUpAdhoc();

   // void SetupAddresses();

    void SetupEnergy();

    void SetupBasicRouting();

    void SetRoutingOnNode(uint32_t nodeIndex,uint32_t targetA, uint32_t targetB);

    void SetRoutingOnNode(uint32_t nodeIndex,uint32_t target);
    //重构时执行，将到目的地址的路由信息删除
    void RemoveRoutingOnNode(uint32_t nodeIndex);

    void SetupApplication();
    //在Simulator::Run之后调用
    void MonitorEvent();

    double GetLifeTime();
    
//流量监控
    FlowMonitorHelper flowMoHelper;   //只能创建一次
    Ptr<FlowMonitor> flowMonitor;   //唯一的监控
   

  //容器
    NodeContainer allNodes; //全部节点,大小23，下标0-23，0、1、2分别是网关和AP1、AP2
    NodeContainer adhocNodes; //包含AP1和AP2和其他节点，大小22,节点下标1-23,使用时是0-22
    NodeContainer remainNodesContainer;  //除了三个特殊节点以外的全部节点，共20个
   // NodeContainer csmaNodes;  //包含网关和AP1和AP2,大小3,下标0-2

    NetDeviceContainer adhocDevice;
    //NetDeviceContainer csmaDevice;

   // Ipv4InterfaceContainer csmaAddr;  
    Ipv4InterfaceContainer adhocAddr; //节点下标1-23,使用时应是0-22，使用index时记得减1

    EnergySourceContainer energyContainer;  //包含了用电池的节点的电池模型

    Ipv4InterfaceAddress DestAddr = Ipv4InterfaceAddress (Ipv4Address ("192.168.1.1"), Ipv4Mask ("/32"));
  private:
    //double defaultRange = 30.0;   //距离值
    //运行SetupNodes
    uint32_t MonitorNum = 0;  //运行Monitor的次数

    std::vector<double> oldConsumption;   //记录能量消耗值
    

    std::map<uint32_t,std::vector<EdgeInfo>> baseGraph; //拓扑图G,vector的插入其实效率更高

    std::vector<NodeInfo> upNodes;    //已经加入的节点，只增不减

    std::vector<uint32_t> remainNodes;  //未加入的节点，已知信息应该只有下标和用电信息，只减不增

    std::vector<SuccessorInfo> candidate2UP;  //和上行节点有多条边相连的节点下标

    std::vector<NodeInfo> candidate1wayUP;

    std::vector<SuccessorInfo> successors;

    std::map<uint32_t,std::vector<SuccessorInfo>> successorMap;
};


double 
TryRouting::GetNodeDistance(Ptr<Node> A,Ptr<Node> B)  
{    
  Ptr<MobilityModel> mobility_A = A->GetObject<MobilityModel>();
  double distance = B->GetObject<MobilityModel>()->GetDistanceFrom(mobility_A);

  return distance;
}

void 
TryRouting::InitializeGraph(NodeContainer container)
{
  std::vector<EdgeInfo> nullnodelist;  //空列表
  for(uint32_t i=0;i!=container.GetN();i++)
  {
    uint32_t id = container.Get(i)->GetId();
   // std::cout<<"ID:"<<id<<std::endl;
    baseGraph.insert(std::make_pair(id,nullnodelist));

  }

  if(baseGraph.empty())   //如果图结构是空的
  {
    std::cerr<<"Graph empty!"<<std::endl;
    //NS3的警告机制以及报错,或者抛出异常
  }
}

void 
TryRouting::UpdateGraph(NodeContainer nodeContainer,double range)
{
  double distance = 0;
  for(uint32_t i=0;i!=nodeContainer.GetN();i++)   //A
  {
    for(uint32_t j=i+1;j!=nodeContainer.GetN();j++)   //B
    {
      distance = GetNodeDistance(nodeContainer.Get(i),nodeContainer.Get(j));

      if(distance < range && distance > 0)         //这里的范围非常关键
      {
        uint32_t idA = nodeContainer.Get(i)->GetId(); 
        uint32_t idB = nodeContainer.Get(j)->GetId();
        if(idA == 1&& idB==2)
        {
          continue;
        }
        bool battery = false;
        if((idB!=2)&&(idB%2 == 1))    //B不是AP2
        {
            battery = true;
        }
       
        EdgeInfo edge = {idB,battery,distance}; //A到B

        std::map<uint32_t,std::vector<EdgeInfo>>::iterator map_it = baseGraph.find(idA);
        if(map_it!=baseGraph.end())
        {
            map_it->second.push_back(edge); //A到B
        }
        
        battery = false;
        if((idA!=1)&&(idA%2 == 1))  //A不是AP1
        {
            battery = true;
        }
        
        edge.index = idA;
        edge.battery = battery;
        map_it = baseGraph.find(idB);
        if(map_it!=baseGraph.end())
        {
            map_it->second.push_back(edge); //B到A
        }

      }

    }
  }
}

//初始化节点的列表
void
TryRouting::InitializeUpList(NodeContainer nodeContainer) //这里node container是存的除网关接入点外的全部节点的容器
{
  //这里的网关和接入点的index都没想好，只是暂定
   NodeInfo GW = {0,0,false};
   NodeInfo AP1 = {1,1,false};
   NodeInfo AP2 = {2,1,false};

   upNodes.push_back(GW);
   upNodes.push_back(AP1);
   upNodes.push_back(AP2);

   remainNodes.resize(nodeContainer.GetN());

   for(uint32_t i = 0;i!=remainNodes.size();i++)
   {
     uint32_t id = nodeContainer.Get(i)->GetId();
     //std::cout<<"Remain iD"<<id<<std::endl;
     remainNodes[i] = nodeContainer.Get(i)->GetId();    //节点id，预期中给个别id的节点添加电源，可以考虑成奇数id的节点
     //节点ID应该是唯一的
   }

}

bool
TryRouting::FindNode2Up()   //为了避免重复，要不要在进行Find之前清空候选列表？
{
  candidate2UP.clear();       //为了避免内容重复，每次使用前清空，这样只要节点列表内容唯一，候选列表内容也唯一
  uint16_t count;
  for(uint32_t i = 0;i!=remainNodes.size();i++)
  {
    uint32_t nodeIndex = remainNodes[i];  
    std::map<uint32_t,std::vector<EdgeInfo>>::iterator edge = baseGraph.find(nodeIndex);
    if(edge != baseGraph.end())
    {
        count = 0;
        //目的是想在寻找有无多个上行节点的同时更新信息
        for(uint32_t k = 0;k!=upNodes.size();k++)
        {  
          for(uint32_t j = 0;j!=edge->second.size();j++)
          {
            /*
            if(edge->second[j].inUplink)    //已经在上行图内（此前遍历过）,有问题就删掉
            {
              count++;
              continue;
            }
              */
            if(edge->second[j].index == upNodes[k].index)   //下标对应相等，说明邻接表中的节点是上行节点
            {
                count++;
                edge->second[j].hop_cache = upNodes[k].hop;   //顺便进行信息更新
                edge->second[j].inUplink = true;
            }
          }
        }

        if(count>1)
        {
          bool power = false;
          if(nodeIndex%2==1)
          {
            power = true;
          }
          std::cout<<"Candidate Node 2-Edge index:"<<nodeIndex<<" ,power:"<<power<<std::endl;
          candidate2UP.push_back({nodeIndex,0,power,0});    //除了index都是默认值，因为这里都不知道
          //问题:这里可能会存在重复，所以要检查重复内容，或者直接在使用前clear
        }
      /**
       *  不在这里处理一条边节点的原因是，这一次只有一条边的节点，在下一次搜寻中可能有两条或者多条边，这样就会造成两个列表之间内容的重复，
       *  所以不如两条边的点找不到了再找一条边的。
       *  那么有没有必要检查两个列表中是否有重复的内容呢？如果有重复内容该怎么处理呢？
       *  还要检查upNodes里是否有重复内容。。
       */
       
    }
  }
  if(!candidate2UP.empty())  //找到了
  {
    return true;
  }
  else        //没找到,接下来该找和上行节点有一条边相连的节点
  {
    return false;
  }
    
  
}

void 
TryRouting::ChooseUpNode(double wh,double wp,double ws,double dist)   //需要在前一个函数返回true后运行
{
  
  uint32_t max_hop = 0;
  uint32_t MAX_HOP = 0;

  for(uint32_t i = 0;i!=candidate2UP.size();i++)
  {
    successors.clear();   //缓存的下一跳节点列表，对应某个候选节点
    uint32_t nodeIndex = candidate2UP[i].index;    //候选下标
    std::map<uint32_t,std::vector<EdgeInfo>>::iterator base_it = baseGraph.find(nodeIndex);
    if(base_it!=baseGraph.end())
    {
      for(uint32_t j = 0;j!=base_it->second.size();j++)
      {
        if(base_it->second[j].inUplink)     //
        {
          SuccessorInfo s {base_it->second[j].index,base_it->second[j].hop_cache,base_it->second[j].battery,base_it->second[j].distance};
          successors.push_back(s);
          max_hop =std::max(base_it->second[j].hop_cache, max_hop);
        }
      }
    }

      for(std::vector<SuccessorInfo>::iterator it = successors.begin();it!=successors.end();it++)
      {
        uint16_t power = it->battery?1:0;
        it->cost =  wh * (double)(it->hop*1.0/max_hop) + wp * power + ws * (std::min(it->distance - dist, 0.0)/dist);   //计算下一跳节点的cost
      }
      std::sort(successors.begin(),successors.end(),CompareCost);   //预期是按照升序排序的;顺序其实不重要
      //在当前index下
      candidate2UP[i].hop = 1 + (successors[0].hop + successors[1].hop) / 2 ;
      candidate2UP[i].distance = (successors[0].distance + successors[1].distance)/2.0;
      MAX_HOP = std::max(candidate2UP[i].hop , MAX_HOP);
      
      //此处应该把获得的上行节点信息存储起来，目前想到用map结构

      std::map<uint32_t, std::vector<SuccessorInfo>>::iterator s_it = successorMap.find(nodeIndex);
      //避免内容重复
      if(s_it!=successorMap.end())  //如果找到了，则可以更新；已经在上行图中的节点信息就不更新了
      {
        s_it->second.clear();
        s_it->second = {successors[0],successors[1]};
      }
      else          //未找到，则写入新的信息
      {
        std::vector<SuccessorInfo> tempVec = {successors[0],successors[1]};
        successorMap.insert(std::make_pair(nodeIndex, tempVec));    //目的是要避免重复index的出现
      }

  }


  for(std::vector<SuccessorInfo>::iterator it = candidate2UP.begin();it!=candidate2UP.end();it++)
  {
    uint16_t power = it->battery?1:0;
    it->cost = it->cost = wh * (double)(it->hop*1.0/max_hop) + wp * power + ws * (std::min(it->distance - dist, 0.0)/dist);   //计算候选节点的cost
  }

  std::sort(candidate2UP.begin(),candidate2UP.end(),CompareCost);
  std::vector<SuccessorInfo>::iterator best_node = candidate2UP.begin();      //选出候选节点
  std::cout<<"* Best 2-Edge Candidate Node index: "<<best_node->index<<" ,hop: "<<best_node->hop<<" , distance: "<<best_node->distance<<std::endl;
  std::map<uint32_t, std::vector<SuccessorInfo>>::iterator s_it = successorMap.find(best_node->index);
  std::cout<<"* Next Hop index A: "<<s_it->second[0].index<<" ,index B: "<<s_it->second[1].index<<std::endl;
//要进行写入的前提，一是要设置node，二是要配置device和ip地址  
  SetRoutingOnNode(best_node->index,s_it->second[0].index,s_it->second[1].index);

  struct NodeInfo info = {best_node->index,best_node->hop,best_node->battery};
  upNodes.push_back(info);      //加入上行节点列表中

  for(std::vector<uint32_t>::iterator rit = remainNodes.begin();rit != remainNodes.end();) //从节点列表中删除候选节点
  {
    if(*rit== best_node->index)   //这里要保证两个vector里的index都是唯一的
    {
        rit = remainNodes.erase(rit);
    }
    else
    {
        rit++;
    }
  }
/*    如果每次都用新列表就不用删除了
  for(std::vector<SuccessorInfo>::iterator cit = candidate2UP.begin();cit != candidate2UP.end();) //从候选列表里删除候选节点
  {
    if(cit->index == best_node->index)   //这里要保证两个vector里的index都是唯一的
    {
        cit = candidate2UP.erase(cit);
    }
    else
    {
        cit++;
    }
  }
*/
  //std::map<uint32_t,std::vector<SuccessorInfo>>::iterator succ_it = successorMap.find(best_node->index); //2个下一跳节点,应该进行路由表的写入处理

}
  

//这个函数运行时应该所有多边的节点都已经遍历到了，只剩下一条边的节点;
//也就是说，在FindNode2UP返回false时运行
bool
TryRouting::FindNode1UP()
{
  candidate1wayUP.clear();    //类似的，保证每次使用时是空的
  uint16_t count;
  for(uint32_t i = 0 ;i!=remainNodes.size();i++)
  {
    count = 0;   
    uint32_t nodeIndex = remainNodes[i];
    std::map<uint32_t,std::vector<EdgeInfo>>::iterator edge = baseGraph.find(nodeIndex);
    if(edge != baseGraph.end())
    {
       for(uint32_t j = 0;j!=edge->second.size();j++)
        {  
          for(uint32_t k = 0;k!=upNodes.size();k++)
          {
            if(edge->second[j].index == upNodes[k].index)   //下标对应相等，说明邻接表中的节点是上行节点
            {
              count++;
                edge->second[j].hop_cache = upNodes[k].hop;
                edge->second[j].inUplink = true;
            }
          }

        }
      if(count == 1)   //想法是找一条边的节点，想必可以和FindNode2函数整合
        {
          bool power = false;
          if(nodeIndex%2==1)
          {
            power = true;
          }
          std::cout<<"Candidate Node 1-Edge index:"<<nodeIndex<<" ,power:"<<power<<std::endl;
          candidate1wayUP.push_back({nodeIndex,0,power});
        }


    }

  }

  if(!candidate1wayUP.empty())
    return true;
  else 
    return false;
  
}

void 
TryRouting::Choose1WayUP()
{
  uint32_t size;
  uint32_t bigger_size = 0;
  //struct NodeInfo bigger_info = {0,0,false};
  uint32_t bigger_index = 0;
  for(uint32_t i=0;i!=candidate1wayUP.size();i++)
  {
    uint32_t nodeIndex = candidate1wayUP[i].index;
    std::map<uint32_t,std::vector<EdgeInfo>>::iterator edge = baseGraph.find(nodeIndex);
    if(edge != baseGraph.end())
    {
      for(uint32_t j = 0;j!=edge->second.size();j++)
      {
        if(edge->second[j].inUplink)    //按理来说这时只有一个
        {
          candidate1wayUP[i].hop = edge->second[j].hop_cache + 1;
        }
      } 
      
      size = edge->second.size();

      if(size>bigger_size)    //找到邻居数最多的节点
      {
        bigger_size = size;
        bigger_index = i;
        
      }
    }
  }

  struct NodeInfo best = candidate1wayUP[bigger_index];
  upNodes.push_back(best);        //存入上行节点列表
  uint32_t index = candidate1wayUP[bigger_index].index;
  SuccessorInfo next_hop = {0,0,false,0};
  std::map<uint32_t,std::vector<EdgeInfo>>::iterator edge = baseGraph.find(index);
  if(edge != baseGraph.end())
  {
    for(uint32_t i = 0;i!=edge->second.size();i++)
    {
      if(edge->second[i].inUplink)  //按理来说这时只有一个
        {
          next_hop.index = edge->second[i].index;
          next_hop.hop = edge->second[i].hop_cache;
          next_hop.battery = edge->second[i].battery;
          next_hop.distance = edge->second[i].distance;
        }
    }
  }
  //这时候的next_hop就是所需的下一跳节点
  std::vector<SuccessorInfo> temp = {next_hop};
  std::cout<<"* Best 1-Edge Candidate Node index:"<<best.index<<" ,hop:"<<best.hop<<std::endl;
  std::cout<<"* Next hop index:"<<next_hop.index<<std::endl;

  SetRoutingOnNode(best.index,next_hop.index);

  std::map<uint32_t, std::vector<SuccessorInfo>>::iterator s_it = successorMap.find(index);
  if(s_it!=successorMap.end())
  {
    s_it->second.clear();
    s_it->second = {next_hop};
  }
  else 
  {
    successorMap.insert(std::make_pair(index, temp));     //不存也没关系，重要的是把下一跳信息写进路由表
  }

  

  for(std::vector<uint32_t>::iterator rit = remainNodes.begin();rit != remainNodes.end();)   //从节点列表中删除该节点
  {
    if(*rit== best.index)   //这里要保证两个vector里的index都是唯一的
    {
        rit = remainNodes.erase(rit);
    }
    else
    {
        rit++;
    }
  }
/*
  for(std::vector<NodeInfo>::iterator cit = candidate1wayUP.begin();cit != candidate1wayUP.end();)
  {
    if(cit->index == best.index)   //这里要保证两个vector里的index都是唯一的
    {
        cit = candidate1wayUP.erase(cit);
    }
    else
    {
        cit++;
    }
  }
*/
}

void
TryRouting::Run_epoch(double wh, double wp, double ws, double dist)
{
  //初始化依赖节点配置！ 最好这周内完成！
  //InitializeUpList();

  while(!remainNodes.empty())
  {
    bool found = FindNode2Up();
    if(found)
    {
      ChooseUpNode( wh,  wp,  ws,  dist);
    }
    else
    {
      FindNode1UP();
      Choose1WayUP();
    }
  }
}

void
TryRouting::InitializeRouting(double range)
{
  SetUpNodes(); //配置节点
  SetUpAdhoc();   //配置adhoc
 
  SetupBasicRouting();  //配置接入点到网关的转发路由
  SetupEnergy();    //配置电池模型
  InitializeGraph(adhocNodes);   //拓扑图建立,注意不是带入全部节点，而是adhoc节点
  UpdateGraph(adhocNodes,range);   //拓扑图信息写入
  InitializeUpList(remainNodesContainer);   //初始化节点列表
  Initialize();   //Q学习的初始化
}

void
TryRouting::PrintBaseGraph()
{
  std::map<uint32_t,std::vector<EdgeInfo>>::iterator it = baseGraph.begin();
  for(;it!=baseGraph.end();it++)
  {
    std::cout<<"Node "<<it->first<<" neighbors:"<<std::endl;
    for(uint32_t i = 0;i!=it->second.size();i++)
    {
      std::cout<<"No."<<it->second[i].index<<" ,Distance:"<<it->second[i].distance<<" ,Hop:"<<it->second[i].hop_cache<<" ,in Uplink:"<<it->second[i].inUplink<<std::endl;
      
    }
    std::cout<<" "<<std::endl;
  }
}

void 
TryRouting::PrintRemainNodeList()
{
  std::cout<<"Remain Node List:"<<std::endl;
  for(uint32_t i = 0;i!=remainNodes.size();i++)
  {
    std::cout<<remainNodes[i]<<" ";
  }
  std::cout<<std::endl;
}

void 
TryRouting::PrintUplinkNodeList()
{
  std::cout<<"Uplink Node List:"<<std::endl;
  for(uint32_t i=0;i!=upNodes.size();i++)
  {
    std::cout<<"index:"<<upNodes[i].index<<" ,hop:"<<upNodes[i].hop<<std::endl;
  }
}

//引入Q值，需要给出回报、距离阈值和待加入的节点的container
void
TryRouting::Run_EpochWithQvalue(double reward,double distance)
{
  Run(reward);      //Q学习运行一轮
  struct state nowState = GetCurrentState();
  double wh = nowState.weight_H/100.0;
  double wp = nowState.weight_P/100.0;
  double ws = nowState.weight_S/100.0;
  std::cout<<"Weights:"<<wh<<","<<wp<<","<<ws<<std::endl;
  Run_epoch(wh,wp,ws,distance);
  InitializeUpList(remainNodesContainer);     //每完成一轮都得初始化表，不然就有bug
}

//配置网关、AP、adhoc节点以及p2p连接
void
TryRouting::SetUpNodes()
{
  Ptr<Node> Gateway = CreateObject<Node>();   //网关,index 0
  Ptr<Node> AP1 = CreateObject<Node>();     //接入点1,放左侧 index 1
  Ptr<Node> AP2 = CreateObject<Node>();      //接入点2,index 2

  NodeContainer basicNodes = NodeContainer(Gateway,AP1,AP2);

  MobilityHelper basicMobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  positionAlloc->Add(Vector( 0.0, 0.0, 0.0));
  positionAlloc->Add(Vector(-5.0, 0.0, 0.0));
  positionAlloc->Add(Vector( 5.0, 0.0, 0.0));
  basicMobility.SetPositionAllocator(positionAlloc);
  basicMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  basicMobility.Install(basicNodes);

  remainNodesContainer.Create(20);

  RngSeedManager::SetSeed (10);   //10
 
  MobilityHelper anotherMobility;
  anotherMobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                          "X",StringValue("0.0"),
                                          "Y",StringValue("0.0"),
                                          "Theta",StringValue("ns3::NormalRandomVariable[Mean=3.1415|Variance=10.0|Bound=3.1415]"),    //0到2pi之间的正态分布
                                          "Rho",StringValue("ns3::NormalRandomVariable[Mean=30.0|Variance=50.0|Bound=20.0]"));   //位置分布也很重要！
  anotherMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  anotherMobility.Install(remainNodesContainer);

  adhocNodes = NodeContainer(AP1,AP2,remainNodesContainer);
  //csmaNodes = NodeContainer(Gateway,AP1,AP2);
  allNodes = NodeContainer(basicNodes,remainNodesContainer);

  InternetStackHelper internet;
  internet.Install(allNodes);
//配置p2p
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute("DataRate", StringValue("6Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));
    

  NodeContainer ApLeft = NodeContainer(Gateway,AP1);
  NodeContainer ApRight = NodeContainer(Gateway,AP2);

  NetDeviceContainer LeftDevice = p2p.Install(ApLeft);      //网关和AP1的device
  NetDeviceContainer RightDevice = p2p.Install(ApRight);    //网关和AP2的device
//p2p地址
  Ipv4AddressHelper ipv4AddrHelper;
  ipv4AddrHelper.SetBase("10.10.1.0", "255.255.255.0");
  Ipv4InterfaceContainer Ap1P2PAddrs = ipv4AddrHelper.Assign(LeftDevice);  //包含网关到AP1之间的地址
  ipv4AddrHelper.SetBase("10.10.2.0", "255.255.255.0");
  Ipv4InterfaceContainer Ap2P2PAddrs = ipv4AddrHelper.Assign(RightDevice);  
  /**
   * 网关左边p2p接口地址是10.10.1.1，右边p2p接口地址是10.10.2.1
   * AP1的p2p接口地址是10.10.1.2,AP2的p2p接口地址是10.10.2.2
   * 
   */
/* 
  Ptr<Ipv4> ipv4_GW = Gateway->GetObject<Ipv4>();   //网关中的csma是第3个device
  Ptr<CsmaNetDevice> deviceGW = CreateObject<CsmaNetDevice> ();
  deviceGW->SetAddress (Mac48Address::Allocate ());
  Gateway->AddDevice (deviceGW);
  deviceGW->SetQueue (CreateObject<DropTailQueue<Packet> > ());

  int32_t ifIndexGW = ipv4_GW->AddInterface(deviceGW);    //给网关添加CSMA，地址是192.168.1.1,即目标地址
  ipv4_GW->AddAddress (ifIndexGW,DestAddr);
  ipv4_GW->SetMetric(ifIndexGW,1);
  ipv4_GW->SetUp(ifIndexGW);
*/
  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", DataRateValue (6000000));   //6Mbps
  csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (2))); //2ms
  NetDeviceContainer GWDevice = csma.Install(Gateway); 

  ipv4AddrHelper.SetBase("192.168.1.1", "255.255.255.255");
  Ipv4InterfaceContainer GWAddr = ipv4AddrHelper.Assign(GWDevice);    //目标地址 192.168.1.1

}
//配置WiFI Adhoc
void
TryRouting::SetUpAdhoc()
{
/*  
 //配置csma,改用p2p也可以
  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", DataRateValue (6000000));   //6Mbps
  csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (2))); //2ms
  csmaDevice = csma.Install(csmaNodes);
*/  
//配置wifi
  std::string phyMode ("DsssRate5_5Mbps");

  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));
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
//wifi物理层
  wifiPhy.Set("TxGain",DoubleValue(0));   //-10
  wifiPhy.Set("TxPowerStart",DoubleValue(10));   //-1
  wifiPhy.Set("TxPowerEnd",DoubleValue(10));     //-1
  wifiPhy.Set("RxGain",DoubleValue(0));       //

  WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode",StringValue(phyMode),"ControlMode",StringValue(phyMode));

  adhocDevice = wifi.Install(wifiPhy,wifiMac,adhocNodes); 
//分配adhoc的地址
  Ipv4AddressHelper ipv4AddrHelper;
  ipv4AddrHelper.SetBase("10.1.1.0", "255.255.255.0");
  adhocAddr = ipv4AddrHelper.Assign(adhocDevice);
  /**
   * AP1的adhoc接口地址是10.1.1.1,AP2的adhoc接口地址是10.1.1.2，接入点中的adhoc是第2个device
   * 其他节点(adhoc)的地址从10.1.1.3开始，一直到10.1.1.22
   * 
   */
}
  

  
//配置电池和用电模型,需要进一步验证
void
TryRouting::SetupEnergy()
{
  double txCurrent = 0.0105;    //1.05mA
  double rxCurrent = 0.0075;
  double sleepCurrent = 4.5e-5;
  double idleCurrent = 7.5e-4; 
  double switchCurrent = 0.006;
  double voltage = 3.6;
  double txPowerStart = 10;

  BasicEnergySourceHelper basicEnergy;
  WifiRadioEnergyModelHelper wifiEnergy;
    
  basicEnergy.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(220320));
  basicEnergy.Set("BasicEnergySupplyVoltageV",DoubleValue(voltage));

  wifiEnergy.Set("TxCurrentA",DoubleValue(txCurrent));
  wifiEnergy.Set("RxCurrentA",DoubleValue(rxCurrent));
  wifiEnergy.Set("IdleCurrentA",DoubleValue(idleCurrent));
  wifiEnergy.Set("SleepCurrentA",DoubleValue(sleepCurrent));
  wifiEnergy.Set("SwitchingCurrentA",DoubleValue(switchCurrent));

  double eta = DbmToW (txPowerStart) / ((txCurrent - idleCurrent) * voltage);

  wifiEnergy.SetTxCurrentModel("ns3::LinearWifiTxCurrentModel",
                                  "Voltage",DoubleValue(voltage),
                                  "IdleCurrent",DoubleValue(idleCurrent),
                                  "Eta",DoubleValue(eta));

  for(uint32_t i = 3;i!=allNodes.GetN();i++)   //0,1,2分别是网关、AP1和AP2，这里取其他adhoc节点
  {
    if(i%2 == 1)
    {
      //在奇数的index的节点上安装电源
      energyContainer.Add(basicEnergy.Install(allNodes.Get(i)));

      Ptr<WifiNetDevice> wifinetdevice;   //这里如果用wifinetdevice会报错，不知道为什么是incomplete

      //1是wifi netdevice，0是loopback
      wifinetdevice = allNodes.Get(i)->GetDevice(1)->GetObject<WifiNetDevice>();    //这里不知道对不对, 可以改成DynamicCast<WifiNetDevice>() 试试看
      if(wifinetdevice!=0)
      {
        wifiEnergy.Install(wifinetdevice,energyContainer.Get(energyContainer.GetN()-1));

      }
      //std::cout<<"Battery Node:"<<i;
    }
    //std::cout<<std::endl;
  }
  
}
//配置两个AP向网关的转发路由
void 
TryRouting::SetupBasicRouting()
{
  Ptr<Node> AP1 = allNodes.Get(1);
  Ptr<Node> AP2 = allNodes.Get(2);

  Ptr<Ipv4> ipv4_AP1 = AP1->GetObject<Ipv4>();
  Ptr<Ipv4> ipv4_AP2 = AP2->GetObject<Ipv4>();

  Ipv4StaticRoutingHelper ipv4StaticRouting;
  Ptr<Ipv4StaticRouting> AP1Routing = ipv4StaticRouting.GetStaticRouting(ipv4_AP1);
  Ptr<Ipv4StaticRouting> AP2Routing = ipv4StaticRouting.GetStaticRouting(ipv4_AP2);

  AP1Routing->AddHostRouteTo(Ipv4Address("192.168.1.1"),Ipv4Address("10.10.1.1"),1);    //1应该是p2p
  AP2Routing->AddHostRouteTo(Ipv4Address("192.168.1.1"),Ipv4Address("10.10.2.1"),1);

}


//adhoc节点的路由信息写入，对应两条边的节点
//这几个index都是全局的，在查找ip地址时应该注意减去1
void 
TryRouting::SetRoutingOnNode(uint32_t nodeIndex,uint32_t targetA, uint32_t targetB)
{
  RemoveRoutingOnNode(nodeIndex);   //第一轮时不会触发
  Ptr<Node> node = allNodes.Get(nodeIndex);
  Ptr<Ipv4> ipv4_node = node->GetObject<Ipv4>();

  Ipv4StaticRoutingHelper ipv4StaticRouting;
  Ptr<Ipv4StaticRouting> NodeRouting = ipv4StaticRouting.GetStaticRouting(ipv4_node);

  Ipv4Address targetA_Addr = adhocAddr.GetAddress(targetA - 1);   //adhoc的容器内下标要减1
  Ipv4Address targetB_Addr = adhocAddr.GetAddress(targetB - 1);

  NodeRouting->AddHostRouteTo(Ipv4Address("192.168.1.1"),targetA_Addr,1,5); //1是adhoc的device
  NodeRouting->AddHostRouteTo(Ipv4Address("192.168.1.1"),targetB_Addr,1,10);

if(printRoutingTable)
{
  std::string filename = "try-route-";
  filename.append(std::to_string(nodeIndex));
  filename.append(".tr");
  Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(filename,std::ios::out);
  NodeRouting->PrintRoutingTable(stream);
}
  

  std::cout<<"# Set Routing on Node."<<nodeIndex<<" to Node."<<targetA<<" and Node."<<targetB<<std::endl;
}

//adhoc节点的路由写入，对应一条边的节点
void 
TryRouting::SetRoutingOnNode(uint32_t nodeIndex,uint32_t target)
{
  RemoveRoutingOnNode(nodeIndex);   //第一轮时不会触发
  Ptr<Node> node = allNodes.Get(nodeIndex);
  Ptr<Ipv4> ipv4_node = node->GetObject<Ipv4>();

  Ipv4StaticRoutingHelper ipv4StaticRouting;
  Ptr<Ipv4StaticRouting> NodeRouting = ipv4StaticRouting.GetStaticRouting(ipv4_node);

  Ipv4Address target_Addr = adhocAddr.GetAddress(target - 1);

  NodeRouting->AddHostRouteTo(Ipv4Address("192.168.1.1"),target_Addr,1);
if(printRoutingTable)
{
  std::string filename = "try-route-";
  filename.append(std::to_string(nodeIndex));
  filename.append(".tr");
  Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(filename,std::ios::out);
  NodeRouting->PrintRoutingTable(stream);

}
  
  std::cout<<"# Set Routing on Node."<<nodeIndex<<" to Node."<<target<<std::endl;

}

/**
 * 路由表中除了第一项是loopback，第二项会是本网段的地址，第三、四项则是到目的地的路由
 * 要删除的是到目标节点的地址
 * 单纯靠遍历-匹配-删除的方式不可行，因为必然有循环，而原代码里写了return
 */
//重构时执行
void
TryRouting::RemoveRoutingOnNode(uint32_t nodeIndex)
{
   Ptr<Node> node = allNodes.Get(nodeIndex);
  Ptr<Ipv4> ipv4_node = node->GetObject<Ipv4>();

  Ipv4StaticRoutingHelper ipv4StaticRouting;
  Ptr<Ipv4StaticRouting> NodeRouting = ipv4StaticRouting.GetStaticRouting(ipv4_node);

  if(NodeRouting->GetNRoutes()>3)   //写入了两条路由
    {
        NodeRouting->RemoveRoute(3);    //删除最后两条
        NodeRouting->RemoveRoute(2);
    }
  else if(NodeRouting->GetNRoutes()==3) //写入了一条路由
    {

        NodeRouting->RemoveRoute(2);    //删除最后一条
    }
}

void 
TryRouting::SetupApplication()
{
  Ptr<Node> Gateway = allNodes.Get(0);

  uint16_t port = 4000; //服务器端接收端口
  Time interPacketInterval = Seconds (11.0);    //发包时间间隔

  UdpServerHelper server (port);
  ApplicationContainer serverApp = server.Install (Gateway);
  serverApp.Start (Seconds (1.0));
  serverApp.Stop (Seconds (610.0));

  //Ptr<Node> node3 = allNodes.Get(3);

  //port = 9;   //客户端发送端口
  uint32_t MaxPacketSize = 1472;  // Back off 20 (IP) + 8 (UDP) bytes from MTU
  UdpClientHelper client (Ipv4Address("192.168.1.1"), port);
  client.SetAttribute ("PacketSize", UintegerValue (MaxPacketSize));
  client.SetAttribute ("Interval", TimeValue (interPacketInterval));
  ApplicationContainer clientApp = client.Install (remainNodesContainer);
  for(uint32_t i=0;i!=clientApp.GetN();i++) 
  {
    double time = 3.0 * (i+1);
    clientApp.Get(i)->SetStartTime(Seconds(time));  //想法是每个节点间隔3s开始发包，之后可以考虑改成随机变量
    clientApp.Get(i)->SetStopTime(Seconds(600.0));  //
  }


  flowMonitor = flowMoHelper.InstallAll();

  Simulator::Schedule(Seconds(300),&TryRouting::MonitorEvent,this);
  Simulator::Schedule(Seconds(100),&TryRouting::GetLifeTime,this);
    std::cout<<"Start Simulation"<<std::endl;
    Simulator::Stop(Seconds(620.0));
    Simulator::Run ();

    //routing.MonitorEvent();
    double delay = serverApp.Get(0)->GetObject<UdpServer>()->GetAvgDelay();
    double loss = serverApp.Get(0)->GetObject<UdpServer>()->GetLost();
   // std::cout<<"Delay "<<delay<<"s"<<std::endl;
   // std::cout<<"Lost "<<loss<<"packets"<<std::endl;
    
    //routing.flowMonitor->SerializeToXmlFile("tryrouting.xml",false,true);
    Simulator::Destroy ();
    std::cout<<"Simulation End"<<std::endl;

}

void
TryRouting::MonitorEvent()    //试试放在simulator：：run前
{
  uint32_t totalTxPacket = 0;   //不同flow的tx包的总和
  uint32_t totalRxPacket = 0;
  int64_t latencySum = 0 ;

  //double AvgThroughput;  //吞吐量需要决定好时间间隔，先搁置

  flowMonitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> flowClassifier = DynamicCast<Ipv4FlowClassifier> (flowMoHelper.GetClassifier ());
  FlowMonitor::FlowStatsContainer flowStats = flowMonitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator it = flowStats.begin (); it != flowStats.end (); ++it)
  {
    Ipv4FlowClassifier::FiveTuple t =flowClassifier->FindFlow (it->first);  //目的地址、目的端口、协议号、源地址、源端口

    totalTxPacket += it->second.txPackets;
    totalRxPacket +=it->second.rxPackets;
    latencySum += it->second.delaySum.GetMicroSeconds();    //微秒单位，不知道对不对

  }
  double packetDeliveredRatio = (totalRxPacket *1.0/ totalTxPacket)*100.0;   //%
  double AvgDelay = (latencySum*1.0/totalRxPacket);   //ms
  std::cout<<"No."<<++MonitorNum<<std::endl;
  std::cout<<"Total trans "<<totalTxPacket<<"packets"<<std::endl;
  std::cout<<"Total recv "<<totalRxPacket<<"packets"<<std::endl;
  std::cout<<"PDR is "<<packetDeliveredRatio<<"%"<<std::endl;
  std::cout<<"Average Delay is "<<AvgDelay<<"μs"<<std::endl;    //想办法把delay存起来
  
  Simulator::Schedule(Seconds(300),&TryRouting::MonitorEvent,this); //指示下次调用的时间，是想实现按周期的测量

}

double
TryRouting::GetLifeTime()
{
  double minTime = 1e15;
  uint32_t minIndex = 0;
  oldConsumption.resize(energyContainer.GetN());    //resize不改变原有的内容,大小应该是10
  for(uint32_t i = 0; i!=energyContainer.GetN();i++)
  {
    Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>(energyContainer.Get(i));

    Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
    
    double nowEnergy = basicSourcePtr->GetRemainingEnergy();
    double newConsumption = basicRadioModelPtr->GetTotalEnergyConsumption();

    double deltaValue = newConsumption - oldConsumption[i];

    double lifetime = nowEnergy / deltaValue;

    if(lifetime < minTime)
    {
      minTime = lifetime;
      minIndex = i;
    }

    oldConsumption[i] = newConsumption;


  }
  std::cout<<"+ Min TIME "<<minTime<<" on Node "<<(2*minIndex+3)<<std::endl;
  
  Simulator::Schedule(Seconds(100),&TryRouting::GetLifeTime,this);

  return minTime;
}


NS_LOG_COMPONENT_DEFINE ("TryRoutingExample");
int main(int argc, char *argv[])
{
   

    CommandLine cmd(__FILE__);
    cmd.Parse (argc,argv);

//下一步是配置发包的应用
    TryRouting routing;
    routing.InitializeRouting(30.0);


    //routing.PrintBaseGraph();
    //routing.PrintRemainNodeList();
    //routing.PrintUplinkNodeList();
    //
/*
    bool s =routing.FindNode2Up();
    routing.PrintBaseGraph();
    if(s)
    {
      
      routing.ChooseUpNode(0.3, 0.42, 0.28, 15);
      std::cout<<"Success"<<std::endl;
      routing.PrintRemainNodeList();
      routing.PrintUplinkNodeList();
    }
*/

   routing.Run_EpochWithQvalue(0, 15);

  

    routing.SetupApplication();
   //routing.Run_EpochWithQvalue(0, 15);
   


     
    return 0 ;
}