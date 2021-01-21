#include <iostream>
#include <algorithm>
#include <utility>
#include "ns3/basic-energy-source.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/node-container.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/qlearning.h"
#include "ns3/string.h"
#include "ns3/command-line.h"


using namespace ns3;


/**
 * 需要完成的事情：
 * 1、寻路代码的验证和完善
 * 2、路由转发的端口转发
 * 3、reward的获取（latency和lifetime的测量）
 * 4、PRN、吞吐量等信息的获取和统计
 * 
 * 可能存在的问题：
 * 1、列表之间、列表内部内容的重复
 * 2、clear并不释放内存，担心程序中的几个列表会爆内存;（Map也需要这样释放吗？）
 * 3、时间上的复杂度
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

class TryRouting
{
  public:

    double GetNodeDistance(Ptr<Node> A,Ptr<Node> B); 
    //初始化拓扑图
    void InitializeGraph(NodeContainer container);
    //向拓扑图中添加信息
    void UpdateGraph(NodeContainer nodeContainer);

    //初始化信息
    void InitializeUpList(NodeContainer nodeContainer);   
    //寻找和上行节点有多条边连接的候选节点
    bool FindNode2Up();     

    void ChooseUpNode(double wh,double wp,double ws,double dist);

    bool FindNode1UP();

    void Choose1WayUP();

    void Initialize(NodeContainer remain,NodeContainer all);

    void Run_epoch(double wh,double wp,double ws,double dist);

    void PrintBaseGraph();

    void PrintRemainNodeList();

    void PrintUplinkNodeList();


  private:
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
TryRouting::UpdateGraph(NodeContainer nodeContainer)
{
  double distance = 0;
  for(uint32_t i=0;i!=nodeContainer.GetN();i++)   //A
  {
    for(uint32_t j=i+1;j!=nodeContainer.GetN();j++)   //B
    {
      distance = GetNodeDistance(nodeContainer.Get(i),nodeContainer.Get(j));

      if(distance < 30 && distance > 0)
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

/**
 * 下面的代码还都没经过验证
 * 主要想实现的功能是初始化各列表、查找候选节点、寻找候选节点的下一跳节点（缓存）
 * 
 */

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
          std::cout<<"Candidate Node index:"<<nodeIndex<<" ,power:"<<power<<std::endl;
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
        it->cost =  wh * (double)(it->hop/max_hop) + wp * power + ws * (std::min(it->distance - dist, 0.0)/dist);   //计算下一跳节点的cost
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
    it->cost = it->cost = wh * (double)(it->hop/max_hop) + wp * power + ws * (std::min(it->distance - dist, 0.0)/dist);   //计算候选节点的cost
  }

  std::sort(candidate2UP.begin(),candidate2UP.end(),CompareCost);
  std::vector<SuccessorInfo>::iterator best_node = candidate2UP.begin();      //选出候选节点
  std::cout<<"Best Candidate Node index:"<<best_node->index<<" ,hop:"<<best_node->hop<<" , distance:"<<best_node->distance<<std::endl;
  std::map<uint32_t, std::vector<SuccessorInfo>>::iterator s_it = successorMap.find(best_node->index);
  std::cout<<"Next Hop index A:"<<s_it->second[0].index<<" ,index B:"<<s_it->second[1].index<<std::endl;

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
  std::map<uint32_t,std::vector<SuccessorInfo>>::iterator succ_it = successorMap.find(best_node->index); //2个下一跳节点,应该进行路由表的写入处理

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
  successorMap.insert(std::make_pair(index, temp));     //不存也没关系，重要的是把下一跳信息写进路由表

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
TryRouting::Initialize(NodeContainer remain,NodeContainer all)
{
  InitializeGraph(all);
  UpdateGraph(all);
  InitializeUpList(remain);
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



int main(int argc, char *argv[])
{
    std::string phyMode ("DsssRate11Mbps");
    uint16_t nums = 20;
    uint16_t nPackets = 1;
    uint16_t PacketSize = 1000;

    CommandLine cmd(__FILE__);
    cmd.Parse (argc,argv);

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

    NodeContainer otherNodes;
    otherNodes.Create(20);

    MobilityHelper anotherMobility;
    anotherMobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                          "X",StringValue("0.0"),
                                          "Y",StringValue("0.0"),
                                          "Rho",StringValue("ns3::UniformRandomVariable[Min=10|Max=100]"));   //位置可能需要改
    anotherMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    anotherMobility.Install(otherNodes);

    NodeContainer adhocNodes = NodeContainer(AP1,AP2,otherNodes);
    NodeContainer csmaNodes = NodeContainer(basicNodes);


    TryRouting routing;
    routing.Initialize(otherNodes,adhocNodes);
    //routing.PrintRemainNodeList();
    //routing.PrintUplinkNodeList();
    //

    bool s =routing.FindNode2Up();
    //routing.PrintBaseGraph();
    if(s)
    {
      
      routing.ChooseUpNode(0.3, 0.42, 0.28, 15);
      std::cout<<"Success"<<std::endl;
      routing.PrintRemainNodeList();
      routing.PrintUplinkNodeList();
    }

    return 0 ;
}