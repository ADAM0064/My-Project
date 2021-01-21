/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "qlearning.h"
#include "ns3/integer.h"

namespace ns3 {




     /**
     * 问题1：state的转移可能因为double类型在加减时存在的数据精度误差原因不能和设定的权值相匹配，结果带来失败；
     * 解决方案：所有权值以及转移都用整数，在实际使用前再除以100；
     * 
     * 
     */

void
Qlearning::SetNumberOfState(uint32_t num)
{
    m_stateNum = num;
}

void
Qlearning::SetNumberOfAction(uint32_t num)
{
    m_actionNum = num;
}

//初始化Q表
void
Qlearning::InitializeQtable()
{
    Q_table.resize(m_stateNum);
    for(int i=0;i<Q_table.size();i++)
    {
        Q_table[i].resize(m_actionNum);
    }

}

//在当前状态下采取动作
state
Qlearning::takeAction(int actionNo,state currentState)
{
   struct state tempstate = currentState;
   
        switch(actionNo)
      {
        case A0 :
            tempstate.weight_H -=  14;
            tempstate.weight_P +=  14;
            break;
        case A1 :
            tempstate.weight_H +=  14;
            tempstate.weight_P -=  14;
            break;
        case A2 :
            tempstate.weight_P -=  14;
            tempstate.weight_S +=  14;
            break;
        case A3 :
            tempstate.weight_P +=  14;
            tempstate.weight_S -=  14;
            break;
        case A4 :
            tempstate.weight_H -=  14;
            tempstate.weight_S +=  14;
            break;
        case A5 :
            tempstate.weight_H +=  14;
            tempstate.weight_S -=  14;
            break;
      }

      return tempstate;
      /*
     if(tempstate.weight_P>0 && tempstate.weight_S>0 && tempstate.weight_H>0)
     {
         m_nextState = tempstate;
     }
     else
     {
        std::cout<<"action invalid"<<std::endl; //action invalid
     }
    */
}

//初始化状态向量
void 
Qlearning::InitializeStateVector()
{
   state_vector.resize(15);
   state_vector[0] = { 72, 14, 14};
   state_vector[1] = { 58, 28, 14};
   state_vector[2] = { 44, 42, 14};

   state_vector[3] = { 30, 56, 14};
   state_vector[4] = { 16, 70, 14};
   state_vector[5] = { 58, 14, 28};

   state_vector[6] = { 44, 28, 28};
   state_vector[7] = { 30, 42, 28};
   state_vector[8] = { 16, 56, 28};

   state_vector[9] = { 44, 14, 42};
   state_vector[10] ={ 30, 28, 42};
   state_vector[11] ={ 16, 42, 42};

   state_vector[12] ={ 30, 14, 56};
   state_vector[13] ={ 16, 28, 56};
   state_vector[14] ={ 16, 14, 70};

}

//从状态向量中寻找对应状态的对应下标，以便在Q表中搜索;
//注意：如果没找到就返回-1！
int
Qlearning::GetStateIndex(state currState)
{
    int index =-1;
    for(uint16_t i=0;i!=state_vector.size();i++)
    {
        if(state_vector[i] == currState)
        {
            index = i;
            break;
        }

    }
    return index;
}

//通过遍历各个状态下采取各个动作权值是否小于0来判断动作是否可用，如果可用则标对应Q值为0,否则则标-1；
//在初始化Q表和状态向量之后进行
void
Qlearning::InitializePossibleActions()
{
    state zeroState = {2,2,2};
    for(int stateIndex = 0;stateIndex!=m_stateNum;stateIndex++)     //所有的状态
    {
        for(int actionIndex =0;actionIndex!=m_actionNum;actionIndex++)  //所有的动作
        {
            state newState = takeAction(actionIndex,state_vector[stateIndex]);  //某一状态下采取动作，改变权值
            if(newState <= zeroState) 
            {
                Q_table[stateIndex][actionIndex] = -1;   //不可以使用的动作,在进行epsilon贪心时需要避免使用
            }
            else
            {
                Q_table[stateIndex][actionIndex] = 0;  //可以使用的动作
            }

        }
    }

}

//更新Q值的函数，需要给出回报以及待使用的动作下标;
//只有在更新时会修改当前状态m_currentState
void
Qlearning::Update(double reward,int actionIndex)
{
 int state_now = GetStateIndex(m_currentState);             //s(t) 当前轮的状态
 struct state nextState = takeAction(actionIndex,m_currentState);       //s(t+1) 下一轮的状态
 int state_next = GetStateIndex(nextState);
 std::cout<<"take action"<<actionIndex<<std::endl;
 std::cout<<"next state"<<state_next<<std::endl;
 std::cout<<nextState.weight_H<<nextState.weight_P<<nextState.weight_S<<std::endl;
 if(state_next>=0)
 {
     double next_max_value = *std::max_element(Q_table[state_next].begin(),Q_table[state_next].end());  //下一状态行的最大值,相同值也可以找到最大
     Q_table[state_now][actionIndex] = (1-alpha)*Q_table[state_now][actionIndex] + alpha * (reward + gamma* next_max_value );
     iteration++;                   //更新轮数
     m_currentState = nextState;    //更新状态

 }
}


//寻找可用的动作，在Q表某行遍历一遍，将所有不小于0的Q值的列下标存起来，然后根据epsilon贪心进行动作选择
//需要给出当前状态的下标
//epsilon贪心还未经验证
int
Qlearning::GetAvailableAction(int stateIndex)
{   
    srand((unsigned)time(NULL));            
    std::vector<int> availableAction;       //可用的状态
    for(int actionIndex=0;actionIndex!=m_actionNum;actionIndex++)
    {
        double Qvalue = Q_table[stateIndex][actionIndex];       
        if(Qvalue>=0)
        {
            availableAction.push_back(actionIndex);     //可用动作的下标
        }
    }
    int size= availableAction.size(); 
    std::cout<<"size:"<<size<<std::endl;
    double p = (rand()%101);  //取[0,100]之间的整数
    if(p<epsilon*100)        //对应epsilon，explore，在可用的动作中随机选择                
    {
        srand((unsigned)time(NULL));
        int index = rand()%(size);    //取得[0,size)内的随机整数，是vector的下标
        int actionIndex = availableAction[index];
        return actionIndex;
    }
    else  //对应1-epsilon，exploit，选择当前状态下Q值最大的动作，即当前行内最大值的列下标
    {
        std::vector<double>::iterator biggestQ = std::max_element(Q_table[stateIndex].begin(),Q_table[stateIndex].end());//这一行内最大的Q值
        int actionIndex = std::distance(Q_table[stateIndex].begin(),biggestQ);                                                 //这一行内最大Q值的对应位置
        return actionIndex;
    }
}


//初始化函数
void 
Qlearning::Initialize()
{
    InitializeQtable();         //初始化Q表    
    InitializeStateVector();    //状态向量
    InitializePossibleActions();//可用的动作
}

state
Qlearning::GetCurrentState()
{
    return m_currentState;
}

//运行函数，务必要先初始化;
//运行一轮后再取m_currentState即可
void
Qlearning::Run(double reward)
{
    state nowState = GetCurrentState();     //当前状态
    int nowStateIndex = GetStateIndex(nowState);        //状态下标
    std::cout<<"now State is:"<<nowStateIndex<<std::endl;
    if(nowStateIndex>=0)
    {
        int nowActionIndex = GetAvailableAction(nowStateIndex);     //动作下标
        Update(reward,nowActionIndex);
        //std::cout<<"S&A:"<<nowStateIndex<<","<<nowActionIndex<<std::endl;
    }
    
    
  
}   

void
Qlearning::PrintQtable()
{
    for(int stateIndex = 0;stateIndex!=m_stateNum;stateIndex++)
    {
        for(int actionIndex = 0;actionIndex!=m_actionNum;actionIndex++)
        {
            std::cout<<" "<<Q_table[stateIndex][actionIndex];
        }
            std::cout<<"\n";
    }
    std::cout<<std::endl;
}

}//namespace ns3

