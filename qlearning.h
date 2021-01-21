/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef QLEARNING_H
#define QLEARNING_H

#include <bits/stdint-uintn.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include "stdlib.h"
namespace ns3 {

struct state //放在头文件中
    {
        int weight_H ;
        int weight_P ;
        int weight_S ;

        bool operator==(const state s)
        {
            if((weight_H==s.weight_H)&&(weight_P==s.weight_P)&&(weight_S==s.weight_S))
                return true;
            else
                return false;
        }
        bool operator >(const state s)
        {
            if(weight_H>s.weight_H && weight_P>s.weight_P && weight_S>s.weight_S)
                return true;
            else
                return false;
        }
        bool operator <(const state s)
        {
            if(weight_H<s.weight_H && weight_P<s.weight_P && weight_S<s.weight_S)
                return true;
            else
                return false;
        }
        bool operator <=(const state s)
        {
            if(weight_H<=s.weight_H || weight_P<=s.weight_P || weight_S<=s.weight_S)
                return true;
            else
                return false;
        }
        bool operator !=(const state s)
        {
            if(weight_H!=s.weight_H || weight_P!=s.weight_P || weight_S!=s.weight_S)
                return true;
            else
                return false;
        }
    };

//记得权值全都是整数，待用时处以100
class Qlearning
{
    public:
    uint32_t iteration = 0;         //轮数,只在Update中更新
    
    void SetNumberOfState(uint32_t num);
    void SetNumberOfAction(uint32_t num);
    void InitializeQtable();
    void InitializeStateVector();
    state GetCurrentState();
    
    state takeAction(int actionNo,state currentState);

    int GetStateIndex(state currState);

    void InitializePossibleActions();

    void Update(double reward,int actionIndex);

    int GetAvailableAction(int stateIndex);

    void Initialize();
    void Run(double reward);
      
    void PrintQtable();

    private:
    uint32_t m_stateNum = 15;
    uint32_t m_actionNum = 6;
    double alpha = 0.1;     //学习率
    double gamma = 0.8;     //长期回报率
    double epsilon = 0.3;   //epsilon贪心阈值

    std::vector< std::vector<double> > Q_table;
    

   
   
    struct state m_currentState={30,42,28};  //当前轮的状态,只在Update函数里修改

    //struct state m_nextState = {0,0,0};            //下一轮的状态


    enum action
    {
        A0 = 0,
        A1 = 1,
        A2 = 2,
        A3 = 3,
        A4 = 4,
        A5 = 5,
        A6 = 6
    };

    std::vector<state> state_vector;
    
  
};

}

#endif /* QLEARNING_H */

