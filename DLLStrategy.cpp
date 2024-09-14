// DLLStrategy.cpp : 定义 DLL 应用程序的导出函数。

#include "stdafx.h"
#include "adapter.h"
#include "bottom.h"
#include "platform.h"

#include<iostream>
#include "xstring"
#include<string>
#include<typeinfo>
#include<sstream>
#include<locale>
#include<vector>

using namespace Simuro;
using namespace Adapter;


int race_state = -1;          //处于何种定位球状态，0是开球，其他遵从JudgeType
int race_state_trigger = -1;  //哪一方触发了定位球

JudgeType result;  //事件
Team actor;        //执行者


//事件选择
void OnEvent(EventType type, void *argument)
{
    SendLog(L"V/Strategy:OnEvent()");
    if (type == EventType::MatchShootOutStart) {
        SendLog(L"Penalty Shootout Start");
    }
    else if (type == EventType::JudgeResult) {
        JudgeResultEvent *judgeResult = static_cast<JudgeResultEvent *>(argument);
        race_state = judgeResult->type;
        race_state_trigger = judgeResult->actor;
        if (judgeResult->type == JudgeType::PenaltyKick) {
            SendLog(L"Penalty Kick");
        }
        switch (judgeResult->actor) {
            case Team::Self:
                SendLog(L"By self");
                break;
            case Team::Opponent:
                SendLog(L"By opp");
                break;
            case Team::None:
                SendLog(L"By both");
                break;
        }
    }

}

/**
* 获得队伍信息
*/
void GetTeamInfo(TeamInfo *teamInfo)
{
    SendLog(L"V/Strategy:GetTeamInfo()");
    static const wchar_t teamName[] = L"点球大战";
    static constexpr size_t len = sizeof(teamName);
    memcpy(teamInfo->teamName, teamName, len);
}

/**
* 摆位信息，进行定位球摆位
*/
void GetPlacement(Field *field)
{
    SendLog(L"V/Strategy:GetPlacement()");
    auto robots = field->selfRobots;

    if (race_state == JudgeType::PenaltyKick)  //点球
    {
        if (race_state_trigger == Team::Self)  //点球进攻
        {
            robots[3].position.x = 3;  
            robots[3].position.y = 10;
            robots[3].rotation = 0;

            robots[2].position.x = 3;  
            robots[2].position.y = -10;
            robots[2].rotation = 0;

            robots[0].position.x = -5;//110 + 7
            robots[0].position.y = 0;
            robots[0].rotation = 90;

            //
            robots[1].position.x = -45;  //点球机器人
            robots[1].position.y = 0;
            robots[1].rotation = -180;

            robots[4].position.x = 3;  
            robots[4].position.y = 0;
            robots[4].rotation = 0;
        }
        else if (race_state_trigger == Team::Opponent)  //点球防守
        {
            robots[0].position.x =  106;
            robots[0].position.y = 0;
            robots[0].rotation = 180;

            robots[1].position.x = -6;
            robots[1].position.y = 85;
            robots[1].rotation = -90;

            robots[2].position.x = -10;
            robots[2].position.y = -20;
            robots[2].rotation = -90;

            robots[3].position.x = -10;
            robots[3].position.y = 40;
            robots[3].rotation = -90;

            robots[4].position.x = -10;
            robots[4].position.y = -40;
            robots[4].rotation = -90;
        }
        else  // None人触发
        {
        }
    }
}



/**
*   策略主函数运行
*/
void GetInstruction(Field *field)
{
    if (race_state_trigger == Team::Self) {
        penalty_1(&field->selfRobots[1]);
    }
    else if (race_state_trigger == Team::Opponent) {
        penalty_defend(&field->selfRobots[0]);
    }
    else  // None人触发
    {

    }
}
