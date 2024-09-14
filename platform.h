#pragma once
#include <cstdint>

namespace Simuro
{
    static const int PLAYERS_PER_SIDE = 5;
    static const int MAX_STRING_LEN = 128;

   enum EventType {
        JudgeResult = 0,
        MatchStart = 1,
        MatchStop = 2,
        FirstHalfStart = 3,
        SecondHalfStart = 4,
        OvertimeStart = 5,
        PenaltyShootoutStart = 6,
        MatchShootOutStart = 7,
        MatchBlockStart = 8,
    };


    enum JudgeType {
        PlaceKick = 0,         //开球
        GoalKick = 1,          //门球
        PenaltyKick = 2,       //罚球
        FreeKickRightTop = 3,  //争球 右上角
        FreeKickRightBot = 4,  //争球 右下角
        FreeKickLeftTop = 5,   //争球 左上角
        FreeKickLeftBot = 6,   //争球 左下角
    };

    enum Team {
        Self,      //我方执行
        Opponent,  //对方执行
        None,
    };

    struct Vector2 {
        float x;  // x轴的坐标值
        float y;  // y轴的坐标值
    };

    struct TeamInfo {
        wchar_t teamName[MAX_STRING_LEN];  //队名
    };

    struct Ball {
        Vector2 position;
    };

    struct Wheel {
        float leftSpeed;   //左轮速
        float rightSpeed;  //右轮速
    };

    struct Robot {
        Vector2 position;
        float rotation;  //角度(机器人的朝向)
        Wheel wheel;
    };

    struct Field {
        Robot selfRobots[PLAYERS_PER_SIDE];      //我方机器人的标号 { 0, 1, 2, 3, 4 }
        Robot opponentRobots[PLAYERS_PER_SIDE];  //敌方机器人的标号 { 0, 1, 2, 3, 4 }
        Ball ball;
        int32_t tick;  //拍数
    };

    struct JudgeResultEvent {
        JudgeType type;
        Team actor;
        wchar_t reason[MAX_STRING_LEN];  //文本消息的字段长度
    };
}  // namespace Simuro
