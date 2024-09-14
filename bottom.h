// ---------------------------
// 这个文件存放的是函数的声明、全局变量的定义、外部变量的声明
// 关键字 extern 修饰的变量或函数，与之对应的是 Data.cpp
// 当你做出修改时，请注意可能需要同时修改 Data.cpp文件里的内容

#pragma once
#include "stdafx.h"
#include "platform.h"
using namespace Simuro;

#include <cmath>
#include <array>
#include <vector>
#include <string>


// 圆周率的表示法
// #define PI acos(-1)
// #define PI 2*asin(1)
// #define PI 4*atan(1)
// static const float PI = 3.14159f;

static const float PI = std::acos(-1.f);  //圆周率
static const float FTOP = 90.f;           //场地y轴上边界
static const float FBOT = -90.f;          //场地y轴下边界
static const float FLEFTX = -110.f;       //场地x轴左边界
static const float FRIGHTX = 110.f;       //场地x轴右边界
static const float GTOPY = 20.f;          //小禁区y轴上边界
static const float GBOTY = -20.f;         //小禁区y轴下边界
static const float ROBOTA = 7.5f;         //机器人宽度
static const float BALL = 4.3f;           //球的直径


/*****************************************************************/
/* --------------------↓extern 修饰的变量↓-------------------- */
// 此类变量意为：外部变量
// 主要目的是为了在多文件中共享数据
// 此类变量需要在 Data.cpp 中进行定义，此处仅为声明

extern std::vector<std::string> Debug_str;  //记录字符串，用于debug

extern int32_t Time;               //记录当前的周期
extern int32_t Note;

extern float ball_x;               //记录当前周期下 球的x坐标
extern float ball_y;               //记录当前周期下 球的y坐标
extern std::array<int, 5> Diss_s;  //记录距离球最近的 我方机器人下标
extern std::array<int, 5> Diss_o;  //记录距离球最近的 敌方机器人下标
extern std::array<std::array<float, 2>, 3> lastball;

extern int flag_goal;       //控制门球策略选择
extern int flag_penalty;    //控制罚球策略选择
extern int flag_penalty_2;  //控制罚球策略2里面的方案选择
extern int flag_penalty_3;  //控制罚球策略3里面的方案选择

/* --------------------↑extern 修饰的变量↑-------------------- */
/*****************************************************************/



/* ------------------------------------↑变量声明↑------------------------------------*/
/***************************************************************************************/
/* ------------------------------------↓函数声明↓------------------------------------*/

inline float Diss(const float x1, const float y1, const float x2, const float y2);
inline float DissRobot(const Robot *robot, const float x, const float y);
inline float Angle(const float x1, const float y1, const float x2, const float y2);
inline float AngleRobot(const Robot *robot, const float x, const float y);




void Go1(Robot *robot, const float x, const float y);
void Go3(Robot *robot, const float x, const float y, const float angle);
void Go4(Robot *robot, const float x, const float y);  //存在历史遗留问题，无法正常使用


void Go_Aviod(Field *field, Robot *robot, const int rid, const float x, const float y);
inline float Aviod_Angle(const Field *field, const int rid, const float x, const float y);
inline int Aviod_Component(const Field *field, const int rid, float angle, const float x, const float y);



void Turn_to(Robot *robot, float angle);
inline void Run(Robot *robot, const float left_speed, const float right_speed);


void GoalieJudgeMove(Field &field, Robot &robot);
const float Will_Pos(const Simuro::Field &field);
const float Will_Y();
const float BisectorBallWill();
const int CommentDistance();
float ballv();
const int CommentBallSpeed();
const int CommentBallAngle();
bool Ispositive();






void penalty_1(Robot *robot);
void penalty_2(Field *field, Robot *robot);
void penalty_3(Field *field, Robot *robot_A, const int A_rid, Robot *robot_B);

void penalty_defend(Robot *robot);




void Initialize(const Field *field);
void Debug(const Field *field);

/* ------------------------------------↑函数声明↑------------------------------------*/
/***************************************************************************************/
/* ------------------------------------↓数据保护↓------------------------------------*/
// 禁止隐式的类型转换，防止数据丢失

template <typename T1, typename T2, typename T3>
inline void Run(T3 *, T1, T2) = delete;

template <typename T1, typename T2, typename T3, typename T4>
inline float Diss(T1, T2, T3, T4) = delete;
template <typename T1, typename T2, typename T3>
inline float DissRobot(const T3 *, T1, T2) = delete;

template <typename T1, typename T2, typename T3, typename T4>
inline float Angle(T1, T2, T3, T4) = delete;
template <typename T1, typename T2, typename T3>
inline float AngleRobot(const T1 *, T2, T3) = delete;
