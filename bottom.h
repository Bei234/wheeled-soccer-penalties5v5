// ---------------------------
// ����ļ���ŵ��Ǻ�����������ȫ�ֱ����Ķ��塢�ⲿ����������
// �ؼ��� extern ���εı�����������֮��Ӧ���� Data.cpp
// ���������޸�ʱ����ע�������Ҫͬʱ�޸� Data.cpp�ļ��������

#pragma once
#include "stdafx.h"
#include "platform.h"
using namespace Simuro;

#include <cmath>
#include <array>
#include <vector>
#include <string>


// Բ���ʵı�ʾ��
// #define PI acos(-1)
// #define PI 2*asin(1)
// #define PI 4*atan(1)
// static const float PI = 3.14159f;

static const float PI = std::acos(-1.f);  //Բ����
static const float FTOP = 90.f;           //����y���ϱ߽�
static const float FBOT = -90.f;          //����y���±߽�
static const float FLEFTX = -110.f;       //����x����߽�
static const float FRIGHTX = 110.f;       //����x���ұ߽�
static const float GTOPY = 20.f;          //С����y���ϱ߽�
static const float GBOTY = -20.f;         //С����y���±߽�
static const float ROBOTA = 7.5f;         //�����˿��
static const float BALL = 4.3f;           //���ֱ��


/*****************************************************************/
/* --------------------��extern ���εı�����-------------------- */
// ���������Ϊ���ⲿ����
// ��ҪĿ����Ϊ���ڶ��ļ��й�������
// ���������Ҫ�� Data.cpp �н��ж��壬�˴���Ϊ����

extern std::vector<std::string> Debug_str;  //��¼�ַ���������debug

extern int32_t Time;               //��¼��ǰ������
extern int32_t Note;

extern float ball_x;               //��¼��ǰ������ ���x����
extern float ball_y;               //��¼��ǰ������ ���y����
extern std::array<int, 5> Diss_s;  //��¼����������� �ҷ��������±�
extern std::array<int, 5> Diss_o;  //��¼����������� �з��������±�
extern std::array<std::array<float, 2>, 3> lastball;

extern int flag_goal;       //�����������ѡ��
extern int flag_penalty;    //���Ʒ������ѡ��
extern int flag_penalty_2;  //���Ʒ������2����ķ���ѡ��
extern int flag_penalty_3;  //���Ʒ������3����ķ���ѡ��

/* --------------------��extern ���εı�����-------------------- */
/*****************************************************************/



/* ------------------------------------������������------------------------------------*/
/***************************************************************************************/
/* ------------------------------------������������------------------------------------*/

inline float Diss(const float x1, const float y1, const float x2, const float y2);
inline float DissRobot(const Robot *robot, const float x, const float y);
inline float Angle(const float x1, const float y1, const float x2, const float y2);
inline float AngleRobot(const Robot *robot, const float x, const float y);




void Go1(Robot *robot, const float x, const float y);
void Go3(Robot *robot, const float x, const float y, const float angle);
void Go4(Robot *robot, const float x, const float y);  //������ʷ�������⣬�޷�����ʹ��


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

/* ------------------------------------������������------------------------------------*/
/***************************************************************************************/
/* ------------------------------------�����ݱ�����------------------------------------*/
// ��ֹ��ʽ������ת������ֹ���ݶ�ʧ

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
