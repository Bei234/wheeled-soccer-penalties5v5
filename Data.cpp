// ---------------------------
// ����ļ���ŵ����� bottom.h �ļ��б��ؼ��� extern ���εı�������
// �����ڴ˴�������Ķ����ʵ��
// ���ļ��� bottom.h �ļ������ݿ��룬�� bottom.h ������صĸĶ�֮�������˴����޶�

#include "stdafx.h"
#include "bottom.h"

std::vector<std::string> Debug_str;

int32_t Time = 0;
int32_t Note{};

float ball_x = 0.f;
float ball_y = 0.f;
std::array<int, 5> Diss_s{ 0, 1, 2, 3, 4 };
std::array<int, 5> Diss_o{ 0, 1, 2, 3, 4 };
std::array<std::array<float, 2>, 3> lastball{};

int flag_goal = 0;
int flag_penalty = 0;
int flag_penalty_2 = 0;
int flag_penalty_3 = 0;

