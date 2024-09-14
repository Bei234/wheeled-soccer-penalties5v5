// ---------------------------
// 这个文件存放的是在 bottom.h 文件中被关键字 extern 修饰的变量或函数
// 它们在此处做具体的定义或实现
// 此文件向 bottom.h 文件的内容看齐，在 bottom.h 中有相关的改动之后，再来此处做修订

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

