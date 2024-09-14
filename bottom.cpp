// ---------------------------
// ����ļ���ŵ��Ǻ����Ķ��壨����ʵ�֣�
// ��֮��Ӧ���� bottom.h�����������޸�ʱ����ע�������Ҫͬʱ�޸�����ͷ�ļ�

#include "stdafx.h"
#include "bottom.h"

#include <iostream>
#include <fstream>
#include <queue>

static constexpr float PRIRY = 105.6F;  //����Ա�ĳ�ʼλ��
/***************************************************************************************/
/* ------------------------------------���������ܡ�------------------------------------*/

struct Ascending {  //�Զ�����������
    template <typename T1, typename T2>
    bool operator()(const std::pair<T1, T2> &left, const std::pair<T1, T2> &right) const
    {
        return left.second > right.second;
    }
};
struct Descending {  //�Զ������򣬽���
    template <typename T1, typename T2>
    bool operator()(const std::pair<T1, T2> &left, const std::pair<T1, T2> &right) const
    {
        return left.second < right.second;
    }
};

// ��ʼ������
void Initialize(const Field *field)
{
    Debug_str.emplace_back("���� Initialize ������");

    // ���µ�ǰ������
    Debug_str.emplace_back("  ����ǰ�Ľ����� Time = " + std::to_string(Time));
    Time = field->tick;
    Debug_str.emplace_back("  ���º�Ľ����� Time = " + std::to_string(Time));

    // ���¿���״̬
    flag_goal = Time % 2000 / 1000;     // 0 1
    flag_penalty = Time % 3000 / 1000;  // 0 1 2

    // ���µ�ǰ�������
    ball_x = field->ball.position.x;
    ball_y = field->ball.position.y;

    std::priority_queue<std::pair<size_t, float>, std::vector<std::pair<size_t, float>>, Ascending> Sub_Dis_A;
    // ���ݻ����˺���ľ��룬���ҷ������˵��±������������
    for (size_t i = 0; i < 5; ++i) {
        Sub_Dis_A.emplace(i, DissRobot(&field->selfRobots[i], ball_x, ball_y));
    }
    for (auto &n : Diss_s) {
        n = Sub_Dis_A.top().first;
        Sub_Dis_A.pop();
    }
    // ���ݻ����˺���ľ��룬�Եз������˵��±������������
    for (size_t j = 0; j < 5; ++j) {
        Sub_Dis_A.emplace(j, DissRobot(&field->opponentRobots[j], ball_x, ball_y));
    }
    for (auto &n : Diss_o) {
        n = Sub_Dis_A.top().first;
        Sub_Dis_A.pop();
    }

    Debug_str.emplace_back("�˳� Initialize ���� ��\n");
}

// �����������ڵ��ļ����£������ⲿ�ļ���¼���ϵ�����
void Debug(const Field *field)
{
    std::ofstream ofs;
    if (Time <= 3) {
        ofs.open("./Debut_data.txt", std::ios::out);  //�Ը�д��ģʽ���ļ�
    }
    else {
        ofs.open("./Debut_data.txt", std::ios::app);  //����д��ģʽ���ļ�
    }

    if (ofs) {  //�ļ���(����)�ɹ�
        ofs.setf(std::ios::fixed);
        ofs.setf(std::ios::showpoint);
        ofs.precision(7);  //���ø���������ЧλΪ7λ
        int count = 0;

        ofs << "\n------�� " << Time << " ���ڿ�ʼ-----" << std::endl;

        ofs << "������꣺(" << ball_x << ", " << ball_y << ")\n"
            << std::endl;

        // �����ҷ��������˵�״̬
        for (const auto &n : field->selfRobots) {
            ofs << count << " �Ż����˵�״̬��" << std::endl;
            ofs << "   ���꣺(" << n.position.x << ", " << n.position.y << ")" << std::endl;
            ofs << "   �Ƕȣ�" << n.rotation << std::endl;
            ofs << "   ������= " << n.wheel.leftSpeed << "��������= " << n.wheel.rightSpeed << std::endl
                << std::endl;
            count++;
        }
        count = 0;


        // �����з��������˵�״̬
        /*for (const auto &n : field->opponentRobots) {
            ofs << "%d �Ż����˵�״̬��" << count << std::endl;
            ofs << count << " �Ż����˵�״̬��" << std::endl;
            ofs << "   ���꣺(" << n.position.x << ", " << n.position.y << ")" << std::endl;
            ofs << "   �Ƕȣ�" << n.rotation << std::endl;
            ofs << "   ������= " << n.wheel.leftSpeed << "��������= " << n.wheel.rightSpeed << std::endl
                << std::endl;
            count++;
        }
        count = 0;*/

        ofs << "------�����ڽ���-----\n"
            << std::endl;
        ofs.close();  //�ر��ļ�
    }


    if (Time <= 3) {
        ofs.open("./Debut_str.txt", std::ios::out);
    }
    else {
        ofs.open("./Debut_str.txt", std::ios::app);
    }

    if (ofs) {
        ofs << "\n------�� " << Time << " ���ڿ�ʼ-----" << std::endl;

        for (const auto &str : Debug_str) {
            ofs << str << std::endl;
        }
        Debug_str.clear();  //���vector����

        ofs << "------�����ڽ���-----\n"
            << std::endl;
        ofs.close();
    }
}


/* ------------------------------------���������ܡ�------------------------------------*/
/***************************************************************************************/




/***************************************************************************************/
/* ------------------------------------����ѧ�����------------------------------------*/

// �� ������ ֮���ֱ�߾���
inline float Diss(const float x1, const float y1, const float x2, const float y2)
{
    return sqrt(((x1 - x2) * (x1 - x2)) +  //
                ((y1 - y2) * (y1 - y2)));
}
inline float DissRobot(const Robot *robot, const float x, const float y)
{
    return Diss(robot->position.x, robot->position.y, x, y);
}

// �� ������ ֮��������н�
inline float Angle(const float x1, const float y1, const float x2, const float y2)
{
    float at = 0.f, l = Diss(x1, y1, x2, y2);

    if (l == 0) l = 0.0001f;  //�����Լ���ʱ����ܻ�����������

    if ((y2 - y1) / l > 1.f)
        at = 90.f;
    else if ((y2 - y1) / l < -1.f)
        at = -90.f;
    else
        at = asin((y2 - y1) / l) * 180.f / PI;

    if (y2 > y1 && x2 < x1) at = 180.f - at;
    if (y2 < y1 && x2 > x1) at = 360.f + at;
    if (y2 < y1 && x2 < x1) at = 180.f - at;

    return at;
}
inline float AngleRobot(const Robot *robot, const float x, const float y)
{
    return Angle(robot->position.x, robot->position.y, x, y);
}

/* ------------------------------------����ѧ�����------------------------------------*/
/***************************************************************************************/




/***************************************************************************************/
/* -----------------------------------���˶��������-----------------------------------*/

// ʹһ���������ܵ���x,y��Ŀ��λ����
void Go1(Robot *robot, const float x, const float y)
{
    float angle_robot = robot->rotation;
    if (angle_robot < 0.f) angle_robot = angle_robot + 360.f;

    float angle_need = angle_robot - AngleRobot(robot, x, y);
    if (angle_need > 180.f) { angle_need -= 360.f; }
    else if (angle_need < -180.f) {
        angle_need += 360.f;
    }

    bool foward = true;
    float aCircle = 0.f;
    if (fabs(angle_need) < 90.f) {
        foward = false;
        aCircle = fabs(angle_need);
    }
    else {
        aCircle = 180.f - fabs(angle_need);
    }

    float vc = 0.f;
    if (aCircle > 80.f) { vc = 75.f; }
    else if (aCircle > 40.f) {
        vc = 60.f;
    }
    else if (aCircle > 10.f) {
        vc = 30.f;
    }
    else if (aCircle > 5.f) {
        vc = 10.f;
    }
    else if (aCircle > 2.f) {
        vc = 2.f;
    }
    else {
        vc = 0.f;
    }

    float l = DissRobot(robot, x, y);
    float vl = 0.f, vr = 0.f, vv = 0.f;
    if (l < 3.5f) {
        vv = 0.f;
        vc = 0.f;
    }
    else if (l < 8.f) {
        vv = 15.f;
    }
    else if (l > 30.f) {
        vv = 112.f;
    }
    else {
        vv = 80.f;
    }

    if (foward) {
        vv = -vv;
        vc = -vc;
    }

    if (angle_need >= 0.f) {
        vl = vv;
        vr = vv - vc;
    }
    else {
        vl = vv - vc;
        vr = vv;
    }
    Run(robot, vl, vr);
}
void Go3(Robot *robot, const float x, const float y, const float angle)
{
    float angle_robot = robot->rotation, l = DissRobot(robot, x, y);
    float angle_robot_t = AngleRobot(robot, x, y), angle_robot_f = 0.f;

    if (angle_robot < 0.f) { angle_robot += 360.f; }
    if (angle_robot < 180.f) { angle_robot_f = angle_robot + 180.f; }
    else
        angle_robot_f = angle_robot - 180.f;

    if (fabs(angle_robot_t - angle) > 40.f || l > 20.f) {
        float rx = robot->position.x, ry = robot->position.y;
        float px = (x - rx) / 2.f, py = (y - ry) / 2.f;
        float angle = 0.f;

        if (x - rx > 0.f) { angle = atan((y - ry) / (x - rx)) * 180.f / PI; }
        else {
            angle = 180.f + atan((y - ry) / (x - rx)) * 180.f / PI;
        }
        float angle2 = angle - 90.f;
        float qx = l * cos(angle2 / 180.f * PI);
        float qy = l * sin(angle2 / 180.f * PI);
        Go1(robot, x + px + qx, y + py + qy);
    }
    else {
        if (angle_robot_t > 180.f) { angle_robot_t -= 180.f; }
        if (fabs(angle_robot - angle_robot_t) < 90.f) { Turn_to(robot, angle_robot_t); }
        else
            Turn_to(robot, angle_robot_t + 180.f);
        if (fabs(angle_robot_t - angle_robot) < 20.f || fabs(angle_robot_t - angle_robot_f) < 20.f) { Go1(robot, x, y); }
    }
}
void Go4(Robot *robot, const float x, const float y)
{
    float angle_robot = robot->rotation;
    if (angle_robot < 0.f) { angle_robot += 360.f; }
    float angle_need = angle_robot - AngleRobot(robot, x, y);
    if (angle_need > 180.f) { angle_need -= 360.f; }
    else if (angle_need < -180.f) {
        angle_need += 360.f;
    }

    bool foward = true;
    if (fabs(angle_need) < 90.f) { foward = false; }

    float aCircle = 0.f;
    if (foward) { aCircle = fabs(angle_need); }
    else {
        aCircle = 180.f - fabs(angle_need);
    }

    float length = DissRobot(robot, x, y) - ROBOTA - BALL;
    float v = 0.f, vv = 0.f;
    // v = Predictrobot(field, 'v', rid); //ΪԤ��Ļ������ٶ�
    if (length - v - 0.116737f > 0.f) { vv = 125.f; }
    else
        vv = -(0.116737f * v / (length - v - 0.116737f)) * 125.f / 1.912f;

    float l = DissRobot(robot, x, y);
    float vl = 0.f, vr = 0.f, vc = 0.f;
    vc = ((l / (2.f * cos(90.f - aCircle) / 180.f * PI) - ROBOTA) / /**/
          (l / (2.f * cos(90.f - aCircle) / 180.f * PI) + ROBOTA))
         * vv;

    if (foward) {
        v = -vv;
        vc = -vc;
    }

    if (angle_need >= 0.f) {
        vl = v;
        vr = vc;
    }
    else {
        vl = vc;
        vr = v;
    }

    Run(robot, vl, vr);
}

// �������׺��� --ֱ�ӵ��� Go_Aviod (���Ż�)
void Go_Aviod(Field *field, Robot *robot, const int rid, const float x, const float y)
{
    Go1(robot, x, robot->position.y +                                  //
                      tan(Aviod_Angle(field, rid, x, y) / 180.f * PI)  //
                          * (x - robot->position.x));
}
inline float Aviod_Angle(const Field *field, const int rid, const float x, const float y)
{
    // Ϊ���Ϻ������غ��ʵĽǶ�
    float angle = atan((y - field->selfRobots[rid].position.y) / (x - field->selfRobots[rid].position.x)) * 180.f / PI;
    float Aviod_Angle = angle;
    for (int j = 0; (0.5f * static_cast<float>(j)) < 180.f; ++j) {
        if (angle + 0.5f * static_cast<float>(j) < 90.f) {
            if (Aviod_Component(field, rid, (angle + 0.5f * static_cast<float>(j) / 180.f * PI), x, y) == 9) {
                Aviod_Angle = angle + 0.5f * static_cast<float>(j);
                break;
            }
        }
        if (angle - 0.5f * static_cast<float>(j) > -90.f) {
            if (Aviod_Component(field, rid, (angle - 0.5f * static_cast<float>(j) / 180.f * PI), x, y) == 9) {
                Aviod_Angle = angle - 0.5f * static_cast<float>(j);
                break;
            }
        }
    }
    return Aviod_Angle;
}
inline int Aviod_Component(const Field *field, const int rid, float angle, const float x, const float y)
{
    std::array<float, 11> X, Y;           // [0]�����ˣ�[10]����Ŀ�꣬����[i]Ϊ�ϰ������꣨xi,yi��
    float L = 2.95275f;                   //���岻��
    float d = sqrt(L * L * 2.f) * 2.54f;  //���岻��,�²���Ӣ�������֮��ĵ�λת��
    float judge = 0.f;                    //���岻��

    for (size_t j = 0; j < 11; ++j) {
        if (j < 5) {
            X[j] = field->selfRobots[j].position.x;
            Y[j] = field->selfRobots[j].position.y;
        }
        else if (j < 10) {
            X[j] = field->opponentRobots[j - 5].position.x;
            Y[j] = field->opponentRobots[j - 5].position.y;
        }
        else if (j == 10) {
            X[10] = x;
            Y[10] = y;
        }
    }
    if (rid != 0) {  //������ϻ����˲���һ�Ż����ˣ���һ�Ż����˵�λ�ú͸û����˻���
        X[rid] = X[0];
        Y[rid] = Y[0];
        X[0] = field->selfRobots[rid].position.x;
        Y[0] = field->selfRobots[rid].position.y;
    }
    int K = 0;  //����ͳ�Ƶ�ǰɨ��ֱ����9���ϰ�������˵�����
    const float C_xita = tan(angle);
    for (size_t i = 1; i < 10; ++i) {
        if ((X[i] > X[0] && X[i] < X[10]) || (X[i] < X[0] && X[i] > X[0])) {  //����ϰ�������ͱ��ϻ�����֮��
            judge = 4.f * (C_xita * (Y[0] - Y[i]) - X[i] - C_xita * C_xita * X[0])
                        * (C_xita * (Y[0] - Y[i]) - X[i] - C_xita * C_xita * X[0])
                    - 4.f * (1.f + C_xita * C_xita) * (X[i] * X[i] + (Y[0] - Y[i] - C_xita * X[0]) * (Y[0] - Y[i] - C_xita * X[0]) - d * d);

            //����֮������ϰ���
            if (judge <= 0.f) ++K;
        }
        else {
            ++K;
        }
    }
    return K;  //���K=9˵����ǰû���ϰ���
}

// ���ƻ�����ԭ����ת���ı�����ĳ���Ƕȣ�(-180 <= angle <= 180) --���ڷ�Ӧ������������д
void Turn_to(Robot *robot, float angle)
{
    float robot_angle = robot->rotation, angle_need = 0.f;
    float vl = 0.f, vr = 0.f;  //ָ�������˵���������

    // �����Ƕ�ֵת��Ϊ����
    if (angle < 0.f) angle += 360.f;
    if (robot_angle < 0.f) robot_angle += 360.f;
    angle_need = angle - robot_angle;  //Ŀ��Ƕ� - ��ǰ�Ƕ�

    if (angle_need < 0.f) {  //��Ҫ����������ת �� ������ > ������
        if (angle_need < -60.f) {
            vr = -30.f;
            vl = -vr;
        }
        else if (angle_need < -35.f) {
            vr = -10.f;
            vl = -vr;
        }
        else if (angle_need < -20.f) {
            vr = -8.f;
            vl = -vr;
        }
        else if (angle_need < -15.f) {
            vr = -6.f;
            vl = -vr;
        }
        else if (angle_need < -10.f) {
            vr = -5.f;
            vl = -vr;
        }
        else if (angle_need < -5.f) {
            vr = -4.f;
            vl = -vr;
        }
        else if (angle_need < -4.f) {
            vr = -3.f;
            vl = -vr;
        }
        else if (angle_need < -3.f) {
            vr = -2.f;
            vl = -vr;
        }
        else if (angle_need < -2.f) {
            vr = -1.f;
            vl = -vr;
        }
        else if (angle_need < -1.f) {
            vr = -0.5f;
            vl = -vr;
        }
        else if (angle_need < -0.5f) {
            vr = -0.2f;
            vl = -vr;
        }
        else if (angle_need < -0.25f) {
            vr = -0.1f;
            vl = -vr;
        }
        else if (angle_need < -0.05f) {
            vr = -0.05f;
            vl = -vr;
        }
        else {
            vl = vr = 0.f;
        }
    }

    else {  //��Ҫ����������ת �� ������ > ������
        if (angle_need >= 60.f) {
            vr = 30.f;
            vl = -vr;
        }
        else if (angle_need > 35.f) {
            vr = 10.f;
            vl = -vr;
        }
        else if (angle_need > 20.f) {
            vr = 8.f;
            vl = -vr;
        }
        else if (angle_need > 15.f) {
            vr = 6.f;
            vl = -vr;
        }
        else if (angle_need > 10.f) {
            vr = 5.f;
            vl = -vr;
        }
        else if (angle_need > 5.f) {
            vr = 4.f;
            vl = -vr;
        }
        else if (angle_need > 4.f) {
            vr = 3.f;
            vl = -vr;
        }
        else if (angle_need > 3.f) {
            vr = 2.f;
            vl = -vr;
        }
        else if (angle_need > 2.f) {
            vr = 1.f;
            vl = -vr;
        }
        else if (angle_need > 1.f) {
            vr = 0.5f;
            vl = -vr;
        }
        else if (angle_need > 0.5f) {
            vr = 0.2f;
            vl = -vr;
        }
        else if (angle_need > 0.25f) {
            vr = 0.1f;
            vl = -vr;
        }
        else if (angle_need > 0.05f) {
            vr = 0.05f;
            vl = -vr;
        }
        else {
            vl = vr = 0.f;
        }
    }

    Run(robot, vl, vr);
}

// ���û����˵����٣�����֤���ٵķ�Χ�� [-125��125]
inline void Run(Robot *robot, const float left_speed, const float right_speed)
{
    if (left_speed >= 0.f) {
        robot->wheel.leftSpeed = std::min(125.f, left_speed);
    }
    else {
        robot->wheel.leftSpeed = std::max(-125.f, left_speed);
    }

    if (right_speed >= 0.f) {
        robot->wheel.rightSpeed = std::min(125.f, right_speed);
    }
    else {
        robot->wheel.rightSpeed = std::max(-125.f, right_speed);
    }
}


/* -----------------------------------���˶��������-----------------------------------*/
/***************************************************************************************/

//opp & ball Ԥ��
void GoalieJudgeMove(Field &field, Robot &robot)
{
    Go1(&robot, PRIRY, 0.F);  //��λ --��ֹʧ��
    const float ry = field.selfRobots[0].position.y;
    const float rx = field.selfRobots[0].position.x;
    const float vr = field.selfRobots[0].rotation;
    const float Next_y = Will_Pos(field);  //next + bis /
    const float Will_y = Will_Y();         //will_y + bis_y /
    const float Bis_y = BisectorBallWill();
   //float AVE_Y = (Next_y + Will_y) / 2.F;
    float VOL_Y = (Will_y + Bis_y) / 2.F; 
    const float dl = Diss(rx, ry, ball_x, ball_y);
    const float pl = Diss(ball_x, ball_y, field.opponentRobots[Diss_o[0]].position.x, field.opponentRobots[Diss_o[0]].position.y);
    

    //��Զ���Ŷ�ν���
    if (CommentDistance() == 1) {
        if (CommentBallSpeed() == 1 || CommentBallSpeed() == 0)
            Go1(&robot, PRIRY, Will_y);
        else if (CommentBallSpeed() == 2 || CommentBallSpeed() == 3)
            Go1(&robot, PRIRY, VOL_Y);

        if (CommentBallAngle() == 0 || CommentBallAngle() == 1) {
            /* if (CommentBallSpeed() == 1 || CommentBallSpeed() == 0)
                             Go1(&robot,105.F, AVE_Y);
                         else if (CommentBallSpeed() == 2 || CommentBallSpeed() == 3) */
            Go1(&robot, PRIRY, Will_y);
        }
        else if (CommentBallAngle() == 2 || CommentBallAngle() == 3)
            Go1(&robot, PRIRY, VOL_Y);

        if (pl <= 6.F) {
            Go1(&robot, PRIRY, Next_y);
        }
        else
            Go1(&robot, PRIRY, VOL_Y);

        if (ballv() >= 3.F && pl > 10.F) {
            if (Ispositive())
                Go1(&robot, PRIRY, Will_y - 1.5F);
            else
                Go1(&robot, PRIRY, Will_y + 1.5F);
        }
    }

    if (CommentDistance() == 0) {
        const float sr = field.selfRobots[0].rotation;

        if (ball_y >= -25.F && ball_y <= 25.F) {
            if (CommentBallSpeed() == 2 || CommentBallSpeed() == 3)
                Go1(&robot,PRIRY, ball_y);
            else if (CommentBallSpeed() == 1 || CommentBallSpeed() == 0) {
                Go1(&robot,PRIRY, Will_y - 1.F);


                if (dl < 8.F)
                    Go1(&robot,PRIRY, ball_y);
            }
            if (CommentBallAngle() == 0 || CommentBallAngle() == 1)
                Go1(&robot,PRIRY, Will_y);
            else if (CommentBallAngle() == 3 || CommentBallAngle() == 2)
                Go1(&robot,PRIRY, ball_y);
        }

        else if (ball_y >= -90.F && ball_y < -25.F || ball_y >= 25.F && ball_y < 90.F) {
            if (ballv() >= 3.F && pl > 10.F) {
                if (Ispositive())
                    Go1(&robot,PRIRY, Will_y - 1.5F);
                else
                    Go1(&robot,PRIRY, Will_y + 1.5F);
            }
        }

        //�ڶ�v1 -- ��ʧ��
        /*else if (ball_y < -25.F) {
                Go1(&robot, PRIRY, GBOTY - 3.F);
                if (lr4 > 2.F) {
                    if (sr > 60.F && sr < 120.F) 
                        goalie.Run(-60.F, -60.F);
                    if (sr > -120.F && sr < 60.F) 
                        goalie.Run(60.F, 60.F);
                }
                if (ry < -23.F) 
                    Go1(&robot, PRIRY, GBOTY - 3.F);
            }
            else if (ball_y > 25.F) {
                Go1(&robot, PRIRY, GTOPY + 3.F);
                if (lr5 > 2.F) {
                    if (sr > 60.F && sr < 120.F) 
                        goalie.Run(60.F, 60.F);
                    if (sr > -120.F && sr < 60.F) 
                        goalie.Run(-60.F, -60.F);
                }
                if (ry > 23.F) 
                    Go1(&robot, PRIRY, GTOPY + 3.F);
            }*/
    }
}

const float Will_Pos(const Simuro::Field &field)  //�����opp��ball��һ�η�����line to ball�����yֵ--linerԤ��ֵ
{
    // float vr = field->opponentRobots[Diss_o[0]].rotation;
    const float ny = field.opponentRobots[Diss_o[0]].position.y;
    const float nx = field.opponentRobots[Diss_o[0]].position.x;
    float delx = nx - ball_x;
    float dely = ny - ball_y;
    float kp = 0;

    if (delx != 0)
        kp = dely / delx;
    else
        kp = 0;
    float bp = ball_y - kp * ball_x;

    /*if (kp != 0)
        bp = BALL_Y - kp * BALL_X; 
    else
        bp = BALL_Y;*/
    float next_y = kp * PRIRY + bp;
    if (next_y > 20.F)
        next_y = 20.F;
    else if (next_y < -20.F)
        next_y = -20.F;
    return next_y;
}

const float Will_Y()  // ��������linerԤ��ֵ ���Σ��ϵ��ʹ��
{
    const float BALL_X1 = lastball.at(0).at(0);
    const float BALL_X2 = lastball.at(2).at(0);
    float delx = BALL_X1 - BALL_X2;
    const float BALL_Y1 = lastball.at(0).at(1);
    const float BALL_Y2 = lastball.at(2).at(1);
    float dely = BALL_Y1 - BALL_Y2;
    float kb = dely / delx;
    float NEXT_Y;
    if (delx != 0)
        kb = dely / delx;
    else
        kb = 0;
    float bx = BALL_Y2 - kb * BALL_X2;
    NEXT_Y = kb * PRIRY + bx;
    if (NEXT_Y > 20.F)
        NEXT_Y = 20.F;
    else if (NEXT_Y < -20.F)
        NEXT_Y = -20.F;
    return NEXT_Y;
}

const float BisectorBallWill()  //�򵥽�ƽ����Ԥ�Ⲣ��������ֵ -- ����ball
{
    // kֵ�ж� -- Σ�սǶ�Ԥ����k
    float delx = ball_x - 110.F;
    float dely1 = ball_y - 20.F;
    float dely2 = ball_y + 20.F;
    float ka = 0, kb = 0;

    if (delx != 0) {
        ka = dely1 / delx;
        kb = dely2 / delx;
    }
    else
        ka = 0, kb = 0;

    float Bis_K = (ka + kb) / 2;  //����ֵk
    float bx = ball_y - Bis_K * ball_x;
    float Bis_y = Bis_K * PRIRY + bx;
    if (Bis_y > 20.F)
        Bis_y = 20.F;
    else if (Bis_y < -20.F)
        Bis_y = -20.F;
    return Bis_y;
}

const int CommentDistance()
{
    int CommentDiss = 0;
    if (ball_x >= 95.F && ball_x <= 110.F)
        CommentDiss = 0;
    if (ball_x >= 75.F && ball_x < 95.F)
        CommentDiss = 1;
    if (ball_x >= 55.F && ball_x < 75.F)
        CommentDiss = 2;
    if (ball_x >= 0 && ball_x < 55.F)
        CommentDiss = 3;
    if (ball_x >= -110.F && ball_x < 0.F)
        CommentDiss = 4;
    return CommentDiss;
}

float ballv()
{
    const float lc1 = Diss(lastball.at(0).at(0), lastball.at(0).at(1), lastball.at(1).at(0), lastball.at(1).at(1));
    const float lc2 = Diss(lastball.at(1).at(0), lastball.at(1).at(1), lastball.at(2).at(0), lastball.at(2).at(1));
    const float angle = atan2(lastball.at(2).at(1) - lastball.at(1).at(1), lastball.at(2).at(0) - lastball.at(1).at(0));
    float ball_v = fabs((lc1 + lc2) / 2);
    return ball_v = fabs((lc1 + lc2) / 2);
}

//��������
const int CommentBallSpeed()
{
    int CommentBS = 0;
    const float ball_v = ballv();  //Ԥ������
    if (ball_v >= 2.F && ball_v < 3.F)
        CommentBS = 2;  //����
    else if (ball_v >= 0.F && ball_v < 2.F)
        CommentBS = 3;  //����
    else if (ball_v >= 3.F && ball_v < 4.F)
        CommentBS = 1;  //����
    else if (ball_v >= 4.F && ball_v < 5.F)
        CommentBS = 0;  //����
    return CommentBS;
}

//��ƫ������
const int CommentBallAngle()
{
    int CommentBA = 0;
    const float angle = atan2(lastball.at(2).at(1) - lastball.at(1).at(1), lastball.at(2).at(0) - lastball.at(1).at(0));
    if (angle >= 0.F && angle <= 25.F)
        CommentBA = 0;
    else if (angle > 25.F && angle <= 45.F)
        CommentBA = 1;
    else if (angle > 45.F && angle <= 75.F)
        CommentBA = 2;
    else if (angle > 75.F && angle < 90.F)
        CommentBA = 3;
    return CommentBA;
}

bool Ispositive()
{
    const float BALL_Y1 = lastball.at(0).at(1);
    const float BALL_Y2 = lastball.at(2).at(1);
    float dely = BALL_Y1 - BALL_Y2;
    if (dely < 0.F)
        return true;
    else
        return false;
}


// 1.ֱ�巣��
void penalty_1(Robot *robot)
{
    // ִ�з���Ļ��������� ��x, y�� = ��-28.f, 24.f��  �Ƕȣ�-154.f
    // ִ�з���Ļ��������� ��x, y�� = ��-35.f, -20.f�� �Ƕȣ�155.f

    if (ball_x < -77.f) { Run(robot, 0.f, 0.f); }
    else
        Run(robot, 125.f, 125.f);
}

//������---�������
void penalty_defend(Robot *robot)
{
    //����Ա (105.f��0.f) �Ƕ� = 180.f
    if (robot->position.x < 84.67f) { 
        Run(robot, 0.f, 0.f); 
    }
    else {
        Run(robot, 125.f, 125.f);
    }
}

// 1.������---�������
void PenaltyDefend_1(Field *field, Robot *robot)
{
    // ����Ա (106.F, 0.F) �Ƕ� = ��180.F
    // ��72.5F��0.F
    if (fabs(ball_y) <= 0.25F && fabs(ball_x - 72.5F) <= 0.25F) {  //�ж����Ƿ��ڷ����
        Run(robot, 125.f, 125.f);
        if (DissRobot(robot, 72.5F, 0.F) < 12.5F) { 
            Run(robot, 0.f, 0.f); 
        }
    }
    else {
        GoalieJudgeMove(*field, field->selfRobots[0]);
        //GoalkeeperMain(field, robot);  //��ת������Ա������
    }
}
// 2.������---������ش���ת
void PenaltyDefend_2(Field *field, Robot *robot)
{
    // ����Ա (106.F, 0.F) �Ƕ� = ��180.F
    // ��72.5F��0.F
    if (fabs(ball_y) <= 0.25F && fabs(ball_x - 72.5F) <= 0.25F) {  //�ж����Ƿ��ڷ����
        Run(robot, 125.F, 125.F);
        if (DissRobot(robot, 72.5F, 0.F) < 12.5F) { 
            Run(robot, 125.F, -125.F); 
        }
    }
    else {
        GoalieJudgeMove(*field, field->selfRobots[0]);
        //GoalkeeperMain(field, robot);  //��ת������Ա������
    }
}


void update(Field *field)  //��¼����
{
    if (Time - Note >= 1)  //ÿ��һ�ĸ���һ������
    {
        /*for (size_t i = 0; i < 2; i++) {
            lastball.at(i).at(0) = lastball.at(i + 1).at(0);
            lastball.at(i).at(1) = lastball.at(i + 1).at(1);
            Note = Time;
        }*/

        lastball.at(0) = lastball.at(1);
        lastball.at(1) = lastball.at(2);
        Note = Time;

        //lastball.at(2).at(0) = ball_x;  //��¼��ǰ����
        //lastball.at(2).at(1) = ball_y;
        lastball.at(2) = { ball_x, ball_y };
    }
}
