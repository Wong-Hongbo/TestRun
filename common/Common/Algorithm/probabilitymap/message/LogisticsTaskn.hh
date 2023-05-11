#ifndef __LOGISTICSTASK__
#define __LOGISTICSTASK__

#include "rcs.hh"
#include "NMLmsgExn.hh"


#define LOGISTICSTASK_MSG_TYPE 18322


struct  TaskPoint
{
    double x;   
    double y;
    char   turn_info;// ֹͣ�ߺ�·�ڣ�0:no turn, 1: left turn, 2: forward, 3: right turn, 4: u-turn,  5:·�ڳ��ڵ�ͬʱ��ʾ·��ֹͣ��λ��
    unsigned short  light_type;// ��λ����ת�����ͣ�0���ޣ�1��Բ�ƣ�2����ͷ����ʮλ��ֱ�е����ͣ�0���ޣ�1��Բ�ƣ�2����ͷ����
                               // ��λ����ת�����ͣ�0���ޣ�1����ͷ����ǧλ��uturn�����ͣ�0���ޣ�1����ͷ��,
                               // ��λ���Ƶ����з�ʽ��0������Ϣ��1�����ţ�2�����ţ�

    char   light_pos_exit;    // �Ƿ��еƵ�λ����Ϣ��0��û�У�1����
    short  light_x;    // ���뵱ǰȫ�ֵ�����λ�ú͸߶ȣ���λΪcm
    short  light_y;
    short  light_height;

    int    env_type;    // ����������ֹͣ�ߣ������ߣ����ˣ���ͨ�����ͣ�
    int    act_type;    // ������Ϊ������ͣ��������ʱ�䣬������������
    int    curve_radius;    // ����
    int    max_speed;    // ����
    int    slope;    // �¶�
    int    point_id;    // ȫ��id��
};


class LOGISTICSTASK_MSG : public NMLmsgEx
{
public:
    LOGISTICSTASK_MSG();

    void update(CMS *);

    int point_num;    //��������

    TaskPoint task_point[500];    //ȫ�������

    int road_num;    //·������
    int       task_road[500];    //ȫ��·id����
};

// Declare NML format function
extern int LogisticsTaskFormat(NMLTYPE, void *, CMS *);

#endif