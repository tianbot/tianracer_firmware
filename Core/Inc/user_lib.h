#ifndef USER_LIB_H
#define USER_LIB_H

#include "stm32f4xx_hal.h"
#pragma pack(push)
#pragma pack(1)
typedef struct
{
    float input;        //��������
    float out;          //�������
    float min_value;    //�޷���Сֵ
    float max_value;    //�޷����ֵ
    float frame_period; //ʱ����
} ramp_function_source_t;

typedef struct
{
    float input;        //��������
    float out;          //�˲����������
    float num[1];       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
#pragma pack(pop)
//���ٿ���
extern float invSqrt(float num);

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);
//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
//��������
extern void abs_limit(float *num, float Limit);
//�жϷ���λ
extern float sign(float value);
//��������
extern float float_deadline(float Value, float minValue, float maxValue);
//int26����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern float float_constrain(float Value, float minValue, float maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern float loop_float_constrain(float Input, float minValue, float maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern float theta_format(float Ang);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
