#ifndef __IMU_H__
#define __IMU_H__

#include "debug.h"


/**
 * @brief IMU �Ĵ�����ַӳ���
 * 
 */
typedef struct __imu_reg_lut__{
	u8 address;
	u8 ID;				// ��������ʶ
	u8 Acc;				// ���ٶȼƻ�ַ
	u8 Gyro;			// �����ǻ�ַ
	u8 Temp;			// �¶�
	u8 DataRate;		// ������
	u8 Scale_Acc;		// ���ٶȼ�����
	u8 Scale_Gyro;		// ����������
} IMU_Reg;

u8 IMU_Init(void);                                 //��ʼ��IMU


#endif
