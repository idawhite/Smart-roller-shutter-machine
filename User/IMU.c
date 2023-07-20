#include "IMU.h"
#include "debug.h"
#include "IIC.h"
#include "qmi8658.h"

const IMU_Reg Reg_QMI8658 = {
	.address = QMI8658_SLAVE_ADDR_H,	   // I2C ��ַ
	.ID = Qmi8658Register_WhoAmI,				   // ��������ʶ
	.Acc = Qmi8658Register_Ax_L,		   // ���ٶȼƻ�ַ
	.Gyro = Qmi8658Register_Gx_L,		   // �����ǻ�ַ
	.Temp = Qmi8658Register_Tempearture_L, // �¶�
	.DataRate = 0,						   // ������
	.Scale_Acc = 0,						   // ���ٶȼ�����
	.Scale_Gyro = 0						   // ����������
};

// ��ʼ�� IMU
// ����ֵ:0,�ɹ�
//     ����,�������
u8 IMU_Init(void)
{
	return 0;
}
