#ifndef __TASK_HANDLEMODE_H_
#define __TASK_HANDLEMODE_H_

void Task_HandleMode_Process(HandleModeInfo HMInfo);
void Task_HandleMode_Clear(void);
//uint16_t isButtonPressed(char Analysis_flag);

#define MASK_X_LEFT   0x80  // ��X�Ḻ����
#define MASK_X_RIGHT  0x40  // �ң�X��������
#define MASK_Y_UP     0x20  // �ϣ�Y��������
#define MASK_Y_DOWN   0x10  // �£�Y�Ḻ����
#define MASK_Z_SINK   0x08  // ������Z�Ḻ����
#define MASK_Z_RISE   0x04  // ����Z��������
#define MASK_PITCH_DOWN 0x02  // ������Pitch �£�
#define MASK_PITCH_UP 0x01  // ����Pitch �ϣ�


#endif
