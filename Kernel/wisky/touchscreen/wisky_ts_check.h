#ifndef _WISKY_TS_CHECK_H_
#define _WISKY_TS_CHECK_H_

//�������������
enum ts_type{
	TS_TYPE_QIUTIAN = 0,//(����΢)
	TS_TYPE_PINGBO,//(ƽ��)
	TS_TYPE_RUISHI,//(���ӹ��)
	TS_TYPE_HAOEN,//(����)
	TS_TYPE_NULL1,//δ����
	TS_TYPE_ERROR = -1,
};

//param:none
//return: negative number--no correct ts detect, positive number--ts type
//����ֵ�ο����ļ�:wisky_ts_check.h �ļ��е�enum ts_type
extern int ts_check_type(void);

#endif//_WISKY_TS_CHECK_H_

