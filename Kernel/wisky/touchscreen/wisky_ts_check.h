#ifndef _WISKY_TS_CHECK_H_
#define _WISKY_TS_CHECK_H_

//触摸屏面板类型
enum ts_type{
	TS_TYPE_QIUTIAN = 0,//(秋田微)
	TS_TYPE_PINGBO,//(平波)
	TS_TYPE_RUISHI,//(瑞视光电)
	TS_TYPE_HAOEN,//(豪恩)
	TS_TYPE_NULL1,//未定义
	TS_TYPE_ERROR = -1,
};

//param:none
//return: negative number--no correct ts detect, positive number--ts type
//返回值参考请文件:wisky_ts_check.h 文件中的enum ts_type
extern int ts_check_type(void);

#endif//_WISKY_TS_CHECK_H_

