��ӿ���logo���ں�wiskyĿ¼�ķ�����

�ڴ������ logo_linux_clut224.ppmΪ�����������ppm����logo������ơ�

1)�� logo_linux_clut224.ppm�ļ�������wisky\logo\build_logo\ppm\Ŀ¼��

2)���ļ� wisky\logo\wisky_logo_main.h ������������ݽṹ����������
	extern const struct linux_logo logo_linux_clut224;
	
3)���ļ� wisky\logo\wisky_logo_main.c ���������logo�������ã�

#if defined(WISKY_LOGO_LINUX)
		logo = &logo_linux_clut224;
#endif

4)��wisky/include/wisky_env.h����Դ��ġ�Boot Logo������Ӻ���logog��Դ���Թ���������Ҳ����ʹ��
	* WISKY_LOGO_LINUX = ѡ��linux 224-color ����logo

5)��wisky\logo\wisky_logo_data_all.c�ļ��а���Ҫ����Ŀ���logo��
	#elif defined(WISKY_LOGO_LINUX)
	#include "./logo_data_all/logo_linux_clut224.c"

6)������Ӧ��OEM����ͷ�ļ�(wisky\include\oem\config_xxx.h)�ж������ʹ�øÿ���logo��
	#define WISKY_LOGO_LINUX
	
7)���ں˸�Ŀ¼ִ��������б��룺
./buildkernel.sh



--------------------------------------------------------
������ʽͼƬ��PNG��JPG��
--------------------------------------------------------
1.�����PNG��ʽͼƬ����ʹ�ñ�Ŀ¼��png�ļ������pngtoppm.sh��logo.png��ʽͼƬת��ΪPPM��ʽͼƬ��
2.�����JPG��ʽͼƬ������xp�ϵĻ�ͼ�������PS��JGPͼƬ�����ΪPNG��ʽͼƬ��
	��ʹ��png�ļ������pngtoppm.sh��logo.png��ʽͼƬת��ΪPPM��ʽͼƬ��




