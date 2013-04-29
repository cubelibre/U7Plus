添加开机logo到内核wisky目录的方法：

在此以添加 logo_linux_clut224.ppm为例，添加其他ppm开机logo与此类似。

1)将 logo_linux_clut224.ppm文件拷贝到wisky\logo\build_logo\ppm\目录下

2)在文件 wisky\logo\wisky_logo_main.h 中添加如下数据结构变量声明：
	extern const struct linux_logo logo_linux_clut224;
	
3)在文件 wisky\logo\wisky_logo_main.c 中添加如下logo数据引用：

#if defined(WISKY_LOGO_LINUX)
		logo = &logo_linux_clut224;
#endif

4)在wisky/include/wisky_env.h的资源表的“Boot Logo”项添加宏新logog资源，以供其他工程也可以使用
	* WISKY_LOGO_LINUX = 选择linux 224-color 开机logo

5)在wisky\logo\wisky_logo_data_all.c文件中包含要编译的开机logo：
	#elif defined(WISKY_LOGO_LINUX)
	#include "./logo_data_all/logo_linux_clut224.c"

6)在你相应的OEM配置头文件(wisky\include\oem\config_xxx.h)中定义宏以使用该开机logo：
	#define WISKY_LOGO_LINUX
	
7)到内核根目录执行命令进行编译：
./buildkernel.sh



--------------------------------------------------------
其他格式图片（PNG、JPG）
--------------------------------------------------------
1.如果是PNG格式图片，先使用本目录的png文件夹里的pngtoppm.sh将logo.png格式图片转换为PPM格式图片。
2.如果是JPG格式图片，先用xp上的画图软件或者PS打开JGP图片后另存为PNG格式图片，
	再使用png文件夹里的pngtoppm.sh将logo.png格式图片转换为PPM格式图片。




