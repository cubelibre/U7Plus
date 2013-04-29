一、编译方法
执行以下命令编译：
./buildkernel.sh

要一次编译所以工程客户固件，执行以下命令：
./build_all.sh

原始编译命令：
make kernel.img

二、更新说明

ker_ver1.15@20120928
1.修改ft5x驱动，m101b增加reset引脚控制.
2.m101b增加ft5x平波配置，wifi配置修改为rk903.
3.m830r lcd 修改为LP097X02.
4.修改explay_surface801 oem配置.
5.修改ployer_momo9 oem配置.

ker_ver1.14@20120923
1.解决M876R开机屏闪.
2.更新ft5x_m830r，ft5x_momo9,zet6221_m876hr触摸屏配置.
3.新增ployer_momo9,mingxuan_m100,dns开机logo.
4.更新dns_e75,p76e客户配置.
5.修改m101b,m830r,m868r,m876r,m876hr wifi配置为RTL8192C.

ker_ver1.13@20120914
1.更新m868 豪恩zet6221触摸屏配置.
2.更新w001 zet6221配置.
3.修改按键驱动，adc值为0为合法键值.
4.修改摄像头驱动，前置兼容hi704,gc0308
5.将wifi驱动移到wisky目录.
6.修改gsensor驱动，调整上报值为g值附近.
7.改善w001前后置摄像头效果.
8.增加DNS,P76E,explay_surfer801等客户配置.

ker_ver1.12@20120912
1.增加switch_gpio驱动.
2.修改momo9,w001频率电压表，提高vdd-cpu电压，解决死机问题.
3.修改zet6221触摸屏驱动，改中断触发方式为低电平，解决触摸屏反应迟钝问题.
4.增加gt811触摸屏驱动.
5.增加es8323 codec驱动.
6.增加momo9,w001工程.
7.将switch,codec驱动移到wisky/switch,wisky/sound目录下
8.更新默认配置文件wisky_rk3066_axp202_defconfig，wisky_rk3066_defconfig，wisky_rk3066_wm8326_defconfig
9.修改使用axp202机型，按power会重启问题.