/*

0）所有的定义采用两个字符来表示 "XX",前一个表示类别，后一个表示索引
1）字符 A ~ Z 分别表示了不同的含义 ，如果有新的类别定义，请选择一个没有使用的字符来表示
2）在一个类别下添加新的开关，请使用数字和字母的组合，并且不要重复，否则编译报错 1,2,3,4,5,6,7,8,9 ,A,B,C,D,....X,Y,Z
3) 在一个类别中添加的新的开关不要插入在类别的中间，一律插入在该类别的末尾位置
4）如果需要添加与设置菜单无关的选项，一律加在其他的类别中，即开始字符为 "Z"的类别中，并且加在末尾的位置

配置标号说明:

A 无线和网络设置
B 通话设置
C 声音
D 显示
E 位置和安全
F 应用程序
G 帐户与同步
H 隐私权
I 存储
J 语言和键盘
K 语音输入与输出
L 辅助功能
M 日期和时间
N 关于手机
Z 其他配置

 */

#if defined(WISKY_OEM_ALL_ON)

#ifndef _CONFIG_ALL_ON_H_
#define _CONFIG_ALL_ON_H_


#define MAX_CONFIG_BUFFLEN     64//最长的BUFFER长度

//A 无线和网络设置
#define ANDROID_CONFIG_A1     "1"//A1）飞行模式 0-关闭 1-开启（默认0）
#define ANDROID_CONFIG_A2     "1"//A2）WIFI 0-关闭 1-开启（默认0）
#define ANDROID_CONFIG_A3     "1"//A3）蓝牙 0-关闭 1-开启（默认0）
#define ANDROID_CONFIG_A4     "1"//A4) 以太网0-关闭,1-开启(默认0)


//B 通话设置


//C 声音
#define ANDROID_CONFIG_C1      "1"//C1）静音模式 0-关闭 1-开启（默认0）
#define ANDROID_CONFIG_C2      "3"//C2）振动 0-始终 1-一律不 2-仅在静音模式下来电振动 4-仅在非静音模式下来电振动（默认3）
#define ANDROID_CONFIG_C3      "100"//C3）音量 -铃声音量  0~100的百分比表示 （默认 70）
#define ANDROID_CONFIG_C4      "100"//C4）音量 -媒体音量  0~100的百分比表示 （默认 70）
#define ANDROID_CONFIG_C5      "100"//C5）音量 ---闹钟音量  0~100的百分比表示 （默认 70）
#define ANDROID_CONFIG_C6      "1"//C6）音量--将来电音量用作通知音量 0-关闭 1-开启 （默认1）
#define ANDROID_CONFIG_C7      "1"//C7）手机铃声  直接用索引来表示 （默认1）0-静音 1-BeatPlucker 2-BentleyDubs 3-BirdLoop ……..
#define ANDROID_CONFIG_C8      "12"//C8）通知铃声  直接用索引来表示 （默认12）0-静音 1-Beat_Box_Android 2-CaffeineSnake 3-DearDeer …….. 
#define ANDROID_CONFIG_C9      "1"//C9）按键操作音 0-关闭 1-开启（默认1）
#define ANDROID_CONFIG_CA      "1"//CA）选择操作音 0-关闭 1-开启（默认0）
#define ANDROID_CONFIG_CB      "1"//CB）屏幕锁定提示音 0-关闭 1-开启（默认0）
#define ANDROID_CONFIG_CC      "1"//CC）触感  0-关闭 1-开启（默认1）

//D 显示
#define ANDROID_CONFIG_D1      "70"//D1）亮度  0~100的百分比表示
#define ANDROID_CONFIG_D2      "1"//D2）自动旋转屏幕 0-关闭 1-开启（默认1）
#define ANDROID_CONFIG_D3      "2"//D3）动画  0-无动画  1-部分动画 2-所有动画 （默认2）
#define ANDROID_CONFIG_D4      "2"//D4）屏幕超时  直接用索引来表示 （默认2） 0-15秒 1-30秒 2-1分钟 3-2分钟

//E 位置和安全
#define ANDROID_CONFIG_E1      "1"//E1）使用无限网络 0-关闭 1 -开启（默认0）
#define ANDROID_CONFIG_E2      "1"//E2）使用GPS卫星 0-关闭 1 -开启（默认1）
#define ANDROID_CONFIG_E5      "1"//E5）密码可见 0-关闭 1-开启（默认1）
#define ANDROID_CONFIG_E7      "1"//E7）使用安全凭证功能 0-关闭1-开启 （默认0）

//F 应用程序
#define ANDROID_CONFIG_F1      "1"//F1）未知来源 0-关闭 1-开启（默认0）
#define ANDROID_CONFIG_F6      "0"//F6）USB调试 0-关闭 1-开启 （默认1）
#define ANDROID_CONFIG_F7      "1"//F7）保持唤醒状态 0--关闭 1-开启 （默认0）
#define ANDROID_CONFIG_F8      "1"//F8）允许模拟地点 0-关闭 1-开启 （默认1）

//G 账户与同步
#define ANDROID_CONFIG_G1      "1"//G1）背景数据 0-关闭 1-开启（默认1）
#define ANDROID_CONFIG_G2      "1"//G2）自动同步 0-关闭 1-开启（默认1）

//H 隐私权

//I 存储
#define ANDROID_CONFIG_I1      "2"//I1）存储设备USB连接方式:0---MTP,1--PTP,2---UMS（默认2）

//J 语言和键盘
//各国语言对应代号
//da_DK  丹麦语  丹麦  
//DE_AT  德语  奥地利  
//DE_CH  德语  瑞士  
//DE_DE  德语  德国  
//el_GR  希腊语  希腊  
//en_CA  英语  加拿大  
//en_GB  英语  联合王国  
//en_IE  英语  爱尔兰  
//en_US  英语  美国  
//es_ES  西班牙语  西班牙  
//fi_FI  芬兰语  芬兰  
//fr_BE  法语  比利时  
//fr_CA  法语  加拿大  
//fr_CH  法语  瑞士  
//fr_FR  法语  法国  
//it_CH  意大利语  瑞士  
//it_IT  意大利语  意大利  
//ja_JP  日语  日本  
//ko_KR  韩国语  韩国  
//nl_BE  荷兰语  比利时  
//nl_NL  荷兰语  荷兰  
//no_NO  挪威语 (Nynorsk)  挪威  
//no_NO_B  挪威语 (Bokm?l)  挪威  
//pt_PT  葡萄牙语  葡萄牙  
//sv_SE  瑞典语  瑞典  
//tr_TR  土耳其语  土耳其  
//zh_CN  中文（简体）  中国  
//zh_TW  中文（繁体）  中国台湾
#define ANDROID_CONFIG_J1     "en_US"//J1）选择语言 （默认简体中文）
#define ANDROID_CONFIG_J2     "0"//J2）支持语言列表，0---全部语言，其他需选择加'@'间隔

//K 语音输入与输出

//L 辅助功能
#define ANDROID_CONFIG_L1     "1"//L1）辅助功能 0-关闭 1-开启（默认设置0）
#define ANDROID_CONFIG_L2     "1"//L2）按"电源"按钮结束通话 0-关闭 1-开启（默认设置0）


//M 日期和时间设置
#define ANDROID_CONFIG_M1     "1"//M1）自动   0-关闭 1-开启（默认设置1）
#define ANDROID_CONFIG_M5     "1"//M5）使用24小时格式   0-关闭 1-开启（默认设置0）
#define ANDROID_CONFIG_M6     "0"//M6）选择日期格式   （默认0） 0-普通（2011-12-31）1-12-31-2011  2-31-12-2011  3-2011-12-31


//N 关于手机

//Z 其他配置
#define ANDROID_CONFIG_Z1     "1"//Z1）是否支持后台定时任务查杀 0-关闭 1-开启 （默认0）
#define ANDROID_CONFIG_Z2     "0"//Z2）充电时电池电量的显示模式 0 --显示电量模式 1 --不显示电量，仅仅显示正在充电的状态 （默认0）
#define ANDROID_CONFIG_Z3     "1"//Z3）查拔耳机是否有提示框 0-- 无提示框，1--有提示框信息 （默认0）
#define ANDROID_CONFIG_Z4     "1"//Z4）充电时是否闪烁按键灯 0--不闪烁 ，1 --慢速闪烁 （默认0）
#define ANDROID_CONFIG_Z5     "1"//Z5）USB连接后有数据拷贝动作时快速闪烁按键灯 0--不闪烁 ，1--快速闪烁 （默认0）
#define ANDROID_CONFIG_Z6     "1"//Z6）是否启用充电模式，即关机状态下插入充电器不开机，直接充电 0--不启用，1--启用 （默认0）
#define ANDROID_CONFIG_Z7     "1"//Z7）GPS在空闲时是否提示关闭对话框 0--不提示 1-- 提示 （默认0）
#define ANDROID_CONFIG_Z8     "1"//Z8）长按ESC是否加锁GSENSOR功能 0--不加 1-- 加，长按该键缩定GSENSOR （默认0）
#define ANDROID_CONFIG_Z9     "/flash"//Z9）磁盘的路径  （字符串信息，主要是SD卡和NAND的路径名称）
#define ANDROID_CONFIG_ZA     "android"//ZA）磁盘的名称  （客户需要在我的电脑中显示的磁盘名称字符串信息）
#define ANDROID_CONFIG_ZB     "android"//ZB）USB信息 （客户需要在USB驱动中显示信息）
#define ANDROID_CONFIG_ZC     "android"//ZC）客户名称  （字符串信息）
#define ANDROID_CONFIG_ZD     WISKY_HW_VERSION //ZD）客户的产品型号 （字符串信息）
#define ANDROID_CONFIG_ZE     "All config on (V1.0)"//ZE）客户的版本号 （字符串信息）
#define ANDROID_CONFIG_ZF	"1"//ZF) 开机时的"ANDROID_"闪烁光标是否可见0--不可见1--可见
#if defined(WISKY_TOUCHSCREEN_AK4183)|| defined(WISKY_TOUCHSCREEN_TCS9135) || defined(WISKY_TOUCHSCREEN_LZ300) \
	||defined(WISKY_TOUCHSCREEN_TCS9046)
#define ANDROID_CONFIG_ZG	"1"//ZG) 是否使用触摸屏校准0--不使用1--使用
#else
#define ANDROID_CONFIG_ZG	"0"//ZG) 是否使用触摸屏校准0--不使用1--使用
#endif
#define ANDROID_CONFIG_ZH	"xxxx"//ZH)	默认壁纸，为壁纸图片名称
#define ANDROID_CONFIG_ZI	"1"//ZI)	休眠唤醒后是否显示竖屏解锁界面 0 -- 不显示，原来是什么状态就是什么状态 1-- 强制转换成竖屏显示状态来解锁（默认0）
#define ANDROID_CONFIG_ZJ	"1"//ZJ)	光传感器亮度自动调节默认开关状态0---关闭，1---打开（默认0）
#define ANDROID_CONFIG_ZK	"0"//ZK)	硬件板使用ADC 按键，0---不使用ADC 按键，1---使用ADC按键（默认0）
#define ANDROID_CONFIG_ZL     "android4"//ZL）ADB名称显示信息 （客户需要在PC设备管理器中看到的ADB名称）
#if (WISKY_LCD_WIDTH == 800)
#define ANDROID_CONFIG_ZM	"800" //ZM) 屏幕分辨率的宽度(像素点为单位)
#elif(WISKY_LCD_WIDTH == 1024)
#define ANDROID_CONFIG_ZM	"1024" //ZM) 屏幕分辨率的宽度(像素点为单位)
#elif(WISKY_LCD_WIDTH == 1280)
#define ANDROID_CONFIG_ZM	"1280" //ZM) 屏幕分辨率的宽度(像素点为单位)
#else
#error No correct LCD width setup
#endif

#if (WISKY_LCD_HEIGHT == 480)
#define ANDROID_CONFIG_ZN	"480" //ZN) 屏幕分辨率的高度(像素点为单位)
#elif(WISKY_LCD_HEIGHT == 600)
#define ANDROID_CONFIG_ZN	"600" //ZN) 屏幕分辨率的高度(像素点为单位)
#elif(WISKY_LCD_HEIGHT == 768)
#define ANDROID_CONFIG_ZN	"768" //ZN) 屏幕分辨率的高度(像素点为单位)
#elif(WISKY_LCD_HEIGHT == 800)
#define ANDROID_CONFIG_ZN	"800" //ZN) 屏幕分辨率的高度(像素点为单位)
#else
#error No correct LCD height setup
#endif
#define ANDROID_CONFIG_ZO	"1"//ZO) 是否使用动态壁纸0---不使用，1---使用
#define ANDROID_CONFIG_ZP	WISKY_KERNEL_VERSION //ZP)Wisky内部内核版本号

#if WISKY_ENABLE_WIFI
#define ANDROID_CONFIG_ZQ	"1" //ZQ)硬件上是否使用WiFi模块0--不使用，1--使用
#else
#define ANDROID_CONFIG_ZQ	"1" //ZQ)硬件上是否使用WiFi模块0--不使用，1--使用
#endif
#if WISKY_ENABLE_BLUETOOTH
#define ANDROID_CONFIG_ZR	"1" //ZR)硬件上是否使用Bluetooth模块0--不使用，1--使用
#else
#define ANDROID_CONFIG_ZR	"1" //ZR)硬件上是否使用Bluetooth模块0--不使用，1--使用
#endif
#if WISKY_ENABLE_HDMI
#define ANDROID_CONFIG_ZS	"1" //ZS)硬件上是否使用HDMI模块0--不使用，1--使用
#else
#define ANDROID_CONFIG_ZS	"1" //ZS)硬件上是否使用HDMI模块0--不使用，1--使用
#endif
#if WISKY_ENABLE_GPS
#define ANDROID_CONFIG_ZT	"1" //ZT)硬件上是否使用GPS模块0--不使用，1--使用
#else
#define ANDROID_CONFIG_ZT	"1" //ZT)硬件上是否使用GPS模块0--不使用，1--使用
#endif

#define ANDROID_CONFIG_ZU	"1" //ZU)硬件上是否使用3G模块0--不使用，1--使用

#if WISKY_ENABLE_CAMERA
#define ANDROID_CONFIG_ZV	"1" //ZV)硬件上是否使用Camera模块0--不使用，1--使用
#else
#define ANDROID_CONFIG_ZV	"1" //ZV)硬件上是否使用Camera模块0--不使用，1--使用
#endif
#if WISKY_ENABLE_VIBRATOR
#define ANDROID_CONFIG_ZW	"1" //ZW)硬件上是否使用Vibrator振动器模块0--不使用，1--使用
#else
#define ANDROID_CONFIG_ZW	"1" //ZW)硬件上是否使用Vibrator振动器模块0--不使用，1--使用
#endif

#define ANDROID_CONFIG_ZX	"1" //ZX)硬件上是否使用LightSensor光传感器模块0--不使用，1--使用

#if WISKY_ENABLE_GSENSOR
#define ANDROID_CONFIG_ZY	"1" //ZY)硬件上是否使用G-Sensor模块0--不使用，1--使用
#else
#define ANDROID_CONFIG_ZY	"1" //ZY)硬件上是否使用G-Sensor模块0--不使用，1--使用
#endif
#define ANDROID_CONFIG_ZZ	"1" //ZY)是否使用3G通话功能，0--不使用，1--使用
#define ANDROID_CONFIG_Y1	"1" //Y1)是否开启设置菜单中的静音设置0--不开启 1--开启
#define ANDROID_CONFIG_Y2	"1" //Y2)是否开启设置菜单中的以太网设置项0--不开启 1--开启
#define ANDROID_CONFIG_Y3	"0" //Y3)默认输入法设定0-android键盘 1-wisky输入法 2-谷歌拼音输入法
#define ANDROID_CONFIG_Y4	"0" //Y4)是否屏蔽关于设备中的“电量使用情况”0--不屏蔽 1--屏蔽 （默认0）
#define ANDROID_CONFIG_Y5	"0" //Y5)是否屏蔽声音设置中的“紧急提示音” 0--不屏蔽 1--屏蔽 （默认0）
#define ANDROID_CONFIG_Y6	"http://www.google.com.hk/" //Y6)浏览器的默认主页  （字符串信息）（默认空）
#define ANDROID_CONFIG_Y7	"1" //Y7)是否在通知栏上添加时间标识    0--不添加 1--添加 （默认0）
#define ANDROID_CONFIG_Y8	"1" //Y8)解锁方式选择:0---Android默认解锁，1---Wisky解锁方式
#define ANDROID_CONFIG_Y9	"4" //Y9)开机默认屏幕方向：0--横屏，1--竖屏，2---反横屏,3---反竖屏,其他值为可旋转
#define ANDROID_CONFIG_YA	"1" //YA)禁止系统进入深度睡眠模式 0--开启，1--禁止 （默认0）
#define ANDROID_CONFIG_YB	"0" //YB)触感是否使用连续振动0---按下时振动一次，1---按下时连续振动
#define ANDROID_CONFIG_YC	"0" //YD)是否使用连接指定WiFi网络功能0---不使用，由用户手动连接1---自动连接用户指定WiFi
#define ANDROID_CONFIG_YD	"0" //YD)是否使用状态栏音量控制按钮0---默认使用，1---不使用
#define ANDROID_CONFIG_YE	"1" //YE)WiFi休眠策略:0---屏幕关闭时休眠,1---充电时永不休眠,2---永不休眠
#define ANDROID_CONFIG_YF	"0"//YF)默认是否使用状态栏返回键0---使用，1---不使用(默认0)
#define ANDROID_CONFIG_YG	"1"//YG)是否显示本机硬件配置信息 0-不显示 1-显示2-只显示Flash内存总容量(默认0)

#define ANDROID_CONFIG_YH	"1500MHz"//YH)系统CPU最高主频率，需带上单位M

#define ANDROID_CONFIG_YI	"1"//YI)是否显示状态栏下拉菜单功能0---不使用，1---使用(默认0)
#define ANDROID_CONFIG_YJ	"100"//YJ)振动器振动时间(ms):默认100
#define ANDROID_CONFIG_YK	"4GB"//YK)系统Flash 内存总容量，单位GB
#define ANDROID_CONFIG_YL	"0"//YL)是否使用状态栏MENU菜单键,0---使用，1---不使用(默认值0)
#define ANDROID_CONFIG_YM	"0"//YM)是否使用状态栏HOME键,0---使用，1---不使用(默认值0)
#define ANDROID_CONFIG_YN	"0"//YN)是否显示ZP)Wisky内部内核版本号，0---显示，1-不显示(默认0)
#define ANDROID_CONFIG_YO	"1"//YO)是否显示设置菜单中内置3G 模块电源控制0-不显示，1-显示(默认0)
#define ANDROID_CONFIG_YP	"0"//YP)横屏时壁纸滑动步阶,使用方式为(oem+9)/9,oem=0 则不滑动 其余的值可以随便设 但最好为不超过10的整数
#define ANDROID_CONFIG_YQ	"2"//YQ)GoogleMarket应用，0-显示应用无设置，1-不显示应用有设置,2-显示应用有设置
#define ANDROID_CONFIG_YR	"1"//YR)是否使用多指触摸功能，0---不使用，1---使用
#define ANDROID_CONFIG_YS	"0"//YS)关机菜单提示是否使用单个按键，0---不使用(默认)，1---使用
#define ANDROID_CONFIG_YT	"1"/*YT)机器通过USB连接电脑是否直接挂载存储设备到电脑
								0：不自动挂载且不弹出操作界面
								1、弹出操作界面但不自动连接
								2、弹出操作界面且自动连接
								3、自动连接但不弹出操作界面*/
#define ANDROID_CONFIG_YU	"1"//YU)桌面切换工作区时是否显示区域框，0---不显示，1---显示(默认0)
#if defined(WISKY_WIFI_BT_BCM4329)
#define ANDROID_CONFIG_YV	"BCM4329"//YV)WiFi 模块型号标志,目前有: BCM4329(BC),RTL8192(RT)
#elif defined(WISKY_WIFI_RTL8192C)||defined(WISKY_WIFI_RTL8192C_EMI)
#define ANDROID_CONFIG_YV	"RTL8192C"//YV)WiFi 模块型号标志,目前有: BCM4329(BC),RTL8192(RT)
#else
#define ANDROID_CONFIG_YV	"NULL"//YV)WiFi 模块型号标志,目前有: BCM4329(BC),RTL8192(RT)
#endif
#define ANDROID_CONFIG_YW	WISKY_HW_VERSION//YW) 产品的硬件版本型号
#define ANDROID_CONFIG_YX	"0"//YX)
#define ANDROID_CONFIG_YY	"/flash/auto_install_apk_dir/"//YY)开机自安装客户APK 放置路径
#define ANDROID_CONFIG_YZ	"1"//YZ)安装完"YY"中的应用后是否删除原文件，0---不删除，1--删除

#define ANDROID_CONFIG_X1	"6"//X1)音乐和视频桌面控件的位置，0---无控件，1至9表示放置在第几屏
#define ANDROID_CONFIG_X2	"7"//X2)电子书和图片桌面控件的位置，0---无控件，1至9表示放置在第几屏
#define ANDROID_CONFIG_X3	"0"//X3)除电源键外的按键唤醒功能:0---可以唤醒，1--禁止唤醒
#define ANDROID_CONFIG_X4	"1"//X4)状态栏下拉菜单按钮是否显示文字:0---不显示，1---显示(默认)
#define ANDROID_CONFIG_X5	"0"//X5)HDMI配置菜单中是否显示分辨率和缩放设置项:0--不显示(默认),1--显示
#define ANDROID_CONFIG_X6	"1"//X6)状态栏下拉菜单中是否含任务查杀:0--不含(默认)，1--含有
#define ANDROID_CONFIG_X7	"1"//X7)是否使用触摸按键灯:0---不使用，1---使用(默认)
#define ANDROID_CONFIG_X8	"0"//X8)是否屏蔽MENU键解锁功能:0---不屏蔽,1--屏蔽MENU解锁功能 
#define ANDROID_CONFIG_X9	"1111111111"//X9)Wisky自定义桌面控件是否显示在桌面添加菜单中，0--否，1--是
#define ANDROID_CONFIG_XA	"0"//XA)连接电脑后是否禁止按键退出连接对话框:0---允许退出,1--禁止退出
/*Android键盘输入法语言列表:
English (US) : en_US : -921088104
 English (UK) : en_GB : -1337596075
 Arabic : ar : 1494081088
 Czech : cs : 758984400
 Danish : da : 770990173
 German : de : 774684257
 German QWERTY : de : 1383867817
 Spanish : es : 816242702
 Finnish : fi : 835636643
 French : fr : 843948332
 French (Canada) : fr_CA : -354699631
 French (Switzerland) : fr_CH : -348234984
 Croatian : hr : 901206634
 Hungarian : hu : 903977197
 Italian : it : 931682827
 Hebrew : iw : 1727731901
 Norwegian Bokmal: 1058205204
 Dutch : nl : 1067440414
 Polish : pl : 1124698716
 Portuguese : pt : 1132086884
 Russian : ru : 1983547218
 Serbian : sr : 2009405806
 Swedish : sv : 1219821379
 Turkish : tr : 1244756446
 (XB 列表格式为"-921088104;774684257;903977197")
*/
#define ANDROID_CONFIG_XB	"0"//XB)Android键盘语言列表(十位整数值)，0---使用系统默认值，其他需选择加';'间隔,列表第一个为默认选定值
#define ANDROID_CONFIG_XC	"0"//XC)Android键盘默认输入语言
#define ANDROID_CONFIG_XD	"0"//XD)关机菜单是否加入reboot复位项，0---不加，1---加
#define ANDROID_CONFIG_XE	"1"//XD)是否去掉自动更新时间选项，0---去掉，1---加
#define ANDROID_CONFIG_XF	"0"//XF)3G 连接信号指示0---一直显示1---不使用3G时不显示
#define ANDROID_CONFIG_XG	"0"//XG)PC 是否加入套件0----不加，1----加
#define ANDROID_CONFIG_XH	"0"//XH)键盘映射功能，0---不加，1---加
#define ANDROID_CONFIG_XI	"0"//XI)Camera摄像头APK应用界面菜单是否对掉适应M821: 0---否(原始) 1--是
#define ANDROID_CONFIG_XJ	"0"//XJ)Recoder 录音机指针摆动角度放到倍数:0和1为不放大, 大于1则放大相应倍数
#define ANDROID_CONFIG_XK	"1"//XK)//无线网络设置中USB & WiFi 热点是否隐藏:0---隐藏,1---显示
#define ANDROID_CONFIG_XL	"0"//XL//是否在相机应用中增加返回键:0-->不增加,1--->增加
#define ANDROID_CONFIG_XM	"1"//XM//DC充电时是否禁止系统深度休眠:0-->可以深度休眠(默认),1-->禁止深度休眠
#define ANDROID_CONFIG_XN	"0"//XN//拔除HDMI线是否返回到主界面(解决M821视频反问题):0-->不返回,1-->返回到主界面
#define ANDROID_CONFIG_XO	"0"//XO//按电源键是否休眠、是否弹出10s对话框:0--不提示直接休眠(默认),1--提示休眠对话框
#define ANDROID_CONFIG_XP	"0"//XP//Launcher Market 快捷功能:0--系统原始1--APPCenter, 2---隐藏
#define ANDROID_CONFIG_XQ	"1"//XQ//设置菜单中的系统更新项:0--显示, 1--隐藏
#define ANDROID_CONFIG_XR	"1"//XR//3G上网卡和USB以太网支持列表:0--隐藏(默认),1--显示
#define ANDROID_CONFIG_XS	"1.0"//XS//默认字体大小:0.85(小),1.0(正常),1.15(大),1.30(巨大)
#define ANDROID_CONFIG_XT	"0"//XT//默认打开网页的浏览器:0---系统默认方式, 1--UC浏览器
#define ANDROID_CONFIG_XU	"0"//XU//开发选项中的"开发设备ID"是否隐藏:0---系统默认为显示,1---隐藏
#define ANDROID_CONFIG_XV	"0"//XV//是否隐藏图库MENU菜单中'允许离线使用':0--系统默认不隐藏,1--隐藏
#define ANDROID_CONFIG_XW     "0"//XW）是否显示‘客户名称  ’菜单项:0--隐藏(默认)，1--显示
#define ANDROID_CONFIG_XX     "0"//XX)关闭升级格式化用户盘:0--默认格式化,1---不格式化
#define ANDROID_CONFIG_XY	"0"//XY)浏览器用户代理:0--系统默认,1--客户定制
#define ANDROID_CONFIG_XZ "0" //是否显示以太网设置：0--显示，1--隐藏

//其他内核使用的配置
#define WISKY_LOGO_LINUX	1	//开机内核显示Logo

#endif
#endif
