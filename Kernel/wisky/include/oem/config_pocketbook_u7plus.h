/*
0�����еĶ�����������ַ����ʾ "XX",ǰһ����ʾ���𣬺�һ����ʾ����
1���ַ� A ~ Z �ֱ���ʾ�˲�ͬ�ĺ��� ���������µ��������壬��ѡ��һ��û��ʹ�õ��ַ����ʾ
2����һ�������������µĿ��أ���ʹ�����ֺ���ĸ�����ϣ����Ҳ�Ҫ�ظ����������뱨�� 1,2,3,4,5,6,7,8,9 ,A,B,C,D,....X,Y,Z
3) ��һ�����������ӵ��µĿ��ز�Ҫ�������������м䣬һ�ɲ����ڸ�������ĩβλ��
4��������Ҫ���������ò˵��޹ص�ѡ�һ�ɼ��������������У�����ʼ�ַ�Ϊ "Z"�������У����Ҽ���ĩβ��λ��

���ñ���˵��:

A ���ߺ���������
B ͨ������
C ����
D ��ʾ
E λ�úͰ�ȫ
F Ӧ�ó���
G �ʻ���ͬ��
H ��˽Ȩ
I �洢
J ���Ժͼ���
K ��������������
L ��������
M ���ں�ʱ��
N �����ֻ�
Z ��������
*/

#if defined(WISKY_OEM_POCKETBOOK_U7PLUS)

#ifndef _CONFIG_POCKETBOOK_U7PLUS_H_
#define _CONFIG_POCKETBOOK_U7PLUS_H_


#define MAX_CONFIG_BUFFLEN     64//���BUFFER����

//A ���ߺ���������
#define ANDROID_CONFIG_A1     "0"//A1������ģʽ 0-�ر� 1-������Ĭ��0��
#define ANDROID_CONFIG_A2     "0"//A2��WIFI 0-�ر� 1-������Ĭ��0��
#define ANDROID_CONFIG_A3     "0"//A3����� 0-�ر� 1-������Ĭ��0��
#define ANDROID_CONFIG_A4     "0"//A4) ��̫��0-�ر�,1-����(Ĭ��0)


//B ͨ������


//C ����
#define ANDROID_CONFIG_C1      "0"//C1������ģʽ 0-�ر� 1-������Ĭ��0��
#define ANDROID_CONFIG_C2      "3"//C2������ 0-ʼ�� 1-һ�ɲ� 2-���ھ���ģʽ��������� 4-���ڷǾ���ģʽ������񶯣�Ĭ��3��
#define ANDROID_CONFIG_C3      "70"//C3����� -�������  0~100�İٷֱȱ�ʾ ��Ĭ�� 70��
#define ANDROID_CONFIG_C4      "70"//C4����� -ý�����  0~100�İٷֱȱ�ʾ ��Ĭ�� 70��
#define ANDROID_CONFIG_C5      "70"//C5����� ---�������  0~100�İٷֱȱ�ʾ ��Ĭ�� 70��
#define ANDROID_CONFIG_C6      "1"//C6�����--������������֪ͨ��� 0-�ر� 1-���� ��Ĭ��1��
#define ANDROID_CONFIG_C7      "1"//C7���ֻ�����  ֱ�����������ʾ ��Ĭ��1��0-���� 1-BeatPlucker 2-BentleyDubs 3-BirdLoop ����..
#define ANDROID_CONFIG_C8      "12"//C8��֪ͨ����  ֱ�����������ʾ ��Ĭ��12��0-���� 1-Beat_Box_Android 2-CaffeineSnake 3-DearDeer ����.. 
#define ANDROID_CONFIG_C9      "1"//C9������������ 0-�ر� 1-������Ĭ��1��
#define ANDROID_CONFIG_CA      "0"//CA��ѡ�������� 0-�ر� 1-������Ĭ��0��
#define ANDROID_CONFIG_CB      "0"//CB����Ļ������ʾ�� 0-�ر� 1-������Ĭ��0��
#define ANDROID_CONFIG_CC      "1"//CC������  0-�ر� 1-������Ĭ��1��

//D ��ʾ
#define ANDROID_CONFIG_D1      "70"//D1�����  0~100�İٷֱȱ�ʾ
#define ANDROID_CONFIG_D2      "1"//D2���Զ���ת��Ļ 0-�ر� 1-������Ĭ��1��
#define ANDROID_CONFIG_D3      "2"//D3������  0-�޶���  1-���ֶ��� 2-���ж��� ��Ĭ��2��
#define ANDROID_CONFIG_D4      "2"//D4����Ļ��ʱ  ֱ�����������ʾ ��Ĭ��2�� 0-15�� 1-30�� 2-1���� 3-2����

//E λ�úͰ�ȫ
#define ANDROID_CONFIG_E1      "0"//E1��ʹ���������� 0-�ر� 1 -������Ĭ��0��
#define ANDROID_CONFIG_E2      "1"//E2��ʹ��GPS���� 0-�ر� 1 -������Ĭ��1��
#define ANDROID_CONFIG_E5      "1"//E5�������ɼ� 0-�ر� 1-������Ĭ��1��
#define ANDROID_CONFIG_E7      "0"//E7��ʹ�ð�ȫƾ֤���� 0-�ر�1-���� ��Ĭ��0��

//F Ӧ�ó���
#define ANDROID_CONFIG_F1      "0"//F1��δ֪�Դ 0-�ر� 1-������Ĭ��0��
#define ANDROID_CONFIG_F6      "0"//F6��USB���� 0-�ر� 1-���� ��Ĭ��1��
#define ANDROID_CONFIG_F7      "0"//F7�����ֻ���״̬ 0--�ر� 1-���� ��Ĭ��0��
#define ANDROID_CONFIG_F8      "1"//F8������ģ���ص� 0-�ر� 1-���� ��Ĭ��1��

//G �˻���ͬ��
#define ANDROID_CONFIG_G1      "1"//G1���������� 0-�ر� 1-������Ĭ��1��
#define ANDROID_CONFIG_G2      "1"//G2���Զ�ͬ�� 0-�ر� 1-������Ĭ��1��

//H ��˽Ȩ

//I �洢
#define ANDROID_CONFIG_I1      "2"//I1���洢�豸USB��ӷ�ʽ:0---MTP,1--PTP,2---UMS��Ĭ��2��

//J ���Ժͼ���
//�������Զ�Ӧ����
//da_DK  ������  ����  
//DE_AT  ����  �µ���  
//DE_CH  ����  ��ʿ  
//DE_DE  ����  �¹�  
//el_GR  ϣ���  ϣ�  
//en_CA  Ӣ��  ���ô�  
//en_GB  Ӣ��  �������  
//en_IE  Ӣ��  �����  
//en_US  Ӣ��  ���  
//es_ES  ��������  ������  
//fi_FI  �����  ���  
//fr_BE  ����  ����ʱ  
//fr_CA  ����  ���ô�  
//fr_CH  ����  ��ʿ  
//fr_FR  ����  ����  
//it_CH  ��������  ��ʿ  
//it_IT  ��������  ������  
//ja_JP  ����  �ձ�  
//ko_KR  ������  ����  
//nl_BE  �����  ����ʱ  
//nl_NL  �����  ���  
//no_NO  Ų���� (Nynorsk)  Ų��  
//no_NO_B  Ų���� (Bokm?l)  Ų��  
//pt_PT  ��������  ������  
//sv_SE  ������  ����  
//tr_TR  �������  �����  
//zh_CN  ���ģ����壩  �й�  
//zh_TW  ���ģ����壩  �й�̨��
#define ANDROID_CONFIG_J1     "en_US"//J1��ѡ������ ��Ĭ�ϼ������ģ�
#define ANDROID_CONFIG_J2     "0"//J2��֧�������б�0---ȫ�����ԣ�������ѡ����'@'����

//K ��������������

//L ��������
#define ANDROID_CONFIG_L1     "0"//L1���������� 0-�ر� 1-������Ĭ������0��
#define ANDROID_CONFIG_L2     "0"//L2����"��Դ"��ť����ͨ�� 0-�ر� 1-������Ĭ������0��


//M ���ں�ʱ������
#define ANDROID_CONFIG_M1     "1"//M1���Զ�   0-�ر� 1-������Ĭ������1��
#define ANDROID_CONFIG_M5     "0"//M5��ʹ��24Сʱ��ʽ   0-�ر� 1-������Ĭ������0��
#define ANDROID_CONFIG_M6     "0"//M6��ѡ�����ڸ�ʽ   ��Ĭ��0�� 0-��ͨ��2011-12-31��1-12-31-2011  2-31-12-2011  3-2011-12-31


//N �����ֻ�

//Z ��������
#define ANDROID_CONFIG_Z1     "0"//Z1���Ƿ�֧�ֺ�̨��ʱ������ɱ 0-�ر� 1-���� ��Ĭ��0��
#define ANDROID_CONFIG_Z2     "0"//Z2������ʱ���ص������ʾģʽ 0 --��ʾ���ģʽ 1 --����ʾ�����������ʾ���ڳ�����״̬ ��Ĭ��0��
#define ANDROID_CONFIG_Z3     "1"//Z3�����ζ����Ƿ�����ʾ�� 0-- ����ʾ����1--����ʾ����Ϣ ��Ĭ��0��
#define ANDROID_CONFIG_Z4     "1"//Z4������ʱ�Ƿ���˸������ 0--����˸ ��1 --������˸ ��Ĭ��0��
#define ANDROID_CONFIG_Z5     "1"//Z5��USB��Ӻ������ݿ�������ʱ������˸������ 0--����˸ ��1--������˸ ��Ĭ��0��
#define ANDROID_CONFIG_Z6     "1"//Z6���Ƿ����ó���ģʽ�����ػ�״̬�²��������������ֱ�ӳ��� 0--�����ã�1--���� ��Ĭ��0��
#define ANDROID_CONFIG_Z7     "0"//Z7��GPS�ڿ���ʱ�Ƿ���ʾ�رնԻ��� 0--����ʾ 1-- ��ʾ ��Ĭ��0��
#define ANDROID_CONFIG_Z8     "1"//Z8������ESC�Ƿ�����GSENSOR���� 0--���� 1-- �ӣ������ü�����GSENSOR ��Ĭ��0��
#define ANDROID_CONFIG_Z9     "/flash"//Z9�����̵�·��  ���ַ�����Ϣ����Ҫ��SD����NAND��·�����ƣ�
#define ANDROID_CONFIG_ZA     "android"//ZA�����̵�����  ���ͻ���Ҫ���ҵĵ�������ʾ�Ĵ��������ַ�����Ϣ��
#define ANDROID_CONFIG_ZB     "android"//ZB��USB��Ϣ ���ͻ���Ҫ��USB��������ʾ��Ϣ��
#define ANDROID_CONFIG_ZC     "android"//ZC���ͻ�����  ���ַ�����Ϣ��
#define ANDROID_CONFIG_ZD     WISKY_HW_VERSION //ZD���ͻ��Ĳ�Ʒ�ͺ� ���ַ�����Ϣ��
#define ANDROID_CONFIG_ZE     "V1.00_20130424"//ZE���ͻ��İ汾�� ���ַ�����Ϣ��
#define ANDROID_CONFIG_ZF	"1"//ZF) ����ʱ��"ANDROID_"��˸�����Ƿ��ɼ�0--���ɼ�1--�ɼ�
#if defined(WISKY_TOUCHSCREEN_AK4183)|| defined(WISKY_TOUCHSCREEN_TCS9135) || defined(WISKY_TOUCHSCREEN_LZ300) \
	||defined(WISKY_TOUCHSCREEN_TCS9046)
#define ANDROID_CONFIG_ZG	"1"//ZG) �Ƿ�ʹ�ô�����У׼0--��ʹ��1--ʹ��
#else
#define ANDROID_CONFIG_ZG	"0"//ZG) �Ƿ�ʹ�ô�����У׼0--��ʹ��1--ʹ��
#endif
#define ANDROID_CONFIG_ZH	"xxxx"//ZH)	Ĭ�ϱ�ֽ��Ϊ��ֽͼƬ����
#define ANDROID_CONFIG_ZI	"0"//ZI)	���߻��Ѻ��Ƿ���ʾ����������� 0 -- ����ʾ��ԭ���ʲô״̬����ʲô״̬ 1-- ǿ��ת����������ʾ״̬�������Ĭ��0��
#define ANDROID_CONFIG_ZJ	"0"//ZJ)	�⴫��������Զ�����Ĭ�Ͽ���״̬0---�رգ�1---�򿪣�Ĭ��0��
#define ANDROID_CONFIG_ZK	"0"//ZK)	Ӳ����ʹ��ADC ������0---��ʹ��ADC ������1---ʹ��ADC������Ĭ��0��
#define ANDROID_CONFIG_ZL     "android"//ZL��ADB������ʾ��Ϣ ���ͻ���Ҫ��PC�豸�������п�����ADB���ƣ�

#if (WISKY_LCD_WIDTH == 800)
#define ANDROID_CONFIG_ZM	"800" //ZM) ��Ļ�ֱ��ʵĿ���(���ص�Ϊ��λ)
#elif(WISKY_LCD_WIDTH == 1024)
#define ANDROID_CONFIG_ZM	"1024" //ZM) ��Ļ�ֱ��ʵĿ���(���ص�Ϊ��λ)
#elif(WISKY_LCD_WIDTH == 1280)
#define ANDROID_CONFIG_ZM	"1280" //ZM) ��Ļ�ֱ��ʵĿ���(���ص�Ϊ��λ)
#else
#error No correct LCD width setup
#endif

#if (WISKY_LCD_HEIGHT == 480)
#define ANDROID_CONFIG_ZN	"480" //ZN) ��Ļ�ֱ��ʵĸ߶�(���ص�Ϊ��λ)
#elif(WISKY_LCD_HEIGHT == 600)
#define ANDROID_CONFIG_ZN	"600" //ZN) ��Ļ�ֱ��ʵĸ߶�(���ص�Ϊ��λ)
#elif(WISKY_LCD_HEIGHT == 768)
#define ANDROID_CONFIG_ZN	"768" //ZN) ��Ļ�ֱ��ʵĸ߶�(���ص�Ϊ��λ)
#elif(WISKY_LCD_HEIGHT == 800)
#define ANDROID_CONFIG_ZN	"800" //ZN) ��Ļ�ֱ��ʵĸ߶�(���ص�Ϊ��λ)
#elif(WISKY_LCD_HEIGHT == 1280)
#define ANDROID_CONFIG_ZN	"1280" //ZN) ��Ļ�ֱ��ʵĸ߶�(���ص�Ϊ��λ)
#else
#error No correct LCD height setup
#endif

#define ANDROID_CONFIG_ZO	"0"//ZO) �Ƿ�ʹ�ö�̬��ֽ0---��ʹ�ã�1---ʹ��
#define ANDROID_CONFIG_ZP	WISKY_KERNEL_VERSION //ZP)Wisky�ڲ��ں˰汾��

#define ANDROID_CONFIG_ZQ	"1" //ZQ)Ӳ�����Ƿ�ʹ��WiFiģ��0--��ʹ�ã�1--ʹ��
#if WISKY_ENABLE_BLUETOOTH
#define ANDROID_CONFIG_ZR	"1" //ZR)Ӳ�����Ƿ�ʹ��Bluetoothģ��0--��ʹ�ã�1--ʹ��
#else
#define ANDROID_CONFIG_ZR	"0" //ZR)Ӳ�����Ƿ�ʹ��Bluetoothģ��0--��ʹ�ã�1--ʹ��
#endif
#if WISKY_ENABLE_HDMI
#define ANDROID_CONFIG_ZS	"1" //ZS)Ӳ�����Ƿ�ʹ��HDMIģ��0--��ʹ�ã�1--ʹ��
#else
#define ANDROID_CONFIG_ZS	"0" //ZS)Ӳ�����Ƿ�ʹ��HDMIģ��0--��ʹ�ã�1--ʹ��
#endif
#define ANDROID_CONFIG_ZT	"0" //ZT)Ӳ�����Ƿ�ʹ��GPSģ��0--��ʹ�ã�1--ʹ��
#define ANDROID_CONFIG_ZU	"1" //ZU)Ӳ�����Ƿ�ʹ��3Gģ��0--��ʹ�ã�1--ʹ��
#define ANDROID_CONFIG_ZV	"1" //ZV)Ӳ�����Ƿ�ʹ��Cameraģ��0--��ʹ�ã�1--ʹ��
#if WISKY_ENABLE_VIBRATOR
#define ANDROID_CONFIG_ZW	"1" //ZW)Ӳ�����Ƿ�ʹ��Vibrator������ģ��0--��ʹ�ã�1--ʹ��
#else
#define ANDROID_CONFIG_ZW	"0" //ZW)Ӳ�����Ƿ�ʹ��Vibrator������ģ��0--��ʹ�ã�1--ʹ��
#endif
#define ANDROID_CONFIG_ZX	"0" //ZX)Ӳ�����Ƿ�ʹ��LightSensor�⴫����ģ��0--��ʹ�ã�1--ʹ��
#define ANDROID_CONFIG_ZY	"1" //ZY)Ӳ�����Ƿ�ʹ��G-Sensorģ��0--��ʹ�ã�1--ʹ��


#define ANDROID_CONFIG_ZZ	"0" //ZY)�Ƿ�ʹ��3Gͨ�����ܣ�0--��ʹ�ã�1--ʹ��
#define ANDROID_CONFIG_Y1	"0" //Y1)�Ƿ��������ò˵��еľ�������0--������ 1--����
#define ANDROID_CONFIG_Y2	"1" //Y2)�Ƿ��������ò˵��е���̫��������0--������ 1--����
#define ANDROID_CONFIG_Y3	"0" //Y3)Ĭ�����뷨�趨0-android���� 1-wisky���뷨 2-�ȸ�ƴ�����뷨
#define ANDROID_CONFIG_Y4	"0" //Y4)�Ƿ���ι����豸�еġ����ʹ��������0--����� 1--��� ��Ĭ��0��
#define ANDROID_CONFIG_Y5	"1" //Y5)�Ƿ�������������еġ�������ʾ���� 0--����� 1--��� ��Ĭ��0��
#define ANDROID_CONFIG_Y6	"http://www.google.com.hk/" //Y6)��������Ĭ����ҳ  ���ַ�����Ϣ����Ĭ�Ͽգ�
#define ANDROID_CONFIG_Y7	"0" //Y7)�Ƿ���֪ͨ�������ʱ����ʶ    0--������ 1--���� ��Ĭ��0��
#define ANDROID_CONFIG_Y8	"0" //Y8)������ʽѡ��:0---AndroidĬ�Ͻ�����1---Wisky������ʽ
#define ANDROID_CONFIG_Y9	"4" //Y9)����Ĭ����Ļ������0--�����1--�����2---������,3---������,����ֵΪ����ת
#define ANDROID_CONFIG_YA	"0" //YA)��ֹϵͳ��������˯��ģʽ 0--������1--��ֹ ��Ĭ��0��
#define ANDROID_CONFIG_YB	"0" //YB)�����Ƿ�ʹ���������0---����ʱ����һ�Σ�1---����ʱ�������
#define ANDROID_CONFIG_YC	"0" //YD)�Ƿ�ʹ�����ָ��WiFi���繦��0---��ʹ�ã����û��ֶ����1---�Զ�����û�ָ��WiFi
#define ANDROID_CONFIG_YD	"0" //YD)�Ƿ�ʹ��״̬�������ư�ť0---Ĭ��ʹ�ã�1---��ʹ��
#define ANDROID_CONFIG_YE	"0" //YE)WiFi���߲���:0---��Ļ�ر�ʱ����,1---����ʱ�������,2---�������
#define ANDROID_CONFIG_YF	"0"//YF)Ĭ���Ƿ�ʹ��״̬����ؼ�0---ʹ�ã�1---��ʹ��(Ĭ��0)
#define ANDROID_CONFIG_YG	"0"//YG)�Ƿ���ʾ����Ӳ��������Ϣ 0-����ʾ 1-��ʾ2-ֻ��ʾFlash�ڴ������(Ĭ��0)
#define ANDROID_CONFIG_YH	"1500MHz"//YH)ϵͳCPU������Ƶ�ʣ������ϵ�λM
#define ANDROID_CONFIG_YI	"1"//YI)�Ƿ���ʾ״̬�����˵�����0---��ʹ�ã�1---ʹ��(Ĭ��0)
#define ANDROID_CONFIG_YJ	"100"//YJ)����������ʱ��(ms):Ĭ��100
#define ANDROID_CONFIG_YK	"4GB"//YK)ϵͳFlash �ڴ����������λGB
#define ANDROID_CONFIG_YL	"0"//YL)�Ƿ�ʹ��״̬�MENU�˵���,0---ʹ�ã�1---��ʹ��(Ĭ��ֵ0)
#define ANDROID_CONFIG_YM	"0"//YM)�Ƿ�ʹ��״̬�HOME��,0---ʹ�ã�1---��ʹ��(Ĭ��ֵ0)
#define ANDROID_CONFIG_YN	"0"//YN)�Ƿ���ʾZP)Wisky�ڲ��ں˰汾�ţ�0---��ʾ��1-����ʾ(Ĭ��0)
#define ANDROID_CONFIG_YO	"0"//YO)�Ƿ���ʾ���ò˵�������3G ģ����Դ����0-����ʾ��1-��ʾ(Ĭ��0)
#define ANDROID_CONFIG_YP	"0"//YP)����ʱ��ֽ��������,ʹ�÷�ʽΪ(oem+9)/9,oem=0 �򲻻��� ������ֵ���������� ������Ϊ������10������
#define ANDROID_CONFIG_YQ	"0"//YQ)GoogleMarketӦ�ã�0-��ʾӦ�������ã�1-����ʾӦ��������,2-��ʾӦ��������
#define ANDROID_CONFIG_YR	"1"//YR)�Ƿ�ʹ�ö�ָ�������ܣ�0---��ʹ�ã�1---ʹ��
#define ANDROID_CONFIG_YS	"0"//YS)�ػ��˵���ʾ�Ƿ�ʹ�õ������0---��ʹ��(Ĭ��)��1---ʹ��
#define ANDROID_CONFIG_YT	"1"/*YT)����ͨ��USB��ӵ����Ƿ�ֱ�ӹ��ش洢�豸������
								0�����Զ������Ҳ�������������
								1�������������浫���Զ����
								2�����������������Զ����
								3���Զ���ӵ���������������*/
#define ANDROID_CONFIG_YU	"0"//YU)�����л�������ʱ�Ƿ���ʾ��������0---����ʾ��1---��ʾ(Ĭ��0)
#if defined(WISKY_WIFI_BT_BCM4329)
#define ANDROID_CONFIG_YV	"BCM4329"//YV)WiFi ģ���ͺű�־,Ŀǰ��: BCM4329(BC),RTL8192(RT)
#elif defined(WISKY_WIFI_RTL8192C)||defined(WISKY_WIFI_RTL8192C_EMI)
#define ANDROID_CONFIG_YV	"RTL8192C"//YV)WiFi ģ���ͺű�־,Ŀǰ��: BCM4329(BC),RTL8192(RT)
#else
#define ANDROID_CONFIG_YV	"NULL"//YV)WiFi ģ���ͺű�־,Ŀǰ��: BCM4329(BC),RTL8192(RT)
#endif
#define ANDROID_CONFIG_YW	WISKY_HW_VERSION//YW) ��Ʒ��Ӳ���汾�ͺ�
#define ANDROID_CONFIG_YX	"0"//YX)
#define ANDROID_CONFIG_YY	"/flash/auto_install_apk_dir/"//YY)�����԰�װ�ͻ�APK ����·��
#define ANDROID_CONFIG_YZ	"1"//YZ)��װ��"YY"�е�Ӧ�ú��Ƿ�ɾ��ԭ�ļ���0---��ɾ����1--ɾ��

#define ANDROID_CONFIG_X1	"6"//X1)���ֺ���Ƶ�����ؼ���λ�ã�0---�޿ؼ���1��9��ʾ�����ڵڼ���
#define ANDROID_CONFIG_X2	"7"//X2)��������ͼƬ�����ؼ���λ�ã�0---�޿ؼ���1��9��ʾ�����ڵڼ���
#define ANDROID_CONFIG_X3	"0"//X3)����Դ�����İ������ѹ���:0---���Ի��ѣ�1--��ֹ����
#define ANDROID_CONFIG_X4	"1"//X4)״̬�����˵���ť�Ƿ���ʾ����:0---����ʾ��1---��ʾ(Ĭ��)
#define ANDROID_CONFIG_X5	"0"//X5)HDMI���ò˵����Ƿ���ʾ�ֱ��ʺ�����������:0--����ʾ(Ĭ��),1--��ʾ
#define ANDROID_CONFIG_X6	"0"//X6)״̬�����˵����Ƿ���������ɱ:0--����(Ĭ��)��1--����
#define ANDROID_CONFIG_X7	"1"//X7)�Ƿ�ʹ�ô���������:0---��ʹ�ã�1---ʹ��(Ĭ��)
#define ANDROID_CONFIG_X8	"0"//X8)�Ƿ����MENU����������:0---�����,1--���MENU�������� 
#define ANDROID_CONFIG_X9	"0000000000"//X9)Wisky�Զ��������ؼ��Ƿ���ʾ���������Ӳ˵��У�0--����1--��
#define ANDROID_CONFIG_XA	"0"//XA)��ӵ��Ժ��Ƿ���ֹ�����˳���ӶԻ���:0---�����˳�,1--��ֹ�˳�
/*Android�������뷨�����б�:
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
 (XB �б���ʽΪ"-921088104;774684257;903977197")
*/
#define ANDROID_CONFIG_XB	"0"//XB)Android���������б�(ʮλ����ֵ)��0---ʹ��ϵͳĬ��ֵ��������ѡ����';'����,�б���һ��ΪĬ��ѡ��ֵ
#define ANDROID_CONFIG_XC	"0"//XC)Android����Ĭ����������
#define ANDROID_CONFIG_XD	"0"//XD)�ػ��˵��Ƿ�����reboot��λ�0---���ӣ�1---��
#define ANDROID_CONFIG_XE	"1"//XD)�Ƿ�ȥ���Զ�����ʱ��ѡ�0---ȥ����1---��
#define ANDROID_CONFIG_XF	"0"//XF)3G ����ź�ָʾ0---һֱ��ʾ1---��ʹ��3Gʱ����ʾ
#define ANDROID_CONFIG_XG	"0"//XG)PC �Ƿ������׼�0----���ӣ�1----��
#define ANDROID_CONFIG_XH	"0"//XH)����ӳ�书�ܣ�0---���ӣ�1---��
#define ANDROID_CONFIG_XI	"0"//XI)Camera����ͷAPKӦ�ý����˵��Ƿ��Ե���ӦM821: 0---��(ԭʼ) 1--��
#define ANDROID_CONFIG_XJ	"0"//XJ)Recoder ¼����ָ���ڶ��Ƕȷŵ�����:0��1Ϊ���Ŵ�, ����1���Ŵ���Ӧ����
#define ANDROID_CONFIG_XK	"1"//XK)//��������������USB & WiFi �ȵ��Ƿ�����:0---����,1---��ʾ
#define ANDROID_CONFIG_XL	"0"//XL//�Ƿ�������Ӧ�������ӷ��ؼ�:0-->������,1--->����
#define ANDROID_CONFIG_XM	"1"//XM//DC����ʱ�Ƿ���ֹϵͳ��������:0-->������������(Ĭ��),1-->��ֹ��������
#define ANDROID_CONFIG_XN	"0"//XN//�γ�HDMI���Ƿ񷵻ص�������(����M821��Ƶ������):0-->������,1-->���ص�������
#define ANDROID_CONFIG_XO	"0"//XO//����Դ���Ƿ����ߡ��Ƿ񵯳�10s�Ի���:0--����ʾֱ������(Ĭ��),1--��ʾ���߶Ի���
#define ANDROID_CONFIG_XP	"0"//XP//Launcher Market ���ݹ���:0--ϵͳԭʼ1--APPCenter, 2---����
#define ANDROID_CONFIG_XQ	"1"//XQ//���ò˵��е�ϵͳ������:0--��ʾ, 1--����
#define ANDROID_CONFIG_XR	"0"//XR//3G��������USB��̫��֧���б�:0--����(Ĭ��),1--��ʾ
#define ANDROID_CONFIG_XS	"1.0"//XS//Ĭ��������С:0.85(С),1.0(����),1.15(��),1.30(�޴�)
#define ANDROID_CONFIG_XT	"0"//XT//Ĭ�ϴ�����ҳ��������:0---ϵͳĬ�Ϸ�ʽ, 1--UC������
#define ANDROID_CONFIG_XU	"0"//XU//����ѡ���е�"�����豸ID"�Ƿ�����:0---ϵͳĬ��Ϊ��ʾ,1---����
#define ANDROID_CONFIG_XV	"0"//XV//�Ƿ�����ͼ��MENU�˵���'��������ʹ��':0--ϵͳĬ�ϲ�����,1--����
#define ANDROID_CONFIG_XW     "0"//XW���Ƿ���ʾ���ͻ�����  ���˵���:0--����(Ĭ��)��1--��ʾ
#define ANDROID_CONFIG_XX     "0"//XX)�ر�������ʽ���û���:0--Ĭ�ϸ�ʽ��,1---����ʽ��
#define ANDROID_CONFIG_XY	"0"//XY)�������û�����:0--ϵͳĬ��,1--�ͻ�����
#define ANDROID_CONFIG_XZ "0" //�Ƿ���ʾ��̫�����ã�0--��ʾ��1--����

#define ANDROID_CONFIG_W1 "0" //�Ƿ�ȫ����ʾ 0--��ȫ�� 1--ȫ��
#define ANDROID_CONFIG_W2 "0" //�������ؼ�������� 0--�� 1--��
#define ANDROID_CONFIG_W3 "0" //�Ƿ���ʾ����ģʽ�˵� 0--�� 1--��
#define ANDROID_CONFIG_W4 "1" //0--�������ؼ�����ӣ������˵�������� 1--�������ؼ����������˵��������
//�����ں�ʹ�õ�����
#define WISKY_LOGO_LINUX		1	//�����ں���ʾLogo
//#define  WISKY_LOGO_POCKETBOOK_U7PLUS_800X1280	1

#endif
#endif

