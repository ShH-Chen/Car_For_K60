/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,Ұ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�Ұ���ѧ��̳ http://www.chuxue123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����Ұ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��Ұ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_mcg.h
 * @brief      MCG PLL����
 * @author     Ұ��Ƽ�
 * @version    v5.0
 * @date       2013-06-29
 */


#ifndef __MK60_MCG_H__
#define __MK60_MCG_H__

#define MK60DZ10                1
#define EXTAL_IN_MHz            50
#define CORE_CLK                PLL150      // �� PLL_e ��ѡ�� ���ṩ�� ���÷���
                                            // bus �� flex bus �� flash Ƶ�� ���� core ��������Ƶ����
#define MAX_BUS_CLK             50         // bus      (bus        >= core/16  )
#define MAX_FLEXBUS_CLK         50          // flex bus (flex bus   >= core/16  )
#define MAX_FLASH_CLK           25          // flash    (flash      >= core/16  )

/*********************   �Զ��� ʱ��Ƶ�� ��Ƶ����   ********************/
//��� CORE_CLK Ϊ PLLUSR ����Ϊ�Զ���ģʽ ������������Ч
//����� F15 ϵ�У� MCG_CLK_MHZ = 50u*(VDIV+16)/(PRDIV+1)/2
//����� DZ10 ϵ�У�MCG_CLK_MHZ = 50u/*(VDIV+24)(PRDIV+1)
#define PRDIV             16         //n
#define VDIV              24          //VDIV+24
#define CORE_DIV          0         //  core = mcg/ ( CORE_DIV  + 1 )
#define BUS_DIV           2         //  bus  = mcg/ ( BUS_DIV   + 1 )
#define FLEX_DIV          5         //  flex = mcg/ ( FLEX_DIV  + 1 )
#define FLASH_DIV         5         //  flash= mcg/ ( FLASH_DIV + 1 )

typedef unsigned char       uint8;  /*  8 bits */
typedef unsigned short int  uint16; /* 16 bits */
typedef unsigned long int   uint32; /* 32 bits */
typedef unsigned long long  uint64; /* 64 bits */ 
/********************************************************************/
#if defined(MK60DZ10)
typedef enum
{
    PLLUSR      ,  //�Զ������÷�Ƶϵ��ģʽ��ֱ�Ӽ��� ȫ�ֱ��� mcg_div ��ֵ
    PLL48,
    PLL50,
    PLL96,
    PLL100,
    PLL110,
    PLL120,
    PLL125,
    PLL130,
    PLL140,
    PLL144,
    PLL150,
    PLL160,
    PLL170,
    PLL180,
    PLL200,
    PLL225,
    PLL250,

    PLL_MAX,
} PLL_e;

#elif defined(MK60F15)
typedef enum
{
    PLLUSR      ,  //�Զ������÷�Ƶϵ��ģʽ��ֱ�Ӽ��� ȫ�ֱ��� mcg_div ��ֵ
    PLL50       ,
    PLL100      ,
    PLL110      ,
    PLL120      ,
    PLL130      ,
    PLL140      ,
    PLL150      ,
    PLL160      ,
    PLL170      ,
    PLL180      ,
    PLL190      ,
    PLL200      ,
    PLL210      ,
    PLL220      ,
    PLL225      ,
    PLL230      ,
    PLL235      ,
    PLL244      ,
    PLL250      ,
    PLL275      ,             //K60FX512LQV15 ,ʵ����� 275M
    PLL300      ,

    //�������Ǻ�����ʱ������MHzΪ��λ����֧��С����
    //������Ҫ����Ƶ�ʵ����ѣ������Զ����Ƶϵ���ķ������������޸Ĵ���

    PLL_MAX,
} PLL_e;
#endif



typedef struct
{
    unsigned int  clk;         //
    unsigned char   prdiv;       //�ⲿ�����Ƶ����ѡ��
    unsigned char   vdiv;        //�ⲿ����Ƶ����ѡ��
} mcg_cfg_t;

//ʱ�ӷ�Ƶ����
typedef struct
{
    unsigned char core_div;    //�ں�ʱ�ӷ�Ƶ����
    unsigned char bus_div;     //����ʱ�ӷ�Ƶ����
    unsigned char flex_div;    //flexʱ�ӷ�Ƶ����
    unsigned char flash_div;   //flashʱ�ӷ�Ƶ����
} mcg_div_t;


uint8 pll_init(PLL_e pll);

 void set_sys_dividers(uint32 outdiv1, uint32 outdiv2, uint32 outdiv3, uint32 outdiv4);
 


/********************************************************************/
#endif /* __MK60_MCG_H__ */
