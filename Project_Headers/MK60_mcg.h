/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,野火科技
 *     All rights reserved.
 *     技术讨论：野火初学论坛 http://www.chuxue123.com
 *
 *     除注明出处外，以下所有内容版权均属野火科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留野火科技的版权声明。
 *
 * @file       MK60_mcg.h
 * @brief      MCG PLL驱动
 * @author     野火科技
 * @version    v5.0
 * @date       2013-06-29
 */


#ifndef __MK60_MCG_H__
#define __MK60_MCG_H__

#define MK60DZ10                1
#define EXTAL_IN_MHz            50
#define CORE_CLK                PLL150      // 从 PLL_e 里选择 已提供的 配置方案
                                            // bus 、 flex bus 、 flash 频率 都是 core 整数倍分频而来
#define MAX_BUS_CLK             50         // bus      (bus        >= core/16  )
#define MAX_FLEXBUS_CLK         50          // flex bus (flex bus   >= core/16  )
#define MAX_FLASH_CLK           25          // flash    (flash      >= core/16  )

/*********************   自定义 时钟频率 分频因子   ********************/
//如果 CORE_CLK 为 PLLUSR ，则为自定义模式 ，如下配置生效
//如果是 F15 系列： MCG_CLK_MHZ = 50u*(VDIV+16)/(PRDIV+1)/2
//如果是 DZ10 系列：MCG_CLK_MHZ = 50u/*(VDIV+24)(PRDIV+1)
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
    PLLUSR      ,  //自定义设置分频系数模式，直接加载 全局变量 mcg_div 的值
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
    PLLUSR      ,  //自定义设置分频系数模式，直接加载 全局变量 mcg_div 的值
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
    PLL275      ,             //K60FX512LQV15 ,实测最大 275M
    PLL300      ,

    //由于我们函数的时钟是已MHz为单位，不支持小数。
    //所以需要其他频率的朋友，请用自定义分频系数的方法，并自行修改代码

    PLL_MAX,
} PLL_e;
#endif



typedef struct
{
    unsigned int  clk;         //
    unsigned char   prdiv;       //外部晶振分频因子选项
    unsigned char   vdiv;        //外部晶振倍频因子选项
} mcg_cfg_t;

//时钟分频因子
typedef struct
{
    unsigned char core_div;    //内核时钟分频因子
    unsigned char bus_div;     //总线时钟分频因子
    unsigned char flex_div;    //flex时钟分频因子
    unsigned char flash_div;   //flash时钟分频因子
} mcg_div_t;


uint8 pll_init(PLL_e pll);

 void set_sys_dividers(uint32 outdiv1, uint32 outdiv2, uint32 outdiv3, uint32 outdiv4);
 


/********************************************************************/
#endif /* __MK60_MCG_H__ */
