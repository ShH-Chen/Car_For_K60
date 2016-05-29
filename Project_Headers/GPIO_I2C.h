/*
 * GPIO_I2C.h
 *
 *  Created on: Mar 1, 2014
 *      Author: sheng
 */

#ifndef GPIO_I2C_H_
#define GPIO_I2C_H_


//SCCB 管脚配置
#define SCCB_SCL        PTA26
#define SCCB_SDA        PTA25


#define SCL_H()         I2CSCL_SetVal()  //PTXn_T(SCCB_SCL,OUT) = 1
#define SCL_L()         I2CSCL_ClrVal() //PTXn_T(SCCB_SCL,OUT) = 0
#define SCL_DDR_OUT()   I2CSCL_SetOutput()  //PTXn_T(SCCB_SCL,DDR) = 1
#define SCL_DDR_IN()    I2CSCL_SetInput()   //PTXn_T(SCCB_SCL,DDR) = 0

#define SDA_H()         I2CSDA_SetVal()    //PTXn_T(SCCB_SDA,OUT) = 1
#define SDA_L()         I2CSDA_ClrVal() //PTXn_T(SCCB_SDA,OUT) = 0
#define SDA_IN()        I2CSDA_GetVal() //PTXn_T(SCCB_SDA,IN)
#define SDA_DDR_OUT()   I2CSDA_SetOutput()  //PTXn_T(SCCB_SDA,DDR) = 1
#define SDA_DDR_IN()    I2CSDA_SetInput()  //PTXn_T(SCCB_SDA,DDR) = 0

#define ADDR_OV7725   0x42
#define ADDR_OV7620   0x42

#define DEV_ADR  ADDR_OV7725             /*设备地址定义*/

#define SCCB_DELAY()    SCCB_delay(400)


extern void SCCB_GPIO_init(void);
extern int SCCB_WriteByte( unsigned short WriteAddress , unsigned char SendByte);
extern int SCCB_ReadByte(unsigned char *pBuffer,   unsigned short length,   unsigned char ReadAddress);


#endif /* GPIO_I2C_H_ */
