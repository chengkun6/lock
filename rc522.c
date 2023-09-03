
/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-10     35166       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <drv_spi.h>
#include <board.h>
#include "RC522.h"
#include <drv_common.h>
#include <rtdbg.h>
#define DBG_TAG "RC522"
#define DBG_LVL DBG_LOG

//#define LED0_PIN    10


#define BME280_SPI_DEVICE_NAME "spi10"
#define BEM280_REG_ID 0XD0
#define get

extern rt_sem_t sem_i_rc;//信号量

struct Led_s
{
    uint8_t LED_0;

};  // 定义一个RGB结构体

struct Led_s Led;
rt_bool_t initialnized = RT_FALSE;



/**********线程优先级*********/
#define RC522_THREAD_PRIORITY     15

// M1卡分为16个扇区，每个扇区由四个块（块0、块1、块2、块3）组成
// 将16个扇区的64个块按绝对地址编号为：0~63
// 第0个扇区的块0（即绝对地址0块），用于存放厂商代码，已经固化不可更改
// 每个扇区的块0、块1、块2为数据块，可用于存放数据
// 每个扇区的块3为控制块（绝对地址为:块3、块7、块11.....）包括密码A，存取控制、密码B等

/*******************************
*连线说明：
*1--SDA  <----->PA4
*2--SCK  <----->PA5
*3--MOSI <----->PA7
*4--MISO <----->PA6
*5--悬空
*6--GND <----->GND
*7--RST <----->PC4
*8--VCC <----->VCC
************************************/
#define MAXRLEN 18

/*
 * 函数名：PcdInit
 * 描述  ：引脚初始化
 * 返回  : 状态值
 *         = MI_OK，成功
 */
void PcdInit()
{
    rt_pin_mode(MF522_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MF522_MISO_PIN, PIN_MODE_INPUT);
    rt_pin_mode(MF522_MOSI_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MF522_SCK_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MF522_NSS_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
}

/*
 * 函数名：PcdRequest
 * 描述  ：寻卡
 * 输入  ：req_code，寻卡方式
 *                     = 0x52，寻感应区内所有符合14443A标准的卡
 *                     = 0x26，寻未进入休眠状态的卡
 *         pTagType，卡片类型代码
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
   char status;
   unsigned int  unLen;
   unsigned char ucComMF522Buf[MAXRLEN];
   ClearBitMask(Status2Reg,0x08);
   WriteRawRC(BitFramingReg,0x07);

   SetBitMask(TxControlReg,0x03);

   ucComMF522Buf[0] = req_code;

   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);

   if ((status == MI_OK) && (unLen == 0x10))
   {
       *pTagType     = ucComMF522Buf[0];
       *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {   status = MI_ERR;   }

   return status;
}

/*
 * 函数名：PcdAnticoll
 * 描述  ：防冲撞
 * 输入  ：pSnr，卡片序列号，4字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];


    ClearBitMask(Status2Reg,0x08);
    WriteRawRC(BitFramingReg,0x00);
    ClearBitMask(CollReg,0x80);

    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if (status == MI_OK)
    {
         for (i=0; i<4; i++)
         {
             *(pSnr+i)  = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])
         {   status = MI_ERR;    }
    }

    SetBitMask(CollReg,0x80);
    return status;
}

/*
 * 函数名：PcdSelect
 * 描述  ：选定卡片
 * 输入  ：pSnr，卡片序列号，4字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdSelect(unsigned char *pSnr)
{
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
        ucComMF522Buf[i+2] = *(pSnr+i);
        ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);

    ClearBitMask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);

    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

/*
 * 函数名：PcdAuthState
 * 描述  ：验证卡片密码
 * 输入  ：auth_mode，密码验证模式
 *                     = 0x60，验证A密钥
 *                     = 0x61，验证B密钥
 *         u8 ucAddr，块地址
 *         pKey，密码
 *         pSnr，卡片序列号，4字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }

    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }

    return status;
}

/*
 * 函数名：PcdRead
 * 描述  ：读取M1卡一块数据
 * 输入  ：u8 addr，块地址
 *         pData，读出的数据，16字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdRead(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }

    return status;
}

/*
 * 函数名：PcdWrite
 * 描述  ：写数据到M1卡一块
 * 输入  ：u8 addr，块地址
 *         pData，写入的数据，16字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdWrite(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    if (status == MI_OK)
    {

        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }

    return status;
}

/*
 * 函数名：PcdSpicelWrite
 * 描述  ：写数据到M1卡一块
 * 输入  ：u8 addr，块地址
 *        pData，写入的数据，16字节
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdSpicelWrite(unsigned char addr, unsigned char *pData)
{
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    //需要使用以下步骤开启后门
    PcdHalt();
    WriteRawRC(BitFramingReg, 0x07);
    ucComMF522Buf[0] = 0x40;
    PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);

    WriteRawRC(BitFramingReg, 0x00);
    ucComMF522Buf[0] = 0x43;
    PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);

    return PcdWrite(addr, pData);
}

/*
 * 函数名：PcdHalt
 * 描述  ：命令卡片进入休眠状态
 * 输入  ：无
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：外部调用
 */
char PcdHalt(void)
{
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}

/*
 * 函数名：CalulateCRC
 * 描述  ：用RC522计算CRC16
 * 输入  ：    pIndata，计算CRC16的数组
 *         len，计算CRC16的数组字节长度
 *         pOutData，存放计算结果存放的首地址
 * 返回  : 无
 * 调用  ：内部调用
 */
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   WriteRawRC(FIFODataReg, *(pIndata+i));   }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
}

/*
 * 函数名：PcdRese
 * 描述  ：复位RC522
 * 输入  ：无
 * 返回  : 无
 * 调用  ：外部调用
 */
char PcdReset(void)
{
    RST_H;
    rt_thread_mdelay(10);
    RST_L;
    rt_thread_mdelay(10);
    RST_H;
    rt_thread_mdelay(100);

    if(ReadRawRC(0x02) == 0x80)
    {
    }

    WriteRawRC(CommandReg,PCD_RESETPHASE);

    WriteRawRC(ModeReg,0x3D);
    WriteRawRC(TReloadRegL,30);
    WriteRawRC(TReloadRegH,0);
    WriteRawRC(TModeReg,0x8D);
    WriteRawRC(TPrescalerReg,0x3E);
    WriteRawRC(TxAutoReg,0x40);
    return MI_OK;
}

/*
 * 函数名：M500PcdConfigISOType
 * 描述  ：设置RC522的工作方式
 * 输入  ：ucType，工作方式
 * 返回  : 无
 * 调用  ：外部调用
 */
char M500PcdConfigISOType(unsigned char type)
{
   if (type == 'A')
   {
      ClearBitMask(Status2Reg,0x08);
      WriteRawRC(ModeReg,0x3D);//3F
      WriteRawRC(RxSelReg,0x86);//84
      WriteRawRC(RFCfgReg,0x7F);   //4F
      WriteRawRC(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec)
      WriteRawRC(TReloadRegH,0);
      WriteRawRC(TModeReg,0x8D);
      WriteRawRC(TPrescalerReg,0x3E);
      rt_thread_mdelay(10);
      PcdAntennaOn();
   }
   else{ return (char)-1; }
   return MI_OK;
}
/*
 * 函数名：ReadRawRC
 * 描述  ：读RC522寄存器
 * 输入  ：ucAddress，寄存器地址
 * 返回  : 寄存器的当前值
 * 调用  ：内部调用
 */
unsigned char ReadRawRC(unsigned char Address)
{
     unsigned char i, ucAddr;
     unsigned char ucResult=0;
     NSS_L;
     ucAddr = ((Address<<1)&0x7E)|0x80;
     for(i=8;i>0;i--)
     {
         SCK_L;
         if(ucAddr&0x80)
            MOSI_H;
         else
            MOSI_L;
         SCK_H;
         ucAddr <<= 1;
     }
     for(i=8;i>0;i--)
     {
         SCK_L;
         ucResult <<= 1;
         SCK_H;
         if(READ_MISO == 1)
            ucResult |= 1;
     }
     NSS_H;
     SCK_H;
     return ucResult;
}
/*
 * 函数名：WriteRawRC
 * 描述  ：写RC522寄存器
 * 输入  ：ucAddress，寄存器地址
 *         ucValue，写入寄存器的值
 * 返回  : 无
 * 调用  ：内部调用
 */
void WriteRawRC(unsigned char Address, unsigned char value)
{
    unsigned char i, ucAddr;
    SCK_L;
    NSS_L;
    ucAddr = ((Address<<1)&0x7E);
    for(i=8;i>0;i--)
    {
        if(ucAddr&0x80)
            MOSI_H;
        else
            MOSI_L;
        SCK_H;
        ucAddr <<= 1;
        SCK_L;
    }
    for(i=8;i>0;i--)
    {
        if(value&0x80)
            MOSI_H;
        else
            MOSI_L;
        SCK_H;
        value <<= 1;
        SCK_L;
    }
    NSS_H;
    SCK_H;
}

/*
 * 函数名：SetBitMask
 * 描述  ：对RC522寄存器置位
 * 输入  ：ucReg，寄存器地址
 *         ucMask，置位值
 * 返回  : 无
 * 调用  ：内部调用
 */
void SetBitMask(unsigned char reg,unsigned char mask)
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
}
/*
 * 函数名：ClearBitMask
 * 描述  ：对RC522寄存器清位
 * 输入  ：ucReg，寄存器地址
 *         ucMask，清位值
 * 返回  : 无
 * 调用  ：内部调用
 */
void ClearBitMask(unsigned char reg,unsigned char mask)
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
}
/*
 * 函数名：PcdComMF522
 * 描述  ：通过RC522和ISO14443卡通讯
 * 输入  ：Command，RC522命令字
 *         pInData，通过RC522发送到卡片的数据
 *         InLenByte，发送数据的字节长度
 *         pOutData，接收到的卡片返回数据
 *         pOutLenBit，返回数据的位长度
 * 返回  : 状态值
 *         = MI_OK，成功
 * 调用  ：内部调用
 */
char PcdComMF522(unsigned char Command,
                 unsigned char *pInData,
                 unsigned char InLenByte,
                 unsigned char *pOutData,
                 unsigned int  *pOutLenBit)
{
    char status = MI_ERR;
    unsigned char irqEn   = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
    switch (Command)
    {
       case PCD_AUTHENT:
          irqEn   = 0x12;
          waitFor = 0x10;
          break;
       case PCD_TRANSCEIVE:
       case PCD_SPECIAL_COPY:
          irqEn   = 0x77;
          waitFor = 0x30;
          break;
       default:
         break;
    }
    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<InLenByte; i++)
    {   WriteRawRC(FIFODataReg, pInData[i]);    }
    WriteRawRC(CommandReg, Command);
    if (Command == PCD_TRANSCEIVE)
    {    SetBitMask(BitFramingReg,0x80);  }
 i = 2000;
    do
    {
         n = ReadRawRC(ComIrqReg);
         i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));
    ClearBitMask(BitFramingReg,0x80);
    if (i!=0)
    {
         if(!(ReadRawRC(ErrorReg)&0x1B))
         {
             status = MI_OK;
             if (n & irqEn & 0x01)
             {   status = MI_NOTAGERR;   }
             if (Command == PCD_TRANSCEIVE)
             {
                n = ReadRawRC(FIFOLevelReg);
                lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {   *pOutLenBit = (n-1)*8 + lastBits;   }
                else
                {   *pOutLenBit = n*8;   }
                if (n == 0)
                {   n = 1;    }
                if (n > MAXRLEN)
                {   n = MAXRLEN;   }
                for (i=0; i<n; i++)
                {   pOutData[i] = ReadRawRC(FIFODataReg);
                }
            }
         }

         else
         {   status = MI_ERR;   }
   }
   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE);
   return status;
}
/*
 * 函数名：PcdAntennaOn
 * 描述  ：开启天线
 * 输入  ：无
 * 返回  : 无
 * 调用  ：内部调用
 */
void PcdAntennaOn()
{
    unsigned char i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}
/*
 * 函数名：PcdAntennaOff
 * 描述  ：开启天线
 * 输入  ：无
 * 返回  : 无
 * 调用  ：内部调用
 */
void PcdAntennaOff()
{
    ClearBitMask(TxControlReg, 0x03);
}
void WaitCardOff(void)
{
    char          status;
  unsigned char TagType[2];
    while(1)
    {
        status = PcdRequest(REQ_ALL, TagType);
        if(status)
        {
            status = PcdRequest(REQ_ALL, TagType);
            if(status)
            {
                status = PcdRequest(REQ_ALL, TagType);
                if(status)
                {
                    return;
                }
            }
        }
        rt_thread_mdelay(100);
    }
}

static void RC522_thread_entry(void *parameter)
//void RC522_thread_entry(void *parameter)
{
    char status;
    unsigned char buf[16];                                               //数据缓存
    unsigned char TagType[2];                                            //卡片类型
    unsigned char SelectedSnr[4];                                          //卡片序列号
 //   unsigned char DefaultKey1[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};     //卡默认密码
    unsigned char DefaultKey2[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};     //卡默认密码
  //  unsigned char inDoor[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};                                                                  //写入的数据
    unsigned char inDoor[16] = {0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00};

    //rt_sem_take(sem_i_rc, RT_WAITING_FOREVER);//获取信号量

    PcdInit();
    PcdReset();
    PcdAntennaOff();
    PcdAntennaOn();
    M500PcdConfigISOType( 'A' );
    rt_kprintf( "rc522 init over!\n" );
   // rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);  //输出模式
    //HAL_GPIO_Write(LED0_PIN, LED0_PIN, PinState)

    while(1)
    {
        status= PcdRequest( REQ_ALL , TagType );    //寻卡
        if(status == MI_OK)
        {
            status = PcdAnticoll(SelectedSnr);      //防冲撞
            if(status == MI_OK)
            {
                status = PcdSelect(SelectedSnr);    //选定卡片
                if(status == MI_OK)
                {
                    for(int i = 0;i < 4 ;i++)                                              //读写16个扇区
                        {
                            status = PcdAuthState(KEYA, (i*4+3), DefaultKey2, SelectedSnr);  //验证卡片当前扇区的密码
                            status = PcdAuthState(KEYB, (i*4+3), DefaultKey2, SelectedSnr);  //验证卡片当前扇区的密码

//                            if(i == 0)
//                            {
//                                status = PcdWrite(1, inDoor);                              //第一扇区第1块写值(该扇区第0块不能写) (需要先验证)
//                                rt_kprintf( "sta5:%d!\n",status );
//                            }

                            {
                                for(int j = 0; j < 4; j++)
                                {
                                    status = PcdRead((i*4+j), buf);                             //读取当前扇区每个块的值
                                    if(status == MI_OK)

                                    {


                                        rt_pin_write(LED0_PIN, 1);   //使能灯闪烁
                                        rt_thread_mdelay(5000);
                                        rt_pin_write(LED0_PIN, 0);
                                    //rt_kprintf("3! %x \n", status);
                                        for(int k = 0; k < 16; k++)                             //打印当前块的值
                                        {
                                            rt_kprintf("%x ", (uint16_t)buf[k]);
                                        }
                                       // rt_kprintf("\n");
                                    }
                                }
                            }
                        }
                        }

                    rt_kprintf("operate finished! %x \n", status);
                    WaitCardOff();
                }
            }
        rt_thread_mdelay(100);
        }

    }

static int RC522_Priority_temp(void)
{
    rt_thread_t RC522_thread;
    RC522_thread = rt_thread_create("RC522_thread",
                                     RC522_thread_entry,
                                     "RC522_thread",
                                     1024,
                                     RC522_THREAD_PRIORITY,
                                     20);
    if (RC522_thread != RT_NULL)
    {
        rt_thread_startup(RC522_thread);
    }
    return RT_EOK;
}
INIT_APP_EXPORT(RC522_Priority_temp);


static rt_err_t spi_configure(struct rt_spi_device *device,
                              struct rt_spi_configuration *configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    struct stm32_spi *spi_drv =  rt_container_of(device->bus, struct stm32_spi, spi_bus);
    spi_drv->cfg = configuration;

    return stm32_spi_init(spi_drv, configuration);
}

static const struct rt_spi_ops stm_spi_ops =
{
    .configure = spi_configure,

};
