#include "stm32f10x.h"

#include "oled.h"
#include "oled_font.h"
#include "bsp_soft_i2c.h"

/* 引脚操作全部委托给 BSP：只有 bsp/ 层允许直接操作 GPIO。
 * 开漏配置由 BSP_SoftI2C_Init() 统一完成，本模块只负责送电平。 */
#define OLED_W_SCL(x)       BSP_SoftI2C_WriteScl((uint8_t)(x))
#define OLED_W_SDA(x)       BSP_SoftI2C_WriteSda((uint8_t)(x))

/* 半周期延时：原来一个延时都没有，72 MHz 下 SCL 达到 2~2.5 MHz，
 * 远超 SSD1306 的 400 kHz 上限，表现为「有时亮、有时白屏、偶发乱码」。
 * 4 个 __NOP 约 0.4 us，SCL 约 1.2 MHz —— 实测稳定，整屏刷新仍只要 ~30 ms。
 * 换长线 / 劣质模块还不亮，就加到 8（约 600 kHz）或 16（约 300 kHz）。 */
#define OLED_I2C_DELAY_NOP  4U

static uint8_t s_oled_present = 0U;   /* 地址字节收到 ACK 置 1；0 = 屏没应答 */

static void OLED_Delay(void)
{
    volatile uint8_t i = OLED_I2C_DELAY_NOP;
    while (i-- != 0U)
    {
        __NOP();
    }
}

/* 引脚初始化：开漏输出 + 释放总线 */
void OLED_I2C_Init(void)
{
    BSP_SoftI2C_Init();
}

/**
  * @brief  I2C开始
  */
void OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_Delay();
    OLED_W_SDA(0);          /* SCL 高时 SDA 下降 = 起始条件 */
    OLED_Delay();
    OLED_W_SCL(0);
}

/**
  * @brief  I2C停止
  */
void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_Delay();
    OLED_W_SCL(1);          /* SCL 高时 SDA 上升 = 停止条件 */
    OLED_Delay();
    OLED_W_SDA(1);
    OLED_Delay();
}

/**
  * @brief  I2C发送一个字节，并读从机应答
  * @retval 1 = 从机应答（ACK）；0 = 无应答（NACK / 没接屏）
  */
uint8_t OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8; i++)
    {
        OLED_W_SDA(!!(Byte & (0x80 >> i)));
        OLED_Delay();
        OLED_W_SCL(1);
        OLED_Delay();
        OLED_W_SCL(0);
        OLED_Delay();
    }

    /* 第 9 个时钟读应答：先释放 SDA，再打一个时钟，看从机有没有把它拉低 */
    OLED_W_SDA(1);
    OLED_Delay();
    OLED_W_SCL(1);
    OLED_Delay();
    ack = (BSP_SoftI2C_ReadSda() == 0U) ? 1U : 0U;
    OLED_W_SCL(0);
    OLED_Delay();

    return ack;
}

/**
  * @brief  OLED写命令
  */
void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    s_oled_present = OLED_I2C_SendByte(0x78);   /* 从机地址，顺便取应答 */
    (void)OLED_I2C_SendByte(0x00);              /* 控制字节：命令 */
    (void)OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据
  */
void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    s_oled_present = OLED_I2C_SendByte(0x78);   /* 从机地址，顺便取应答 */
    (void)OLED_I2C_SendByte(0x40);              /* 控制字节：数据 */
    (void)OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(s_oled_font[Char - ' '][i]);			//显示上半部分内容
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(s_oled_font[Char - ' '][i + 8]);		//显示下半部分内容
	}
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~4294967295
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	
    OLED_I2C_Init();            /* 端口初始化：开漏 + 释放总线 */

    /* 总线复位：MCU 若在传输中途复位，SSD1306 会卡在「等第 9 个时钟」并拉住 SDA，
     * 之后所有通信全废、只能给屏断电。这里先补 9 个时钟 + 一个 Stop 把它踢出来。 */
    OLED_W_SDA(1);
    for (i = 0U; i < 9U; i++)
    {
        OLED_W_SCL(1);
        OLED_Delay();
        OLED_W_SCL(0);
        OLED_Delay();
    }
    OLED_W_SDA(0);
    OLED_Delay();
    OLED_W_SCL(1);
    OLED_Delay();
    OLED_W_SDA(1);
    OLED_Delay();
	
	OLED_WriteCommand(0xAE);	//关闭显示
	
	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);	//设置显示开始行
	
	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示
		
	OLED_Clear();				//OLED清屏
}

/**
  * @brief  屏是否在线（最近一次寻址有没有收到应答）
  * @retval 1 = 有应答；0 = 没供电 / 没接好 / 地址不对 / 总线卡死
  */
uint8_t OLED_IsPresent(void)
{
    return s_oled_present;
}
