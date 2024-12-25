#define _CRT_SECURE_NO_WARNINGS 1
#include <Usart.h>

/*STM32的串口重定义*/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(FILE *f)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
/*
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);

	return ch;
}
GETCHAR_PROTOTYPE
{
	uint8_t ch = 0;
	HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 0xFFFF);

	if (ch == '\r')
	{
		__io_putchar('\r');
		ch = '\n';
	}

	return __io_putchar(ch);
}
*/

/*串口空闲中断*/
/*空闲中断使能
__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
HAL_UART_Receive_DMA(&huart2, (uint8_t*)Receive_buff, BUFFER_SIZE);//串口DMA接收信号使能
*/
/*空闲中断函数
void USER_UART_IRQHandler(UART_HandleTypeDef* huart)
{
	if (huart->Instance == USART2)
	{
		if (RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))//判断是否处于空闲状态
		{
			__HAL_UART_CLEAR_IDLEFLAG(huart);//清除空闲状态标志位
			USER_UART_IDLECallback(huart);//定义空闲回调函数
		}
	}
}
*/
/*空闲回调函数
void USER_UART_IDLECallback(UART_HandleTypeDef* huart)
{
	HAL_UART_DMAStop(huart);//串口DMA接收信号停止
	uint8_t SPEED_MODE_FLAG = 0;
	SPEED_MODE_FLAG = UART_RxMode();
	switch (SPEED_MODE_FLAG)
	{
	case 1:
	{
		rtU.speed_ref = rtU.speed_ref + 500.0;
	}break;
	case 2:
	{
		rtU.speed_ref = rtU.speed_ref - 500.0;
	}break;
	case 3:
	{
		rtU.speed_ref = 12000.0;
	}break;
	case 4:
	{
		rtU.speed_ref = 17000.0;
	}break;
	case 5:
	{
		rtU.speed_ref = 20000.0;
	}break;
	case 6:
	{
		rtU.speed_ref = 22000.0;
	}break;
	case 7:
	{
		rtU.speed_ref = 24000.0;
	}break;
	case 8:
	{
		rtU.speed_ref = 26000.0;
	}break;
	case 9:
	{
		rtU.speed_ref = 28000.0;
	}break;
	case 10:
	{
		rtU.speed_ref = 30000.0;
	}break;
	case 11:
	{
		rtU.speed_ref = 33000.0;
	}break;

	default:
		break;
		//		printf("%f\r\n",rtU.speed_ref);
	}
	printf("%f\r\n", rtU.speed_ref);
	HAL_UART_Receive_DMA(&huart2, (uint8_t*)Receive_buff, BUFFER_SIZE);//串口DMA接收信号使能
}
*/
/*
uint8_t UART_RxMode(void)
{
	uint8_t UART_FLAG = 0;
	if (strncmp((char*)Receive_buff, "SAdd", 4) == 0)
	{
		UART_FLAG = 1;
	}
	else if (strncmp((char*)Receive_buff, "SSub", 4) == 0)
	{
		UART_FLAG = 2;
	}
	else if (strncmp((char*)Receive_buff, "P1", 2) == 0)
	{
		UART_FLAG = 3;
	}
	else if (strncmp((char*)Receive_buff, "P2", 2) == 0)
	{
		UART_FLAG = 4;
	}
	else if (strncmp((char*)Receive_buff, "P3", 2) == 0)
	{
		UART_FLAG = 5;
	}
	else if (strncmp((char*)Receive_buff, "P4", 2) == 0)
	{
		UART_FLAG = 6;
	}
	else if (strncmp((char*)Receive_buff, "P5", 2) == 0)
	{
		UART_FLAG = 7;
	}
	else if (strncmp((char*)Receive_buff, "P6", 2) == 0)
	{
		UART_FLAG = 8;
	}
	else if (strncmp((char*)Receive_buff, "P7", 2) == 0)
	{
		UART_FLAG = 9;
	}
	else if (strncmp((char*)Receive_buff, "P8", 2) == 0)
	{
		UART_FLAG = 10;
	}
	else if (strncmp((char*)Receive_buff, "P9", 2) == 0)
	{
		UART_FLAG = 11;
	}
	else
	{

	}
	return UART_FLAG;
}
*/