#define _CRT_SECURE_NO_WARNINGS 1
#include <main.h>
#include <FOC_Model.h>

/*双ADC注入通道同步采样使能*/
/*
HAL_ADCEx_InjectedStart(&hadc2);
HAL_ADCEx_InjectedStart_IT(&hadc1);

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)//ADC注入通道中断回调函数
*/
/*ADC采样，DMA数据传输使能、规则通道触发*/
/*
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)Vbus_Buff, 3);
HAL_TIM_Base_Start(&htim3);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)//ADC规则通道中断回调函数
*/

/*滑动滤波器*/

float _Filter_SlidingWindowAvg(float data, MAF_Filter* localD)
{
	
	uint16_t index = 0;
	float sum = 0;
	uint16_t InvNUM = 0;
	float IdcSUM = 0.0f;

	index = localD->index;
	sum = localD->SumV;
	InvNUM = localD->InvNUM;
	if (index != NUM)
	{
		localD->Buff[index] = data;
		index++;
		sum = sum + data;
		IdcSUM = sum / index;

		localD->index = index;
		localD->SumV = sum;
		return IdcSUM;
	}

	sum = sum + data - localD->Buff[InvNUM];
	localD->Buff[InvNUM] = data;
	InvNUM++;
	InvNUM = (InvNUM == NUM) ? 0 : InvNUM;
	IdcSUM = sum / NUM;
	localD->SumV = sum;
	localD->InvNUM = InvNUM;
	localD->Buff[InvNUM] = data;
	return IdcSUM;

}
int main(void)
{
	return 0;
}

/*
<<<<<<<<< Temporary merge branch 1
合并1
=========
合并111
>>>>>>>>> Temporary merge branch 2
*/