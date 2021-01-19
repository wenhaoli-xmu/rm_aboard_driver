#include "pidtools.h"
#include <stdlib.h>
#include <string.h>


static void two_float2signal( float refer, float trace, char * tmp )
{
	int stack[18];
	int top = 0, cnt = 0;

	float f1 = refer * 1000;
	float f2 = trace * 1000;
	int n1 = f1, n2 = f2;

	//处理帧头
	tmp[cnt++] = '@';

	//处理第一个数据
	if ( n1 != 0 )
	{
		if ( n1 < 0 )
		{
			n1 = -n1;
			tmp[cnt++] = '-';
		}
		while ( n1 > 0 )
		{
			stack[top++] = n1 % 10;
			n1 /= 10;
		}
		while ( (--top) > 0 )
		{
			tmp[cnt++] = '0' + stack[top];
		}
	}
	else
	{
		tmp[cnt++] = '0';
	}

	//分隔符
	tmp[cnt++] = ',';
	top = 0;

	//处理第二个数据
	if ( n2 != 0 )
	{
		if ( n2 < 0 )
		{
			n2 = -n2;
			tmp[cnt++] = '-';
		}
		while ( n2 > 0 )
		{
			stack[top++] = n2 % 10;
			n2 /= 10;
		}
		while ( (--top) > 0 )
		{
			tmp[cnt++] = '0' + stack[top];
		}
	}
	else
	{
		tmp[cnt++] = '0';
	}

	//处理帧尾
	tmp[cnt++] = '!';
	tmp[cnt++] = '\0';
}

void PidTools_Open( PidTools_t * p, UART_HandleTypeDef * huart )
{
	p->huart = huart;
	p->rx_length = 20;

	SET_BIT( huart->Instance->CR3, USART_CR3_DMAR );
	__HAL_UART_ENABLE_IT( huart, UART_IT_IDLE );
	__HAL_DMA_DISABLE( huart->hdmarx );
	while ( huart->hdmarx->Instance->CR & DMA_SxCR_EN )
	{
		__HAL_DMA_DISABLE( huart->hdmarx );
	}
	huart->hdmarx->Instance->PAR = ( uint32_t ) & ( huart->Instance->DR );
	huart->hdmarx->Instance->M0AR = ( uint32_t )( p->rx_data1 );
	huart->hdmarx->Instance->M1AR = ( uint32_t )( p->rx_data2 );
	huart->hdmarx->Instance->NDTR = p->rx_length * 2;
	SET_BIT( huart->hdmarx->Instance->CR, DMA_SxCR_DBM );
	__HAL_DMA_ENABLE( huart->hdmarx );
}

void PidTools_RxUpdate( PidTools_t * p )
{
	static uint8_t i;

	if ( p->huart->Instance->SR & UART_FLAG_IDLE ) {

		static uint16_t this_time_rx_len = 0;
		static UART_HandleTypeDef * huart;
		huart = p->huart;
		__HAL_UART_CLEAR_PEFLAG( huart );
		if ( ( huart->hdmarx->Instance->CR & DMA_SxCR_CT ) == RESET )
		{
			__HAL_DMA_DISABLE( huart->hdmarx );

			this_time_rx_len = p->rx_length * 2 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = p->rx_length * 2;
			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE( huart->hdmarx );

			if ( this_time_rx_len == p->rx_length )
			{
				for ( i = 0; i < p->rx_length; i ++ ) p->rx_addr[i] = p->rx_data2[i];
			}
		}

		else
		{
			__HAL_DMA_DISABLE( huart->hdmarx );

			this_time_rx_len = p->rx_length * 2 - huart->hdmarx->Instance->NDTR;
			huart->hdmarx->Instance->NDTR = p->rx_length * 2;
			huart->hdmarx->Instance->CR &= ~( DMA_SxCR_CT );
			__HAL_DMA_ENABLE( huart->hdmarx );

			if ( this_time_rx_len == p->rx_length ) {
				for ( i = 0; i < p->rx_length; i ++ ) p->rx_addr[i] = p->rx_data1[i];
			}
		}

		/* 接收到一帧数据 */
		float * pf = ( float * )p->rx_addr;
		p->params[0] = *pf;
		pf = ( float * )( p->rx_addr + 4 );
		p->params[1] = *pf;
		pf = ( float * )( p->rx_addr + 8 );
		p->params[2] = *pf;
		pf = ( float * )( p->rx_addr + 12 );
		p->params[3] = *pf;
		pf = ( float * )( p->rx_addr + 16 );
		p->params[4] = *pf;
		PidTools_Callback( p );
	}
}

__weak void PidTools_Callback( PidTools_t * p )
{

}

float * PidTools_GetParams( PidTools_t * p )
{
	return p->params;
}

void PidTools_SetTxDataAdd( PidTools_t * p, void * refer_data, void * trace_data, uint8_t data_type )
{
	p->data_type = data_type;
	p->refer_add = refer_data;
	p->trace_add = trace_data;
}

static void transmit_data( PidTools_t * p )
{
	if ( p->data_type == PID_TOOLS_FLOAT )
	{
		float a, b;
		a = * ( float * )p->refer_add;
		b = * ( float * )p->trace_add;
		two_float2signal( a, b, p->tmp );
		HAL_UART_Transmit( p->huart, (uint8_t*)p->tmp, strlen( p->tmp ), 0xffff );
	}

	else if ( p->data_type == PID_TOOLS_INT )
	{
		int a, b;
		a = * ( int * )p->refer_add;
		b = * ( int * )p->trace_add;
		two_float2signal( (float)a, (float)b, p->tmp );
		HAL_UART_Transmit( p->huart, (uint8_t*)p->tmp, strlen( p->tmp ), 0xffff );
	}
}

void PidTools_MainTask( PidTools_t * p )
{
	transmit_data( p );
}
