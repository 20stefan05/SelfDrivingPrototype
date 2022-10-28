#include "Lidar.h"
static uint8_t LidarStopScan[COMMAND_SIZE] = {0xA5, 0x40};
static uint8_t LidarStartScan[COMMAND_SIZE] = {0xA5, 0x20};
static uint8_t Debug_AngleLow = 0;
static uint8_t Debug_AngleHigh = 0;
static uint8_t Debug_DistLow = 0;
static uint8_t Debug_DistHigh = 0;
extern uint64_t Interval, Millis;
static void SendFirstCommands(UART_HandleTypeDef* Huart1){
	  HAL_UART_Transmit(Huart1, LidarStopScan, COMMAND_SIZE, 10);
	  HAL_Delay(1200);
	  HAL_UART_Transmit(Huart1, LidarStartScan, COMMAND_SIZE, 10);
	  __HAL_UART_CLEAR_IT(Huart1, UART_CLEAR_OREF);
}
void LidarInit(uint8_t* LidarRxBuff, bool* Received, UART_HandleTypeDef* Huart1, uint8_t* LidarState)
{
	  uint64_t Timer = 0; /* timer handler for SWDelay */
	  uint8_t Idx = 0;
	  uint8_t FirstBuff[FIRST_BUFF_SZ];
	  uint64_t Ticks = 0;
	  	  SendFirstCommands(Huart1);
	  	  HAL_UART_Receive_DMA(Huart1, FirstBuff, FIRST_BUFF_SZ);
	  	  TimerSWStartup(&Timer, WAIT_MS);
	  	  for(Ticks = 0; ((*Received)==false) && (Ticks<MAX_TICKS); Ticks++)
	  	  {
	  		  if (Idx < MAX_SEND_NR)
	  		  {
	  			  if (TimerSWCheckExpire(&Timer))
	  			  {
	  				  HAL_UART_Transmit(Huart1, LidarStartScan, COMMAND_SIZE, COM_DELAY);
	  				  Idx++;
	  				  TimerSWStartup(&Timer, WAIT_MS);
	  			  }
	  		  }
	  		  else
	  		  {
	  			  *LidarState = 0;
	  		  }
	  	  }
	  	  __HAL_UART_CLEAR_IT(Huart1, UART_CLEAR_OREF);
	  	  HAL_UART_Receive_DMA(Huart1, LidarRxBuff, BUFF_SIZE);
}
static uint8_t Check(const uint8_t* LidarRxBuff, const uint8_t Idx)
{
	uint8_t SCheck = 0;
	uint8_t C;
	uint8_t RetVal = 0;
		if(((LidarRxBuff[Idx]&0x03)==0x01)||
			((LidarRxBuff[Idx]&0x03)==0x02)) SCheck = 1;
		C = LidarRxBuff[Idx+1]&(0x01);
		RetVal = SCheck & C;
	return RetVal;
}
static uint8_t GetQuality(const uint8_t* LidarRxBuff, const uint8_t Idx)
{
	return LidarRxBuff[Idx]>>2;
}
static float GetAngle(const uint8_t* LidarRxBuff, const uint8_t Idx)
{
	uint16_t AngleLow = 0;
	uint16_t AngleHigh = 0;
	float FullAngleWord = 0.0f;
	float ScaledAngle = 0.0f;
		/* Processing the angle according to the datasheet */
		AngleLow = (uint16_t)LidarRxBuff[Idx + ANGLE_LOW];
		AngleHigh = (((uint16_t)LidarRxBuff[Idx + ANGLE_HIGH]) << 8u);
		Debug_AngleLow = AngleLow;
		Debug_AngleHigh = AngleHigh;
		AngleHigh |= AngleLow; /* Now angle high contains the whole word */
		AngleHigh >>= 1u; /* Now angle high contains the whole word without the check bit*/
		FullAngleWord = (float)(AngleHigh);
		ScaledAngle = FullAngleWord / ANGLE_SCALE_COEFFICIENT;
	return ScaledAngle;
}
static float GetDistance(const uint8_t* LidarRxBuff, const uint8_t Idx)
{
	uint16_t DistLow = 0;
	uint16_t DistHigh = 0;
	float FullDistWord = 0.0f;
	float ScaledDist = 0.0f;
		/* Processing the distance according to the datasheet */
		DistLow = (uint16_t)LidarRxBuff[Idx + DIST_LOW];
		DistHigh = ((((uint16_t)LidarRxBuff[Idx + DIST_HIGH]) << 8u)&(0xff00));
		Debug_DistLow = DistLow;
		Debug_DistHigh = DistHigh;
		DistHigh |= DistLow; /* Now dist high contains the whole word */
		FullDistWord = (float)(DistHigh);
		ScaledDist = FullDistWord / DISTANCE_SCALE_COEFFICIENT;
	return ScaledDist;
}
static void ClearArray(float* Array, const uint16_t Size)
{
	uint16_t Idx;
	for(Idx = 0; Idx<Size; Idx++){
		Array[Idx] = -1.0f;
	}
}
float PolarToCartesianX(const float AngleDeg, const float Distance)
{
	float AngleRad = 0.0f;
	float X = 0.0f;
    	AngleRad = AngleDeg * PI /HALF_CIRCLE;
    	X = Distance * cos(AngleRad);
    return X;
}
float PolarToCartesianY(const float AngleDeg, const float Distance)
{
	float AngleRad = 0.0f;
	float Y = 0.0f;
    	AngleRad = AngleDeg * PI /HALF_CIRCLE;
    	Y = Distance * sin(AngleRad);
    return Y;
}

uint8_t LidarGetData(const uint8_t* LidarRxBuff, const bool LidarStarted, float* PosXBuff, float* PosYBuff, const uint8_t LidarState)
{
	uint8_t OutputBuffIdx = 0;
	uint8_t NrErr = 0;
	uint8_t LidarBuffIdx = 0;
	uint8_t ReturnStatus = LIDAR_OK;
	uint8_t Quality = 0;
	float Angle = 0;
	float Distance = 0;
	if ((LidarStarted != false) && (LidarState == LIDAR_OK))
	{
		for (LidarBuffIdx = 0; LidarBuffIdx < BUFF_SIZE; LidarBuffIdx +=
				PACK_SIZE)

		{
			if (Check(LidarRxBuff, LidarBuffIdx) == 1 && Interval - Millis <= 2000)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

				Angle = GetAngle(LidarRxBuff, LidarBuffIdx);
				Distance = GetDistance(LidarRxBuff, LidarBuffIdx);
				Quality = GetQuality(LidarRxBuff, LidarBuffIdx);
				if(Quality > MAX_QUALITY/2)
				{
					PosXBuff[OutputBuffIdx] = PolarToCartesianX(Angle, Distance);
					PosYBuff[OutputBuffIdx] = PolarToCartesianY(Angle, Distance);
				}
				if ((Angle > CIRCLE_SIZE_DEG_F) || (Angle < MIN_ANGLE))
				{
					ClearArray(PosXBuff, CIRCLE_SIZE_DEG_I);
					ClearArray(PosYBuff, CIRCLE_SIZE_DEG_I);
				}
				if (Angle == MIN_ANGLE)
				{
					NrErr++;
				}
				if (NrErr >= MAX_ERR) {
					ClearArray(PosXBuff, CIRCLE_SIZE_DEG_I);
					ClearArray(PosYBuff, CIRCLE_SIZE_DEG_I);
				}
				OutputBuffIdx++;
			}
			else
			{
				if(Interval - Millis > 2000)
				{
					ReturnStatus = LIDAR_ERR;
				}
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
				LidarBuffIdx++;
			}
		}
	}
	return ReturnStatus;
}
