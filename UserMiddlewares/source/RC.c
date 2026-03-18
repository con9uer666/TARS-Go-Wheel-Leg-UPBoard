#include "RC.h"
#include "usart.h"
#include "USER_CAN.h"
#include "Detect.h"
#include "UserFreertos.h"
#include "Judge.h"
#include "Board2Board.h"
#include "Com.h"
#include "Chassis.h"
#include "crc.h"
#define EN_RC_TASK		 // 使能任务
uint8_t usart5RxBuf[36]; // 串口5缓冲区
uint8_t Usart1RxBuf[200];// 串口1缓冲区 图传链路
RC_TypeDef rcInfo = {0}; // 遥控器信息
Image_Trans_TypeDef itInfo = {0}; // 图传链路信息
Key keyList[KEY_NUM];	 // 按键列表(包含所有可用键盘按键和鼠标左右键)
int rc_true_flag;
uint8_t flag_rc_vision = 1;
uint8_t stop_flag_t = 1;
extern remote_control_t RemoteControl;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern Chassis chassis;
// 内部函数
// 初始化判定时间
void RC_InitKeyJudgeTime(uint32_t key, uint16_t clickDelay, uint16_t longPressDelay);
// 初始化所有按键
void RC_InitKeys(void);
// 更新按键信息
void RC_UpdateKeys(void);
// 遥控任务回调
void Task_RC_Callback(void);
//掉线函数
void RC_LostCallback(void);


// 遥控器初始化
void RC_Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5RxBuf, sizeof(usart5RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Usart1RxBuf, sizeof(Usart1RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	RC_InitKeys(); // 初始化按键
}

// 解析串口数据
void RC_ParseUsart(uint8_t *buff)
{

	rcInfo.ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
	rcInfo.ch1 -= 1024;
	rcInfo.ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	rcInfo.ch2 -= 1024;
	rcInfo.ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	rcInfo.ch3 -= 1024;
	rcInfo.ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	rcInfo.ch4 -= 1024;

	/* prevent remote control zero deviation */
	if (rcInfo.ch1 <= 5 && rcInfo.ch1 >= -5)
		rcInfo.ch1 = 0;
	if (rcInfo.ch2 <= 5 && rcInfo.ch2 >= -5)
		rcInfo.ch2 = 0;
	if (rcInfo.ch3 <= 5 && rcInfo.ch3 >= -5)
		rcInfo.ch3 = 0;
	if (rcInfo.ch4 <= 5 && rcInfo.ch4 >= -5)
		rcInfo.ch4 = 0;

	rcInfo.left = ((buff[5] >> 4) & 0x000C) >> 2; // sw1   中间是3，上边是1，下边是2
	rcInfo.right = (buff[5] >> 4) & 0x0003;		  // sw2
	/*
	 if ((abs(rc->ch1) > 660) || \
		 (abs(rc->ch2) > 660) || \
		 (abs(rc->ch3) > 660) || \
		 (abs(rc->ch4) > 660))
	 {
	   memset(rc, 0, sizeof(struct rc));
	   return ;
	 }
   */
	rcInfo.mouse.x = buff[6] | (buff[7] << 8); // x axis
	rcInfo.mouse.y = buff[8] | (buff[9] << 8);
	rcInfo.mouse.z = buff[10] | (buff[11] << 8);

	rcInfo.mouse.l = buff[12];
	rcInfo.mouse.r = buff[13];

	rcInfo.kb.key_code = buff[14] | buff[15] << 8; // key borad code
	rcInfo.wheel = (buff[16] | buff[17] << 8) - 1024;
}

int Image_Trans_error_times;
uint16_t data_length;

void judge(uint8_t *buff)
{
	if(buff[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(buff,5))
		{
			data_length = (buff[1] | buff[2] << 8);
			buff = buff + 9 + data_length;
			judge(buff);
		}
	}
	else
	{
		return;
	}
	
}

void Image_Trans_Analysis(uint8_t *buff)
{
	
	judge(buff);
	
	if(buff[0] == 0xA9 && buff[1] == 0x53)
	{
		if(Verify_CRC16_Check_Sum(buff,21))
		{
			itInfo.ch1 = (buff[2] | buff[3] << 8) & 0x07FF;
			itInfo.ch1 -= 1024;
			itInfo.ch2 = (buff[3] >> 3 | buff[4] << 5) & 0x07FF;
			itInfo.ch2 -= 1024;
			itInfo.ch3 = (buff[4] >> 6 | buff[5] << 2 | buff[6] << 10) & 0x07FF;
			itInfo.ch3 -= 1024;
			itInfo.ch4 = (buff[6] >> 1 | buff[7] << 7) & 0x07FF;
			itInfo.ch4 -= 1024;
			itInfo.wheel = (buff[8] >> 1 | buff[9] << 7) & 0x07FF;
			itInfo.wheel -= 1024;
			
			itInfo.sw = (buff[7] >> 4) & 0x03;
			itInfo.pause = (buff[7] >> 6) & 0x01;
			itInfo.left = (buff[7] >> 7) & 0x01;
			itInfo.right = (buff[8] >> 0) & 0x01;
			itInfo.trigger = (buff[9] >> 4) & 0x01;
			
			itInfo.mouse.x = (buff[10] | buff[11] << 8);
			itInfo.mouse.y = buff[12] | (buff[13] << 8);
			itInfo.mouse.z = buff[14] | (buff[15] << 8);
			itInfo.mouse.left = buff[16] & 0x03;
			itInfo.mouse.right = (buff[16] >> 2) & 0x03;
			itInfo.mouse.middle = (buff[16] >> 4) & 0x03;
			
			itInfo.kb.key_code = (buff[17] | buff[18] << 8);
		}
		else
		{
			Image_Trans_error_times++;
		}
	}
}

// 串口5中断回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart5)
	{
		RC_ParseUsart(usart5RxBuf);
		Detect_Update(DeviceID_RC);
		rc_true_flag = 0;
	}
	if (huart == &huart2)
	{
		RS485_Rec();
	}
	if (huart == &huart1)
	{
		Image_Trans_Analysis(Usart1RxBuf);
	}
}

// 注册一个按键回调
void RC_Register(uint32_t key, KeyCombineType combine, KeyEventType event, KeyCallbackFunc func)
{
	// 寻找要操作的按键
	for (uint8_t index = 0; index < KEY_NUM; index++)
	{
		if (key & (0x01 << index))
		{
			// 根据按键事件注册回调
			switch (event)
			{
			case KeyEvent_OnClick:
				keyList[index].onClickCb.combineKey[keyList[index].onClickCb.number] = combine;
				keyList[index].onClickCb.func[keyList[index].onClickCb.number] = func;
				keyList[index].onClickCb.number++;
				break;
			case KeyEvent_OnLongPress:
				keyList[index].onLongCb.combineKey[keyList[index].onLongCb.number] = combine;
				keyList[index].onLongCb.func[keyList[index].onLongCb.number] = func;
				keyList[index].onLongCb.number++;
				break;
			case KeyEvent_OnDown:
				keyList[index].onDownCb.combineKey[keyList[index].onDownCb.number] = combine;
				keyList[index].onDownCb.func[keyList[index].onDownCb.number] = func;
				keyList[index].onDownCb.number++;
				break;
			case KeyEvent_OnUp:
				keyList[index].onUpCb.combineKey[keyList[index].onUpCb.number] = combine;
				keyList[index].onUpCb.func[keyList[index].onUpCb.number] = func;
				keyList[index].onUpCb.number++;
				break;
			case KeyEvent_OnPressing:
				keyList[index].onPressCb.combineKey[keyList[index].onPressCb.number] = combine;
				keyList[index].onPressCb.func[keyList[index].onPressCb.number] = func;
				keyList[index].onPressCb.number++;
				break;
			}
		}
	}
}

// 初始化一个按键的判定时间(键位ID，单击判定时间，长按判定时间)
void RC_InitKeyJudgeTime(uint32_t key, uint16_t clickDelay, uint16_t longPressDelay)
{
	for (uint8_t i = 0; i < 18; i++)
	{
		if (key & (0x01 << i))
		{
			keyList[i].clickDelayTime = clickDelay;
			keyList[i].longPressTime = longPressDelay;
		}
	}
}

// 初始化所有按键
void RC_InitKeys()
{
	RC_InitKeyJudgeTime(Key_All, 50, 200);
}

// 更新按键状态
void RC_UpdateKeys(void)
{
	static uint32_t lastUpdateTime;
	uint32_t presentTime = HAL_GetTick();

	// 检查组合键
	KeyCombineType combineKey = CombineKey_None;
	if (rcInfo.kb.bit.CTRL)
		combineKey = CombineKey_Ctrl;
	else if (rcInfo.kb.bit.SHIFT)
		combineKey = CombineKey_Shift;

	for (uint8_t key = 0; key < 18; key++)
	{
		// 读取按键状态
		uint8_t thisState = 0;
		if (key < 16)
			thisState = (rcInfo.kb.key_code & (0x01 << key)) ? 1 : 0; // 取出键盘对应位
		else if (key == 16)
			thisState = rcInfo.mouse.l;
		else if (key == 17)
			thisState = rcInfo.mouse.r;

		uint16_t lastPressTime = lastUpdateTime - keyList[key].startPressTime; // 上次更新时按下的时间
		uint16_t pressTime = presentTime - keyList[key].startPressTime;		   // 当前按下的时间

		// 按键状态判定
		/*******按下的一瞬间********/
		if (!keyList[key].lastState && thisState)
		{
			keyList[key].startPressTime = presentTime; // 记录按下时间
			keyList[key].isPressing = 1;

			// 依次执行回调
			for (uint8_t i = 0; i < keyList[key].onDownCb.number; i++)
				if (keyList[key].onDownCb.combineKey[i] == combineKey) // 符合组合键条件
					keyList[key].onDownCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnDown);
		}
		/*******松开的一瞬间********/
		else if (keyList[key].lastState && !thisState)
		{
			keyList[key].isLongPressed = 0;
			keyList[key].isPressing = 0;

			// 按键抬起
			keyList[key].isUp = 1;
			// 依次执行回调
			for (uint8_t i = 0; i < keyList[key].onUpCb.number; i++)
				if (keyList[key].onUpCb.combineKey[i] == combineKey) // 符合组合键条件
					keyList[key].onUpCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnUp);

			// 单击判定
			if (pressTime > keyList[key].clickDelayTime && pressTime < keyList[key].longPressTime)
			{
				keyList[key].isClicked = 1;
				// 依次执行回调
				for (uint8_t i = 0; i < keyList[key].onClickCb.number; i++)
					if (keyList[key].onClickCb.combineKey[i] == combineKey) // 符合组合键条件
						keyList[key].onClickCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnClick);
			}
		}
		/*******按键持续按下********/
		else if (keyList[key].lastState && thisState)
		{
			// 执行一直按下的事件回调
			for (uint8_t i = 0; i < keyList[key].onPressCb.number; i++)
				if (keyList[key].onPressCb.combineKey[i] == combineKey) // 符合组合键条件
					keyList[key].onPressCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnPressing);

			// 长按判定
			if (lastPressTime <= keyList[key].longPressTime && pressTime > keyList[key].longPressTime)
			{
				keyList[key].isLongPressed = 1;
				// 依次执行回调
				for (uint8_t i = 0; i < keyList[key].onLongCb.number; i++)
					if (keyList[key].onLongCb.combineKey[i] == combineKey) // 符合组合键条件
						keyList[key].onLongCb.func[i]((KeyType)(0x01 << key), combineKey, KeyEvent_OnLongPress);
			}
			else
				keyList[key].isLongPressed = 0;
		}
		/*******按键持续松开********/
		else
		{
			keyList[key].isClicked = 0;
			keyList[key].isLongPressed = 0;
			keyList[key].isUp = 0;
		}

		keyList[key].lastState = thisState; // 记录按键状态
	}

	lastUpdateTime = presentTime; // 记录更新事件
}

void RC_LostCallback()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5RxBuf, sizeof(usart5RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Usart1RxBuf, sizeof(Usart1RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	rc_true_flag++;
}

void Judge_UpdateKeys(void)
{
	rcInfo.kb.key_code = itInfo.kb.key_code;
	rcInfo.mouse.l = itInfo.mouse.left;
	rcInfo.mouse.r = itInfo.mouse.right;
	rcInfo.mouse.x = itInfo.mouse.x;
	rcInfo.mouse.y = itInfo.mouse.y;
	rcInfo.mouse.z = itInfo.mouse.z;
}
#include "vision.h"
// 遥控任务回调
extern uint8_t Vision_Mode;
uint8_t flagRC = 1;
extern uint8_t fastMode; // 快速模式

void Task_RC_Callback()
{
	// 更新按键状态
	RC_UpdateKeys();

	/**********特殊情况处理*********************/
	if (rcInfo.right == 2) // 遥控器右拨码开关向下，急停
	{

		HAL_Delay(1);
		USER_CAN_SetMotorCurrent(&hfdcan3, 0x1FF, 0, 0, 0, 0);
		USER_CAN_SetMotorCurrent(&hfdcan3, 0x200, 0, 0, 0, 0); // 防止邮箱刚刚塞满
		USER_CAN_SetMotorCurrent(&hfdcan2, 0x200, 0, 0, 0, 0); // 关断电机
		HAL_Delay(1);
		USER_CAN_SetMotorCurrent(&hfdcan2, 0x141, 0x8000, 0,
								 0, 0);
		USER_CAN_SetMotorCurrent(&hfdcan2, 0x142, 0x8000, 0,
								 0, 0);
		HAL_Delay(1);
		USER_CAN_SetMotorCurrent(&hfdcan2, 0x143, 0x8000, 0,
								 0, 0);
		USER_CAN_SetMotorCurrent(&hfdcan2, 0x144, 0x8000, 0,
								 0, 0);
		HAL_Delay(1);
		USER_CAN_SetMotorCurrent(&hfdcan2, 0x200, 0, 0, 0, 0);

		USER_CAN_SetMotorCurrent(&hfdcan2, 0x141, 0x8000, 0,
								 0, 0);
		USER_CAN_SetMotorCurrent(&hfdcan2, 0x142, 0x8000, 0,
								 0, 0);
		HAL_Delay(1);
		USER_CAN_SetMotorCurrent(&hfdcan2, 0x143, 0x8000, 0,
								 0, 0);
		USER_CAN_SetMotorCurrent(&hfdcan2, 0x144, 0x8000, 0,
								 0, 0);
		STOPFLAG = 1;
		Rs485_Trans();

		osThreadResume(ErrorTaskHandle); // 恢复错误任务 饿死其他任务
	}

	// 坏掉的遥控器死区
	if (rcInfo.wheel < 20 &&rcInfo.wheel > -20)
	{
		flagRC = 1;
	}
}

/************************freertos任务****************************/
#ifdef EN_RC_TASK // 使能任务
void OS_RcCallback(void const *argument)
{
	for (;;)
	{
		rc_true_flag++;
		if(rc_true_flag > 50)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5RxBuf, sizeof(usart5RxBuf));
			__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
		}

		if (rc_true_flag >= 100)
		{
			Judge_UpdateKeys();
		}
		
		Task_RC_Callback();
		osDelay(15);
	}
}
#endif
