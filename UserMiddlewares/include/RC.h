#ifndef _RC_H_
#define _RC_H_
#include "stdint.h"
#include "usart.h"
#include "main.h"
#include "stdbool.h"
#define KEY_NUM 18
#define MAX_KEY_CALLBACK_NUM 10

//������λ��ID������Ӧ�ṹ����keyList�е��±�
typedef enum _KeyType
{
	//���ü��̰�����˳������ң����������ͬ
	Key_W=1<<0,
	Key_S=1<<1,
	Key_A=1<<2,
	Key_D=1<<3,
	Key_Shift=1<<4,
	Key_Ctrl=1<<5,
	Key_Q=1<<6,
	Key_E=1<<7,
	Key_R=1<<8,
	Key_F=1<<9,
	Key_G=1<<10,
	Key_Z=1<<11,
	Key_X=1<<12,
	Key_C=1<<13,
	Key_V=1<<14,
	Key_B=1<<15,
	//������Ҽ�
	Key_Left=1<<16,
	Key_Right=1<<17,
	Key_All=0x3ffff
}KeyType;

//�����¼�����
typedef enum _KeyEventType
{
	KeyEvent_OnClick,
	KeyEvent_OnLongPress,
	KeyEvent_OnDown,
	KeyEvent_OnUp,
	KeyEvent_OnPressing//ֻҪ���¾ͻ���ÿ��������ڵ���һ��
}KeyEventType;

//��ϼ�����
typedef enum _KeyCombineType
{
	CombineKey_None,
	CombineKey_Ctrl,
	CombineKey_Shift
}KeyCombineType;

//�ص�����
typedef void (*KeyCallbackFunc)(KeyType,KeyCombineType,KeyEventType);

/**
  * @brief  ң�����ݽṹ��
  */
typedef struct 
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t left; //上中下132
  uint8_t right;//上中下132
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
}RC_TypeDef;

typedef struct 
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
	int16_t wheel;
  /* left and right lever information */
  uint8_t pause;
	uint8_t left;
  uint8_t right;
	uint8_t sw;
	uint8_t trigger;
	
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t left;
    uint8_t right;
		uint8_t middle;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
}Image_Trans_TypeDef;

//�����ṹ�壬���ڼ������/���İ����¼�
typedef struct _Key
{
	//��Ҫ���õĲ���
	uint16_t clickDelayTime;//���¶�ò��㵥��һ��
	uint16_t longPressTime;//���¶�ò��㳤��
	
	//����ʹ�õĲ��������ڶ�Ӧ������Ч��һ˲��Ϊ1
	uint8_t isClicked;
	uint8_t isLongPressed;
	uint8_t isUp;
	uint8_t isPressing;
	
	//�ص�
	struct
	{
		KeyCombineType combineKey[MAX_KEY_CALLBACK_NUM];//��ϼ������б�
		KeyCallbackFunc func[MAX_KEY_CALLBACK_NUM];//�ص������б�
		uint8_t number;//��ע��Ļص�����
	}onClickCb,onLongCb,onDownCb,onUpCb,onPressCb;//���ְ����¼��Ļص�
	
	//�м����
	uint8_t lastState;//1/0Ϊ����/�ɿ�
	uint32_t startPressTime;
}Key;

extern RC_TypeDef rcInfo;
extern uint8_t stop_flag_t ;

//RC��ʼ��
void RC_Init(void);
//ע��һ�������ص�
void RC_Register(uint32_t key,KeyCombineType combine,KeyEventType event,KeyCallbackFunc func);
void RC_LostCallback(void);

#endif


