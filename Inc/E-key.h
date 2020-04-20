#ifndef E_key_H_
#define E_key_H_

#define ControlPort					GPIOA
#define IsConnectionUpPin 	LL_GPIO_PIN_1
#define BTStatusPin 				LL_GPIO_PIN_4
#define OpenLockBtnPin			LL_GPIO_PIN_5
#define CloseLocBtnkPin			LL_GPIO_PIN_6
#define BTPowerControlPin		LL_GPIO_PIN_7
#define BTModeControlPin		LL_GPIO_PIN_9

#define BTPowerOff		0
#define BTPowerOn			1
#define BTPowerReset	2

#define BTModeFullAT	0
#define BTModeData		1

#endif