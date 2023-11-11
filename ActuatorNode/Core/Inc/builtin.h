#ifndef __BUILT_IN_H__
#define __BUILT_IN_H__

#define NUM_MODE 				3								/* number of mode */
#define NUM_BUTTON				2								/* number of user button */
#define DEBOUNCE_DELAY			50U								/* software debounce delay */

typedef void (*LED_Blink)(void);
typedef void (*Short_Press)(void);
typedef void (*Long_Press)(void);
extern Short_Press Short_Press_Button[];
extern Long_Press Long_Press_Button[];

extern int isButtonPressed;
extern int LED_Blink_Current_Mode;
extern volatile int timeElapsed;
extern volatile int pressTime;
extern LED_Blink LED_Blink_Mode[];

void LED_Blink_Mode_Config(int p_mode);
void LED_Blink_Mode_0(void);
void LED_Blink_Mode_1(void);
void LED_Blink_Mode_2(void);
void Short_Press_Button_1(void);
void Short_Press_Button_2(void);
void Long_Press_Button_1(void);
void Long_Press_Button_2(void);
void Buttons_Check(void);
void Monitor_Show();
int checkButtonInMode();


#endif /* __BUILT_IN_H__ */
