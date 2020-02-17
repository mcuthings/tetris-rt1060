/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  Standard C Included Files */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "emwin_support.h"

#include "GUI.h"
#include "GUIDRV_Lin.h"
#include "BUTTON.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "clock_config.h"
#include "fsl_gpt.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FIELD_WIDTH  12
#define FIELD_HEIGHT 22
#define MINO_WIDTH  4
#define MINO_HEIGHT 4
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_EnableLcdInterrupt(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
bool gptIsrFlag;
char field[FIELD_HEIGHT][FIELD_WIDTH];
char dispBuffer[FIELD_HEIGHT][FIELD_WIDTH]={0};
GUI_MEMDEV_Handle hMem;

int minoX=5, minoY=1;
uint32_t minoType=0, minoAngle=0;
int cell_xSize = LCD_HEIGHT/12;
int cell_ySize = LCD_WIDTH/22;
/*******************************************************************************
 * Code
 ******************************************************************************/
void GPT1_IRQHandler(void)
{
    /* GPTタイマーのアウトプットコンペアの割込みフラグをクリアする*/
    GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare1Flag);

    /* gptIsrFlagを立てる */
    gptIsrFlag = true;

/*　ここから下はARMのエラッタに対する修正コードです。　*/
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F, Cortex-M7, Cortex-M7F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U || __CORTEX_M == 7U)
    __DSB();
#endif
}

/* Initialize the LCD_DISP. */
void BOARD_InitLcd(void)
{
    volatile uint32_t i = 0x100U;

    gpio_pin_config_t config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* Reset the LCD. */
    GPIO_PinInit(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, &config);

    GPIO_WritePinOutput(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);

    while (i--)
    {
    }

    GPIO_WritePinOutput(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);

    /* Backlight. */
    config.outputLogic = 1;
    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);

    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);
}

void BOARD_InitLcdifPixelClock(void)
{
    /*
     * The desired output frame rate is 60Hz. So the pixel clock frequency is:
     * (480 + 41 + 4 + 18) * (272 + 10 + 4 + 2) * 60 = 9.2M.
     * Here set the LCDIF pixel clock to 9.3M.
     */

    /*
     * Initialize the Video PLL.
     * Video PLL output clock is OSC24M * (loopDivider + (denominator / numerator)) / postDivider = 93MHz.
     */
    clock_video_pll_config_t config = {
        .loopDivider = 31,
        .postDivider = 8,
        .numerator   = 0,
        .denominator = 0,
    };

    CLOCK_InitVideoPll(&config);

    /*
     * 000 derive clock from PLL2
     * 001 derive clock from PLL3 PFD3
     * 010 derive clock from PLL5
     * 011 derive clock from PLL2 PFD0
     * 100 derive clock from PLL2 PFD1
     * 101 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);

    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 4);

    CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);
}

enum{
	MINO_TYPE_I,
	MINO_TYPE_O,
	MINO_TYPE_S,
	MINO_TYPE_Z,
	MINO_TYPE_J,
	MINO_TYPE_L,
	MINO_TYPE_T,
	MINO_TYPE_MAX
};

enum{
	MINO_ANGLE_0,
	MINO_ANGLE_90,
	MINO_ANGLE_180,
	MINO_ANGLE_270,
	MINO_ANGLE_MAX
};

enum {
	MINO_COLOR_GRAY=1,
	MINO_COLOR_LIGHT_BLUE,
	MINO_COLOR_YELLOW,
	MINO_COLOR_GREEN,
	MINO_COLOR_RED,
	MINO_COLOR_BLUE,
	MINO_COLOR_ORANGE,
	MINO_COLOR_MAGENTA,
	MINO_COLOR_MAX
};

char minoShapes[MINO_TYPE_MAX][MINO_ANGLE_MAX][MINO_WIDTH][MINO_HEIGHT] = {
		{//MINO_TYPE_I, LIGHT_BLUE
			//MINO_ANGLE_0,
			{
					{0, 2, 0, 0},
					{0, 2, 0, 0},
					{0, 2, 0, 0},
					{0, 2, 0, 0},
			},
				//MINO_ANGLE_90,
			{
					{0, 0, 0, 0},
					{0, 0, 0, 0},
					{2, 2, 2, 2},
					{0, 0, 0, 0},
			},
				//MINO_ANGLE_180,
			{
					{0, 0, 2 ,0},
					{0, 0, 2, 0},
					{0, 0, 2, 0},
					{0, 0, 2, 0},
			},
				//MINO_ANGLE_270,
			{
					{0, 0 ,0 ,0},
					{2, 2 ,2 ,2},
					{0, 0 ,0 ,0},
					{0, 0 ,0 ,0},
			},
		},
		//MINO_TYPE_O,YELLOW
		{
			//MINO_ANGLE_0,
			{
					{0, 0, 0 ,0},
					{0, 3, 3 ,0},
					{0, 3, 3 ,0},
					{0, 0, 0 ,0},
			},
			//MINO_ANGLE_90,
			{
					{0, 0, 0, 0},
					{0, 3, 3, 0},
					{0, 3, 3, 0},
					{0, 0, 0 ,0},
			},
			//MINO_ANGLE_180,
			{
					{0, 0, 0, 0},
					{0, 3, 3, 0},
					{0, 3, 3, 0},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_270,
			{
					{0, 0, 0, 0},
					{0, 3, 3, 0},
					{0, 3 ,3, 0},
					{0, 0, 0, 0},
			},
		},
		//MINO_TYPE_S,GREEN
		{
			//MINO_ANGLE_0,
			{
					{0, 0, 0, 0},
					{0, 4, 4, 0},
					{4, 4, 0, 0},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_90,
			{
					{0, 4, 0, 0},
					{0, 4, 4, 0},
					{0, 0, 4, 0},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_180,
			{
					{0, 0, 0 ,0},
					{0, 0, 4, 4},
					{0, 4, 4 ,0},
					{0, 0, 0 ,0},
			},
			//MINO_ANGLE_270,
			{
					{0, 0, 0, 0},
					{0, 4, 0, 0},
					{0, 4, 4, 0},
					{0, 0, 4, 0},
			},
		},
		//MINO_TYPE_Z,
		{
			//MINO_ANGLE_0,
			{
					{0, 0, 0, 0},
					{5, 5, 0, 0},
					{0, 5, 5, 0},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_90,
			{
					{0, 0, 0, 0},
					{0, 0, 5, 0},
					{0, 5, 5, 0},
					{0, 5, 0, 0},
			},
			//MINO_ANGLE_180,
			{
					{0, 0, 0, 0},
					{0, 5, 5, 0},
					{0, 0, 5, 5},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_270,
			{
					{0, 0, 5, 0},
					{0, 5, 5, 0},
					{0, 5, 0, 0},
					{0, 0, 0, 0},
			},
		},
		//MINO_TYPE_J,
		{
			//MINO_ANGLE_0,
			{
					{0, 0, 6, 0},
					{0, 0, 6, 0},
					{0, 6, 6, 0},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_90,
			{
					{0, 0, 0, 0},
					{6, 6, 6, 0},
					{0, 0, 6, 0},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_180,
			{
					{0, 0, 0, 0},
					{0, 6, 6, 0},
					{0, 6, 0, 0},
					{0, 6, 0, 0},
			},
			//MINO_ANGLE_270,
			{
					{0, 0, 0, 0},
					{0, 6, 0, 0},
					{0, 6, 6, 6},
					{0, 0, 0, 0},
			},
		},
		//MINO_TYPE_L,
		{
			//MINO_ANGLE_0,
			{
					{0, 7, 0, 0},
					{0, 7, 0, 0},
					{0, 7, 7, 0},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_90,
			{
					{0, 0, 0, 0},
				    {0, 0, 7, 0},
					{7, 7, 7, 0},
					{0, 0, 0, 0},
	        },
			//MINO_ANGLE_180,
			{
					{0, 0, 0, 0},
					{0, 7, 7, 0},
					{0, 0, 7, 0},
					{0, 0, 7, 0},
		    },
			//MINO_ANGLE_270,
			{
					{0, 0, 0, 0},
					{0, 7, 7, 7},
					{0, 7, 0, 0},
					{0, 0, 0, 0},
            },
		},
		//MINO_TYPE_T,
		{
			//MINO_ANGLE_0,
			{
					{0, 0, 0, 0},
					{8, 8, 8, 0},
					{0, 8, 0, 0},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_90,
			{
					{0, 0, 0, 0},
					{0, 8, 0, 0},
					{0, 8, 8, 0},
					{0, 8, 0, 0},
			},
			//MINO_ANGLE_180,
			{
					{0, 0, 0, 0},
					{0, 0, 8, 0},
					{0, 8, 8, 8},
					{0, 0, 0, 0},
			},
			//MINO_ANGLE_270,
			{
					{0, 0, 8, 0},
					{0, 8, 8, 0},
					{0, 0, 8, 0},
					{0, 0, 0, 0},
			},
		},

};

/* テトリミノが移動できるかどうかの関数 */
bool isHit(uint32_t _minoX, uint32_t _minoY,uint32_t _minoType, uint32_t _minoAngle){
	for(uint32_t y=0; y < MINO_HEIGHT; y++){
		for(uint32_t x=0; x <MINO_WIDTH; x++){
			if(minoShapes[_minoType][_minoAngle][y][x]&&field[_minoY +y][_minoX +x])
				return true;
		}
	}
	return false;
}

/* 降ってくるテトリミノのランダム選択関数 */
void resetMino(){
	minoX = 5, minoY =0;

	minoType = rand() % MINO_TYPE_MAX;
	minoAngle = rand() % MINO_ANGLE_MAX;
}

void fieldDraw(int _color, int _y, int _x){
	switch(_color){
		case MINO_COLOR_GRAY:
			GUI_SetColor(GUI_GRAY);
			break;
		case MINO_COLOR_LIGHT_BLUE:
			GUI_SetColor(GUI_LIGHTBLUE);
			break;
		case MINO_COLOR_YELLOW:
			GUI_SetColor(GUI_YELLOW);
			break;
		case MINO_COLOR_GREEN:
			GUI_SetColor(GUI_GREEN);
			break;
		case MINO_COLOR_RED:
			GUI_SetColor(GUI_RED);
			break;
		case MINO_COLOR_BLUE:
			GUI_SetColor(GUI_BLUE);
			break;
		case MINO_COLOR_ORANGE:
			GUI_SetColor(GUI_ORANGE);
			break;
		case MINO_COLOR_MAGENTA:
			GUI_SetColor(GUI_MAGENTA);
			break;
		default:
			break;
	}
	GUI_FillRect(_y * cell_ySize, _x * cell_xSize, (_y * cell_ySize) + cell_ySize -2 , (_x * cell_xSize) + cell_xSize -2);
}

void display(){
	gptIsrFlag = false;
	/* Screen clear  */
	//PRINTF("\033[2J");   	//Escape Sequence Clear screen
	//system("cls");
	//PRINTF("\033[1;1f");	//Cursor moves to (1,1)
	GUI_MEMDEV_Select(hMem);
	GUI_Clear();
	memcpy(dispBuffer, field, sizeof(field));
	for (int y =0; y< MINO_HEIGHT; y++)
		for(int x =0; x < MINO_WIDTH; x++)
			dispBuffer[minoY +y][minoX+x] |= minoShapes[minoType][minoAngle][y][x];

	/* Drawing */
	for (int y =0; y < FIELD_HEIGHT; y++){
		for(int x=0; x<FIELD_WIDTH; x++){
			int color = 0;
		    if ((color = dispBuffer[y][x])){
		    	/* BMPイメージの描画は時間が掛かりすぎるのでボツ */
		    	//GUI_BMP_Draw(gray, y * cell_ySize, x * cell_xSize);

		    	/*GUI_FillRect()を関数化*/
		    	//GUI_FillRect(y * cell_ySize,x * cell_xSize, (y * cell_ySize) + cell_ySize -2 , (x * cell_xSize) + cell_xSize -2);
		    	fieldDraw(color, y, x);

		    	//PRINTF("◽");
		    }else{
				//PRINTF("  ");
		    }

		}
		//PRINTF("\n\r");
	}
	GUI_MEMDEV_Select(0);
	GUI_MEMDEV_CopyToLCD(hMem);
}




int main(void)
{
	gpt_config_t gpt_config;
	uint32_t gptFreq;
	gptIsrFlag = false;

	/* Board pin, clock, debug console init */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_InitI2C1Pins();
    BOARD_InitSemcPins();
    BOARD_BootClockRUN();
    BOARD_InitLcdifPixelClock();
    BOARD_InitDebugConsole();
    BOARD_InitLcd();

    /* emWin start */
    GUI_Init();

    WM_SetSize(WM_HBKWIN, LCD_WIDTH, LCD_HEIGHT);

    /* Solid color display */
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
    WM_Exec();

    hMem = GUI_MEMDEV_Create(0, 0, 480, 272);

    /***コンフィグ設定のデフォルト値
    *    config->clockSource = kGPT_ClockSource_Periph;
    *    config->divider = 1U;
    *    config->enableRunInStop = true;
    *    config->enableRunInWait = true;
    *    config->enableRunInDoze = false;
    *    config->enableRunInDbg = false;
    *    config->enableFreeRun = true;
    *    config->enableMode  = true;
    */
    /*　GPTタイマーのデフォルトコンフィグ設定の取得と初期化*/
        GPT_GetDefaultConfig(&gpt_config);
        GPT_Init(GPT1, &gpt_config);

    /* GPTタイマーのクロックソースのクロック設定*/
        GPT_SetClockDivider(GPT1, 3);

    /* GPT timerのタイマーのタイマー（時刻：アウトプットコンペア）設定 */
        gptFreq= CLOCK_GetFreq(kCLOCK_PerClk);
        gptFreq /=3;
        GPT_SetOutputCompareValue(GPT1, kGPT_OutputCompare_Channel1, gptFreq);

    /* GPTタイマーのアウトプットコンペア割込みをイネーブル */
        GPT_EnableInterrupts(GPT1, kGPT_OutputCompare1InterruptEnable);

    /* GPTタイマーの割込み監視をスタート */
        EnableIRQ(GPT1_IRQn);
    /* GPTタイマーをスタート　*/
        GPT_StartTimer(GPT1);

    for (int y=0; y< FIELD_HEIGHT; y++){
    	field[y][0] = field[y][FIELD_WIDTH-1] = 1;
    }
    for(int x=0; x < FIELD_WIDTH; x++)
    	field[FIELD_HEIGHT-1][x]=1;

    display();

    while(1){
    	char ch;
		//if (USART0->STAT&USART_STAT_RXRDY_MASK){
		if (LPUART1->STAT&LPUART_STAT_RDRF_MASK){
			switch(ch=DbgConsole_Getchar()){
				case 'a':
					if(!isHit(
						//minoX-1,	//uint32_t _minoX,
						minoX+1,	//uint32_t _minoX,
						minoY,		//uint32_t _minoY,
						minoType,	//uint32_t _minoType,
						minoAngle	//uint32_t _minoAngle)
						))
						//minoX--;
						minoX++;
					break;
				case 'd':
					if(!isHit(
						//minoX+1,	//uint32_t _minoX,
						minoX-1,	//uint32_t _minoX,
						minoY,		//uint32_t _minoY,
						minoType,	//uint32_t _minoType,
						minoAngle	//uint32_t _minoAngle)
						))
						//minoX++;
						minoX--;
					break;
				case 's':
					if(!isHit(
						minoX,		//uint32_t _minoX,
						minoY+1,	//uint32_t _minoY,
						minoType,	//uint32_t _minoType,
						minoAngle	//uint32_t _minoAngle)
						))
						minoY++;
					break;
				case 0x20:
					if(!isHit(
						minoX,		//uint32_t _minoX,
						minoY,		//uint32_t _minoY,
						minoType,	//uint32_t _minoType,
						(minoAngle+1) % MINO_ANGLE_MAX//uint32_t _minoAngle)
						))
						minoAngle = (minoAngle+1) % MINO_ANGLE_MAX;
					break;
				default: break;
			}
			display();
		}

    	/*　以下while文はmain()関数内に配置します。　GPTタイマー
    	* が１秒カウントすると全体のブロックを１段下に落とす。 */
        //if(utickIntFlag){
		if(gptIsrFlag){
			for (int y=0; y< FIELD_HEIGHT; y++){
				field[y][0] = field[y][FIELD_WIDTH-1] = 1;
			}

			for(int x=0; x < FIELD_WIDTH; x++)
				field[FIELD_HEIGHT-1][x]=1;

			if(isHit(
				minoX,//uint32_t _minoX,
				minoY+1,//uint32_t _minoY,
				minoType,//uint32_t _minoType,
				minoAngle//uint32_t _minoAngle
				)){
				for(uint32_t y=0; y < MINO_HEIGHT; y++){
					for(uint32_t x=0; x <MINO_WIDTH; x++){
					field[minoY + y][minoX + x] |= minoShapes[minoType][minoAngle][y][x];
					}
				}

				for (uint8_t y=0; y<FIELD_HEIGHT-1; y++){
					bool lineFill = true;
					for(uint8_t x=1; x<FIELD_WIDTH-1; x++){
						if(!field[y][x])
							lineFill=false;
					}
					if (lineFill){
					//for (uint8_t x=1; x<FIELD_WIDTH-1; x++)
					//　　　field[y][x]=0;
						for (uint8_t j = y; 0<j; j--)
							memcpy(field[j], field[j-1], FIELD_WIDTH);
					}
				}
					resetMino();
			}else{
				minoY++;
			}
			display();
		}
    }
}

