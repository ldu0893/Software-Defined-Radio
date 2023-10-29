/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NS 1000
#define FULL_BUF_SIZE 256
#define HALF_BUF_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
volatile uint32_t adc_val[FULL_BUF_SIZE];
volatile uint32_t dac_val[FULL_BUF_SIZE];

uint32_t* in_buf_ptr;
uint32_t* out_buf_ptr;
uint32_t out_out_ptr = 0;

int flag = 0;
int counter = 0;
int counter_counter = 0;


float Wave_LUT[NS] = {
  
     0        ,  0.00628314,  0.01256604,  0.01884844,  0.0251301 ,
  0.03141076,  0.03769018,  0.04396812,  0.05024432,  0.05651853,
  0.06279052,  0.06906003,  0.07532681,  0.08159061,  0.0878512 ,
  0.09410831,  0.10036171,  0.10661115,  0.11285638,  0.11909716,
  0.12533323,  0.13156436,  0.13779029,  0.14401078,  0.15022559,
  0.15643447,  0.16263717,  0.16883344,  0.17502306,  0.18120576,
  0.18738131,  0.19354947,  0.19970998,  0.20586261,  0.21200711,
  0.21814324,  0.22427076,  0.23038943,  0.236499  ,  0.24259923,
  0.24868989,  0.25477073,  0.26084151,  0.26690199,  0.27295194,
  0.27899111,  0.28501926,  0.29103617,  0.29704158,  0.30303527,
  0.30901699,  0.31498652,  0.32094361,  0.32688803,  0.33281954,
  0.33873792,  0.34464292,  0.35053432,  0.35641188,  0.36227537,
  0.36812455,  0.37395921,  0.3797791 ,  0.38558399,  0.39137367,
  0.39714789,  0.40290644,  0.40864907,  0.41437558,  0.42008573,
  0.42577929,  0.43145605,  0.43711577,  0.44275823,  0.44838322,
  0.4539905 ,  0.45957986,  0.46515108,  0.47070393,  0.4762382 ,
  0.48175367,  0.48725013,  0.49272734,  0.49818511,  0.5036232 ,
  0.50904142,  0.51443953,  0.51981734,  0.52517463,  0.53051118,
  0.53582679,  0.54112125,  0.54639435,  0.55164587,  0.55687562,
  0.56208338,  0.56726895,  0.57243213,  0.5775727 ,  0.58269048,
  0.58778525,  0.59285682,  0.59790498,  0.60292954,  0.6079303 ,
  0.61290705,  0.61785961,  0.62278778,  0.62769136,  0.63257016,
  0.63742399,  0.64225265,  0.64705596,  0.65183373,  0.65658576,
  0.66131187,  0.66601187,  0.67068558,  0.67533281,  0.67995338,
  0.68454711,  0.68911381,  0.69365331,  0.69816542,  0.70264997,
  0.70710678,  0.71153568,  0.71593648,  0.72030902,  0.72465313,
  0.72896863,  0.73325535,  0.73751312,  0.74174177,  0.74594115,
  0.75011107,  0.75425138,  0.75836192,  0.76244251,  0.76649301,
  0.77051324,  0.77450306,  0.7784623 ,  0.78239081,  0.78628843,
  0.79015501,  0.7939904 ,  0.79779444,  0.80156698,  0.80530789,
  0.80901699,  0.81269416,  0.81633925,  0.81995211,  0.8235326 ,
  0.82708057,  0.8305959 ,  0.83407843,  0.83752804,  0.84094458,
  0.84432793,  0.84767794,  0.85099448,  0.85427743,  0.85752666,
  0.86074203,  0.86392342,  0.8670707 ,  0.87018375,  0.87326245,
  0.87630668,  0.87931631,  0.88229123,  0.88523131,  0.88813645,
  0.89100652,  0.89384142,  0.89664104,  0.89940525,  0.90213396,
  0.90482705,  0.90748442,  0.91010597,  0.91269159,  0.91524117,
  0.91775463,  0.92023185,  0.92267274,  0.92507721,  0.92744515,
  0.92977649,  0.93207111,  0.93432894,  0.93654989,  0.93873386,
  0.94088077,  0.94299054,  0.94506308,  0.9470983 ,  0.94909614,
  0.95105652,  0.95297934,  0.95486454,  0.95671205,  0.95852179,
  0.96029369,  0.96202767,  0.96372368,  0.96538164,  0.96700149,
  0.96858316,  0.9701266 ,  0.97163173,  0.97309851,  0.97452687,
  0.97591676,  0.97726812,  0.9785809 ,  0.97985505,  0.98109052,
  0.98228725,  0.9834452 ,  0.98456433,  0.9856446 ,  0.98668594,
  0.98768834,  0.98865174,  0.98957612,  0.99046143,  0.99130763,
  0.9921147 ,  0.9928826 ,  0.99361131,  0.99430079,  0.99495102,
  0.99556196,  0.99613361,  0.99666593,  0.9971589 ,  0.99761251,
  0.99802673,  0.99840155,  0.99873696,  0.99903293,  0.99928947,
  0.99950656,  0.99968419,  0.99982235,  0.99992104,  0.99998026,
  1        ,  0.99998026,  0.99992104,  0.99982235,  0.99968419,
  0.99950656,  0.99928947,  0.99903293,  0.99873696,  0.99840155,
  0.99802673,  0.99761251,  0.9971589 ,  0.99666593,  0.99613361,
  0.99556196,  0.99495102,  0.99430079,  0.99361131,  0.9928826 ,
  0.9921147 ,  0.99130763,  0.99046143,  0.98957612,  0.98865174,
  0.98768834,  0.98668594,  0.9856446 ,  0.98456433,  0.9834452 ,
  0.98228725,  0.98109052,  0.97985505,  0.9785809 ,  0.97726812,
  0.97591676,  0.97452687,  0.97309851,  0.97163173,  0.9701266 ,
  0.96858316,  0.96700149,  0.96538164,  0.96372368,  0.96202767,
  0.96029369,  0.95852179,  0.95671205,  0.95486454,  0.95297934,
  0.95105652,  0.94909614,  0.9470983 ,  0.94506308,  0.94299054,
  0.94088077,  0.93873386,  0.93654989,  0.93432894,  0.93207111,
  0.92977649,  0.92744515,  0.92507721,  0.92267274,  0.92023185,
  0.91775463,  0.91524117,  0.91269159,  0.91010597,  0.90748442,
  0.90482705,  0.90213396,  0.89940525,  0.89664104,  0.89384142,
  0.89100652,  0.88813645,  0.88523131,  0.88229123,  0.87931631,
  0.87630668,  0.87326245,  0.87018375,  0.8670707 ,  0.86392342,
  0.86074203,  0.85752666,  0.85427743,  0.85099448,  0.84767794,
  0.84432793,  0.84094458,  0.83752804,  0.83407843,  0.8305959 ,
  0.82708057,  0.8235326 ,  0.81995211,  0.81633925,  0.81269416,
  0.80901699,  0.80530789,  0.80156698,  0.79779444,  0.7939904 ,
  0.79015501,  0.78628843,  0.78239081,  0.7784623 ,  0.77450306,
  0.77051324,  0.76649301,  0.76244251,  0.75836192,  0.75425138,
  0.75011107,  0.74594115,  0.74174177,  0.73751312,  0.73325535,
  0.72896863,  0.72465313,  0.72030902,  0.71593648,  0.71153568,
  0.70710678,  0.70264997,  0.69816542,  0.69365331,  0.68911381,
  0.68454711,  0.67995338,  0.67533281,  0.67068558,  0.66601187,
  0.66131187,  0.65658576,  0.65183373,  0.64705596,  0.64225265,
  0.63742399,  0.63257016,  0.62769136,  0.62278778,  0.61785961,
  0.61290705,  0.6079303 ,  0.60292954,  0.59790498,  0.59285682,
  0.58778525,  0.58269048,  0.5775727 ,  0.57243213,  0.56726895,
  0.56208338,  0.55687562,  0.55164587,  0.54639435,  0.54112125,
  0.53582679,  0.53051118,  0.52517463,  0.51981734,  0.51443953,
  0.50904142,  0.5036232 ,  0.49818511,  0.49272734,  0.48725013,
  0.48175367,  0.4762382 ,  0.47070393,  0.46515108,  0.45957986,
  0.4539905 ,  0.44838322,  0.44275823,  0.43711577,  0.43145605,
  0.42577929,  0.42008573,  0.41437558,  0.40864907,  0.40290644,
  0.39714789,  0.39137367,  0.38558399,  0.3797791 ,  0.37395921,
  0.36812455,  0.36227537,  0.35641188,  0.35053432,  0.34464292,
  0.33873792,  0.33281954,  0.32688803,  0.32094361,  0.31498652,
  0.30901699,  0.30303527,  0.29704158,  0.29103617,  0.28501926,
  0.27899111,  0.27295194,  0.26690199,  0.26084151,  0.25477073,
  0.24868989,  0.24259923,  0.236499  ,  0.23038943,  0.22427076,
  0.21814324,  0.21200711,  0.20586261,  0.19970998,  0.19354947,
  0.18738131,  0.18120576,  0.17502306,  0.16883344,  0.16263717,
  0.15643447,  0.15022559,  0.14401078,  0.13779029,  0.13156436,
  0.12533323,  0.11909716,  0.11285638,  0.10661115,  0.10036171,
  0.09410831,  0.0878512 ,  0.08159061,  0.07532681,  0.06906003,
  0.06279052,  0.05651853,  0.05024432,  0.04396812,  0.03769018,
  0.03141076,  0.0251301 ,  0.01884844,  0.01256604,  0.00628314,
  0        , -0.00628314, -0.01256604, -0.01884844, -0.0251301 ,
 -0.03141076, -0.03769018, -0.04396812, -0.05024432, -0.05651853,
 -0.06279052, -0.06906003, -0.07532681, -0.08159061, -0.0878512 ,
 -0.09410831, -0.10036171, -0.10661115, -0.11285638, -0.11909716,
 -0.12533323, -0.13156436, -0.13779029, -0.14401078, -0.15022559,
 -0.15643447, -0.16263717, -0.16883344, -0.17502306, -0.18120576,
 -0.18738131, -0.19354947, -0.19970998, -0.20586261, -0.21200711,
 -0.21814324, -0.22427076, -0.23038943, -0.236499  , -0.24259923,
 -0.24868989, -0.25477073, -0.26084151, -0.26690199, -0.27295194,
 -0.27899111, -0.28501926, -0.29103617, -0.29704158, -0.30303527,
 -0.30901699, -0.31498652, -0.32094361, -0.32688803, -0.33281954,
 -0.33873792, -0.34464292, -0.35053432, -0.35641188, -0.36227537,
 -0.36812455, -0.37395921, -0.3797791 , -0.38558399, -0.39137367,
 -0.39714789, -0.40290644, -0.40864907, -0.41437558, -0.42008573,
 -0.42577929, -0.43145605, -0.43711577, -0.44275823, -0.44838322,
 -0.4539905 , -0.45957986, -0.46515108, -0.47070393, -0.4762382 ,
 -0.48175367, -0.48725013, -0.49272734, -0.49818511, -0.5036232 ,
 -0.50904142, -0.51443953, -0.51981734, -0.52517463, -0.53051118,
 -0.53582679, -0.54112125, -0.54639435, -0.55164587, -0.55687562,
 -0.56208338, -0.56726895, -0.57243213, -0.5775727 , -0.58269048,
 -0.58778525, -0.59285682, -0.59790498, -0.60292954, -0.6079303 ,
 -0.61290705, -0.61785961, -0.62278778, -0.62769136, -0.63257016,
 -0.63742399, -0.64225265, -0.64705596, -0.65183373, -0.65658576,
 -0.66131187, -0.66601187, -0.67068558, -0.67533281, -0.67995338,
 -0.68454711, -0.68911381, -0.69365331, -0.69816542, -0.70264997,
 -0.70710678, -0.71153568, -0.71593648, -0.72030902, -0.72465313,
 -0.72896863, -0.73325535, -0.73751312, -0.74174177, -0.74594115,
 -0.75011107, -0.75425138, -0.75836192, -0.76244251, -0.76649301,
 -0.77051324, -0.77450306, -0.7784623 , -0.78239081, -0.78628843,
 -0.79015501, -0.7939904 , -0.79779444, -0.80156698, -0.80530789,
 -0.80901699, -0.81269416, -0.81633925, -0.81995211, -0.8235326 ,
 -0.82708057, -0.8305959 , -0.83407843, -0.83752804, -0.84094458,
 -0.84432793, -0.84767794, -0.85099448, -0.85427743, -0.85752666,
 -0.86074203, -0.86392342, -0.8670707 , -0.87018375, -0.87326245,
 -0.87630668, -0.87931631, -0.88229123, -0.88523131, -0.88813645,
 -0.89100652, -0.89384142, -0.89664104, -0.89940525, -0.90213396,
 -0.90482705, -0.90748442, -0.91010597, -0.91269159, -0.91524117,
 -0.91775463, -0.92023185, -0.92267274, -0.92507721, -0.92744515,
 -0.92977649, -0.93207111, -0.93432894, -0.93654989, -0.93873386,
 -0.94088077, -0.94299054, -0.94506308, -0.9470983 , -0.94909614,
 -0.95105652, -0.95297934, -0.95486454, -0.95671205, -0.95852179,
 -0.96029369, -0.96202767, -0.96372368, -0.96538164, -0.96700149,
 -0.96858316, -0.9701266 , -0.97163173, -0.97309851, -0.97452687,
 -0.97591676, -0.97726812, -0.9785809 , -0.97985505, -0.98109052,
 -0.98228725, -0.9834452 , -0.98456433, -0.9856446 , -0.98668594,
 -0.98768834, -0.98865174, -0.98957612, -0.99046143, -0.99130763,
 -0.9921147 , -0.9928826 , -0.99361131, -0.99430079, -0.99495102,
 -0.99556196, -0.99613361, -0.99666593, -0.9971589 , -0.99761251,
 -0.99802673, -0.99840155, -0.99873696, -0.99903293, -0.99928947,
 -0.99950656, -0.99968419, -0.99982235, -0.99992104, -0.99998026,
 -1        , -0.99998026, -0.99992104, -0.99982235, -0.99968419,
 -0.99950656, -0.99928947, -0.99903293, -0.99873696, -0.99840155,
 -0.99802673, -0.99761251, -0.9971589 , -0.99666593, -0.99613361,
 -0.99556196, -0.99495102, -0.99430079, -0.99361131, -0.9928826 ,
 -0.9921147 , -0.99130763, -0.99046143, -0.98957612, -0.98865174,
 -0.98768834, -0.98668594, -0.9856446 , -0.98456433, -0.9834452 ,
 -0.98228725, -0.98109052, -0.97985505, -0.9785809 , -0.97726812,
 -0.97591676, -0.97452687, -0.97309851, -0.97163173, -0.9701266 ,
 -0.96858316, -0.96700149, -0.96538164, -0.96372368, -0.96202767,
 -0.96029369, -0.95852179, -0.95671205, -0.95486454, -0.95297934,
 -0.95105652, -0.94909614, -0.9470983 , -0.94506308, -0.94299054,
 -0.94088077, -0.93873386, -0.93654989, -0.93432894, -0.93207111,
 -0.92977649, -0.92744515, -0.92507721, -0.92267274, -0.92023185,
 -0.91775463, -0.91524117, -0.91269159, -0.91010597, -0.90748442,
 -0.90482705, -0.90213396, -0.89940525, -0.89664104, -0.89384142,
 -0.89100652, -0.88813645, -0.88523131, -0.88229123, -0.87931631,
 -0.87630668, -0.87326245, -0.87018375, -0.8670707 , -0.86392342,
 -0.86074203, -0.85752666, -0.85427743, -0.85099448, -0.84767794,
 -0.84432793, -0.84094458, -0.83752804, -0.83407843, -0.8305959 ,
 -0.82708057, -0.8235326 , -0.81995211, -0.81633925, -0.81269416,
 -0.80901699, -0.80530789, -0.80156698, -0.79779444, -0.7939904 ,
 -0.79015501, -0.78628843, -0.78239081, -0.7784623 , -0.77450306,
 -0.77051324, -0.76649301, -0.76244251, -0.75836192, -0.75425138,
 -0.75011107, -0.74594115, -0.74174177, -0.73751312, -0.73325535,
 -0.72896863, -0.72465313, -0.72030902, -0.71593648, -0.71153568,
 -0.70710678, -0.70264997, -0.69816542, -0.69365331, -0.68911381,
 -0.68454711, -0.67995338, -0.67533281, -0.67068558, -0.66601187,
 -0.66131187, -0.65658576, -0.65183373, -0.64705596, -0.64225265,
 -0.63742399, -0.63257016, -0.62769136, -0.62278778, -0.61785961,
 -0.61290705, -0.6079303 , -0.60292954, -0.59790498, -0.59285682,
 -0.58778525, -0.58269048, -0.5775727 , -0.57243213, -0.56726895,
 -0.56208338, -0.55687562, -0.55164587, -0.54639435, -0.54112125,
 -0.53582679, -0.53051118, -0.52517463, -0.51981734, -0.51443953,
 -0.50904142, -0.5036232 , -0.49818511, -0.49272734, -0.48725013,
 -0.48175367, -0.4762382 , -0.47070393, -0.46515108, -0.45957986,
 -0.4539905 , -0.44838322, -0.44275823, -0.43711577, -0.43145605,
 -0.42577929, -0.42008573, -0.41437558, -0.40864907, -0.40290644,
 -0.39714789, -0.39137367, -0.38558399, -0.3797791 , -0.37395921,
 -0.36812455, -0.36227537, -0.35641188, -0.35053432, -0.34464292,
 -0.33873792, -0.33281954, -0.32688803, -0.32094361, -0.31498652,
 -0.30901699, -0.30303527, -0.29704158, -0.29103617, -0.28501926,
 -0.27899111, -0.27295194, -0.26690199, -0.26084151, -0.25477073,
 -0.24868989, -0.24259923, -0.236499  , -0.23038943, -0.22427076,
 -0.21814324, -0.21200711, -0.20586261, -0.19970998, -0.19354947,
 -0.18738131, -0.18120576, -0.17502306, -0.16883344, -0.16263717,
 -0.15643447, -0.15022559, -0.14401078, -0.13779029, -0.13156436,
 -0.12533323, -0.11909716, -0.11285638, -0.10661115, -0.10036171,
 -0.09410831, -0.0878512 , -0.08159061, -0.07532681, -0.06906003,
 -0.06279052, -0.05651853, -0.05024432, -0.04396812, -0.03769018,
 -0.03141076, -0.0251301 , -0.01884844, -0.01256604, -0.00628314  
       };

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 200000 Hz

* 0 Hz - 16000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 0 dB

*/

#define FILTER_TAP_NUM 5

static double filter_taps[FILTER_TAP_NUM] = {
  // -4.4408920985006264e-17,
  // 4.4408920985006264e-17,
  1,0,0,0,0
  // 4.4408920985006264e-17,
  // -4.4408920985006264e-17
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t fir(int);

int numbytes = 0;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  //first half of adc buffer is full
  in_buf_ptr = &adc_val[0];
  out_buf_ptr = &dac_val[HALF_BUF_SIZE];// + HALF_BUF_SIZE;

  //numbytes++;
  //if (numbytes > HALF_BUF_SIZE - 1) numbytes=0;
  

  //uint32_t test = fir((int)in_buf_ptr[numbytes]);
  //out_buf_ptr[numbytes] = test;

  flag=1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  //second half of adc buffer is full
  in_buf_ptr = &adc_val[HALF_BUF_SIZE];// + HALF_BUF_SIZE;
  out_buf_ptr = &dac_val[0];
  
  //numbytes = FULL_BUF_SIZE - __HAL_DMA_GET_COUNTER(hadc1.hdmarx);
  //numbytes++;
  //if (numbytes > HALF_BUF_SIZE - 1) numbytes=0;

  //uint32_t test = fir((int)in_buf_ptr[numbytes]);
  //out_buf_ptr[numbytes] = test;

  flag=1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim7) {
    //alpha+=0.05;
    //if (alpha > 1) alpha = 0;
  }

  if (htim == &htim6) {
    //out_out_ptr++;
    //if (out_out_ptr > HALF_BUF_SIZE-1) out_out_ptr = 0;
    // process_data();
  }
}

float ema_alpha = 0.1f;
float ema_out = 0;
void ema_lpf(float in) {
  ema_out = ema_alpha * in + (1.0f - ema_alpha) * ema_out;
}

float firdata[FILTER_TAP_NUM];
int firptr[FILTER_TAP_NUM];
int fir_w_ptr = 0;

uint32_t fir(int in) {
  float in_f = (float)in;
  float fir_out = 0;
  for (int i=0;i<FILTER_TAP_NUM;i++) {
    fir_out += filter_taps[firptr[i]] * firdata[i];
    firptr[i]++;
  }
  firdata[fir_w_ptr] = in_f;
  firptr[fir_w_ptr] = 0;
  fir_w_ptr++;
  if (fir_w_ptr == FILTER_TAP_NUM) fir_w_ptr = 0;

  return (uint32_t) fir_out;
}

void process_data() {
  for (int i=0;i<HALF_BUF_SIZE;i++) {
    //out_buf_ptr[i] = in_buf_ptr[i];

    float in = ((float) in_buf_ptr[i]);
    ema_lpf(in);
    out_buf_ptr[i] = (uint32_t)(ema_out);

    //if (bufIndex > 0) bufIndex--;
    //else bufIndex = HALF_BUF_SIZE - 1;
    //bufIndex = 0;

    //fir_out += filter_taps[i] * (float)(in_buf_ptr[bufIndex]);

    //float temp = ((float)(in_buf_ptr[i] * 1.0/2048.0) - 1.0);
    //out_buf_ptr[i]=(uint32_t)(temp * 2047+2048);
    //out_buf_ptr[i]=((float)(in_buf_ptr[i] * 1.0/2048.0 - 1) * (Wave_LUT[counter]))*2047 + 2048;
    //out_buf_ptr[i] = Wave_LUT[counter]*2047 + 2048;
    //counter_counter++;
    //if (counter_counter == 1) {
      //counter++;
      //counter_counter = 0;
    //}
  }
  //out_buf_ptr[out_out_ptr] = (fir_out / 3000000000 * 4096);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_LPUART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_val, FULL_BUF_SIZE);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) dac_val, FULL_BUF_SIZE, DAC_ALIGN_12B_R);

  //arm_conv_f32(a, 5, b, 5, c);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
      if (flag) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        flag = 0;
        process_data();
      }
    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    //HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 400;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
