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
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NS 2304
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

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint32_t adc_val[FULL_BUF_SIZE];
uint32_t dac_val[FULL_BUF_SIZE];

uint32_t* in_buf_ptr;
uint32_t* out_buf_ptr;

int flag = 0;
int counter = 0;

float32_t a[5] = {1,2,3,4,5};
float32_t b[5] = {1,2,3,4,5};
float32_t c[9];

uint32_t Wave_LUT[NS] = {
2048, 2053, 2059, 2064, 2070, 2075, 2081, 2087, 2092, 2098, 2103, 2109, 2115, 2120, 2126,
2131, 2137, 2142, 2148, 2154, 2159, 2165, 2170, 2176, 2181, 2187, 2193, 2198, 2204, 2209,
2215, 2220, 2226, 2232, 2237, 2243, 2248, 2254, 2259, 2265, 2271, 2276, 2282, 2287, 2293,
2298, 2304, 2309, 2315, 2320, 2326, 2331, 2337, 2343, 2348, 2354, 2359, 2365, 2370, 2376,
2381, 2387, 2392, 2398, 2403, 2409, 2414, 2420, 2425, 2431, 2436, 2442, 2447, 2453, 2458,
2464, 2469, 2474, 2480, 2485, 2491, 2496, 2502, 2507, 2513, 2518, 2524, 2529, 2534, 2540,
2545, 2551, 2556, 2561, 2567, 2572, 2578, 2583, 2588, 2594, 2599, 2605, 2610, 2615, 2621,
2626, 2631, 2637, 2642, 2647, 2653, 2658, 2663, 2669, 2674, 2679, 2685, 2690, 2695, 2701,
2706, 2711, 2716, 2722, 2727, 2732, 2738, 2743, 2748, 2753, 2759, 2764, 2769, 2774, 2779,
2785, 2790, 2795, 2800, 2805, 2811, 2816, 2821, 2826, 2831, 2837, 2842, 2847, 2852, 2857,
2862, 2867, 2872, 2878, 2883, 2888, 2893, 2898, 2903, 2908, 2913, 2918, 2923, 2928, 2933,
2938, 2943, 2948, 2953, 2958, 2963, 2968, 2973, 2978, 2983, 2988, 2993, 2998, 3003, 3008,
3013, 3018, 3023, 3028, 3033, 3038, 3042, 3047, 3052, 3057, 3062, 3067, 3072, 3076, 3081,
3086, 3091, 3096, 3101, 3105, 3110, 3115, 3120, 3124, 3129, 3134, 3139, 3143, 3148, 3153,
3157, 3162, 3167, 3171, 3176, 3181, 3185, 3190, 3195, 3199, 3204, 3209, 3213, 3218, 3222,
3227, 3231, 3236, 3241, 3245, 3250, 3254, 3259, 3263, 3268, 3272, 3277, 3281, 3286, 3290,
3294, 3299, 3303, 3308, 3312, 3316, 3321, 3325, 3330, 3334, 3338, 3343, 3347, 3351, 3356,
3360, 3364, 3368, 3373, 3377, 3381, 3385, 3390, 3394, 3398, 3402, 3406, 3411, 3415, 3419,
3423, 3427, 3431, 3435, 3439, 3444, 3448, 3452, 3456, 3460, 3464, 3468, 3472, 3476, 3480,
3484, 3488, 3492, 3496, 3500, 3504, 3508, 3512, 3515, 3519, 3523, 3527, 3531, 3535, 3539,
3542, 3546, 3550, 3554, 3558, 3561, 3565, 3569, 3573, 3576, 3580, 3584, 3587, 3591, 3595,
3598, 3602, 3606, 3609, 3613, 3616, 3620, 3624, 3627, 3631, 3634, 3638, 3641, 3645, 3648,
3652, 3655, 3659, 3662, 3666, 3669, 3672, 3676, 3679, 3683, 3686, 3689, 3693, 3696, 3699,
3702, 3706, 3709, 3712, 3716, 3719, 3722, 3725, 3728, 3732, 3735, 3738, 3741, 3744, 3747,
3750, 3754, 3757, 3760, 3763, 3766, 3769, 3772, 3775, 3778, 3781, 3784, 3787, 3790, 3793,
3796, 3798, 3801, 3804, 3807, 3810, 3813, 3816, 3818, 3821, 3824, 3827, 3829, 3832, 3835,
3838, 3840, 3843, 3846, 3848, 3851, 3854, 3856, 3859, 3862, 3864, 3867, 3869, 3872, 3874,
3877, 3879, 3882, 3884, 3887, 3889, 3892, 3894, 3896, 3899, 3901, 3904, 3906, 3908, 3911,
3913, 3915, 3917, 3920, 3922, 3924, 3926, 3929, 3931, 3933, 3935, 3937, 3940, 3942, 3944,
3946, 3948, 3950, 3952, 3954, 3956, 3958, 3960, 3962, 3964, 3966, 3968, 3970, 3972, 3974,
3976, 3978, 3979, 3981, 3983, 3985, 3987, 3988, 3990, 3992, 3994, 3995, 3997, 3999, 4001,
4002, 4004, 4006, 4007, 4009, 4010, 4012, 4014, 4015, 4017, 4018, 4020, 4021, 4023, 4024,
4026, 4027, 4028, 4030, 4031, 4033, 4034, 4035, 4037, 4038, 4039, 4041, 4042, 4043, 4044,
4046, 4047, 4048, 4049, 4050, 4051, 4053, 4054, 4055, 4056, 4057, 4058, 4059, 4060, 4061,
4062, 4063, 4064, 4065, 4066, 4067, 4068, 4069, 4070, 4071, 4071, 4072, 4073, 4074, 4075,
4075, 4076, 4077, 4078, 4078, 4079, 4080, 4080, 4081, 4082, 4082, 4083, 4084, 4084, 4085,
4085, 4086, 4086, 4087, 4087, 4088, 4088, 4089, 4089, 4090, 4090, 4090, 4091, 4091, 4091,
4092, 4092, 4092, 4093, 4093, 4093, 4093, 4094, 4094, 4094, 4094, 4094, 4094, 4095, 4095,
4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4094,
4094, 4094, 4094, 4094, 4094, 4093, 4093, 4093, 4093, 4092, 4092, 4092, 4092, 4091, 4091,
4091, 4090, 4090, 4089, 4089, 4088, 4088, 4088, 4087, 4087, 4086, 4086, 4085, 4084, 4084,
4083, 4083, 4082, 4081, 4081, 4080, 4079, 4079, 4078, 4077, 4077, 4076, 4075, 4074, 4073,
4073, 4072, 4071, 4070, 4069, 4068, 4067, 4066, 4066, 4065, 4064, 4063, 4062, 4061, 4060,
4059, 4058, 4056, 4055, 4054, 4053, 4052, 4051, 4050, 4049, 4047, 4046, 4045, 4044, 4042,
4041, 4040, 4039, 4037, 4036, 4035, 4033, 4032, 4030, 4029, 4028, 4026, 4025, 4023, 4022,
4020, 4019, 4017, 4016, 4014, 4013, 4011, 4010, 4008, 4006, 4005, 4003, 4001, 4000, 3998,
3996, 3995, 3993, 3991, 3989, 3988, 3986, 3984, 3982, 3980, 3978, 3977, 3975, 3973, 3971,
3969, 3967, 3965, 3963, 3961, 3959, 3957, 3955, 3953, 3951, 3949, 3947, 3945, 3943, 3941,
3938, 3936, 3934, 3932, 3930, 3928, 3925, 3923, 3921, 3919, 3916, 3914, 3912, 3909, 3907,
3905, 3902, 3900, 3898, 3895, 3893, 3890, 3888, 3886, 3883, 3881, 3878, 3876, 3873, 3871,
3868, 3865, 3863, 3860, 3858, 3855, 3852, 3850, 3847, 3844, 3842, 3839, 3836, 3834, 3831,
3828, 3825, 3823, 3820, 3817, 3814, 3811, 3808, 3806, 3803, 3800, 3797, 3794, 3791, 3788,
3785, 3782, 3779, 3776, 3773, 3770, 3767, 3764, 3761, 3758, 3755, 3752, 3749, 3746, 3743,
3739, 3736, 3733, 3730, 3727, 3724, 3720, 3717, 3714, 3711, 3707, 3704, 3701, 3698, 3694,
3691, 3688, 3684, 3681, 3677, 3674, 3671, 3667, 3664, 3660, 3657, 3654, 3650, 3647, 3643,
3640, 3636, 3633, 3629, 3625, 3622, 3618, 3615, 3611, 3607, 3604, 3600, 3597, 3593, 3589,
3586, 3582, 3578, 3574, 3571, 3567, 3563, 3559, 3556, 3552, 3548, 3544, 3540, 3537, 3533,
3529, 3525, 3521, 3517, 3513, 3510, 3506, 3502, 3498, 3494, 3490, 3486, 3482, 3478, 3474,
3470, 3466, 3462, 3458, 3454, 3450, 3446, 3442, 3437, 3433, 3429, 3425, 3421, 3417, 3413,
3408, 3404, 3400, 3396, 3392, 3387, 3383, 3379, 3375, 3370, 3366, 3362, 3358, 3353, 3349,
3345, 3340, 3336, 3332, 3327, 3323, 3319, 3314, 3310, 3305, 3301, 3297, 3292, 3288, 3283,
3279, 3274, 3270, 3265, 3261, 3256, 3252, 3247, 3243, 3238, 3234, 3229, 3225, 3220, 3215,
3211, 3206, 3202, 3197, 3192, 3188, 3183, 3178, 3174, 3169, 3164, 3160, 3155, 3150, 3146,
3141, 3136, 3132, 3127, 3122, 3117, 3112, 3108, 3103, 3098, 3093, 3089, 3084, 3079, 3074,
3069, 3064, 3060, 3055, 3050, 3045, 3040, 3035, 3030, 3025, 3020, 3016, 3011, 3006, 3001,
2996, 2991, 2986, 2981, 2976, 2971, 2966, 2961, 2956, 2951, 2946, 2941, 2936, 2931, 2926,
2921, 2916, 2911, 2906, 2900, 2895, 2890, 2885, 2880, 2875, 2870, 2865, 2860, 2855, 2849,
2844, 2839, 2834, 2829, 2824, 2818, 2813, 2808, 2803, 2798, 2793, 2787, 2782, 2777, 2772,
2766, 2761, 2756, 2751, 2745, 2740, 2735, 2730, 2724, 2719, 2714, 2709, 2703, 2698, 2693,
2687, 2682, 2677, 2671, 2666, 2661, 2655, 2650, 2645, 2639, 2634, 2629, 2623, 2618, 2613,
2607, 2602, 2597, 2591, 2586, 2580, 2575, 2570, 2564, 2559, 2553, 2548, 2543, 2537, 2532,
2526, 2521, 2515, 2510, 2504, 2499, 2494, 2488, 2483, 2477, 2472, 2466, 2461, 2455, 2450,
2444, 2439, 2433, 2428, 2422, 2417, 2411, 2406, 2400, 2395, 2389, 2384, 2378, 2373, 2367,
2362, 2356, 2351, 2345, 2340, 2334, 2329, 2323, 2318, 2312, 2307, 2301, 2295, 2290, 2284,
2279, 2273, 2268, 2262, 2257, 2251, 2245, 2240, 2234, 2229, 2223, 2218, 2212, 2207, 2201,
2195, 2190, 2184, 2179, 2173, 2168, 2162, 2156, 2151, 2145, 2140, 2134, 2128, 2123, 2117,
2112, 2106, 2101, 2095, 2089, 2084, 2078, 2073, 2067, 2061, 2056, 2050, 2045, 2039, 2034,
2028, 2022, 2017, 2011, 2006, 2000, 1994, 1989, 1983, 1978, 1972, 1967, 1961, 1955, 1950,
1944, 1939, 1933, 1927, 1922, 1916, 1911, 1905, 1900, 1894, 1888, 1883, 1877, 1872, 1866,
1861, 1855, 1850, 1844, 1838, 1833, 1827, 1822, 1816, 1811, 1805, 1800, 1794, 1788, 1783,
1777, 1772, 1766, 1761, 1755, 1750, 1744, 1739, 1733, 1728, 1722, 1717, 1711, 1706, 1700,
1695, 1689, 1684, 1678, 1673, 1667, 1662, 1656, 1651, 1645, 1640, 1634, 1629, 1623, 1618,
1612, 1607, 1601, 1596, 1591, 1585, 1580, 1574, 1569, 1563, 1558, 1552, 1547, 1542, 1536,
1531, 1525, 1520, 1515, 1509, 1504, 1498, 1493, 1488, 1482, 1477, 1472, 1466, 1461, 1456,
1450, 1445, 1440, 1434, 1429, 1424, 1418, 1413, 1408, 1402, 1397, 1392, 1386, 1381, 1376,
1371, 1365, 1360, 1355, 1350, 1344, 1339, 1334, 1329, 1323, 1318, 1313, 1308, 1302, 1297,
1292, 1287, 1282, 1277, 1271, 1266, 1261, 1256, 1251, 1246, 1240, 1235, 1230, 1225, 1220,
1215, 1210, 1205, 1200, 1195, 1189, 1184, 1179, 1174, 1169, 1164, 1159, 1154, 1149, 1144,
1139, 1134, 1129, 1124, 1119, 1114, 1109, 1104, 1099, 1094, 1089, 1084, 1079, 1075, 1070,
1065, 1060, 1055, 1050, 1045, 1040, 1035, 1031, 1026, 1021, 1016, 1011, 1006, 1002, 997,
992, 987, 983, 978, 973, 968, 963, 959, 954, 949, 945, 940, 935, 931, 926,
921, 917, 912, 907, 903, 898, 893, 889, 884, 880, 875, 870, 866, 861, 857,
852, 848, 843, 839, 834, 830, 825, 821, 816, 812, 807, 803, 798, 794, 790,
785, 781, 776, 772, 768, 763, 759, 755, 750, 746, 742, 737, 733, 729, 725,
720, 716, 712, 708, 703, 699, 695, 691, 687, 682, 678, 674, 670, 666, 662,
658, 653, 649, 645, 641, 637, 633, 629, 625, 621, 617, 613, 609, 605, 601,
597, 593, 589, 585, 582, 578, 574, 570, 566, 562, 558, 555, 551, 547, 543,
539, 536, 532, 528, 524, 521, 517, 513, 509, 506, 502, 498, 495, 491, 488,
484, 480, 477, 473, 470, 466, 462, 459, 455, 452, 448, 445, 441, 438, 435,
431, 428, 424, 421, 418, 414, 411, 407, 404, 401, 397, 394, 391, 388, 384,
381, 378, 375, 371, 368, 365, 362, 359, 356, 352, 349, 346, 343, 340, 337,
334, 331, 328, 325, 322, 319, 316, 313, 310, 307, 304, 301, 298, 295, 292,
289, 287, 284, 281, 278, 275, 272, 270, 267, 264, 261, 259, 256, 253, 251,
248, 245, 243, 240, 237, 235, 232, 230, 227, 224, 222, 219, 217, 214, 212,
209, 207, 205, 202, 200, 197, 195, 193, 190, 188, 186, 183, 181, 179, 176,
174, 172, 170, 167, 165, 163, 161, 159, 157, 154, 152, 150, 148, 146, 144,
142, 140, 138, 136, 134, 132, 130, 128, 126, 124, 122, 120, 118, 117, 115,
113, 111, 109, 107, 106, 104, 102, 100, 99, 97, 95, 94, 92, 90, 89,
87, 85, 84, 82, 81, 79, 78, 76, 75, 73, 72, 70, 69, 67, 66,
65, 63, 62, 60, 59, 58, 56, 55, 54, 53, 51, 50, 49, 48, 46,
45, 44, 43, 42, 41, 40, 39, 37, 36, 35, 34, 33, 32, 31, 30,
29, 29, 28, 27, 26, 25, 24, 23, 22, 22, 21, 20, 19, 18, 18,
17, 16, 16, 15, 14, 14, 13, 12, 12, 11, 11, 10, 9, 9, 8,
8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 4, 3, 3, 3, 3,
2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5,
6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13,
14, 15, 15, 16, 17, 17, 18, 19, 20, 20, 21, 22, 23, 24, 24,
25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
40, 41, 42, 44, 45, 46, 47, 48, 49, 51, 52, 53, 54, 56, 57,
58, 60, 61, 62, 64, 65, 67, 68, 69, 71, 72, 74, 75, 77, 78,
80, 81, 83, 85, 86, 88, 89, 91, 93, 94, 96, 98, 100, 101, 103,
105, 107, 108, 110, 112, 114, 116, 117, 119, 121, 123, 125, 127, 129, 131,
133, 135, 137, 139, 141, 143, 145, 147, 149, 151, 153, 155, 158, 160, 162,
164, 166, 169, 171, 173, 175, 178, 180, 182, 184, 187, 189, 191, 194, 196,
199, 201, 203, 206, 208, 211, 213, 216, 218, 221, 223, 226, 228, 231, 233,
236, 239, 241, 244, 247, 249, 252, 255, 257, 260, 263, 266, 268, 271, 274,
277, 279, 282, 285, 288, 291, 294, 297, 299, 302, 305, 308, 311, 314, 317,
320, 323, 326, 329, 332, 335, 338, 341, 345, 348, 351, 354, 357, 360, 363,
367, 370, 373, 376, 379, 383, 386, 389, 393, 396, 399, 402, 406, 409, 412,
416, 419, 423, 426, 429, 433, 436, 440, 443, 447, 450, 454, 457, 461, 464,
468, 471, 475, 479, 482, 486, 489, 493, 497, 500, 504, 508, 511, 515, 519,
522, 526, 530, 534, 537, 541, 545, 549, 553, 556, 560, 564, 568, 572, 576,
580, 583, 587, 591, 595, 599, 603, 607, 611, 615, 619, 623, 627, 631, 635,
639, 643, 647, 651, 656, 660, 664, 668, 672, 676, 680, 684, 689, 693, 697,
701, 705, 710, 714, 718, 722, 727, 731, 735, 739, 744, 748, 752, 757, 761,
765, 770, 774, 779, 783, 787, 792, 796, 801, 805, 809, 814, 818, 823, 827,
832, 836, 841, 845, 850, 854, 859, 864, 868, 873, 877, 882, 886, 891, 896,
900, 905, 910, 914, 919, 924, 928, 933, 938, 942, 947, 952, 956, 961, 966,
971, 975, 980, 985, 990, 994, 999, 1004, 1009, 1014, 1019, 1023, 1028, 1033, 1038,
1043, 1048, 1053, 1057, 1062, 1067, 1072, 1077, 1082, 1087, 1092, 1097, 1102, 1107, 1112,
1117, 1122, 1127, 1132, 1137, 1142, 1147, 1152, 1157, 1162, 1167, 1172, 1177, 1182, 1187,
1192, 1197, 1202, 1207, 1212, 1217, 1223, 1228, 1233, 1238, 1243, 1248, 1253, 1258, 1264,
1269, 1274, 1279, 1284, 1290, 1295, 1300, 1305, 1310, 1316, 1321, 1326, 1331, 1336, 1342,
1347, 1352, 1357, 1363, 1368, 1373, 1379, 1384, 1389, 1394, 1400, 1405, 1410, 1416, 1421,
1426, 1432, 1437, 1442, 1448, 1453, 1458, 1464, 1469, 1474, 1480, 1485, 1490, 1496, 1501,
1507, 1512, 1517, 1523, 1528, 1534, 1539, 1544, 1550, 1555, 1561, 1566, 1571, 1577, 1582,
1588, 1593, 1599, 1604, 1610, 1615, 1621, 1626, 1631, 1637, 1642, 1648, 1653, 1659, 1664,
1670, 1675, 1681, 1686, 1692, 1697, 1703, 1708, 1714, 1719, 1725, 1730, 1736, 1741, 1747,
1752, 1758, 1764, 1769, 1775, 1780, 1786, 1791, 1797, 1802, 1808, 1813, 1819, 1824, 1830,
1836, 1841, 1847, 1852, 1858, 1863, 1869, 1875, 1880, 1886, 1891, 1897, 1902, 1908, 1914,
1919, 1925, 1930, 1936, 1941, 1947, 1953, 1958, 1964, 1969, 1975, 1980, 1986, 1992, 1997,
2003, 2008, 2014, 2020, 2025, 2031, 2036, 2042, 2047
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  //first half of adc buffer is full
  in_buf_ptr = &adc_val[0];
  out_buf_ptr = &dac_val[HALF_BUF_SIZE];// + HALF_BUF_SIZE;
  flag=1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  //second half of adc buffer is full
  in_buf_ptr = &adc_val[HALF_BUF_SIZE];// + HALF_BUF_SIZE;
  out_buf_ptr = &dac_val[0];
  flag=1;
}

void process_data() {
  for (int i=0;i<HALF_BUF_SIZE;i++) {
    out_buf_ptr[i]=in_buf_ptr[i]; //* Wave_LUT[counter];
    counter++;
    if (counter==NS) counter = 0;
  }
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
  MX_LPUART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_val, FULL_BUF_SIZE);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) dac_val, FULL_BUF_SIZE, DAC_ALIGN_12B_R);

  arm_conv_f32(a, 5, b, 5, c);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (flag) {
		  process_data();
		  flag = 0;
	  }
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
  RCC_OscInitStruct.PLL.PLLN = 71;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  htim6.Init.Period = 492;
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
