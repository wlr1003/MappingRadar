/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsl_reg.h"
#include "usbd_cdc_if.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum run_mode {none = 'x', range = 'r', speed = 's', idle = 'i', map = 'm'} run_mode;

typedef struct Control {
	uint16_t run_time_sec;
	run_mode mode_instructed;
	run_mode mode_running;
	uint8_t transmit_data_flag;
	uint32_t time_out;
	uint8_t digital_pot_instructed;
	uint8_t digital_pot_current_setting;
} control;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TX_BUF_DIM 512 // buffer length for usb transmitting
#define DMA_BUF_LEN 128 // buffer length for adc1 dma, mixer o/p
#define VTUNE_SAMPLE_LEN 2000 // number of samples in VTUNE waveform

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static const uint8_t DIGITAL_POT_ADDR = 0x2f << 1; // Use 7-bit address '101111' for digital potentiometer
static uint16_t adc1_dma_buf_mixer_out[DMA_BUF_LEN];
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
//static uint8_t tx_buffer[TX_BUF_DIM];
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE]; // usb receive buffer
extern uint8_t input_received_flag; // flag for usb input received
// Flag to indicate when the SPI transfer is complete
volatile uint8_t spi_complete_flag = 0;
uint8_t VTune_first_cycle_complete;
uint32_t IDLE = 3724; // 3Volt with 3.3V VDDA = 2.48GHz
uint32_t CTune = 3103; /* 2.5Volt with 3.3V VDDA */
uint32_t VTune[VTUNE_SAMPLE_LEN] = {
		2482, 2483, 2484, 2486, 2487, 2488, 2489, 2491, 2492, 2493, 2494, 2495, 2497, 2498, 2499,
		2500, 2502, 2503, 2504, 2505, 2507, 2508, 2509, 2510, 2512, 2513, 2514, 2515, 2517, 2518,
		2519, 2520, 2522, 2523, 2524, 2525, 2527, 2528, 2529, 2530, 2531, 2533, 2534, 2535, 2536,
		2538, 2539, 2540, 2541, 2543, 2544, 2545, 2546, 2548, 2549, 2550, 2551, 2553, 2554, 2555,
		2556, 2558, 2559, 2560, 2561, 2563, 2564, 2565, 2566, 2567, 2569, 2570, 2571, 2572, 2574,
		2575, 2576, 2577, 2579, 2580, 2581, 2582, 2584, 2585, 2586, 2587, 2589, 2590, 2591, 2592,
		2594, 2595, 2596, 2597, 2599, 2600, 2601, 2602, 2603, 2605, 2606, 2607, 2608, 2610, 2611,
		2612, 2613, 2615, 2616, 2617, 2618, 2620, 2621, 2622, 2623, 2625, 2626, 2627, 2628, 2630,
		2631, 2632, 2633, 2635, 2636, 2637, 2638, 2639, 2641, 2642, 2643, 2644, 2646, 2647, 2648,
		2649, 2651, 2652, 2653, 2654, 2656, 2657, 2658, 2659, 2661, 2662, 2663, 2664, 2666, 2667,
		2668, 2669, 2671, 2672, 2673, 2674, 2675, 2677, 2678, 2679, 2680, 2682, 2683, 2684, 2685,
		2687, 2688, 2689, 2690, 2692, 2693, 2694, 2695, 2697, 2698, 2699, 2700, 2702, 2703, 2704,
		2705, 2707, 2708, 2709, 2710, 2712, 2713, 2714, 2715, 2716, 2718, 2719, 2720, 2721, 2723,
		2724, 2725, 2726, 2728, 2729, 2730, 2731, 2733, 2734, 2735, 2736, 2738, 2739, 2740, 2741,
		2743, 2744, 2745, 2746, 2748, 2749, 2750, 2751, 2752, 2754, 2755, 2756, 2757, 2759, 2760,
		2761, 2762, 2764, 2765, 2766, 2767, 2769, 2770, 2771, 2772, 2774, 2775, 2776, 2777, 2779,
		2780, 2781, 2782, 2784, 2785, 2786, 2787, 2788, 2790, 2791, 2792, 2793, 2795, 2796, 2797,
		2798, 2800, 2801, 2802, 2803, 2805, 2806, 2807, 2808, 2810, 2811, 2812, 2813, 2815, 2816,
		2817, 2818, 2820, 2821, 2822, 2823, 2824, 2826, 2827, 2828, 2829, 2831, 2832, 2833, 2834,
		2836, 2837, 2838, 2839, 2841, 2842, 2843, 2844, 2846, 2847, 2848, 2849, 2851, 2852, 2853,
		2854, 2856, 2857, 2858, 2859, 2860, 2862, 2863, 2864, 2865, 2867, 2868, 2869, 2870, 2872,
		2873, 2874, 2875, 2877, 2878, 2879, 2880, 2882, 2883, 2884, 2885, 2887, 2888, 2889, 2890,
		2892, 2893, 2894, 2895, 2896, 2898, 2899, 2900, 2901, 2903, 2904, 2905, 2906, 2908, 2909,
		2910, 2911, 2913, 2914, 2915, 2916, 2918, 2919, 2920, 2921, 2923, 2924, 2925, 2926, 2928,
		2929, 2930, 2931, 2932, 2934, 2935, 2936, 2937, 2939, 2940, 2941, 2942, 2944, 2945, 2946,
		2947, 2949, 2950, 2951, 2952, 2954, 2955, 2956, 2957, 2959, 2960, 2961, 2962, 2964, 2965,
		2966, 2967, 2968, 2970, 2971, 2972, 2973, 2975, 2976, 2977, 2978, 2980, 2981, 2982, 2983,
		2985, 2986, 2987, 2988, 2990, 2991, 2992, 2993, 2995, 2996, 2997, 2998, 3000, 3001, 3002,
		3003, 3005, 3006, 3007, 3008, 3009, 3011, 3012, 3013, 3014, 3016, 3017, 3018, 3019, 3021,
		3022, 3023, 3024, 3026, 3027, 3028, 3029, 3031, 3032, 3033, 3034, 3036, 3037, 3038, 3039,
		3041, 3042, 3043, 3044, 3045, 3047, 3048, 3049, 3050, 3052, 3053, 3054, 3055, 3057, 3058,
		3059, 3060, 3062, 3063, 3064, 3065, 3067, 3068, 3069, 3070, 3072, 3073, 3074, 3075, 3077,
		3078, 3079, 3080, 3081, 3083, 3084, 3085, 3086, 3088, 3089, 3090, 3091, 3093, 3094, 3095,
		3096, 3098, 3099, 3100, 3101, 3103, 3104, 3105, 3106, 3108, 3109, 3110, 3111, 3113, 3114,
		3115, 3116, 3117, 3119, 3120, 3121, 3122, 3124, 3125, 3126, 3127, 3129, 3130, 3131, 3132,
		3134, 3135, 3136, 3137, 3139, 3140, 3141, 3142, 3144, 3145, 3146, 3147, 3149, 3150, 3151,
		3152, 3153, 3155, 3156, 3157, 3158, 3160, 3161, 3162, 3163, 3165, 3166, 3167, 3168, 3170,
		3171, 3172, 3173, 3175, 3176, 3177, 3178, 3180, 3181, 3182, 3183, 3185, 3186, 3187, 3188,
		3189, 3191, 3192, 3193, 3194, 3196, 3197, 3198, 3199, 3201, 3202, 3203, 3204, 3206, 3207,
		3208, 3209, 3211, 3212, 3213, 3214, 3216, 3217, 3218, 3219, 3221, 3222, 3223, 3224, 3225,
		3227, 3228, 3229, 3230, 3232, 3233, 3234, 3235, 3237, 3238, 3239, 3240, 3242, 3243, 3244,
		3245, 3247, 3248, 3249, 3250, 3252, 3253, 3254, 3255, 3257, 3258, 3259, 3260, 3261, 3263,
		3264, 3265, 3266, 3268, 3269, 3270, 3271, 3273, 3274, 3275, 3276, 3278, 3279, 3280, 3281,
		3283, 3284, 3285, 3286, 3288, 3289, 3290, 3291, 3293, 3294, 3295, 3296, 3298, 3299, 3300,
		3301, 3302, 3304, 3305, 3306, 3307, 3309, 3310, 3311, 3312, 3314, 3315, 3316, 3317, 3319,
		3320, 3321, 3322, 3324, 3325, 3326, 3327, 3329, 3330, 3331, 3332, 3334, 3335, 3336, 3337,
		3338, 3340, 3341, 3342, 3343, 3345, 3346, 3347, 3348, 3350, 3351, 3352, 3353, 3355, 3356,
		3357, 3358, 3360, 3361, 3362, 3363, 3365, 3366, 3367, 3368, 3370, 3371, 3372, 3373, 3374,
		3376, 3377, 3378, 3379, 3381, 3382, 3383, 3384, 3386, 3387, 3388, 3389, 3391, 3392, 3393,
		3394, 3396, 3397, 3398, 3399, 3401, 3402, 3403, 3404, 3406, 3407, 3408, 3409, 3410, 3412,
		3413, 3414, 3415, 3417, 3418, 3419, 3420, 3422, 3423, 3424, 3425, 3427, 3428, 3429, 3430,
		3432, 3433, 3434, 3435, 3437, 3438, 3439, 3440, 3442, 3443, 3444, 3445, 3446, 3448, 3449,
		3450, 3451, 3453, 3454, 3455, 3456, 3458, 3459, 3460, 3461, 3463, 3464, 3465, 3466, 3468,
		3469, 3470, 3471, 3473, 3474, 3475, 3476, 3478, 3479, 3480, 3481, 3482, 3484, 3485, 3486,
		3487, 3489, 3490, 3491, 3492, 3494, 3495, 3496, 3497, 3499, 3500, 3501, 3502, 3504, 3505,
		3506, 3507, 3509, 3510, 3511, 3512, 3514, 3515, 3516, 3517, 3518, 3520, 3521, 3522, 3523,
		3525, 3526, 3527, 3528, 3530, 3531, 3532, 3533, 3535, 3536, 3537, 3538, 3540, 3541, 3542,
		3543, 3545, 3546, 3547, 3548, 3550, 3551, 3552, 3553, 3554, 3556, 3557, 3558, 3559, 3561,
		3562, 3563, 3564, 3566, 3567, 3568, 3569, 3571, 3572, 3573, 3574, 3576, 3577, 3578, 3579,
		3581, 3582, 3583, 3584, 3586, 3587, 3588, 3589, 3591, 3592, 3593, 3594, 3595, 3597, 3598,
		3599, 3600, 3602, 3603, 3604, 3605, 3607, 3608, 3609, 3610, 3612, 3613, 3614, 3615, 3617,
		3618, 3619, 3620, 3622, 3623, 3624, 3625, 3627, 3628, 3629, 3630, 3631, 3633, 3634, 3635,
		3636, 3638, 3639, 3640, 3641, 3643, 3644, 3645, 3646, 3648, 3649, 3650, 3651, 3653, 3654,
		3655, 3656, 3658, 3659, 3660, 3661, 3663, 3664, 3665, 3666, 3667, 3669, 3670, 3671, 3672,
		3674, 3675, 3676, 3677, 3679, 3680, 3681, 3682, 3684, 3685, 3686, 3687, 3689, 3690, 3691,
		3692, 3694, 3695, 3696, 3697, 3699, 3700, 3701, 3702, 3703, 3705, 3706, 3707, 3708, 3710,
		3711, 3712, 3713, 3715, 3716, 3717, 3718, 3720, 3721, 3722, 3722, 3721, 3720, 3718, 3717,
		3716, 3715, 3713, 3712, 3711, 3710, 3708, 3707, 3706, 3705, 3703, 3702, 3701, 3700, 3699,
		3697, 3696, 3695, 3694, 3692, 3691, 3690, 3689, 3687, 3686, 3685, 3684, 3682, 3681, 3680,
		3679, 3677, 3676, 3675, 3674, 3672, 3671, 3670, 3669, 3667, 3666, 3665, 3664, 3663, 3661,
		3660, 3659, 3658, 3656, 3655, 3654, 3653, 3651, 3650, 3649, 3648, 3646, 3645, 3644, 3643,
		3641, 3640, 3639, 3638, 3636, 3635, 3634, 3633, 3631, 3630, 3629, 3628, 3627, 3625, 3624,
		3623, 3622, 3620, 3619, 3618, 3617, 3615, 3614, 3613, 3612, 3610, 3609, 3608, 3607, 3605,
		3604, 3603, 3602, 3600, 3599, 3598, 3597, 3595, 3594, 3593, 3592, 3591, 3589, 3588, 3587,
		3586, 3584, 3583, 3582, 3581, 3579, 3578, 3577, 3576, 3574, 3573, 3572, 3571, 3569, 3568,
		3567, 3566, 3564, 3563, 3562, 3561, 3559, 3558, 3557, 3556, 3554, 3553, 3552, 3551, 3550,
		3548, 3547, 3546, 3545, 3543, 3542, 3541, 3540, 3538, 3537, 3536, 3535, 3533, 3532, 3531,
		3530, 3528, 3527, 3526, 3525, 3523, 3522, 3521, 3520, 3518, 3517, 3516, 3515, 3514, 3512,
		3511, 3510, 3509, 3507, 3506, 3505, 3504, 3502, 3501, 3500, 3499, 3497, 3496, 3495, 3494,
		3492, 3491, 3490, 3489, 3487, 3486, 3485, 3484, 3482, 3481, 3480, 3479, 3478, 3476, 3475,
		3474, 3473, 3471, 3470, 3469, 3468, 3466, 3465, 3464, 3463, 3461, 3460, 3459, 3458, 3456,
		3455, 3454, 3453, 3451, 3450, 3449, 3448, 3446, 3445, 3444, 3443, 3442, 3440, 3439, 3438,
		3437, 3435, 3434, 3433, 3432, 3430, 3429, 3428, 3427, 3425, 3424, 3423, 3422, 3420, 3419,
		3418, 3417, 3415, 3414, 3413, 3412, 3410, 3409, 3408, 3407, 3406, 3404, 3403, 3402, 3401,
		3399, 3398, 3397, 3396, 3394, 3393, 3392, 3391, 3389, 3388, 3387, 3386, 3384, 3383, 3382,
		3381, 3379, 3378, 3377, 3376, 3374, 3373, 3372, 3371, 3370, 3368, 3367, 3366, 3365, 3363,
		3362, 3361, 3360, 3358, 3357, 3356, 3355, 3353, 3352, 3351, 3350, 3348, 3347, 3346, 3345,
		3343, 3342, 3341, 3340, 3338, 3337, 3336, 3335, 3334, 3332, 3331, 3330, 3329, 3327, 3326,
		3325, 3324, 3322, 3321, 3320, 3319, 3317, 3316, 3315, 3314, 3312, 3311, 3310, 3309, 3307,
		3306, 3305, 3304, 3302, 3301, 3300, 3299, 3298, 3296, 3295, 3294, 3293, 3291, 3290, 3289,
		3288, 3286, 3285, 3284, 3283, 3281, 3280, 3279, 3278, 3276, 3275, 3274, 3273, 3271, 3270,
		3269, 3268, 3266, 3265, 3264, 3263, 3261, 3260, 3259, 3258, 3257, 3255, 3254, 3253, 3252,
		3250, 3249, 3248, 3247, 3245, 3244, 3243, 3242, 3240, 3239, 3238, 3237, 3235, 3234, 3233,
		3232, 3230, 3229, 3228, 3227, 3225, 3224, 3223, 3222, 3221, 3219, 3218, 3217, 3216, 3214,
		3213, 3212, 3211, 3209, 3208, 3207, 3206, 3204, 3203, 3202, 3201, 3199, 3198, 3197, 3196,
		3194, 3193, 3192, 3191, 3189, 3188, 3187, 3186, 3185, 3183, 3182, 3181, 3180, 3178, 3177,
		3176, 3175, 3173, 3172, 3171, 3170, 3168, 3167, 3166, 3165, 3163, 3162, 3161, 3160, 3158,
		3157, 3156, 3155, 3153, 3152, 3151, 3150, 3149, 3147, 3146, 3145, 3144, 3142, 3141, 3140,
		3139, 3137, 3136, 3135, 3134, 3132, 3131, 3130, 3129, 3127, 3126, 3125, 3124, 3122, 3121,
		3120, 3119, 3117, 3116, 3115, 3114, 3113, 3111, 3110, 3109, 3108, 3106, 3105, 3104, 3103,
		3101, 3100, 3099, 3098, 3096, 3095, 3094, 3093, 3091, 3090, 3089, 3088, 3086, 3085, 3084,
		3083, 3081, 3080, 3079, 3078, 3077, 3075, 3074, 3073, 3072, 3070, 3069, 3068, 3067, 3065,
		3064, 3063, 3062, 3060, 3059, 3058, 3057, 3055, 3054, 3053, 3052, 3050, 3049, 3048, 3047,
		3045, 3044, 3043, 3042, 3041, 3039, 3038, 3037, 3036, 3034, 3033, 3032, 3031, 3029, 3028,
		3027, 3026, 3024, 3023, 3022, 3021, 3019, 3018, 3017, 3016, 3014, 3013, 3012, 3011, 3009,
		3008, 3007, 3006, 3005, 3003, 3002, 3001, 3000, 2998, 2997, 2996, 2995, 2993, 2992, 2991,
		2990, 2988, 2987, 2986, 2985, 2983, 2982, 2981, 2980, 2978, 2977, 2976, 2975, 2973, 2972,
		2971, 2970, 2968, 2967, 2966, 2965, 2964, 2962, 2961, 2960, 2959, 2957, 2956, 2955, 2954,
		2952, 2951, 2950, 2949, 2947, 2946, 2945, 2944, 2942, 2941, 2940, 2939, 2937, 2936, 2935,
		2934, 2932, 2931, 2930, 2929, 2928, 2926, 2925, 2924, 2923, 2921, 2920, 2919, 2918, 2916,
		2915, 2914, 2913, 2911, 2910, 2909, 2908, 2906, 2905, 2904, 2903, 2901, 2900, 2899, 2898,
		2896, 2895, 2894, 2893, 2892, 2890, 2889, 2888, 2887, 2885, 2884, 2883, 2882, 2880, 2879,
		2878, 2877, 2875, 2874, 2873, 2872, 2870, 2869, 2868, 2867, 2865, 2864, 2863, 2862, 2860,
		2859, 2858, 2857, 2856, 2854, 2853, 2852, 2851, 2849, 2848, 2847, 2846, 2844, 2843, 2842,
		2841, 2839, 2838, 2837, 2836, 2834, 2833, 2832, 2831, 2829, 2828, 2827, 2826, 2824, 2823,
		2822, 2821, 2820, 2818, 2817, 2816, 2815, 2813, 2812, 2811, 2810, 2808, 2807, 2806, 2805,
		2803, 2802, 2801, 2800, 2798, 2797, 2796, 2795, 2793, 2792, 2791, 2790, 2788, 2787, 2786,
		2785, 2784, 2782, 2781, 2780, 2779, 2777, 2776, 2775, 2774, 2772, 2771, 2770, 2769, 2767,
		2766, 2765, 2764, 2762, 2761, 2760, 2759, 2757, 2756, 2755, 2754, 2752, 2751, 2750, 2749,
		2748, 2746, 2745, 2744, 2743, 2741, 2740, 2739, 2738, 2736, 2735, 2734, 2733, 2731, 2730,
		2729, 2728, 2726, 2725, 2724, 2723, 2721, 2720, 2719, 2718, 2716, 2715, 2714, 2713, 2712,
		2710, 2709, 2708, 2707, 2705, 2704, 2703, 2702, 2700, 2699, 2698, 2697, 2695, 2694, 2693,
		2692, 2690, 2689, 2688, 2687, 2685, 2684, 2683, 2682, 2680, 2679, 2678, 2677, 2675, 2674,
		2673, 2672, 2671, 2669, 2668, 2667, 2666, 2664, 2663, 2662, 2661, 2659, 2658, 2657, 2656,
		2654, 2653, 2652, 2651, 2649, 2648, 2647, 2646, 2644, 2643, 2642, 2641, 2639, 2638, 2637,
		2636, 2635, 2633, 2632, 2631, 2630, 2628, 2627, 2626, 2625, 2623, 2622, 2621, 2620, 2618,
		2617, 2616, 2615, 2613, 2612, 2611, 2610, 2608, 2607, 2606, 2605, 2603, 2602, 2601, 2600,
		2599, 2597, 2596, 2595, 2594, 2592, 2591, 2590, 2589, 2587, 2586, 2585, 2584, 2582, 2581,
		2580, 2579, 2577, 2576, 2575, 2574, 2572, 2571, 2570, 2569, 2567, 2566, 2565, 2564, 2563,
		2561, 2560, 2559, 2558, 2556, 2555, 2554, 2553, 2551, 2550, 2549, 2548, 2546, 2545, 2544,
		2543, 2541, 2540, 2539, 2538, 2536, 2535, 2534, 2533, 2531, 2530, 2529, 2528, 2527, 2525,
		2524, 2523, 2522, 2520, 2519, 2518, 2517, 2515, 2514, 2513, 2512, 2510, 2509, 2508, 2507,
		2505, 2504, 2503, 2502, 2500, 2499, 2498, 2497, 2495, 2494, 2493, 2492, 2491, 2489, 2488,
		2487, 2486, 2484, 2483, 2482};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
// platform read/write used to read/write to lsm6dsl
/** Please note that is MANDATORY: return 0 -> no Error.**/
static int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);

// usb transmit
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void process_input(const uint8_t *arr, control *pControl);
uint8_t strcontains(const char* str1,const char* str2);
void my_strcpy(char* cpy, const char* orig, uint8_t len);
uint8_t isValid(const char checkChar,const char* validModes);
void set_DAC_for_VCO(control *ctrl_ptr, uint8_t cycle_DAC);
HAL_StatusTypeDef set_digital_pot(control* ctrl_ptr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
input_received_flag = 0;
VTune_first_cycle_complete = 0;
// uint8_t lsm6dslError[] ="LSM6DSL whoAmI error";
uint32_t runtime_additional_time_ms = 00;
uint8_t digital_pot_check = 255; // register used to read potentiometer register
HAL_StatusTypeDef ret; // used for checking return values when function returns HAL status

// initialize command struct and set default values
control user_input;
user_input.mode_instructed = range; // r:range, s:speed, i: idle
user_input.mode_running = none; // x:none
user_input.run_time_sec=0; // length of time in seconds to operate
user_input.time_out = 3600000; // will run in range mode upon start up for 1 hour before setting VCO to idle freq
user_input.digital_pot_instructed = 0x3f;  // 0x7f is full scale, 0x3f is midscale, 0 is zero scale
user_input.digital_pot_current_setting = 0x3f;  // digital pot will initialize itself to 0x3f on power up

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USB_Device_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  set_DAC_for_VCO(&user_input, 0); // starts timer and sets dac output used for VCO
  HAL_TIM_Base_Start(&htim1); // start timer 1 for adc1 conversion for radar mixer o/p
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3); // sets output compare for timer1, sets PA10 to toggle on timer1 register reload (40kHz)

//  set the digital pot to value in control struct
  ret = set_digital_pot(&user_input);

  /* initialize accelerometer/gyroscope on lsm6dsl */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hspi1;
  /* Check device ID */
    whoamI = 0;
    lsm6dsl_device_id_get(&dev_ctx, &whoamI);

    if ( whoamI != LSM6DSL_ID ) {
//    	CDC_Transmit_FS(lsm6dslError,sizeof(lsm6dslError));
    }
//    /* Restore default configuration */
//    lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
//
//    do {
//      lsm6dsl_reset_get(&dev_ctx, &rst);
//    } while (rst);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // on command from PC, read command
	  if (input_received_flag)
	  {
		    // read command
	 	    process_input(&UserRxBufferFS,&user_input);
	 	    // set digital pot on command from PC
	 	    if (user_input.digital_pot_instructed != user_input.digital_pot_current_setting) {
	 	    	ret = set_digital_pot(&user_input);
	 	    }
	 	    // when instructed
	 	    // start the DAC for VCO according to command
	 	    // start ADC for reading input/outputting to PC
	 	    if (user_input.mode_instructed != none) {
	 	    	if (user_input.mode_instructed == range) {
	 	    		VTune_first_cycle_complete = 0; // clear cycle complete flag for vtune
	 	    	}
	 	    	set_DAC_for_VCO(&user_input, 1);  // set DAC/ADC and calculate end time of run
	 	    	if (user_input.mode_instructed == range) { // wait for 1 VTune cycle complete
	 	    		while (VTune_first_cycle_complete != 1) {
	 	    			// stuck in loop until first dac_complete callback sets VTune_first _cycle_complete flag
	 	    		}
	 	    		HAL_Delay(2.5);
	 	    	}
	 	    	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buf_mixer_out, DMA_BUF_LEN); // start the adc with dma
	 	    	user_input.time_out = HAL_GetTick() + (user_input.run_time_sec * 1000) + runtime_additional_time_ms;	// HAL_GetTick returns milliseconds
	 	 	}
	 	        input_received_flag=0; // clear input flag
	  }
	  // when running, check for time elapsed of run being greater than time instructed
	  if (user_input.mode_running != idle) {
		  if (HAL_GetTick() > user_input.time_out) {
			  // when run times out, set mode_instructed to i, stop the ADC and set DAC to "off mode"
			  user_input.mode_instructed = idle;
			  HAL_ADC_Stop_DMA(&hadc1);	// stop ADC
	 	      set_DAC_for_VCO(&user_input, 0);  // set DAC
		  }
		  // digital pot does not acknowledge after address sent
//	  uint8_t digital_pot_buf = 0x7f; // 0x7f is full scale, 0x3f is midscale, 0 is zero scale
//
//	  if (ret != HAL_OK) {
//		  HAL_Delay(10);
//		  ret = HAL_I2C_Master_Transmit(&hi2c2, DIGITAL_POT_ADDR, &digital_pot_buf, 1, 1000);
//		 // HAL_Delay(250);
//	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV12;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
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
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30909DEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LSM6DSL_ncs_GPIO_Port, LSM6DSL_ncs_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                           LED5_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM6DSL_ncs_Pin */
  GPIO_InitStruct.Pin = LSM6DSL_ncs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LSM6DSL_ncs_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_GPIO_WritePin(LSM6DSL_ncs_GPIO_Port, LSM6DSL_ncs_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &reg, 1, 2);
  HAL_SPI_Transmit(&hspi1, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(LSM6DSL_ncs_GPIO_Port, LSM6DSL_ncs_Pin, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	uint8_t tx_data[2];
	tx_data[0] = 0x80 | reg;
	tx_data[1] = 0x00;
	// get spi state
	HAL_SPI_StateTypeDef tmp_state;
	tmp_state = HAL_SPI_GetState(handle);

	// Start the SPI transfer
	HAL_GPIO_WritePin(LSM6DSL_ncs_GPIO_Port, LSM6DSL_ncs_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(handle, &tx_data, bufp, len + 1);
//	  if(HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)
//	  {
//	    /* Transfer error in transmission process */
//	    Error_Handler();
//	  }
	// Wait for the transfer to complete
    while(HAL_SPI_GetState(handle) != tmp_state);

	HAL_GPIO_WritePin(LSM6DSL_ncs_GPIO_Port, LSM6DSL_ncs_Pin, GPIO_PIN_SET);
  return 0;
}


/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  spi_complete_flag = 1;
}

/**
  * @brief  Tx and Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  spi_complete_flag = 1;
}

/**
  * @brief  Conversion complete callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint8_t len = DMA_BUF_LEN/2;
	uint8_t halfIndex = len-1;
//	memcpy(tx_buffer[halfIndex],adc1_dma_buf_mixer_out[halfIndex],len);
	CDC_Transmit_FS(&adc1_dma_buf_mixer_out[halfIndex], len);
}

/**
  * @brief  Conversion DMA half-transfer callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint8_t len = DMA_BUF_LEN/2;
//	memcpy(tx_buffer[0],adc1_dma_buf_mixer_out[0],len);
	CDC_Transmit_FS(&adc1_dma_buf_mixer_out[0], len);
}


void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	VTune_first_cycle_complete = 1;
}


/*
 * process the input received over usb to extract the operating mode and the time of operation
 *
 * @param arr    user input array
 * @param pCommand command struct to return mode and time
*/
void process_input(const uint8_t *arr, control *pControl) {
	char mode[]="mode:";
    char time[] = "time:";
    char pot[] = "pot:";
    char validMode[] = {'r', 's', 'm', 'i'}; // range speed map
    char word[64] = {0};
    uint8_t i = sizeof(mode);
    uint8_t j= sizeof(time);
    uint8_t k = sizeof(pot);
    // check input to ensure "mode:" is received
    my_strcpy(word, arr, i);
    if (strcontains(word,mode)) {
    	if (isValid(arr[i-1],validMode)) {
    		pControl->mode_instructed=arr[i-1];     // set mode in command
    	}
    	else { // invalid mode command
    		return;
    	}
    }
    else { // invalid command
    	return;
    }
    // move index past command for mode and then '\n'
    while(arr[i]=='\n') {
    	i++;
    }
    // check input to ensure "time:" is received
    my_strcpy(word, &arr[i], j);
    	if (strcontains(word,time)) {//mode:r\ntime:10\npot:63
    	   // set i to index one past command for time
			i=i+j-1;
			j=i+1;
			// get index of last digit
			while (arr[j]!='\n'&& arr[j]!='\0') {
			j++;
			}
			// set run time to zero
			pControl->run_time_sec=0;
			// add each digits value,
			// *10 to shift current value left one digit for adding next digit
			// -48 converts from ascii to int
			while (i < j) {
			pControl->run_time_sec=(pControl->run_time_sec*10)+arr[i]-48;
			i++;
			}
      }
      // move index past command for mode and then '\n'
      while(arr[i]=='\n') {
    	    i++;
      }
	// check input to ensure "pot:" is received
	my_strcpy(word, &arr[i], k);
		if (strcontains(word,pot)) {//mode:r\ntime:10\npot:63
		   // set i to index one past command for time
			i=i+k-1;
			j=i+1;
			// get index of last digit
			while (arr[j]!='\n'&& arr[j]!='\0') {
			j++;
			}
			// set digital pot setting to zero
			pControl->digital_pot_instructed = 0;
			// add each digits value,
			// *10 to shift current value left one digit for adding next digit
			// -48 converts from ascii to int
			while (i < j) {
			pControl->digital_pot_instructed = (pControl->digital_pot_instructed*10)+arr[i]-48;
			i++;
			}
	  }
	  pControl->transmit_data_flag=1; // set flag for transmit data
}

/*
 * Compare two strings,
 * return 1 for same string, 0 for different strings
 */
uint8_t strcontains(const char* str1,const char* str2) {
	  uint8_t i = 0, strings_match = 1; // strings match = true
	  while (str2[i]!='\0') { // while both strings have a character
		  if(str1[i] != str2[i]) {		// if check character doesn't match
			  	  strings_match = 0;	// strings match = false
		  }
		  i++;
	  }
	  if (str2[i]!='\0') { // if either string has a character
	        strings_match = 0;	// strings match = false
	  }
	  return strings_match;
}

/*
 * copy original string into copy
 * len is original strings length
 * WARNING copy must be adequate length
 * will stop early on null byte
 */
void my_strcpy(char* cpy, const char* orig, uint8_t len) {
	uint8_t i = 0;
	while(orig[i]!='\0' && i<=len) {
		cpy[i]=orig[i];
		i++;
	}
}

/*
 * checks the checkChar is within the validModes char*
 * returns 1 for true, 0 for false
 */
uint8_t isValid(const char checkChar,const char* validModes) {
	uint8_t i;
	for(i=0;sizeof(validModes);i++) {
		if (checkChar==validModes[i]) {
			return 1;
		}
	}
	return 0;
}



/*
 * @ brief: function controls DAC output for the VCO
 * @param ctrl_ptr: pointer to control structure
 * @param cycle_DAC: 1 = turn off and back on the DAC for synchronization on restart during a run, 0 = cycling not required
 * four modes of operation,
 * 		i = idle mode, DAC sets VCO to 2.48 GHz
 * 		r = range mode, DAC set with VTune signal
 * 		s = speed mode, DAC sets VCO to 2.455 GHz
 * 		m = map, DAC set with VTune signal
 */
void set_DAC_for_VCO(control *ctrl_ptr, uint8_t cycle_DAC) {
	// if currently running is as instructed, return
	if (ctrl_ptr->mode_running == ctrl_ptr->mode_instructed && cycle_DAC == 0) {
		return;
	}

	// if currently running in other mode, turn it off,
	if (ctrl_ptr->mode_running == range || ctrl_ptr->mode_running == map) {
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		HAL_TIM_Base_Stop(&htim2);
	} else if (ctrl_ptr->mode_running == speed || ctrl_ptr->mode_running == idle) {
		HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
	}

	  /* Set DAC_CH_1 to CTune VTune or IDLE based on user input for mode */
	if (ctrl_ptr->mode_instructed == range || ctrl_ptr->mode_instructed == map) {
		// turn on dac using dma and timer 2
		HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,(uint32_t*)VTune,VTUNE_SAMPLE_LEN,DAC_ALIGN_12B_R);
		HAL_TIM_Base_Start(&htim2);
	} else if (ctrl_ptr->mode_instructed == speed) {
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, CTune);

	} else {
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, IDLE);
	}
	ctrl_ptr->mode_running = ctrl_ptr->mode_instructed;
}


/*
 * set the digital pot using i2c
 * function will set LED1 on initial fail and LED2 on secondary fail
 * @param ctrl_ptr: pointer to control structure
 * @return: HAL status of i2c communication
 */
HAL_StatusTypeDef set_digital_pot(control* ctrl_ptr) {
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c2, DIGITAL_POT_ADDR, &ctrl_ptr->digital_pot_instructed, 1, 1000);
	if (ret != HAL_OK) {
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		uint8_t software_reset = 0xFF;
		HAL_I2C_Master_Transmit(&hi2c2,software_reset , &software_reset, 1, 100);
		ret = HAL_I2C_Master_Transmit(&hi2c2, DIGITAL_POT_ADDR, &ctrl_ptr->digital_pot_instructed, 1, 1000);
		if (ret != HAL_OK) {
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		} else {
			ctrl_ptr->digital_pot_current_setting = ctrl_ptr->digital_pot_instructed;
		}
	} else {
		ctrl_ptr->digital_pot_current_setting = ctrl_ptr->digital_pot_instructed;
	}
	return ret;
}


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
