/**
 * @file    motor_config.h
 * @brief   FOC 电机控制共享参数 (单一事实来源)
 * @note    所有与 PWM / 电机参数 / 数学常量相关的定义集中在此文件.
 *          CubeMX 生成 hrtim.c 时 PERIOD=30000, 此处需保持一致.
 */

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*  数学常量                                                                    */
/* -------------------------------------------------------------------------- */

#define MOTOR_PI              3.14159265358979323846f
#define MOTOR_TWO_PI          6.28318530717958647692f
#define MOTOR_INV_SQRT3       0.57735026918962576451f   /**< 1/sqrt(3)      */
#define MOTOR_SQRT3_2         0.86602540378443864676f   /**< sqrt(3)/2      */
#define MOTOR_DEG_TO_RAD      0.01745329251994329577f   /**< PI/180         */
#define MOTOR_RAD_TO_DEG      57.29577951308232087680f  /**< 180/PI         */

/* -------------------------------------------------------------------------- */
/*  HRTIM / PWM 参数 (与 hrtim.c PERIOD=30000 UpDown 模式保持一致)              */
/* -------------------------------------------------------------------------- */

#define HRTIM_PERIOD           30000U     /**< Master 定时器周期 (UpDown 有效=60000) */
#define CMP_CENTER             15000U     /**< 50% 占空比比较值                       */
#define CMP_AMPLITUDE          13500U     /**< 调制幅值 (偏离中心)                    */
#define CMP_MIN                 1500U     /**< 最小比较值 (死区安全余量, ~95% duty)   */
#define CMP_MAX                28500U     /**< 最大比较值 (死区安全余量, ~5% duty)    */

/* -------------------------------------------------------------------------- */
/*  电机参数                                                                    */
/* -------------------------------------------------------------------------- */

#define MOTOR_PWM_FREQ         20000.0f   /**< PWM / 电流环频率 (Hz)                  */
#define MOTOR_POLE_PAIRS       7          /**< 极对数                                  */

/* -------------------------------------------------------------------------- */
/*  电流采样参数                                                                */
/* -------------------------------------------------------------------------- */

#define CURR_SENSE_ADC_VREF         3.3f       /**< ADC 参考电压 (V)              */
#define CURR_SENSE_ADC_RESOLUTION   4096.0f    /**< 12-bit 量程                   */
#define CURR_SENSE_SHUNT_RESISTANCE 0.002f     /**< 采样电阻 2mΩ                  */
#define CURR_SENSE_OPAMP_GAIN       50.0f      /**< 运放增益 100k/2k = 50         */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONFIG_H */
