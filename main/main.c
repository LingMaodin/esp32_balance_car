#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
#include "i2c_bus.h"
#include "bmi270_api.h"

#define TAG "main"
#define PWM_HZ 20000
#define MOTOR_VOLTAGE 7.4//额定电压
#define BUTTRY_VOLTAGE 12.0//电池电压
#define DUTY_RESOLUTION ((1<<10)-1)//占空比分辨率
#define BASE_PWM_DUTY (MOTOR_VOLTAGE/BUTTRY_VOLTAGE*(float)DUTY_RESOLUTION)//基础占空比
#define TARGET_RPM 300.0//目标转速
#define GEAR_RATIO 20.049//减速比
#define ENCODER_PPR 13.0*4//每转脉冲数
#define CONTROL_PERIOD_MS 5e-3//控制周期
#define TARGET_PULSES_PER_PERIOD (TARGET_RPM*GEAR_RATIO*ENCODER_PPR*CONTROL_PERIOD_MS/60.0)//目标每周期脉冲数

typedef struct{
    gpio_num_t pcnt_s1_num;
    gpio_num_t pcnt_s2_num;
    pcnt_unit_handle_t pcnt_unit_hdl;
    pcnt_channel_handle_t pcnt_channel_s1_hdl;
    pcnt_channel_handle_t pcnt_channel_s2_hdl;

    ledc_channel_t ledc_channel;
    gpio_num_t pwm_num;
    gpio_num_t in1_num;
    gpio_num_t in2_num;
}motor_hardware_t;

typedef struct{
    float target_pulses;
    int current_pulses;
    float speed_error;
    float last_speed_error;
    double duty;
    float Kp;
    float Ki;
}motor_software_t;

typedef enum{
    stop=0,
    forward=1,
    backward=2,
}motor_status_t;

typedef struct{
    motor_hardware_t hw;
    motor_software_t sw;
    volatile motor_status_t status;
}motor_t;

typedef struct{
    i2c_bus_handle_t i2c_bus_hdl;
    uint32_t i2c_hz;
    i2c_port_t i2c_port;
    int i2c_sda_num;
    int i2c_scl_num;

    bmi270_handle_t bmi270_hdl;
    uint64_t bmi270_intr_num;

    struct bmi2_sens_data sensor_data;
}bmi270_t;

static volatile TaskHandle_t xTaskToNotify=NULL;//任务通知代替二进制信号量唤醒任务
static motor_t left_motor={
    .hw={
        .pcnt_s1_num=GPIO_NUM_9,
        .pcnt_s2_num=GPIO_NUM_10,
        .pcnt_channel_s1_hdl=NULL,
        .pcnt_channel_s2_hdl=NULL,
        .pcnt_unit_hdl=NULL,
        .ledc_channel=LEDC_CHANNEL_0,
        .pwm_num=GPIO_NUM_11,
        .in1_num=GPIO_NUM_12,
        .in2_num=GPIO_NUM_13,
    },
    .sw={
        .current_pulses=0,
        .speed_error=0,
        .last_speed_error=0,
        .duty=0,
        .Kp=2.0,
        .Ki=0.02,
    },
    .status=stop,
};
static motor_t right_motor={
    .hw={
        .pcnt_s1_num=GPIO_NUM_38,
        .pcnt_s2_num=GPIO_NUM_39,
        .pcnt_channel_s1_hdl=NULL,
        .pcnt_channel_s2_hdl=NULL,
        .pcnt_unit_hdl=NULL,
        .ledc_channel=LEDC_CHANNEL_1,
        .pwm_num=GPIO_NUM_40,
        .in1_num=GPIO_NUM_41,
        .in2_num=GPIO_NUM_42,
    },
    .sw={
        .target_pulses=TARGET_PULSES_PER_PERIOD,
        .current_pulses=0,
        .speed_error=0,
        .last_speed_error=0,
        .duty=0,
        .Kp=2.0,
        .Ki=0.02,
    },
    .status=stop,
};
static bmi270_t bmi270={
    .i2c_bus_hdl=NULL,
    .i2c_hz=400*1000,
    .i2c_port=I2C_NUM_0,
    .i2c_sda_num=GPIO_NUM_4,
    .i2c_scl_num=GPIO_NUM_5,
    .bmi270_hdl=NULL,
    .bmi270_intr_num=GPIO_NUM_6,
    .sensor_data={},
};

static void IRAM_ATTR gpio_isr_edge_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    configASSERT(xTaskToNotify!=NULL);
    vTaskNotifyGiveFromISR(xTaskToNotify,&xHigherPriorityTaskWoken);
    xTaskToNotify=NULL;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

esp_err_t bmi270_init(bmi270_t *sensor)
{
    const i2c_config_t i2c_bus_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sensor->i2c_sda_num,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = sensor->i2c_scl_num,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = sensor->i2c_hz
    };

    sensor->i2c_bus_hdl = i2c_bus_create(sensor->i2c_port, &i2c_bus_conf);
    if (!sensor->i2c_bus_hdl) {
        ESP_LOGE("MAIN", "I2C bus create failed");
        return ESP_FAIL;
    }

    esp_err_t ret = bmi270_sensor_create(sensor->i2c_bus_hdl, &sensor->bmi270_hdl, bmi270_config_file, 0);
    if (ret != ESP_OK || sensor->bmi270_hdl == NULL) {
        ESP_LOGE("MAIN", "BMI270 create failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static int8_t set_accel_gyro_config(bmi270_t *sensor)
{
    int8_t rslt;
    struct bmi2_sens_config config[2];
    struct bmi2_int_pin_config pin_config = { 0 };

    config[BMI2_ACCEL].type = BMI2_ACCEL;
    config[BMI2_GYRO].type = BMI2_GYRO;

    rslt = bmi2_get_int_pin_config(&pin_config, sensor->bmi270_hdl);
    bmi2_error_codes_print_result(rslt);

    rslt = bmi2_get_sensor_config(config, 2, sensor->bmi270_hdl);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
        /* Configure accelerometer output data rate */
        config[BMI2_ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;        /* 200Hz sampling rate */
        config[BMI2_ACCEL].cfg.acc.range = BMI2_ACC_RANGE_4G;      /* ±4G range */
        config[BMI2_ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;      /* Standard averaging */
        config[BMI2_ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; /* Filter performance */

        /* Configure gyroscope output data rate */
        config[BMI2_GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;         /* 200Hz sampling rate */
        config[BMI2_GYRO].cfg.gyr.range = BMI2_GYR_RANGE_250;      /* ±250dps range */
        config[BMI2_GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;       /* Standard filtering */
        config[BMI2_GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;  /* Noise performance */
        config[BMI2_GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; /* Filter performance */

        /* Configure interrupt pin */
        pin_config.pin_type = BMI2_INT1;
        pin_config.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
        pin_config.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;
        pin_config.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
        pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
        pin_config.int_latch = BMI2_INT_LATCH;

        rslt = bmi2_set_int_pin_config(&pin_config, sensor->bmi270_hdl);
        bmi2_error_codes_print_result(rslt);

        rslt = bmi2_set_sensor_config(config, 2, sensor->bmi270_hdl);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

static esp_err_t bmi270_enable(bmi270_t *sensor)
{
    int8_t rslt;
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    // disable aps mode
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, sensor->bmi270_hdl);
    bmi2_error_codes_print_result(rslt);

    // Set accel/gyro config
    rslt = set_accel_gyro_config(sensor);
    bmi2_error_codes_print_result(rslt);

    rslt=bmi2_map_feat_int(BMI2_ANY_MOTION, BMI2_INT1, sensor->bmi270_hdl);
    bmi2_error_codes_print_result(rslt);

    // Enable sensors
    rslt = bmi2_sensor_enable(sens_list, 2, sensor->bmi270_hdl);
    bmi2_error_codes_print_result(rslt);

    return (rslt == BMI2_OK) ? ESP_OK : ESP_FAIL;
}

void gpio_init(const motor_t *motor,const bmi270_t *sensor)//GPIO初始化
{
    gpio_config_t motor_gpio_conf={
        .mode=GPIO_MODE_OUTPUT,//输出模式
        .pin_bit_mask=(1ULL<<motor->hw.in1_num)|(1ULL<<motor->hw.in2_num),//配置in1_num、in2_num为输出
    };
    gpio_config_t sensor_gpio_conf={
        .mode=GPIO_MODE_INPUT,//输入模式
        .pin_bit_mask=(1ULL<<sensor->bmi270_intr_num),
        .pull_down_en=GPIO_PULLDOWN_ENABLE,
        .intr_type=GPIO_INTR_ANYEDGE,//双边沿触发
    };

    ESP_ERROR_CHECK(gpio_config(&motor_gpio_conf));//配置GPIO并判断是否成功
    ESP_ERROR_CHECK(gpio_config(&sensor_gpio_conf));//配置GPIO并判断是否成功
    gpio_install_isr_service(0);
    gpio_isr_handler_add(sensor->bmi270_intr_num, gpio_isr_edge_handler, (void*)(sensor->bmi270_intr_num));
    return;
}

void pcnt_init(motor_t *motor)//pcnt初始化
{
    //s1通道：上升沿+1，下降沿-1，高电平翻转，低电平保持
    //s2通道：上升沿+1，下降沿-1，高电平保持，低电平翻转
    //每次旋转s1、s2各接受一个上升沿一个下降沿，计数单元共计+4或-4
    pcnt_unit_config_t pcnt_unit_cfg={//配置pcnt单元
        .low_limit=-1000,//最小计数值
        .high_limit=1000,//最大计数值
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_cfg,&motor->hw.pcnt_unit_hdl));//新建pcnt单元并判断是否成功

    pcnt_chan_config_t pcnt_channel_s1_cfg={//配置pcnt通道1
        .edge_gpio_num=motor->hw.pcnt_s1_num,//输入s1边沿
        .level_gpio_num=motor->hw.pcnt_s2_num,//输入s2电平
    };
    pcnt_chan_config_t pcnt_channel_s2_cfg={//配置pcnt通道2
        .edge_gpio_num=motor->hw.pcnt_s2_num,//输入s2边沿
        .level_gpio_num=motor->hw.pcnt_s1_num,//输入s1电平
    };
    ESP_ERROR_CHECK(pcnt_new_channel(motor->hw.pcnt_unit_hdl,&pcnt_channel_s1_cfg,&motor->hw.pcnt_channel_s1_hdl));//新建pcnt通道1并判断是否成功
    ESP_ERROR_CHECK(pcnt_new_channel(motor->hw.pcnt_unit_hdl,&pcnt_channel_s2_cfg,&motor->hw.pcnt_channel_s2_hdl));//新建pcnt通道2并判断是否成功

    pcnt_glitch_filter_config_t filte_cfg={//配置毛刺过滤器
        .max_glitch_ns=1000,//过滤1000ns以下毛刺
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(motor->hw.pcnt_unit_hdl, &filte_cfg));//新建毛刺过滤器并判断是否成功
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor->hw.pcnt_channel_s1_hdl,
                                                PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                PCNT_CHANNEL_EDGE_ACTION_DECREASE));//上升沿+1，下降沿-1                                           
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(motor->hw.pcnt_channel_s1_hdl,
                                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
                                                PCNT_CHANNEL_LEVEL_ACTION_KEEP));//高电平翻转，低电平保持                                            
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor->hw.pcnt_channel_s2_hdl,
                                                PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                PCNT_CHANNEL_EDGE_ACTION_DECREASE));//上升沿+1，下降沿-1                                          
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(motor->hw.pcnt_channel_s2_hdl,
                                                PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE));//高电平保持，低电平翻转

    ESP_ERROR_CHECK(pcnt_unit_enable(motor->hw.pcnt_unit_hdl));//使能pcnt单元
    ESP_ERROR_CHECK(pcnt_unit_clear_count(motor->hw.pcnt_unit_hdl));//清除计数值
    ESP_ERROR_CHECK(pcnt_unit_start(motor->hw.pcnt_unit_hdl));//启动pcnt单元
    return;
}

void ledc_init(motor_t *motor)
{
    ledc_timer_config_t ledc_timer_cfg={
        .speed_mode=LEDC_LOW_SPEED_MODE,
        .timer_num=LEDC_TIMER_0,
        .freq_hz=PWM_HZ,
        .duty_resolution=LEDC_TIMER_10_BIT,
        .clk_cfg=LEDC_APB_CLK,
    };
    ledc_timer_config(&ledc_timer_cfg);
    ledc_channel_config_t ledc_channel_cfg={
        .gpio_num=motor->hw.pwm_num,
        .speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=motor->hw.ledc_channel,
        .intr_type=LEDC_INTR_DISABLE,
        .timer_sel=LEDC_TIMER_0,
        .duty=0,
        .hpoint=0,
    };
    ledc_channel_config(&ledc_channel_cfg);
}

esp_err_t read_pcnt(motor_t *motor)
{
    esp_err_t ret = pcnt_unit_get_count(motor->hw.pcnt_unit_hdl,&(motor->sw.current_pulses));
    if (ret != ESP_OK) 
        return ESP_FAIL;
    ret = pcnt_unit_clear_count(motor->hw.pcnt_unit_hdl);
    if (ret != ESP_OK) 
        return ESP_FAIL;
    return ESP_OK;
}

esp_err_t read_bmi270(bmi270_t *sensor)
{
    int8_t rslt = bmi2_get_sensor_data(&sensor->sensor_data, sensor->bmi270_hdl);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) 
        return ESP_FAIL;
    return ESP_OK;
}

void motor_control_task(void *pvParameters)
{
    uint32_t ulNotificationValue;
    esp_err_t ret;
    const TickType_t xMaxBlockTime=pdMS_TO_TICKS(5);  
    configASSERT(xTaskToNotify==NULL);
    xTaskToNotify=xTaskGetCurrentTaskHandle();
    ulNotificationValue = ulTaskNotifyTake(pdTRUE,xMaxBlockTime);
    if(ulNotificationValue==1)
    {
        ret = read_pcnt(&left_motor);
        if (ret != ESP_OK)
            ESP_LOGE(TAG, "Failed to read left motor PCNT");

        ret = read_pcnt(&right_motor);
        if (ret != ESP_OK) 
            ESP_LOGE(TAG, "Failed to read right motor PCNT");

        ret = read_bmi270(&bmi270);
        if (ret != ESP_OK) 
            ESP_LOGE(TAG, "Failed to read BMI270 sensor data");
    }

}

void app_main(void)
{
    ESP_ERROR_CHECK(bmi270_init(&bmi270));//BMI270初始化
    ESP_ERROR_CHECK(bmi270_enable(&bmi270));//BMI270使能
    gpio_init(&left_motor,&bmi270);//GPIO初始化
    gpio_init(&right_motor,&bmi270);//GPIO初始化
    gpio_set_level(left_motor.hw.in1_num,0);
    gpio_set_level(left_motor.hw.in2_num,0);
    gpio_set_level(right_motor.hw.in1_num,0);
    gpio_set_level(right_motor.hw.in2_num,0);    
    pcnt_init(&left_motor);//左电机pcnt初始化
    pcnt_init(&right_motor);//右电机pcnt初始化
    ledc_init(&left_motor);//左电机ledc初始化
    ledc_init(&right_motor);//右电机ledc初始化
    xTaskCreate(motor_control_task,"motor control task",8192,NULL,10,NULL);//创建电机控制任务
}
