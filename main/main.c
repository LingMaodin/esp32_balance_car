#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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
#define TARGET_PULSES_PER_PERIOD 0//目标每周期脉冲数

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
    int current_pulses;
    float pulse_error;
    float last_pulse_error;
    float Kp;
    float Ki;
    float speed_output;
}speed_controller_t;

typedef struct{
    float target_angle;
    float current_angle;
    float last_angle;
    float angle_error;
    float last_angle_error;
    float Kp;
    float Kd;
    float balance_output;
}balance_controller_t;

typedef struct{
    float xxxx_output
}xxxx_controller_t;

typedef enum{
    STOP=0,
    FORWARD=1,
    BACKWARD=2,
}direction_t;

typedef struct{
    uint32_t left_motor_duty;
    uint32_t right_motor_duty;
    direction_t direction;
}duty_t;

typedef struct{
    motor_hardware_t left_motor;
    motor_hardware_t right_motor;
    speed_controller_t speed;
    balance_controller_t balance;
    xxxx_controller_t xxxx;
    duty_t output;
}chassis_t;

typedef struct{
    i2c_bus_handle_t i2c_bus_hdl;
    uint32_t i2c_hz;
    i2c_port_t i2c_port;
    int i2c_sda_num;
    int i2c_scl_num;

    bmi270_handle_t bmi270_hdl;
    uint64_t bmi270_intr_num;

    struct bmi2_sens_data sensor_data;
    float accel_angle;
    float gyro_dps;
}bmi270_t;

static volatile TaskHandle_t xTaskToNotify=NULL;//任务通知代替二进制信号量唤醒任务
static chassis_t chassis={
    .left_motor={
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
    .right_motor={
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
    .speed={
        .current_pulses=0,
        .pulse_error=0,
        .last_pulse_error=0,
        .Kp=2.0,
        .Ki=0.02,
        .speed_output=0,
    },
    .balance={
        .target_angle=0,
        .current_angle=0,
        .last_angle=0,
        .angle_error=0,
        .last_angle_error=0,
        .Kp=0,
        .Kd=0,
        .balance_output=0,
    },
    .output={
        .left_motor_duty=0,
        .right_motor_duty=0,
        .direction=STOP,
    },
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
    .accel_angle=0,
    .gyro_dps=0,
};

static void IRAM_ATTR gpio_isr_edge_handler(void* arg)
{
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

void interrupt_gpio_init(const bmi270_t *sensor)
{
    gpio_config_t sensor_gpio_conf={
        .mode=GPIO_MODE_INPUT,//输入模式
        .pin_bit_mask=(1ULL<<sensor->bmi270_intr_num),
        .pull_down_en=GPIO_PULLDOWN_ENABLE,
        .intr_type=GPIO_INTR_ANYEDGE,//双边沿触发
    };
    ESP_ERROR_CHECK(gpio_config(&sensor_gpio_conf));//配置GPIO并判断是否成功
    gpio_install_isr_service(0);
    gpio_isr_handler_add(sensor->bmi270_intr_num, gpio_isr_edge_handler,NULL);    
}

void gpio_init(const motor_hardware_t *motor)//GPIO初始化
{
    gpio_config_t motor_gpio_conf={
        .mode=GPIO_MODE_OUTPUT,//输出模式
        .pin_bit_mask=(1ULL<<motor->in1_num)|(1ULL<<motor->in2_num),//配置in1_num、in2_num为输出
    };
    ESP_ERROR_CHECK(gpio_config(&motor_gpio_conf));//配置GPIO并判断是否成功
}

void pcnt_init(motor_hardware_t *motor)//pcnt初始化
{
    //s1通道：上升沿+1，下降沿-1，高电平翻转，低电平保持
    //s2通道：上升沿+1，下降沿-1，高电平保持，低电平翻转
    //每次旋转s1、s2各接受一个上升沿一个下降沿，计数单元共计+4或-4
    pcnt_unit_config_t pcnt_unit_cfg={//配置pcnt单元
        .low_limit=-1000,//最小计数值
        .high_limit=1000,//最大计数值
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_cfg,&motor->pcnt_unit_hdl));//新建pcnt单元并判断是否成功

    pcnt_chan_config_t pcnt_channel_s1_cfg={//配置pcnt通道1
        .edge_gpio_num=motor->pcnt_s1_num,//输入s1边沿
        .level_gpio_num=motor->pcnt_s2_num,//输入s2电平
    };
    pcnt_chan_config_t pcnt_channel_s2_cfg={//配置pcnt通道2
        .edge_gpio_num=motor->pcnt_s2_num,//输入s2边沿
        .level_gpio_num=motor->pcnt_s1_num,//输入s1电平
    };
    ESP_ERROR_CHECK(pcnt_new_channel(motor->pcnt_unit_hdl,&pcnt_channel_s1_cfg,&motor->pcnt_channel_s1_hdl));//新建pcnt通道1并判断是否成功
    ESP_ERROR_CHECK(pcnt_new_channel(motor->pcnt_unit_hdl,&pcnt_channel_s2_cfg,&motor->pcnt_channel_s2_hdl));//新建pcnt通道2并判断是否成功

    pcnt_glitch_filter_config_t filte_cfg={//配置毛刺过滤器
        .max_glitch_ns=1000,//过滤1000ns以下毛刺
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(motor->pcnt_unit_hdl, &filte_cfg));//新建毛刺过滤器并判断是否成功
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor->pcnt_channel_s1_hdl,
                                                PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                PCNT_CHANNEL_EDGE_ACTION_DECREASE));//上升沿+1，下降沿-1                                           
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(motor->pcnt_channel_s1_hdl,
                                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
                                                PCNT_CHANNEL_LEVEL_ACTION_KEEP));//高电平翻转，低电平保持                                            
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(motor->pcnt_channel_s2_hdl,
                                                PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                PCNT_CHANNEL_EDGE_ACTION_DECREASE));//上升沿+1，下降沿-1                                          
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(motor->pcnt_channel_s2_hdl,
                                                PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE));//高电平保持，低电平翻转

    ESP_ERROR_CHECK(pcnt_unit_enable(motor->pcnt_unit_hdl));//使能pcnt单元
    ESP_ERROR_CHECK(pcnt_unit_clear_count(motor->pcnt_unit_hdl));//清除计数值
    ESP_ERROR_CHECK(pcnt_unit_start(motor->pcnt_unit_hdl));//启动pcnt单元
    return;
}

void ledc_init(motor_hardware_t *motor)
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
        .gpio_num=motor->pwm_num,
        .speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=motor->ledc_channel,
        .intr_type=LEDC_INTR_DISABLE,
        .timer_sel=LEDC_TIMER_0,
        .duty=0,
        .hpoint=0,
    };
    ledc_channel_config(&ledc_channel_cfg);
}

void read_pcnt(motor_hardware_t *motor, int *current_pulses)
{
    esp_err_t ret = pcnt_unit_get_count(motor->pcnt_unit_hdl, current_pulses);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get PCNT count");
        return;
    }
    pcnt_unit_clear_count(motor->pcnt_unit_hdl);
}

void read_bmi270(bmi270_t *sensor)
{
    int8_t rslt = bmi2_get_sensor_data(&sensor->sensor_data, sensor->bmi270_hdl);
    bmi2_error_codes_print_result(rslt);
    float acc_g_x=sensor->sensor_data.acc.x/8192.0f;
    float acc_g_z=sensor->sensor_data.acc.z/8192.0f;
    sensor->accel_angle=atan2f(-acc_g_x,acc_g_z)*(180.0f/M_PI);
    sensor->gyro_dps=sensor->sensor_data.gyr.y/131.072f;
}

void speed_controller(chassis_t *chassis,int average_pulses)
{
    chassis->speed.pulse_error = TARGET_PULSES_PER_PERIOD - average_pulses;
    chassis->speed.speed_output += chassis->speed.Kp * (chassis->speed.pulse_error - chassis->speed.last_pulse_error)
                                   + chassis->speed.Ki * chassis->speed.pulse_error;
    //限制目标角度在[-10,10]范围内
    if (chassis->speed.speed_output > 10) 
        chassis->speed.speed_output = 10;
    else if (chassis->speed.speed_output < -10) 
        chassis->speed.speed_output = -10;

    chassis->balance.target_angle = chassis->speed.speed_output;//将目标角度传递给平衡环

    chassis->speed.last_pulse_error = chassis->speed.pulse_error;//更新上一次脉冲误差
}

void balance_controller(chassis_t *chassis)
{
    chassis->balance.angle_error=chassis->balance.target_angle-chassis->balance.current_angle;
    chassis->balance.balance_output=chassis->balance.Kp*chassis->balance.angle_error
                        +chassis->balance.Kd*(chassis->balance.angle_error-chassis->balance.last_angle_error);

    chassis->balance.last_angle_error=chassis->balance.angle_error;
}

void xxxx_controller(chassis_t *chassis)
{

}

void calculate_duty(chassis_t *chassis)
{
    if (chassis->balance.balance_output>0)
    {
        chassis->output.direction=FORWARD;
        chassis->output.left_motor_duty=(uint32_t)(chassis->balance.balance_output+chassis->xxxx.xxxx_output);
        chassis->output.right_motor_duty=(uint32_t)(chassis->balance.balance_output-chassis->xxxx.xxxx_output);
    }
    else if (chassis->balance.balance_output<0)
    {
        chassis->output.direction=BACKWARD;
        chassis->output.left_motor_duty=(uint32_t)fabsf(chassis->balance.balance_output+chassis->xxxx.xxxx_output);
        chassis->output.right_motor_duty=(uint32_t)fabsf(chassis->balance.balance_output-chassis->xxxx.xxxx_output);
    }
    else
        chassis->output.direction=STOP;
}

void set_duty(chassis_t *chassis)
{
    switch (chassis->output.direction)
    {
    case STOP:
        gpio_set_level(chassis->left_motor.in1_num,0);
        gpio_set_level(chassis->left_motor.in2_num,0);
        gpio_set_level(chassis->right_motor.in1_num,0);
        gpio_set_level(chassis->right_motor.in2_num,0);
        chassis->output.left_motor_duty=0;
        chassis->output.right_motor_duty=0;
        break;
    case FORWARD:
        gpio_set_level(chassis->left_motor.in1_num,1);
        gpio_set_level(chassis->left_motor.in2_num,0);
        gpio_set_level(chassis->right_motor.in1_num,1);
        gpio_set_level(chassis->right_motor.in2_num,0);
        break;
    case BACKWARD:
        gpio_set_level(chassis->left_motor.in1_num,0);
        gpio_set_level(chassis->left_motor.in2_num,1);
        gpio_set_level(chassis->right_motor.in1_num,0);
        gpio_set_level(chassis->right_motor.in2_num,1);
        break;
    default:
        break;
    }
    if (chassis->output.left_motor_duty>BASE_PWM_DUTY)
        chassis->output.left_motor_duty=BASE_PWM_DUTY;
    if (chassis->output.right_motor_duty>BASE_PWM_DUTY)
        chassis->output.right_motor_duty=BASE_PWM_DUTY;
    
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,chassis->left_motor.ledc_channel,chassis->output.left_motor_duty,0);
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,chassis->right_motor.ledc_channel,chassis->output.right_motor_duty,0);
}

void motor_control_task(void *pvParameters)
{
    uint32_t ulNotificationValue;
    const TickType_t xMaxBlockTime=pdMS_TO_TICKS(5);
    int left_pulses=0;
    int right_pulses=0;
    int pulse_average=0;
    int times=0;
    while(1)
    {
        configASSERT(xTaskToNotify==NULL);
        xTaskToNotify=xTaskGetCurrentTaskHandle();
        ulNotificationValue = ulTaskNotifyTake(pdTRUE,xMaxBlockTime);
        if(ulNotificationValue==1)
        {
            if(times>=4)//上电后前4次速度环开环
            {
                //外环速度环
                read_pcnt(&chassis.left_motor, &left_pulses);
                read_pcnt(&chassis.right_motor, &right_pulses);
                pulse_average=(left_pulses+right_pulses)/2;
                speed_controller(&chassis,pulse_average);
                times=0;
            }
            times++;
            //内环直立环
            read_bmi270(&bmi270); 
            chassis.balance.current_angle=0.95*(chassis.balance.last_angle+bmi270.gyro_dps*0.005)+0.05*bmi270.accel_angle;//一阶互补滤波
            chassis.balance.last_angle=chassis.balance.current_angle;
            balance_controller(&chassis);
            //转向环

            //计算占空比
            calculate_duty(&chassis);
            //设置占空比
            set_duty(&chassis);
        }
        else
            ESP_LOGW(TAG,"Main task timeout");
    }
}

void app_main(void)
{
    interrupt_gpio_init(&bmi270);//中断GPIO初始化
    ESP_ERROR_CHECK(bmi270_init(&bmi270));//BMI270初始化
    ESP_ERROR_CHECK(bmi270_enable(&bmi270));//BMI270使能
    gpio_init(&chassis.left_motor);//GPIO初始化
    gpio_init(&chassis.right_motor);//GPIO初始化
    gpio_set_level(chassis.left_motor.in1_num,0);
    gpio_set_level(chassis.left_motor.in2_num,0);
    gpio_set_level(chassis.right_motor.in1_num,0);
    gpio_set_level(chassis.right_motor.in2_num,0);
    pcnt_init(&chassis.left_motor);//左电机pcnt初始化
    pcnt_init(&chassis.right_motor);//右电机pcnt初始化
    ledc_init(&chassis.left_motor);//左电机ledc初始化
    ledc_init(&chassis.right_motor);//右电机ledc初始化
    xTaskCreate(motor_control_task,"motor control task",8192,NULL,10,NULL);//创建电机控制任务
}
