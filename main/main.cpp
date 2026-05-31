#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "bmi270.hpp"

#define TAG "main"

#define TIMER_PERIOD_US 5*1000 //定时器周期,单位us
#define PWM_HZ 20*1000 //PWM频率,单位Hz
#define BASE_PWM_DUTY (7.4/12.0*(float)((1<<10)-1))
#define ENCODER_PPR 13.0*4//每转脉冲数
#define TARGET_RPM 300.0//目标转速
#define GEAR_RATIO 20.049//减速比
#define TARGET_PULSES_PER_PERIOD (TARGET_RPM*GEAR_RATIO*ENCODER_PPR*(TIMER_PERIOD_US/1e6)/60.0)//目标每周期脉冲数
#define BMI270_MISO_PIN GPIO_NUM_4
#define BMI270_MOSI_PIN GPIO_NUM_5
#define BMI270_SCLK_PIN GPIO_NUM_6
#define BMI270_CS_PIN   GPIO_NUM_7 

typedef struct{
    esp_timer_handle_t esptimer_hdl;

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
    motor_status_t status;
}motor_t;

static motor_t left_motor={
    .hw={
        .esptimer_hdl=NULL,
        .pcnt_s1_num=GPIO_NUM_9,
        .pcnt_s2_num=GPIO_NUM_10,
        .pcnt_unit_hdl=NULL,
        .pcnt_channel_s1_hdl=NULL,
        .pcnt_channel_s2_hdl=NULL,
        .ledc_channel=LEDC_CHANNEL_0,
        .pwm_num=GPIO_NUM_11,
        .in1_num=GPIO_NUM_12,
        .in2_num=GPIO_NUM_13,
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
static motor_t right_motor={
    .hw={
        .esptimer_hdl=NULL,
        .pcnt_s1_num=GPIO_NUM_38,
        .pcnt_s2_num=GPIO_NUM_39,
        .pcnt_unit_hdl=NULL,
        .pcnt_channel_s1_hdl=NULL,
        .pcnt_channel_s2_hdl=NULL,
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
static spi_device_handle_t bmi270_spi_handle;

// BMI270 SPI 写回调函数（兼容 I2C 模板：address 参数在 SPI 模式下不使用，由 CS 引脚选片）
bool bmi270_spi_write(uint8_t address, const uint8_t *data, size_t length) {
    if (length == 0) return false;
    spi_transaction_t t = {};
    t.length = length * 8;
    t.tx_buffer = data;
    return spi_device_transmit(bmi270_spi_handle, &t) == ESP_OK;
}

// BMI270 SPI 读寄存器回调函数（兼容 I2C 模板：address 参数在 SPI 模式下不使用）
bool bmi270_spi_read(uint8_t address, uint8_t *data, size_t length) {
    if (length == 0) return false;
    spi_transaction_t t = {};
    t.length = length * 8;
    t.rxlength = length * 8;
    t.tx_buffer = data; // 写入寄存器地址(和dummy bytes(如果有))
    t.rx_buffer = data;
    return spi_device_transmit(bmi270_spi_handle, &t) == ESP_OK;
}

// BMI270 初始化与读取示例
void bmi270_init() {
    ESP_LOGI(TAG, "Initializing SPI for BMI270");
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = BMI270_MISO_PIN;
    buscfg.mosi_io_num = BMI270_MOSI_PIN;
    buscfg.sclk_io_num = BMI270_SCLK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 8192 + 8; // BMI270初始化需上传约8KB配置文件

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 10 * 1000 * 1000; // 10MHz 时钟
    devcfg.mode = 0; // SPI 模式 0
    devcfg.spics_io_num = BMI270_CS_PIN;
    devcfg.queue_size = 7;
    
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &bmi270_spi_handle));

    using Imu = espp::Bmi270<>;  // 使用 I2C 模板（默认），SPI 回调忽略 address 参数

    Imu::Config config = {
        .write = bmi270_spi_write,
        .read = bmi270_spi_read,
        .imu_config = {
            .accelerometer_range = Imu::AccelerometerRange::RANGE_4G,
            .accelerometer_odr = Imu::AccelerometerODR::ODR_100_HZ,
            .accelerometer_bandwidth = Imu::AccelerometerBandwidth::NORMAL_AVG4,
            .gyroscope_range = Imu::GyroscopeRange::RANGE_1000DPS,
            .gyroscope_odr = Imu::GyroscopeODR::ODR_100_HZ,
            .gyroscope_bandwidth = Imu::GyroscopeBandwidth::NORMAL_MODE,
            .gyroscope_performance_mode = Imu::GyroscopePerformanceMode::PERFORMANCE_OPTIMIZED,
        },
        .burst_write_size = 1024,
        .auto_init = true,
        .log_level = espp::Logger::Verbosity::INFO,
    };

    // 初始化 IMU，此时内部将完成 8KB 配置文件加载及初始化
    static Imu imu(config);

    // 初始化完毕后，提取一次原始数据打印测试
    std::error_code ec;
    auto accel = imu.get_accelerometer();
    auto gyro = imu.get_gyroscope();
    
    ESP_LOGI(TAG, "BMI270 Accel: X=%.2f, Y=%.2f, Z=%.2f", accel.x, accel.y, accel.z);
    ESP_LOGI(TAG, "BMI270 Gyro : X=%.2f, Y=%.2f, Z=%.2f", gyro.x, gyro.y, gyro.z);
}

void gpio_init(const motor_t motor)//GPIO初始化
{
    ESP_LOGI(TAG,"Initializing GPIO IN1:%d and IN2:%d",motor.hw.in1_num,motor.hw.in2_num);
    gpio_config_t gpio_conf = {};
    gpio_conf.pin_bit_mask = (1ULL<<motor.hw.in1_num)|(1ULL<<motor.hw.in2_num);//配置in1_num、in2_num为输出
    gpio_conf.mode=GPIO_MODE_OUTPUT;//输出模式
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));//配置GPIO并判断是否成功
    return;
}

void pcnt_init(motor_t *motor)//pcnt初始化
{
    //s1通道：上升沿+1，下降沿-1，高电平翻转，低电平保持
    //s2通道：上升沿+1，下降沿-1，高电平保持，低电平翻转
    //每次旋转s1、s2各接受一个上升沿一个下降沿，计数单元共计+4或-4
    ESP_LOGI(TAG,"Initializing PCNT uint");
    pcnt_unit_config_t pcnt_unit_cfg = {};//配置pcnt单元
    pcnt_unit_cfg.low_limit=-1000;//最小计数值
    pcnt_unit_cfg.high_limit=1000;//最大计数值
    
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_cfg,&motor->hw.pcnt_unit_hdl));//新建pcnt单元并判断是否成功
    ESP_LOGI(TAG,"Initializing PCNT channels S1:%d and S2:%d",motor->hw.pcnt_s1_num,motor->hw.pcnt_s2_num);
    pcnt_chan_config_t pcnt_channel_s1_cfg = {};//配置pcnt通道1
    pcnt_channel_s1_cfg.edge_gpio_num=motor->hw.pcnt_s1_num;//输入s1边沿
    pcnt_channel_s1_cfg.level_gpio_num=motor->hw.pcnt_s2_num;//输入s2电平
    
    pcnt_chan_config_t pcnt_channel_s2_cfg = {};//配置pcnt通道2
    pcnt_channel_s2_cfg.edge_gpio_num=motor->hw.pcnt_s2_num;//输入s2边沿
    pcnt_channel_s2_cfg.level_gpio_num=motor->hw.pcnt_s1_num;//输入s1电平
    
    ESP_ERROR_CHECK(pcnt_new_channel(motor->hw.pcnt_unit_hdl,&pcnt_channel_s1_cfg,&motor->hw.pcnt_channel_s1_hdl));//新建pcnt通道1并判断是否成功
    ESP_ERROR_CHECK(pcnt_new_channel(motor->hw.pcnt_unit_hdl,&pcnt_channel_s2_cfg,&motor->hw.pcnt_channel_s2_hdl));//新建pcnt通道2并判断是否成功
    ESP_LOGI(TAG,"Initializing PCNT glitch filter");
    pcnt_glitch_filter_config_t filte_cfg = {};//配置毛刺过滤器
    filte_cfg.max_glitch_ns=1000;//过滤1000ns以下毛刺
    
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
    ESP_LOGI(TAG,"Enabling PCNT unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(motor->hw.pcnt_unit_hdl));//使能pcnt单元
    ESP_ERROR_CHECK(pcnt_unit_clear_count(motor->hw.pcnt_unit_hdl));//清除计数值
    ESP_ERROR_CHECK(pcnt_unit_start(motor->hw.pcnt_unit_hdl));//启动pcnt单元
    return;
}

void esptimer_cb(void *arg)//定时器回调函数
{
    double delta_duty=0;
    motor_t *motor = (motor_t *)arg;
    pcnt_unit_get_count(motor->hw.pcnt_unit_hdl,&(motor->sw.current_pulses));
    pcnt_unit_clear_count(motor->hw.pcnt_unit_hdl);
    switch (motor->status)
    {
    case stop:
        motor->sw.speed_error=0;
        motor->sw.last_speed_error=0;
        motor->sw.duty=0;
        break;
    case forward:
    case backward:
        motor->sw.speed_error=motor->sw.target_pulses-(float)abs(motor->sw.current_pulses);
        break;
    default:
        break;
    }

    delta_duty=motor->sw.Kp*(motor->sw.speed_error-motor->sw.last_speed_error)+motor->sw.Ki*motor->sw.speed_error;
    if(delta_duty>100) delta_duty=100;
    if(delta_duty<-100) delta_duty=-100;
    
    motor->sw.duty+=delta_duty;
    if (motor->sw.duty>1023) motor->sw.duty=1023;
    if (motor->sw.duty<0) motor->sw.duty=0;
    
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,motor->hw.ledc_channel,(uint32_t)motor->sw.duty,0);
    motor->sw.last_speed_error=motor->sw.speed_error;
}

void esptimer_init(motor_t *motor)
{
    ESP_LOGI(TAG,"Initializing ESP timer");
    esp_timer_create_args_t esptimer_cfg = {};
    esptimer_cfg.callback=esptimer_cb;
    esptimer_cfg.arg=motor;
    esptimer_cfg.dispatch_method=ESP_TIMER_TASK;
    esptimer_cfg.name="pulse_count_reader";
    
    ESP_ERROR_CHECK(esp_timer_create(&esptimer_cfg,&motor->hw.esptimer_hdl));
    ESP_ERROR_CHECK(esp_timer_start_periodic(motor->hw.esptimer_hdl,TIMER_PERIOD_US));
}

void ledc_init(motor_t motor)
{
    ESP_LOGI(TAG,"Initializing LEDC timer");
    ledc_timer_config_t ledc_timer_cfg = {};
    ledc_timer_cfg.speed_mode=LEDC_LOW_SPEED_MODE;
    ledc_timer_cfg.duty_resolution=LEDC_TIMER_10_BIT;
    ledc_timer_cfg.timer_num=LEDC_TIMER_0;
    ledc_timer_cfg.freq_hz=PWM_HZ;
    ledc_timer_cfg.clk_cfg=LEDC_USE_APB_CLK;
    
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_cfg));//配置ledc定时器并判断是否成功
    ESP_LOGI(TAG,"Initializing LEDC channel:%d on GPIO:%d",motor.hw.ledc_channel,motor.hw.pwm_num);
    ledc_channel_config_t ledc_channel_cfg = {};
    ledc_channel_cfg.gpio_num=motor.hw.pwm_num;
    ledc_channel_cfg.speed_mode=LEDC_LOW_SPEED_MODE;
    ledc_channel_cfg.channel=motor.hw.ledc_channel;
    ledc_channel_cfg.intr_type=LEDC_INTR_DISABLE;
    ledc_channel_cfg.timer_sel=LEDC_TIMER_0;
    ledc_channel_cfg.duty=0;
    ledc_channel_cfg.hpoint=0;
    
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cfg));//配置ledc通道并判断是否成功
}

extern "C" void app_main(void)
{
    gpio_init(left_motor);//GPIO初始化
    gpio_init(right_motor);//GPIO初始化
    gpio_set_level(left_motor.hw.in1_num,0);
    gpio_set_level(left_motor.hw.in2_num,0);
    gpio_set_level(right_motor.hw.in1_num,0);
    gpio_set_level(right_motor.hw.in2_num,0);    
    pcnt_init(&left_motor);//左电机pcnt初始化
    pcnt_init(&right_motor);//右电机pcnt初始化
    esptimer_init(&left_motor);//左电机esptimer初始化
    esptimer_init(&right_motor);//右电机esptimer初始化
    ledc_init(left_motor);//左电机ledc初始化
    ledc_init(right_motor);//右电机ledc初始化

    // 初始化BMI270和SPI总线
    bmi270_init();
}
