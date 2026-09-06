#ifndef HOVERCRAFT_H
#define HOVERCRAFT_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

// Red Light Related Code

void red_light_setup();

//Turn off the Red Light if below the min. Output maximum brightness if past max. Scales brightness between min and max.
template <typename T, typename A> 
void red_light_range(T value,A min,A max){
    uint8_t pwm_value;
    if (value <= min) pwm_value = 250; // To avoid any conflict with other digital sensors. 
    else if (value >= max) pwm_value = 26;
    else pwm_value = 255 - ((value - min) * 229)/(max - min);
    OCR2A = pwm_value;
}

// Clock Related Code

/*Use timer1 as main clock since it has the longest duration.*/

#define F_CPU 16000000UL

#define US_PER_TICKS (1000000/F_CPU)

class hovercraft_clock
{ 
public:
    static volatile uint32_t timer2_ms; // Current clock time in ms
    static volatile uint32_t timer1_overflows;
    volatile uint32_t last_time = 0; // Last ticks for delay
    hovercraft_clock();
    ~hovercraft_clock();
};

unsigned int get_timer0_prescaler();
unsigned int get_timer1_prescaler();
unsigned int get_timer2_prescaler();

void clock0_setup();

void clock1_setup(); //IMU & US time

void clock2_setup();

unsigned int time_to_clock_ticks(uint16_t time_in_ms, unsigned int timer_prescaler);

unsigned int clock_ticks_to_time(uint16_t ticks, unsigned int prescaler);

//Refresh the ticks of the global clock
void update_current_ticks(); //35 min of runtime before overflow. If you want more precise, use ISR and tune timer with prescaler.

// UART Related Code
#define BAUD 9600 //Default
#define MYUBRR F_CPU/16/BAUD-1

class uart : public hovercraft_clock
{
private:
  
public:
  uart();
  ~uart();
};

void UART_Init(unsigned int);

void UART_Transmit(char); //Do not use this function in main file

//Print a sequence of character to the terminal
void UART_Print(char*);

//Print a integer number to the terminal
void UART_Print_Int(int16_t);

//Print a float number to the terminal
void UART_Print_Float(float);

// US Related Code

//Configure depending on hardware
#define TRIGGER_PIN  3  // PB3
#define ECHO_PIN     2  // PD2 (INT0)

class ultrasonic : public hovercraft_clock
{
private:
  
public:
  volatile uint32_t pulse_start = 0;
  volatile uint32_t pulse_end = 0;
  volatile uint8_t measurement_ready = 0; // Allows program to process data
  volatile uint8_t distance_cm = 0;
  ultrasonic(/* args */);
  ~ultrasonic();

  // Trigger a measurement
  void ultrasonic_trigger(void);

  // Check if measurement is ready
  uint8_t ultrasonic_ready(void);

  //Forces non-ready state. TIP: Usually at the end of main.
  void ultrasonic_notready(void);

  // Get distance in centimeters
  uint16_t ultrasonic_get_cm(void);

  // Get raw pulse duration (microseconds)
  uint32_t ultrasonic_get_pulse_us(void);
};

// Initialize sensor for INT0 or both INT0 and INT1
void ultrasonic_init(int8_t select);

//IMU Related Code

//For quick code setup
#define ACCEL_SENS   2 // ±2g, ±4g, ±8g and ±16g
#define GYRO_SENS    250 // ±250, ±500, ±1000, and ±2000°/sec
#define TO_CM 1 //0 for m, 1 for cm

// MPU6050 Configuration
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define CONFIG       0x1A

//Gravity Setup
#if ACCEL_SENS == 2
const int16_t GRAVITY_LSB = 16384;
#elif ACCEL_SENS == 4
const int16_t GRAVITY_LSB = 8192;
#elif ACCEL_SENS == 8
const int16_t GRAVITY_LSB = 4096;
#elif ACCEL_SENS == 16
const int16_t GRAVITY_LSB = 2048;
#else
const int16_t GRAVITY_LSB = 16384;
#endif

// Filter Configuration
#define FILTER_TYPE      1    // 0=Moving Average, 1=EMA
#define FILTER_WINDOW    8    // Power of 2 for MA
#define EMA_ALPHA       0.2f  // EMA smoothing factor
#define ALPHA 0.96  // Adjusted for better stability
#define DT 0.02f    // 50Hz update rate

class MPU6050 : public hovercraft_clock{
    private:
    //Time
    float gyro_pitch_bias = 0, gyro_roll_bias = 0;

    // Calibration offsets
    int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
    int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

    public:
    typedef struct {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
    } SensorData;
    
    typedef struct {
      float ax_g, ay_g, az_g;   // Accelerometer in g-forces
      float ax_ms2, ay_ms2, az_ms2; // Accelerometer in m/s²
      float gx_dps, gy_dps, gz_dps; // Gyro in degrees/s
      float gx_rps, gy_rps, gz_rps; // Gyro in radians/s
    } ConvertedData;
    
    typedef struct {
      float pitch;  // X-axis rotation
      float roll;   // Y-axis rotation
      float yaw;    // Z-axis rotation
    } Angles;

    static volatile uint8_t imu_ready; // Flag for IMU data ready
    SensorData raw, filtered;
    ConvertedData converted;
    Angles angles = {0};
    float velocity_x = 0.0f, position_x = 0.0f;
    volatile float DT_actual = DT;

    #if FILTER_TYPE == 0
    // Moving Average Implementation
    SensorData filter_buf[FILTER_WINDOW];
    uint8_t filter_idx = 0;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    #else
    // EMA Implementation
    SensorData ema = {0};
    #endif

    MPU6050();
    ~MPU6050();

    void read_accel();
    void read_gyro();

    void apply_offset();

    //EMA filtering
    void apply_filter();

    void GyroCalibration();
    void AccelCalibration();

    void ConvertUnits();
    void CalculateAngles();
    void CalculatePosition(float threshold);

};

void I2C_Init(); //Included in MPU6050_Init
    
void I2C_Start(); //Included in MPU6050_Init

void I2C_Stop(); //Included in MPU6050_Init

void I2C_Write(uint8_t); //Included in MPU6050_Init

uint8_t I2C_Read_Ack(); //Included in MPU6050_Init

uint8_t I2C_Read_Nack(); //Included in MPU6050_Init

void MPU6050_Init();

//Status LED Related Code
#define STATUS_LED  5  // Sector B 

void status_led_init();

template <typename T, typename A> 
void status_led_toggle(T value, A min, A max){
  if (value < min || value > max) {
    PORTB |= (1 << STATUS_LED);
  } 
  else {
    PORTB &= ~(1 << STATUS_LED);
  }
}

template <typename T, typename A, typename B, typename C> 
void status_led_blink(T value, A min, A max, B current_time, B last_time, C delay_desired){
  if (value < min || value > max) {
    if (current_time - last_time >= delay_desired) { // 250ms @ prescaler 8 (0.5μs/tick) + 250ms or more required => 250ms/0.5μs/tick = 500000 ticks
        PORTB ^= (1 << STATUS_LED);
        last_time = current_time;
    }
  } 
  else {
      PORTB &= ~(1 << STATUS_LED);
  }
}

//Servo
void servo_setup();
void servo_set_angle(uint8_t angle); // 0-180 degrees

//Fan
void fans_max_speed_setup();

#endif