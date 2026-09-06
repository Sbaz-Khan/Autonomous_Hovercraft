#include "hovercraft.h"

#define PWM_LED_PIN  3  // Sector B 


// Red Light Related Code

void red_light_setup(){
    clock2_setup();
    DDRB |= (1 << PWM_LED_PIN);  // PB3 as output, sets also the PWM_LED_PIN to output
}

// Clock Related Code

volatile uint32_t hovercraft_clock:: timer2_ms = 0;
volatile uint32_t hovercraft_clock:: timer1_overflows = 0;

hovercraft_clock::hovercraft_clock()
{
    last_time = 0;
}

hovercraft_clock::~hovercraft_clock()
{
}

unsigned int get_timer0_prescaler() {
    uint8_t cs_bits = TCCR0B & ((1 << CS02) | (1 << CS01) | (1 << CS00));
    switch (cs_bits) {
      case (1 << CS00): return 1;
      case (1 << CS01): return 8;
      case (1 << CS01 | 1 << CS00): return 64;
      case (1 << CS02): return 256;
      case (1 << CS02 | 1 << CS00): return 1024;
      default: return 0; // Error
    }
}

unsigned int get_timer1_prescaler() {
    uint8_t cs_bits = TCCR1B & ((1 << CS12) | (1 << CS11) | (1 << CS10));
    switch (cs_bits) {
      case (1 << CS10): return 1;
      case (1 << CS11): return 8;
      case (1 << CS11 | 1 << CS10): return 64;
      case (1 << CS12): return 256;
      case (1 << CS12 | 1 << CS10): return 1024;
      default: return 0; // Error
    }
}

unsigned int get_timer2_prescaler() {
    uint8_t cs_bits = TCCR2B & ((1 << CS22) | (1 << CS21) | (1 << CS20));
    switch (cs_bits) {
      case (1 << CS20): return 1;
      case (1 << CS21): return 8;
      case (1 << CS21 | 1 << CS20): return 64;
      case (1 << CS22): return 256;
      case (1 << CS22 | 1 << CS20): return 1024;
      default: return 0; // Error
    }
}

void clock0_setup(){
    TCCR0A = (1 << WGM01); // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
    OCR0A = 249; // 1ms interval
    TIMSK0 |= (1 << OCIE0A); // Enable interrupt
}


void clock1_setup() {
    TCCR1A = 0;                     // Normal mode
    TCCR1B = (1 << WGM12) |         // CTC mode
             (1 << CS11);           // Prescaler = 8 (0.5µs/tick @ 16MHz)
    OCR1A = 1999;                   // 1ms interval: (16000000 / 8) * 0.001 - 1 = 1999
}

void clock2_setup() {
    TCCR2A = (1 << WGM21); // CTC mode
    TCCR2B = (1 << CS21) | (1 << CS20); // Prescaler 64
    OCR2A = 249; // 1ms interval
    TIMSK2 |= (1 << OCIE2A); // Enable interrupt
}

unsigned int time_to_clock_ticks(uint16_t ms, unsigned int prescaler){
    return ms/(US_PER_TICKS*prescaler)*1000;
}

unsigned int clock_ticks_to_time(uint16_t ticks, unsigned int prescaler){
    return ticks*(US_PER_TICKS*prescaler)/1000;
}

ISR(TIMER2_COMPA_vect) {
    hovercraft_clock::timer2_ms++; 
}

//UART Related Code

uart::uart(){

}
uart::~uart(){

}

void UART_Init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (3 << UCSZ00);
}

void UART_Transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_Print(char *str) {
    while (*str) UART_Transmit(*str++);
}

void UART_Print_Int(int16_t num) {
    char buffer[7];
    itoa(num, buffer, 10);
    UART_Print(buffer);
}

void UART_Print_Float(float num) {
    char buffer[10];
    dtostrf(num, 6, 2, buffer);
    UART_Print(buffer);
}

//US Related Code
// Internal variables
ultrasonic::ultrasonic(/* args */){

}
ultrasonic::~ultrasonic(){
    
}

// Initialize US for INT0 or both INT0 and INT1
void ultrasonic_init(int8_t select) {
    // Configure Timer1
    clock1_setup();

    switch (select)
    {
    case 2 :
        EICRA = (1 << ISC11) | (1 << ISC10); // Rising/Falling edge trigger
        EIMSK = (1 << INT1);
        DDRB |= (1 << 5);  //Trigger PB5 as output
        DDRD &= ~(1 << 3);    //Echo PD3 as input
    case 1 :
        EICRA = (1 << ISC01) | (1 << ISC00); // Rising/Falling edge trigger
        EIMSK = (1 << INT0);
        DDRB |= (1 << 3);  //Trigger PB3 as output, sets also the PWM_LED_PIN to output
        DDRD &= ~(1 << 2);    //Echo PD2 as input
        break;
    
    default:
        EICRA = (1 << ISC01) | (1 << ISC00); // Rising/Falling edge trigger
        EIMSK = (1 << INT0);
        DDRB |= (1 << 3);  //Trigger PB3 as output, sets also the PWM_LED_PIN to output
        DDRD &= ~(1 << 2);    //Echo PD2 as input
        break;
    }
}

// Trigger a pulse
void ultrasonic::ultrasonic_trigger(void) {
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);
}

// Check measurement status
uint8_t ultrasonic::ultrasonic_ready(void) {
    return measurement_ready;
}

void ultrasonic::ultrasonic_notready(void){
    measurement_ready = 0;
}

// Get distance in cm
uint16_t ultrasonic::ultrasonic_get_cm(void) {
    uint32_t duration = pulse_end - pulse_start;
    duration *= get_timer1_prescaler(); // Convert ticks to microseconds (0.5μs/tick @ prescaler 8)
    return (duration * 343) / 20000; // Speed of sound calculation
}

// Get raw pulse duration
uint32_t ultrasonic::ultrasonic_get_pulse_us(void) {
    return (pulse_end - pulse_start) * get_timer1_prescaler();
}

//IMU Related Code
volatile uint8_t MPU6050:: imu_ready = 0;

MPU6050::MPU6050(){

}
MPU6050::~MPU6050(){

}

void I2C_Init() {
    TWSR = 0x00;
    TWBR = 0x48;
    TWCR = (1 << TWEN);
}

void I2C_Start() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void I2C_Stop() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void I2C_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t I2C_Read_Ack() {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t I2C_Read_Nack() {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

void MPU6050_Init() {
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0);
    I2C_Write(PWR_MGMT_1);
    I2C_Write(0x00);
    I2C_Stop();

    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0);
    I2C_Write(GYRO_CONFIG);
    #if GYRO_SENS == 250
    I2C_Write(0x00);
    #elif GYRO_SENS == 500
    I2C_Write(0x08);
    #elif GYRO_SENS == 1000
    I2C_Write(0x10);
    #elif GYRO_SENS == 2000
    I2C_Write(0x18);
    #else
    I2C_Write(0x00);
    #endif
    I2C_Stop();

    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0);
    I2C_Write(ACCEL_CONFIG);
    #if ACCEL_SENS == 2
    I2C_Write(0x00);
    #elif ACCEL_SENS == 4
    I2C_Write(0x08);
    #elif ACCEL_SENS == 8
    I2C_Write(0x10);
    #elif ACCEL_SENS == 16
    I2C_Write(0x18);
    #else
    I2C_Write(0x00);
    #endif
    I2C_Stop();

    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0);
    I2C_Write(CONFIG);
    I2C_Write(0x03);
    I2C_Stop();
}

volatile uint8_t imu_ready = 0;

void MPU6050::read_accel() {
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0);
    I2C_Write(ACCEL_XOUT_H);
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 1);

    raw.ax = (I2C_Read_Ack() << 8) | I2C_Read_Ack();
    raw.ay = (I2C_Read_Ack() << 8) | I2C_Read_Ack();
    raw.az = (I2C_Read_Ack() << 8) | I2C_Read_Nack();
    I2C_Stop();
}

void MPU6050::read_gyro() {
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 0);
    I2C_Write(GYRO_XOUT_H);
    I2C_Start();
    I2C_Write((MPU6050_ADDR << 1) | 1);

    raw.gx = (I2C_Read_Ack() << 8) | I2C_Read_Ack();
    raw.gy = (I2C_Read_Ack() << 8) | I2C_Read_Ack();
    raw.gz = (I2C_Read_Ack() << 8) | I2C_Read_Nack();
    I2C_Stop();
}

void MPU6050::apply_offset(){
    raw.ax -= ax_offset;
    raw.ay -= ay_offset;
    raw.az -= az_offset;
    raw.gx -= gx_offset;
    raw.gy -= gy_offset;
    raw.gz -= gz_offset;
}

void MPU6050::apply_filter() {
    ema.ax = (EMA_ALPHA * raw.ax) + ((1 - EMA_ALPHA) * ema.ax);
    ema.ay = (EMA_ALPHA * raw.ay) + ((1 - EMA_ALPHA) * ema.ay);
    ema.az = (EMA_ALPHA * raw.az) + ((1 - EMA_ALPHA) * ema.az);
    ema.gx = (EMA_ALPHA * raw.gx) + ((1 - EMA_ALPHA) * ema.gx);
    ema.gy = (EMA_ALPHA * raw.gy) + ((1 - EMA_ALPHA) * ema.gy);
    ema.gz = (EMA_ALPHA * raw.gz) + ((1 - EMA_ALPHA) * ema.gz);
    raw = ema;
}

void MPU6050::GyroCalibration() {
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    for(int i=0; i<500; i++) {
        read_gyro();
        gx_sum += raw.gx;
        gy_sum += raw.gy;
        gz_sum += raw.gz;
        _delay_ms(2);
    }
    
    gx_offset = gx_sum / 500;
    gy_offset = gy_sum / 500;
    gz_offset = gz_sum / 500;
  }
  
  void MPU6050::AccelCalibration() {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    
    for(int i=0; i<500; i++) {
        read_accel();
        ax_sum += raw.ax;
        ay_sum += raw.ay;
        az_sum += raw.az - GRAVITY_LSB;  // Subtract 1g from Z-axis
        _delay_ms(2);
    }
    
    ax_offset = ax_sum / 500;
    ay_offset = ay_sum / 500;
    az_offset = az_sum / 500;
  }

  void MPU6050::ConvertUnits() {
    // Accelerometer conversions
      #if ACCEL_SENS == 2
          converted.ax_g = raw.ax / 16384.0f;
          converted.ay_g = raw.ay / 16384.0f;
          converted.az_g = raw.az / 16384.0f;
          #elif ACCEL_SENS == 4
          converted.ax_g = raw.ax / 8192.0f;
          converted.ay_g = raw.ay / 8192.0f;
          converted.az_g = raw.az / 8192.0f;
          #elif ACCEL_SENS == 8
          converted.ax_g = raw.ax / 4096.0f;
          converted.ay_g = raw.ay / 4096.0f;
          converted.az_g = raw.az / 4096.0f;
          #elif ACCEL_SENS == 16
          converted.ax_g = raw.ax / 2048.0f;
          converted.ay_g = raw.ay / 2048.0f;
          converted.az_g = raw.az / 2048.0f;
          #else
          converted.ax_g = raw.ax / 16384.0f;
          converted.ay_g = raw.ay / 16384.0f;
          converted.az_g = raw.az / 16384.0f;
          #endif
      #if TO_CM == 1
          converted.ax_ms2 = converted.ax_g * 980.665f;
          converted.ay_ms2 = converted.ay_g * 980.665f;
          converted.az_ms2 = converted.az_g * 980.665f;
      #else
        converted.ax_ms2 = converted.ax_g * 9.80665f;
        converted.ay_ms2 = converted.ay_g * 9.80665f;
        converted.az_ms2 = converted.az_g * 9.80665f;
      #endif
  
      // Gyroscope conversions
      #if GYRO_SENS == 250
      converted.gx_dps = raw.gx / 131.0f;
      converted.gy_dps = raw.gy / 131.0f;
      converted.gz_dps = raw.gz / 131.0f;
      #elif GYRO_SENS == 500
      converted.gx_dps = raw.gx / 65.5f;
      converted.gy_dps = raw.gy / 65.5f;
      converted.gz_dps = raw.gz / 65.5f;
      #elif GYRO_SENS == 1000
      converted.gx_dps = raw.gx / 32.8f;
      converted.gy_dps = raw.gy / 32.8f;
      converted.gz_dps = raw.gz / 32.8f;
      #elif GYRO_SENS == 2000
      converted.gx_dps = raw.gx / 16.4f;
      converted.gy_dps = raw.gy / 16.4f;
      converted.gz_dps = raw.gz / 16.4f;
      #else
      converted.gx_dps = raw.gx / 131.0f;
      converted.gy_dps = raw.gy / 131.0f;
      converted.gz_dps = raw.gz / 131.0f;
      #endif
    
    const float deg2rad = M_PI / 180.0f;
    converted.gx_rps = converted.gx_dps * deg2rad;
    converted.gy_rps = converted.gy_dps * deg2rad;
    converted.gz_rps = converted.gz_dps * deg2rad;
  }

  void MPU6050::CalculateAngles() {
    // 1. Convert to physical units
    float ax = converted.ax_g;
    float ay = converted.ay_g;
    float az = converted.az_g;
    float gx = converted.gx_dps;
    float gy = converted.gy_dps;
    float gz = converted.gz_dps;
  
    // 2. Calculate accelerometer angles (safe math)
    float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f/M_PI;
    float accel_roll = atan2f(ay, az) * 180.0f/M_PI;
    
    // 3. Auto-calibrate gyro bias when stationary
    if(fabsf(accel_pitch) < 1.0f && fabsf(accel_roll) < 1.0f) {
        gyro_pitch_bias += gy * DT_actual  * 0.01f;
        gyro_roll_bias += gx * DT_actual  * 0.01f;
    }
    
    // 4. Integrate gyro data (with bias removal)
    angles.pitch = ALPHA * (angles.pitch + (gy - gyro_pitch_bias) * DT_actual ) + 
                  (1 - ALPHA) * accel_pitch;
    angles.roll = ALPHA * (angles.roll + (gx - gyro_roll_bias) * DT_actual ) + 
                 (1 - ALPHA) * accel_roll;
  
    // 5. Integrate yaw from gyro Z
    angles.yaw += converted.gz_dps * DT_actual;
  }

void MPU6050::CalculatePosition(float threshold){
    velocity_x += converted.ax_ms2 * DT_actual;
    // Detect stillness using tighter thresholds
    if (converted.ax_g <= threshold) {
        velocity_x = 0.0f;
        // position_x = 0.0f; // Optional: Reset position if needed
    }
    position_x += velocity_x * DT_actual;
}

//Status LED Related Code
void status_led_init(){
    DDRB |= (1 << PB1);               // OC1A (D9) as output
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM, ICR1 top, prescaler=8
    ICR1 = 39999;                     // 50Hz period (20ms)
}

//Servo
void servo_setup() {
    DDRB |= (1 << PB1);               // OC1A (D9) as output
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM, ICR1 top, prescaler=8
    ICR1 = 39999;                     // 50Hz period (20ms)             // 50Hz period (20ms)
  }
  
  void servo_set_angle(uint8_t angle) {
    // Clamp angle between 0-180
    if (angle > 180) angle = 180;
    else if (angle < 0) angle = 0;
  
    // Convert angle to PWM ticks (1ms-2ms pulse)
    OCR1A = (uint16_t)(1150 + (angle * 18.66) + 0.5); 
    }

  //Fan
  void fans_max_speed_setup() {
    DDRD |= (1 << PD6) | (1 << PD5);  // Set PD6 (OC0A) and PD5 (OC0B) as outputs
    
    // Fast PWM mode, non-inverting output on OC0A and OC0B
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) |  // Non-inverting mode for both channels
             (1 << WGM01)  | (1 << WGM00);    // Fast PWM (TOP = 0xFF)
    
    TCCR0B = (1 << CS00);   // Prescaler = 1 (no prescaling, 16MHz clock)
    
    OCR0A = 255;  // 100% duty cycle for OC0A
    OCR0B = 255;  // 100% duty cycle for OC0B
  }
