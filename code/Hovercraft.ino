#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "hovercraft.h"

// Global Variables
ultrasonic us_left;
ultrasonic us_right;

volatile uint32_t current_ticks = 0;

MPU6050 imu;

volatile uint8_t ADC_sample = 0;
volatile uint16_t ADC_acc = 0;
volatile uint8_t ADC_value = 0;
volatile bool stop = 0;

uart uart_control;

//-------------------------Ultrasonic-------------------------------

//US 1
#define TRIGGER_PIN1 3  // Sector B 
#define ECHO_PIN1    2  // Sector D

//US 2
#define TRIGGER_PIN2 5  // Sector B 
#define ECHO_PIN2    3  // Sector D

//US 3
#define TRIGGER_PIN3 5  // Sector B 
#define ECHO_PIN3    0  // Sector B

void ultrasonics_init(){
  EICRA = (1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10); // Rising/Falling edge trigger
  EIMSK = (1 << INT0) | (1 << INT1);

  DDRB |= (1 << TRIGGER_PIN2);     //Trigger PB5 as output
  DDRD &= ~(1 << ECHO_PIN2);    //Echo PD3 as input

  DDRB |= (1 << TRIGGER_PIN1);     //Trigger PB3 as output, sets also the PWM_LED_PIN to output
  DDRD &= ~(1 << ECHO_PIN1);    //Echo PD2 as input

  DDRB |= (1 << TRIGGER_PIN3);  // PB5 as trigger
  DDRB &= ~(1 << ECHO_PIN3);     // PB0 (ICP1) as input

}

void ultrasonics_trigger(void) {
  PORTB |= (1 << TRIGGER_PIN1);
  PORTB |= (1 << TRIGGER_PIN2);
  _delay_us(10);
  PORTB &= ~(1 << TRIGGER_PIN1);
  PORTB |= (1 << TRIGGER_PIN2);
}

ISR(INT0_vect) {
  if (PIND & (1 << ECHO_PIN1)) {
      us_left.pulse_start =  TCNT1;
      EICRA &= ~(1 << ISC00); 
  } else {
      us_left.pulse_end  =  TCNT1;
      EICRA |= (1 << ISC00);
      us_left.measurement_ready = 1;
  }
}

ISR(INT1_vect) {
  if (PIND & (1 << ECHO_PIN2)) { 
      us_right.pulse_start =  TCNT1;
      EICRA &= ~(1 << ISC10);
  } else {
      us_right.pulse_end =  TCNT1;
      EICRA |= (1 << ISC10);
      us_right.measurement_ready = 1;
  }
}


//-------------------------IR-------------------------------
#define ADC_sample_max 50
#define V_REF 5.0
#define V_IN_NADC ((V_REF * 1000.0) / 1023.0) // For 10-bit ADC

void infrared_init(){
  ADMUX = (1 << REFS0) | (1 << MUX1);
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | 
            (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | 
            (1 << ADPS0);
}

ISR(ADC_vect) {
  if (ADC_sample < ADC_sample_max) {
      ADC_acc += ADCL | (ADCH << 8);
      ADC_sample++;
  } else {
      ADC_value = ADC_acc / ADC_sample_max;
      ADC_sample = 0;
      ADC_acc = 0;
  }
}

//-----------------------Servo-----------------------------

void Servo_Init() {
  DDRB |= (1 << PB1); // Set PB1 (OC1A) as output
}


//-----------------------Functions--------------------------
const int stabilize_const = 5;
const int activate_turn_sequence = 30;
int k=1; //turning speed constant
int pos;
uint32_t turn_start_time = 0;

// PID Constants (Tune these values experimentally)
#define KP 3.5    // Proportional gain Faster response to error
#define KI 0.05   // Integral gain (prevents windup)
#define KD 1.2    // Derivative gain stabilize overshoot

// Control variables
float prev_error = 0;
float integral = 0;

// Timing
const uint16_t control_interval = 50; // PID update interval (ms)

enum States {
  STATE_CRUISE,      // Centered navigation
  STATE_TURN,         // Execute 90° turn
  STATE_STOP          // Stop when bar detected
};
States current_state = STATE_CRUISE;

// Thresholds (tune experimentally)
const int GAP_THRESHOLD = 26;   // cm (sudden increase in one sensor)
const int WALL_THRESHOLD = 23;  // cm (minimum wall distance)

float left_prev = 0, right_prev = 0;
const int SENSOR_DELTA_THRESHOLD = 26; // cm (sudden change = gap)

#define BAR_HEIGHT       35    // Target vertical height (cm)
#define HEIGHT_TOLERANCE 7     // ±7cm tolerance
#define US_FRONT_ANGLE   20    // Sensor tilt angle (degrees)
#define DEG_TO_RAD       (M_PI / 180.0)
#define DEBOUNCE_TIME    200   // Minimum sustained detection time (ms)
#define MIN_ADC          999 //TUNE IT =====================================
#define MAX_ADC          999

/*// Calculate valid distance range
const float min_distance = (BAR_HEIGHT - HEIGHT_TOLERANCE) / sin(US_FRONT_ANGLE * DEG_TO_RAD);
const float max_distance = (BAR_HEIGHT + HEIGHT_TOLERANCE) / sin(US_FRONT_ANGLE * DEG_TO_RAD);
*/

// Debounce variables
bool bar_detected = false;
uint32_t bar_detect_start = 0;

int main(void) {
  //Init Zone
  clock1_setup();
  clock2_setup();
  UART_Init(MYUBRR);
  I2C_Init();
  ultrasonics_init();
  infrared_init();
  MPU6050_Init();
  servo_setup();   // Initialize servo PWM
  fans_max_speed_setup();

  //Calibration Zone
  imu.GyroCalibration();
  imu.AccelCalibration();

  sei();
  while(1) {
      static uint32_t last_pid_time = 0;
      static float initial_yaw = 0;
      // Get measurement for all sensors

      //US

      // Trigger US alternately every 30ms
      if (hovercraft_clock::timer2_ms - us_left.last_time > 30) {
        PORTB |= (1 << TRIGGER_PIN1);
        _delay_us(10);
        PORTB &= ~(1 << TRIGGER_PIN1);
        us_left.last_time = hovercraft_clock::timer2_ms;
      }
      if (hovercraft_clock::timer2_ms - us_right.last_time > 30) {
        PORTB |= (1 << TRIGGER_PIN2);
        _delay_us(10);
        PORTB &= ~(1 << TRIGGER_PIN2);
        us_right.last_time = hovercraft_clock::timer2_ms;
      }
     

      // Process US 1
      if (us_left.measurement_ready) {
          uint32_t duration = us_left.pulse_end - us_left.pulse_start;
          duration *= 0.5;
          us_left.distance_cm = (duration * 343) / 20000;
          us_left.measurement_ready = 0;
      }

      // Process US 2
      if (us_right.measurement_ready) {
          uint32_t duration2 = us_right.pulse_end - us_right.pulse_start;
          duration2 *= 0.5;
          us_right.distance_cm = (duration2 * 343) / 20000;
          us_right.measurement_ready = 0;
      }


      //IMU

      static int32_t last_imu_call = 0;

      if (hovercraft_clock::timer2_ms - last_imu_call >= 20) { //50hz
        imu.imu_ready = 1;
        last_imu_call = hovercraft_clock::timer2_ms;
      }

      if(imu.imu_ready){
        imu.read_accel();
        imu.read_gyro();

        // Apply calibration
        imu.apply_offset();

        // Apply filter
        imu.apply_filter();
        imu.filtered = imu.raw;

        imu.ConvertUnits();
        imu.CalculateAngles();

        imu.CalculatePosition(0.02f);
        imu.imu_ready = 0; // Clear the flag once processed.
      }

      //IR


      //Hovercraft Operations
      
      //Test Zone  (Need to comment this or Real Application)
      //servo_set_angle(45);

      //Real Application

    //----- Debounced Bar Detection -----
if (current_state != STATE_STOP) {
      bool current_detection =  (ADC_value >= MIN_ADC) && 
                                (ADC_value <= MAX_ADC);

      if (current_detection) {
          if (!bar_detected) {
              // First detection - start timer
              bar_detect_start = hovercraft_clock::timer2_ms;
              bar_detected = true;
              UART_Print("BAR DETECTED - DEBOUNCING...\n");
          } else {
              // Sustained detection - check duration
              if ((hovercraft_clock::timer2_ms - bar_detect_start) >= DEBOUNCE_TIME) {
                  current_state = STATE_STOP;
                  UART_Print("BAR CONFIRMED - SHUTTING DOWN\n");
              }
          }
      } else {
          // Reset detection if condition breaks
          bar_detected = false;
          bar_detect_start = 0;

      }
  }

      switch (current_state) {
        //---------------------------------------
        case STATE_CRUISE: {
          // Detect sudden gap (opening) on one side
          float left_delta = us_left.distance_cm - left_prev;
          float right_delta = us_right.distance_cm - right_prev;

          if ((left_delta > SENSOR_DELTA_THRESHOLD || us_left.distance_cm > GAP_THRESHOLD) && 
    us_right.distance_cm < WALL_THRESHOLD) {
            // Left gap detected, turn left
            UART_Print("Left gap detected, turn left");
            initial_yaw = imu.angles.yaw;
            current_state = STATE_TURN;
            servo_set_angle(135);
            turn_start_time = hovercraft_clock::timer2_ms;
          } 
          else if ((right_delta > SENSOR_DELTA_THRESHOLD || us_right.distance_cm > GAP_THRESHOLD) && 
          us_left.distance_cm < WALL_THRESHOLD) {
            // Right gap detected, turn right
            UART_Print("Right gap detected, turn right");
            initial_yaw = imu.angles.yaw;
            current_state = STATE_TURN;
            servo_set_angle(45);
            turn_start_time = hovercraft_clock::timer2_ms;
          } 
          else {
            // Normal PID centering (from previous code)
            if (hovercraft_clock::timer2_ms - last_pid_time >= control_interval) {
              // Calculate position error (difference between sensors)
              float error = us_right.distance_cm - us_left.distance_cm;
              
              // PID terms
              float P = KP * error;
              integral += KI * error;
              float D = KD * (error - prev_error);
              
              // Calculate steering angle (-45° to +45°)
              float output = P + integral + D;
              output = constrain(output, -45.0, 45.0);
              
              // Convert to servo angle (90° neutral ± output)
              uint8_t steer_angle = 90 + output;
              
              // Apply steering
              servo_set_angle(steer_angle);
              
              // Update variables
              prev_error = error;
              last_pid_time = hovercraft_clock::timer2_ms;
      
              // Debugging output
              /*UART_Print("L:");
              UART_Print_Int(us_left.distance_cm);
              UART_Print("cm R:");
              UART_Print_Int(us_right.distance_cm);
              UART_Print(" Error:");
              UART_Print_Float(error);
              UART_Print(" Steer:");
              UART_Print_Int(steer_angle);
              UART_Print("\n");*/
              
            }
          }
          break;
        }
        
        //---------------------------------------
        case STATE_TURN: {
          if (hovercraft_clock::timer2_ms - turn_start_time > 4000) { // 4s timeout
              current_state = STATE_CRUISE;
          }
          // Use IMU yaw to complete 90° turn
          float yaw_change = fabs(imu.angles.yaw - initial_yaw);
          
          if (yaw_change >= 85.0 && yaw_change <= 95.0) { // ±5° tolerance
            // Turn complete
            servo_set_angle(90); // Return to neutral
            current_state = STATE_CRUISE;
          }
          break;
        }
        case STATE_STOP: {
          // Full system shutdown
          servo_set_angle(90);       // Neutral steering
          PORTD &= ~((1 << PD5) | (1 << PD6)); // Stop fans
          UART_Print("BAR DETECTED: SYSTEM HALTED\n");
          while(1); // Permanent stop
          break;
        }
      }
    
      // Update sensor history
      left_prev = us_left.distance_cm;
      right_prev = us_right.distance_cm;

      //End of Real Application
      
      //UART FOR TESTING -- Should be commented for final

      if (hovercraft_clock::timer2_ms - uart_control.last_time >= 1000) {
        UART_Print("Pitch: ");
        UART_Print_Float(imu.angles.pitch);
        UART_Print("°, Roll: ");
        UART_Print_Float(imu.angles.roll);
        UART_Print("°, Yaw: ");
        UART_Print_Float(imu.angles.yaw);
        UART_Print("°\n");
        UART_Print("Ax: ");
        UART_Print_Float(imu.converted.ax_g);
        UART_Print("g, Ay: ");
        UART_Print_Float(imu.converted.ay_g);
        UART_Print("g, Az: ");
        UART_Print_Float(imu.converted.az_g);
        UART_Print("g\n");
        UART_Print("Pos X: ");
        UART_Print_Float(imu.position_x);
        UART_Print("cm\n");

        UART_Print("ADC Value: ");
        UART_Print_Int(ADC_value);
        UART_Print("\n");

        UART_Print("Distance Right: ");
        UART_Print_Int(us_right.distance_cm);
        UART_Print("cm\n");


        UART_Print("Distance Left: ");
        UART_Print_Int(us_left.distance_cm);
        UART_Print("cm\n\n");
      
        uart_control.last_time = hovercraft_clock::timer2_ms;
      }   
  }
}