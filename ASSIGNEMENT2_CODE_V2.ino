
/* ENGR 290 sample-TWI-based demonstration, ATmega328P @ 16 MHz.
 * Article-style iterative hardware-offset calibration: capture once, hard-code register offsets.
 * Board: controller v3.11, Arduino Nano/ATmega328P with known working bootloader.
 * Wiring: MPU-6050 on connector P7 (P7/1=SCL, P7/2=SDA, P7/3=3.3V, P7/4=GND);
 * servo on connector P9 (P9/1=PB1/OC1A signal, P9/2=+5V, P9/3=GND).
 * Basic Arduino peripheral helper functions are intentionally not used.
 * NOTE: IMU-only yaw and double-integrated position drift over time.
 * NOTE: BATTERY mode needs board-specific voltage calibration; see constants below.
 * NOTE: Original TWI function signatures, flow, and error codes are preserved.
 * WARNING: v3.11 LED L/PB5 and battery channel must be confirmed on real PCB.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <math.h>
#include <stdlib.h>
#include "TWI_290.h"
extern "C" volatile uint8_t TWI_byte;

// Article pp. 7-11: meanSensors + calibrate_ iterative hardware offset writes.
// First upload: 1 (find and print offset register values). Second upload: 0 (apply them).
//#define CALIBRATION_CAPTURE 1

// These numbers are DEVICE HARDWARE OFFSET REGISTER values, not software biases.
// Never copy the example numbers from the article: measure your own module.
#define OFFSET_AX_RAW -275
#define OFFSET_AY_RAW -1214
#define OFFSET_AZ_RAW 755
#define OFFSET_GX_RAW -52
#define OFFSET_GY_RAW -63
#define OFFSET_GZ_RAW 30

#define CALIBRATION_BUFFER_SIZE 1000
#define ACCEL_DEADZONE 8
#define GYRO_DEADZONE 1
#define CALIBRATION_MAX_ITERATIONS 100  // safety stop, not in article

#define MPU_ADDR_0 0x68
#define MPU_ADDR_1 0x69
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75

// Board-specific settings to verify against the physical v3.11 PCB.
#define BATTERY_ADC_CHANNEL 6   // newer pin sheet says ADC6; older says ADC7
#define BATTERY_LOW_ADC 10      // controller notes suggest ADC < 10 -> depleted
#define BATTERY_FULL_ADC 150    // EXAMPLE brightness scale; calibrate on hardware
// A conversion from ADC readings to ACTUAL volts is NOT documented consistently.
// Until calibrated, the terminal reports ADC counts, not invented battery volts.

static volatile uint32_t milliseconds = 0;
ISR(TIMER0_COMPA_vect) { ++milliseconds; }

static uint32_t now_ms(void) {
  uint32_t v;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = milliseconds; }
  return v;
}

static void timer0_init_1ms(void) {
  // 16 MHz / 64 / (249+1) = 1000 ticks/s => one interrupt each ms.
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);
}

static void uart_init(void) {
  // 38400 bps at 16MHz (UBRR=25): shorter output stalls than 9600.
  // IMPORTANT: set the Serial Monitor to 38400 baud.
  UCSR0A = 0;
  UBRR0H = 0;
  UBRR0L = 25;
  UCSR0B = (1 << TXEN0) | (1 << RXEN0); // RX is needed for r/s commands.
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
static void uart_char(char c) {
  while (!(UCSR0A & (1 << UDRE0))) {}
  UDR0 = (uint8_t)c;
}
static void uart_text(const char *s) { while (*s) uart_char(*s++); }
static void uart_u32(uint32_t v) {
  char buf[11]; uint8_t n=0;
  do { buf[n++] = (char)('0' + v % 10); v /= 10; } while (v);
  while (n) uart_char(buf[--n]);
}
static void uart_i32(int32_t value) {
  if (value < 0) { uart_char('-'); uart_u32((uint32_t)(-value)); }
  else uart_u32((uint32_t)value);
}
static void uart_fixed(float value, uint8_t decimals) {
  if (value < 0) { uart_char('-'); value = -value; }
  uint32_t scale = decimals == 3 ? 1000UL : decimals == 2 ? 100UL : 10UL;
  uint32_t scaled = (uint32_t)(value * scale + 0.5f);
  uart_u32(scaled / scale);
  uart_char('.');
  uint32_t rem = scaled % scale;
  for (uint32_t d = scale / 10; d > 0; d /= 10) uart_char((char)('0' + (rem / d) % 10));
}
static void uart_endline(void) { uart_text("\r\n"); }

static void servo_init(void) {
  // Servo connected to physical connector P9: signal PB1/OC1A (not Arduino digital pin 9).
  // Timer1 phase-correct PWM mode 10.
  // TOP=2500, /64, up-down cycle = 20ms (course programming hints).
  DDRB |= (1 << PB1);
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << CS11) | (1 << CS10);
  ICR1 = 2500;
  OCR1A = 188;                  // ~1.5ms, nominal center
}
static void servo_yaw(float yaw) {
  if (yaw > 80.0f) yaw = 80.0f; //angle for servo motor
  if (yaw < -80.0f) yaw = -80.0f;
  // Nominal 1.0 ms at -90 deg, 1.5 at 0, 2.0 at +90.
  OCR1A = (uint16_t)(187.5f + yaw * (75.0f / 90.0f) + 0.5f);
}

static void leds_init(void) {
  // Onboard L generally PB5 (Arduino Nano LED); active LOW.
  DDRB |= (1 << PB5) | (1 << PB3);
  PORTB |= (1 << PB5); // LED L off
  // D3 = PB3 / OC2A per older v3.10 sheet; verify on actual v3.11 board.
  // Non-inverting output gives HIGH duty OCR2A/255. LED is active LOW,
  // so duty must DECREASE as requested brightness increases.
  TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << CS22);        // /64, ~976 Hz PWM
  OCR2A = 255;                // HIGH -> LED off
}
static void led_l(bool on) {
    if (on) PORTB |= (1 << PB5); 
    else PORTB &= (uint8_t)~(1 << PB5);
}
static void led_d3_percent(float percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  OCR2A = (uint8_t)(255.0f - percent * 255.0f / 100.0f + 0.5f);
}

static void adc_init(void) {
  // WARNING: use existing EXTERNAL AREF. REFS1:REFS0 both zero.
  ADMUX = (1 << ADLAR) | BATTERY_ADC_CHANNEL; // left-adjust 8-bit readings
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRB = 0;
}
static uint8_t adc_battery(void) {
  ADMUX = (1 << ADLAR) | BATTERY_ADC_CHANNEL;
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC)) {}
  return ADCH;
}

static uint8_t imu_addr = MPU_ADDR_0;
static bool imu_present = false;
// Offsets are programmed into the IMU registers; do not also subtract in software.
static float yaw_deg=0, roll_deg=0, pitch_deg=0;
static float ax_g=0, ay_g=0, az_g=0;
static float x_velocity_mps=0, x_position_m=0;
static uint32_t last_motion_ms=0;

// Experiment 4/5: X displacement in the IMU's BODY frame, not world X.
// This deliberately keeps the expected ~cos(45deg) component in Experiment 5.
enum DistanceState { DIST_CALIBRATING, DIST_RUNNING, DIST_FROZEN };
static DistanceState distance_state = DIST_CALIBRATING;
#define GRAVITY_CAL_SAMPLES 150U // about 1.5 s at 100 Hz, held still.
static uint16_t grav_cal_count = 0;
static float grav_sum_ax=0, grav_sum_ay=0, grav_sum_az=0;
static float grav_sum_wx=0, grav_sum_wy=0, grav_sum_wz=0;
static float base_ax_g=0, base_ay_g=0, base_az_g=1;
static float base_wx_dps=0, base_wy_dps=0, base_wz_dps=0;
// Unit vector of local gravity in the IMU's axes, integrated using gyro.
static float grav_x=0, grav_y=0, grav_z=1;
static float grav_initial_x=0;
static float x_linear_accel_mps2=0;
static float experiment_elapsed_s=0;

static void begin_distance_trial(void) {
  distance_state = DIST_CALIBRATING;
  grav_cal_count = 0;
  grav_sum_ax=grav_sum_ay=grav_sum_az=0;
  grav_sum_wx=grav_sum_wy=grav_sum_wz=0;
  x_velocity_mps=x_position_m=0;
  x_linear_accel_mps2=experiment_elapsed_s=0;
  uart_text("CAL: hold still at 0cm ~1.5s");
  uart_endline();
}

static void end_distance_trial(void) {
  if (distance_state != DIST_RUNNING) return;
  float uncorrected_m = x_position_m;
  float final_velocity_error = x_velocity_mps;
  // With both endpoints at rest, velocity error is mainly integrated bias.
  // Approximate constant-bias correction: delta_x = -0.5*v_end*T.
  x_position_m = uncorrected_m - 0.5f*final_velocity_error*experiment_elapsed_s;
  x_velocity_mps = 0;
  distance_state = DIST_FROZEN;
  uart_text("End raw="); uart_fixed(uncorrected_m*100.0f,2);
  uart_text("cm final="); uart_fixed(x_position_m*100.0f,2);
  uart_text("cm vend="); uart_fixed(final_velocity_error,3);
  uart_text("m/s T="); uart_fixed(experiment_elapsed_s,2);
  uart_text(" s"); uart_endline();
  uart_text("r: new trial"); uart_endline();
}

static void check_distance_commands(void) {
  while (UCSR0A & (1 << RXC0)) {
    char c = (char)UDR0;
    if (c=='r' || c=='R') begin_distance_trial();
    else if (c=='s' || c=='S') end_distance_trial();
  }
}

static int16_t signed16(uint8_t hi, uint8_t lo) {
  uint16_t bits = ((uint16_t)hi << 8) | lo;
  // Convert the 16-bit two's-complement pattern without depending on
  // implementation-defined unsigned-to-signed overflow conversion.
  return bits <= 32767U ? (int16_t)bits : (int16_t)((int32_t)bits - 65536L);
}
static bool imu_raw(int16_t *a, int16_t *g) {
  int16_t raw_words[7];
  if (Read_Reg_N(imu_addr, REG_ACCEL_XOUT_H, 14, raw_words)) return false;
  uint8_t *b = (uint8_t*)raw_words;
  for (uint8_t i=0; i<3; ++i) a[i] = signed16(b[2*i], b[2*i+1]);
  for (uint8_t i=0; i<3; ++i) g[i] = signed16(b[8+2*i], b[9+2*i]);
  return true;
}
static bool imu_initialize(void) {
  if (!Read_Reg(MPU_ADDR_0, REG_WHO_AM_I) && TWI_byte==0x68) imu_addr=MPU_ADDR_0;
  else if (!Read_Reg(MPU_ADDR_1, REG_WHO_AM_I) && TWI_byte==0x68) imu_addr=MPU_ADDR_1;
  else return false;

  // Wake device, use gyro X PLL clock, LPF = ~44Hz, sample rate 100Hz.
  // Gyro +/-250 dps (131 LSB/dps), accel +/-2g (16384 LSB/g).
  if (Write_Reg(imu_addr, REG_PWR_MGMT_1, 0x01)) return false;
  if (Write_Reg(imu_addr, REG_CONFIG, 0x03)) return false;
  if (Write_Reg(imu_addr, REG_SMPLRT_DIV, 9)) return false;
  if (Write_Reg(imu_addr, REG_GYRO_CONFIG, 0x00)) return false;
  if (Write_Reg(imu_addr, REG_ACCEL_CONFIG, 0x00)) return false;
  return true;
}
// Based on "The MPU6050 Explained", pages 7-11: meanSensors_() and calibrate_().
// The article uses a separate Jeff Rowberg class, so only I/O calls are adapted
// to the ORIGINAL professor-provided TWI_290.c Read_Reg/Write_Reg functions.
// REQUIREMENT: sensor motionless, flat, with +Z up and measurement range +/-2g.
// Offsets are INTERNAL MPU-6050 register settings, NOT software subtractions.
static const uint8_t accel_offset_regs[3] = {0x06, 0x08, 0x0A};
static const uint8_t gyro_offset_regs[3] = {0x13, 0x15, 0x17};
static int32_t mean_accel[3] = {0,0,0};
static int32_t mean_gyro[3] = {0,0,0};

// The accelerometer low offset-register bit 0 is temperature compensation.
// Retain the existing bit when writing a new accelerometer offset.
static bool set_sensor_offset(uint8_t first_reg, int16_t value, bool accelerometer) {
  uint8_t low = (uint8_t)value;
  if (accelerometer) {
    if (Read_Reg(imu_addr, (uint8_t)(first_reg+1))) return false;
    low = (uint8_t)((low & 0xFE) | (TWI_byte & 0x01));
  }
  if (Write_Reg(imu_addr, first_reg, (uint8_t)((uint16_t)value >> 8))) return false;
  if (Write_Reg(imu_addr, (uint8_t)(first_reg+1), low)) return false;
  return true;
}
static bool set_all_offsets(const int16_t a_off[3], const int16_t g_off[3]) {
  for (uint8_t i=0; i<3; i++) {
    if (!set_sensor_offset(accel_offset_regs[i], a_off[i], true)) return false;
    if (!set_sensor_offset(gyro_offset_regs[i], g_off[i], false)) return false;
  }
  return true;
}

// Article meanSensors_: discard first 100, average the following 1000.
static bool mean_sensors(void) {
  int32_t ta[3]={0,0,0}, tg[3]={0,0,0};
  // Original PDF uses delay(2). We use our hardware timer instead.
  for (uint16_t i=0; i<CALIBRATION_BUFFER_SIZE+100; i++) {
    int16_t a[3], g[3];
    if (!imu_raw(a,g)) return false;
    if (i>=100) {
      for (uint8_t k=0;k<3;k++) { ta[k]+=a[k]; tg[k]+=g[k]; }
    }
    uint32_t start=now_ms();
    while ((uint32_t)(now_ms()-start)<2UL) {}
  }
  for (uint8_t k=0;k<3;k++) {
    mean_accel[k]=ta[k]/CALIBRATION_BUFFER_SIZE;
    mean_gyro[k]=tg[k]/CALIBRATION_BUFFER_SIZE;
  }
  return true;
}

static void print_article_offsets(const int16_t a[3], const int16_t g[3]) {
  uart_text("#define OFFSET_AX_RAW "); uart_i32(a[0]); uart_endline();
  uart_text("#define OFFSET_AY_RAW "); uart_i32(a[1]); uart_endline();
  uart_text("#define OFFSET_AZ_RAW "); uart_i32(a[2]); uart_endline();
  uart_text("#define OFFSET_GX_RAW "); uart_i32(g[0]); uart_endline();
  uart_text("#define OFFSET_GY_RAW "); uart_i32(g[1]); uart_endline();
  uart_text("#define OFFSET_GZ_RAW "); uart_i32(g[2]); uart_endline();
}
static void show_means(void) {
  uart_text("Mean accelerometer readings X/Y/Z: ");
  for (uint8_t i=0;i<3;i++) { uart_i32(mean_accel[i]); uart_char(' '); }
  uart_endline();
  uart_text("Mean gyrometer readings X/Y/Z: ");
  for (uint8_t i=0;i<3;i++) { uart_i32(mean_gyro[i]); uart_char(' '); }
  uart_endline();
}

// Article calibrate_(): initial offset guesses, then iterative adjustments.
static bool calibrate_article_method(void) {
  const int16_t zero[3]={0,0,0};
  int16_t ao[3]={0,0,0}, go[3]={0,0,0};
  uart_text("Article method: zeroing all six offsets..."); uart_endline();
  if (!set_all_offsets(zero,zero)) return false;
  if (!mean_sensors()) return false;
  show_means();

  // These six initial guesses are copied from the PDF's calibrate_().
  ao[0]=(int16_t)(-mean_accel[0]/8);
  ao[1]=(int16_t)(-mean_accel[1]/8);
  ao[2]=(int16_t)((16384L-mean_accel[2])/8);
  go[0]=(int16_t)(-mean_gyro[0]/4);
  go[1]=(int16_t)(-mean_gyro[1]/4);
  go[2]=(int16_t)(-mean_gyro[2]/4);

  for (uint16_t iteration=0;iteration<CALIBRATION_MAX_ITERATIONS;iteration++) {
    if (!set_all_offsets(ao,go)) return false;
    if (!mean_sensors()) return false;
    uart_text("Iteration "); uart_u32((uint32_t)(iteration+1)); uart_endline();
    show_means();
    uint8_t ready=0;
    // Exactly the adjustment formulas and deadzones from article calibrate_().
    if (labs(mean_accel[0])<=ACCEL_DEADZONE) ready++;
    else ao[0]=(int16_t)(ao[0]-mean_accel[0]/ACCEL_DEADZONE);
    if (labs(mean_accel[1])<=ACCEL_DEADZONE) ready++;
    else ao[1]=(int16_t)(ao[1]-mean_accel[1]/ACCEL_DEADZONE);
    if (labs(16384L-mean_accel[2])<=ACCEL_DEADZONE) ready++;
    else ao[2]=(int16_t)(ao[2]+(16384L-mean_accel[2])/ACCEL_DEADZONE);
    for (uint8_t i=0;i<3;i++) {
      if (labs(mean_gyro[i])<=GYRO_DEADZONE) ready++;
      else go[i]=(int16_t)(go[i]-mean_gyro[i]/(GYRO_DEADZONE+1));
    }
    uart_text("Ready axes: "); uart_u32(ready); uart_text(" / 6"); uart_endline();
    if (ready==6) {
      uart_text("CALIBRATION SUCCESS. Copy these hardware register values:"); uart_endline();
      print_article_offsets(ao,go);
      uart_text("Set CALIBRATION_CAPTURE=0 and upload again."); uart_endline();
      return true;
    }
  }
  uart_text("Calibration stopped without converging; do NOT hard-code trial values."); uart_endline();
  return false;
}

static bool apply_hardcoded_article_offsets(void) {
  const int16_t a[3]={(int16_t)OFFSET_AX_RAW,(int16_t)OFFSET_AY_RAW,(int16_t)OFFSET_AZ_RAW};
  const int16_t g[3]={(int16_t)OFFSET_GX_RAW,(int16_t)OFFSET_GY_RAW,(int16_t)OFFSET_GZ_RAW};
  return set_all_offsets(a,g);
}

static float abs_f(float x) {return x < 0 ? -x : x;}
static float clamp_f(float x,float minv,float maxv) {
  return x<minv ? minv : x>maxv ? maxv : x;
}
static void motion_update(float dt) {
  int16_t a[3], g[3];
  if (!imu_raw(a,g)) { imu_present=false; return; }
  ax_g = (float)a[0]/16384.0f;
  ay_g = (float)a[1]/16384.0f;
  az_g = (float)a[2]/16384.0f;
  float wx=(float)g[0]/131.0f;
  float wy=(float)g[1]/131.0f;
  float wz=(float)g[2]/131.0f;

  const float rad_to_deg=57.2957795f, deg_to_rad=0.01745329252f;
  // Simple tilt-compensated roll/pitch (valid for moderate acceleration).
  float acc_roll=atan2f(ay_g,az_g)*rad_to_deg;
  float acc_pitch=atan2f(-ax_g,sqrtf(ay_g*ay_g+az_g*az_g))*rad_to_deg;
  roll_deg=0.98f*(roll_deg+wx*dt)+0.02f*acc_roll;
  pitch_deg=0.98f*(pitch_deg+wy*dt)+0.02f*acc_pitch;
  yaw_deg += wz*dt; // RELATIVE yaw only; magnetometer-free yaw drifts.

  servo_yaw(yaw_deg);
  led_l(abs_f(yaw_deg)>85.0f);
  led_d3_percent(clamp_f((abs_f(ax_g)-0.11f)*100.0f,0,100));

  // ----- Experiment 4/5: gravity-compensated BODY-X displacement -----
  // First establish the X gravity + offset measured at the start, at rest.
  if (distance_state == DIST_CALIBRATING) {
    grav_sum_ax += ax_g; grav_sum_ay += ay_g; grav_sum_az += az_g;
    grav_sum_wx += wx; grav_sum_wy += wy; grav_sum_wz += wz;
    grav_cal_count++;
    if (grav_cal_count >= GRAVITY_CAL_SAMPLES) {
      const float inv_n=1.0f/(float)GRAVITY_CAL_SAMPLES;
      base_ax_g=grav_sum_ax*inv_n;
      base_ay_g=grav_sum_ay*inv_n;
      base_az_g=grav_sum_az*inv_n;
      base_wx_dps=grav_sum_wx*inv_n;
      base_wy_dps=grav_sum_wy*inv_n;
      base_wz_dps=grav_sum_wz*inv_n;
      float norm=sqrtf(base_ax_g*base_ax_g+base_ay_g*base_ay_g+base_az_g*base_az_g);
      if (norm<0.75f || norm>1.25f) {
        uart_text("CAL failed. Hold still; type r");
        uart_endline();
        distance_state=DIST_FROZEN;
        return;
      }
      grav_x=base_ax_g/norm; grav_y=base_ay_g/norm; grav_z=base_az_g/norm;
      grav_initial_x=grav_x;
      x_velocity_mps=x_position_m=0;
      x_linear_accel_mps2=experiment_elapsed_s=0;
      distance_state=DIST_RUNNING;
      uart_text("READY: move +X to 30cm; stop; s");
      uart_endline();
      uart_text("ax0="); uart_fixed(base_ax_g,3);
      uart_text("g gx0="); uart_fixed(grav_initial_x,3);
      uart_text("g"); uart_endline();
    }
    return;
  }
  if (distance_state != DIST_RUNNING) return;  // freeze result after 's'.

  // Update the estimated gravity direction FROM THE GYROSCOPE, not from
  // the accelerometer's instantaneous pitch, which mistakes forward
  // acceleration for tilt. For body-frame gravity g: dg/dt = -omega X g.
  // Subtract residual stationary gyro biases measured during calibration.
  float wx_r=(wx-base_wx_dps)*deg_to_rad;
  float wy_r=(wy-base_wy_dps)*deg_to_rad;
  float wz_r=(wz-base_wz_dps)*deg_to_rad;
  float dgx=-(wy_r*grav_z-wz_r*grav_y)*dt;
  float dgy=-(wz_r*grav_x-wx_r*grav_z)*dt;
  float dgz=-(wx_r*grav_y-wy_r*grav_x)*dt;
  grav_x+=dgx; grav_y+=dgy; grav_z+=dgz;
  float grav_norm=sqrtf(grav_x*grav_x+grav_y*grav_y+grav_z*grav_z);
  if (grav_norm > 0.5f) { // Keep unit length despite Euler integration drift.
    grav_x/=grav_norm; grav_y/=grav_norm; grav_z/=grav_norm;
  }

  // Remove constant start-of-trial offset/gravity AND subsequent tilt gravity.
  // ax_g - base_ax_g removes the stationary component at 0 cm.
  // grav_x - grav_initial_x removes the change in X gravity due to rotation.
  float linear_ax_g=(ax_g-base_ax_g)-(grav_x-grav_initial_x);
  x_linear_accel_mps2=9.80665f*linear_ax_g;

  // Integrate ONLY while a trial is active. We do not infer stationarity
  // from |acceleration|=1g: motion at constant speed can also be 1g.
  x_position_m += x_velocity_mps*dt+0.5f*x_linear_accel_mps2*dt*dt;
  x_velocity_mps += x_linear_accel_mps2*dt;
  experiment_elapsed_s += dt;
}

static void report_motion(void) {
  uart_text("Roll="); uart_fixed(roll_deg,2);
  uart_text(" Pitch="); uart_fixed(pitch_deg,2);
  uart_text(" Yaw="); uart_fixed(yaw_deg,2); uart_endline();
  uart_text("Ax="); uart_fixed(ax_g,3);
  uart_text("g Ay="); uart_fixed(ay_g,3);
  uart_text("g Az="); uart_fixed(az_g,3);
  uart_text("g"); uart_endline();
}
static void report_distance(void) {
  if (distance_state == DIST_CALIBRATING) {
    uart_text("X: calibrating"); uart_endline();
    return;
  }
  uart_text("X=");
  uart_fixed(x_position_m*100.0f,2);
  uart_text("cm ax="); uart_fixed(x_linear_accel_mps2,3);
  uart_text("m/s2 vx="); uart_fixed(x_velocity_mps,3);
  uart_text(" m/s");
  if (distance_state == DIST_FROZEN) uart_text(" [FROZEN]");
  uart_endline();
}
static void battery_mode_tick(void) {
  uint8_t reading=adc_battery();
  uart_text("Battery ADC8 = "); uart_u32(reading);
  uart_text(" (voltage conversion requires board calibration)"); uart_endline();
  if (reading < BATTERY_LOW_ADC) {
    led_d3_percent(100); // flash and turn OFF
    for (volatile uint32_t i=0;i<40000UL;++i) {}
    led_d3_percent(0);
    uart_text("Battery depleted"); uart_endline();
    while (!(UCSR0A & (1 << UDRE0))) {} // wait until transmit holding register is empty
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
  } else {
    led_d3_percent(100.0f*clamp_f((float)reading/BATTERY_FULL_ADC,0,1));
  }
}

void setup(void) {
  cli();
  uart_init();
  timer0_init_1ms();
  servo_init();
  leds_init();
  adc_init();
  TWI_init(); // Hardware TWI uses PC4=SDA and PC5=SCL, routed to connector P7.
  sei();
  uart_text("ENGR290 MPU6050 register demo starting"); uart_endline();
  imu_present=imu_initialize();
  if (imu_present) {
#if CALIBRATION_CAPTURE
    if (!calibrate_article_method()) uart_text("Calibration failed or did not converge.\r\n");
    // Hold here so servo and LED behavior cannot move during calibration testing.
    while (1) {}
#else
    if (!apply_hardcoded_article_offsets()) {
      uart_text("Could not apply hardware offsets"); uart_endline();
      imu_present=false;
    } else {
      uart_text("Using hard-coded MPU hardware offsets"); uart_endline();
      begin_distance_trial();
      uart_text("38400 baud; r=reset, s=stop");
      uart_endline();
    }
#endif
  } else {
    uart_text("No MPU detected: entering battery-monitor display mode"); uart_endline();
  }
  last_motion_ms=now_ms();
}

void loop(void) {
  static uint32_t last_orientation=0, last_distance=0, last_battery=0;
  check_distance_commands();
  uint32_t t=now_ms();
  if (imu_present) {
    if ((uint32_t)(t-last_motion_ms)>=10UL) {
      float dt=(float)(t-last_motion_ms)/1000.0f;
      last_motion_ms=t;
      motion_update(dt);
    }
    if ((uint32_t)(t-last_orientation)>=1500UL) {
      last_orientation=t;
      report_motion();
    }
    if ((uint32_t)(t-last_distance)>=1000UL) {
      last_distance=t;
      report_distance();
    }
  } else if ((uint32_t)(t-last_battery)>=1000UL) {
    last_battery=t;
    battery_mode_tick();
  }
}
