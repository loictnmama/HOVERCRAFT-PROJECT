/* ENGR 290 sample-TWI-based demonstration, ATmega328P @ 16 MHz.
 * Article-style iterative hardware-offset calibration: capture once, hard-code register offsets.
 * Board: controller v3.11, Arduino Nano/ATmega328P with known working bootloader.
 * Wiring: MPU-6050 on connector P7 (P7/1=SCL, P7/2=SDA, P7/3=3.3V, P7/4=GND);
 * servo on connector P9 (P9/1=PB1/OC1A signal, P9/2=+5V, P9/3=GND).
 * Basic Arduino peripheral helper functions are intentionally not used.
 * NOTE: IMU-only yaw and double-integrated position drift over time.
 * Experiment 6: ADC7 battery warning and AVR power-down sleep.
 * NOTE: Original TWI function signatures, flow, and error codes are preserved.
 * ADC7 battery input: board v3.11 pin-assignment sheet, V(ADC7)=max(Vbatt-12V,0).
 * ADC AREF value below MUST be checked against the real board for accurate volts.
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

// ENGR 290 v3.11 uCU pin assignment: ADC7 = battery-monitor input.
// Board condition: V(ADC7) = max(Vbattery - 12 V, 0).
// ADC6 is for an external sensor, NOT the battery on this revision.
#define BATTERY_ADC_CHANNEL 7

// IMPORTANT: 3.30 V is only an initial example for the EXTERNAL AREF.
// Measure AREF against controller GND with a meter or calibrate against the
// SIGLENT's actual voltage readback. Edit the number if it is different.
// NEVER switch the ADC to AVcc or internal reference on this PCB.
#define BATTERY_AREF_VOLTS 3.30f

// At/under 12 V, ADC7 is approximately zero by HARDWARE design.
// 1 ADC count allows a little conversion noise/quantization tolerance.
#define BATTERY_DEPLETED_MAX_ADC8 1
// 4 consecutive 100-ms low measurements to reject single noisy samples.
#define BATTERY_LOW_CONSECUTIVE 4
#define BATTERY_SAMPLE_PERIOD_MS 100UL
#define BATTERY_REPORT_PERIOD_MS 1000UL
// D3 battery brightness: 0% around 12 V and 100% around 15 V.
#define BATTERY_BRIGHTNESS_FULL_VOLTS 15.0f

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
  // 9600 bps, 8N1, at 16 MHz (normal speed: UBRR=103).
  UCSR0A = 0;
  UBRR0H = 0;
  UBRR0L = 103;
  UCSR0B = (1 << TXEN0);
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
  // 8-bit conversion is the method used in the course example.
  // Average eight readings to reduce random noise. All samples use ADC7.
  ADMUX = (1 << ADLAR) | BATTERY_ADC_CHANNEL;  // REFS1:REFS0 remain 0
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {}
    sum += ADCH;
  }
  return (uint8_t)((sum + 4U) / 8U);
}

static uint8_t imu_addr = MPU_ADDR_0;
static bool imu_present = false;
// Offsets are programmed into the IMU registers; do not also subtract in software.
static float yaw_deg=0, roll_deg=0, pitch_deg=0;
static float ax_g=0, ay_g=0, az_g=0;
static float x_velocity_mps=0, x_position_m=0;
static uint32_t last_motion_ms=0;

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

  // Demonstration-only inertial x displacement. IMU-only double integration
  // cannot remain accurate for general moves: bias/drift/tilt cause large errors.
  float r=roll_deg*deg_to_rad, p=pitch_deg*deg_to_rad;
  float cy=cosf(yaw_deg*deg_to_rad),sy=sinf(yaw_deg*deg_to_rad);
  float cr=cosf(r), sr=sinf(r), cp=cosf(p), sp=sinf(p);
  // R(body->world) first row; world-vertical gravity cancels automatically.
  float x_acc_g=(cp*cy)*ax_g+(sr*sp*cy-cr*sy)*ay_g+(cr*sp*cy+sr*sy)*az_g;
  float acceleration=x_acc_g*9.80665f;
  // Simple stationary gating limits, but doesn't eliminate, drift.
  float gnorm=sqrtf(ax_g*ax_g+ay_g*ay_g+az_g*az_g);
  if (abs_f(gnorm-1.0f)<0.035f && abs_f(wx)+abs_f(wy)+abs_f(wz)<2.0f) {
    x_velocity_mps=0;
  } else {
    x_position_m += x_velocity_mps*dt + 0.5f*acceleration*dt*dt;
    x_velocity_mps += acceleration*dt;
  }
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
  uart_text("Estimated X displacement = ");
  uart_fixed(x_position_m*100.0f,2);
  uart_text(" cm (drifts)"); uart_endline();
}
// Experiment 6 power management, using ONLY direct AVR register access.
// WARNING: the Nano must still have power (e.g. USB) long enough to print
// the message and execute sleep after the J1 voltage falls below 12 V.
static uint8_t low_battery_samples = 0;
static uint32_t last_battery_sample_ms = 0;
static uint32_t last_battery_report_ms = 0;

static void wait_milliseconds(uint16_t duration_ms) {
  uint32_t start = now_ms();
  while ((uint32_t)(now_ms() - start) < duration_ms) {}
}

static float battery_volts_from_adc8(uint8_t adc8) {
  // The ADC pin sees battery volts MINUS 12 V (v3.11 analog hardware).
  // AREF MUST be measured for accurate terminal output.
  // An ADC8 reading of zero means Vbattery <= ~12 V, not exactly 12 V.
  return 12.0f + ((float)adc8 * BATTERY_AREF_VOLTS / 256.0f);
}

static void battery_display(uint8_t raw8) {
  uart_text("Battery ADC7 = "); uart_u32(raw8);
  uart_text(" /255; ");
  if (raw8 <= BATTERY_DEPLETED_MAX_ADC8) {
    uart_text("Vbattery <= ~12.0 V (hardware cutoff)");
  } else {
    uart_text("Vbattery ~= ");
    uart_fixed(battery_volts_from_adc8(raw8), 2);
    uart_text(" V (check external AREF calibration)");
  }
  uart_endline();
}

static void battery_warning_then_sleep(void) {
  // On this board D3 uses active-LOW PWM: 100%=bright, 0%=off.
  // Three visible warning flashes, ending OFF.
  for (uint8_t i = 0; i < 3; ++i) {
    led_d3_percent(100.0f);
    wait_milliseconds(180);
    led_d3_percent(0.0f);
    wait_milliseconds(180);
  }

  // FINISH serial transmission before disabling UART and clock.
  // TXC0 is write-one-to-clear: clear the stale complete flag first.
  UCSR0A |= (1 << TXC0);
  uart_text("Battery depleted"); uart_endline();
  uart_text("Entering deep sleep (reset required to resume)"); uart_endline();
  while (!(UCSR0A & (1 << TXC0))) {}

  cli();
  // Disconnect output compare peripherals and leave all motors/fans OFF.
  TCCR1A = 0; TCCR1B = 0;       // stop servo PWM (P9 / PB1)
  DDRB |= (1 << PB1) | (1 << PB3) | (1 << PB5);
  PORTB &= (uint8_t)~((1 << PB1) | (1 << PB5));  // servo LOW; L LED OFF
  PORTB |= (1 << PB3);         // D3 OFF (active-low)
  DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
  PORTD &= (uint8_t)~((1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7));

  TCCR2A = 0; TCCR2B = 0;       // stop D3 PWM
  TCCR0A = 0; TCCR0B = 0;       // stop 1-ms tick
  TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; // no timer interrupts
  EIMSK = 0; PCICR = 0;        // disable external and pin-change interrupts
  ADCSRA &= (uint8_t)~(1 << ADEN); // shut off ADC
  ACSR |= (1 << ACD);          // turn off analog comparator
  TWCR = 0;                   // shut off I2C peripheral
  UCSR0B = 0;                 // turn off UART after TXC0

  // Reduce remaining AVR peripheral power consumption.
  PRR = (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM1) |
        (1 << PRTIM0) | (1 << PRSPI) | (1 << PRADC) |
        (1 << PRUSART0);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  // AVR continues to have Vcc, but the CPU clock and timed tasks stop.
  sei();
  sleep_cpu();
  for (;;) { sleep_cpu(); }   // remain asleep if an unexpected wake occurs
}

static void battery_monitor_tick(uint32_t now) {
  if ((uint32_t)(now - last_battery_sample_ms) < BATTERY_SAMPLE_PERIOD_MS) return;
  last_battery_sample_ms = now;

  uint8_t raw8 = adc_battery();
  // Monitor even while IMU is present; the battery is a safety requirement.
  if (raw8 <= BATTERY_DEPLETED_MAX_ADC8) {
    if (low_battery_samples < BATTERY_LOW_CONSECUTIVE) ++low_battery_samples;
  } else {
    low_battery_samples = 0;
  }

  // Requirement: D3 tracks BATTERY voltage only when the IMU is disconnected.
  // Otherwise the existing IMU acceleration/brightness code still controls it.
  if (!imu_present && low_battery_samples < BATTERY_LOW_CONSECUTIVE) {
    float brightness = (battery_volts_from_adc8(raw8) - 12.0f) *
                       (100.0f / (BATTERY_BRIGHTNESS_FULL_VOLTS - 12.0f));
    led_d3_percent(clamp_f(brightness, 0.0f, 100.0f));
  }

  if ((uint32_t)(now - last_battery_report_ms) >= BATTERY_REPORT_PERIOD_MS) {
    last_battery_report_ms = now;
    battery_display(raw8);
  }

  if (low_battery_samples >= BATTERY_LOW_CONSECUTIVE) {
    battery_display(raw8); // last reading before sleep
    battery_warning_then_sleep();
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
  uart_text("ENGR290 MPU6050 + battery ADC7 deep-sleep demo"); uart_endline();
  uart_text("Battery: ADC7=Vbattery-12V; external AREF must be checked."); uart_endline();
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
    } else { uart_text("Using hard-coded MPU hardware offsets"); uart_endline(); }
#endif
  } else {
    uart_text("No MPU detected: entering battery-monitor display mode"); uart_endline();
  }
  last_motion_ms=now_ms();
}

void loop(void) {
  static uint32_t last_orientation=0, last_distance=0;
  uint32_t t=now_ms();
  // Always poll battery, including when the IMU is connected.
  battery_monitor_tick(t);
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
  }
}
