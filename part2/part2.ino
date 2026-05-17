#include "I2C_GPIO.h"
#include "nn_ops.h"

#define I2C_use_GPIO 1

static const uint8_t MPU_ADDR = 0x68;

#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B

#define LED_PIN       6
#define BTN_PIN       12

#define DEBOUNCE_MS   30

#define SAMPLE_RATE_HZ      100
#define SAMPLE_INTERVAL_US  10000UL   // 100 Hz = 10 ms
#define NUM_SAMPLES         150       // 100 Hz * 1.5 sec
#define PRINT_STRIDE        5         // print every 5th sample → 30 rows displayed

bool isRecording = false;

unsigned long lastDebounceMs = 0;
int lastButtonReading = HIGH;
int buttonState = HIGH;

int16_t raw_samples[NN_INPUT_LEN][NN_INPUT_CH];
int8_t nn_input[NN_INPUT_LEN][NN_INPUT_CH];

void MPU6050_wakeup();
bool readImu6_raw(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz);
void handleButton();
void recordOneSample();

// Part1: I2C Communication and Data Collection
// Todo 1.1: Wake up MPU6050 by writing to its power management register.
// Requirements:
// 1) Must use your GPIO-based I2C functions (NO Wire.h).
// 2) Must perform a valid I2C register write transaction:
//    - START
//    - SLA+W
//    - register address
//    - data
//    - STOP
// Notes:
// - MPU6050 PWR_MGMT_1 register address: 0x6B
// - Wakeup value: 0x00 (clear sleep bit)
void MPU6050_wakeup() {
  I2C_start();
  if (I2C_write_byte((MPU_ADDR << 1) | 0x00) &&
      I2C_write_byte(PWR_MGMT_1) &&
      I2C_write_byte(0x00)) {
  }
  else {
    Serial.println("# ERROR: Failed to wake up MPU6050");
  }
  I2C_stop();
}

/*******************************************************************************/
// Todo 1.2: Read raw accelerometer data from MPU6050.
//
// This function performs a complete I2C register read transaction using
// the GPIO-based I2C implementation.
//
// Requirements:
// 1. Use a register pointer write to select the starting register.
// 2. Use a repeated START condition to switch to read mode.
// 3. Read six bytes of data (XH, XL, YH, YL, ZH, ZL).
// 4. Combine high and low bytes into 16-bit signed integers.
//
// Transaction flow:
//
//   START
//   SLA+W
//   register address (ACCEL_XOUT_H)
//   REPEATED START
//   SLA+R
//   read ax_H, ax_L     (ACK, ACK)   buf[0..1]
//   read ay_H, ay_L     (ACK, ACK)   buf[2..3]
//   read az_H, az_L     (ACK, ACK)   buf[4..5]
//   read temp_H, temp_L (ACK, ACK)   buf[6..7]  (discarded)
//   read gx_H, gx_L     (ACK, ACK)   buf[8..9]
//   read gy_H, gy_L     (ACK, ACK)   buf[10..11]
//   read gz_H           (ACK)        buf[12]
//   read gz_L           (NACK)       buf[13]
//   STOP
//
// The function returns true if the transaction succeeds.
bool readImu6_raw(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz) {
  I2C_start();
  if (!I2C_write_byte((MPU_ADDR << 1) | 0x00) ||
      !I2C_write_byte(ACCEL_XOUT_H)) {
    I2C_stop();
    return false;
  }

  I2C_repeated_start();
  if (!I2C_write_byte((MPU_ADDR << 1) | 0x01)) {
    I2C_stop();
    return false;
  }

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) {
    buf[i] = I2C_read_byte(i < 13);
  }
  I2C_stop();

  *ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  *ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
  *az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
  *gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
  *gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
  *gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  MPU6050_wakeup();
  analogWrite(LED_PIN, 0);

  Serial.println("IMU preprocess viewer ready.");
  Serial.println("Push btn to record and compare raw vs preprocessed.");
}

void loop() {
  handleButton();
}

// Print val right-justified in a field of given width
void printW(int32_t val, int width) {
  int len = 1;
  int32_t tmp = val;
  if (tmp < 0) { len++; tmp = -tmp; }
  while (tmp >= 10) { len++; tmp /= 10; }
  for (int i = len; i < width; i++) Serial.print(' ');
  Serial.print(val);
}

void handleButton() {
  int reading = digitalRead(BTN_PIN);

  if (reading != lastButtonReading) {
    lastDebounceMs = millis();
  }

  if ((millis() - lastDebounceMs) > DEBOUNCE_MS) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW && !isRecording) {
        recordOneSample();
      }
    }
  }

  lastButtonReading = reading;
}

void recordOneSample() {
  isRecording = true;

  Serial.println("# GET_READY");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);

  Serial.println("# RECORDING...");
  unsigned long nextSampleUs = micros();

  for (int i = 0; i < NUM_SAMPLES; i++) {
    while ((long)(micros() - nextSampleUs) < 0);

    // ****************************************************************************
    // TODO 2.2: Declare six int16_t variables: ax, ay, az, gx, gy, gz. 
    // (The call to readImu6_raw() and error handling below depend on these 
    // variables being declared here.)
    int16_t ax, ay, az, gx, gy, gz;

    // ****************************************************************************
    bool ok = readImu6_raw(&ax, &ay, &az, &gx, &gy, &gz);

    if (!ok) {
      Serial.print("# ERROR at timestep ");
      Serial.println(i);
      digitalWrite(LED_PIN, LOW);
      isRecording = false;
      return;
    }

    // ****************************************************************************
    // TODO 2.3: Copy the six axis values into raw_samples[i][].
    //           Channel mapping: 0→ax, 1→ay, 2→az (accelerometer, ±2 g range)
    //                            3→gx, 4→gy, 5→gz (gyroscope, ±250 °/s range)
    //           raw_samples holds int16_t ADC counts; preprocess_input() called
    //           later in runInference() converts them to physical units and int8.
    raw_samples[i][0] = ax;
    raw_samples[i][1] = ay;
    raw_samples[i][2] = az;
    raw_samples[i][3] = gx;
    raw_samples[i][4] = gy;
    raw_samples[i][5] = gz;
    
    // *****************************************************************************

    nextSampleUs += SAMPLE_INTERVAL_US;
  }

  // ************************************************
  // TODO 2.4: Preprocess raw_samples into nn_input using preprocess_input() implemented in nn_ops.h.
  preprocess_input(raw_samples, nn_input);


  // ************************************************

  // Two-row-per-sample layout, columns aligned, every PRINT_STRIDE-th sample
  Serial.println("# --- RAW vs PREPROCESSED ---");
  Serial.println("           |     ax     ay     az     gx     gy     gz");
  Serial.println("-----------|----------------------------------------------");

  for (int i = 0; i < NUM_SAMPLES; i += PRINT_STRIDE) {
    Serial.print("t="); printW(i, 3); Serial.print("  RAW |");
    for (int c = 0; c < NN_INPUT_CH; c++) printW(raw_samples[i][c], 7);
    Serial.println();

    Serial.print("       PRE |");
    for (int c = 0; c < NN_INPUT_CH; c++) printW((int)nn_input[i][c], 7);
    Serial.println();

    Serial.println();
  }

  Serial.println("# --- END ---");
  digitalWrite(LED_PIN, LOW);
  isRecording = false;
}
