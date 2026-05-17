#include "I2C_GPIO.h"
#define I2C_use_GPIO 1

static const uint8_t MPU_ADDR = 0x68;

#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B

#define LED_PIN       6
#define BTN_PIN       12

#define DEBOUNCE_MS   30

#define SAMPLE_RATE_HZ      100
#define SAMPLE_INTERVAL_US  10000UL   // 100 Hz = 10 ms
#define RECORD_SECONDS      1.5
#define NUM_SAMPLES         150       // 100 Hz * 1.5 sec

bool isRecording = false;

unsigned long lastDebounceMs = 0;
int lastButtonReading = HIGH;
int buttonState = HIGH;

uint32_t sampleId = 0;


void MPU6050_wakeup();
bool readImu6_raw(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz);
void handleButton();
void recordOneSample();
void printLabelField(const char *label, int width, bool last);
void printIntField(long value, int width, bool last);

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
  uint8_t b[14];

  I2C_start();
  if (!I2C_write_byte(MPU_ADDR << 1) ||
      !I2C_write_byte(ACCEL_XOUT_H)) {
    I2C_stop();
    return false;
  }

  I2C_repeated_start();
  if (!I2C_write_byte((MPU_ADDR << 1) | 1)) {
    I2C_stop();
    return false;
  }

  for (uint8_t i = 0; i < 13; i++) {
    b[i] = I2C_read_byte(true);
  }
  b[13] = I2C_read_byte(false);
  I2C_stop();

  *ax = (int16_t)((b[0] << 8) | b[1]);
  *ay = (int16_t)((b[2] << 8) | b[3]);
  *az = (int16_t)((b[4] << 8) | b[5]);// 跳過溫度資料
  *gx = (int16_t)((b[8] << 8) | b[9]);
  *gy = (int16_t)((b[10] << 8) | b[11]);
  *gz = (int16_t)((b[12] << 8) | b[13]);

  return true;
}



void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  MPU6050_wakeup();
  analogWrite(LED_PIN, 0);

  Serial.println("Dataset collector ready.");
  Serial.println("Push btn to start recording.");
}

void loop() {
  handleButton();
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

  Serial.println("# START");
  printLabelField("ts", 3, false);
  printLabelField("t_us", 8, false);
  printLabelField("ax", 6, false);
  printLabelField("ay", 6, false);
  printLabelField("az", 6, false);
  printLabelField("gx", 6, false);
  printLabelField("gy", 6, false);
  printLabelField("gz", 6, true);
  Serial.println();

  unsigned long nextSampleUs = micros();

  for (int i = 0; i < NUM_SAMPLES; i++) {
    while ((long)(micros() - nextSampleUs) < 0) {
      // wait until next 10 ms slot
    }

    unsigned long tUs = micros();
    int16_t s[6];

    bool ok = readImu6_raw(&s[0], &s[1], &s[2], &s[3], &s[4], &s[5]);

    if (!ok) {
      Serial.print("# ERROR at timestep ");
      Serial.println(i);
      digitalWrite(LED_PIN, LOW);
      isRecording = false;
      return;
    }


    printIntField(i, 3, false);
    printIntField((long)tUs, 8, false);
    for (int c = 0; c < 6; c++) {
      printIntField(s[c], 6, c == 5);
    }
    Serial.println();

    nextSampleUs += SAMPLE_INTERVAL_US;
  }


  Serial.println("# END");
  digitalWrite(LED_PIN, LOW);
  sampleId++;
  isRecording = false;
}

void printLabelField(const char *label, int width, bool last) {
  int len = (int)strlen(label);
  for (int i = len; i < width; i++) {
    Serial.print(' ');
  }
  Serial.print(label);
  if (!last) {
    Serial.print('|');
  }
}

void printIntField(long value, int width, bool last) {
  char buf[16];
  ltoa(value, buf, 10);
  int len = (int)strlen(buf);
  for (int i = len; i < width; i++) {
    Serial.print(' ');
  }
  Serial.print(buf);
  if (!last) {
    Serial.print('|');
  }
}
