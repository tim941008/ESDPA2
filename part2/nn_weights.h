// Quantized INT8 weights and fixed-point params
#ifndef NN_WEIGHTS_H
#define NN_WEIGHTS_H

#include <stdint.h>

#define NN_INPUT_LEN 150
#define NN_INPUT_CH 6

// Activation scales (float) for reference
#define SCALE_INPUT 5.0141622683e-02f
#define SCALE_CONV1_RELU 7.5148693848e-02f
#define SCALE_POOL1 7.5148693848e-02f
#define SCALE_CONV2_RELU 1.3900290709e-01f
#define SCALE_GAP 4.2190570306e-02f
#define SCALE_DENSE1_RELU 1.1482807945e-01f
#define SCALE_LOGITS 2.0173662102e-01f

// Preprocess: raw → physical units → standardize → quantize to int8
// accel: raw / 16384.0 → g;  gyro: raw / 131.0 → deg/s
// standardize: (physical - MEAN) / STD
// quantize: round(standardized / SCALE_INPUT), clamped to [-128, 127]
const float NN_MEAN[NN_INPUT_CH] = {
  -0.039071f,  // ax [g]
   0.110681f,  // ay [g]
   0.895139f,  // az [g]
  -1.229286f,  // gx [deg/s]
   8.567729f,  // gy [deg/s]
   0.238436f,  // gz [deg/s]
};
const float NN_STD[NN_INPUT_CH] = {
  0.490854f,  // ax [g]
  0.296745f,  // ay [g]
  0.405138f,  // az [g]
  70.066f,    // gx [deg/s]
  38.428f,    // gy [deg/s]
  96.658f,    // gz [deg/s]
};




#endif // NN_WEIGHTS_H