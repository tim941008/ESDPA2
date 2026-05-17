#ifndef NN_OPS_H
#define NN_OPS_H

#include <stdint.h>
#include "nn_weights.h"

// TODO 2.1: Implement preprocess_input() -- convert raw int16 IMU samples to quantized int8 NN inputs.
//
// The process for each sample (i) and channel (c):
//   1. PHYSICAL: convert raw ADC counts to physical units
//        accel (c=0,1,2): physical = raw / 16384.0f   -- units: g
//        gyro  (c=3,4,5): physical = raw / 131.0f     -- units: deg/s
//   2. STANDARDIZE: zero-mean, unit-variance normalization
//        standardized = (physical - NN_MEAN[c]) / NN_STD[c]
//   3. QUANTIZE: map standardized float to int8
//        q = round(standardized / SCALE_INPUT)
//        clamp q to [-128, 127], then store as int8.

inline void preprocess_input(
    const int16_t raw[NN_INPUT_LEN][NN_INPUT_CH],
    int8_t out[NN_INPUT_LEN][NN_INPUT_CH]
) {
  for (int i = 0; i < NN_INPUT_LEN; i++) {
    for (int c = 0; c < NN_INPUT_CH; c++) {
      // Your implementation
      // 1. PHYSICAL: 將原始 ADC 刻度轉換為實體單位
      float physical;

      if (c < 3) {
        // 加速度計 (c=0: ax, c=1: ay, c=2: az) -> 除以 16384.0f，單位: g
        physical = (float)raw[i][c] / 16384.0f;
      } else {
        // 陀螺儀 (c=3: gx, c=4: gy, c=5: gz) -> 除以 131.0f，單位: deg/s
        physical = (float)raw[i][c] / 131.0f;
      }

      // 2. STANDARDIZE: 零均值、單位方差正規化 (減去平均數，除以標準差)
      float standardized = (physical - NN_MEAN[c]) / NN_STD[c];

      // 3. QUANTIZE: 將標準化後的浮點數映射到 int8
      float q_float = standardized / SCALE_INPUT;
      
      // 實作四捨五入 (經由加上/減去 0.5f 後強轉整數來達成)
      int32_t q = (q_float >= 0.0f) ? (int32_t)(q_float + 0.5f) : (int32_t)(q_float - 0.5f);

      // CLAMP: 確保數值嚴格限制在 int8_t 的合法範圍 [-128, 127] 內，防止溢位
      if (q > 127) {
        q = 127;
      } else if (q < -128) {
        q = -128;
      }

      // 將最終結果安全地存入輸出的 int8_t 矩陣
      out[i][c] = (int8_t)q;
    
    }
  }
}

#endif // NN_OPS_H
