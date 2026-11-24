#include "driver/dac_cosine.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/dac_types.h"
#include "hal/i2c_types.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define SAMPLE_COUNT 256
#define MATRIX_ADDR 0x60
#define SAMPLE_INTERVAL_US 250
#define PI 3.14159265358979323846f
#define FFT_N 256

static int samples[SAMPLE_COUNT];
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t matrix_dev;
static float re[FFT_N];
static float im[FFT_N];

uint8_t bars[8];

void square_wave_init(void) {
  ledc_timer_config_t timer_cfg = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .freq_hz = 500,
      .clk_cfg = LEDC_AUTO_CLK,
  };

  ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

  ledc_channel_config_t channel_cfg = {
      .gpio_num = GPIO_NUM_25,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 512,
      .hpoint = 0,
  };

  ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void matrix_write_reg(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};

  ESP_ERROR_CHECK(i2c_master_transmit(matrix_dev, data, sizeof(data), 100));
}

static void matrix_init(void) {
  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = GPIO_NUM_21,
      .scl_io_num = GPIO_NUM_22,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

  // Проверяем, что матрица отвечает на адрес 0x60
  ESP_ERROR_CHECK(i2c_master_probe(i2c_bus, MATRIX_ADDR, 100));

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MATRIX_ADDR,
      .scl_speed_hz = 100000,
  };

  ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &matrix_dev));

  matrix_write_reg(0x00, 0x00); // Обычный режим 8x8
  matrix_write_reg(0x0D, 0x08); // Минимальный ток: 5 мА
  matrix_write_reg(0x0F, 0x00); // Выключить встроенный эквалайзер

  for (int i = 0; i < 8; i++)
    matrix_write_reg(0x01 + i, 0x00);

  matrix_write_reg(0x0C, 0xFF);
}

void prepare_fft(void) {
  float mean = 0.0f;

  for (int i = 0; i < FFT_N; i++) {
    mean += samples[i];
  }

  mean /= FFT_N;

  for (int i = 0; i < FFT_N; i++) {
    re[i] = (float)samples[i] - mean;
    im[i] = 0.0f;
  }
}

unsigned reverse_bits(unsigned value, unsigned bits) {
  unsigned result = 0;
  if (bits == 0) {
    return 0;
  }
  for (unsigned i = 0; i < bits; i++) {
    result = (result << 1) | (value & 1u);
    value >>= 1;
  }
  return result;
}

void bit_reverse_permutation(void) {
  for (unsigned i = 0; i < FFT_N; i++) {
    unsigned j = reverse_bits(i, 8);

    if (i < j) {
      float buffer = re[i];
      re[i] = re[j];
      re[j] = buffer;

      buffer = im[i];
      im[i] = im[j];
      im[j] = buffer;
    }
  }
}

void fft(void) {
  bit_reverse_permutation();

  for (int len = 2; len <= FFT_N; len *= 2) {

    for (int base = 0; base < FFT_N; base += len) {

      for (int k = 0; k < len / 2; k++) {

        int a = base + k;
        int b = a + len / 2;

        float angle = -2.0f * PI * k / len;
        float wr = cosf(angle);
        float wi = sinf(angle);

        float vr = re[b] * wr - im[b] * wi;
        float vi = re[b] * wi + im[b] * wr;

        float ur = re[a];
        float ui = im[a];

        re[a] = ur + vr;
        im[a] = ui + vi;

        re[b] = ur - vr;
        im[b] = ui - vi;
      }
    }
  }
}

static const int band_edges[9] = {1, 4, 8, 12, 16, 24, 40, 64, 128};

void compute_bands(float power[8]) {
  for (int band = 0; band < 8; band++) {
    power[band] = 0.0f;

    for (int k = band_edges[band]; k < band_edges[band + 1]; k++) {

      float p = re[k] * re[k] + im[k] * im[k];

      if (p > power[band])
        power[band] = p;
    }
  }
}

void compute_heights(const float power[8], uint8_t bars[8]) {
  float max_power = 0.0f;

  for (int i = 0; i < 8; i++) {
    if (power[i] > max_power) {
      max_power = power[i];
    }
  }

  if (max_power <= 0.0f) {
    for (int i = 0; i < 8; i++) {
      bars[i] = 0;
    }
    return;
  }

  for (int i = 0; i < 8; i++) {
    float amplitude = sqrtf(power[i] / max_power);

    int height = (int)(amplitude * 8.0f + 0.5f);

    if (height > 8)
      height = 8;

    bars[i] = (uint8_t)height;
  }
}

void print_matrix(const uint8_t bars[8]) {
  for (int row = 8; row >= 1; row--) {
    for (int col = 0; col < 8; col++) {
      putchar(bars[col] >= row ? '#' : '.');
    }
    putchar('\n');
  }
  putchar('\n');
}

static void matrix_show_bars(const uint8_t bars[8]) {
  for (int row = 0; row < 8; row++) {
    uint8_t pixels = 0;

    for (int col = 0; col < 8; col++) {
      if (bars[col] > 7 - row) {
        pixels |= (1u << col);
      }
    }

    matrix_write_reg(0x01 + row, pixels);
  }

  matrix_write_reg(0x0C, 0xFF);
}

void app_main(void) {
  adc_oneshot_unit_handle_t adc;

  adc_oneshot_unit_init_cfg_t cfg = {
      .unit_id = ADC_UNIT_1,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg, &adc));

  adc_oneshot_chan_cfg_t channel_cfg = {
      .bitwidth = ADC_BITWIDTH_12,
      .atten = ADC_ATTEN_DB_12,
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC_CHANNEL_6, &channel_cfg));
  square_wave_init();

  matrix_init();

  const uint8_t test_bars[8] = {1, 2, 3, 4, 5, 6, 7, 8};

  matrix_show_bars(test_bars);
  vTaskDelay(pdMS_TO_TICKS(1000));

  int threshold = 1800;
  for (;;) {
    int first_crossing = -1;
    int last_crossing = -1;
    int64_t min_dt = 1000000;
    int64_t max_dt = 0;
    int max_adc = 0;
    int min_adc = 100000;
    int64_t previous = 0;
    int raw = 0;
    int crossings = 0;
    int64_t start = esp_timer_get_time();
    for (int i = 0; i < SAMPLE_COUNT; i++) {
      int64_t target = start + (int64_t)i * SAMPLE_INTERVAL_US;
      while (esp_timer_get_time() < target) {
      }
      int64_t actual = esp_timer_get_time();

      if (i > 0) {
        int64_t dt = actual - previous;

        if (dt < min_dt)
          min_dt = dt;
        if (dt > max_dt)
          max_dt = dt;
      }

      previous = actual;
      ESP_ERROR_CHECK(adc_oneshot_read(adc, ADC_CHANNEL_6, &raw));
      samples[i] = raw;
      if (raw > max_adc) {
        max_adc = raw;
      }
      if (raw < min_adc) {
        min_adc = raw;
      }
    }

    prepare_fft();
    fft();

    float power[8];
    uint8_t bars[8];

    compute_bands(power);
    compute_heights(power, bars);
    print_matrix(bars);
    matrix_show_bars(bars);
    int peak_bin = 1;
    float peak_power = 0.0f;

    for (int k = 1; k < FFT_N / 2; k++) {
      float power = re[k] * re[k] + im[k] * im[k];

      if (power > peak_power) {
        peak_power = power;
        peak_bin = k;
      }
    }

    float fs = 1000000.0f / SAMPLE_INTERVAL_US;
    float frequency = peak_bin * fs / FFT_N;

    printf("FFT peak: %.2f Hz, bin=%d\n", frequency, peak_bin);
    square_wave_init();

    printf("re[0]=%.2f im[0]=%.2f\n", re[0], im[0]);
    threshold = min_adc + (max_adc - min_adc) / 2;

    for (int i = 1; i < SAMPLE_COUNT; i++) {
      if (samples[i - 1] < threshold && samples[i] >= threshold) {

        if (first_crossing == -1)
          first_crossing = i;

        last_crossing = i;
        crossings++;
      }
    }

    if (crossings >= 2) {
      float period_samples =
          (float)(last_crossing - first_crossing) / (crossings - 1);

      float frequency = 1000000.0f / (period_samples * SAMPLE_INTERVAL_US);

      printf("Frequency: %.2f Hz\n", frequency);
    }

    for (int i = 0; i < 32; i++) {
      printf("%d\n", samples[i]);
    }

    printf("min_dt=%lld us, max_dt=%lld us\n", (long long)min_dt,
           (long long)max_dt);
    printf("Crossings: %d\n", crossings);
    printf("ADC: min=%d max=%d threshold=%d\n", min_adc, max_adc, threshold);
  }
}
