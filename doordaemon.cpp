#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "vl53lx_class.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for
// information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

#ifndef GATEWAY_IP
#error "Gateway IP undefined"
#endif
#ifndef GATEWAY_PORT
#error "Gateway port undefined"
#endif
#ifndef GATEWAY_SSID
#error "Gateway SSID undefined"
#endif
#ifndef GATEWAY_PWD
#error "Gateway password undefined"
#endif

#define BUF_SIZE 2048
#define POLL_TIME_S 5

#define MSG_MAX 64
#define MIN_CLOSE_TIME_S 2
#define DISTANCE_THRESHOLD_MM 750

#define FAIL(fmt, ...)          \
  do {                          \
    printf(fmt, ##__VA_ARGS__); \
  } while (1)

static void tcp_client_err(void* arg, err_t err) {
  printf("Error: %d\n", err);
}

static err_t tcp_client_sent(void* arg, struct tcp_pcb* tpcb, u16_t len) {
  printf("Sent %d bytes\n", len);
  return ERR_OK;
}

static err_t tcp_client_connected(void* arg, struct tcp_pcb* tpcb, err_t err) {
  static char message[MSG_MAX];

  int message_len = snprintf(message, sizeof(message), "ALERT %d", (int)arg);

  err = ERR_OK;
  if ((err = tcp_write(tpcb, message, message_len, 0)) != ERR_OK) {
    printf("Error writing data: %d\n", err);
    return err;
  }
  if ((err = tcp_output(tpcb)) != ERR_OK) {
    printf("Error sending data: %d\n", err);
  }

  return err;
}

static err_t tcp_client_poll(void* arg, struct tcp_pcb* tpcb) {
  printf("Closing\n");
  return tcp_close(tpcb);
}

void alert_remote_host(int16_t distance_mm) {
  ip_addr_t remote_addr;
  ip4addr_aton(GATEWAY_IP, &remote_addr);

  tcp_pcb* pcb = tcp_new_ip_type(IP_GET_TYPE(&remote_addr));
  if (pcb == NULL) {
    printf("Could not create PCB\n");
    return;
  }
  tcp_arg(pcb, (void*)distance_mm);
  tcp_err(pcb, tcp_client_err);
  tcp_sent(pcb, tcp_client_sent);
  tcp_poll(pcb, tcp_client_poll, POLL_TIME_S / 2);

  cyw43_arch_lwip_begin();
  err_t err =
      tcp_connect(pcb, &remote_addr, GATEWAY_PORT, tcp_client_connected);
  cyw43_arch_lwip_end();

  if (err != ERR_OK) {
    printf("Failed to connect\n");
  }
}

void core1_entry() {
  cyw43_arch_enable_sta_mode();

  if (0 != cyw43_arch_wifi_connect_async(GATEWAY_SSID,
                                         GATEWAY_PWD,
                                         CYW43_AUTH_WPA2_AES_PSK)) {
    FAIL("Wi-Fi failed to connect\n");
  }
  printf("Wi-Fi connecting...\n");

  multicore_fifo_push_blocking(0xDEADBEEF);

  uint32_t data;
  int status = 0;
  while (true) {
    uint32_t data = multicore_fifo_pop_blocking();

    status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

    if (CYW43_LINK_UP != status) {
      printf("Link down\n");
      continue;
    }

    alert_remote_host(data);
  }
}

int main() {
  stdio_init_all();

  if (cyw43_arch_init_with_country(CYW43_COUNTRY_GERMANY)) {
    FAIL("Wi-Fi init failed\n");
  }

  multicore_launch_core1(&core1_entry);
  multicore_fifo_pop_blocking();

  // Initialise the Wi-Fi chip

  // I2C Initialisation. Using it at 100Khz.
  i2c_init(I2C_PORT, 100 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
  // For more examples of I2C use see
  // https://github.com/raspberrypi/pico-examples/tree/master/i2c

  VL53LX toffifee(I2C_PORT, -1);

  VL53LX_Error err = toffifee.InitSensor(VL53LX_DEFAULT_DEVICE_ADDRESS);
  if (err != VL53LX_ERROR_NONE)
    FAIL("Error initialising sensor: %d\n", err);

  err = toffifee.VL53LX_SetDistanceMode(VL53LX_DISTANCEMODE_SHORT);
  if (err != VL53LX_ERROR_NONE)
    printf("Error setting distance mode: %d\n", err);

  err = toffifee.VL53LX_StartMeasurement();
  if (err != VL53LX_ERROR_NONE)
    FAIL("Error starting measurement: %d\n", err);

  absolute_time_t close_interval_start;
  bool somebody_is_close = false;

  while (true) {
    uint8_t data_ready = 0;
    int status = 0;

    do {
      status = toffifee.VL53LX_GetMeasurementDataReady(&data_ready);
    } while (data_ready == 0);

    if (0 != status) {
      continue;
    }

    VL53LX_MultiRangingData_t tof_data;
    memset(&tof_data, 0, sizeof(tof_data));

    status = toffifee.VL53LX_GetMultiRangingData(&tof_data);

    if (0 != status) {
      continue;
    }

    int16_t closest_mm = INT16_MAX;
    for (int i = 0; i < tof_data.NumberOfObjectsFound; i++) {
      VL53LX_TargetRangeData_t range_data = tof_data.RangeData[i];
      if (0 != range_data.RangeStatus) {
        continue;
      }
      int16_t range = range_data.RangeMilliMeter;
      if (range < closest_mm) {
        closest_mm = range;
      }
    }
    printf("\r%d mm", closest_mm);

    if (closest_mm < DISTANCE_THRESHOLD_MM) {
      if (!somebody_is_close) {
        close_interval_start = get_absolute_time();
      }
      somebody_is_close = true;
    } else {
      somebody_is_close = false;
    }

    if (somebody_is_close) {
      uint64_t us_since_close = to_us_since_boot(get_absolute_time()) -
                                to_us_since_boot(close_interval_start);

      if (MIN_CLOSE_TIME_S < us_since_close / 1000000) {
        multicore_fifo_push_blocking(closest_mm);
        somebody_is_close = false;
      }
    }

    toffifee.VL53LX_ClearInterruptAndStartMeasurement();
  }
}
