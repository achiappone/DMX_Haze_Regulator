// Definitive: route UART1 TX back to its own RX inside the peripheral. No pads
// involved, so nothing external can affect the result. If the DMX pattern comes
// back, the ESP32 is generating correct DMX and the fault is on the shield side.
#include <driver/uart.h>

#define PKT 513
uint8_t tx[PKT], rx[PKT + 16];

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  uart_config_t u = {};
  u.baud_rate = 250000;
  u.data_bits = UART_DATA_8_BITS;
  u.parity = UART_PARITY_DISABLE;
  u.stop_bits = UART_STOP_BITS_2;
  u.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  u.source_clk = UART_SCLK_DEFAULT;
  uart_driver_install(UART_NUM_1, 2048, 2048, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &u);
  uart_set_pin(UART_NUM_1, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_loop_back(UART_NUM_1, true);

  tx[0] = 0; tx[1] = 11; tx[2] = 133; tx[3] = 255; tx[4] = 77;
  Serial.println("internal loopback armed");
}

void loop() {
  uart_flush_input(UART_NUM_1);
  uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(50));
  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV);
  delayMicroseconds(100);
  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_INV_DISABLE);
  delayMicroseconds(12);
  uart_write_bytes(UART_NUM_1, tx, PKT);

  int n = uart_read_bytes(UART_NUM_1, rx, PKT + 8, pdMS_TO_TICKS(150));
  Serial.printf("rx %d bytes (want ~513):", n);
  for (int i = 0; i < 8 && i < n; i++) Serial.printf(" %02X", rx[i]);
  Serial.println("   want 00 0B 85 FF 4D ...");
  delay(900);
}
