#include <device.h>
#include <toolchain.h>

/* 1 : /soc/rcc@40023800:
 * - (/soc)
 * - (/clocks/pll)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_rcc_40023800[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 2 : /soc/pin-controller@40020000/gpio@40022800:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40022800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 3 : /soc/pin-controller@40020000/gpio@40022400:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40022400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 4 : /soc/pin-controller@40020000/gpio@40022000:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40022000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 5 : /soc/pin-controller@40020000/gpio@40021C00:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40021c00[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 6 : /soc/pin-controller@40020000/gpio@40021800:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40021800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 7 : /soc/pin-controller@40020000/gpio@40021400:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40021400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 8 : /soc/pin-controller@40020000/gpio@40021000:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40021000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 9 : /soc/pin-controller@40020000/gpio@40020C00:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40020c00[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 10 : /soc/pin-controller@40020000/gpio@40020800:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40020800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 11 : /soc/pin-controller@40020000/gpio@40020400:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40020400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 12 : /soc/pin-controller@40020000/gpio@40020000:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40020000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 13 : /soc/interrupt-controller@40013c00:
 * - (/soc)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_interrupt_controller_40013c00[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 14 : /soc/serial@40011400:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/usart6_cts_pg13)
 * - (/soc/pin-controller@40020000/usart6_rts_pg12)
 * - (/soc/pin-controller@40020000/usart6_rx_pg9)
 * - (/soc/pin-controller@40020000/usart6_tx_pg14)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40011400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 15 : /soc/serial@40004c00:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/uart4_rx_pa1)
 * - (/soc/pin-controller@40020000/uart4_tx_pa0)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40004c00[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 16 : /soc/serial@40004400:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/usart2_rx_pd6)
 * - (/soc/pin-controller@40020000/usart2_tx_pd5)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40004400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 17 : /soc/serial@40011000:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/usart1_rx_pa10)
 * - (/soc/pin-controller@40020000/usart1_tx_pa9)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40011000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 18 : sysinit:
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_sys_init_sys_clock_driver_init0[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 19 : /soc/adc@40012000:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/adc1_in10_pc0)
 * - (/soc/pin-controller@40020000/adc1_in11_pc1)
 * - (/soc/pin-controller@40020000/adc1_in12_pc2)
 * - (/soc/pin-controller@40020000/adc1_in13_pc3)
 * - (/soc/pin-controller@40020000/adc1_in8_pb0)
 * - (/soc/pin-controller@40020000/adc1_in9_pb1)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_adc_40012000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 20 : /soc/i2c@40006000:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/i2c4_scl_pf14)
 * - (/soc/pin-controller@40020000/i2c4_sda_pf15)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_i2c_40006000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 21 : /soc/i2c@40005800:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/i2c2_scl_pf1)
 * - (/soc/pin-controller@40020000/i2c2_sda_pf0)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_i2c_40005800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 22 : /soc/timers@40010400/pwm:
 * - (/soc/timers@40010400)
 * - (/soc/pin-controller@40020000/tim8_ch1_pi5)
 * - (/soc/pin-controller@40020000/tim8_ch2_pi6)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_timers_40010400_S_pwm[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 23 : /soc/timers@40000c00/pwm:
 * - (/soc/timers@40000c00)
 * - (/soc/pin-controller@40020000/tim5_ch1_ph10)
 * - (/soc/pin-controller@40020000/tim5_ch2_ph11)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_timers_40000c00_S_pwm[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 24 : /soc/timers@40000800/pwm:
 * - (/soc/timers@40000800)
 * - (/soc/pin-controller@40020000/tim4_ch1_pd12)
 * - (/soc/pin-controller@40020000/tim4_ch2_pd13)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_timers_40000800_S_pwm[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 25 : /soc/timers@40000400/pwm:
 * - (/soc/timers@40000400)
 * - (/soc/pin-controller@40020000/tim3_ch1_pc6)
 * - (/soc/pin-controller@40020000/tim3_ch2_pc7)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_timers_40000400_S_pwm[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 26 : /soc/timers@40000000/pwm:
 * - (/soc/timers@40000000)
 * - (/soc/pin-controller@40020000/tim2_ch3_pb10)
 * - (/soc/pin-controller@40020000/tim2_ch4_pb11)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_timers_40000000_S_pwm[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 27 : /soc/timers@40010000/pwm:
 * - (/soc/timers@40010000)
 * - (/soc/pin-controller@40020000/tim1_ch1_pe9)
 * - (/soc/pin-controller@40020000/tim1_ch2_pe11)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_timers_40010000_S_pwm[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 28 : /soc/can@40006800:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/can2_rx_pb12)
 * - (/soc/pin-controller@40020000/can2_tx_pb13)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_can_40006800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 29 : /soc/flash-controller@40023c00:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_flash_controller_40023c00[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 30 : /soc/spi@40015400:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/spi6_miso_pa6)
 * - (/soc/pin-controller@40020000/spi6_mosi_pa7)
 * - (/soc/pin-controller@40020000/spi6_sck_pa5)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40015400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 31 : /soc/spi@40015000:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/spi5_miso_pf8)
 * - (/soc/pin-controller@40020000/spi5_mosi_pf9)
 * - (/soc/pin-controller@40020000/spi5_nss_pf6)
 * - (/soc/pin-controller@40020000/spi5_sck_pf7)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40015000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 32 : /soc/spi@40013400:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/spi4_miso_pe5)
 * - (/soc/pin-controller@40020000/spi4_mosi_pe6)
 * - (/soc/pin-controller@40020000/spi4_sck_pe2)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 33 : /soc/spi@40003800:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/spi2_miso_pi2)
 * - (/soc/pin-controller@40020000/spi2_mosi_pi3)
 * - (/soc/pin-controller@40020000/spi2_sck_pi1)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40003800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 34 : /soc/spi@40013000:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/spi1_miso_pb4)
 * - (/soc/pin-controller@40020000/spi1_mosi_pb5)
 * - (/soc/pin-controller@40020000/spi1_sck_pg11)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 35 : /soc/sdmmc@40012c00:
 * - (/soc)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/sdmmc1_ck_pc12)
 * - (/soc/pin-controller@40020000/sdmmc1_cmd_pd2)
 * - (/soc/pin-controller@40020000/sdmmc1_d0_pc8)
 * - (/soc/pin-controller@40020000/sdmmc1_d1_pc9)
 * - (/soc/pin-controller@40020000/sdmmc1_d2_pc10)
 * - (/soc/pin-controller@40020000/sdmmc1_d3_pc11)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_sdmmc_40012c00[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 36 : /soc/spi@40015400/ws2812@2:
 * - /soc/spi@40015400
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40015400_S_ws2812_2[] = { 30, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 37 : /soc/spi@40013400/ws2812@0:
 * - /soc/spi@40013400
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013400_S_ws2812_0[] = { 32, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 38 : /soc/spi@40003800/ws2812@3:
 * - /soc/spi@40003800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40003800_S_ws2812_3[] = { 33, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 39 : /soc/spi@40013000/ws2812@1:
 * - /soc/spi@40013000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013000_S_ws2812_1[] = { 34, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 40 : /soc/spi@40015000/adis16470@0:
 * - /soc/spi@40015000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40015000_S_adis16470_0[] = { 31, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 41 : sysinit:
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_maxbotix_4[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 42 : sysinit:
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_maxbotix_3[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 43 : sysinit:
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_maxbotix_2[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 44 : sysinit:
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_maxbotix_1[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 45 : sysinit:
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_maxbotix_0[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };
