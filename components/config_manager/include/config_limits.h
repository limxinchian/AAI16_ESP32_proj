#pragma once
/* Validation limits for ConfigApply. Array maxima (ops, decode, profiles, tx
 * bytes) live in protocol/aai16.options and must stay in step with these. */

#define CM_BUS_COUNT          4      /* GPIO, I2C, UART, SPI: one active profile per bus */

#define CM_SENSOR_ID_MIN      1
#define CM_SENSOR_ID_MAX      127   /* 128..255 are virtual (bridge) sensors */

#define CM_PERIOD_MS_MIN      10
#define CM_PERIOD_MS_MAX      3600000

#define CM_MAX_RX_BYTES       64     /* total bytes produced by one op sequence (= Reading.raw) */
#define CM_MAX_BIT_FRAME_BYTES 16
#define CM_DELAY_US_MAX       1000000
#define CM_TIMEOUT_US_MAX     1000000
#define CM_THRESHOLD_US_MAX   1000

#define CM_I2C_ADDR_MIN       0x08
#define CM_I2C_ADDR_MAX       0x77
#define CM_I2C_HZ_MIN         10000
#define CM_I2C_HZ_MAX         1000000

#define CM_UART_PORT_MIN      1      /* UART0 is the console */
#define CM_UART_PORT_MAX      2
#define CM_UART_BAUD_MIN      1200
#define CM_UART_BAUD_MAX      2000000

#define CM_SPI_HZ_MIN         100000
#define CM_SPI_HZ_MAX         20000000
#define CM_SPI_MODE_MAX       3

#define CM_OUTPUT_MAX         8      /* = Reading.values max_count; outputs are r0..r7 */
#define CM_REG_COUNT          32
#define CM_DECODE_MAX         32     /* = SensorProfile.decode max_count */
#define CM_INIT_OPS_MAX       6
#define CM_INIT_RX_MAX        64     /* total bytes produced by init_ops */
#define CM_CALC_STEPS_MAX     32
#define CM_PROGRAM_MAX        256
#define CM_STACK_MAX          8
#define CM_CHECKS_MAX         2
#define CM_TAIL_MAX           4

#define CM_SCHEMA_VERSION     1      /* CommittedConfig.schema_version */
