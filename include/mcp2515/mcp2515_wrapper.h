#ifndef BFE120FE_8A02_4E25_837D_AA640A7C4F98
#define BFE120FE_8A02_4E25_837D_AA640A7C4F98

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can.h"

#ifdef __cplusplus
#include "mcp2515.h"
typedef struct MCP2515 mcp2515_hd;
extern "C" {
#else
typedef struct mcp2515_hd {
} mcp2515_hd;
#endif

#include "hardware/spi.h"

typedef enum {
  MCP2515_ERROR_OK = 0,
  MCP2515_ERROR_FAIL = 1,
  MCP2515_ERROR_ALLTXBUSY = 2,
  MCP2515_ERROR_FAILINIT = 3,
  MCP2515_ERROR_FAILTX = 4,
  MCP2515_ERROR_NOMSG = 5
} mcp2515_error_t;

typedef enum {
  MCP2515_MASK0 = 0,
  MCP2515_MASK1 = 1
} mcp2515_mask_t;

typedef enum {
  MCP2515_RXF0 = 0,
  MCP2515_RXF1 = 1,
  MCP2515_RXF2 = 2,
  MCP2515_RXF3 = 3,
  MCP2515_RXF4 = 4,
  MCP2515_RXF5 = 5
} mcp2515_rxf_t;

typedef enum {
  MCP2515_RXB0 = 0,
  MCP2515_RXB1 = 1
} mcp2515_rxbn_t;

typedef enum {
  MCP2515_TXB0 = 0,
  MCP2515_TXB1 = 1,
  MCP2515_TXB2 = 2
} mcp2515_txbn_t;

typedef enum {
  MCP2515_CANINTF_RX0IF = 0x01,
  MCP2515_CANINTF_RX1IF = 0x02,
  MCP2515_CANINTF_TX0IF = 0x04,
  MCP2515_CANINTF_TX1IF = 0x08,
  MCP2515_CANINTF_TX2IF = 0x10,
  MCP2515_CANINTF_ERRIF = 0x20,
  MCP2515_CANINTF_WAKIF = 0x40,
  MCP2515_CANINTF_MERRF = 0x80
} mcp2515_canintf_t;

typedef enum {
  MCP2515_EFLG_RX1OVR = (1 << 7),
  MCP2515_EFLG_RX0OVR = (1 << 6),
  MCP2515_EFLG_TXBO = (1 << 5),
  MCP2515_EFLG_TXEP = (1 << 4),
  MCP2515_EFLG_RXEP = (1 << 3),
  MCP2515_EFLG_TXWAR = (1 << 2),
  MCP2515_EFLG_RXWAR = (1 << 1),
  MCP2515_EFLG_EWARN = (1 << 0)
} mcp2515_eflg_t;

typedef enum {
  MCP2515_CAN_5KBPS,
  MCP2515_CAN_10KBPS,
  MCP2515_CAN_20KBPS,
  MCP2515_CAN_31K25BPS,
  MCP2515_CAN_33KBPS,
  MCP2515_CAN_40KBPS,
  MCP2515_CAN_50KBPS,
  MCP2515_CAN_80KBPS,
  MCP2515_CAN_83K3BPS,
  MCP2515_CAN_95KBPS,
  MCP2515_CAN_100KBPS,
  MCP2515_CAN_125KBPS,
  MCP2515_CAN_200KBPS,
  MCP2515_CAN_250KBPS,
  MCP2515_CAN_500KBPS,
  MCP2515_CAN_1000KBPS
} mcp2515_can_speed_t;

typedef enum {
  MCP2515_CLKOUT_DISABLE = -1,
  MCP2515_CLKOUT_DIV1 = 0x0,
  MCP2515_CLKOUT_DIV2 = 0x1,
  MCP2515_CLKOUT_DIV4 = 0x2,
  MCP2515_CLKOUT_DIV8 = 0x3,
} mcp2515_clkout_t;

typedef enum {
  MCP2515_20MHZ,
  MCP2515_16MHZ,
  MCP2515_8MHZ
} mcp2515_can_clock_t;

mcp2515_hd *mcp2515_init(spi_inst_t *channel,
                         uint8_t cs_pin,
                         uint8_t tx_pin,
                         uint8_t rx_pin,
                         uint8_t sck_pin,
                         uint32_t spi_clock);

mcp2515_error_t mcp2515_reset(mcp2515_hd *hd);
mcp2515_error_t mcp2515_set_config_mode(mcp2515_hd *hd);
mcp2515_error_t mcp2515_set_listen_only_mode(mcp2515_hd *hd);
mcp2515_error_t mcp2515_set_sleep_mode(mcp2515_hd *hd);
mcp2515_error_t mcp2515_set_loopback_mode(mcp2515_hd *hd);
mcp2515_error_t mcp2515_set_normal_mode(mcp2515_hd *hd);
mcp2515_error_t mcp2515_set_clk_out(mcp2515_hd *hd, const mcp2515_clkout_t divisor);
mcp2515_error_t mcp2515_set_bitrate(mcp2515_hd *hd, const mcp2515_can_speed_t can_speed);
mcp2515_error_t mcp2515_set_bitrate_with_clock(mcp2515_hd *hd, const mcp2515_can_speed_t can_speed, const mcp2515_can_clock_t can_clock);
mcp2515_error_t mcp2515_set_filter_mask(mcp2515_hd *hd, const mcp2515_mask_t num, bool ext, uint32_t ul_data);
mcp2515_error_t mcp2515_set_filter(mcp2515_hd *hd, const mcp2515_rxf_t num, bool ext, uint32_t ul_data);
mcp2515_error_t mcp2515_send_message(mcp2515_hd *hd, const mcp2515_txbn_t txbn, const struct can_frame *frame);
mcp2515_error_t mcp2515_send_message_simple(mcp2515_hd *hd, const struct can_frame *frame);
mcp2515_error_t mcp2515_read_message(mcp2515_hd *hd, const mcp2515_rxbn_t rxbn, struct can_frame *frame);
mcp2515_error_t mcp2515_read_message_simple(mcp2515_hd *hd, struct can_frame *frame);
bool mcp2515_check_receive(mcp2515_hd *hd);
bool mcp2515_check_error(mcp2515_hd *hd);
uint8_t mcp2515_get_error_flags(mcp2515_hd *hd);
void mcp2515_clear_rx_novr_flags(mcp2515_hd *hd);
uint8_t mcp2515_get_interrupts(mcp2515_hd *hd);
uint8_t mcp2515_get_interrupt_mask(mcp2515_hd *hd);
void mcp2515_clear_interrupts(mcp2515_hd *hd);
void mcp2515_clear_tx_interrupts(mcp2515_hd *hd);
uint8_t mcp2515_get_status(mcp2515_hd *hd);
void mcp2515_clear_rx_novr(mcp2515_hd *hd);
void mcp2515_clear_merr(mcp2515_hd *hd);
void mcp2515_clear_errif(mcp2515_hd *hd);
uint8_t mcp2515_error_count_rx(mcp2515_hd *hd);
uint8_t mcp2515_error_count_tx(mcp2515_hd *hd);

#ifdef __cplusplus
}
#endif

#endif /* BFE120FE_8A02_4E25_837D_AA640A7C4F98 */
