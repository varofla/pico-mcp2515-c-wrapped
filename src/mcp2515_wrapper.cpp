#include "../include/mcp2515/mcp2515_wrapper.h"

static MCP2515 *s_mcp2515_instance = nullptr;

mcp2515_hd *mcp2515_init(spi_inst_t *channel,
                         uint8_t cs_pin,
                         uint8_t tx_pin,
                         uint8_t rx_pin,
                         uint8_t sck_pin,
                         uint32_t spi_clock) {
  s_mcp2515_instance = new MCP2515(channel, cs_pin, tx_pin, rx_pin, sck_pin, spi_clock);
  return s_mcp2515_instance;
}

mcp2515_error_t mcp2515_reset(mcp2515_hd *hd) {
  return static_cast<mcp2515_error_t>(hd->reset());
}

mcp2515_error_t mcp2515_set_config_mode(mcp2515_hd *hd) {
  return static_cast<mcp2515_error_t>(hd->setConfigMode());
}

mcp2515_error_t mcp2515_set_listen_only_mode(mcp2515_hd *hd) {
  return static_cast<mcp2515_error_t>(hd->setListenOnlyMode());
}

mcp2515_error_t mcp2515_set_sleep_mode(mcp2515_hd *hd) {
  return static_cast<mcp2515_error_t>(hd->setSleepMode());
}

mcp2515_error_t mcp2515_set_loopback_mode(mcp2515_hd *hd) {
  return static_cast<mcp2515_error_t>(hd->setLoopbackMode());
}

mcp2515_error_t mcp2515_set_normal_mode(mcp2515_hd *hd) {
  return static_cast<mcp2515_error_t>(hd->setNormalMode());
}

mcp2515_error_t mcp2515_set_clk_out(mcp2515_hd *hd, const CAN_CLKOUT divisor) {
  return static_cast<mcp2515_error_t>(hd->setClkOut(divisor));
}

mcp2515_error_t mcp2515_set_bitrate(mcp2515_hd *hd, const mcp2515_can_speed_t can_speed) {
  return static_cast<mcp2515_error_t>(hd->setBitrate(static_cast<CAN_SPEED>(can_speed)));
}

mcp2515_error_t mcp2515_set_bitrate_with_clock(mcp2515_hd *hd, const mcp2515_can_speed_t can_speed, const mcp2515_can_clock_t can_clock) {
  return static_cast<mcp2515_error_t>(hd->setBitrate(static_cast<CAN_SPEED>(can_speed), static_cast<CAN_CLOCK>(can_clock)));
}

mcp2515_error_t mcp2515_set_filter_mask(mcp2515_hd *hd, const mcp2515_mask_t num, bool ext, uint32_t ul_data) {
  return static_cast<mcp2515_error_t>(hd->setFilterMask(static_cast<MCP2515::MASK>(num), ext, ul_data));
}

mcp2515_error_t mcp2515_set_filter(mcp2515_hd *hd, const mcp2515_rxf_t num, bool ext, uint32_t ul_data) {
  return static_cast<mcp2515_error_t>(hd->setFilter(static_cast<MCP2515::RXF>(num), ext, ul_data));
}

mcp2515_error_t mcp2515_send_message(mcp2515_hd *hd, const mcp2515_txbn_t txbn, const struct can_frame *frame) {
  return static_cast<mcp2515_error_t>(
      hd->sendMessage(
          static_cast<MCP2515::TXBn>(txbn),
          frame));
}

mcp2515_error_t mcp2515_send_message_simple(mcp2515_hd *hd, const struct can_frame *frame) {
  return static_cast<mcp2515_error_t>(hd->sendMessage(frame));
}

mcp2515_error_t mcp2515_read_message(mcp2515_hd *hd, const mcp2515_rxbn_t rxbn, struct can_frame *frame) {
  return static_cast<mcp2515_error_t>(
      hd->readMessage(
          static_cast<MCP2515::RXBn>(rxbn),
          frame));
}

mcp2515_error_t mcp2515_read_message_simple(mcp2515_hd *hd, struct can_frame *frame) {
  return static_cast<mcp2515_error_t>(hd->readMessage(frame));
}

bool mcp2515_check_receive(mcp2515_hd *hd) {
  return hd->checkReceive();
}

bool mcp2515_check_error(mcp2515_hd *hd) {
  return hd->checkError();
}

uint8_t mcp2515_get_error_flags(mcp2515_hd *hd) {
  return hd->getErrorFlags();
}

void mcp2515_clear_rx_novr_flags(mcp2515_hd *hd) {
  hd->clearRXnOVRFlags();
}

uint8_t mcp2515_get_interrupts(mcp2515_hd *hd) {
  return hd->getInterrupts();
}

uint8_t mcp2515_get_interrupt_mask(mcp2515_hd *hd) {
  return hd->getInterruptMask();
}

void mcp2515_clear_interrupts(mcp2515_hd *hd) {
  hd->clearInterrupts();
}

void mcp2515_clear_tx_interrupts(mcp2515_hd *hd) {
  hd->clearTXInterrupts();
}

uint8_t mcp2515_get_status(mcp2515_hd *hd) {
  return hd->getStatus();
}

void mcp2515_clear_rx_novr(mcp2515_hd *hd) {
  hd->clearRXnOVR();
}

void mcp2515_clear_merr(mcp2515_hd *hd) {
  hd->clearMERR();
}

void mcp2515_clear_errif(mcp2515_hd *hd) {
  hd->clearERRIF();
}

uint8_t mcp2515_error_count_rx(mcp2515_hd *hd) {
  return hd->errorCountRX();
}

uint8_t mcp2515_error_count_tx(mcp2515_hd *hd) {
  return hd->errorCountTX();
}
