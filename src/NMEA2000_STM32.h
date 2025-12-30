
#ifndef STM32_CAN_H
#define STM32_CAN_H

#include <Arduino.h>
#include "NMEA2000.h"
#include "N2kMsg.h"
#include <RingBuffer.h>

/** Handling special cases for IRQ Handlers */
#if defined(STM32F0xx)
#if defined(STM32F042x6) || defined(STM32F072xB) || defined(STM32F091xC) || defined(STM32F098xx)

/**
 * NOTE: STM32F0 share IRQ Handler with HDMI CEC
 * and there is only a single IRQ Handler not 4
 * CEC_CAN_IRQn | CEC_CAN_IRQHandler
 *
 * define all in one alias for IRQn and Handler
 */
#define CAN1_IRQn_AIO CEC_CAN_IRQn
#define CAN1_IRQHandler_AIO CEC_CAN_IRQHandler
/**
 * NOTE: CAN IRQ is shared with CEC
 * To use CEC with CAN declare:
 * CEC_HandleTypeDef * phcec;
 * and point to your CEC handle.
 * Internal IRQ Handler will call CEC Handler as well.
 */
#define STM32_CAN1_SHARED_WITH_CEC

#endif
#endif

#if defined(STM32F1xx)
#if defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)
/**
 * NOTE: STM32F103xx uses shared IRQ Handler with USB
 * USB_HP_CAN1_TX_IRQn  | USB_HP_CAN1_TX_IRQHandler
 * USB_LP_CAN1_RX0_IRQn | USB_LP_CAN1_RX0_IRQHandler
 *
 * the CMSIS files define already aliases for the CAN *_IRQHandler and *_IRQn
 * conforming with standard naming convention:
 * CAN1_TX_IRQn  | CAN1_TX_IRQHandler
 * CAN1_RX0_IRQn | CAN1_RX0_IRQHandler
 *
 * when USB is enabled the USBDevice driver also implements these making a concurrent use impossible.
 *
 * Following are unaffected:
 * CAN1_RX1_IRQn | CAN1_RX1_IRQHandler
 * CAN1_SCE_IRQn | CAN1_SCE_IRQHandler
 */

#ifdef USBCON
#define STM32_CAN1_TX_RX0_BLOCKED_BY_USB
#endif

#endif
#endif

#if defined(STM32F3xx)
#if defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE) || defined(STM32F303xC) || defined(STM32F303xE)

/**
 * NOTE: STM32F3 with USB share IRQ Handler with it
 * USB_HP_CAN_TX_IRQn  | USB_HP_CAN_TX_IRQHandler
 * USB_LP_CAN_RX0_IRQn | USB_LP_CAN_RX0_IRQHandler
 *
 * the CMSIS files define already aliases for the CAN *_IRQHandler and *_IRQn
 * missing peripheral index:
 * CAN_TX_IRQn  | CAN_TX_IRQHandler
 * CAN_RX0_IRQn | CAN_RX0_IRQHandler
 *
 * when USB is enabled the USBDevice driver also implements these making a concurrent use impossible.
 *
 * Following are unaffected:
 * CAN_RX1_IRQn | CAN_RX1_IRQHandler
 * CAN_SCE_IRQn | CAN_SCE_IRQHandler
 *
 * define more aliases with peripheral index
 */

#define CAN1_TX_IRQn USB_HP_CAN_TX_IRQn
#define CAN1_RX0_IRQn USB_LP_CAN_RX0_IRQn
#define CAN1_RX1_IRQn CAN_RX1_IRQn
#define CAN1_SCE_IRQn CAN_SCE_IRQn

#define CAN1_TX_IRQHandler USB_HP_CAN_TX_IRQHandler
#define CAN1_RX0_IRQHandler USB_LP_CAN_RX0_IRQHandler
#define CAN1_RX1_IRQHandler CAN_RX1_IRQHandler
#define CAN1_SCE_IRQHandler CAN_SCE_IRQHandler

/** NOTE: USE_USB_INTERRUPT_REMAPPED may be used to use
 * different USB IRQs and not block the CAN IRQ handlers */
#if defined(USBCON) && !defined(USE_USB_INTERRUPT_REMAPPED)
#define STM32_CAN1_TX_RX0_BLOCKED_BY_USB
#endif

#elif defined(STM32F303x8) || defined(STM32F328xx) || defined(STM32F334x8) || defined(STM32F358xx) || defined(STM32F373xC) || defined(STM32F378xx) || defined(STM32F398xx)

/**
 * NOTE: STM32F3 without USB define symbols without peripheral index
 * define more aliases with peripheral index
 */

#define CAN1_TX_IRQn CAN_TX_IRQn
#define CAN1_RX0_IRQn CAN_RX0_IRQn
#define CAN1_RX1_IRQn CAN_RX1_IRQn
#define CAN1_SCE_IRQn CAN_SCE_IRQn

#define CAN1_TX_IRQHandler CAN_TX_IRQHandler
#define CAN1_RX0_IRQHandler CAN_RX0_IRQHandler
#define CAN1_RX1_IRQHandler CAN_RX1_IRQHandler
#define CAN1_SCE_IRQHandler CAN_SCE_IRQHandler
#endif
#endif

#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && !defined(STM32_CAN_USB_WORKAROUND_POLLING)
#error "USB and CAN interrupts are shared on the F1/F3 platform, driver is not compatible with USBDevice of Arduino core. Can define STM32_CAN_USB_WORKAROUND_POLLING to disable error msg and call STM32_CAN_Poll_IRQ_Handler to poll for Tx IRQ events. Only use FIFO 1."
#elif defined(USBCON) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
#warning "CAN IRQ Handler is used by USBDevice driver, call STM32_CAN_Poll_IRQ_Handler() frequently to handle CAN events."
extern "C" void STM32_CAN_Poll_IRQ_Handler(void);
#define CAN_FILTER_DEFAULT_FIFO CAN_FILTER_FIFO1
#define CAN_FILTER_DEFAULT_ACTION STORE_FIFO1
#else
#define CAN_FILTER_DEFAULT_FIFO CAN_FILTER_FIFO0
#define CAN_FILTER_DEFAULT_ACTION STORE_FIFO0
#endif

// This struct is directly copied from Teensy FlexCAN library to retain compatibility with it. Not all are in use with STM32.
// Source: https://github.com/tonton81/FlexCAN_T4/

typedef struct CAN_message_t
{
  uint32_t id = 0;        // can identifier
  uint16_t timestamp = 0; // time when message arrived
  uint8_t idhit = 0;      // filter that id came from
  struct
  {
    bool extended = 0; // identifier is extended (29-bit)
    bool remote = 0;   // remote transmission request packet type
    bool overrun = 0;  // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;      // length of data
  uint8_t buf[8] = {0}; // data
  int8_t mb = 0;        // used to identify mailbox reception
  uint8_t bus = 1;      // used to identify where the message came (CAN1, CAN2 or CAN3)
  bool seq = 0;         // sequential frames
} CAN_message_t;

typedef struct
{
  uint32_t baudrate;
  uint16_t prescaler;
  uint8_t time_quanta;
  uint8_t timeseg1;
  uint8_t timeseg2;
} Baudrate_entry_t;

typedef enum CAN_PINS
{
  DEF,
  ALT,
  ALT_2,
} CAN_PINS;

typedef enum IDE
{
  STD = 0,
  EXT = 1,
  AUTO = 2
} IDE;

typedef struct
{
  void *__this;
  CAN_HandleTypeDef handle;
  uint32_t bus;
} stm32_can_t;

class tNMEA2000_STM32 : public tNMEA2000
{

public:
  tNMEA2000_STM32(uint32_t rx, uint32_t tx = PNUM_NOT_DEFINED);
  tNMEA2000_STM32(PinName rx, PinName tx = NC);
  tNMEA2000_STM32(CAN_TypeDef *canPort);
  tNMEA2000_STM32(CAN_TypeDef *_canBus, CAN_PINS pins);
  virtual ~tNMEA2000_STM32();

  virtual void InitCANFrameBuffers() override;
  virtual bool CANOpen() override;
  virtual bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent = true) override;
  virtual bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override;

  enum MODE
  {
    NORMAL = CAN_MODE_NORMAL,
    SILENT = CAN_MODE_SILENT,
    SILENT_LOOPBACK = CAN_MODE_SILENT_LOOPBACK,
    LOOPBACK = CAN_MODE_LOOPBACK
  };

  enum FILTER_ACTION
  {
    STORE_FIFO0,
    STORE_FIFO1,
  };

  enum TX_BUFFER_MODE
  {
    FIFO = ENABLE,  /** Sequential transfers order */
    QUEUE = DISABLE /** Sequence based on msg ID priorities. Only effects hardware queue. */
  };

  /**-------------------------------------------------------------
   *     setup functions
   *     no effect after begin()
   * -------------------------------------------------------------
   */
  void setIRQPriority(uint32_t preemptPriority, uint32_t subPriority);

  /** send message again on arbitration failure */
  void setAutoRetransmission(bool enabled);

  /** If locked incoming msg is dropped when fifo is full,
   *  when unlocked last msg in fifo is overwritten
   *  2nd arg has no effect, setting effects both fifos */
  void setRxFIFOLock(bool fifo0locked, bool fifo1locked = true);
  void setTxBufferMode(TX_BUFFER_MODE mode);
  void setTimestampCounter(bool enabled);

  void setMode(MODE mode);
  void setAutoBusOffRecovery(bool enabled);

  /**-------------------------------------------------------------
   *     lifecycle functions
   *     setBaudRate may be called before or after begin
   * -------------------------------------------------------------
   */
  // Begin. By default the automatic retransmission is enabled. If it causes problems, use begin(false) to disable it.
  void begin();
  void end(void);

  void setBaudRate(uint32_t baud);

  /**-------------------------------------------------------------
   *     post begin(), setup filters, data transfer
   * -------------------------------------------------------------
   */
  bool sendFromTxRing();

  /** returns number of available filter banks. If hasSharedFilterBanks() is false counts may differ by id type. */
  uint8_t getFilterBankCount(IDE std_ext = STD);
  /** returns if filter count and index are shared (true) or dedicated per id type (false) */
  bool hasSharedFilterBanks()
  {
    return true;
  }

  /**
   * Manually set STM32 filter bank parameters
   * These return true on success
   */
  /** set filter state and action, keeps filter rules intact */
  bool setFilter(uint8_t bank_num, bool enabled, FILTER_ACTION action = CAN_FILTER_DEFAULT_ACTION);
  bool setFilterRaw(uint8_t bank_num, uint32_t id, uint32_t mask, uint32_t filter_mode, uint32_t filter_scale, FILTER_ACTION action = CAN_FILTER_DEFAULT_ACTION, bool enabled = true);
  /** Legacy, broken! Only works correctly for 32 bit mask mode
   * Returns true on Error, false on Success (like Teensy functions, opposite of STM32 function)
   */
  bool setFilter(uint8_t bank_num, uint32_t filter_id, uint32_t mask, IDE = AUTO, uint32_t filter_mode = CAN_FILTERMODE_IDMASK, uint32_t filter_scale = CAN_FILTERSCALE_32BIT, uint32_t fifo = CAN_FILTER_DEFAULT_FIFO);

  void enableFIFO(bool status = 1);
  void enableMBInterrupts();
  void disableMBInterrupts();

  bool addToRingBuffer(const CAN_message_t &msg);


protected:
  tPriorityRingBuffer<CAN_message_t> *rxRing1;
  tPriorityRingBuffer<CAN_message_t> *txRing1;

private:
  bool IsOpen;

  void init(void);
  CAN_TypeDef *getPeripheral(void);
  bool allocatePeripheral(CAN_TypeDef *instance);
  bool freePeripheral(void);
  bool hasPeripheral(void);
  void start(void);
  void stop(void);
  void initializeFilters();
  bool isInitialized() { return rx_buffer != 0; }
  void initializeBuffers(void);
  void freeBuffers(void);

  template <typename T, size_t N>
  bool lookupBaudrate(int Baudrate, const T (&table)[N]);
  bool calculateBaudrate(int Baudrate);
  void setBaudRateValues(uint16_t prescaler, uint8_t timeseg1,
                         uint8_t timeseg2, uint8_t sjw);
  uint32_t getCanPeripheralClock(void);
  uint32_t fixPinFunction(uint32_t function);

  volatile CAN_message_t *rx_buffer = nullptr;
  // volatile CAN_message_t *tx_buffer = nullptr;

  static constexpr Baudrate_entry_t BAUD_RATE_TABLE_48M[]{
      {1000000, 3, 16, 13, 2},
      {800000, 4, 15, 12, 2},
      {500000, 6, 16, 13, 2},
      {250000, 12, 16, 13, 2},
      {125000, 24, 16, 13, 2},
      {100000, 30, 16, 13, 2}};

  static constexpr Baudrate_entry_t BAUD_RATE_TABLE_45M[]{
      {1000000, 3, 15, 12, 2},
      {500000, 5, 18, 15, 2},
      {250000, 10, 18, 15, 2},
      {125000, 20, 18, 15, 2},
      {100000, 25, 18, 15, 2}};

  static constexpr Baudrate_entry_t BAUD_RATE_TABLE_42M[]{
      {1000000, 3, 14, 11, 2},
      {500000, 6, 14, 11, 2},
      {250000, 12, 14, 11, 2},
      {125000, 21, 16, 13, 2},
      {100000, 28, 15, 12, 2}};

  static constexpr Baudrate_entry_t BAUD_RATE_TABLE_36M[]{
      {1000000, 2, 18, 15, 2},
      {500000, 4, 18, 15, 2},
      {250000, 9, 16, 13, 2},
      {125000, 18, 16, 13, 2},
      {100000, 20, 18, 15, 2}};

  bool _canIsActive = false;

  uint32_t baudrate;
  bool filtersInitialized;

  PinName rx;
  PinName tx;

  uint32_t preemptPriority;
  uint32_t subPriority;

 stm32_can_t _can;
};

#endif
