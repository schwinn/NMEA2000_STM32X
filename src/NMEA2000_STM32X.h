#ifndef STM32_CAN_H
#define STM32_CAN_H

#include <Arduino.h>
#include "NMEA2000.h"
#include "N2kMsg.h"
#include <RingBuffer.h>

// ===========================================================================
//  Peripheral selection
//  Automatic detection based on HAL defines; can be overridden manually
//  by setting before the include:
//    #define STM32X_USE_FDCAN   or   #define STM32X_USE_BXCAN
// ===========================================================================
#if !defined(STM32X_USE_FDCAN) && !defined(STM32X_USE_BXCAN)
#if defined(FDCAN1) || defined(FDCAN2)
#define STM32X_USE_FDCAN
#else
#define STM32X_USE_BXCAN
#endif
#endif

// ===========================================================================
//  bxCAN-specific definitions
// ===========================================================================
#if defined(STM32X_USE_BXCAN)

/** Handling special cases for IRQ Handlers */
#if defined(STM32F0xx)
#if defined(STM32F042x6) || defined(STM32F072xB) || defined(STM32F091xC) || defined(STM32F098xx)
/** NOTE: STM32F0 share IRQ Handler with HDMI CEC */
#define CAN1_IRQn_AIO CEC_CAN_IRQn
#define CAN1_IRQHandler_AIO CEC_CAN_IRQHandler
#define STM32_CAN1_SHARED_WITH_CEC
#endif
#endif // STM32F0xx

#if defined(STM32F1xx)
#if defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)
/** NOTE: STM32F103xx shares IRQ Handler with USB */
#ifdef USBCON
#define STM32_CAN1_TX_RX0_BLOCKED_BY_USB
#endif
#endif
#endif // STM32F1xx

#if defined(STM32F3xx)
#if defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE) || \
    defined(STM32F303xC) || defined(STM32F303xE)
#define CAN1_TX_IRQn USB_HP_CAN_TX_IRQn
#define CAN1_RX0_IRQn USB_LP_CAN_RX0_IRQn
#define CAN1_RX1_IRQn CAN_RX1_IRQn
#define CAN1_SCE_IRQn CAN_SCE_IRQn
#define CAN1_TX_IRQHandler USB_HP_CAN_TX_IRQHandler
#define CAN1_RX0_IRQHandler USB_LP_CAN_RX0_IRQHandler
#define CAN1_RX1_IRQHandler CAN_RX1_IRQHandler
#define CAN1_SCE_IRQHandler CAN_SCE_IRQHandler
#if defined(USBCON) && !defined(USE_USB_INTERRUPT_REMAPPED)
#define STM32_CAN1_TX_RX0_BLOCKED_BY_USB
#endif
#elif defined(STM32F303x8) || defined(STM32F328xx) || defined(STM32F334x8) || \
    defined(STM32F358xx) || defined(STM32F373xC) || defined(STM32F378xx) ||   \
    defined(STM32F398xx)
#define CAN1_TX_IRQn CAN_TX_IRQn
#define CAN1_RX0_IRQn CAN_RX0_IRQn
#define CAN1_RX1_IRQn CAN_RX1_IRQn
#define CAN1_SCE_IRQn CAN_SCE_IRQn
#define CAN1_TX_IRQHandler CAN_TX_IRQHandler
#define CAN1_RX0_IRQHandler CAN_RX0_IRQHandler
#define CAN1_RX1_IRQHandler CAN_RX1_IRQHandler
#define CAN1_SCE_IRQHandler CAN_SCE_IRQHandler
#endif
#endif // STM32F3xx

#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && !defined(STM32_CAN_USB_WORKAROUND_POLLING)
#error "USB and CAN interrupts are shared on the F1/F3 platform, driver is not compatible with USBDevice of Arduino core. \
Define STM32_CAN_USB_WORKAROUND_POLLING to disable this error and call STM32_CAN_Poll_IRQ_Handler() to poll for Tx IRQ events. Only use FIFO 1."
#elif defined(USBCON) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
#warning "CAN IRQ Handler is used by USBDevice driver, call STM32_CAN_Poll_IRQ_Handler() frequently to handle CAN events."
extern "C" void STM32_CAN_Poll_IRQ_Handler(void);
#define CAN_FILTER_DEFAULT_FIFO CAN_FILTER_FIFO1
#define CAN_FILTER_DEFAULT_ACTION STORE_FIFO1
#else
#define CAN_FILTER_DEFAULT_FIFO CAN_FILTER_FIFO0
#define CAN_FILTER_DEFAULT_ACTION STORE_FIFO0
#endif

typedef struct
{
  uint32_t baudrate;
  uint16_t prescaler;
  uint8_t time_quanta;
  uint8_t timeseg1;
  uint8_t timeseg2;
} Baudrate_entry_t;

typedef struct
{
  void *__this;
  CAN_HandleTypeDef handle;
  uint32_t bus;
} stm32_can_t;

#endif // STM32X_USE_BXCAN

// ===========================================================================
//  FDCAN-specific definitions
// ===========================================================================
#if defined(STM32X_USE_FDCAN)

struct CanTiming
{
  uint32_t prescaler;
  uint32_t sjw;
  uint32_t tseg1;
  uint32_t tseg2;
};

#endif // STM32X_USE_FDCAN

// ===========================================================================
//  CAN_message_t – compatible with the Teensy FlexCAN API
//  Source: https://github.com/tonton81/FlexCAN_T4/
// ===========================================================================
typedef struct CAN_message_t
{
  uint32_t id = 0;
  uint16_t timestamp = 0;
  uint8_t idhit = 0;
  struct
  {
    bool extended = 0;
    bool remote = 0;
    bool overrun = 0;
    bool reserved = 0;
  } flags;
  uint8_t len = 8;
  uint8_t buf[8] = {0};
  int8_t mb = 0;
  uint8_t bus = 1;
  bool seq = 0;
} CAN_message_t;

// ===========================================================================
//  Class
// ===========================================================================
class tNMEA2000_STM32X : public tNMEA2000
{
public:
  // --- Constructors --------------------------------------------------------
#if defined(STM32X_USE_BXCAN)
  tNMEA2000_STM32X(uint32_t rx, uint32_t tx = PNUM_NOT_DEFINED);
  tNMEA2000_STM32X(CAN_TypeDef *canPort);
#endif
  tNMEA2000_STM32X(PinName rx, PinName tx = NC);
  virtual ~tNMEA2000_STM32X();

  // --- tNMEA2000 overrides -------------------------------------------------
  virtual void InitCANFrameBuffers() override;
  virtual bool CANOpen() override;
  virtual bool CANSendFrame(unsigned long id, unsigned char len,
                            const unsigned char *buf,
                            bool wait_sent = true) override;
  virtual bool CANGetFrame(unsigned long &id, unsigned char &len,
                           unsigned char *buf) override;

  // --- Lifecycle -----------------------------------------------------------
  // begin() returns false if the underlying HAL CAN/FDCAN peripheral failed
  // to initialize (bad pin mapping, peripheral already in use, or a HAL
  // init/start error) so CANOpen() can report failure instead of silently
  // reporting success while CAN never actually came up.
  bool begin();
  void end();

  // --- ISR interface (must remain public) ----------------------------------
  bool addToRingBuffer(const CAN_message_t &msg);
  bool sendFromTxRing();

#if defined(STM32X_USE_BXCAN)
  // --- bxCAN-specific ------------------------------------------------------
  enum FILTER_ACTION
  {
    STORE_FIFO0,
    STORE_FIFO1,
  };

  void setBaudRate(uint32_t baud);
  void setIRQPriority(uint32_t preemptPriority, uint32_t subPriority);

  bool setFilterRaw(uint8_t bank_num, uint32_t id, uint32_t mask,
                    uint32_t filter_mode, uint32_t filter_scale,
                    FILTER_ACTION action = CAN_FILTER_DEFAULT_ACTION,
                    bool enabled = true);

  void enableMBInterrupts();
  void disableMBInterrupts();
#endif

#if defined(STM32X_USE_FDCAN)
  // --- FDCAN: handle must be public so that external ISR wrappers can access it --
  static FDCAN_HandleTypeDef hcan_;

  bool getFDCANErrorCounters(uint8_t &tec, uint8_t &rec);


#endif

protected:
  tPriorityRingBuffer<CAN_message_t> *rxRing1;
  tPriorityRingBuffer<CAN_message_t> *txRing1;

private:
  bool IsOpen = false;
  PinName rx;
  PinName tx;

  void init();
  void applyFilter(); // shared (different implementations)

  // ----- bxCAN private -----
#if defined(STM32X_USE_BXCAN)
  stm32_can_t _can;
  uint32_t baudrate;
  bool filtersInitialized;
  bool _canIsActive = false;

  uint32_t preemptPriority;
  uint32_t subPriority;

  CAN_TypeDef *getPeripheral();
  bool allocatePeripheral(CAN_TypeDef *instance);
  bool freePeripheral();
  bool hasPeripheral();
  void start();
  void stop();
  void initializeFilters();

  template <typename T, size_t N>
  bool lookupBaudrate(int Baudrate, const T (&table)[N]);
  bool calculateBaudrate(int Baudrate);
  void setBaudRateValues(uint16_t prescaler, uint8_t timeseg1,
                         uint8_t timeseg2, uint8_t sjw);
  uint32_t getCanPeripheralClock();
  uint32_t fixPinFunction(uint32_t function);

  volatile CAN_message_t *rx_buffer = nullptr;

  static constexpr Baudrate_entry_t BAUD_RATE_TABLE_48M[]{
      {1000000, 3, 16, 13, 2}, {800000, 4, 15, 12, 2}, {500000, 6, 16, 13, 2}, {250000, 12, 16, 13, 2}, {125000, 24, 16, 13, 2}, {100000, 30, 16, 13, 2}};

  static constexpr Baudrate_entry_t BAUD_RATE_TABLE_45M[]{
      {1000000, 3, 15, 12, 2}, {500000, 5, 18, 15, 2}, {250000, 10, 18, 15, 2}, {125000, 20, 18, 15, 2}, {100000, 25, 18, 15, 2}};

  static constexpr Baudrate_entry_t BAUD_RATE_TABLE_42M[]{
      {1000000, 3, 14, 11, 2}, {500000, 6, 14, 11, 2}, {250000, 12, 14, 11, 2}, {125000, 21, 16, 13, 2}, {100000, 28, 15, 12, 2}};

  static constexpr Baudrate_entry_t BAUD_RATE_TABLE_36M[]{
      {1000000, 2, 18, 15, 2}, {500000, 4, 18, 15, 2}, {250000, 9, 16, 13, 2}, {125000, 18, 16, 13, 2}, {100000, 20, 18, 15, 2}};
#endif // STM32X_USE_BXCAN

  // ----- FDCAN private -----
#if defined(STM32X_USE_FDCAN)
  bool started_ = false;
  CanTiming solveCanTiming(uint32_t clockFreq, uint32_t bitrate,
                           uint8_t multiplier = 1);

  uint32_t lengthToDLC(uint32_t length);
  void subscribe();
  void unsubscribe();
#endif // STM32X_USE_FDCAN
};

#endif // STM32_CAN_H
