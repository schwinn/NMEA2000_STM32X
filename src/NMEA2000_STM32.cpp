#include "NMEA2000_STM32.h"

#include "core_debug.h"

#if defined(HAL_CEC_MODULE_ENABLED) && defined(STM32_CAN1_SHARED_WITH_CEC)
/** Pointer to CEC_HandleTypeDef structure that contains
 * the configuration information for the specified CEC.
 * Application have to declare them properly to be able to call
 * the HAL_CEC_IRQHandler().
 */
extern CEC_HandleTypeDef *phcec;
#endif

#define STM32_CAN_SINGLE_CAN_FILTER_COUNT 14
#define STM32_CAN_DUAL_CAN_FILTER_COUNT 28
#define STM32_CAN_CAN2_FILTER_OFFSET 14

// Max value, lowest priority
#define MAX_IRQ_PRIO_VALUE ((1UL << __NVIC_PRIO_BITS) - 1UL)

constexpr Baudrate_entry_t tNMEA2000_STM32::BAUD_RATE_TABLE_48M[];
constexpr Baudrate_entry_t tNMEA2000_STM32::BAUD_RATE_TABLE_45M[];
constexpr Baudrate_entry_t tNMEA2000_STM32::BAUD_RATE_TABLE_42M[];
constexpr Baudrate_entry_t tNMEA2000_STM32::BAUD_RATE_TABLE_36M[];

uint32_t test = 0;

typedef enum
{
#ifdef CAN1
  CAN1_INDEX,
#endif
#ifdef CAN2
  CAN2_INDEX,
#endif
#ifdef CAN3
  CAN3_INDEX,
#endif
  CAN_NUM,
  CAN_UNKNOWN = 0xFFFF
} can_index_t;

static stm32_can_t *canObj[CAN_NUM] = {NULL};

stm32_can_t *get_can_obj(CAN_HandleTypeDef *hcan)
{
  stm32_can_t *obj;
  obj = (stm32_can_t *)((char *)hcan - offsetof(stm32_can_t, handle));
  return (obj);
}

can_index_t get_can_index(CAN_TypeDef *instance)
{
  can_index_t index = CAN_UNKNOWN;
#if defined(CAN1)
  if (instance == CAN1)
  {
    index = CAN1_INDEX;
  }
#endif
#if defined(CAN2)
  if (instance == CAN2)
  {
    index = CAN2_INDEX;
  }
#endif
#if defined(CAN3)
  if (instance == CAN3)
  {
    index = CAN3_INDEX;
  }
#endif
  if (index == CAN_UNKNOWN)
  {
    Error_Handler();
  }
  return index;
}

bool tNMEA2000_STM32::allocatePeripheral(CAN_TypeDef *instance)
{
  can_index_t index = get_can_index(instance);
  if (index >= CAN_NUM)
  {
    return false;
  }
  if (canObj[index])
  {
    // bus already in use by other instance
    Error_Handler();
    return false;
  }
  _can.handle.Instance = instance;
  // register with global, we own this instance now
  canObj[index] = &_can;
  return true;
}

bool tNMEA2000_STM32::freePeripheral()
{
  if (_can.handle.Instance == nullptr)
    return false;
  can_index_t index = get_can_index(_can.handle.Instance);
  if (index >= CAN_NUM)
  {
    return false;
  }
  if (canObj[index] == &_can)
  {
    canObj[index] = nullptr;
    _can.handle.Instance = nullptr;
    return true;
  }
  Error_Handler();
  return false;
}

bool tNMEA2000_STM32::hasPeripheral()
{
  if (_can.handle.Instance == nullptr)
    return false;
  can_index_t index = get_can_index(_can.handle.Instance);
  if (index >= CAN_NUM)
  {
    return false;
  }
  return canObj[index] == &_can;
}

// legacy pin config for compatibility
tNMEA2000_STM32::tNMEA2000_STM32(CAN_TypeDef *canPort, CAN_PINS pins)
    : rx(NC), tx(NC), rxRing1(0), txRing1(0),
      preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0), IsOpen(false), tNMEA2000()
{
  if (canPort == CAN1)
  {
    switch (pins)
    {
    case DEF:
      rx = PA_11;
      tx = PA_12;
      break;
    case ALT:
      rx = PB_8;
      tx = PB_9;
      break;
#if defined(__HAL_RCC_GPIOD_CLK_ENABLE)
    case ALT_2:
      rx = PD_0;
      tx = PD_1;
      break;
#endif
    }
  }
#ifdef CAN2
  else if (canPort == CAN2)
  {
    switch (pins)
    {
    case DEF:
      rx = PB_12;
      tx = PB_13;
      break;
    case ALT:
      rx = PB_5;
      tx = PB_6;
      break;
    }
  }
#endif
#ifdef CAN3
  else if (canPort == CAN3)
  {
    switch (pins)
    {
    case DEF:
      rx = PA_8;
      tx = PA_15;
      break;
    case ALT:
      rx = PB_3;
      tx = PB_4;
      break;
    }
  }
#endif
  init();
}

tNMEA2000_STM32::tNMEA2000_STM32(uint32_t rx, uint32_t tx)
    : rxRing1(0), txRing1(0),
      preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0), IsOpen(false), tNMEA2000()
{
  this->rx = digitalPinToPinName(rx);
  this->tx = digitalPinToPinName(tx);
  init();
}

tNMEA2000_STM32::tNMEA2000_STM32(PinName rx, PinName tx)
    : rx(rx), tx(tx), rxRing1(0), txRing1(0),
      preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0), IsOpen(false), tNMEA2000()
{
  init();
}

tNMEA2000_STM32::tNMEA2000_STM32(CAN_TypeDef *canPort)
    : rxRing1(0), txRing1(0),
      preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0), IsOpen(false), tNMEA2000()
{
  // get first matching pins from map
  rx = pinmap_find_pin(canPort, PinMap_CAN_RD);
  tx = pinmap_find_pin(canPort, PinMap_CAN_TD);
  init();
}

tNMEA2000_STM32::~tNMEA2000_STM32()
{
  end();
}

bool tNMEA2000_STM32::CANOpen()
{

  if (IsOpen)
    return true;

  IsOpen = true;

  begin();
  setBaudRate(250000);

#if defined(STMCANDEBUG)
  Serial.begin(115200);
#endif
  return IsOpen;
}



extern void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);

bool tNMEA2000_STM32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent)
{
  if (!_can.handle.Instance)
    return false;

#if !defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB)
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif

  uint8_t prio = (id >> 26) & 0x7;
  bool mailboxBusy = (HAL_CAN_GetTxMailboxesFreeLevel(&_can.handle) == 0);
  uint32_t TxMailbox;
  uint32_t dlc = (len > 8) ? 8 : len;

  CAN_TxHeaderTypeDef TxHeader = {
      .ExtId = id,
      .IDE = CAN_ID_EXT,
      .RTR = CAN_RTR_DATA,
      .DLC = dlc,
      .TransmitGlobalTime = DISABLE};

#if defined(STMCANDEBUG)
  unsigned long pgn;
  unsigned char src, dst;
  CanIdToN2k(id, prio, pgn, src, dst);
  Serial.printf("%6lu - CANSendFrame pgn:%6lu, prio:%u, src:%u, dst:%u, data:%02X\n",
                millis(), pgn, prio, src, dst, buf[0]);
#endif


  bool sendFromBuffer = false;

  if (!txRing1->isEmpty() || mailboxBusy)
  {
    CAN_message_t *msg = txRing1->getAddRef(prio);
    if (msg)
    {
      msg->id = id;
      msg->len = TxHeader.DLC;
      memcpy(msg->buf, buf, TxHeader.DLC);
    }
#if defined(STMCANDEBUG)
    Serial.printf("  frame buffered\n");
#endif
    sendFromBuffer = true;
  }

  bool result = true;

  if (!mailboxBusy)
  {
    if (sendFromBuffer)
    {
      sendFromTxRing();
    }
    else
    {
      if (HAL_CAN_AddTxMessage(&_can.handle, &TxHeader, buf, &TxMailbox) != HAL_OK)
        result = false;
    }
  }

#if !defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB)
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif

  return result;
}

bool tNMEA2000_STM32::sendFromTxRing()
{
  const CAN_message_t *txMsg;

  txMsg = txRing1->getReadRef();
  if (txMsg != 0)
  {
#if defined(STMCANDEBUG)
    unsigned long pgn;
    unsigned char src;
    unsigned char dst;
    CanIdToN2k(txMsg->id, prio, pgn, src, dst);
    Serial.printf("sendFromTxRing pgn:%6lu, prio:%u, src:%u, dst:%u, data:%02X\n", pgn, prio, src, dst, txMsg->buf[0]);
#endif

    CAN_TxHeaderTypeDef TxHeader;
    CAN_message_t tmp = *txMsg;

    TxHeader.ExtId = tmp.id;
    TxHeader.IDE = CAN_ID_EXT;

    if (tmp.flags.remote == 1) // Remote frame when CAN_tx_msg.flags.remote is 1
    {
      TxHeader.RTR = CAN_RTR_REMOTE;
      TxHeader.DLC = 0;
    }
    else
    {
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.DLC = tmp.len;
    }

    TxHeader.TransmitGlobalTime = DISABLE;
    uint32_t TxMailbox;
    if (HAL_CAN_AddTxMessage(&_can.handle, &TxHeader, tmp.buf, &TxMailbox) != HAL_OK)
    {
#if defined(STMCANDEBUG)
      Serial.println("sendFromTxRing failed");
#endif
    }
    return true;
  }

  return false;
}

bool tNMEA2000_STM32::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{

  bool ret = false;

#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
#else
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif

  const CAN_message_t *msg = rxRing1->getReadRef();
  if (msg != 0)
  {
    id = msg->id;
    len = msg->len;
    if (len > 8)
      len = 8;
    memcpy(buf, msg->buf, len);
    ret = true;
  }

#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
#else
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif

#if defined(STMCANDEBUG)
  if (msg != 0)
  {
    unsigned long pgn;
    unsigned char prio;
    unsigned char src;
    unsigned char dst;
    CanIdToN2k(id, prio, pgn, src, dst);
    Serial.printf("%6lu - tNMEA2000_Teensyx::CANGetFrame pgn:%6lu, prio:%u, src:%u, dst:%u, data:%02X\n", millis(), pgn, prio, src, dst, buf[0]);
  }
#endif

  return ret;
}

void tNMEA2000_STM32::InitCANFrameBuffers()
{

  if (MaxCANReceiveFrames == 0)
    MaxCANReceiveFrames = 60; // Use default, if not set
  if (MaxCANReceiveFrames < 30)
    MaxCANReceiveFrames = 30; // Do not allow less than 30 - Teensy 4.x should have enough memory.
  if (MaxCANSendFrames == 0)
    MaxCANSendFrames = 80; // Use big enough default buffer
  if (MaxCANSendFrames < 50)
    MaxCANSendFrames = 50; // Do not allow less than 30 - Teensy should have enough memory.

  if (rxRing1 != 0 && rxRing1->getSize() != MaxCANReceiveFrames)
  {
    delete rxRing1;
    rxRing1 = 0;
  }
  if (txRing1 != 0 && txRing1->getSize() != MaxCANSendFrames)
  {
    delete rxRing1;
    txRing1 = 0;
  }

  if (rxRing1 == 0)
    rxRing1 = new tPriorityRingBuffer<CAN_message_t>(MaxCANReceiveFrames, 8);
  if (txRing1 == 0)
    txRing1 = new tPriorityRingBuffer<CAN_message_t>(MaxCANSendFrames, 8);
}

void tNMEA2000_STM32::init(void)
{
  _can.__this = (void *)this;
  _can.handle.Instance = nullptr;
  baudrate = 0UL;
  filtersInitialized = false;

  setTimestampCounter(false);
  setAutoBusOffRecovery(true);
  _can.handle.Init.AutoWakeUp = DISABLE;
  setRxFIFOLock(false);
  setTxBufferMode(TX_BUFFER_MODE::FIFO);
  setMode(MODE::NORMAL);
  setAutoRetransmission(true);
}

CAN_TypeDef *tNMEA2000_STM32::getPeripheral()
{
  CAN_TypeDef *canPort_rx = (CAN_TypeDef *)pinmap_peripheral(rx, PinMap_CAN_RD);
  CAN_TypeDef *canPort_tx = (CAN_TypeDef *)pinmap_peripheral(tx, PinMap_CAN_TD);
  if ((canPort_rx != canPort_tx && canPort_tx != NP) || canPort_rx == NP)
  {
    // ensure pins relate to same peripheral OR only Rx is set/valid
    //  rx only can be used as listen only but needs a 3rd node for valid ACKs

    // do not allow Tx only since that would break arbitration
    return NP;
  }

#ifdef STM32F1xx
  /** AF remapping on the F1 platform only possible in pairs
   *  Verify that both pins use the same remapping
   *  Only enforced if both pins are used, in Rx only Tx pin is not set to AF*/
  if (canPort_rx != NP && canPort_tx != NP)
  {
    uint32_t rx_func = pinmap_function(rx, PinMap_CAN_RD);
    uint32_t tx_func = pinmap_function(tx, PinMap_CAN_TD);
    uint32_t rx_afnum = STM_PIN_AFNUM(rx_func);
    uint32_t tx_afnum = STM_PIN_AFNUM(tx_func);
    if (rx_afnum != tx_afnum)
    {
      // ERROR
      return NP;
    }
  }
#endif

  // clear tx pin in case it was set but does not match a peripheral
  if (canPort_tx == NP)
    tx = NC;

  return canPort_rx;
}

/**-------------------------------------------------------------
 *     setup functions
 *     no effect after begin()
 * -------------------------------------------------------------
 */
void tNMEA2000_STM32::setIRQPriority(uint32_t preemptPriority, uint32_t subPriority)
{
  // NOTE: limiting the IRQ prio, but not accounting for group setting
  this->preemptPriority = min(preemptPriority, MAX_IRQ_PRIO_VALUE);
  this->subPriority = min(subPriority, MAX_IRQ_PRIO_VALUE);
}

void tNMEA2000_STM32::setAutoRetransmission(bool enabled)
{
  _can.handle.Init.AutoRetransmission = enabled ? (ENABLE) : (DISABLE);
}

void tNMEA2000_STM32::setRxFIFOLock(bool fifo0locked, bool fifo1locked)
{
  (void)fifo1locked;
  _can.handle.Init.ReceiveFifoLocked = fifo0locked ? (ENABLE) : (DISABLE);
}

void tNMEA2000_STM32::setTxBufferMode(TX_BUFFER_MODE mode)
{
  _can.handle.Init.TransmitFifoPriority = (FunctionalState)mode;
}

void tNMEA2000_STM32::setTimestampCounter(bool enabled)
{
  _can.handle.Init.TimeTriggeredMode = enabled ? (ENABLE) : (DISABLE);
}

void tNMEA2000_STM32::setMode(MODE mode)
{
  _can.handle.Init.Mode = mode;
}

void tNMEA2000_STM32::setAutoBusOffRecovery(bool enabled)
{
  _can.handle.Init.AutoBusOff = enabled ? (ENABLE) : (DISABLE);
}

/**-------------------------------------------------------------
 *     lifecycle functions
 *     setBaudRate may be called before or after begin
 * -------------------------------------------------------------
 */
// Init and start CAN
void tNMEA2000_STM32::begin()
{

  // exit if CAN already is active
  if (_canIsActive)
    return;

  auto instance = getPeripheral();
  if (instance == NP)
  {
    // impossible pinconfig, done here
    return;
  }
  if (!allocatePeripheral(instance))
  {
    // peripheral already in use
    return;
  }
  _canIsActive = true;

  /**
   * NOTE: enabling the internal pullup of RX pin
   * in case no external circuitry (CAN Transceiver) is connected this will ensure a valid recessive level.
   * A valid recessive level on RX is needed to leave init mode and enter normal mode.
   * This is even the case if loopback is enabled where the input is internally connected.
   * This lets loopback only tests without external circuitry sill function.
   */
  uint32_t rx_func = pinmap_function(rx, PinMap_CAN_RD);
  rx_func = (rx_func & ~(STM_PIN_PUPD_MASK << STM_PIN_PUPD_SHIFT)) | (GPIO_PULLUP << STM_PIN_PUPD_SHIFT);
  pin_function(rx, fixPinFunction(rx_func));
  if (tx != NC)
  {
    pin_function(tx, fixPinFunction(pinmap_function(tx, PinMap_CAN_TD)));
  }

  // Configure CAN
  if (_can.handle.Instance == CAN1)
  {
    // CAN1
    __HAL_RCC_CAN1_CLK_ENABLE();

#ifdef CAN1_IRQn_AIO
    // NVIC configuration for CAN1 common interrupt
    HAL_NVIC_SetPriority(CAN1_IRQn_AIO, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_IRQn_AIO);
#else
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
    /** CAN Tx and Rx0 blocked by USB, only using Rx1 */
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
#else
    // NVIC configuration for CAN1 Reception complete interrupt
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    // NVIC configuration for CAN1 Transmission complete interrupt
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
#endif
#endif /** else defined(CAN1_IRQn_AIO) */

    _can.bus = 1;
  }
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    // CAN2
    __HAL_RCC_CAN1_CLK_ENABLE(); // CAN1 clock needs to be enabled too, because CAN2 works as CAN1 slave.
    __HAL_RCC_CAN2_CLK_ENABLE();

    // NVIC configuration for CAN2 Reception complete interrupt
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    // NVIC configuration for CAN2 Transmission complete interrupt
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);

    _can.bus = 2;
  }
#endif

#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
  {
    // CAN3
    __HAL_RCC_CAN3_CLK_ENABLE();

    // NVIC configuration for CAN3 Reception complete interrupt
    HAL_NVIC_SetPriority(CAN3_RX0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
    // NVIC configuration for CAN3 Transmission complete interrupt
    HAL_NVIC_SetPriority(CAN3_TX_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);

    _can.bus = 3;
  }
#endif

  setAutoRetransmission(true);

  filtersInitialized = false;

  // try to start in case baudrate was set earlier
  setBaudRate(baudrate);
}

void tNMEA2000_STM32::end()
{
  if (!hasPeripheral())
  {
    return;
  }

  stop();

  disableMBInterrupts();

  if (_can.handle.Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();
  }
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    __HAL_RCC_CAN2_CLK_DISABLE();
    // only disable CAN1 clock if its not used
    if (canObj[CAN1_INDEX] == nullptr)
    {
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  }
#endif
#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
  {
    __HAL_RCC_CAN3_CLK_DISABLE();
  }
#endif

  /** un-init pins, enable tx PULLUP for weak driving of recessive state */
  pin_function(rx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, GPIO_AF_NONE));
  if (tx != NC)
    pin_function(tx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, GPIO_AF_NONE));

  // freeBuffers();

  freePeripheral();

  _canIsActive = false;
}

void tNMEA2000_STM32::setBaudRate(uint32_t baud)
{
  baudrate = baud;

  if (!hasPeripheral())
  {
    return;
  }

  // Calculate and set baudrate
  if (!calculateBaudrate(baud))
  {
    return;
  }

  // (re)-start
  stop();
  start();
}

void tNMEA2000_STM32::start()
{
  // Initializes CAN
  HAL_CAN_Init(&_can.handle);

  _can.handle.Instance->MCR &= ~CAN_MCR_NART;

  initializeFilters();

  // Start the CAN peripheral
  HAL_CAN_Start(&_can.handle);

// Activate CAN notifications
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  HAL_CAN_ActivateNotification(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
#else
  HAL_CAN_ActivateNotification(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
#endif
}

void tNMEA2000_STM32::stop()
{
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  HAL_CAN_DeactivateNotification(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
#else
  HAL_CAN_DeactivateNotification(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_DeactivateNotification(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
#endif

  /** Calls Stop internally, clears all errors */
  HAL_CAN_DeInit(&_can.handle);
}

/**-------------------------------------------------------------
 *     post begin(), setup filters, data transfer
 * -------------------------------------------------------------
 */

uint8_t tNMEA2000_STM32::getFilterBankCount(IDE std_ext)
{
  (void)std_ext;
  if (_can.handle.Instance == nullptr)
    return 0;
#ifdef CAN2
  if (_can.handle.Instance == CAN1)
  {
    return STM32_CAN_CAN2_FILTER_OFFSET;
  }
  if (_can.handle.Instance == CAN2)
  {
    return STM32_CAN_DUAL_CAN_FILTER_COUNT - STM32_CAN_CAN2_FILTER_OFFSET;
  }
#endif
  return STM32_CAN_SINGLE_CAN_FILTER_COUNT;
}

static uint32_t format32bitFilter(uint32_t id, IDE std_ext, bool mask)
{
  uint32_t id_reg;
  if (std_ext == AUTO)
  {
    std_ext = (id <= 0x7FF) ? STD : EXT;
  }
  if (std_ext == STD)
  {
    id <<= 18;
  }
  id_reg = id << 3;
  // set IDE bit
  if (mask || std_ext == EXT)
  {
    id_reg |= (1 << 2);
  }
  return id_reg;
}

static uint32_t format16bitFilter(uint32_t id, IDE std_ext, bool mask)
{
  uint32_t id_reg;
  if (std_ext == AUTO)
  {
    std_ext = (id <= 0x7FF) ? STD : EXT;
  }
  if (std_ext == STD)
  {
    id <<= 18;
  }
  // set STID
  id_reg = (id >> (18 - 5)) & 0xFFE0UL;
  // set EXTI [17:15]
  id_reg |= (id >> (18 - 3)) & 0x003UL;
  // set IDE bit
  if (mask || std_ext == EXT)
  {
    id_reg |= (1 << 3);
  }
  return id_reg;
}

bool tNMEA2000_STM32::setFilter(uint8_t bank_num, bool enabled, FILTER_ACTION action)
{
  CAN_TypeDef *can_ip = _can.handle.Instance;
  if (!_can.handle.Instance)
    return false;
/** CAN2 shares filter banks with CAN1
 * Driver allocates equal amount to each
 * Filter Banks located at CAN1 base address
 */
#ifdef CAN2
  if (_can.handle.Instance == CAN2)
  {
    can_ip = CAN1;
    bank_num += STM32_CAN_CAN2_FILTER_OFFSET;
  }
#endif

  uint32_t filternbrbitpos = (uint32_t)1 << (bank_num & 0x1FU);

  /* Initialisation mode for the filter */
  SET_BIT(can_ip->FMR, CAN_FMR_FINIT);

  /* Filter Deactivation */
  CLEAR_BIT(can_ip->FA1R, filternbrbitpos);

  /* Filter FIFO assignment */
  switch (action)
  {
  case FILTER_ACTION::STORE_FIFO0:
    CLEAR_BIT(can_ip->FFA1R, filternbrbitpos);
    break;
  case FILTER_ACTION::STORE_FIFO1:
    SET_BIT(can_ip->FFA1R, filternbrbitpos);
    break;
  }

  /* Filter activation */
  if (enabled)
  {
    SET_BIT(can_ip->FA1R, filternbrbitpos);
  }
  /* Leave the initialisation mode for the filter */
  CLEAR_BIT(can_ip->FMR, CAN_FMR_FINIT);
  return true;
}

bool tNMEA2000_STM32::setFilter(uint8_t bank_num, uint32_t filter_id, uint32_t mask, IDE std_ext, uint32_t filter_mode, uint32_t filter_scale, uint32_t fifo)
{
  /** NOTE: legacy, this function only implemented 32 bit scaling mode in mask mode, other modes will be broken*/
  if (filter_scale != CAN_FILTERSCALE_32BIT)
  {
    core_debug("WARNING: legacy function only implements 32 bit filter scale. Filter will be broken!\n");
  }
  if (filter_scale != CAN_FILTERMODE_IDMASK)
  {
    core_debug("WARNING: legacy function only implements ID Mask mode. Filter will be broken!\n");
  }
  /** re-implement broken implementation for legacy behaviour */
  uint32_t id_reg = format32bitFilter(filter_id, std_ext, false);
  uint32_t mask_reg = format32bitFilter(mask, std_ext, true);
  FILTER_ACTION action = (fifo == CAN_FILTER_FIFO0) ? FILTER_ACTION::STORE_FIFO0 : FILTER_ACTION::STORE_FIFO1;
  return !setFilterRaw(bank_num, id_reg, mask_reg, filter_mode, filter_scale, action);
}

bool tNMEA2000_STM32::setFilterRaw(uint8_t bank_num, uint32_t id, uint32_t mask, uint32_t filter_mode, uint32_t filter_scale, FILTER_ACTION action, bool enabled)
{
  CAN_FilterTypeDef sFilterConfig;
  if (!_can.handle.Instance)
    return false;

  sFilterConfig.FilterBank = bank_num;
  sFilterConfig.FilterMode = filter_mode;
  sFilterConfig.FilterScale = filter_scale;
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  if (action == FILTER_ACTION::STORE_FIFO0)
  {
    core_debug("WARNING: RX0 IRQ is blocked by USB Driver. Events only handled by polling and RX1 events!\n");
  }
#endif
  switch (action)
  {
  case FILTER_ACTION::STORE_FIFO0:
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    break;
  case FILTER_ACTION::STORE_FIFO1:
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    break;
  }
  sFilterConfig.FilterActivation = enabled ? ENABLE : DISABLE;

  sFilterConfig.FilterIdLow = id & 0xFFFFUL;
  sFilterConfig.FilterIdHigh = id >> 16;
  sFilterConfig.FilterMaskIdLow = mask & 0xFFFFUL;
  sFilterConfig.FilterMaskIdHigh = mask >> 16;

#ifdef CAN2
  sFilterConfig.SlaveStartFilterBank = STM32_CAN_CAN2_FILTER_OFFSET;
  if (_can.handle.Instance == CAN2)
  {
    sFilterConfig.FilterBank += STM32_CAN_CAN2_FILTER_OFFSET;
    if (sFilterConfig.FilterBank >= STM32_CAN_DUAL_CAN_FILTER_COUNT)
      return false;
  }
  else
#endif
      if (sFilterConfig.FilterBank >= STM32_CAN_SINGLE_CAN_FILTER_COUNT)
  {
    return false;
  }
  // Enable filter
  return (HAL_CAN_ConfigFilter(&_can.handle, &sFilterConfig) == HAL_OK);
}

void tNMEA2000_STM32::initializeFilters()
{
  if (filtersInitialized)
    return;
  filtersInitialized = true;

  /** Let everything in by default */
  setFilterRaw(0, 0UL, 0UL, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT,
               FILTER_ACTION::CAN_FILTER_DEFAULT_ACTION, true);

  /** turn off all other filters that might sill be setup from before */
  for (uint8_t bank_num = 1; bank_num < STM32_CAN_SINGLE_CAN_FILTER_COUNT; bank_num++)
  {
    setFilter(bank_num, false);
  }
}

bool tNMEA2000_STM32::addToRingBuffer(const CAN_message_t &msg)
{
  uint8_t prio;
  uint32_t id;
  CAN_message_t *rxMsg;

  id = msg.id;
  prio = (uint8_t)((id >> 26) & 0x7);

  rxMsg = rxRing1->getAddRef(prio);

  // Nachricht hineinkopieren
  if (rxMsg != nullptr)
  {
    *rxMsg = msg; // ← WICHTIG!
    return true;
  }

  return (true);
}

void tNMEA2000_STM32::setBaudRateValues(uint16_t prescaler, uint8_t timeseg1,
                                        uint8_t timeseg2, uint8_t sjw)
{
  uint32_t _SyncJumpWidth = 0;
  uint32_t _TimeSeg1 = 0;
  uint32_t _TimeSeg2 = 0;
  uint32_t _Prescaler = 0;

  /* the CAN specification (v2.0) states that SJW shall be programmable between
   * 1 and min(4, timeseg1)... the bxCAN documentation doesn't mention this
   */
  if (sjw > 4)
    sjw = 4;
  if (sjw > timeseg1)
    sjw = timeseg1;

  switch (sjw)
  {
  case 0:
  case 1:
    _SyncJumpWidth = CAN_SJW_1TQ;
    break;
  case 2:
    _SyncJumpWidth = CAN_SJW_2TQ;
    break;
  case 3:
    _SyncJumpWidth = CAN_SJW_3TQ;
    break;
  case 4:
  default: /* limit to 4 */
    _SyncJumpWidth = CAN_SJW_4TQ;
    break;
  }

  switch (timeseg1)
  {
  case 1:
    _TimeSeg1 = CAN_BS1_1TQ;
    break;
  case 2:
    _TimeSeg1 = CAN_BS1_2TQ;
    break;
  case 3:
    _TimeSeg1 = CAN_BS1_3TQ;
    break;
  case 4:
    _TimeSeg1 = CAN_BS1_4TQ;
    break;
  case 5:
    _TimeSeg1 = CAN_BS1_5TQ;
    break;
  case 6:
    _TimeSeg1 = CAN_BS1_6TQ;
    break;
  case 7:
    _TimeSeg1 = CAN_BS1_7TQ;
    break;
  case 8:
    _TimeSeg1 = CAN_BS1_8TQ;
    break;
  case 9:
    _TimeSeg1 = CAN_BS1_9TQ;
    break;
  case 10:
    _TimeSeg1 = CAN_BS1_10TQ;
    break;
  case 11:
    _TimeSeg1 = CAN_BS1_11TQ;
    break;
  case 12:
    _TimeSeg1 = CAN_BS1_12TQ;
    break;
  case 13:
    _TimeSeg1 = CAN_BS1_13TQ;
    break;
  case 14:
    _TimeSeg1 = CAN_BS1_14TQ;
    break;
  case 15:
    _TimeSeg1 = CAN_BS1_15TQ;
    break;
  case 16:
    _TimeSeg1 = CAN_BS1_16TQ;
    break;
  default:
    // should not happen
    _TimeSeg1 = CAN_BS1_1TQ;
    break;
  }

  switch (timeseg2)
  {
  case 1:
    _TimeSeg2 = CAN_BS2_1TQ;
    break;
  case 2:
    _TimeSeg2 = CAN_BS2_2TQ;
    break;
  case 3:
    _TimeSeg2 = CAN_BS2_3TQ;
    break;
  case 4:
    _TimeSeg2 = CAN_BS2_4TQ;
    break;
  case 5:
    _TimeSeg2 = CAN_BS2_5TQ;
    break;
  case 6:
    _TimeSeg2 = CAN_BS2_6TQ;
    break;
  case 7:
    _TimeSeg2 = CAN_BS2_7TQ;
    break;
  case 8:
    _TimeSeg2 = CAN_BS2_8TQ;
    break;
  default:
    // should not happen
    _TimeSeg2 = CAN_BS2_1TQ;
    break;
  }
  _Prescaler = prescaler;

  _can.handle.Init.SyncJumpWidth = _SyncJumpWidth;
  _can.handle.Init.TimeSeg1 = _TimeSeg1;
  _can.handle.Init.TimeSeg2 = _TimeSeg2;
  _can.handle.Init.Prescaler = _Prescaler;
}

template <typename T, size_t N>
bool tNMEA2000_STM32::lookupBaudrate(int baud, const T (&table)[N])
{
  for (size_t i = 0; i < N; i++)
  {
    if (baud != (int)table[i].baudrate)
    {
      continue;
    }

    /* for the best chance at interoperability, use the widest SJW possible */
    setBaudRateValues(table[i].prescaler, table[i].timeseg1, table[i].timeseg2, 4);
    return true;
  }

  return false;
}

bool tNMEA2000_STM32::calculateBaudrate(int baud)
{
  uint8_t bs1;
  uint8_t bs2;
  uint16_t prescaler;

  const uint32_t frequency = getCanPeripheralClock();

  if (frequency == 48000000)
  {
    if (lookupBaudrate(baud, BAUD_RATE_TABLE_48M))
      return true;
  }
  else if (frequency == 45000000)
  {
    if (lookupBaudrate(baud, BAUD_RATE_TABLE_45M))
      return true;
  }
  else if (frequency == 42000000)
  {
    if (lookupBaudrate(baud, BAUD_RATE_TABLE_42M))
      return true;
  }
  else if (frequency == 36000000)
  {
    if (lookupBaudrate(baud, BAUD_RATE_TABLE_36M))
      return true;
  }

  /* this loop seeks a precise baudrate match, with the sample point positioned
   * at between ~75-95%. the nominal bit time is produced from N time quanta,
   * running at the prescaled clock rate (where N = 1 + bs1 + bs2). this algorithm
   * prefers the lowest prescaler (most time quanter per bit).
   *
   * many configuration sets can be discarded due to an out-of-bounds sample point,
   * or being unable to reach the desired baudrate.
   *
   * for the best chance at interoperability, we use the widest SJW possible.
   *
   * for more details + justification, see: https://github.com/pazi88/STM32_CAN/pull/41
   */
  for (prescaler = 1; prescaler <= 1024; prescaler += 1)
  {
    const uint32_t can_freq = frequency / prescaler;
    const uint32_t baud_min = can_freq / (1 + 5 + 16);

    /* skip all prescaler values that can't possibly achieve the desired baudrate */
    if (baud_min > baud)
      continue;

    for (bs2 = 1; bs2 <= 5; bs2 += 1)
    {
      for (bs1 = (bs2 * 3) - 1; bs1 <= 16; bs1 += 1)
      {
        const uint32_t baud_cur = can_freq / (1 + bs1 + bs2);

        if (baud_cur != baud)
          continue;

        setBaudRateValues(prescaler, bs1, bs2, 4);
        return true;
      }
    }
  }

  /* uhoh, failed to calculate an acceptable baud rate... */
  return false;
}

uint32_t tNMEA2000_STM32::getCanPeripheralClock()
{
  // All bxCAN get clocked by APB1 / PCLK1
  return HAL_RCC_GetPCLK1Freq();
}

uint32_t tNMEA2000_STM32::fixPinFunction(uint32_t function)
{
#ifdef STM32F1xx
  /**
   * NOTE: F103 pinmaps defines AFIO_NONE for first (default) pinmaping.
   * should be AFIO_CAN1_1.
   */
  uint32_t af = STM_PIN_AFNUM(function);
  if (af == AFIO_NONE)
  {
    af = AFIO_CAN1_1;
  }

  function &= ~(STM_PIN_AFNUM_MASK << STM_PIN_AFNUM_SHIFT);
  function |= ((af & STM_PIN_AFNUM_MASK) << STM_PIN_AFNUM_SHIFT);
#endif
  return function;
}

void tNMEA2000_STM32::enableMBInterrupts()
{
  if (_can.handle.Instance == CAN1)
  {
#ifdef CAN1_IRQn_AIO
    HAL_NVIC_EnableIRQ(CAN1_IRQn_AIO);
#else
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
#else
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
#endif
#endif /** else defined(CAN1_IRQn_AIO) */
  }
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  }
#endif
#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
  {
    HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
  }
#endif
}

void tNMEA2000_STM32::disableMBInterrupts()
{
  if (_can.handle.Instance == CAN1)
  {
#ifdef CAN1_IRQn_AIO
#if defined(HAL_CEC_MODULE_ENABLED) && defined(STM32_CAN1_SHARED_WITH_CEC)
    // only disable if cec instance is not set/used
    if (!phcec)
#endif
    {
      HAL_NVIC_DisableIRQ(CAN1_IRQn_AIO);
    }
#else
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
#else
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
#endif
#endif /** else defined(CAN1_IRQn_AIO) */
  }
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  }
#endif
#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
  {
    HAL_NVIC_DisableIRQ(CAN3_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN3_RX0_IRQn);
  }
#endif
}

void tNMEA2000_STM32::enableFIFO(bool status)
{
  // Nothing to do here. The FIFO is on by default. This is just to work with code made for Teensy FlexCan.
  (void)status;
}

/* Interrupt functions
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
// There is 3 TX mailboxes. Each one has own transmit complete callback function, that we use to pull next message from TX ringbuffer to be sent out in TX mailbox.
extern "C" void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *CanHandle)
{
  stm32_can_t *canObj = get_can_obj(CanHandle);
  tNMEA2000_STM32 *_can = (tNMEA2000_STM32 *)canObj->__this;
  CAN_message_t txmsg;

  _can->sendFromTxRing();
}

extern "C" void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *CanHandle)
{
  stm32_can_t *canObj = get_can_obj(CanHandle);
  tNMEA2000_STM32 *_can = (tNMEA2000_STM32 *)canObj->__this;
  CAN_message_t txmsg;

  _can->sendFromTxRing();
}

extern "C" void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *CanHandle)
{
  stm32_can_t *canObj = get_can_obj(CanHandle);
  tNMEA2000_STM32 *_can = (tNMEA2000_STM32 *)canObj->__this;
  CAN_message_t txmsg;

  _can->sendFromTxRing();
}

// This is called by RX0_IRQHandler when there is message at RX FIFO0 buffer
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
#else
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
#endif
{
  stm32_can_t *canObj = get_can_obj(CanHandle);
  tNMEA2000_STM32 *_can = (tNMEA2000_STM32 *)canObj->__this;
  CAN_message_t rxmsg;
  CAN_message_t *rxMsg;
  CAN_RxHeaderTypeDef RxHeader;
// bool state = Disable_Interrupts();
// HAL_CAN_DeactivateNotification(CanHandle, CAN_IT_TX_MAILBOX_EMPTY);
// move the message from RX FIFO0 to RX ringbuffer
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  const uint32_t fifo = CAN_RX_FIFO1;
#else
  const uint32_t fifo = CAN_RX_FIFO0;
#endif
  do
  {
    if (HAL_CAN_GetRxMessage(CanHandle, fifo, &RxHeader, rxmsg.buf) == HAL_OK)
    {
      if (RxHeader.IDE == CAN_ID_STD)
      {
        rxmsg.id = RxHeader.StdId;
        rxmsg.flags.extended = 0;
      }
      else
      {
        rxmsg.id = RxHeader.ExtId;
        rxmsg.flags.extended = 1;
      }

      rxmsg.flags.remote = RxHeader.RTR;
      rxmsg.mb = RxHeader.FilterMatchIndex;
      rxmsg.timestamp = RxHeader.Timestamp;
      rxmsg.len = RxHeader.DLC;

      rxmsg.bus = canObj->bus;
      _can->addToRingBuffer(rxmsg);
    }

  } while (HAL_CAN_GetRxFifoFillLevel(CanHandle, fifo));
  // Enable_Interrupts(state);
  // HAL_CAN_ActivateNotification(CanHandle, CAN_IT_TX_MAILBOX_EMPTY);
}

#ifdef CAN1_IRQHandler_AIO

extern "C" void CAN1_IRQHandler_AIO(void)
{
  if (canObj[CAN1_INDEX])
  {
    HAL_CAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
  }
#if defined(HAL_CEC_MODULE_ENABLED) && defined(STM32_CAN1_SHARED_WITH_CEC)
  if (phcec)
  {
    HAL_CEC_IRQHandler(phcec);
  }
#endif
}

#else

// RX IRQ handlers
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
/** If USB blocks TX and RX0 IRQs, will use RX1 by default*/
extern "C" void CAN1_RX1_IRQHandler(void)
#else
extern "C" void CAN1_RX0_IRQHandler(void)
#endif
{
  if (canObj[CAN1_INDEX])
  {
    HAL_CAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
  }
}

#ifdef CAN2
extern "C" void CAN2_RX0_IRQHandler(void)
{
  if (canObj[CAN2_INDEX])
  {
    HAL_CAN_IRQHandler(&canObj[CAN2_INDEX]->handle);
  }
}
#endif
#ifdef CAN3
extern "C" void CAN3_RX0_IRQHandler(void)
{
  if (canObj[CAN3_INDEX])
  {
    HAL_CAN_IRQHandler(&canObj[CAN3_INDEX]->handle);
  }
}
#endif

// TX IRQ handlers
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
/** If USB blocks TX and RX0 IRQs, need to poll for Tx events*/
extern "C" void STM32_CAN_Poll_IRQ_Handler(void)
#else
extern "C" void CAN1_TX_IRQHandler(void)
#endif
{
  if (canObj[CAN1_INDEX])
  {
    HAL_CAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
  }
}

#ifdef CAN2
extern "C" void CAN2_TX_IRQHandler(void)
{
  if (canObj[CAN2_INDEX])
  {
    HAL_CAN_IRQHandler(&canObj[CAN2_INDEX]->handle);
  }
}
#endif
#ifdef CAN3
extern "C" void CAN3_TX_IRQHandler(void)
{
  if (canObj[CAN3_INDEX])
  {
    HAL_CAN_IRQHandler(&canObj[CAN3_INDEX]->handle);
  }
}
#endif

#endif /* CAN1_IRQHandler_AIO */
