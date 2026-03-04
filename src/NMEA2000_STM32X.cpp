#include "NMEA2000_STM32X.h"
#include "core_debug.h"

// ===========================================================================
// Common helper function
// ===========================================================================
extern void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn,
                       unsigned char &src, unsigned char &dst);

// ===========================================================================
//  bxCAN – specific code
// ===========================================================================
#if defined(STM32X_USE_BXCAN)

#if defined(HAL_CEC_MODULE_ENABLED) && defined(STM32_CAN1_SHARED_WITH_CEC)
extern CEC_HandleTypeDef *phcec;
#endif

#define STM32_CAN_CAN2_FILTER_OFFSET 14
#define MAX_IRQ_PRIO_VALUE ((1UL << __NVIC_PRIO_BITS) - 1UL)

constexpr Baudrate_entry_t tNMEA2000_STM32X::BAUD_RATE_TABLE_48M[];
constexpr Baudrate_entry_t tNMEA2000_STM32X::BAUD_RATE_TABLE_45M[];
constexpr Baudrate_entry_t tNMEA2000_STM32X::BAUD_RATE_TABLE_42M[];
constexpr Baudrate_entry_t tNMEA2000_STM32X::BAUD_RATE_TABLE_36M[];

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

static stm32_can_t *get_can_obj(CAN_HandleTypeDef *hcan)
{
  return (stm32_can_t *)((char *)hcan - offsetof(stm32_can_t, handle));
}

static can_index_t get_can_index(CAN_TypeDef *instance)
{
  can_index_t index = CAN_UNKNOWN;
#if defined(CAN1)
  if (instance == CAN1)
    index = CAN1_INDEX;
#endif
#if defined(CAN2)
  if (instance == CAN2)
    index = CAN2_INDEX;
#endif
#if defined(CAN3)
  if (instance == CAN3)
    index = CAN3_INDEX;
#endif
  if (index == CAN_UNKNOWN)
    Error_Handler();
  return index;
}

bool tNMEA2000_STM32X::allocatePeripheral(CAN_TypeDef *instance)
{
  can_index_t index = get_can_index(instance);
  if (index >= CAN_NUM)
    return false;
  if (canObj[index])
  {
    Error_Handler();
    return false;
  }
  _can.handle.Instance = instance;
  canObj[index] = &_can;
  return true;
}

bool tNMEA2000_STM32X::freePeripheral()
{
  if (_can.handle.Instance == nullptr)
    return false;
  can_index_t index = get_can_index(_can.handle.Instance);
  if (index >= CAN_NUM)
    return false;
  if (canObj[index] == &_can)
  {
    canObj[index] = nullptr;
    _can.handle.Instance = nullptr;
    return true;
  }
  Error_Handler();
  return false;
}

bool tNMEA2000_STM32X::hasPeripheral()
{
  if (_can.handle.Instance == nullptr)
    return false;
  can_index_t index = get_can_index(_can.handle.Instance);
  if (index >= CAN_NUM)
    return false;
  return canObj[index] == &_can;
}

// --- Constructors (bxCAN) ---
tNMEA2000_STM32X::tNMEA2000_STM32X(uint32_t rx, uint32_t tx)
    : rxRing1(0), txRing1(0),
      preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0),
      IsOpen(false), tNMEA2000()
{
  this->rx = digitalPinToPinName(rx);
  this->tx = digitalPinToPinName(tx);
  init();
}

tNMEA2000_STM32X::tNMEA2000_STM32X(CAN_TypeDef *canPort)
    : rxRing1(0), txRing1(0),
      preemptPriority(MAX_IRQ_PRIO_VALUE), subPriority(0),
      IsOpen(false), tNMEA2000()
{
  rx = pinmap_find_pin(canPort, PinMap_CAN_RD);
  tx = pinmap_find_pin(canPort, PinMap_CAN_TD);
  init();
}

#endif // STM32X_USE_BXCAN

// ===========================================================================
//  FDCAN – specific code (static instance + global pointer)
// ===========================================================================
#if defined(STM32X_USE_FDCAN)

FDCAN_HandleTypeDef tNMEA2000_STM32X::hcan_ = {};
static tNMEA2000_STM32X *g_canInstance = nullptr;

#endif // STM32X_USE_FDCAN

// ===========================================================================
//  Common constructor (PinName) and destructor
// ===========================================================================
tNMEA2000_STM32X::tNMEA2000_STM32X(PinName rx, PinName tx)
    : rx(rx), tx(tx), rxRing1(nullptr), txRing1(nullptr),
      IsOpen(false), tNMEA2000()
{
#if defined(STM32X_USE_BXCAN)
  preemptPriority = MAX_IRQ_PRIO_VALUE;
  subPriority = 0;
#endif
  init();
}

tNMEA2000_STM32X::~tNMEA2000_STM32X()
{
  end();
}

// ===========================================================================
//  tNMEA2000 overrides – shared
// ===========================================================================

bool tNMEA2000_STM32X::CANOpen()
{
  if (IsOpen)
    return true;
  IsOpen = true;
  begin();
  return IsOpen;
}

void tNMEA2000_STM32X::InitCANFrameBuffers()
{

  if (MaxCANReceiveFrames == 0)
    MaxCANReceiveFrames = 60;
  if (MaxCANReceiveFrames < 30)
    MaxCANReceiveFrames = 30;
  if (MaxCANSendFrames == 0)
    MaxCANSendFrames = 80;
  if (MaxCANSendFrames < 50)
    MaxCANSendFrames = 50;

  if (rxRing1 != nullptr && rxRing1->getSize() != MaxCANReceiveFrames)
  {
    delete rxRing1;
    rxRing1 = nullptr;
  }
  if (txRing1 != nullptr && txRing1->getSize() != MaxCANSendFrames)
  {
    delete txRing1;
    txRing1 = nullptr;
  }

  if (rxRing1 == nullptr)
    rxRing1 = new tPriorityRingBuffer<CAN_message_t>(MaxCANReceiveFrames, 8);
  if (txRing1 == nullptr)
    txRing1 = new tPriorityRingBuffer<CAN_message_t>(MaxCANSendFrames, 8);
}

// ---------------------------------------------------------------------------
bool tNMEA2000_STM32X::CANSendFrame(unsigned long id, unsigned char len,
                                    const unsigned char *buf, bool /*wait_sent*/)
{
  uint8_t prio = (id >> 26) & 0x7;
  uint8_t dlc = (len > 8) ? 8 : len;

#if defined(STM32X_USE_FDCAN)
  __HAL_FDCAN_DISABLE_IT(&hcan_, FDCAN_IT_TX_COMPLETE);
  __HAL_FDCAN_DISABLE_IT(&hcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  bool mailboxBusy = (HAL_FDCAN_GetTxFifoFreeLevel(&hcan_) == 0);

  FDCAN_TxHeaderTypeDef TxHeader = {
      .Identifier = id & 0x1FFFFFFFU,
      .IdType = FDCAN_EXTENDED_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = lengthToDLC(dlc),
      .FDFormat = FDCAN_CLASSIC_CAN,
      .MessageMarker = 0};
#else // bxCAN
  if (!_can.handle.Instance)
    return false;
#if !defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB)
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif
  bool mailboxBusy = (HAL_CAN_GetTxMailboxesFreeLevel(&_can.handle) == 0);
  uint32_t TxMailbox;

  CAN_TxHeaderTypeDef TxHeader = {
      .ExtId = id,
      .IDE = CAN_ID_EXT,
      .RTR = CAN_RTR_DATA,
      .DLC = dlc,
      .TransmitGlobalTime = DISABLE};
#endif

#if defined(STMCANDEBUG)
  {
    unsigned long pgn;
    unsigned char src, dst;
    CanIdToN2k(id, prio, pgn, src, dst);
    Serial.printf("CANSendFrame pgn:%6lu, prio:%u, src:%u, dst:%u, data:%02X %02X\n",
                  pgn, prio, src, dst, buf[0], buf[1]);
  }
#endif

  bool sendFromBuffer = false;
  if (!txRing1->isEmpty() || mailboxBusy)
  {
    CAN_message_t *msg = txRing1->getAddRef(prio);
    if (msg)
    {
      msg->id = id;
      msg->len = dlc;
      memcpy(msg->buf, buf, dlc);
    }
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
#if defined(STM32X_USE_FDCAN)
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hcan_, &TxHeader, buf) != HAL_OK)
        result = false;
#else
      if (HAL_CAN_AddTxMessage(&_can.handle, &TxHeader, buf, &TxMailbox) != HAL_OK)
        result = false;
#endif
    }
  }

#if defined(STM32X_USE_FDCAN)
  __HAL_FDCAN_ENABLE_IT(&hcan_, FDCAN_IT_TX_COMPLETE);
  __HAL_FDCAN_ENABLE_IT(&hcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
#else
#if !defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB)
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif
#endif

  return result;
}

// ---------------------------------------------------------------------------
bool tNMEA2000_STM32X::sendFromTxRing()
{
  const CAN_message_t *txMsg = txRing1->getReadRef();
  if (txMsg == nullptr)
    return false;

#if defined(STMCANDEBUG)
  {
    unsigned long pgn;
    unsigned char prio, src, dst;
    CanIdToN2k(txMsg->id, prio, pgn, src, dst);
    Serial.printf("sendFromTxRing pgn:%6lu, prio:%u, src:%u, dst:%u, data:%02X\n",
                  pgn, prio, src, dst, txMsg->buf[0]);
  }
#endif

#if defined(STM32X_USE_FDCAN)
  FDCAN_TxHeaderTypeDef TxHeader = {
      .Identifier = txMsg->id & 0x1FFFFFFFU,
      .IdType = FDCAN_EXTENDED_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = lengthToDLC(txMsg->len),
      .FDFormat = FDCAN_CLASSIC_CAN,
      .MessageMarker = 0};
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hcan_, &TxHeader, txMsg->buf) != HAL_OK)
  {
#if defined(STMCANDEBUG)
    Serial.println("sendFromTxRing failed");
#endif
  }
#else // bxCAN
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.ExtId = txMsg->id;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = txMsg->len;
  TxHeader.TransmitGlobalTime = DISABLE;
  uint32_t TxMailbox;
  if (HAL_CAN_AddTxMessage(&_can.handle, &TxHeader, txMsg->buf, &TxMailbox) != HAL_OK)
  {
#if defined(STMCANDEBUG)
    Serial.println("sendFromTxRing failed");
#endif
  }
#endif

  return true;
}

// ---------------------------------------------------------------------------
bool tNMEA2000_STM32X::CANGetFrame(unsigned long &id, unsigned char &len,
                                   unsigned char *buf)
{
#if defined(STM32X_USE_FDCAN)
  __HAL_FDCAN_DISABLE_IT(&hcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
#elif defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
#else
  __HAL_CAN_DISABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif

  bool ret = false;
  const CAN_message_t *msg = rxRing1->getReadRef();
  if (msg != nullptr)
  {
    id = msg->id;
    len = msg->len;
    if (len > 8)
      len = 8;
    memcpy(buf, msg->buf, len);
    ret = true;
  }

#if defined(STM32X_USE_FDCAN)
  __HAL_FDCAN_ENABLE_IT(&hcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
#elif defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
#else
  __HAL_CAN_ENABLE_IT(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif

#if defined(STMCANDEBUG)
  if (ret)
  {
    unsigned long pgn;
    unsigned char prio, src, dst;
    CanIdToN2k(id, prio, pgn, src, dst);
    Serial.printf("CANGetFrame pgn:%6lu, prio:%u, src:%u, dst:%u, data:%02X\n",
                  pgn, prio, src, dst, buf[0]);
  }
#endif

  return ret;
}

// ---------------------------------------------------------------------------
bool tNMEA2000_STM32X::addToRingBuffer(const CAN_message_t &msg)
{
  if (rxRing1 == nullptr)
    return false;
  uint8_t prio = (uint8_t)((msg.id >> 26) & 0x7);
  CAN_message_t *rxMsg = rxRing1->getAddRef(prio);
  if (rxMsg != nullptr)
  {
    *rxMsg = msg;
    return true;
  }
  return false;
}

// ===========================================================================
//  init() – peripheral-specific
// ===========================================================================
void tNMEA2000_STM32X::init()
{
#if defined(STM32X_USE_FDCAN)
  hcan_.Instance = FDCAN1;
  g_canInstance = this;
#else
  _can.__this = (void *)this;
  _can.handle.Instance = nullptr;
  baudrate = 250000UL;
  filtersInitialized = false;

  _can.handle.Init.TimeTriggeredMode = DISABLE;
  _can.handle.Init.AutoBusOff = ENABLE;
  _can.handle.Init.AutoWakeUp = DISABLE;
  _can.handle.Init.AutoRetransmission = ENABLE;
  _can.handle.Init.ReceiveFifoLocked = DISABLE;
  _can.handle.Init.TransmitFifoPriority = ENABLE;
  _can.handle.Init.Mode = CAN_MODE_NORMAL;
#endif
}

// ===========================================================================
//  begin() / end()
// ===========================================================================
void tNMEA2000_STM32X::begin()
{
#if defined(STM32X_USE_FDCAN)
#if defined(STMCANDEBUG)
  Serial.println("begin (FDCAN)");
#endif

  if (started_)
    return; // Can be called only once

  const int bitrate = 250000;

#if defined(RCC_PERIPHCLK_FDCAN1)
  uint32_t clockFreq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN1);
#elif defined(RCC_PERIPHCLK_FDCAN)
  uint32_t clockFreq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
#endif
#if defined(STMCANDEBUG)
  Serial.printf("CAN Clock: %d\n", clockFreq);
#endif

  CanTiming timing = solveCanTiming(clockFreq, bitrate);
  FDCAN_InitTypeDef *init = &hcan_.Init;
#ifndef STM32H7
  init->ClockDivider = FDCAN_CLOCK_DIV1;
#endif
  init->FrameFormat = FDCAN_CLASSIC_CAN;
  init->Mode = FDCAN_MODE_NORMAL;
  init->AutoRetransmission = ENABLE;
  init->TransmitPause = ENABLE;
  init->ProtocolException = DISABLE;

  init->NominalPrescaler = (uint16_t)timing.prescaler;
  init->NominalSyncJumpWidth = 1;
  init->NominalTimeSeg1 = timing.tseg1;
  init->NominalTimeSeg2 = timing.tseg2;

  init->DataPrescaler = (uint16_t)timing.prescaler;
  init->DataSyncJumpWidth = 1;
  init->DataTimeSeg1 = timing.tseg1;
  init->DataTimeSeg2 = timing.tseg2;

  init->StdFiltersNbr = 1;
  init->ExtFiltersNbr = 1;
  init->TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  pin_function(rx, pinmap_function(rx, PinMap_CAN_RD));
  pin_function(tx, pinmap_function(tx, PinMap_CAN_TD));

  if (HAL_FDCAN_Init(&hcan_) != HAL_OK)
  {
#if defined(STMCANDEBUG)
    Serial.println("HAL_FDCAN_Init Error");
#endif
  }

  started_ = true;
  applyFilter();

  if (HAL_FDCAN_Start(&hcan_) != HAL_OK)
  {
#if defined(STMCANDEBUG)
    Serial.println("HAL_FDCAN_Start Error");
#endif
  }

  subscribe();

#else // bxCAN ---------------------------------------------------------------
  if (_canIsActive)
    return;

  auto instance = getPeripheral();
  if (instance == NP)
    return;
  if (!allocatePeripheral(instance))
    return;
  _canIsActive = true;

  // Configure RX pin with pull-up
  uint32_t rx_func = pinmap_function(rx, PinMap_CAN_RD);
  rx_func = (rx_func & ~(STM_PIN_PUPD_MASK << STM_PIN_PUPD_SHIFT)) | (GPIO_PULLUP << STM_PIN_PUPD_SHIFT);
  pin_function(rx, fixPinFunction(rx_func));
  if (tx != NC)
    pin_function(tx, fixPinFunction(pinmap_function(tx, PinMap_CAN_TD)));

  // Clock and NVIC
  if (_can.handle.Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
#ifdef CAN1_IRQn_AIO
    HAL_NVIC_SetPriority(CAN1_IRQn_AIO, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_IRQn_AIO);
#else
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
#else
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
#endif
#endif
    _can.bus = 1;
  }
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    _can.bus = 2;
  }
#endif
#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
  {
    __HAL_RCC_CAN3_CLK_ENABLE();
    HAL_NVIC_SetPriority(CAN3_RX0_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN3_TX_IRQn, preemptPriority, subPriority);
    HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
    _can.bus = 3;
  }
#endif

  filtersInitialized = false;
  setBaudRate(baudrate);
#endif // STM32X_USE_FDCAN / STM32X_USE_BXCAN
}

// ---------------------------------------------------------------------------
void tNMEA2000_STM32X::end()
{
#if defined(STM32X_USE_FDCAN)
  if (HAL_FDCAN_Stop(&hcan_) != HAL_OK)
  {
  }
  if (HAL_FDCAN_DeInit(&hcan_) != HAL_OK)
  {
  }

  started_ = false;
#else
  if (!hasPeripheral())
    return;

  stop();
  disableMBInterrupts();

  if (_can.handle.Instance == CAN1)
    __HAL_RCC_CAN1_CLK_DISABLE();
#ifdef CAN2
  else if (_can.handle.Instance == CAN2)
  {
    __HAL_RCC_CAN2_CLK_DISABLE();
    if (canObj[CAN1_INDEX] == nullptr)
      __HAL_RCC_CAN1_CLK_DISABLE();
  }
#endif
#ifdef CAN3
  else if (_can.handle.Instance == CAN3)
    __HAL_RCC_CAN3_CLK_DISABLE();
#endif

  pin_function(rx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, GPIO_AF_NONE));
  if (tx != NC)
    pin_function(tx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, GPIO_AF_NONE));

  freePeripheral();
  _canIsActive = false;
#endif
}

// ===========================================================================
//  applyFilter()
// ===========================================================================
void tNMEA2000_STM32X::applyFilter()
{
#if defined(STM32X_USE_FDCAN)

  HAL_FDCAN_ConfigGlobalFilter(&hcan_,
                               FDCAN_FILTER_REJECT,
                               FDCAN_FILTER_REJECT,
                               FDCAN_FILTER_REMOTE,
                               FDCAN_FILTER_REMOTE);

  FDCAN_FilterTypeDef filter = {
      .IdType = FDCAN_EXTENDED_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_RANGE,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = 0x0000000,
      .FilterID2 = 0x1FFFFFFF,
  };
  HAL_FDCAN_ConfigFilter(&hcan_, &filter);

#endif
}

// ===========================================================================
//  bxCAN – further lifecycle functions
// ===========================================================================
#if defined(STM32X_USE_BXCAN)

void tNMEA2000_STM32X::setIRQPriority(uint32_t preemptPriority, uint32_t subPriority)
{
  this->preemptPriority = min(preemptPriority, MAX_IRQ_PRIO_VALUE);
  this->subPriority = min(subPriority, MAX_IRQ_PRIO_VALUE);
}

void tNMEA2000_STM32X::setBaudRate(uint32_t baud)
{
  baudrate = baud;
  if (!hasPeripheral())
    return;
  if (!calculateBaudrate(baud))
    return;
  stop();
  start();
}

void tNMEA2000_STM32X::start()
{
  HAL_CAN_Init(&_can.handle);
  initializeFilters();
  HAL_CAN_Start(&_can.handle);
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  HAL_CAN_ActivateNotification(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
#else
  HAL_CAN_ActivateNotification(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
#endif
}

void tNMEA2000_STM32X::stop()
{
#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  HAL_CAN_DeactivateNotification(&_can.handle, CAN_IT_RX_FIFO1_MSG_PENDING);
#else
  HAL_CAN_DeactivateNotification(&_can.handle, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_DeactivateNotification(&_can.handle, CAN_IT_TX_MAILBOX_EMPTY);
#endif
  HAL_CAN_DeInit(&_can.handle);
}

bool tNMEA2000_STM32X::setFilterRaw(uint8_t bank_num, uint32_t id, uint32_t mask,
                                    uint32_t filter_mode, uint32_t filter_scale,
                                    FILTER_ACTION action, bool enabled)
{
  if (!_can.handle.Instance)
    return false;
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = bank_num;
  sFilterConfig.FilterMode = filter_mode;
  sFilterConfig.FilterScale = filter_scale;

#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
  if (action == FILTER_ACTION::STORE_FIFO0)
    core_debug("WARNING: RX0 IRQ blocked by USB. Only RX1 events handled!\n");
#endif

  sFilterConfig.FilterFIFOAssignment =
      (action == FILTER_ACTION::STORE_FIFO0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
  sFilterConfig.FilterActivation = enabled ? ENABLE : DISABLE;
  sFilterConfig.FilterIdLow = id & 0xFFFFUL;
  sFilterConfig.FilterIdHigh = id >> 16;
  sFilterConfig.FilterMaskIdLow = mask & 0xFFFFUL;
  sFilterConfig.FilterMaskIdHigh = mask >> 16;

#ifdef CAN2
  sFilterConfig.SlaveStartFilterBank = STM32_CAN_CAN2_FILTER_OFFSET;
  if (_can.handle.Instance == CAN2)
    sFilterConfig.FilterBank += STM32_CAN_CAN2_FILTER_OFFSET;
#endif

  return (HAL_CAN_ConfigFilter(&_can.handle, &sFilterConfig) == HAL_OK);
}

void tNMEA2000_STM32X::initializeFilters()
{
  if (filtersInitialized)
    return;
  filtersInitialized = true;
  setFilterRaw(0, 0UL, 0UL, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT,
               FILTER_ACTION::CAN_FILTER_DEFAULT_ACTION, true);
}

CAN_TypeDef *tNMEA2000_STM32X::getPeripheral()
{
  CAN_TypeDef *canPort_rx = (CAN_TypeDef *)pinmap_peripheral(rx, PinMap_CAN_RD);
  CAN_TypeDef *canPort_tx = (CAN_TypeDef *)pinmap_peripheral(tx, PinMap_CAN_TD);
  if ((canPort_rx != canPort_tx && canPort_tx != NP) || canPort_rx == NP)
    return NP;
#ifdef STM32F1xx
  if (canPort_rx != NP && canPort_tx != NP)
  {
    if (STM_PIN_AFNUM(pinmap_function(rx, PinMap_CAN_RD)) !=
        STM_PIN_AFNUM(pinmap_function(tx, PinMap_CAN_TD)))
      return NP;
  }
#endif
  if (canPort_tx == NP)
    tx = NC;
  return canPort_rx;
}

uint32_t tNMEA2000_STM32X::fixPinFunction(uint32_t function)
{
#ifdef STM32F1xx
  uint32_t af = STM_PIN_AFNUM(function);
  if (af == AFIO_NONE)
    af = AFIO_CAN1_1;
  function &= ~(STM_PIN_AFNUM_MASK << STM_PIN_AFNUM_SHIFT);
  function |= ((af & STM_PIN_AFNUM_MASK) << STM_PIN_AFNUM_SHIFT);
#endif
  return function;
}

void tNMEA2000_STM32X::enableMBInterrupts()
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
#endif
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

void tNMEA2000_STM32X::disableMBInterrupts()
{
  if (_can.handle.Instance == CAN1)
  {
#ifdef CAN1_IRQn_AIO
#if defined(HAL_CEC_MODULE_ENABLED) && defined(STM32_CAN1_SHARED_WITH_CEC)
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
#endif
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

// --- Baudrate calculation (bxCAN) ---

uint32_t tNMEA2000_STM32X::getCanPeripheralClock()
{
  return HAL_RCC_GetPCLK1Freq(); // bxCAN clock is APB1/PCLK1
}

void tNMEA2000_STM32X::setBaudRateValues(uint16_t prescaler, uint8_t timeseg1,
                                         uint8_t timeseg2, uint8_t sjw)
{
  uint32_t _SyncJumpWidth = 0;
  uint32_t _TimeSeg1 = 0;
  uint32_t _TimeSeg2 = 0;

  /* the CAN specification (v2.0) states that SJW shall be programmable between
   * 1 and min(4, timeseg1)
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
  default:
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
    _TimeSeg2 = CAN_BS2_1TQ;
    break;
  }

  _can.handle.Init.SyncJumpWidth = _SyncJumpWidth;
  _can.handle.Init.TimeSeg1 = _TimeSeg1;
  _can.handle.Init.TimeSeg2 = _TimeSeg2;
  _can.handle.Init.Prescaler = prescaler;
}

template <typename T, size_t N>
bool tNMEA2000_STM32X::lookupBaudrate(int baud, const T (&table)[N])
{
  for (size_t i = 0; i < N; i++)
  {
    if (baud != (int)table[i].baudrate)
      continue;
    setBaudRateValues(table[i].prescaler, table[i].timeseg1, table[i].timeseg2, 4);
    return true;
  }
  return false;
}

bool tNMEA2000_STM32X::calculateBaudrate(int baud)
{
  const uint32_t frequency = getCanPeripheralClock();

  if (frequency == 48000000 && lookupBaudrate(baud, BAUD_RATE_TABLE_48M))
    return true;
  if (frequency == 45000000 && lookupBaudrate(baud, BAUD_RATE_TABLE_45M))
    return true;
  if (frequency == 42000000 && lookupBaudrate(baud, BAUD_RATE_TABLE_42M))
    return true;
  if (frequency == 36000000 && lookupBaudrate(baud, BAUD_RATE_TABLE_36M))
    return true;

  // General algorithm (75–95% sample point)
  for (uint16_t prescaler = 1; prescaler <= 1024; prescaler++)
  {
    const uint32_t can_freq = frequency / prescaler;
    if (can_freq / (1 + 5 + 16) > (uint32_t)baud)
      continue;
    for (uint8_t bs2 = 1; bs2 <= 5; bs2++)
    {
      for (uint8_t bs1 = (bs2 * 3) - 1; bs1 <= 16; bs1++)
      {
        if (can_freq / (1 + bs1 + bs2) != (uint32_t)baud)
          continue;
        setBaudRateValues(prescaler, bs1, bs2, 4);
        return true;
      }
    }
  }
  return false;
}

#endif // STM32X_USE_BXCAN

// ===========================================================================
//  FDCAN – Timing algorithm
// ===========================================================================
#if defined(STM32X_USE_FDCAN)

void tNMEA2000_STM32X::subscribe()
{
  HAL_FDCAN_ActivateNotification(&hcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hcan_, FDCAN_IT_TX_COMPLETE, 0x07);
  HAL_FDCAN_ActivateNotification(&hcan_, FDCAN_IT_BUS_OFF, 0);
}

void tNMEA2000_STM32X::unsubscribe()
{
  HAL_FDCAN_DeactivateNotification(&hcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  HAL_FDCAN_DeactivateNotification(&hcan_, FDCAN_IT_TX_COMPLETE);
  HAL_FDCAN_DeactivateNotification(&hcan_, FDCAN_IT_BUS_OFF);
}

#if defined(RCC_PERIPHCLK_FDCAN)
#define FDCAN_PERIPH_CLK RCC_PERIPHCLK_FDCAN
#elif defined(RCC_PERIPHCLK_FDCAN1)
#define FDCAN_PERIPH_CLK RCC_PERIPHCLK_FDCAN1
#else
#error "RCC_PERIPHCLK_FDCAN not found for this STM32 family"
#endif

CanTiming tNMEA2000_STM32X::solveCanTiming(uint32_t clockFreq, uint32_t bitrate,
                                           uint8_t multiplier)
{
  // Inspired by: http://www.bittiming.can-wiki.info/
  CanTiming timing = {};
  const uint32_t baseQuanta = 16;
  uint32_t timeQuanta = baseQuanta;
  uint32_t offset = 0;
  bool found = false;

  while (offset <= 9)
  {
    timeQuanta = baseQuanta - offset;
    if (clockFreq % (bitrate * timeQuanta * multiplier) == 0)
    {
      found = true;
      break;
    }
    timeQuanta = baseQuanta + offset;
    if (clockFreq % (bitrate * timeQuanta * multiplier) == 0)
    {
      found = true;
      break;
    }
    offset++;
  }
  if (!found)
  {
#if defined(STMCANDEBUG)
    Serial.println("Error: FDCAN timing not found");
#endif
  }
  timing.prescaler = clockFreq / (bitrate * timeQuanta);
  timing.sjw = 1;
  timing.tseg1 = uint32_t(0.875f * timeQuanta) - 1;

  float sp = (1.0f + timing.tseg1) / timeQuanta;
  float sp2 = (1.0f + timing.tseg1 + 1) / timeQuanta;
  if (fabsf(sp2 - 0.875f) < fabsf(sp - 0.875f))
  {
    timing.tseg1++;
  }

  timing.tseg2 = timeQuanta - timing.tseg1 - 1;
  return timing;
}

uint32_t tNMEA2000_STM32X::lengthToDLC(uint32_t length)
{
  switch (length)
  {
  case 0:
    return FDCAN_DLC_BYTES_0;
  case 1:
    return FDCAN_DLC_BYTES_1;
  case 2:
    return FDCAN_DLC_BYTES_2;
  case 3:
    return FDCAN_DLC_BYTES_3;
  case 4:
    return FDCAN_DLC_BYTES_4;
  case 5:
    return FDCAN_DLC_BYTES_5;
  case 6:
    return FDCAN_DLC_BYTES_6;
  case 7:
    return FDCAN_DLC_BYTES_7;
  case 8:
    return FDCAN_DLC_BYTES_8;
  case 12:
    return FDCAN_DLC_BYTES_12;
  case 16:
    return FDCAN_DLC_BYTES_16;
  case 20:
    return FDCAN_DLC_BYTES_20;
  case 24:
    return FDCAN_DLC_BYTES_24;
  case 32:
    return FDCAN_DLC_BYTES_32;
  case 48:
    return FDCAN_DLC_BYTES_48;
  case 64:
    return FDCAN_DLC_BYTES_64;
  default:
    return 0;
  }
}

static uint32_t dlcToLength(uint32_t dlc)
{
  switch (dlc)
  {
  case FDCAN_DLC_BYTES_0:
    return 0;
  case FDCAN_DLC_BYTES_1:
    return 1;
  case FDCAN_DLC_BYTES_2:
    return 2;
  case FDCAN_DLC_BYTES_3:
    return 3;
  case FDCAN_DLC_BYTES_4:
    return 4;
  case FDCAN_DLC_BYTES_5:
    return 5;
  case FDCAN_DLC_BYTES_6:
    return 6;
  case FDCAN_DLC_BYTES_7:
    return 7;
  case FDCAN_DLC_BYTES_8:
    return 8;
  case FDCAN_DLC_BYTES_12:
    return 12;
  case FDCAN_DLC_BYTES_16:
    return 16;
  case FDCAN_DLC_BYTES_20:
    return 20;
  case FDCAN_DLC_BYTES_24:
    return 24;
  case FDCAN_DLC_BYTES_32:
    return 32;
  case FDCAN_DLC_BYTES_48:
    return 48;
  case FDCAN_DLC_BYTES_64:
    return 64;
  default:
    return 0;
  }
}

#endif // STM32X_USE_FDCAN

// ===========================================================================
//  Interrupt handlers
// ===========================================================================

// --- FDCAN -----------------------------------------------------------------
#if defined(STM32X_USE_FDCAN)

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  FDCAN_ProtocolStatusTypeDef protocol_status;
  HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status);

  if (protocol_status.BusOff != 0) // If Bus-Off error occurred
  {
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT); // Clear INIT bit to recover from Bus-Off
  }
}

extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *FDCanHandle,
                                                   uint32_t BufferIndexes)
{
  tNMEA2000_STM32X *_can = g_canInstance;
  if (_can == nullptr)
    return;
  while (HAL_FDCAN_GetTxFifoFreeLevel(FDCanHandle) > 0)
  {
    if (!_can->sendFromTxRing())
      break;
  }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo0ITs)
{
  tNMEA2000_STM32X *_can = g_canInstance;
  if (_can == nullptr)
    return;

  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
  {
    CAN_message_t rxmsg;
    FDCAN_RxHeaderTypeDef RxHeader = {};
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rxmsg.buf) == HAL_OK)
      {
        rxmsg.id = RxHeader.Identifier;
        rxmsg.flags.extended = (RxHeader.IdType == FDCAN_EXTENDED_ID);
        rxmsg.flags.remote = (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME);
        rxmsg.len = (uint8_t)dlcToLength(RxHeader.DataLength);
        _can->addToRingBuffer(rxmsg);
      }
    }
  }
}

extern "C" void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&tNMEA2000_STM32X::hcan_);
}

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{

#if defined(__HAL_RCC_FDCAN1_CLK_ENABLE)
  __HAL_RCC_FDCAN1_CLK_ENABLE();
#elif defined(__HAL_RCC_FDCAN_CLK_ENABLE)
  __HAL_RCC_FDCAN_CLK_ENABLE();
#elif
#error "FDCAN Clock Enable for this platform is not defined"
#endif

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}

// --- bxCAN -----------------------------------------------------------------
#else // STM32X_USE_BXCAN

extern "C" void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *CanHandle)
{
  tNMEA2000_STM32X *_can = (tNMEA2000_STM32X *)get_can_obj(CanHandle)->__this;
  _can->sendFromTxRing();
}
extern "C" void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *CanHandle)
{
  tNMEA2000_STM32X *_can = (tNMEA2000_STM32X *)get_can_obj(CanHandle)->__this;
  _can->sendFromTxRing();
}
extern "C" void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *CanHandle)
{
  tNMEA2000_STM32X *_can = (tNMEA2000_STM32X *)get_can_obj(CanHandle)->__this;
  _can->sendFromTxRing();
}

#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
#else
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
#endif
{
  tNMEA2000_STM32X *_can = (tNMEA2000_STM32X *)get_can_obj(CanHandle)->__this;
  CAN_message_t rxmsg;
  CAN_RxHeaderTypeDef RxHeader;

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
      rxmsg.bus = get_can_obj(CanHandle)->bus;
      _can->addToRingBuffer(rxmsg);
    }
  } while (HAL_CAN_GetRxFifoFillLevel(CanHandle, fifo));
}

// --- IRQ-Handler (bxCAN) ---
#ifdef CAN1_IRQHandler_AIO
extern "C" void CAN1_IRQHandler_AIO(void)
{
  if (canObj[CAN1_INDEX])
    HAL_CAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
#if defined(HAL_CEC_MODULE_ENABLED) && defined(STM32_CAN1_SHARED_WITH_CEC)
  if (phcec)
    HAL_CEC_IRQHandler(phcec);
#endif
}
#else

#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
extern "C" void CAN1_RX1_IRQHandler(void)
#else
extern "C" void CAN1_RX0_IRQHandler(void)
#endif
{
  if (canObj[CAN1_INDEX])
    HAL_CAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
}

#ifdef CAN2
extern "C" void CAN2_RX0_IRQHandler(void)
{
  if (canObj[CAN2_INDEX])
    HAL_CAN_IRQHandler(&canObj[CAN2_INDEX]->handle);
}
#endif
#ifdef CAN3
extern "C" void CAN3_RX0_IRQHandler(void)
{
  if (canObj[CAN3_INDEX])
    HAL_CAN_IRQHandler(&canObj[CAN3_INDEX]->handle);
}
#endif

#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
extern "C" void STM32_CAN_Poll_IRQ_Handler(void)
#else
extern "C" void CAN1_TX_IRQHandler(void)
#endif
{
  if (canObj[CAN1_INDEX])
    HAL_CAN_IRQHandler(&canObj[CAN1_INDEX]->handle);
}

#ifdef CAN2
extern "C" void CAN2_TX_IRQHandler(void)
{
  if (canObj[CAN2_INDEX])
    HAL_CAN_IRQHandler(&canObj[CAN2_INDEX]->handle);
}
#endif
#ifdef CAN3
extern "C" void CAN3_TX_IRQHandler(void)
{
  if (canObj[CAN3_INDEX])
    HAL_CAN_IRQHandler(&canObj[CAN3_INDEX]->handle);
}
#endif

#endif // CAN1_IRQHandler_AIO

#endif // STM32X_USE_FDCAN / STM32X_USE_BXCAN
