// Host render: the firmware APIs osd_demo.c / chiptune.c use, declared for the host
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "common/time.h"
#include "common/utils.h"
#include "host_regs.h"

// Time: the harness's virtual clock (DWT at 100 MHz, as the F411 runs)
#define HOST_CPU_HZ 100000000u
timeMs_t millis(void);
timeUs_t micros(void);
timeUs_t microsISR(void);
uint32_t getCycleCounter(void);
uint32_t clockMicrosToCycles(uint32_t micros);
uint32_t clockCyclesToMicros(uint32_t cycles);
void delay(timeMs_t ms);
void delayMicroseconds(timeUs_t us);

// Arming, features, motors
#define ARMED 1
#define ARMING_FLAG(f) (0)
#define FEATURE_OSD (1u << 18)
#define FEATURE_AIRMODE (1u << 22)
bool featureIsEnabled(uint32_t mask);
void featureEnableImmediate(uint32_t mask);
bool motorIsEnabled(void);
void motorDisable(void);

// IO / timers / DMA (HSYNC capture and the slot engine: inert on the host)
typedef uint8_t ioTag_t;
typedef void *IO_t;
typedef uint8_t ioConfig_t;
typedef enum { OWNER_FREE = 0, OWNER_TIMER = 1, OWNER_OSD = 2 } resourceOwner_e;
#define IO_TAG(pin) ((ioTag_t)1)
#define IO_TAG_NONE ((ioTag_t)0)
#define DEFIO_TAG_E(pin) ((ioTag_t)1)
#define DEFIO_TAG_GPIOID(tag) 1
#define DEFIO_TAG_PIN(tag) 3
#define IO_CONFIG(m, s, o, p) 0
#define IOCFG_IN_FLOATING 0
#define GPIO_Speed_50MHz 0
#define GPIO_PuPd_UP 0
#define GPIO_OType_PP 0
#define GPIO_Mode_AF 0
#define GPIO_AF_TIM2 0
IO_t IOGetByTag(ioTag_t tag);
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index);
resourceOwner_e IOGetOwner(IO_t io);
void IOConfigGPIO(IO_t io, ioConfig_t cfg);
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);
typedef uint32_t captureCompare_t;
typedef struct timerEdgeHandlerRec_s { void (*fn)(struct timerEdgeHandlerRec_s *, captureCompare_t); } timerEdgeHandlerRec_t;
typedef void timerEdgeHandlerCallback(timerEdgeHandlerRec_t *self, captureCompare_t capture);
typedef struct timerHardware_s { TIM_TypeDef *tim; ioTag_t tag; uint8_t channel; uint8_t usageFlags; uint8_t output; uint8_t alternateFunction; } timerHardware_t;
typedef uint8_t timerResource_t;
#define TIM_USE_ANY 0
void timerChannelEdgeHandlerInit(timerEdgeHandlerRec_t *self, timerEdgeHandlerCallback *fn);
void timerChannelConfigCallbacks(const timerHardware_t *timHw, timerEdgeHandlerRec_t *edge, void *overflow);
uint32_t timerClock(const timerHardware_t *timHw);
typedef struct dmaChannelDescriptor_s { DMA_TypeDef *dma; DMA_Stream_TypeDef *ref; uint8_t stream; } dmaChannelDescriptor_t;
typedef void (*dmaCallbackHandlerFuncPtr)(dmaChannelDescriptor_t *channelDescriptor);
#define DMA1_ST5_HANDLER 5
void dmaSetHandler(int identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);

// OSD / displayport / profiles
#include "drivers/display.h"
typedef enum { OSD_DISPLAYPORT_DEVICE_NONE = 0, OSD_DISPLAYPORT_DEVICE_AUTO, OSD_DISPLAYPORT_DEVICE_MAX7456 } osdDisplayPortDevice_e;
displayPort_t *osdGetDisplayPort(osdDisplayPortDevice_e *device);
void osdInit(displayPort_t *osdDisplayPortToUse, osdDisplayPortDevice_e displayPortDevice);
typedef struct vcdProfile_s { uint8_t video_system; int8_t h_offset; int8_t v_offset; } vcdProfile_t;
const vcdProfile_t *vcdProfile(void);
bool max7456DisplayPortInit(const vcdProfile_t *vcdProfile, displayPort_t **displayPort);
typedef struct displayPortProfile_s { int8_t colAdjust, rowAdjust; bool invert; uint8_t blackBrightness, whiteBrightness; } displayPortProfile_t;
const displayPortProfile_t *displayPortProfileMax7456(void);
typedef struct { int16_t roll, pitch, yaw; } attitudeValues_t;
typedef union { int16_t raw[3]; attitudeValues_t values; } attitudeEulerAngles_t;
extern attitudeEulerAngles_t attitude;

// Beeper / AY audio hooks the demo calls

// CLI
void cliPrintAsync(const char *line);

// chiptune.c: debug, LEDs, scheduler, gyro, beeper PWM / AY audio path
extern int16_t debug[8];
#define DEBUG_SET(mode, index, value) do { } while (0)
void ledSet(int led, bool state);
void ledToggle(int led);
#define LED0_ON  do { } while (0)
#define LED0_OFF do { } while (0)
#define LED1_ON  do { } while (0)
#define LED1_OFF do { } while (0)
#define LED2_ON  do { } while (0)
#define LED2_OFF do { } while (0)
typedef enum { TASK_CHIPTUNE = 1, TASK_GYRO, TASK_FILTER, TASK_PID, TASK_RX, TASK_ACCEL, TASK_ATTITUDE,
    TASK_BATTERY_VOLTAGE, TASK_BATTERY_CURRENT, TASK_BATTERY_ALERTS, TASK_BEEPER, TASK_COUNT } taskId_e;
typedef struct { bool isEnabled; } taskInfo_t;
void getTaskInfo(taskId_e taskId, taskInfo_t *taskInfo);
#define GYRO_COUNT 1
#define GYRO_NONE 0
typedef struct { struct { int gyroHardware; ioTag_t mpuIntExtiTag; } gyroDev; } hostGyroSensor_t;
typedef struct { hostGyroSensor_t gyroSensor[GYRO_COUNT]; } hostGyro_t;
extern hostGyro_t gyro;
void EXTIDisable(IO_t io);
void EXTIEnable(IO_t io);
#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
void setTaskEnabled(taskId_e taskId, bool newEnabledState);
void rescheduleTask(taskId_e taskId, int32_t newPeriodUs);
