// Host render build: just enough of the platform for osd_demo.c / chiptune.c
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define USE_MAX7456
#define USE_CHIPTUNE
#define USE_CLI
#define USE_OSD
#define OSD_DEMO_HOST 1
#define FAST_CODE
#define FAST_CODE_NOINLINE
#define FAST_DATA_ZERO_INIT
#define FAST_DATA
#define DMA_DATA
#define DMA_DATA_ZERO_INIT
#define STATIC_UNIT_TESTED static
#define UNIT_TESTED
#define MAX7456_SPI_INSTANCE 0
