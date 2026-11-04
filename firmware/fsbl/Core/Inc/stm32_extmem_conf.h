/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stm32_extmem_conf.h
  * @version        : 1.0.0
  * @brief          : Header for extmem.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_EXTMEM_CONF__H__
#define __STM32_EXTMEM_CONF__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/*
  @brief management of the driver layer enable
*/

#define EXTMEM_DRIVER_NOR_SFDP   0
#define EXTMEM_DRIVER_PSRAM      0
#define EXTMEM_DRIVER_SDCARD     0
#define EXTMEM_DRIVER_USER       1

/*
  @brief management of the sal layer enable
*/
#define EXTMEM_SAL_XSPI   0
#define EXTMEM_SAL_SD     0

/* Includes ------------------------------------------------------------------*/
#include "stm32n6xx_hal.h"
#include "stm32_extmem.h"
#include "stm32_extmem_type.h"
#include "boot/stm32_boot_lrun.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Exported constants --------------------------------------------------------*/
/** @defgroup EXTMEM_CONF_Exported_constants EXTMEM_CONF exported constants
  * @{
  */
enum {
  EXTMEMORY_1  = 0 /*!< ID=0 for the first memory  */
};

/*
  @brief management of the boot layer
*/
#define EXTMEM_MEMORY_BOOTLRUN  EXTMEMORY_1
#define EXTMEM_XIP_IMAGE_OFFSET 0x100000
#define EXTMEM_HEADER_OFFSET 0

/* LRUN (Load & Run): copy the NuttX image from the NOR flash (source) into
   internal SRAM (destination) then jump.  NuttX links/runs at 0x34000400,
   the same SRAM window the DEV-boot flow uses. */
#define EXTMEM_LRUN_DESTINATION        EXTMEMORY_1
#define EXTMEM_LRUN_SOURCE             EXTMEMORY_1
#define EXTMEM_LRUN_DESTINATION_INTERNAL
#define EXTMEM_LRUN_DESTINATION_ADDRESS  0x34000400U
#define EXTMEM_LRUN_SOURCE_ADDRESS       0x100000U   /* NuttX at NOR 0x70100000 */

/* 2026-11-03: raised from 0x100000 to 0x110000 (+64 KB).
 *
 * 1 MiB was only ever a round number matching the FSBL's own 1 MiB NOR slot;
 * BOOT_GetApplicationSize() returns this constant verbatim without reading any
 * header, so the app image was silently truncated at exactly 1 MiB and the
 * raster build had 6.5 KB left.  The NOR is empty from 0x70200000 (pose994's
 * old slot, unused by the eye+face build) up to the eye weights at
 * 0x70400000, so growing into it disturbs nothing.
 *
 * The real ceiling is RAM, not NOR: the CPU window ends at 0x34200000 because
 * AXISRAM3..6 (1.75 MB above it) is the NPU tensor pool.  With the initial MSP
 * currently at 0x341f0788 that leaves 63608 bytes of growth, so 0x110000
 * releases essentially all of what is available.
 */
#define EXTMEM_LRUN_SOURCE_SIZE          0x110000U   /* up to 1.0625 MiB image */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Exported configuration --------------------------------------------------------*/
/** @defgroup EXTMEM_CONF_Exported_configuration EXTMEM_CONF exported configuration definition
  * @{
  */

extern EXTMEM_DefinitionTypeDef extmem_list_config[1];
#if defined(EXTMEM_C)
EXTMEM_DefinitionTypeDef extmem_list_config[1];
#endif /* EXTMEM_C */

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN VARIABLES */

/* USER CODE END VARIABLES */

/*
 * -- Insert functions declaration here --
 */
/* USER CODE BEGIN FD */

/* USER CODE END FD */

#ifdef __cplusplus
}
#endif

#endif /* __STM32_EXTMEM_CONF__H__ */
