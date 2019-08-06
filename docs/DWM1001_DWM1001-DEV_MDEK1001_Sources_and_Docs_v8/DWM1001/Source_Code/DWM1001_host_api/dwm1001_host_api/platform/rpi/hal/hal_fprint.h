/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal_fprint.h
 * @brief   utility print to log file
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
 
#ifndef _HAL_FPRINT_H_
#define _HAL_FPRINT_H_

#include <stdio.h>
#include <stdlib.h>

/** 
 * @brief get log file pointer. 
 *        If log file not open yet, initializes the log file. 
 *
 * @param none
 *
 * @return log file pointer
 */
FILE * HAL_Fprint_GetFile(void);

/** 
 * @brief de-initializes the log file. 
 *
 * @param none
 *
 * @return none
 */
void HAL_Fprint_DeInit(void);

/** 
 * @brief prints formated args into log file. 
 *
 * @param[in] formated args
 *
 * @return none
 */
void HAL_Fprint(const char* format, ... );

#endif //_HAL_FPRINT_H_
