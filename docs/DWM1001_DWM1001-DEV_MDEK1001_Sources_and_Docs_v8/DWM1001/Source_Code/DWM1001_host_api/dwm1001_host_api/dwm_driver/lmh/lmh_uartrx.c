/*! ------------------------------------------------------------------------------------------------------------------
 * @file    lmh_uartrx.c
 * @brief   low-level module handshake (LMH) utilities to handle DWM1001 UART
 *          rx functionalities. 
 *
 *          In Makefile, interface configuration needs to be defined as:
 *          INTERFACE_NUMBER = 0
 *
 *          This file describes the RX setup example. 
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "lmh.h"
#include "lmh_uartrx.h"

#define LMH_UART_TIMEOUT_DEFAULT      1000

static bool lmh_uartrx_initialized = false;
static int  lmh_uartrx_timeout = LMH_UART_TIMEOUT_DEFAULT;
static bool lmh_uartrx_flag = false;

/**
 * @brief : on having received any UART incoming bytes, set the lmh_uartrx_flag 
 *
 * @param [in] status, uart signal number, NOT used here. 
 *
 * @return none
 */
static void LMH_UARTRX_Cb(int status)
{
   LMH_UARTRX_Set();  
}  

/**
 * @brief : initialises the UARTRX functions. 
 */
void LMH_UARTRX_Init(void)
{
   HAL_UART_Init();
   LMH_UARTRX_SetTimeout(LMH_UART_TIMEOUT_DEFAULT);
   
   HAL_UART_SetRxCb(&LMH_UARTRX_Cb);   
   LMH_UARTRX_Clear();   
   lmh_uartrx_initialized = true;     
}

/**
 * @brief : de-initialises the UARTRX functions. 
 */
void LMH_UARTRX_DeInit(void)
{
   lmh_uartrx_initialized = false;
   HAL_UART_SetRxCb(NULL);   
}

/**
 * @brief : Sets the lmh_uartrx_flag 
 */
void LMH_UARTRX_Set(void)
{   
   lmh_uartrx_flag = true;   
}

/**
 * @brief : Clears the lmh_uartrx_flag 
 */
void LMH_UARTRX_Clear(void)
{   
   lmh_uartrx_flag = false;   
}

/**
 * @brief : gets the lmh_uartrx_flag flag
 *
 * @return lmh_uartrx_flag
 */
bool LMH_UARTRX_IsSet(void)
{   
   return lmh_uartrx_flag;   
}

/**
 * @brief : Set the UARTRX time out period. 
 *
 * @param [in] timeout, UARTRX time out period in ms
 */
void LMH_UARTRX_SetTimeout(int timeout)
{
   lmh_uartrx_timeout = timeout;
}

/**
 * @brief : wait length=exp_length for max time=lmh_uartrx_timeout
 *          needs LMH_UARTRX_Init() at initialization 
 *          needs LMH_UARTRX_Clear() before Tx the TLV request  
 *
 * @param [out] data,       pointer to received data 
 * @param [out] length,     pointer to received data length 
 * @param [in] exp_length,  expected data length
 *
 * @return RV_OK if success; else RV_ERR
 */
#define _10SECOND_FACTOR 10000000
int LMH_UARTRX_WaitForRx(uint8_t* data, uint16_t* length, uint16_t exp_length)
{
   uint8_t rx_length = HAL_UART_MAX_LENGTH;
   uint64_t start, current;
   
   if(!lmh_uartrx_initialized)
   {
      HAL_Print("\tUART: ERROR: not initialized.");
      return LMH_ERR;
   }
   if(exp_length < DWM1001_TLV_RET_VAL_MIN_SIZE)
   {
      HAL_Print("\tUART: ERROR: exp_length must be >= 3\n");    
      return LMH_ERR;
   }
      
   current = start = HAL_GetTime64()%_10SECOND_FACTOR;
   *length = 0;   
               
   while(((current + _10SECOND_FACTOR - start)%_10SECOND_FACTOR < (lmh_uartrx_timeout*1000)) && (*length < exp_length))
   { 
      HAL_Delay(1);
      if(LMH_UARTRX_IsSet()) 
      {
         LMH_UARTRX_Clear();
         rx_length = HAL_UART_MAX_LENGTH;
         HAL_UART_Rx(data+(*length), &rx_length);
         *length += rx_length;
         if((*length >= DWM1001_TLV_RET_VAL_MIN_SIZE) && (LMH_CheckRetVal(data) != LMH_OK))
         {
            HAL_Delay(1);
            return LMH_ERR;
         }
      }
      current = HAL_GetTime64() % _10SECOND_FACTOR;
   }   
   HAL_Delay(1);
   
   LMH_UARTRX_Clear();    
   
   if(*length == exp_length)
   {
      HAL_Fprint("\tUART: Received all %d bytes, within %0.2f ms \t OK\n", \
      *length, ((float)(current-start)/1000));
      return LMH_OK;
   }
   else if(exp_length == DWM1001_TLV_MAX_SIZE)
   {  
      //set exp_length to DWM1001_TLV_MAX_SIZE (255) if rx length is not fixed
      HAL_Fprint("\tUART: Received %d bytes, until timed out in %d ms \t OK\n", *length, lmh_uartrx_timeout);
      return LMH_OK;
   }      
   else// timed out
   {
      HAL_Print(" >>>>>> TIMED OUT <<<<<< UART: Received %d bytes, expected %d bytes, timed out in %d ms\n", \
      *length, exp_length, lmh_uartrx_timeout);
   
      HAL_Print("\tUART: Received length=%d",*length);
      if (*length >0)
      {
         HAL_Print(", data=0x");
         int i;
         for(i=0; i<*length; i++)
         {
            HAL_Print(" %02x", data[i]);
         }
      }
      HAL_Print("\n");
      
      return LMH_ERR;
   }
}




