/*! ------------------------------------------------------------------------------------------------------------------
 * @file    dwm_api.h
 * @brief   DWM1001 host API header 
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */ 
#include "lmh.h"
#include "dwm_api.h"

void dwm_init(void)
{
   LMH_Init();
}

int dwm_pos_set(dwm_pos_t* pos)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_POS_SET;
   tx_data[tx_len++] = 13;
   *(uint32_t*)(tx_data+tx_len) = pos->x;
   tx_len+=4;
   *(uint32_t*)(tx_data+tx_len) = pos->y;
   tx_len+=4;
   *(uint32_t*)(tx_data+tx_len) = pos->z;
   tx_len+=4;
   tx_data[tx_len++] = pos->qf;
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_pos_get(dwm_pos_t* p_pos)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_POS_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 18) == RV_OK)
   {
      data_cnt = 5;
      p_pos->x = rx_data[data_cnt] 
             + (rx_data[data_cnt+1]<<8) 
             + (rx_data[data_cnt+2]<<16) 
             + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      p_pos->y = rx_data[data_cnt] 
             + (rx_data[data_cnt+1]<<8) 
             + (rx_data[data_cnt+2]<<16) 
             + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      p_pos->z = rx_data[data_cnt] 
             + (rx_data[data_cnt+1]<<8) 
             + (rx_data[data_cnt+2]<<16) 
             + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      p_pos->qf = rx_data[data_cnt];
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_upd_rate_set(uint16_t ur, uint16_t ur_static)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UR_SET;
   tx_data[tx_len++] = 4;
   tx_data[tx_len++] = ur & 0xff;
   tx_data[tx_len++] = (ur>>8) & 0xff;   
   tx_data[tx_len++] = ur_static & 0xff;
   tx_data[tx_len++] = (ur_static>>8) & 0xff;   
   LMH_Tx(tx_data, &tx_len);    
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_upd_rate_get(uint16_t *ur, uint16_t *ur_static)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_UR_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 9) == RV_OK)
   {
      data_cnt = 5;
      *ur = rx_data[data_cnt] + (rx_data[data_cnt+1]<<8);
      data_cnt += 2;
      *ur_static  = rx_data[data_cnt] + (rx_data[data_cnt+1]<<8);
      data_cnt += 2;
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_cfg_tag_set(dwm_cfg_tag_t* cfg)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_CFG_TN_SET;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = (cfg->low_power_en ?        (1<<7):0)
                     + (cfg->loc_engine_en ?       (1<<6):0)
                     + (cfg->common.security_en ?  (1<<5):0)
                     + (cfg->common.led_en ?       (1<<4):0)
                     + (cfg->common.ble_en ?       (1<<3):0)
                     + (cfg->common.fw_update_en ? (1<<2):0)
                     + (((cfg->common.uwb_mode) &  0x03)<<0);
   tx_data[tx_len++] = (cfg->accel_en ?            (1<<2):0)
                     + (((cfg->meas_mode)       &  0x03)<<0);
   LMH_Tx(tx_data, &tx_len);    
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_cfg_anchor_set(dwm_cfg_anchor_t* cfg)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_CFG_AN_SET;
   tx_data[tx_len++] = 1;
   tx_data[tx_len++] = (cfg->initiator ?           (1<<7):0)
                     + (cfg->bridge ?              (1<<6):0)
                     + (cfg->common.security_en ?  (1<<5):0)
                     + (cfg->common.led_en ?       (1<<4):0)
                     + (cfg->common.ble_en ?       (1<<3):0)
                     + (cfg->common.fw_update_en ? (1<<2):0)
                     + (((cfg->common.uwb_mode)  & 0x03)<<0);
   LMH_Tx(tx_data, &tx_len);    
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_cfg_get(dwm_cfg_t* cfg)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_CFG_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 7) == RV_OK)
   {
      cfg->mode                  = (rx_data[6]>>5) & 0x01;
      cfg->initiator             = (rx_data[6]>>4) & 0x01;
      cfg->bridge                = (rx_data[6]>>3) & 0x01;
      cfg->accel_en              = (rx_data[6]>>2) & 0x01;
      cfg->meas_mode             = (rx_data[6]>>0) & 0x03;
      
      cfg->low_power_en          = (rx_data[5]>>7) & 0x01;
      cfg->loc_engine_en         = (rx_data[5]>>6) & 0x01;
      cfg->common.security_en    = (rx_data[5]>>5) & 0x01;
      cfg->common.led_en         = (rx_data[5]>>4) & 0x01;
      cfg->common.ble_en         = (rx_data[5]>>3) & 0x01;
      cfg->common.fw_update_en   = (rx_data[5]>>2) & 0x01;
      cfg->common.uwb_mode       = (rx_data[5]>>0) & 0x03;
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_sleep(void)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_SLEEP;
   tx_data[tx_len++] = 0;  
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

#define LOC_DATA_POS_OFFSET   3
#define LOC_DATA_DIST_OFFSET  LOC_DATA_POS_OFFSET+15
int dwm_loc_get(dwm_loc_data_t* loc)
{ 
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt, i, j;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_LOC_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, DWM1001_TLV_MAX_SIZE) == RV_OK)
   {
      if(rx_len<3+15+3)// ok + pos + distance/range
      {
         return RV_ERR;
      }
      
      if(rx_data[LOC_DATA_POS_OFFSET]==DWM1001_TLV_TYPE_POS_XYZ)//0x41
      {
         // node self position.
         data_cnt = LOC_DATA_POS_OFFSET+2;// jump Type and Length, goto data
         loc->p_pos->x = rx_data[data_cnt] 
                      + (rx_data[data_cnt+1]<<8) 
                      + (rx_data[data_cnt+2]<<16) 
                      + (rx_data[data_cnt+3]<<24); 
         data_cnt += 4;
         loc->p_pos->y = rx_data[data_cnt] 
                      + (rx_data[data_cnt+1]<<8) 
                      + (rx_data[data_cnt+2]<<16) 
                      + (rx_data[data_cnt+3]<<24); 
         data_cnt += 4;
         loc->p_pos->z = rx_data[data_cnt] 
                      + (rx_data[data_cnt+1]<<8) 
                      + (rx_data[data_cnt+2]<<16) 
                      + (rx_data[data_cnt+3]<<24); 
         data_cnt += 4;
         loc->p_pos->qf = rx_data[data_cnt++];
      }
      
      if(rx_data[LOC_DATA_DIST_OFFSET]==DWM1001_TLV_TYPE_DIST)//0x48
      {
         // node is Anchor, recording Tag ID, distances and qf
         loc->anchors.dist.cnt = rx_data[LOC_DATA_DIST_OFFSET+2];
         loc->anchors.an_pos.cnt = 0;
         data_cnt = LOC_DATA_DIST_OFFSET + 3; // jump Type, Length and cnt, goto data
         for (i = 0; i < loc->anchors.dist.cnt; i++)
         {
            // Tag ID
            loc->anchors.dist.addr[i] = 0;
            for (j = 0; j < 8; j++)
            {
               loc->anchors.dist.addr[i] += rx_data[data_cnt++]<<(j*8);
            }
            // Tag distance
            loc->anchors.dist.dist[i] = 0;
            for (j = 0; j < 4; j++)
            {
               loc->anchors.dist.dist[i] += rx_data[data_cnt++]<<(j*8);
            }
            // Tag qf
            loc->anchors.dist.qf[i] = rx_data[data_cnt++];
         }
      }
      else if (rx_data[LOC_DATA_DIST_OFFSET]==DWM1001_TLV_TYPE_RNG_AN_POS_DIST)//0x49
      {
         // node is Tag, recording Anchor ID, distances, qf and positions
         loc->anchors.dist.cnt = rx_data[LOC_DATA_DIST_OFFSET+2];
         loc->anchors.an_pos.cnt = rx_data[LOC_DATA_DIST_OFFSET+2];
         data_cnt = LOC_DATA_DIST_OFFSET + 3; // jump Type, Length and cnt, goto data
         for (i = 0; i < loc->anchors.dist.cnt; i++)
         {
            // anchor ID
            loc->anchors.dist.addr[i] = 0;
            for (j = 0; j < 2; j++)
            {
               loc->anchors.dist.addr[i] += ((uint64_t)rx_data[data_cnt++])<<(j*8);
            }
            // anchor distance
            loc->anchors.dist.dist[i] = 0;
            for (j = 0; j < 4; j++)
            {
               loc->anchors.dist.dist[i] += ((uint32_t)rx_data[data_cnt++])<<(j*8);
            }
            // anchor qf
            loc->anchors.dist.qf[i] = rx_data[data_cnt++];
            // anchor position
            loc->anchors.an_pos.pos[i].x  = rx_data[data_cnt] 
                                         + (rx_data[data_cnt+1]<<8) 
                                         + (rx_data[data_cnt+2]<<16) 
                                         + (rx_data[data_cnt+3]<<24); 
            data_cnt += 4;
            loc->anchors.an_pos.pos[i].y = rx_data[data_cnt] 
                                         + (rx_data[data_cnt+1]<<8) 
                                         + (rx_data[data_cnt+2]<<16) 
                                         + (rx_data[data_cnt+3]<<24); 
            data_cnt += 4;
            loc->anchors.an_pos.pos[i].z = rx_data[data_cnt] 
                                         + (rx_data[data_cnt+1]<<8) 
                                         + (rx_data[data_cnt+2]<<16) 
                                         + (rx_data[data_cnt+3]<<24); 
            data_cnt += 4;
            loc->anchors.an_pos.pos[i].qf = rx_data[data_cnt++];
         }
      }
      else
      {
         return RV_ERR;   
      }
   }
   else
   {
      return RV_ERR;   
   }
   return RV_OK;
}

int dwm_baddr_set(dwm_baddr_t* p_baddr)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_BLE_ADDR_SET;
   tx_data[tx_len++] = 6;  
   tx_data[tx_len++] = p_baddr->byte[0];  
   tx_data[tx_len++] = p_baddr->byte[1];  
   tx_data[tx_len++] = p_baddr->byte[2];  
   tx_data[tx_len++] = p_baddr->byte[3];  
   tx_data[tx_len++] = p_baddr->byte[4];  
   tx_data[tx_len++] = p_baddr->byte[5];  
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_baddr_get(dwm_baddr_t* p_baddr)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_BLE_ADDR_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 11) == RV_OK)
   {
      data_cnt = 5;
      p_baddr->byte[0] = rx_data[data_cnt++];
      p_baddr->byte[1] = rx_data[data_cnt++];
      p_baddr->byte[2] = rx_data[data_cnt++];
      p_baddr->byte[3] = rx_data[data_cnt++];
      p_baddr->byte[4] = rx_data[data_cnt++];
      p_baddr->byte[5] = rx_data[data_cnt++];
      return RV_OK;
   }   
   return RV_ERR;
}


int dwm_reset(void)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_RESET;
   tx_data[tx_len++] = 0;  
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);   
}

int dwm_ver_get(dwm_ver_t* ver)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   uint8_t data_cnt;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_VER_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 21) == RV_OK)
   {
      data_cnt = 3;      
      // read fw_version
      if(rx_data[data_cnt++] != DWM1001_TLV_TYPE_FW_VER)
      {
         return RV_ERR;
      }
      if(rx_data[data_cnt++] != 4)
      {
         return RV_ERR;
      }
      ver->fw.res = (rx_data[data_cnt]>>4) & 0x0f;
      ver->fw.var = rx_data[data_cnt] & 0x0f;
      data_cnt++;
      ver->fw.patch = rx_data[data_cnt++];
      ver->fw.min = rx_data[data_cnt++];
      ver->fw.maj = rx_data[data_cnt++];
      // read cfg_version
      if(rx_data[data_cnt++] != DWM1001_TLV_TYPE_CFG_VER)
      {
         return RV_ERR;
      }
      if(rx_data[data_cnt++] != 4)
      {
         return RV_ERR;
      }
      ver->cfg = rx_data[data_cnt] 
              + (rx_data[data_cnt+1]<<8) 
              + (rx_data[data_cnt+2]<<16) 
              + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      // read hw_version
      if(rx_data[data_cnt++] != DWM1001_TLV_TYPE_HW_VER)
      {
         return RV_ERR;
      }
      if(rx_data[data_cnt++] != 4)
      {
         return RV_ERR;
      }
      ver->hw =  rx_data[data_cnt] 
              + (rx_data[data_cnt+1]<<8) 
              + (rx_data[data_cnt+2]<<16) 
              + (rx_data[data_cnt+3]<<24); 
      data_cnt += 4;
      return RV_OK;
   }   
   return RV_ERR;
}


int dwm_gpio_cfg_output(dwm_gpio_idx_t idx, bool value)
{      
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_CFG_OUTPUT;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = idx & 0xff;
   tx_data[tx_len++] = (uint8_t)value;       
   LMH_Tx(tx_data, &tx_len);      
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_gpio_cfg_input(dwm_gpio_idx_t idx, dwm_gpio_pin_pull_t pull_mode)
{      
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_CFG_INPUT;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = idx & 0xff;
   tx_data[tx_len++] = (uint8_t)pull_mode;      
   LMH_Tx(tx_data, &tx_len);    
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_gpio_value_set(dwm_gpio_idx_t idx, bool value)
{
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_VAL_SET;
   tx_data[tx_len++] = 2;
   tx_data[tx_len++] = idx & 0xff;
   tx_data[tx_len++] = (uint8_t)value;      
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_gpio_value_get(dwm_gpio_idx_t idx, bool* p_value)
{
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_VAL_GET;
   tx_data[tx_len++] = 1;
   tx_data[tx_len++] = idx & 0xff;  
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 6) == RV_OK)
   {
      *p_value = (bool)rx_data[5];
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_gpio_value_toggle(dwm_gpio_idx_t idx)
{
   if(LMH_CheckGPIOIdx(idx)!= RV_OK)
   {
      return RV_ERR;
   }
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_GPIO_VAL_TOGGLE;
   tx_data[tx_len++] = 1;
   tx_data[tx_len++] = idx & 0xff;  
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

int dwm_status_get(dwm_status_t* p_status)
{
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_STATUS_GET;
   tx_data[tx_len++] = 0;   
   LMH_Tx(tx_data, &tx_len);   
   if(LMH_WaitForRx(rx_data, &rx_len, 6) == RV_OK)
   {
      p_status->loc_data = (rx_data[5] & DWM_STATUS_LOC_DATA_READY_MASK)? 1:0;
      p_status->uwbmac_joined = (rx_data[5] & DWM_STATUS_UWBMAC_JOINED_MASK)? 1:0;
      return RV_OK;
   }   
   return RV_ERR;
}

int dwm_int_cfg(uint8_t value)
{        
   uint8_t tx_data[DWM1001_TLV_MAX_SIZE], tx_len = 0;
   uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
   uint16_t rx_len;
   tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_INT_CFG;
   tx_data[tx_len++] = 1;
   tx_data[tx_len++] = (uint8_t)value;    
   LMH_Tx(tx_data, &tx_len);   
   return LMH_WaitForRx(rx_data, &rx_len, 3);
}

