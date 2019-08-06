/*! ------------------------------------------------------------------------------------------------------------------
 * @file    anchor_cfg.c
 * @brief   Decawave device configuration and control functions
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
 
#include "dwm_api.h"
#include "hal.h"
      
int main(void)
{
   dwm_cfg_anchor_t cfg_anchor;
   dwm_cfg_t cfg_node;
   
   HAL_Print("dwm_init(): dev%d\n", HAL_DevNum());
   dwm_init();
   
   HAL_Print("Setting to anchor: dev%d.\n", HAL_DevNum());
   cfg_anchor.initiator = 1; 
   cfg_anchor.bridge = 0;
   cfg_anchor.common.led_en = 1;
   cfg_anchor.common.ble_en = 1;
   cfg_anchor.common.uwb_mode = DWM_UWB_MODE_OFF;
   cfg_anchor.common.fw_update_en = 1;
   HAL_Print("dwm_cfg_anchor_set(&cfg_anchor): dev%d.\n", HAL_DevNum());
   dwm_cfg_anchor_set(&cfg_anchor);

   HAL_Print("Wait 2s for node to reset\n");
   HAL_Delay(2000);
   
   HAL_Print("Getting configurations: dev%d.\n", HAL_DevNum());
   dwm_cfg_get(&cfg_node);
      
   HAL_Print("Comparing set vs. get: dev%d.\n", HAL_DevNum());
   if((cfg_anchor.initiator != cfg_node.initiator) 
   || (cfg_anchor.bridge != cfg_node.bridge) 
   || (cfg_anchor.common.led_en != cfg_node.common.led_en) 
   || (cfg_anchor.common.ble_en != cfg_node.common.ble_en) 
   || (cfg_anchor.common.uwb_mode != cfg_node.common.uwb_mode) 
   || (cfg_anchor.common.fw_update_en != cfg_node.common.fw_update_en))
   {
      HAL_Print("initiator           cfg_anchor=%d : cfg_node=%d\n", cfg_anchor.initiator, cfg_node.initiator); 
      HAL_Print("bridge              cfg_anchor=%d : cfg_node=%d\n", cfg_anchor.bridge, cfg_node.bridge); 
      HAL_Print("common.led_en       cfg_anchor=%d : cfg_node=%d\n", cfg_anchor.common.led_en, cfg_node.common.led_en); 
      HAL_Print("common.ble_en       cfg_anchor=%d : cfg_node=%d\n", cfg_anchor.common.ble_en, cfg_node.common.ble_en); 
      HAL_Print("common.uwb_mode     cfg_anchor=%d : cfg_node=%d\n", cfg_anchor.common.uwb_mode, cfg_node.common.uwb_mode); 
      HAL_Print("common.fw_update_en cfg_anchor=%d : cfg_node=%d\n", cfg_anchor.common.fw_update_en, cfg_node.common.fw_update_en); 
      HAL_Print("\nConfiguration failed.\n\n");
   }
   else
   {
      HAL_Print("\nConfiguration succeeded.\n\n");
   }
   
   return 0;   
}
