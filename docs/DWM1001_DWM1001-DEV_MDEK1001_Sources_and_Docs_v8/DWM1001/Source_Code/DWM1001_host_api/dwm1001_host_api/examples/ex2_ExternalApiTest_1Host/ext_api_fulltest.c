/*! ------------------------------------------------------------------------------------------------------------------
 * @file    ext_api_fulltest.c
 * @brief   Decawave device configuration and control functions
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <string.h>
#include "dwm_api.h"
#include "hal.h"
#include "../test_util/test_util.h"


void example_external_api_fulltest(void)
{   
   int rv, err_cnt = 0;  
   int i, j;
   uint8_t gpio_idx_list[]={
      DWM_GPIO_IDX_2,DWM_GPIO_IDX_8,DWM_GPIO_IDX_9,
      DWM_GPIO_IDX_10,DWM_GPIO_IDX_12,DWM_GPIO_IDX_13,
      DWM_GPIO_IDX_14,DWM_GPIO_IDX_15,DWM_GPIO_IDX_22,
      DWM_GPIO_IDX_23,DWM_GPIO_IDX_27,
      DWM_GPIO_IDX_30,DWM_GPIO_IDX_31 };
   bool gpio_val_list[]={false, true};
   uint8_t gpio_pull_list[]={DWM_GPIO_PIN_NOPULL,DWM_GPIO_PIN_PULLDOWN,DWM_GPIO_PIN_PULLUP};
   
   HAL_Print("Initializing...\n");
   dwm_init();
   HAL_Print("Done\n");
   Test_Report("external_api_fulltest over interface: %s\n", HAL_IF_STR);
   
   HAL_Print("dwm_pos_set(pos_set)\n");
   dwm_pos_t pos_set;
   pos_set.qf = 100;
   pos_set.x = 121;
   pos_set.y = 50;
   pos_set.z = 251;
   rv = Test_CheckTxRx(dwm_pos_set(&pos_set)); 
   Test_Report("dwm_pos_set(&pos_set):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   dwm_pos_t pos_get;
   HAL_Print("dwm_pos_get(&pos_get)\n");
   rv = Test_CheckTxRx(dwm_pos_get(&pos_get));
   if(rv == RV_OK)
   {
      HAL_Print("\t\tpos_get.x = %d\n", pos_get.x);
      HAL_Print("\t\tpos_get.y = %d\n", pos_get.y);
      HAL_Print("\t\tpos_get.z = %d\n", pos_get.z);
      HAL_Print("\t\tpos_get.qf = %d\n", pos_get.qf);
   }
   Test_Report("dwm_pos_get(&pos_get):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   dwm_loc_data_t loc;
   dwm_pos_t pos;
   loc.p_pos = &pos;
   HAL_Print("dwm_loc_get(&loc)\n");
   rv = Test_CheckTxRx(dwm_loc_get(&loc));
   if(rv == RV_OK)
   {
      HAL_Print("[%d,%d,%d,%u]\n", loc.p_pos->x, loc.p_pos->y, loc.p_pos->z,
            loc.p_pos->qf);

      for (i = 0; i < loc.anchors.dist.cnt; ++i) 
      {
         HAL_Print("%u)", i);
         HAL_Print("0x%llx", loc.anchors.dist.addr[i]);
         if (i < loc.anchors.an_pos.cnt) 
         {
            HAL_Print("[%d,%d,%d,%u]", loc.anchors.an_pos.pos[i].x,
                  loc.anchors.an_pos.pos[i].y,
                  loc.anchors.an_pos.pos[i].z,
                  loc.anchors.an_pos.pos[i].qf);
         }
         HAL_Print("=%u,%u\n", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);
      }
   }
   Test_Report("dwm_loc_get(&loc):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;   
   
   uint16_t ur_set = 22;
   uint16_t ur_s_set = 50;
   HAL_Print("dwm_upd_rate_set(%d, %d)\n", ur_set, ur_s_set);
   rv = Test_CheckTxRx(dwm_upd_rate_set(ur_set, ur_s_set));
   Test_Report("dwm_upd_rate_set(ur, ur_s):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   uint16_t ur_get, ur_s_get;
   HAL_Print("dwm_upd_rate_get(&ur, &ur_s)\n");
   rv = Test_CheckTxRx(dwm_upd_rate_get(&ur_get, &ur_s_get));
   if(rv == RV_OK)
   {
      HAL_Print("\t\tur  =%d \n", ur_get);
      HAL_Print("\t\tur_s=%d \n", ur_s_get);
   }
   Test_Report("dwm_upd_rate_get(&ur, &ur_s):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
    
   dwm_cfg_tag_t cfg_tag;
   cfg_tag.accel_en = 1;
   cfg_tag.low_power_en = 0; 
   cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
   cfg_tag.loc_engine_en = 1;
   cfg_tag.common.led_en = 0;
   cfg_tag.common.ble_en = 0;
   cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
   cfg_tag.common.fw_update_en = 0;
   HAL_Print("dwm_cfg_tag_set(&cfg_tag)\n");
   rv = Test_CheckTxRx(dwm_cfg_tag_set(&cfg_tag));
   Test_Report("dwm_cfg_tag_set(&cfg_tag):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   HAL_Delay(1000);

   dwm_cfg_anchor_t cfg_an;    
   cfg_an.initiator = 1;
   cfg_an.bridge = 0;
   cfg_an.common.led_en = 0;
   cfg_an.common.ble_en = 1;
   cfg_an.common.uwb_mode = DWM_UWB_MODE_OFF;
   cfg_an.common.fw_update_en = 1;
   HAL_Print("dwm_cfg_anchor_set(&cfg_an)\n");
   rv = Test_CheckTxRx(dwm_cfg_anchor_set(&cfg_an));
   Test_Report("dwm_cfg_anchor_set(&cfg_an):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv;    
   
   HAL_Delay(1000);
   
   rv = Test_CheckTxRx(dwm_reset());
   Test_Report("dwm_reset():\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   HAL_Print("Wait 1s for node to reset.\n");
   HAL_Delay(1000);
    
   dwm_cfg_t cfg_node;    
   HAL_Print("dwm_cfg_get(&cfg_node):\n");
   rv = Test_CheckTxRx(dwm_cfg_get(&cfg_node));
   if(rv == RV_OK)
   {  
      HAL_Print("\t\tcfg_node.common.fw_update_en= %d\n", cfg_node.common.fw_update_en);
      HAL_Print("\t\tcfg_node.common.uwb_mode    = %d\n", cfg_node.common.uwb_mode);
      HAL_Print("\t\tcfg_node.common.ble_en      = %d\n", cfg_node.common.ble_en);
      HAL_Print("\t\tcfg_node.common.led_en      = %d\n", cfg_node.common.led_en);
      HAL_Print("\t\tcfg_node.loc_engine_en      = %d\n", cfg_node.loc_engine_en);
      HAL_Print("\t\tcfg_node.low_power_en       = %d\n", cfg_node.low_power_en);
      HAL_Print("\t\tcfg_node.accel_en           = %d\n", cfg_node.accel_en);
      HAL_Print("\t\tcfg_node.meas_mode          = %d\n", cfg_node.meas_mode);
      HAL_Print("\t\tcfg_node.bridge             = %d\n", cfg_node.bridge);
      HAL_Print("\t\tcfg_node.initiator          = %d\n", cfg_node.initiator);
      HAL_Print("\t\tcfg_node.mode               = %d\n", cfg_node.mode);
   }   
   Test_Report("dwm_cfg_get(&cfg_node):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   dwm_ver_t ver; 
   HAL_Print("dwm_ver_get(&ver)\n");
   rv = Test_CheckTxRx(dwm_ver_get(&ver));
   if(rv == RV_OK)
   {
      HAL_Print("\t\tver.fw.maj  = %d\n", ver.fw.maj);
      HAL_Print("\t\tver.fw.min  = %d\n", ver.fw.min);
      HAL_Print("\t\tver.fw.patch= %d\n", ver.fw.patch);
      HAL_Print("\t\tver.fw.res  = %d\n", ver.fw.res);
      HAL_Print("\t\tver.fw.var  = %d\n", ver.fw.var);
      HAL_Print("\t\tver.cfg     = %08x\n", ver.cfg);
      HAL_Print("\t\tver.hw      = %08x\n", ver.hw);
   }
   Test_Report("dwm_ver_get(&ver):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   rv = 0;
   HAL_Print("dwm_gpio_cfg_input:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      for(j = 0; j < sizeof(gpio_pull_list); j++)
      {     
         HAL_Print("dwm_gpio_cfg_input(%d, %d)\n", gpio_idx_list[i], gpio_pull_list[j]);
         rv += Test_CheckTxRx(dwm_gpio_cfg_input(gpio_idx_list[i], gpio_pull_list[j]));
      }
   }   
   Test_Report("dwm_gpio_cfg_input:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   rv = 0;
   HAL_Print("dwm_gpio_cfg_output:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      for(j = 0; j < sizeof(gpio_val_list); j++)
      {      
         HAL_Print("dwm_gpio_cfg_output(%d, %d)\n", gpio_idx_list[i], gpio_val_list[j]);
         rv += Test_CheckTxRx(dwm_gpio_cfg_output(gpio_idx_list[i], gpio_val_list[j]));
      }
   }
   Test_Report("dwm_gpio_cfg_output:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
      
   rv = 0;
   HAL_Print("dwm_gpio_value_set:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      for(j = 0; j < sizeof(gpio_val_list); j++)
      {     
         HAL_Print("dwm_gpio_value_set(%d, %d)\n", gpio_idx_list[i], gpio_pull_list[j]);
         rv += Test_CheckTxRx(dwm_gpio_value_set(gpio_idx_list[i], gpio_pull_list[j]));   
      }
   }
   Test_Report("dwm_gpio_value_set:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   rv = 0;
   static bool value_gpio_value_get;
   HAL_Print("dwm_gpio_value_get:\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      HAL_Print("dwm_gpio_value_get(%d, &val)\n", gpio_idx_list[i]);
      rv += Test_CheckTxRx(dwm_gpio_value_get(gpio_idx_list[i], &value_gpio_value_get));
   }
   Test_Report("dwm_gpio_value_get:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   rv = 0;
   HAL_Print("dwm_gpio_value_toggle\n");
   for(i = 0; i < sizeof(gpio_idx_list); i++)
   {
      HAL_Print("dwm_gpio_value_toggle(%d, &val)\n", gpio_idx_list[i]);
      rv += Test_CheckTxRx(dwm_gpio_value_toggle(gpio_idx_list[i])); 
   }
   Test_Report("dwm_gpio_value_toggle:\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   dwm_status_t status;
   HAL_Print("dwm_status_get(&status)\n");
   rv = Test_CheckTxRx(dwm_status_get(&status));
   if(rv == RV_OK)
   {
      HAL_Print("\t\tstatus.loc_data      = %d\n", status.loc_data);
      HAL_Print("\t\tstatus.uwbmac_joined = %d\n", status.uwbmac_joined);
   }
   Test_Report("dwm_status_get(&status):\t\t\t%s\n", rv==0 ? "pass":"fail");
   err_cnt += rv; 
   
   uint8_t value_int_cfg = DWM1001_INTR_SPI_DATA_READY;
   HAL_Print("dwm_int_cfg(%d)\n", value_int_cfg);
   Test_CheckTxRx(dwm_int_cfg(value_int_cfg));
   HAL_Print("dwm_reset()\n");
   Test_CheckTxRx(dwm_reset());
   HAL_Print("Wait 1s for node to reset.\n");
   HAL_Delay(1000);
   
   Test_End();
}

int main(int argc, char*argv[])
{   
   int k=1;
   while(k-->0)
   {
      example_external_api_fulltest();
   }
   return 0;
}

