####################################################
# @file    Makefile
#
# @attention
#
# Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
#
# All rights reserved.
#

####################################################
#  Configurations

####################################################
#  TARGET
#  0: Raspberry-Pi
#  1: else
TARGET = 0

####################################################
#  INTERFACE_NUMBER
#  0: USE_UART  
#  1: USE_SPI    
#  2: USE_SPI_DRDY
INTERFACE_NUMBER = 1

####################################################
#  PRINT_LVL 
#  for   HAL_Print         
#  0:    none      
#  1:    fprintf (log.txt)       
#  2:    printf (terminal)  
#  3:    printf + fprintf    
PRINT_LVL = 3


PROGRAM = tag_cfg
SOURCES = tag_cfg.c

PROJ_DIR += ../../..

CFLAGS += -Wall

include $(PROJ_DIR)/include/dwm1001.mak
