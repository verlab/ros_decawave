/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal_fprint.c
 * @brief   utility print to log file
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

static char log_file_name[]="log.txt";
static FILE * fp = NULL;   

/** 
 * @brief get log file pointer. 
 *
 * @param none
 *
 * @return log file pointer
 */
FILE * HAL_Fprint_GetFile(void)
{
   
   if(fp != NULL)
   {
      return fp;
   }
   printf("Opening log file %s\n", log_file_name); 
   fp = fopen (log_file_name, "w");
   
   return fp;
}

/** 
 * @brief de-initializes the log file. 
 *
 * @param none
 *
 * @return none
 */
void HAL_Fprint_DeInit(void)
{
   if(fp != NULL)
   {
      fclose(fp);  
      fp = NULL;
      printf("See %s for detailed information.\n", log_file_name); 
   }
}

/** 
 * @brief print report to  test_report.txt file. 
 *
 * @param formated strings
 *
 * @return none
 */
void HAL_Fprint(const char* format, ... )
{
   va_list args;
   va_start( args, format );
   vfprintf( HAL_Fprint_GetFile(), format, args );
   va_end( args );
   fflush(HAL_Fprint_GetFile());   
}
