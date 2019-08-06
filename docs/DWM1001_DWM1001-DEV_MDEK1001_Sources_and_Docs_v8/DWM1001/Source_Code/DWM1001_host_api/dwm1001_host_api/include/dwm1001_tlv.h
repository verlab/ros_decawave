#ifndef _DWM1001_TLV_H_
#define _DWM1001_TLV_H_

#include <stdbool.h>
#include <stdint.h>

#define DWM1001_TLV_MAX_SIZE           255
#define DWM1001_TLV_RET_VAL_MIN_SIZE   3
/****************************************************************************//**
 * @brief definitions for TLV Type byte
**/
#define DWM1001_TLV_TYPE_UNKNOWN			      0	/* unknown TLV type */
#define DWM1001_TLV_TYPE_CMD_POS_SET			1	/* request set position coordinates XYZ */
#define DWM1001_TLV_TYPE_CMD_POS_GET			2	/* request get position coordinates XYZ */
#define DWM1001_TLV_TYPE_CMD_UR_SET			   3	/* request set position update rate*/
#define DWM1001_TLV_TYPE_CMD_UR_GET			   4	/* request get position update rate*/
#define DWM1001_TLV_TYPE_CMD_CFG_TN_SET		5	/* request set configuration for the tag */
#define DWM1001_TLV_TYPE_CMD_CFG_AN_SET		7	/* request set configuration for the anchor */
#define DWM1001_TLV_TYPE_CMD_CFG_GET			8	/* request get configuration data */
#define DWM1001_TLV_TYPE_CMD_CFG_SAVE			9	/* request save configuration data */
#define DWM1001_TLV_TYPE_CMD_SLEEP			   10	/* request sleep */
#define DWM1001_TLV_TYPE_CMD_LOC_GET		   12	/* request location get */
#define DWM1001_TLV_TYPE_CMD_BLE_ADDR_SET		15	/* request BLE address set */
#define DWM1001_TLV_TYPE_CMD_BLE_ADDR_GET		16	/* request BLE address get */
#define DWM1001_TLV_TYPE_CMD_RESET			   20	/* request reset */
#define DWM1001_TLV_TYPE_CMD_VER_GET			21	/* request FW version get */
#define DWM1001_TLV_TYPE_CMD_UWB_CFG_ANT_SET	30	/* request set uwb antenna tx, rx delay */
#define DWM1001_TLV_TYPE_CMD_UWB_CFG_ANT_GET	31	/* request get uwb antenna tx, rx delay */
#define DWM1001_TLV_TYPE_CMD_GPIO_CFG_OUTPUT	40	/* request configure output pin and set */
#define DWM1001_TLV_TYPE_CMD_GPIO_CFG_INPUT	41	/* request configure input pin */
#define DWM1001_TLV_TYPE_CMD_GPIO_VAL_SET		42	/* request set pin value */
#define DWM1001_TLV_TYPE_CMD_GPIO_VAL_GET		43	/* request get pin value */
#define DWM1001_TLV_TYPE_CMD_GPIO_VAL_TOGGLE	44	/* request toggle pin value */
#define DWM1001_TLV_TYPE_CMD_STATUS_GET		50	/* request status get */
#define DWM1001_TLV_TYPE_CMD_INT_CFG			52	/* request configure interrupts */
#define DWM1001_TLV_TYPE_CMD_N_POS_SET		   128	/* nested request set position */
//#define DWM1001_TLV_TYPE_CMD_N_LOC_GET		   130	/* nested request location get */
#define DWM1001_TLV_TYPE_RET_VAL			      64	/* request return value (as the response) */
#define DWM1001_TLV_TYPE_POS_XYZ			      65	/* position coordinates x,y,z*/
#define DWM1001_TLV_TYPE_POS_X			      66	/* position coordinate x */
#define DWM1001_TLV_TYPE_POS_Y			      67	/* position coordinate y */
#define DWM1001_TLV_TYPE_POS_Z			      68	/* position coordinate z */
#define DWM1001_TLV_TYPE_UR				      69	/* update rate */
#define DWM1001_TLV_TYPE_CFG				      70	/* configuration data */
#define DWM1001_TLV_TYPE_DIST				      72	/* distances */
#define DWM1001_TLV_TYPE_RNG_AN_POS_DIST	   73	/* ranging anchor distances and positions*/
#define DWM1001_TLV_TYPE_FW_VER				   80	/* fw_version */
#define DWM1001_TLV_TYPE_CFG_VER				   81	/* cfg_version */
#define DWM1001_TLV_TYPE_HW_VER				   82	/* hw_version */
#define DWM1001_TLV_TYPE_PIN_VAL			      85	/* pin value */
#define DWM1001_TLV_TYPE_STATUS			      90	/* status */
#define DWM1001_TLV_TYPE_IDLE                255 /* Dummy byte, or type: idle */


#endif //_DWM1001_TLV_H_
