/**
+ * LEAPS - Low Energy Accurate Positioning System.
 *
 * DWM API application public interface
 *
 * Copyright (c) 2016-2017, LEAPS. All rights reserved.
 *
 */

#ifndef DWM_H_
#define DWM_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief DWM Error codes
 */
#define	DWM_OK				(0)
#define DWM_ERR_INTERNAL	(-1)
#define DWM_ERR_BUSY		(-2)
#define DWM_ERR_INVAL_ADDR	(-3)
#define DWM_ERR_INVAL_PARAM	(-4)
#define DWM_ERR_OVERRUN		(-5)
#define DWM_ERR_I2C_ANACK	(-10)
#define DWM_ERR_I2C_DNACK	(-11)

/**
 * @brief Get system time
 *
 * @note Timer will overflow each 35 minutes
 *
 * @return Time in microseconds since restart
 */
uint32_t dwm_systime_us_get(void);

/**
 * @brief Resets DWM module
 */
void dwm_reset(void);

/**
 * @brief Puts device to sleep mode
 *
 * @return Error code
 * @retval DWM_OK success
 * @retval DWM_ERR_INTERNAL not in low power mode
 */
int dwm_sleep(void);

/**
 * @brief DWM status
 */
typedef struct dwm_status_t {
	bool loc_data;
	bool uwbmac_joined;
} dwm_status_t;

/**
 * @brief Reads DWM status
 *
 * @param[out] p_status
 *
 * @return Error code
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_status_get(dwm_status_t* p_status);

/**
 * @brief Firmware version data
 */
typedef struct dwm_fw_ver_t {
	uint8_t maj;
	uint8_t min;
	uint8_t patch;
	uint8_t res;
	uint8_t var;
}dwm_fw_ver_t;

/**
 * @brief Version data
 */
typedef struct dwm_ver_t {
	dwm_fw_ver_t fw;
	uint32_t cfg;
	uint32_t hw;
} dwm_ver_t;

/**
 * @brief Get firmware version, configuration version and hardware
 * version of the module.
 *
 * @param[out] p_ver pointer to version data
 *
 * @return Error code
 * @retval DWM_OK success
 * @retval DWM_ERR_INTERNAL failed to retrieve FW version
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_ver_get(dwm_ver_t* p_ver);

/**
 * @brief Position measurement modes
 */
typedef enum {
	DWM_MEAS_MODE_TWR = 0,//!< DWM_MEAS_MODE_TWR
	DWM_MEAS_MODE_TDOA = 1//!< DWM_MEAS_MODE_TDOA
} dwm_meas_mode_t;

/**
 * @brief Device modes
 */
typedef enum {
	DWM_MODE_TAG = 0,  //!< DWM_MODE_TAG
	DWM_MODE_ANCHOR = 1//!< DWM_MODE_ANCHOR
} dwm_mode_t;

typedef enum {
	DWM_UWB_MODE_OFF = 0,
	DWM_UWB_MODE_PASSIVE = 1,
	DWM_UWB_MODE_ACTIVE = 2
}dwm_uwb_mode_t;

typedef struct dwm_cfg_common {
	bool fw_update_en;
	dwm_uwb_mode_t uwb_mode;
	bool security_en;
	bool ble_en;
	bool led_en;
} dwm_cfg_common_t;

typedef struct dwm_cfg_anchor {
	dwm_cfg_common_t common;
	bool bridge;
	bool initiator;
} dwm_cfg_anchor_t;

typedef struct dwm_cfg_tag {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool accel_en;
	dwm_meas_mode_t meas_mode;
} dwm_cfg_tag_t;

typedef struct dwm_cfg {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool accel_en;
	dwm_meas_mode_t meas_mode;
	bool bridge;
	bool initiator;
	dwm_mode_t mode;
} dwm_cfg_t;

/**
 * @brief Configures node to anchor mode with given options
 *
 * @param[in] p_cfg Anchor configuration options
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL failed to set configuration
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_cfg_anchor_set(dwm_cfg_anchor_t* p_cfg);

/**
 * @brief Configures node to tag mode with given options
 *
 * @param[in] p_cfg Tag configuration options
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL failed to set configuration
 * @retval DWM_ERR_INVAL_PARAM Invalid measurement mode
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_cfg_tag_set(dwm_cfg_tag_t* p_cfg);

/**
 * @brief Reads configuration of the node
 *
 * @param[out] p_cfg Node configuration
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL failed to read configuration
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_cfg_get(dwm_cfg_t* p_cfg);

/**
 * @brief GPIO pins available for the user application
 */
typedef enum {
	DWM_GPIO_IDX_2 = 2,  //!< DWM_GPIO_IDX_2
	DWM_GPIO_IDX_8 = 8,  //!< DWM_GPIO_IDX_8
	DWM_GPIO_IDX_9 = 9,  //!< DWM_GPIO_IDX_9
	DWM_GPIO_IDX_10 = 10,  //!< DWM_GPIO_IDX_10
	DWM_GPIO_IDX_12 = 12,  //!< DWM_GPIO_IDX_12
	DWM_GPIO_IDX_13 = 13,  //!< DWM_GPIO_IDX_13
	DWM_GPIO_IDX_14 = 14,  //!< DWM_GPIO_IDX_14
	DWM_GPIO_IDX_15 = 15,  //!< DWM_GPIO_IDX_15
	DWM_GPIO_IDX_22 = 22,  //!< DWM_GPIO_IDX_22
	DWM_GPIO_IDX_23 = 23,  //!< DWM_GPIO_IDX_23
	DWM_GPIO_IDX_27 = 27,  //!< DWM_GPIO_IDX_27
	DWM_GPIO_IDX_30 = 30,  //!< DWM_GPIO_IDX_30
	DWM_GPIO_IDX_31 = 31 //!< DWM_GPIO_IDX_31
} dwm_gpio_idx_t;

/**
 * @brief Enumerator used for selecting the pin to be pulled down
 * or up at the time of pin configuration
 */
typedef enum {
	DWM_GPIO_PIN_NOPULL = 0,
	DWM_GPIO_PIN_PULLDOWN = 1,
	DWM_GPIO_PIN_PULLUP = 3
} dwm_gpio_pin_pull_t;

/**
 * @brief Interrupt type for GPIO pins
 */
typedef enum
{
	DWM_IRQ_TYPE_EDGE_RISING = 1,
	DWM_IRQ_TYPE_EDGE_FALLING = 2,
	DWM_IRQ_TYPE_EDGE_BOTH = 3
} dwm_gpio_irq_type_t;

/**
 * @brief Configures pin as output and sets the pin value
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @param[in] value Initial value of the output pin
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application
 */
int dwm_gpio_cfg_output(dwm_gpio_idx_t idx, bool value);

/**
 * @brief Configures pin as input and sets pull mode
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @param[in] pull_mode Pull mode of the input pin (see dwm_gpio_pin_pull_t)
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application or pull mode is not defined
 */
int dwm_gpio_cfg_input(dwm_gpio_idx_t idx, dwm_gpio_pin_pull_t pull_mode);

/**
 * @brief Sets value of the output pin
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @param[in] value Pin value (0, 1)
 *
 * @return Error code
 * @retval DWM_OK success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application
 */
int dwm_gpio_value_set(dwm_gpio_idx_t idx, bool value);

/**
 * @brief Gets value of the input pin
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @param[out] p_value Pointer to the pin value
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_gpio_value_get(dwm_gpio_idx_t idx, bool* p_value);

/**
 * @brief Toggles value of the output pin
 *
 * @param[in] idx Pin index (see dwm_gpio_idx_t)
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Pin is not available to the application
 */
int dwm_gpio_value_toggle(dwm_gpio_idx_t idx);

/**
 * @brief Interrupt callback
 *
 * @param[in] p_data User data
 */
typedef void dwm_gpio_cb_t(void* p_data);

/**
 * @brief Registers interrupt for GPIO pin
 *
 * @note callback is called in interrupt context
 *
 * @param[in] idx Pin index
 * @param[in] irq_type Interrupt type
 * @param[in] p_cb Pointer to interrupt callback
 * @param[in] p_data User data
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM pin is reserved, not available for application
 * @retval DWM_ERR_INTERNAL not possible to register pin, probably maximum number of registered pins has been reached
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_gpio_irq_cfg(dwm_gpio_idx_t idx, dwm_gpio_irq_type_t irq_type, dwm_gpio_cb_t* p_cb, void* p_data);

/**
 * @brief Disable interrupt on the GPIO pin
 *
 * @param[in] idx Pin index
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL pin is not registered
 * @retval DWM_ERR_INVAL_PARAM pin is reserved, not available for application
 */
int dwm_gpio_irq_dis(dwm_gpio_idx_t idx);

/* BLE address length */
#define DWM_BLE_ADDR_LEN 6

/**
 * @brief BLE address
 */
typedef struct {
	uint8_t byte[DWM_BLE_ADDR_LEN];
}dwm_baddr_t;

/**
 * @brief Sets BLE address
 *
 * @param[in] p_baddr Pointer to BLE address structure
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_INTERNAL address can't be set
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_baddr_set(dwm_baddr_t* p_baddr);

/**
 * @brief Gets BLE address
 *
 * @param[out] p_baddr Pointer to BLE address structure
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_INTERNAL address can't be read
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_baddr_get(dwm_baddr_t* p_baddr);

/**
 * @brief Position coordinates in millimeters + quality factor
 */
typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
	uint8_t qf;
} dwm_pos_t;

/**
 * @brief Sets position of anchor node
 *
 * @param[in] p_pos Position coordinates
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Position can't be written
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_pos_set(dwm_pos_t* p_pos);

/**
 * @brief Gets position of the node
 *
 * @param[out] p_pos Pointer to position
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Position can't be read
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_pos_get(dwm_pos_t* p_pos);

/**
 * @brief Sets position of anchor node, can be used when one does not
 * want to set all coordinates
 *
 * @param[in] p_x Pointer to x coordinate (millimeters), ignored if null
 * @param[in] p_y Pointer to y coordinate (millimeters), ignored if null
 * @param[in] p_z Pointer to z coordinate (millimeters), ignored if null
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Position can't be read
 */
int dwm_pos_set_xyz(int32_t* p_x, int32_t* p_y, int32_t* p_z);

/* Maximum count of nodes in location data */
#define DWM_LOC_CNT_MAX	15

/**
 * @brief Distances of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	uint64_t addr[DWM_LOC_CNT_MAX];
	uint32_t dist[DWM_LOC_CNT_MAX];
	uint8_t qf[DWM_LOC_CNT_MAX];
}dwm_distance_t;

/**
 * @brief Position of ranging anchors
 */
typedef struct {
	uint8_t cnt;
	dwm_pos_t pos[DWM_LOC_CNT_MAX];
}dwm_anchor_pos_t;

/**
 * @brief Distances and position of ranging anchors
 */
typedef struct {
	dwm_distance_t dist;
	dwm_anchor_pos_t an_pos;
}dwm_ranging_anchors_t;

/**
 * @brief Location data (position of current node and list of positions
 * and distances of ranging anchors)
 */
typedef struct dwm_loc_data_t {
	dwm_pos_t* p_pos;
	dwm_ranging_anchors_t anchors;
} dwm_loc_data_t;

/**
 * @brief Gets location data
 *
 * @param[out] p_loc Pointer to location data, if NULL function returns
 * with no error and call is ignored
 *
 * @note if p_loc.p_pos is NULL the location data are not read without returning an error
 *
 * @note if p_loc.p_pos is NULL the position is not read and no error is returned
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL can't read location data or there are none
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_loc_get(dwm_loc_data_t* p_loc);

/* maximum and minimum update rate in multiple of 100 ms */
enum dwm_upd_rate{
	DWM_UPD_RATE_MAX = 600,	/* 1 minute */
	DWM_UPD_RATE_MIN = 1	/* 100 ms */
};

/**
 * @brief Sets update rate
 *
 * @param[in] ur Update rate in multiply of 100 ms, [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 * @param[in] urs Stationary update rate in multiply of 100 ms, [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't write the value
 * @retval DWM_ERR_INVAL_PARAM Values not within the limits
 */
int dwm_upd_rate_set(const uint16_t ur, const uint16_t urs);

/**
 * @brief Gets update rate
 *
 * @param[out] p_ur Pointer to update rate, update rate is multiply of 100 ms
 * [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 * @param[out] p_urs Pointer to stationary update rate, update rate is multiply of 100 ms
 * [min,max] = [DWM_UPD_RATE_MIN, DWM_UPD_RATE_MAX]
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INTERNAL Can't read update rate
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_upd_rate_get(uint16_t* p_ur, uint16_t* p_urs);

/**
 * @brief Read data from I2C slave.
 *
 * @param[in] addr Address of a slave device (only 7 LSB)
 * @param[out] p_data Pointer to a receive buffer
 * @param[in] len Number of bytes to be received
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_BUSY     If the driver is not ready for a new transfer.
 * @retval DWM_ERR_INTERNAL      If an error was detected by hardware.
 * @retval DWM_ERR_I2C_ANACK If NACK received after sending the address.
 * @retval DWM_ERR_I2C_DNACK If NACK received after sending a data byte.
 * @retval DWM_ERR_OVERRUN If the unread data was replaced by new data.
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_i2c_read(uint8_t addr, uint8_t* p_data, uint8_t len);

/**
 * @brief Write data to I2C slave.
 *
 * @param[in] addr Address of a slave device (only 7 LSB)
 * @param[in] p_data Pointer to a transmit buffer
 * @param[in] len Number of bytes to be send
 * @param[in] no_stop If set, the stop condition is not generated
 * on the bus after the transfer has completed successfully
 * (allowing for a repeated start in the next transfer)
 *
 * @return Error code
 * @retval DWM_OK Success
 *
 * @retval DWM_ERR_BUSY      If the driver is not ready for a new transfer.
 * @retval DWM_ERR_INTERNAL       If an error was detected by hardware.
 * @retval DWM_ERR_I2C_ANACK If NACK received after sending the address.
 * @retval DWM_ERR_I2C_DNACK If NACK received after sending a data byte.
 * @retval DWM_ERR_INVAL_ADDR If the EasyDMA is used and memory address in not in RAM.
 * @retval DWM_ERR_INVAL_ADDR null pointer supplied
 */
int dwm_i2c_write(uint8_t addr, uint8_t* p_data, uint8_t len, bool no_stop);

/**
 * @brief DWM events IDs
 */
typedef enum {
	DWM_EVT_NEW_LOC_DATA = 0,//!< DWM_EVT_NEW_LOC_DATA
/* new events TBD*/
}dwm_evt_id_t;

/**@brief DWM Event header. */
typedef struct
{
  dwm_evt_id_t id;
  uint16_t len;
} dwm_evt_hdr_t;

/**@brief Common DWM Event type, wrapping the module specific event reports. */
typedef struct {
	dwm_evt_hdr_t header;
	union {
		dwm_loc_data_t loc;
		/* accelerometer data TBD */
	} data;
} dwm_evt_t;

/**
 * @brief Callback to be called when new location data are available
 *
 * @param[in] p_loc Pointer to location data
 * @param[in] p_data User data
 */
typedef void dwm_evt_cb_t(dwm_evt_t* p_evt, void* p_data);

/**
 * @brief Registers callback for new location data
 *
 * @param[in] cb Pointer to the callback, NULL is ignored
 * @param[in] p_data User data
 */
void dwm_evt_cb_register(dwm_evt_cb_t* cb, void* p_data);

/**
 * @brief thread entry function prototype
 * */
typedef void dwm_thread_entry_t(uint32_t);

/**
 * @brief Creates a new thread, at most 5 threads are allowed
 *
 * @param[in] prio Thread priority, values from interval <20,30> are allowed
 * @param[in] entry Thread entry function
 * @param[in] p_data User data
 * @param[in] name Thread name
 * @param[in] stack_size Stack size in bytes
 * @param[out] p_hndl Thread handle
 *
 * @return Error code 0 if sucess
 * @retval DWM_ERR_INTERNAL maximum number of threads has been reached
 * @retval DWM_ERR_INVAL_PARAM invalid priority
 */
int dwm_thread_create(uint8_t prio, dwm_thread_entry_t *entry, void* p_data,
		char *name, uint16_t stack_size, uint8_t* p_hndl);

/**
 * @brief Resumes suspended thread
 *
 * @param[in] hndl Thread handle
 *
 * @return Error code
 */
int dwm_thread_resume(uint8_t hndl);

/**
 * @brief Suspends thread
 *
 * @param[in] hndl Thread handle
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Invalid handle
 */
int dwm_thread_suspend(uint8_t hndl);

/**
 * @brief Deletes the thread, free the stack memory
 *
 * @param[in] hndl Thread handle
 *
 * @return Error code
 * @retval DWM_OK Success
 * @retval DWM_ERR_INVAL_PARAM Invalid handle
 * @retval DWM_ERR_INTERNAL Thread was not deleted
 */
int dwm_thread_delete(uint8_t hndl);

/**
 * @brief Allows a thread to suspend until the specified number of clock
 * ticks have occurred
 *
 * @param[in[ delay Delay in clock ticks, 1 tick is 10 ms
 */
void dwm_thread_delay(uint64_t delay);

/*mutex data type*/
typedef struct dwm_mutex_t dwm_mutex_t;

/**
 * @brief Initializes mutex
 *
 * @param[in] p_mutex Pointer to the mutex
 */
void dwm_mutex_init(dwm_mutex_t* p_mutex);

/**
 * @brief Locks the mutex
 *
 * @param[in] p_mutex Pointer to the mutex
 */
void dwm_mutex_lock(dwm_mutex_t* p_mutex);

/**
 * @brief Unlocks the mutex
 *
 * @param[in] p_mutex Pointer to the mutex
 */
void dwm_mutex_unlock(dwm_mutex_t* p_mutex);

typedef struct dwm_cond_t dwm_cond_t;

/**
 * @brief Initializes the condition variable. A condition
 * variable is attached to a specific mutex.
 *
 * @param[in] p_cond Pointer to condition variable
 * @param[in] p_mutex Pointer to the mutex
 */
void dwm_cond_init(dwm_cond_t* p_cond, dwm_mutex_t* p_mutex);

/**
 * @brief Causes the current thread to wait on the condition variable, while
 * simultaneously unlocking the corresponding mutex. dwm_cond_wait() may be
 * called by a thread which has the corresponding mutex locked.
 *
 * @param[in] p_cond Pointer to condition variable
 * @return "true" if no error, "false" otherwise
 */
bool dwm_cond_wait(dwm_cond_t* p_cond);

/**
 * @brief Wakes up at least one thread which is waiting on the condition variable
 * . When a thread is awakened it will become the owner of the mutex. dwm_cond_signal()
 * may be called by the thread which currently owns the mutex to which the condition
 * variable is attached.
 *
 * @param[in] p_cond Pointer to condition variable
 */
void dwm_cond_signal(dwm_cond_t* p_cond);

/**
 * @brief Destroys the condition variable. This must not be done on a condition
 * variable which is in use. After it has been destroyed, it may be subsequently
 * reinitialized.
 *
 * @param[in] p_cond Pointer to condition variable
 */
void dwm_cond_destroy(dwm_cond_t* p_cond);

/* semaphore data type */
typedef struct dwm_sem_t dwm_sem_t;

/**
 * @brief Initializes a semaphore. The initial semaphore count is set to val.
 *
 * @param[in] p_sem Pointer to the semaphore
 * @param[in] val Initial value of the semaphore
 */
void dwm_semaphore_init(dwm_sem_t* p_sem, uint32_t val);

/**
 * @brief If the semaphore count is zero, the current thread will wait on the
 *  semaphore. If the count is non-zero, it will be decremented and the
 *  thread will continue running.
 *
 * @param[in] p_sem Pointer to the semaphore
 */
void dwm_semaphore_wait(dwm_sem_t* p_sem);

/**
 * @brief If there are threads waiting on this semaphore this will wake
 * exactly one of them. Otherwise it simply increments the semaphore count.
 *
 * @param[in] p_sem Pointer to the semaphore
 */
void dwm_semaphore_post(dwm_sem_t* p_sem);

/**
 * @brief Destroys a semaphore. This must not be done while there are any
 * threads waiting on it.
 *
 * @param[in] p_sem Pointer to the semaphore
 */
void dwm_semaphore_destroy(dwm_sem_t* p_sem);

/* handle data type */
typedef struct dwm_hndl_t dwm_hndl_t;

/* mailbox data type */
typedef struct dwm_mbox_t dwm_mbox_t;

/**
 * @brief Creates a mailbox
 *
 * @param[out] p_hndl Handle object
 * @param[in] p_mbox Mailbox object
 */
void dwm_mbox_create(dwm_hndl_t* p_hndl, dwm_mbox_t* p_mbox);

/**
 * @brief Deletes a mailbox
 *
 * @param[in] hndl Handle object
 */
void dwm_mbox_delete(dwm_hndl_t hndl);

/**
 * @brief This places a message into an mailbox. If the mailbox is already full
 * this function will block until the message is placed into the mbox. If the thread
 * is awaken by the kernel during a wait, this function will return an error.
 *
 * @param[in] hndl Handle object
 * @param[in] p_item Pointer to be placed in the mailbox
 *
 * @return "true" if the message was placed into the mailbox, "false" otherwise
 */
bool dwm_mbox_put(dwm_hndl_t hndl, void* p_item);

/**
 * @brief Reads number of messages in mailbox
 *
 * @param[in] hndl Handle object
 *
 * @return Error code The number of messages currently in the given mailbox.
 */
uint32_t dwm_mbox_peek(dwm_hndl_t hndl);

/**
 * @brief This reads a pointer from an mailbox. If the mailbox has no data in it, this
 * function will block the calling thread until data is available.
 *
 * @param[in] hndl Handle object
 *
 * @return Pointer to data that was placed in the mailbox.
 */
void* dwm_mbox_get(dwm_hndl_t hndl);

/**
 * @brief Enables BLE for compilation
 */
void dwm_ble_compile(void);

/**
 * @brief Enables location engine for compilation
 */
void dwm_le_compile(void);

/**
 * @brief Enables API on SPI, UART interfaces for compilation
 */
void dwm_serial_compile(void);

/**
 * @brief Enables SHELL for compilation
 */
void dwm_shell_compile(void);

#endif /* DWM_H_ */
