/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 /*INCLUDES*/
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mfd/npm1300.h>
#include <zephyr/logging/log.h>
#include "remote.h"

/*NPM1300 PMIC REGISTERS*/
#define NPM1300_I2C_ADDR 0x6B
#define NPM1300_BASE_SHPHLD 0x0BU
#define NPM1300_OFFSET_TASK_ENTER_SHIP_MODE 0x02
#define NPM1300_SHPHLD_MODE 0X01


/*BNO055 REGISTERS*/
#define BNO055_I2C_ADDR 0x28
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_CALIBRATION_ADDR 0x35
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_GRAVITY_DATA_X_LSB_ADDR 0x2E
#define BNO055_OFFSET_ACCEL_X_LSB 0x55

/*BNO055 OPERATING MODES*/
#define BNO055_MODE_CONFIG 0x00
#define BNO055_MODE_NDOF 0x0C
#define BNO055_CRYSTAL_ENABLE 0x80
#define BNO055_PAGE_ID_0 0x00

/*THRESHOLDS VALUES FOR DATA PROCESSING*/
#define STARTING_OFFSET_BIN 10

static const double ANGULAR_THRESHOLDS_MEAN[6] = {0.758, 0.767, 0.775, 0.789, 0.810, 0.815};
static const double ANGULAR_THRESHOLDS_STD[6] = {0.010328, 0.020028, 0.029533, 0.032472, 0.014907, 0.012693};

static const double DEPTH5_THRESHOLDS_MEAN[6] = {1.495, 1.719, 2.061, 2.323, 2.684, 3.026};
static const double DEPTH5_THRESHOLDS_STD[6] = {0.031710, 0.028848, 0.085434, 0.089944, 0.062397, 0.089219};
static const double DEPTH6_THRESHOLDS_MEAN[6] = {1.821, 2.151, 2.668, 2.924, 3.386, 3.886};
static const double DEPTH6_THRESHOLDS_STD[6] = {0.070938, 0.057822, 0.078853, 0.099465, 0.062929, 0.089094};

/*SAMPLING PARAMETERS*/
#define SAMPLE_FREQUENCY 75
#define SAMPLE_SIZE 512
#define FFT_SIZE 512
#define FREQUENCY_RESOLUTION 0.1465f // 75Hz // 512 samples
#define SAMPLING_INTERVAL 13333 // us (75Hz)

/*DEVICE HANDLES FROM DEVICE TREE*/
const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_pmic)); 

const struct device *dev_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/*I2C READ/WRITE BUFFERS*/
static uint8_t buffer_i2c_read[8];
static uint8_t buffer_i2c_write[2];

/*IRS*/
#define DEBOUNCE 200		// Debounce time in milliseconds
#define LONG_DEBOUNCE 4000 	// Long debounce time in milliseconds

#define BUTTON0_NODE DT_ALIAS(button0)
#define BUTTON1_NODE DT_ALIAS(button1)

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios);

static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;

/*WORK QUEUES*/
static struct k_work_delayable button0_work;
static struct k_work_delayable button1_work;

/*BNO055 STATUS AND CALRIBRATION OFFSETS STRUCTURE*/
typedef struct {
	uint16_t device_id;
	uint16_t mag_cal;
	uint16_t acc_cal;
	uint16_t gyr_cal;
	uint16_t sys_cal;
	bool calibrated;
} bno055_status;

typedef struct {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
        int16_t mag_x;
        int16_t mag_y;
        int16_t mag_z;
} calibration_offsets;

calibration_offsets bno055_offsets;
bno055_status bno055_calibration_status;

/*DATA AQUISITION BUFFERS AND VARIABLES*/
int sample_count = 0;
int buffer_index1 = 0;
int buffer_index2 = 0;
bool using_buffer1 = true;

float buffer1[SAMPLE_SIZE];
float buffer2[SAMPLE_SIZE];

float buffer1_dp[SAMPLE_SIZE];
float buffer2_dp[SAMPLE_SIZE];

float fft_input[FFT_SIZE];
float fft_output[FFT_SIZE];
float fft_magnitude[FFT_SIZE/2];

float orientation_input[SAMPLE_SIZE];
float dot_product[SAMPLE_SIZE];

/*RESULT DATA AND MEANING*/
struct result_data {
    int frequency_bin;
	int depth;
	int direction;
};

const int bpm_ranges[] = {88, 97, 106, 114, 123, 133};
const char *direction_strings[] = {"Not perpendicular", "Perpendicular"};
const char *depth_strings[] = {"<5cm", "5-6cm", ">6cm"};

/*SEMAPHORES*/
K_SEM_DEFINE(acc_sem, 0, 1);
K_SEM_DEFINE(buffer1_sem, 0, 1);
K_SEM_DEFINE(buffer2_sem, 0, 1);

/*TIMER FOR PERIODIC SENSOR READING*/	
uint32_t interval_sum = 0;
static void sensor_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(sensor_timer, sensor_timer_handler, NULL);

/*K_MUTEX*/
struct k_mutex buffer1_mutex;
struct k_mutex buffer2_mutex;

/*FFT*/
arm_rfft_fast_instance_f32 fft_inst;

/*BLE VARIABLES AND CALLBACKS*/
static struct bt_conn *current_conn;
static bool advertising = false;
static bool device_connected;
static bool subscribed;
static void on_connected(struct bt_conn *conn, uint8_t err);
static void on_disconnected(struct bt_conn *conn, uint8_t reason);
static void on_notif_changed(enum bt_freq_notifications_enabled status);

struct bt_conn_cb bluetooth_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};

struct bt_remote_srv_cb remote_service_callbacks = {
	.notif_changed = on_notif_changed,

};

/*NVS PARTITION CONFIGURATION*/
#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET 0xF8000 // not using FIXED_PARTITION_OFFSET(NVS_PARTITION), offset hardcoded due to macro issue
#define NVS_PARTITION_SIZE FIXED_PARTITION_SIZE(NVS_PARTITION)
#define NVS_SECTOR_SIZE 4096
#define NVS_SECTOR_COUNT 8

static struct nvs_fs fs = {
	.flash_device = NVS_PARTITION_DEVICE,
	.offset = NVS_PARTITION_OFFSET,
	.sector_size = NVS_SECTOR_SIZE,
	.sector_count = NVS_SECTOR_COUNT,
};

/*FUNCTION PROTOTYPES*/
static void on_connected(struct bt_conn *conn, uint8_t err);
static void on_disconnected(struct bt_conn *conn, uint8_t reason);
static void on_notif_changed(enum bt_freq_notifications_enabled status);
static void ble_init(void);
static void enter_ship_mode(void);
static void button0_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void button1_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void button0_work_handler(struct k_work *work);
static void button1_work_handler(struct k_work *work);
static void set_interrupts(void);
static void read_deviceID(void);
static void set_configMode(void);
static void set_pageId0(void);
static void set_externalCrystal(void);
static void set_opMode(void);
static void read_calibration(void);
static bool read_linearAcceleration_xyz(float *acc_x, float *acc_y, float *acc_z);
static bool read_gravity_xyz(float *grv_x, float *grv_y, float *grv_z);
static void store_accelerationSample(float sample, float dot_product);
static void normalize(float *x, float *y, float *z);
static float compute_dot_product(float acc_x, float acc_y, float acc_z, float grv_x, float grv_y, float grv_z);
static void acquisition_thread(void);
static int get_frequency_bin(float *input, int size);
static void get_magnitude(float *input, float *output, int size);
static bool check_orientation (float mean_dp, int bin);
static int get_depth(float mean_acc, int bin);
static void processing_thread(void);
static void fft(float32_t *input, float32_t *output);
static void init_nvs(void);
static int store_offset_calibration(int id, int16_t value);
static int16_t read_offset_calibration(int id);
static void store_all_offsets(calibration_offsets *offsets);
static void read_all_offsets(calibration_offsets *offsets);
static int set_offsets_bno055(calibration_offsets *offsets);
static int read_offsets_bno055(calibration_offsets *offsets);
static void bno_calibration (bool search_for_offsets);
static void visualization_thread(void);



/*THREAD DEFINITION*/
K_THREAD_DEFINE (acq_tid,1024,acquisition_thread,NULL,NULL,NULL,5,0,0);
K_THREAD_DEFINE (proc_tid,2048,processing_thread,NULL,NULL,NULL,6,0,0);
K_THREAD_DEFINE (vis_tid,2048,visualization_thread,NULL,NULL,NULL,7,0,0);

/*MESSAGE QUEUE DEFINITION*/
K_MSGQ_DEFINE(result_msgq, sizeof(struct result_data), 10, 4); 

/* LOG MODULE*/
LOG_MODULE_REGISTER(my_module);

/*BLE FUNCTIONS*/

// Callback function called when a BLE connection is established
static void on_connected(struct bt_conn *conn, uint8_t err){

	if(err){
		LOG_ERR("Connection err: %d", err);
		return;
	}
	LOG_INF("BLE Device connected.");
	device_connected = true;	// Update flag to indicate the device is connected
	current_conn = bt_conn_ref(conn);	// Store a reference to the current connection
	
}

// Callback function called when a BLE connection is disconnected
static void on_disconnected(struct bt_conn *conn, uint8_t reason){

	LOG_INF("Device disconnected. Reason: %d", reason);
	if (current_conn) {
		bt_conn_unref(current_conn);	// Release the reference to the current connection
		current_conn = NULL;	// Clear the connection pointer
		LOG_INF("Connection unreferenced.");
	} else {
		LOG_INF("No current connection to unreference.");
	}
	device_connected = false;	// Update the flag to reflect disconnection status
	subscribed = false;	  // Reset the notification subscription flag
}

// Callback function called when the notification subscription status changes
static void on_notif_changed(enum bt_freq_notifications_enabled status){

	if (status == BT_FREQ_NOTIFICATIONS_ENABLED) {
		LOG_INF("Notifications enabled.\n");
		subscribed = true;
	} else {
		LOG_INF("Notifications disabled.\n");
		subscribed = false;
	}
}

// Initializes the Bluetooth subsystem and registers the required callbacks
static void ble_init(){

	LOG_INF("bleuetooth initialization...\n");
	int err = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);	// Initialize Bluetooth with callback handlers
	if (err) {
		LOG_ERR("Bluetooth initialization failed: %d\n", err);
	}else {
		LOG_INF("Bluetooth initialized successfully.\n");

	}
}

/*ISR FUNCTIONS*/

// Powers down the device by putting the PMIC into ship mode.
// Enter ship mode by commanding the PMIC to cut power to the board, shutting down the device completely.
// This will stop the program execution, and the device will only restart
// when power is restored by physically pressing the SHPHLD pin on the PMIC.
static void enter_ship_mode(void){

	LOG_INF("Entering ship mode...");
	k_sleep(K_MSEC(1000)); 
	// Send command via I2C to the PMIC to enter ship mode
	int ret = mfd_npm1300_reg_write(pmic,NPM1300_BASE_SHPHLD, NPM1300_OFFSET_TASK_ENTER_SHIP_MODE, NPM1300_SHPHLD_MODE);
	LOG_INF("RET: %d", ret);
	if (ret < 0) {
		LOG_ERR("Failed to enter ship mode: %d", ret);
		return;
	}
}

// Interrupt Service Routine (ISR) triggered when button0 is pressed
static void button0_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins){

		if (!(pins & BIT(button0.pin))) return;		// Check if the interrupt is for the specific button pin
	gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_DISABLE);	// Disable further interrupts on button0 to prevent bouncing issues
    k_work_schedule(&button0_work, K_MSEC(DEBOUNCE));	// Schedule the button0 work handler to run after a debounce delay
}


// Interrupt Service Routine (ISR) triggered when button1 is pressed
static void button1_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins){

    if (!(pins & BIT(button1.pin))) return;		// Check if the interrupt is for the specific button pin    
	gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_DISABLE);	// Disable further interrupts on button1 to prevent bouncing issues
	k_work_schedule(&button1_work, K_MSEC(DEBOUNCE));	// Schedule the button0 work handler to run after a debounce delay
	// No need to re-enable the interrupt here because the device will shut down, 
	// and the ISR will be automatically re-registered when the device restarts.
}			

// Work handler executed after button0 press debounce delay
static void button0_work_handler(struct k_work *work){
	// Suspend the aquisition and processing threads 
	k_thread_suspend(acq_tid);
	k_thread_suspend(proc_tid);
	LOG_INF("OFF button pressed!\n");
	enter_ship_mode();	// Enter ship mode to reduce power consumption 
}

// Work handler executed after button1 press debounce delay
static void button1_work_handler(struct k_work *work){

	LOG_INF("BLE/CLB button pressed!\n");
	k_sleep(K_MSEC(LONG_DEBOUNCE));		// Wait for debounce duration to ensure stable input	
	// Suspend threads to safely perform operations without interference
	k_thread_suspend(acq_tid);
	k_thread_suspend(proc_tid);
	// Check if button1 is still pressed 
	if(gpio_pin_get_dt(&button1) == 1){
		// if Button1 held, start calibration
		LOG_INF("BLE/CLB button held, starting calibration...\n");
		k_sleep(K_MSEC(3000));	 // Wait 3 seconds to allow user to stop pressing the button
		bno_calibration(false);		// Perform calibration routine with no search for offsets in NVS
		LOG_INF("Calibration finished, restarting threads...");
		k_sleep(K_MSEC(3000));
	} else {
		// if Button1 released, start/stop BLE
		LOG_INF("BLE/CLB button released!\n");
		if (advertising) {
			//If advertising is active, stop it and disconnect any current connection if exists
			LOG_INF("Disconnecting device and stoping advertising...\n");
			int err;
			err = bluetooth_stop_advertising();		// Stop BLE advertising
			if (err) {
				LOG_ERR("Failed to stop advertising: %d\n", err);
			} else {
				LOG_INF("Advertising stopped successfully.\n");
				advertising = false;
			}
			if (current_conn) {
				bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);		// Disconnect current connection if exists
			} else {
				LOG_INF("No current connection to disconnect.");
			}	
		} else {
			// If advertising is not active, start it
			LOG_INF("inicialitzant comunicació BLE...\n");
			int err;
			err = bluetooth_start_advertising();
			if (err) {
					LOG_INF("Bluetooth advertising failed: %d\n", err);
        	}
			LOG_INF("Starting advertising...\n");
			advertising = true;
		}
	}
	// Resume the aquisition and processing threads
	k_thread_resume(acq_tid);
	k_thread_resume(proc_tid);
	gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);	 // Re-enable interrupts for button1	
}

// Configure GPIO pins and interrupts for button0 and button1
static void set_interrupts(void){

	int err;
	// Check if the GPIO devices for button0 and button1 are ready
    if (!device_is_ready(button0.port) || !device_is_ready(button1.port)) {
        LOG_ERR("Button device not ready");
        return;
    }
	// Configure button0 as input with internal pull-up resistor
    err = gpio_pin_configure_dt(&button0, GPIO_INPUT | GPIO_PULL_UP);
    if (err != 0) {
		LOG_ERR("Error configuring button 0: %d", err);
        return;
    }
	// Configure button1 as input with internal pull-up resistor
    err = gpio_pin_configure_dt(&button1, GPIO_INPUT | GPIO_PULL_UP);
    if (err != 0) {
        LOG_ERR("EError configuring button 1\n");
        return;
    }
	// Configure interrupt on button0: trigger on falling edge (button press)
    err = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
    if (err != 0) {
        LOG_ERR("Error configuring ISR 0\n");
        return;
    }
	// Configure interrupt on button1: trigger on falling edge (button press)
    err = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
    if (err != 0) {
        LOG_ERR("Error configuring ISR 1\n");
        return;
    }
	// Initialize and register callback for button0
    gpio_init_callback(&button0_cb_data, button0_pressed_isr, BIT(button0.pin));
    gpio_add_callback(button0.port, &button0_cb_data);
	// Initialize and register callback for button1
    gpio_init_callback(&button1_cb_data, button1_pressed_isr, BIT(button1.pin));
    gpio_add_callback(button1.port, &button1_cb_data);
	// Initialize delayed work items associated with each button
	k_work_init_delayable(&button0_work, button0_work_handler);
	k_work_init_delayable(&button1_work, button1_work_handler);

	LOG_INF("Interrupts configured successfully for buttons 0 and 1.\n");
}

/*TIMER FUNCTIONS*/

// Timer handler callback triggered when the sensor timer expires
// This function releases the semaphore to signal that sensor data can be processed
static void sensor_timer_handler(struct k_timer *timer){
	// Release the semaphore to unblock aquisition thread at a fixed frequency = SAMPLE_FREQUENCY or Period = sampel_interval
    k_sem_give(&acc_sem);	
}


/* BNO055 FUNCTIONS*/

// Reads the device ID from the BNO055 sensor over I2C
static void read_deviceID(void){
	buffer_i2c_write[0] = BNO055_CHIP_ID_ADDR;	// Set the register address to read from (BNO055 chip ID register)
	int err;
	// Perform an I2C write (register address) followed by a read (1 byte) from the sensor
	err = i2c_write_read(dev_i2c, BNO055_I2C_ADDR, buffer_i2c_write, 1, buffer_i2c_read, 1);
	if(err<0){
		LOG_ERR("READ_DEVICE_ID failed: %d\n", err);
	}
	else{
		bno055_calibration_status.device_id = buffer_i2c_read[0];	// Store the received device ID in the calibration status struct
		LOG_INF("Chip ID: %d\n", bno055_calibration_status.device_id);
	}
}

// Sets the BNO055 sensor into configuration mode
static void set_configMode(void){
	// Prepare the buffer: first byte is the operation mode register address,
    // second byte is the value for configuration mode
	buffer_i2c_write[0] = BNO055_OPR_MODE_ADDR;
	buffer_i2c_write[1] = BNO055_MODE_CONFIG;
	int err;
	 // Send a 2-byte I2C write to set the BNO055 to configuration mode
	err = i2c_write(dev_i2c, buffer_i2c_write, 2, BNO055_I2C_ADDR);
	if(err<0){
		LOG_ERR("SET_CONFIG_MODE failed: %d\n", err);
	}
	else{
		LOG_INF("BNO055 set to CONFIG_MODE\n");
	}
}

// Sets the BNO055 sensor's register page ID to 0
static void set_pageId0(void){
	// Prepare the buffer: first byte is the PAGE_ID register address,
    // second byte is the value to to select page 0
	buffer_i2c_write[0] = BNO055_PAGE_ID_ADDR;
	buffer_i2c_write[1] = BNO055_PAGE_ID_0;
	int err;
	// Send a 2-byte I2C write to set the register page to 0
	err = i2c_write(dev_i2c, buffer_i2c_write, 2, BNO055_I2C_ADDR);
	if(err<0){
		LOG_ERR("SET_PAGE_ID_0 failed: %d\n", err);
	}
	else{
		LOG_INF("Set page id to 0\n");
	}
}

// Enables the use of an external crystal oscillator on the BNO055
static void set_externalCrystal(void){
	// Prepare the I2C buffer: first byte is the SYS_TRIGGER register address,
    // second byte is the value to enable external crystal
	buffer_i2c_write[0] = BNO055_SYS_TRIGGER_ADDR;
	buffer_i2c_write[1] = BNO055_CRYSTAL_ENABLE;
	int err;
	// Write 2 bytes to the BNO055 to enable external crystal
	err = i2c_write(dev_i2c, buffer_i2c_write, 2, BNO055_I2C_ADDR);
	if(err<0){
		LOG_ERR("SET_EXTERNAL_CRYSTAL failed: %d\n", err);
	}
	else{
		LOG_INF("Using external crystal\n");
	}
}

// Sets the BNO055 sensor to NDOF (fusion) operation mode
static void set_opMode (void){
    // Prepare the I2C write buffer:first byte is the address of the operation mode register,
    // second byte is the value to set NDOF mode (9-axis sensor fusion)
	buffer_i2c_write[0] = BNO055_OPR_MODE_ADDR;
	buffer_i2c_write[1] = BNO055_MODE_NDOF; 
	int err;
	// Send the 2-byte I2C write to configure the sensor mode
	err = i2c_write(dev_i2c, buffer_i2c_write, 2, BNO055_I2C_ADDR);
	if(err<0){
		LOG_ERR("SET_OP_MODE failed: %d\n", err);
	}
	else{
		LOG_INF("BNO055 set to NDOF mode\n");
	}
}

// Reads the calibration status from the BNO055 sensor
static void read_calibration (void){

	buffer_i2c_write[0] = BNO055_CALIBRATION_ADDR;	// Set the register address to read calibration status from
	int err;
	// Perform an I2C write-read operation:
    // Write 1 byte (register address), then read 1 byte (calibration status)
	err = i2c_write_read(dev_i2c, BNO055_I2C_ADDR, buffer_i2c_write, 1, buffer_i2c_read, 1);
	if(err<0){
		LOG_ERR("READ_CALIBRATION failed: %d", err);
	}
	else{
		// Extract individual calibration statuses from the returned byte
		// Each calibration status uses 2 bits:
        // bits 1:0 = magnetometer calibration
        // bits 3:2 = accelerometer calibration
        // bits 5:4 = gyroscope calibration
        // bits 7:6 = system calibration
		bno055_calibration_status.mag_cal = (buffer_i2c_read[0] >> 0) & 0x03;
		bno055_calibration_status.acc_cal = (buffer_i2c_read[0] >> 2) & 0x03;
		bno055_calibration_status.gyr_cal = (buffer_i2c_read[0] >> 4) & 0x03;
		bno055_calibration_status.sys_cal = (buffer_i2c_read[0] >> 6) & 0x03;

		LOG_INF("Status of calibration: mag=%d acc=%d gyr=%d sys=%d\n", bno055_calibration_status.mag_cal, bno055_calibration_status.acc_cal, bno055_calibration_status.gyr_cal, bno055_calibration_status.sys_cal);
		// Check if all calibration bits are set (0xFF), meaning fully calibrated
		if (buffer_i2c_read[0]== 0xFF){
			bno055_calibration_status.calibrated = true;
			LOG_INF("DEVICE CALIBRATED\n");
			}
		else{
			bno055_calibration_status.calibrated = false;
			k_msleep(1000); 
			read_calibration();		// Recursive call to retry reading calibration
		}
	}
}

// Reads the linear acceleration data (X, Y, Z axes) from the BNO055 sensor via I2C
// Parameters: acc_x, acc_y, acc_z - pointers to floats where the acceleration values will be stored (in m/s²)
// Returns: true if the reading was successful, false otherwise
static bool read_linearAcceleration_xyz (float *acc_x, float *acc_y, float *acc_z){

	buffer_i2c_write[0] = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;	// Register address to start reading linear acceleration data
	int err;
	// Write 1 byte (register address), then read 6 bytes (2 bytes per axis: LSB and MSB for X, Y, Z)
	err = i2c_write_read(dev_i2c, BNO055_I2C_ADDR, buffer_i2c_write, 1, buffer_i2c_read, 6);
	if (err<0){
		LOG_ERR("READ_LINEAR_ACCEL failed: %d\n", err);
		return false;
	}
	// Combine LSB and MSB for each axis to form a signed 16-bit integer
	// The raw sensor data is scaled by a factor of 100
	// Dividing by 100 converts the raw value into meters per second squared (m/s²) as a float
	*acc_x = (int16_t)((buffer_i2c_read[1] << 8 | buffer_i2c_read[0])) / 100.0f;
	*acc_y = (int16_t)((buffer_i2c_read[3] << 8 | buffer_i2c_read[2])) / 100.0f;
	*acc_z = (int16_t)((buffer_i2c_read[5] << 8 | buffer_i2c_read[4])) / 100.0f;
	
	return true;
}

// Reads the gravity vector data (X, Y, Z axes) from the BNO055 sensor via I2C
// Parameters:
// grv_x, grv_y, grv_z - pointers to floats where the gravity values will be stored (in m/s²)
// Returns: true if the reading was successful, false otherwise
static bool read_gravity_xyz (float *grv_x, float *grv_y, float *grv_z){

	buffer_i2c_write[0] = BNO055_GRAVITY_DATA_X_LSB_ADDR;	// Register address to start reading gravity data
	int err;
	// Perform an I2C write-read operation:
	// Write 1 byte (register address), then read 6 bytes (2 bytes per axis: LSB and MSB for X, Y, Z)
	err = i2c_write_read(dev_i2c, BNO055_I2C_ADDR, buffer_i2c_write, 1, buffer_i2c_read, 6);
	if (err<0){
		LOG_ERR("READ_GRAVITY failed: %d\n", err);
		return false;
	}
	// Combine LSB and MSB for each axis to form a signed 16-bit integer
	// The raw sensor data is scaled by a factor of 100
	// Dividing by 100 converts the raw value into meters per second squared (m/s²) as a float
	*grv_x = (int16_t)((buffer_i2c_read[1] << 8 | buffer_i2c_read[0])) / 100.0f;
	*grv_y = (int16_t)((buffer_i2c_read[3] << 8 | buffer_i2c_read[2])) / 100.0f;
	*grv_z = (int16_t)((buffer_i2c_read[5] << 8 | buffer_i2c_read[4])) / 100.0f;
	
	return true;
}


/*AQUISITION FUCNTIONS*/

// Stores an acceleration sample and its dot product in a buffer with thread-safe access
// Parameters: 
// sample - acceleration data sample (float)
// dot_product - associated dot product value (float)
static void store_accelerationSample(float sample, float dot_product){

	bool buffer_ready = false;	// Flag indicating if the buffer is full and ready for processing
	if(using_buffer1){
		k_mutex_lock(&buffer1_mutex, K_FOREVER);	// Lock mutex to protect buffers1 from concurrent access
		buffer1[buffer_index1] = sample;
		buffer1_dp[buffer_index1] = dot_product;
		buffer_index1++;
		if (buffer_index1 >= SAMPLE_SIZE) {		// Check if the buffer is full
			buffer_index1 = 0;
			buffer_ready = true;
			
		} 
		k_mutex_unlock(&buffer1_mutex);		// Unlock mutex to allow other accesses
		
		if(buffer_ready){
			k_sem_give(&buffer1_sem);	//Give semaphore to processing thread
		}
	}
	else{
		k_mutex_lock(&buffer2_mutex, K_FOREVER);	// Lock mutex to protect buffers2 from concurrent access
		buffer2[buffer_index2] = sample;
		buffer2_dp[buffer_index2] = dot_product;
		buffer_index2++;
		if (buffer_index2 >= SAMPLE_SIZE) {		// Check if the buffer is full
			buffer_index2= 0;
			buffer_ready = true;
			
		} 
		k_mutex_unlock(&buffer2_mutex);		// Unlock mutex to allow other accesses
		if(buffer_ready){
			k_sem_give(&buffer2_sem);	//Give semaphore to processing thread
		}
	}
}

// Normalizes a 3D vector (x, y, z) to have a magnitude of 1 (unit vector)
// Parameters:
//   x, y, z - pointers to the vector components to be normalized
static void normalize (float *x, float *y, float *z){
	
	float norm = sqrtf((*x)*(*x) + (*y)*(*y) + (*z)*(*z));// Calculate the Euclidean norm (magnitude)
	if (norm > 0.0f){	// Avoid division by zero: only normalize if norm is greater than 0
		*x /= norm;
		*y /= norm;
		*z /= norm;
	}
}

// Computes the absolute value of the dot product between two 3D vectors:
// linear acceleration (acc_x, acc_y, acc_z) and gravity (grv_x, grv_y, grv_z).
// Both vectors are normalized before the dot product calculation.
// Parameters:
// acc_x, acc_y, acc_z - components of the acceleration vector
// grv_x, grv_y, grv_z - components of the gravity vector
// Returns: Absolute value of the dot product between the normalized vectors
static float compute_dot_product (float acc_x, float acc_y, float acc_z, float grv_x, float grv_y, float grv_z){

	normalize(&acc_x, &acc_y, &acc_z);
	normalize(&grv_x, &grv_y, &grv_z);
	float product = fabsf(acc_x*grv_x + acc_y*grv_y + acc_z*grv_z);

	return product;
}

// Thread that reads acceleration and gravity data from the BNO055 sensor,
// calculates magnitude and dot product, stores samples and dot product in buffers.
static void acquisition_thread(void){
	
	while(true){
		if(!bno055_calibration_status.calibrated){	// Check if the sensor is calibrated before starting data acquisition
			k_sleep(K_MSEC(1000));
		}
		else{
			interval_sum = 0;
			sample_count = 0;
			uint32_t past_measureTime = 0;
			// Collect SAMPLE_SIZE samples
			while (sample_count < SAMPLE_SIZE){
				k_sem_take(&acc_sem, K_FOREVER);	 // Wait until timer expires and semaphore is given

				//Calculate the time interval since the last sample to check sampling rate
				uint32_t current_measureTime = k_uptime_get_32();
				uint32_t interval_measureTime = current_measureTime - past_measureTime;

				if (sample_count > 0) {
					interval_sum += interval_measureTime;
				}
					
				float acc_x, acc_y, acc_z, grv_x, grv_y, grv_z;
				bool data_acceleration = read_linearAcceleration_xyz(&acc_x, &acc_y, &acc_z); 	//Read linear acceleration data from BNO055
				bool data_grvity = read_gravity_xyz(&grv_x, &grv_y, &grv_z);	// Read gravity  data from BNO055
				float vector = sqrtf(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
				float dp = compute_dot_product(acc_x, acc_y, acc_z, grv_x, grv_y, grv_z);	
				store_accelerationSample(vector, dp);	
				sample_count++;
				past_measureTime = current_measureTime;
			}
			// Compute the average sampling interval and sampling rate
			float avg_interval_ms = interval_sum / (float)(SAMPLE_SIZE-1);
			float sampling_rate = 1000.0f / avg_interval_ms; // Hz
			LOG_INF("Sapling rate: %.2f Hz\n", sampling_rate);
			// k_sleep(K_MSEC(200));

		}
	}
}

/* PROCESSING FUNCTIONS*/

// Finds the frequency bin with the highest magnitude within the range of interest
// Parameters:
// input - pointer to an array containing the magnitudes of the FFT result (float*)
// size - size of the input array (not used in this implementation, but kept for generality)
// Returns: The index of the frequency bin with the highest magnitude in the specified range
static int get_frequency_bin(float *input, int size){

	int min_bin = 10;	// Minimum bin index to check (corresponds to ~1.465 Hz / 87.90 BPM)
	int max_bin = 15;	// Maximum bin index to check (corresponds to ~2.1975 Hz / 131.85 BPM)
	int max_index = 10;
	float max_magnitude = 0;
	for(int i = min_bin; i <= max_bin; i++){
		if (input[i] > max_magnitude){
			max_magnitude = input[i];
			max_index = i;
		}
	}
	return max_index;
}

// Computes the magnitude spectrum from a complex FFT output
// Parameters:
// input - pointer to the FFT output array (float*), containing interleaved real and imaginary parts
// output - pointer to the output array where magnitudes will be stored (float*)
// size - total number of float elements in the input array (must be even, since it's interleaved)
//
// Only the first half of the FFT result is used
static void get_magnitude(float *input, float *output, int size){
	for(int i = 0; i < size/2; i++){
		output[i] = sqrtf(input[2*i]*input[2*i] + input[2*i+1]*input[2*i+1]);
	}
}

// Checks if the detected orientation is valid based on mean dot product and frequency bin
// Parameters:
// mean_dp - mean value of the dot product between acceleration and gravity vectors (float)
// bin - frequency bin index where  peak was found (int)
// Returns: true if the orientation is valid for the given bin, false otherwise
// 
// The function uses predefined angular thresholds (mean and standard deviation) 
// to determine if the orientation (based on dot product) falls within expected range.
// The index is adjusted based on STARTING_OFFSET_BIN, which aligns bin positions with thresholds.
static bool check_orientation (float mean_dp, int bin){
	int index = bin - STARTING_OFFSET_BIN;

	return (mean_dp > (ANGULAR_THRESHOLDS_MEAN[index] - 1.25*ANGULAR_THRESHOLDS_STD[index]));

}

// Estimates the depth category of a chest compression based on mean acceleration and frequency bin
// Parameters:
// mean_acc - mean vector magnitude of linear acceleration samples (float)
// bin - frequency bin index where the peak was detected (int)
// Returns:
// 0 - depth below 5 cm (insufficient)
// 1 - depth between 5 and 6 cm (correct)
// 2 - depth greater than 6 cm ( excessive)
//
// Uses predefined threshold arrays (mean and standard deviation) for each frequency bin to classify depth.
// The index is aligned with thresholds using STARTING_OFFSET_BIN.
// Thresholds correspond to empirical data for 5 cm and 6 cm compression depths.
//The accuracy depends on the output of chech_orientation(), which should be true for a better estimation of depth.
static int get_depth(float mean_acc, int bin){
	int index = bin - STARTING_OFFSET_BIN;
	if (mean_acc > DEPTH5_THRESHOLDS_MEAN[index] - 1.25*DEPTH5_THRESHOLDS_STD[index]){
		if (mean_acc > DEPTH6_THRESHOLDS_MEAN[index] + 1.25*DEPTH6_THRESHOLDS_STD[index]){
				return 2; 
			} else {
			return 1;
		} 
	} else {
		return 0;
	};
}

// Thread that processes acceleration and dot product data to extract compression frequency, depth, and alignment
static void processing_thread(void){
	while(true){
		int err;
		if(!bno055_calibration_status.calibrated){
			k_sleep(K_MSEC(100));
			continue;
		}

		float* buffer_pointer_ac = NULL;		//Pointer to acceleration buffer
		float* buffer_pointer_dp = NULL;		//Pointer to dot product buffer
		struct k_mutex *target_mutex = NULL;	//Pointer to the mutex for the selected buffer

		// Try to take semaphore from buffer1 or buffer2 without blocking
		if (k_sem_take(&buffer1_sem, K_NO_WAIT) == 0) {
			buffer_pointer_ac = buffer1;
			buffer_pointer_dp = buffer1_dp;
			target_mutex = &buffer1_mutex;
		} 
		else if (k_sem_take(&buffer2_sem, K_NO_WAIT) == 0) {
			buffer_pointer_ac = buffer2;
			buffer_pointer_dp = buffer2_dp;
			target_mutex = &buffer2_mutex;
		} 
		else {
			k_sleep(K_MSEC(10));
			continue;
		}

		if (buffer_pointer_ac != NULL && buffer_pointer_dp != NULL) { //If valid buffers were obtained
			k_mutex_lock(target_mutex, K_FOREVER);
			
			//Copy data from the selected buffer to internal arrays
			for (int i = 0; i < SAMPLE_SIZE; i++) {
				fft_input[i] = buffer_pointer_ac[i];
				orientation_input[i] = buffer_pointer_dp[i];
				
			}
			k_mutex_unlock(target_mutex);
			//Compute mean dot product and mean acceleration
			float sum_dp = 0;
			for (int i = 0; i < SAMPLE_SIZE; i++) {
				sum_dp += orientation_input[i];
			}
			float sum_acc = 0;
			for (int i = 0; i < SAMPLE_SIZE; i++) {
				sum_acc += fft_input[i];
			}
			float mean_acc = sum_acc / SAMPLE_SIZE;
			float mean_dp = sum_dp / SAMPLE_SIZE;

			//Compute FFT on the acceleration data
			fft(fft_input, fft_output);
			get_magnitude(fft_output, fft_magnitude, SAMPLE_SIZE);
			// Find highest bin, check orientation and depth
			int max_bin = get_frequency_bin(fft_magnitude, SAMPLE_SIZE);
			bool aligned = check_orientation(mean_dp, max_bin);
			float bpm = max_bin * FREQUENCY_RESOLUTION * 60.0f; // Convert to bpm
			int depth = get_depth(mean_acc, max_bin);
			LOG_INF("Depth: %d", depth);
			int direction = aligned ? 1 : 0;
			struct result_data result;
			result.frequency_bin = max_bin;
			result.depth = depth;
			result.direction = direction;

			k_msgq_put(&result_msgq, &result, K_FOREVER);
			// if(device_connected && subscribed){// If device is connected and subscribed to notifications
			// 	set_freq_value(max_bin - STARTING_OFFSET_BIN);
			// 	set_direction_value(direction);
			// 	set_depth_value(depth);
			// 	send_freq_notification(current_conn, bpm); 
			// 	send_direction_notification(current_conn, direction);
			// 	send_depth_notification(current_conn, depth);
			// }
			// else{ // If device is not connected send data to screen
			// 	LOG_INF("Depth: %.2d, Mean DP: %.2f, Mean ACC: %.2f, Aligned: %s\n",
			// 		depth, mean_dp, mean_acc, aligned ? "Yes" : "No");
			// }
			k_sleep(K_MSEC(50));
		}			
	}
}

// Initializes the FFT (Fast Fourier Transform) instance
// This function sets up the ARM CMSIS DSP RFFT module with the given FFT size
static void fft_init(void){
	
	arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
}

// Performs Fast Fourier Transform (FFT) on input data
// Parameters:
// input  - pointer to time-domain input data (float32_t array of size FFT_SIZE)
// output - pointer to frequency-domain output data (float32_t array of size FFT_SIZE)
// This function uses the ARM CMSIS-DSP library to perform an efficient FFT
// The transform is configured for real input data, and the result is interleaved (real, imag)
static void fft(float32_t *input, float32_t *output){

	arm_rfft_fast_f32(&fft_inst, input, output, 0);
	
}

/*VISUALIZATION FUNCTIONS*/

// Waits for messages in the `result_msgq` from the processing thread
// and updates the Bluetooth notifications or prints to the console.
static void visualization_thread(void){

	struct result_data result;
	while (true){
		k_msgq_get(&result_msgq, &result, K_FOREVER);

		int bpm = result.frequency_bin - STARTING_OFFSET_BIN; // Remove the offset to get values in range 0-5


		if(device_connected && subscribed){ //check for device ready to receive data

			set_freq_value(bpm);
			set_direction_value(result.direction);
			set_depth_value(result.depth);
			send_freq_notification(current_conn, bpm); 
			send_direction_notification(current_conn, result.direction);
			send_depth_notification(current_conn, result.depth);
		}
		else{ // If device is not ready, print to console
			LOG_INF("BPM: %d, Direction: %s, Depth: %s",bpm_values[bpm_index],
				direction_strings[direction],depth_strings[depth]);
		}

	k_sleep(K_MSEC(50));
	}
}

/* NVS FUNCTIONS */

// Initializes the Non-Volatile Storage (NVS) file system
static void init_nvs(void){
	LOG_INF("NVS offset: 0x%x size: %d", NVS_PARTITION_OFFSET, NVS_PARTITION_SIZE);
	int err = nvs_mount(&fs);
	if (err) {
		LOG_ERR("Failed to initialize NVS: %d", err);
		return;
	}
	else {
		LOG_INF("NVS initialized successfully.");
	}

}

// Stores a calibration offset value in NVS for a given ID
// Parameters:
// id - the identifier key for the calibration data (int)
// offset - the 16-bit calibration offset to store (int16_t)
// Returns: 0 on success, 1 on failure
static int store_offset_calibration(int id, int16_t offset) {
	LOG_DBG("Storing calibration data for ID %d: %d\n", id, offset);
	int err = nvs_write(&fs, id, &offset, sizeof(offset));
	if (err <0) {
		LOG_ERR("Failed to write calibration data: %d", err);
		return 1;
	} else {
		return 0;
	}
}

// Reads a calibration offset value from NVS for a given ID
// Parameters:
// id - the identifier key for the calibration data (int)
// Returns: The stored 16-bit calibration offset on success or INT16_MIN if no calibration data is found or on error
static int16_t read_offset_calibration(int id){
	int16_t stored = 0;
	int err = nvs_read(&fs, id, &stored, sizeof(stored));
	if (err < 0) {
		LOG_INF("No calibration data: %d", err);
		return INT16_MIN;
	} else {
		LOG_INF("Calibration data read successfully: %d", stored);
		return stored;
	}
}

// Stores all calibration offsets into NVS
// Parameters:
// offsets - pointer to a structure containing all 9 calibration offset values
static void store_all_offsets(calibration_offsets *offsets) {
	int16_t *values = (int16_t *)offsets;
	int stored = 0;
	for (int i = 0; i < 9 ; i++) {
		stored += store_offset_calibration (i+1, values[i]);
	}
	if (stored < 0){
		LOG_ERR("Error storing calibration offsets");
	}
}

// Reads all calibration offsets from NVS and stores them in the given structure
// Parameters:
// offsets - pointer to a structure where the 9 calibration offset values will be stored
static void read_all_offsets(calibration_offsets *offsets) {
	int16_t *values = (int16_t *)offsets;
	for (int i = 0; i < 9 ; i++) {
		values[i] = read_offset_calibration(i+1);
	}
}

/*CALIBRATION FUNCTIONS*/

// Sets calibration offsets to the BNO055 sensor using stored values
// Parameters:
// offsets - pointer to structure containing 9 calibration offsets
// Returns: 0 if successful, negative error code otherwise 
static int set_offsets_bno055(calibration_offsets *offsets) {
	int16_t *values = (int16_t *)offsets;
	for (int i = 0; i < 9; i++) {
		if (values[i] == INT16_MIN) {
			return -EINVAL;		//if not stored
		}
	}
	set_configMode();
	k_sleep(K_MSEC(100));	// Wait for config mode to be set
	uint8_t buffer[19];
	buffer[0] = BNO055_OFFSET_ACCEL_X_LSB;
	for (int i = 0; i < 9; i++) {
		buffer[i * 2 + 1] = (values[i] & 0xFF); // LSB
		buffer[i * 2 + 2] = (values[i] >> 8) & 0xFF; // MSB
	}
	int err = i2c_write(dev_i2c, buffer, 19, BNO055_I2C_ADDR);
	if (err) {
		LOG_ERR("Failed to set offsets: %d", err);
	}
	return err;
	
}

// Reads calibration offsets from the BNO055 sensor and stores them in a struct
// Parameters:
// offsets - pointer to structure where the 9 read calibration offsets will be stored
// Returns: 0 if successful, negative error code otherwise
static int read_offsets_bno055(calibration_offsets *offsets) {
	buffer_i2c_write[0] = BNO055_OFFSET_ACCEL_X_LSB;
	uint8_t buffer[17];
	int err = i2c_write_read(dev_i2c, BNO055_I2C_ADDR, buffer_i2c_write, 1, buffer, sizeof(buffer));
	if (err < 0) {
		LOG_ERR("Failed to read offsets: %d", err);
		return err;
	}
	int16_t *values = (int16_t *)offsets;
	for (int i = 0; i < 9; i++) {
        values[i] = (int16_t)((buffer[i * 2 + 1] << 8) | buffer[i * 2]);	//little-endian: LSB first, then MSB
    }
	return 0;
}

// Initializes BNO055 calibration by either restoring saved offsets from NVS 
// or performing a fresh calibration and saving the offsets
// Parameters:
// search_for_offsets - if true, attempts to load calibration data from NVS
static void bno_calibration (bool search_for_offsets){
	read_deviceID();
	k_msleep(100);
	set_configMode();
	k_msleep(100);
	set_pageId0();
	k_msleep(100);
	set_externalCrystal();
	k_msleep(100);
	int calibration_data = INT16_MIN;		// Flag indicating if calibration data was found
	if(search_for_offsets){
		LOG_INF("Searching for calibration offsets in NVS...\n");
		calibration_data = read_offset_calibration(1);	// Try reading first offset from NVS, if not stored returns INT16_MIN
		k_msleep(100);
		if(calibration_data == INT16_MIN){
			LOG_INF("No calibration offsets found in NVS.\n");
		}
	}
	if (calibration_data == INT16_MIN) { // If no data found in NVS, perform calibration and store offsets
		LOG_INF("Starting carlibration...\n");
		set_opMode();
		k_msleep(100);
		read_calibration();
		k_msleep(100);
		set_configMode();
		k_msleep(100);
		set_pageId0();
		k_msleep(100);
		read_offsets_bno055(&bno055_offsets);
		k_msleep(100);
		store_all_offsets(&bno055_offsets);
		set_opMode();
		k_msleep(100);
	}
	else { // If calibration data found in NVS, read offsets and set them to the BNO055
		LOG_INF("Calibration data found, setting offsets...\n");
		read_all_offsets(&bno055_offsets);
		k_msleep(100);
		int err = set_offsets_bno055(&bno055_offsets);
		if (err == -EINVAL){
			LOG_ERR("Offsets not set, calibration data not found in NVS.");
		}
		else if (err < 0) {
			LOG_ERR("Calibration data found but failed to set offsets: %d", err);
		} else {
			LOG_INF("Offsets set successfully.");
		}
		bno055_calibration_status.calibrated = true;
		k_msleep(100);
		set_opMode();
		k_msleep(100);
	}
}

// Main entry point of the program
// Initializes system components, starts BLE, calibrates sensor, initializes FFT,
// sets up mutexes and timers, then enters idle loop.
int main(void){
        k_sleep(K_MSEC(500));
        LOG_INF("INITIALIZING SYSTEM...\n");
		init_nvs();
		k_sleep(K_MSEC(500));
        set_interrupts();
        k_sleep(K_MSEC(500));
		ble_init();
		k_sleep(K_MSEC(500));
        bno_calibration(true);
		k_sleep(K_MSEC(500));
        fft_init();
		k_sleep(K_MSEC(500));
        k_mutex_init(&buffer1_mutex);
        k_mutex_init(&buffer2_mutex);
		k_sleep(K_MSEC(500));
        k_timer_start(&sensor_timer, K_NO_WAIT, K_USEC(SAMPLING_INTERVAL));

        while (true){
                k_sleep(K_FOREVER);
        }
        return 0;
}