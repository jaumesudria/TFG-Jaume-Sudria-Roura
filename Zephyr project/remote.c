#include "remote.h"

static K_SEM_DEFINE(bt_init_ok, 1, 1);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME  //Device name from Kconfig file
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)

/*INTERNAL STAT VARIABLES*/
static int freq_value = -1;
static int direction_status = -1;
static int depth_value = -1;

/*CALLBACK STRUCTURE FOR REMOTE SERVICE*/
static struct bt_remote_srv_cb remote_service_callbacks;

/*ENUMS TO TRACK WETHER NOTIFICATIONS ARE ENABLED*/
enum bt_freq_notifications_enabled freq_notifications_enabled;
enum bt_direction_notifications_enabled direction_notifications_enabled;
enum bt_depth_notifications_enabled depth_notifications_enabled;

/*ADVERTISING DATA*/
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_REMOTE_SERV_VAL),
};

/*FUNCTION DECLARATIONS*/
static ssize_t read_freq_characteristic_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset);

static ssize_t read_direction_characteristic_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset);

static ssize_t read_depth_characteristic_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset);

void freq_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
void direction_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
void depth_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

/*DEFINITION OF CUSTOM GATT SERVICE AND CHARACTERISTICS*/
BT_GATT_SERVICE_DEFINE(remote_srv,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_REMOTE_SERV),
        BT_GATT_CHARACTERISTIC(BT_UUID_REMOTE_FREQ,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ, 
            read_freq_characteristic_cb, NULL, "Frequency"),
        BT_GATT_CCC(freq_chrc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
        BT_GATT_CHARACTERISTIC(BT_UUID_REMOTE_DIRECTION,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ, 
            read_direction_characteristic_cb, NULL, "Direction"),
        BT_GATT_CCC(direction_chrc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
        BT_GATT_CHARACTERISTIC(BT_UUID_REMOTE_DEPTH,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ, 
            read_depth_characteristic_cb, NULL, "Depth"),
        BT_GATT_CCC(depth_chrc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/*CCCD CALLBACKS: HANDLE DEPTH NOTIFICATIONS TOGGLE*/
void depth_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value){
    bool notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    if (notify_enabled) {
        printk("Notifications enabled for depth characteristic\n");
    } else {
        printk("Notifications disabled for depth characteristic\n");
    }
    depth_notifications_enabled = notify_enabled? BT_DEPTH_NOTIFICATIONS_ENABLED : BT_DEPTH_NOTIFICATIONS_DISABLED;

    if(remote_service_callbacks.notif_changed) {
        remote_service_callbacks.notif_changed(depth_notifications_enabled);
    }
}

/*CCCD CALLBACKS: HANDLE DIRECTION NOTIFICATIONS TOGGLE*/
void direction_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr,  uint16_t value){
    bool notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    if (notify_enabled) {
        printk("Notifications enabled for direction characteristic\n");
    } else {
        printk("Notifications disabled for direction characteristic\n");
    }
    direction_notifications_enabled = notify_enabled? BT_DIRECTION_NOTIFICATIONS_ENABLED : BT_DIRECTION_NOTIFICATIONS_DISABLED;

    if(remote_service_callbacks.notif_changed) {
        remote_service_callbacks.notif_changed(direction_notifications_enabled);
    }
}

/*CCCD CALLBACKS: HANDLE FREQUENCY NOTIFICATIONS TOGGLE*/
void freq_chrc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value){
    bool notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    if (notify_enabled) {
        printk("Notifications enabled for frequency characteristic\n");
    } else {
        printk("Notifications disabled for frequency characteristic\n");
    }
    freq_notifications_enabled = notify_enabled? BT_FREQ_NOTIFICATIONS_ENABLED : BT_FREQ_NOTIFICATIONS_DISABLED;

    if(remote_service_callbacks.notif_changed) {
        remote_service_callbacks.notif_changed(freq_notifications_enabled);
    }
}

/*CHARACTERISTIC CALLBACK: DIRECTION*/
static ssize_t read_direction_characteristic_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset){
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &direction_status, sizeof(direction_status));
    }

/*CHARACTERISTIC CALLBACK: FREQUENCY*/
static ssize_t read_freq_characteristic_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset){
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &freq_value, sizeof(freq_value));
    }

/*CHARACTERISTIC CALLBACK: DEPTH*/
static ssize_t read_depth_characteristic_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset){
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &depth_value, sizeof(depth_value));
    }

/*NOTIFICATION SEND COMPLETION CALLBACK*/
void on_sent(struct bt_conn *conn, void *user_data) {
    ARG_UNUSED(user_data);
    printk("Notification sent successfully\n");
}

/*BLUETOOTH READY CALLBACK*/
void bt_ready(int err){
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
        }else{
            printk("Bluetooth ready\n");
        k_sem_give(&bt_init_ok);
        }
}

/*SEND DEPTH VALUE AS A BLE NOTIFICATION*/
int send_depth_notification(struct bt_conn *conn, int value) {
    int err = 0;
    struct bt_gatt_notify_params params = {0};
    const struct bt_gatt_attr *attr = &remote_srv.attrs[8];

    params.attr = attr;
    params.data = &value;
    params.len = sizeof(value);
    params.func = on_sent;

    err = bt_gatt_notify_cb(conn, &params);
    return err;
}

/*SEND DIRECTION VALUE AS A BLE NOTIFICATION*/
int send_direction_notification(struct bt_conn *conn, int value) {
    int err = 0;
    struct bt_gatt_notify_params params = {0};
    const struct bt_gatt_attr *attr = &remote_srv.attrs[5];

    params.attr = attr;
    params.data = &value;
    params.len = sizeof(value);
    params.func = on_sent;

    err = bt_gatt_notify_cb(conn, &params);
    return err;
}

/*SEND FREQUENCY VALUE AS A BLE NOTIFICATION*/
int send_freq_notification(struct bt_conn *conn, int value){
    int err = 0;
    struct bt_gatt_notify_params params = {0};
    const struct bt_gatt_attr *attr = &remote_srv.attrs[2];

    params.attr = attr;
    params.data = &value;
    params.len = sizeof(value);
    params.func = on_sent;

    err = bt_gatt_notify_cb(conn, &params);
    return err;

}

/*SETTERS TO UPDATE INTERNAL VALUES*/
void set_freq_value(int value){
    freq_value = value;
}

void set_direction_value(int value){
    direction_status = value;
}

void set_depth_value(int value){
    depth_value = value;
}

/*INITIALIZE BLUETOOTH AND REGISTER CALLBACKS*/
int bluetooth_init(struct bt_conn_cb *bt_cb, struct bt_remote_srv_cb *remote_srv_cb) {
    int err;
    printk("Initializing Bluetooth...\n");

    if (bt_cb ==NULL || remote_srv_cb == NULL) {
        return -NRFX_ERROR_NULL;
    }

    bt_conn_cb_register(bt_cb);
    remote_service_callbacks.notif_changed = remote_srv_cb->notif_changed;


    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth enable failed (err %d)\n", err);
        return err;
    }

    k_sem_take(&bt_init_ok, K_FOREVER);
    printk("Bluetooth initialized successfully\n");

    return err;
}

/*START ADVERTISING WITH THE DEFINED SERVICE AND CHARACTERISTICS*/
int bluetooth_start_advertising(void) {

    int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
    }else {
        printk("Advertising successfully started\n");
    }
    return err;
}

/* STOPS ADVERTISING THE GATT SERVICE */
int bluetooth_stop_advertising(void) {
    int err = bt_le_adv_stop();
    if (err) {
        printk("Advertising failed to stop (err %d)\n", err);
    } else {
        printk("Advertising successfully stopped\n");
    }
    return err;
}
