#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>


/* DEFINITION OF THE CUSTOM BLUETOOTH UUIDS USED FOR THE REMOTE GATT SERVICE AND ITS CHARACTERISTICS */
#define BT_UUID_REMOTE_SERV_VAL \
    BT_UUID_128_ENCODE(0x02440001, 0xae8a, 0x4b53, 0x8714, 0xccbfea0ba390)

#define BT_UUID_REMOTE_FREQ_VAL \
    BT_UUID_128_ENCODE(0x02440002, 0xae8a, 0x4b53, 0x8714, 0xccbfea0ba390)

#define BT_UUID_REMOTE_DIRECTION_VAL \
BT_UUID_128_ENCODE(0x02440003, 0xae8a, 0x4b53, 0x8714, 0xccbfea0ba390)

#define BT_UUID_REMOTE_DEPTH_VAL \
BT_UUID_128_ENCODE(0x02440004, 0xae8a, 0x4b53, 0x8714, 0xccbfea0ba390)


/* DECLARATION OF UUID TYPES FOR ZEPHYR'S BLUETOOTH STACK */
#define BT_UUID_REMOTE_SERV         BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)
#define BT_UUID_REMOTE_FREQ         BT_UUID_DECLARE_128(BT_UUID_REMOTE_FREQ_VAL)
#define BT_UUID_REMOTE_DIRECTION    BT_UUID_DECLARE_128(BT_UUID_REMOTE_DIRECTION_VAL)
#define BT_UUID_REMOTE_DEPTH        BT_UUID_DECLARE_128(BT_UUID_REMOTE_DEPTH_VAL)

/* NOTIFICATION STATUS ENUMERATIONS FOR EACH CHARACTERISTIC */
enum bt_freq_notifications_enabled {
    BT_FREQ_NOTIFICATIONS_DISABLED,
    BT_FREQ_NOTIFICATIONS_ENABLED
};

enum bt_direction_notifications_enabled {
    BT_DIRECTION_NOTIFICATIONS_DISABLED,
    BT_DIRECTION_NOTIFICATIONS_ENABLED
};

enum bt_depth_notifications_enabled {
    BT_DEPTH_NOTIFICATIONS_DISABLED,
    BT_DEPTH_NOTIFICATIONS_ENABLED
};

/* CALLBACK STRUCTURE FOR NOTIFICATION STATUS CHANGE */
struct bt_remote_srv_cb{
    void (*notif_changed)(enum bt_freq_notifications_enabled status);
};


/* FUNCTION TO SEND A NOTIFICATION FOR THE FREQUENCY CHARACTERISTIC */
int send_freq_notification(struct bt_conn *conn, int value);

/* FUNCTION TO SEND A NOTIFICATION FOR THE DIRECTION CHARACTERISTIC */
int send_direction_notification(struct bt_conn *conn, int value);

/* FUNCTION TO SEND A NOTIFICATION FOR THE DEPTH CHARACTERISTIC */
int send_depth_notification(struct bt_conn *conn, int value);

/* INITIALIZES THE BLUETOOTH SYSTEM AND REGISTERS THE REMOTE SERVICE CALLBACKS */
int bluetooth_init(struct bt_conn_cb *bt_cb, struct bt_remote_srv_cb *remote_srv_cb);

/* SETTERS TO UPDATE THE LOCAL VALUES OF EACH CHARACTERISTIC */
void set_freq_value(int value);

void set_direction_value(int value);

void set_depth_value(int value);

/* STARTS ADVERTISING THE GATT SERVICE */
int bluetooth_start_advertising(void);

/* STOPS ADVERTISING THE GATT SERVICE */
int bluetooth_stop_advertising(void);

