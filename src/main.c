#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ppu_thread.h>
#include <sys/process.h>
#include <sys/prx.h>
#include <sys/synchronization.h>
#include <sys/timer.h>
#include <sys/types.h>
#include <sys/time.h>
#include <cell/sysmodule.h>
#include <cell/pad.h>
#include <cell/pad/libpad_dbg.h>
#include <cell/usbd.h>
#include "ControlStruct.h"

#define THREAD_NAME "xpaddt"
#define STOP_THREAD_NAME "xpadds"
#define MAX_XPAD_DEV_NUM ((int32_t)(sizeof(xpad_info) / sizeof(xpad_info[0])))
#define MAX_XPADW_DEV_NUM ((int32_t)(sizeof(xpadw_info) / sizeof(xpadw_info[0])))
#define MAX_XPAD_NUM CELL_PAD_MAX_PORT_NUM
#define MAX_XPADW_NUM 4
#define XPAD_DATA_LEN 14+2 // +2 for count and size fields
#define XPADW_DATA_LEN 0x13+2
#define MAX_XPAD_DATA_LEN XPADW_DATA_LEN
#define RINGBUF_SIZE  10
#define DESCRIPTOR_TABLE_SIZE (sizeof(descriptor_table)/sizeof(descriptor_table_t))
#define SWAP16(x) ((uint16_t)((((x) & 0x00FF) << 8) | (((x) & 0xFF00) >> 8)))

enum XTYPES {
  XTYPE_XBOX360 = 1,
  XTYPE_XBOX360W = 2
};

typedef struct xpad_device {
	uint16_t vid;
	uint16_t pid;
	const char *name;
} XPAD_INFO_t;

// xpad device info from linux xpad driver
static XPAD_INFO_t xpad_info[] = {
	{0x045e, 0x028e, "Microsoft X-Box 360 pad"},
	{0x046d, 0xc242, "Logitech Chillstream Controller"},
	{0x0738, 0x4716, "Mad Catz Wired Xbox 360 Controller" },
	{0x0738, 0x4728, "Mad Catz Street Fighter IV FightPad"},
	{0x0738, 0x4738, "Mad Catz Wired Xbox 360 Controller (SFIV)"},
	{0x0738, 0xbeef, "Mad Catz JOYTECH NEO SE Advanced GamePad"},
	{0x0e6f, 0x0201, "Pelican PL-3601 'TSZ' Wired Xbox 360 Controller"},
	{0x0e6f, 0x0213, "Afterglow Gamepad for Xbox 360" },
	{0x0f0d, 0x000d, "Hori Fighting Stick EX2"},
	{0x0f0d, 0x0016, "Hori Real Arcade Pro.EX"},
	{0x146b, 0x0601, "BigBen Interactive XBOX 360 Controller"},
	{0x1689, 0xfd00, "Razer Onza Tournament Edition"},
	{0x1689, 0xfd01, "Razer Onza Classic Edition"},
	{0x1bad, 0x0003, "Harmonix Rock Band Drumkit"},
	{0x1bad, 0xf016, "Mad Catz Xbox 360 Controller"},
	{0x1bad, 0xf028, "Street Fighter IV FightPad"},
	{0x1bad, 0xf901, "Gamestop Xbox 360 Controller"},
	{0x1bad, 0xf903, "Tron Xbox 360 controller"},
	{0x24c6, 0x5300, "PowerA MINI PROEX Controller"},
};

static XPAD_INFO_t xpadw_info[] = {
 {0x045e, 0x0291, "Xbox 360 Wireless Receiver (XBOX)"},
 {0x045e, 0x0719, "Xbox 360 Wireless Receiver"},
};

typedef struct {
  uint8_t bDescriptorType;
  int32_t (*dump_descriptor)(int32_t dev_id, void *desc);
} descriptor_table_t;

typedef struct {
  int32_t dev_id; /* Device id */
  int32_t number; /* Xpad number */
  int32_t c_pipe; /* Control pipe id */
  int32_t i_pipe; /* In pipe id */
  int32_t o_pipe; /* Out pipe id */
  int32_t payload; /* Size of payload */
  uint8_t ifnum; /* Interface number */
  uint8_t as; /* Alternate setting number */
  int32_t tcount; /* Transfer counts */
  uint8_t xtype;

  // methods to their respective controllers
  int32_t (*read_input)(int32_t dev_id, void *data);
  int32_t (*set_led)(int32_t dev_id, uint8_t led);
  int32_t (*set_rumble)(int32_t dev_id, uint8_t lval, uint8_t rval);

  /* Ring buffer */
  int32_t rp; /* Read pointer   */
  int32_t wp; /* Write pointer  */
  int32_t rblen; /* Buffer length  */
  unsigned char ringbuf[RINGBUF_SIZE][MAX_XPAD_DATA_LEN]; /* Ring buffer */

  /* Buffer for interrupt transfer */
  unsigned char data[0];

} XPAD_UNIT_t;

typedef struct {
  int32_t next_number;
  int32_t n;
  int32_t is_connected[MAX_XPAD_NUM];
  XPAD_UNIT_t *con_unit[MAX_XPAD_NUM];
} XPAD_t;

int xpadd_start(uint64_t arg);
int xpadd_stop(void);

// wired Xbox 360 controller methods
static int32_t xpad_probe(int32_t dev_id);
static int32_t xpad_attach(int32_t dev_id);
static int32_t xpad_detach(int32_t dev_id);
static int32_t xpad_detach_all(void);
static int32_t xpad_read_input(int32_t id, void *data);
static void xpad_read_report(int32_t id, uint8_t *readBuf);
static int32_t xpad_set_led(int32_t id, uint8_t led);
static int32_t xpad_set_rumble(int32_t id, uint8_t lval, uint8_t rval);

// wireless Xbox 360 controller methods
static int32_t xpadw_probe(int32_t dev_id);
static int32_t xpadw_attach(int32_t dev_id);
static int32_t xpadw_detach(int32_t dev_id);
static int32_t xpadw_detach_all(void);
static int32_t xpadw_read_input(int32_t id, void *data);
static void xpadw_read_report(int32_t id, uint8_t *readBuf);
static int32_t xpadw_set_led(int32_t id, uint8_t led);
static int32_t xpadw_set_rumble(int32_t id, uint8_t lval, uint8_t rval);

// common methods
static void data_transfer_done(int32_t result, int32_t count, void *arg);
static void data_transfer(XPAD_UNIT_t *unit);
static void set_config_done(int32_t result, int32_t count, void *arg);
static void set_interface_done(int32_t result, int32_t count, void *arg);
static XPAD_UNIT_t *unit_alloc(int32_t dev_id, int32_t payload, uint8_t ifnum, uint8_t as, uint8_t xtype);
static void unit_free(XPAD_UNIT_t *unit);
static int32_t check_pad_status(void);
static int32_t register_ldd_controller(XPAD_UNIT_t *unit);
static int32_t unregister_ldd_controller(XPAD_UNIT_t *unit);

// vsh methods
void *getNIDfunc(const char *vsh_module, uint32_t fnid, int32_t offset);
static void show_msg(char *msg);
int (*vshtask_notify)(int, const char *) = NULL;
void *(*vsh_malloc)(unsigned int size) = NULL;
int (*vsh_free)(void *ptr) = NULL;

// usb methods
static int32_t get_device_desc(int32_t dev_id, void *p);
static int32_t get_configration_desc(int32_t dev_id, void *p);
static int32_t get_interface_desc(int32_t dev_id, void *p);
static int32_t get_endpoint_desc(int32_t dev_id, void *p);

descriptor_table_t descriptor_table[] = {
  {USB_DESCRIPTOR_TYPE_DEVICE, get_device_desc},
  {USB_DESCRIPTOR_TYPE_CONFIGURATION, get_configration_desc},
  {USB_DESCRIPTOR_TYPE_INTERFACE, get_interface_desc},
  {USB_DESCRIPTOR_TYPE_ENDPOINT, get_endpoint_desc},
};

static CellUsbdLddOps xpad_ops = {
  0,
  xpad_probe,
  xpad_attach,
  xpad_detach
};

static CellUsbdLddOps xpadw_ops = {
  0,
  xpadw_probe,
  xpadw_attach,
  xpadw_detach
};

static XPAD_t XPAD;
static uint8_t xpad_led[4] = {ledOn1, ledOn2, ledOn3, ledOn4};
static sys_ppu_thread_t thread_id = 1;
static sys_mutex_t xpad_mutex, ringbuf_mutex;
static int32_t handle[CELL_PAD_MAX_PORT_NUM];
static volatile uint8_t running;

SYS_MODULE_INFO(XPADD, 0, 1, 0);
SYS_MODULE_START(xpadd_start);
SYS_MODULE_STOP(xpadd_stop);

static inline void _sys_ppu_thread_exit(uint64_t val) {
  system_call_1(41, val);
}

static inline sys_prx_id_t prx_get_module_id_by_address(void *addr) {
  system_call_1(461, (uint64_t)(uint32_t)addr);
  return((int)p1);
}

static inline void sys_pad_dbg_ldd_register_controller(uint8_t *data, int32_t *handle, uint8_t addr, uint32_t capability) {

  // syscall for registering a virtual controller with custom capabilities
  system_call_4(574, (uint8_t *)data, (int32_t *)handle, (uint8_t)addr, (uint32_t)capability);
}

static inline void sys_pad_dbg_ldd_set_data_insert_mode(int32_t handle, uint16_t addr, uint32_t *mode, uint8_t addr2) {

  // syscall for controlling button data filter (allows a virtual controller to be used in games)
  system_call_4(573, handle, addr, mode, addr2);
}

void *getNIDfunc(const char * vsh_module, uint32_t fnid, int32_t offset) {

  // from webman-MOD source
  // used to find malloc, free, and show notification

  // 0x10000 = ELF
  // 0x10080 = segment 2 start
  // 0x10200 = code start

  uint32_t table = (*(uint32_t *)0x1008C) + 0x984; // vsh table address
  //  uint32_t table = (*(uint32_t*)0x1002C) + 0x214 - 0x10000; // vsh table address
  //  uint32_t table = 0x63A9D4;

  while (((uint32_t)*(uint32_t *)table) != 0) {
    uint32_t *export_stru_ptr = (uint32_t *)*(uint32_t *)table; // ptr to export stub, size 2C, "sys_io" usually... Exports:0000000000635BC0 stru_635BC0:    ExportStub_s <0x1C00, 1, 9, 0x39, 0, 0x2000000, aSys_io, ExportFNIDTa
    const char *lib_name_ptr =  (const char *)*(uint32_t *)((char *)export_stru_ptr + 0x10);
    if(strncmp(vsh_module, lib_name_ptr, strlen(lib_name_ptr)) == 0) {
      // we got the proper export struct
      uint32_t lib_fnid_ptr = *(uint32_t *)((char *)export_stru_ptr + 0x14);
      uint32_t lib_func_ptr = *(uint32_t *)((char *)export_stru_ptr + 0x18);
      uint16_t count = *(uint16_t *)((char *)export_stru_ptr + 6); // number of exports
      for (int i = 0; i < count; i++) {
        if (fnid == *(uint32_t *)((char *)lib_fnid_ptr + i*4)) {
          // take address from OPD
          return (void **)*((uint32_t *)(lib_func_ptr) + i) + offset;
        }
      }
    }
    table = table + 4;
  }
  return(0);
}

static void show_msg(char* msg) {

  // from webman-MOD
  // displays a notification on the PS3
  if (!vshtask_notify) {
    vshtask_notify = (void *)((int)getNIDfunc("vshtask", 0xA02D46E7, 0));
  }
  if (strlen(msg) > 200) {
    msg[200] = 0;
  }
  if (vshtask_notify) {
    vshtask_notify(0, msg);
  }
}

static void *_malloc(unsigned int size) {

  // vsh export for malloc
  if (!vsh_malloc) {
    vsh_malloc = getNIDfunc("allocator", 0x759E0635, 0);
  }
  if (vsh_malloc) {
    return vsh_malloc(size);
  }
  return(NULL);
}

static void _free(void *ptr) {

  // vsh export for free
  if (!vsh_free) {
    vsh_free = (void *)((int)getNIDfunc("allocator", 0x77A602DD, 0));
  }
  if (vsh_free) {
    vsh_free(ptr);
  }
}

static void block(sys_mutex_t mutex) {
  int32_t r;

  if ((r = sys_mutex_lock(mutex, 0)) != CELL_OK) {
    sys_ppu_thread_exit(0);
  }
}

static void unblock(sys_mutex_t mutex) {
  int32_t r;

  if ((r = sys_mutex_unlock(mutex)) != CELL_OK) {
    sys_ppu_thread_exit(0);
  }
}

static void data_transfer_done(int32_t result, int32_t count, void *arg) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)arg;
  unsigned char *xpadbuf;
  (void)result;
  block(ringbuf_mutex);
  if (unit->rblen < RINGBUF_SIZE) {
    xpadbuf = &unit->ringbuf[unit->wp][0];
    xpadbuf[0] = (unsigned char)(++unit->tcount & 0xFF);
    xpadbuf[1] = (unsigned char)(count & 0xFF);
    count = (count <= MAX_XPAD_DATA_LEN - 2) ? count : MAX_XPAD_DATA_LEN - 2;
    memcpy(&xpadbuf[2], unit->data, count);
    if (++unit->wp >= RINGBUF_SIZE) {
      unit->wp = 0;
    }
    unit->rblen++;
  }
  unblock(ringbuf_mutex);
  data_transfer(unit);
}

static void data_transfer(XPAD_UNIT_t *unit) {
  cellUsbdInterruptTransfer(unit->i_pipe, unit->data, unit->payload, data_transfer_done, unit);
}

static void set_interface_done(int32_t result, int32_t count, void *arg) {
  (void)result;
  (void)count;
  data_transfer((XPAD_UNIT_t *)arg);
}

static void set_config_done(int32_t result, int32_t count, void *arg) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)arg;
  (void)result;
  (void)count;
  if (unit->as > 0) {
    cellUsbdSetInterface(unit->c_pipe, unit->ifnum, unit->as, set_interface_done, unit);
  } else {
    data_transfer(unit);
  }
}

static void unit_free(XPAD_UNIT_t *unit) {
  if (unit) {
    _free(unit);
  }
}

static XPAD_UNIT_t *unit_alloc(int32_t dev_id, int32_t payload, uint8_t ifnum, uint8_t as, uint8_t xtype) {
  XPAD_UNIT_t *unit;
  int32_t i;
  if ((unit = (XPAD_UNIT_t *)_malloc(sizeof(XPAD_UNIT_t) + payload)) != NULL) {
    memset(unit, 0, sizeof(XPAD_UNIT_t));
    unit->dev_id = dev_id;
    unit->payload = payload;
    unit->ifnum = ifnum;
    unit->as = as;
    unit->tcount = 0;
    unit->rp = 0;
    unit->wp = 0;
    unit->rblen = 0;
    unit->xtype = xtype;
    if (xtype == XTYPE_XBOX360) {
      unit->read_input = xpad_read_input;
      unit->set_led = xpad_set_led;
      unit->set_rumble = xpad_set_rumble;
    } else if (xtype == XTYPE_XBOX360W) {
      unit->read_input = xpadw_read_input;
      unit->set_led = xpadw_set_led;
      unit->set_rumble = xpadw_set_rumble;
    }
    block(xpad_mutex);
    unit->number = XPAD.next_number;
    for (i = 0; i < MAX_XPAD_NUM; i++) {
      ++XPAD.next_number;
      if (XPAD.next_number >= MAX_XPAD_NUM) {
        XPAD.next_number = 0;
      }
      if (XPAD.con_unit[XPAD.next_number] == NULL) {
        break;
      }
    }
    unblock(xpad_mutex);
    if (i >= MAX_XPAD_NUM) {
      _free(unit);
      return(NULL);
    }
  }
  return(unit);
}

static int32_t register_ldd_controller(XPAD_UNIT_t *unit) {
  uint8_t data[0x114];
  int32_t port;
  uint32_t capability, mode, port_setting;

  // register ldd controller with custom device capability
  if (handle[unit->number] < 0) {
    capability = 0xFFFF; // CELL_PAD_CAPABILITY_PS3_CONFORMITY | CELL_PAD_CAPABILITY_PRESS_MODE | CELL_PAD_CAPABILITY_HP_ANALOG_STICK | CELL_PAD_CAPABILITY_ACTUATOR;
    sys_pad_dbg_ldd_register_controller(data, (int32_t *)&(handle[unit->number]), 5, (uint32_t)capability << 1);
    //handle[unit->number] = cellPadLddRegisterController();
    sys_timer_usleep(1000*10); // allow some time for ps3 to register ldd controller
    if (handle[unit->number] < 0) {
      return(handle[unit->number]);
    }

    // all pad data into games
    mode = CELL_PAD_LDD_INSERT_DATA_INTO_GAME_MODE_ON; // = (1)
    sys_pad_dbg_ldd_set_data_insert_mode((int32_t)handle[unit->number], 0x100, (uint32_t *)&mode, 4);

    // set press and sensor mode on
    port_setting = CELL_PAD_SETTING_PRESS_ON | CELL_PAD_SETTING_SENSOR_ON;
    port = cellPadLddGetPortNo(handle[unit->number]);
    if (port < 0) {
      return(port);
    }
    cellPadSetPortSetting(port, port_setting);

    // set Xbox led corresponding to port number
    unit->set_led(unit->number, xpad_led[port%4]);
  }
  return(CELL_PAD_OK);
}

static int32_t unregister_ldd_controller(XPAD_UNIT_t *unit) {
  int32_t r;

  if (handle[unit->number] >= 0) {
    if ((r = cellPadLddUnregisterController(handle[unit->number])) != CELL_OK) {
      return(r);
    }
    //xpad_set_led(unit->number, xpad_led[ledBlinkingAll]);
    handle[unit->number] = -1;
  }
  return(CELL_PAD_OK);
}

void usb_done_cb(int32_t result, int32_t count, void *arg) {
  XPAD_UNIT_t *unit = (XPAD_UNIT_t *)arg;

  // do nothing
}

static int32_t write_xpad(int32_t id, uint8_t *data, int32_t len) {
  int32_t r;
  XPAD_UNIT_t *unit;

  unit = XPAD.con_unit[id];
  r = cellUsbdInterruptTransfer(unit->o_pipe, data, len, usb_done_cb, unit);
  return(r);
}

// start of wired controller specific methods
static int32_t xpad_probe(int32_t dev_id) {
  uint16_t idVendor, idProduct;
  uint32_t i;
  UsbDeviceDescriptor *ddesc;
  UsbInterfaceDescriptor *idesc;

  block(xpad_mutex);
  if (XPAD.n >= MAX_XPAD_NUM) {
    unblock(xpad_mutex);
    return(CELL_USBD_PROBE_FAILED);
  }
  unblock(xpad_mutex);
  if ((ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE)) == NULL) {
    return(CELL_USBD_PROBE_FAILED);
  }
  idesc = (UsbInterfaceDescriptor *)ddesc;
  if ((idesc = (UsbInterfaceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, idesc, USB_DESCRIPTOR_TYPE_INTERFACE)) == NULL) {
    return(CELL_USBD_PROBE_FAILED);
  }

  // make sure product id and vendor id are valid
  idVendor = SWAP16(ddesc->idVendor);
  idProduct = SWAP16(ddesc->idProduct);
  for (i = 0; i < MAX_XPAD_DEV_NUM; i++) {
    if (xpad_info[i].vid == idVendor && xpad_info[i].pid == idProduct) {
      return(CELL_USBD_PROBE_SUCCEEDED);
    }
  }
  return(CELL_USBD_PROBE_FAILED);
}

static int32_t xpad_attach(int32_t dev_id) {
  int32_t payload, port;
  uint32_t mode, port_setting;
  UsbConfigurationDescriptor *cdesc;
  UsbInterfaceDescriptor *idesc;
  UsbEndpointDescriptor *edesc;
  XPAD_UNIT_t *unit;

  if ((cdesc = (UsbConfigurationDescriptor *) cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_CONFIGURATION)) == NULL) {
    return (CELL_USBD_ATTACH_FAILED);
  }
  idesc = (UsbInterfaceDescriptor *)cdesc;
  if ((idesc = (UsbInterfaceDescriptor *) cellUsbdScanStaticDescriptor(dev_id, idesc, USB_DESCRIPTOR_TYPE_INTERFACE)) == NULL) {
    return(CELL_USBD_ATTACH_FAILED);
  }
  if ((edesc = (UsbEndpointDescriptor *) cellUsbdScanStaticDescriptor(dev_id, idesc, USB_DESCRIPTOR_TYPE_ENDPOINT)) == NULL) {
    return(CELL_USBD_ATTACH_FAILED);
  }
  if (edesc->bEndpointAddress != 0x81) { // XBox 360 controller In endpoint
    return(CELL_USBD_ATTACH_FAILED);
  }
  payload = SWAP16(edesc->wMaxPacketSize);
  if ((unit = unit_alloc(dev_id, payload, idesc->bInterfaceNumber, idesc->bAlternateSetting, XTYPE_XBOX360)) == NULL) {
    return(CELL_USBD_ATTACH_FAILED);
  }
  if ((unit->c_pipe = cellUsbdOpenPipe(dev_id, NULL)) < 0) {
    unit_free(unit);
    return(CELL_USBD_ATTACH_FAILED);
  }
  if ((unit->i_pipe = cellUsbdOpenPipe(dev_id, edesc)) < 0) {
    unit_free(unit);
    return(CELL_USBD_ATTACH_FAILED);
  }
  edesc->bEndpointAddress = 0x01; // XBox 360 controller out endpoint
  if ((unit->o_pipe = cellUsbdOpenPipe(dev_id, edesc)) < 0) {
    edesc->bEndpointAddress = 0x02; // It is 0x02 for some controllers
    if ((unit->o_pipe = cellUsbdOpenPipe(dev_id, edesc)) < 0) {
      unit_free(unit);
      return(CELL_USBD_ATTACH_FAILED);
    }
  }

  // endpoint found, set configuration and add to connected controllers list
  cellUsbdSetPrivateData(dev_id, unit);
  cellUsbdSetConfiguration(unit->c_pipe, cdesc->bConfigurationValue, set_config_done, unit);
  block(xpad_mutex);
  XPAD.n++;
  XPAD.is_connected[unit->number] = 1;
  XPAD.con_unit[unit->number] = unit;
  register_ldd_controller(unit);
  unblock(xpad_mutex);
  return(CELL_USBD_ATTACH_SUCCEEDED);
}

static int32_t xpad_detach(int32_t dev_id) {
  int32_t r;
  XPAD_UNIT_t *unit;

  // Xbox controller has been unplugged
  // disconnect virtual controller associated to it
  if ((unit = (XPAD_UNIT_t *)cellUsbdGetPrivateData(dev_id)) == NULL) {
    return(CELL_USBD_DETACH_FAILED);
  }
  block(xpad_mutex);

  // update common data
  XPAD.n--;
  XPAD.is_connected[unit->number] = 0;
  XPAD.con_unit[unit->number] = NULL;
  unregister_ldd_controller(unit);
  unblock(xpad_mutex);
  unit_free(unit);
  return(CELL_USBD_DETACH_SUCCEEDED);
}

static int32_t xpad_detach_all(void) {
  int32_t i;
  XPAD_UNIT_t *unit;

  // detach all wired controllers
  block(xpad_mutex);
  for (i = 0; i < MAX_XPAD_NUM; i++) {
    if (XPAD.is_connected[i]) {
      unit = XPAD.con_unit[i];
      if (unit->xtype == XTYPE_XBOX360) {
        XPAD.n--;
        XPAD.is_connected[unit->number] = 0;
        XPAD.con_unit[unit->number] = NULL;
        unregister_ldd_controller(unit);
        unit_free(unit);
      }
    }
  }
  unblock(xpad_mutex);
  return(CELL_USBD_DETACH_SUCCEEDED);
}

static void xpad_read_report(int32_t id, uint8_t *readBuf) {
  uint16_t *digit0, *digit1, *digit2,
           *analog_rx, *analog_ry, *analog_lx, *analog_ly,
           *press_l2, *press_r2, *press_l1, *press_r1,
           *press_up, *press_down, *press_left, *press_right,
           *press_cross, *press_square, *press_circle, *press_triangle;
  XBOX360_IN_REPORT *report;
  CellPadData data;

  report = (XBOX360_IN_REPORT *)readBuf;
  memset(&data, 0, sizeof(CellPadData));
  data.len = 24;

  // map location of each button in virtual pad's data
  digit0 = &data.button[0];
  digit1 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL1];
  digit2 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL2];
  analog_rx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X];
  analog_ry = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y];
  analog_lx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X];
  analog_ly = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y];
  press_l2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L2];
  press_r2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R2];
  press_l1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L1];
  press_r1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R1];
  press_right = &data.button[CELL_PAD_BTN_OFFSET_PRESS_RIGHT];
  press_left = &data.button[CELL_PAD_BTN_OFFSET_PRESS_LEFT];
  press_up = &data.button[CELL_PAD_BTN_OFFSET_PRESS_UP];
  press_down = &data.button[CELL_PAD_BTN_OFFSET_PRESS_DOWN];
  press_cross = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CROSS];
  press_square = &data.button[CELL_PAD_BTN_OFFSET_PRESS_SQUARE];
  press_circle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CIRCLE];
  press_triangle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_TRIANGLE];

  // set default controller values
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_X] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Y] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Z] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_G] = 0x0200;

  // read values from Xbox controller and map it to the virtual pad
  *digit0 = (report->buttons & btnXbox) ? *digit0 | CELL_PAD_CTRL_LDD_PS : *digit0 & ~CELL_PAD_CTRL_LDD_PS;
  *digit1 = (report->buttons & btnDigiLeft) ? *digit1 | CELL_PAD_CTRL_LEFT : *digit1 & ~CELL_PAD_CTRL_LEFT;
  *digit1 = (report->buttons & btnDigiDown) ? *digit1 | CELL_PAD_CTRL_DOWN : *digit1 & ~CELL_PAD_CTRL_DOWN;
  *digit1 = (report->buttons & btnDigiRight) ? *digit1 | CELL_PAD_CTRL_RIGHT : *digit1 & ~CELL_PAD_CTRL_RIGHT;
  *digit1 = (report->buttons & btnDigiUp) ? *digit1 | CELL_PAD_CTRL_UP : *digit1 & ~CELL_PAD_CTRL_UP;
  *digit1 = (report->buttons & btnStart) ? *digit1 | CELL_PAD_CTRL_START : *digit1 & ~CELL_PAD_CTRL_START;
  *digit1 = (report->buttons & btnHatRight) ? *digit1 | CELL_PAD_CTRL_R3 : *digit1 & ~CELL_PAD_CTRL_R3;
  *digit1 = (report->buttons & btnHatLeft) ? *digit1 | CELL_PAD_CTRL_L3 : *digit1 & ~CELL_PAD_CTRL_L3;
  *digit1 = (report->buttons & btnBack) ? *digit1 | CELL_PAD_CTRL_SELECT : *digit1 & ~CELL_PAD_CTRL_SELECT;
  *digit2 = (report->buttons & btnX) ? *digit2 | CELL_PAD_CTRL_SQUARE : *digit2 & ~CELL_PAD_CTRL_SQUARE;
  *digit2 = (report->buttons & btnA) ? *digit2 | CELL_PAD_CTRL_CROSS : *digit2 & ~CELL_PAD_CTRL_CROSS;
  *digit2 = (report->buttons & btnB) ? *digit2 | CELL_PAD_CTRL_CIRCLE : *digit2 & ~CELL_PAD_CTRL_CIRCLE;
  *digit2 = (report->buttons & btnY) ? *digit2 | CELL_PAD_CTRL_TRIANGLE : *digit2 & ~CELL_PAD_CTRL_TRIANGLE;
  *digit2 = (report->buttons & btnShoulderRight) ? *digit2 | CELL_PAD_CTRL_R1 : *digit2 & ~CELL_PAD_CTRL_R1;
  *digit2 = (report->buttons & btnShoulderLeft) ? *digit2 | CELL_PAD_CTRL_L1 : *digit2 & ~CELL_PAD_CTRL_L1;
  *digit2 = (report->trigL > 0) ? *digit2 | CELL_PAD_CTRL_L2 : *digit2 & ~CELL_PAD_CTRL_L2;
  *digit2 = (report->trigR > 0) ? *digit2 | CELL_PAD_CTRL_R2 : *digit2 & ~CELL_PAD_CTRL_R2;

  // emulate pressure values except for L2 and R2, button presses correspond to max sensitivity value
  *press_l2 = (report->trigL);
  *press_r2 = (report->trigR);
  *press_l1 = (report->buttons & btnShoulderLeft) ? 0xFF : 0;
  *press_r1 = (report->buttons & btnShoulderRight) ? 0xFF : 0;
  *press_up = (report->buttons & btnDigiUp) ? 0xFF : 0;
  *press_down = (report->buttons & btnDigiDown) ? 0xFF : 0;
  *press_left = (report->buttons & btnDigiLeft) ? 0xFF : 0;
  *press_right = (report->buttons & btnDigiRight) ? 0xFF : 0;
  *press_square = (report->buttons & btnX) ? 0xFF : 0;
  *press_circle = (report->buttons & btnB) ? 0xFF : 0;
  *press_cross = (report->buttons & btnA) ? 0xFF : 0;
  *press_triangle = (report->buttons & btnY) ? 0xFF : 0;

  // PS3 pads use 8 bit values for each axis while Xbox pads use 16 bit
  // convert Xbox analog values for PS3 compatibility
  *analog_rx = (report->right.x-0x80) & 0x00FF;
  *analog_ry = ((report->right.y ^ 0xFF)-0x80) & 0x00FF;
  *analog_lx = (report->left.x-0x80) & 0x00FF;
  *analog_ly = ((report->left.y ^ 0xFF)-0x80) & 0x00FF;

  // send pad data to virtual pad
  cellPadLddDataInsert(handle[id], &data);
}

static int32_t xpad_read_input(int32_t id, void *data) {
  unsigned char *p;
  int32_t i;
  unsigned char *xpadbuf;
  XPAD_UNIT_t *unit;

  p = (unsigned char *)data;
  if (id > MAX_XPAD_NUM) {
    return(-1);
  }
  if ((unit = XPAD.con_unit[id]) == NULL) {
    return(-1);
  }

  // get from ringbuffer
  block(ringbuf_mutex);
  if (unit->rblen > 0) {
    xpadbuf = &unit->ringbuf[unit->rp][0];
    *p++ = xpadbuf[0]; // count
    *p++ = xpadbuf[1]; // size
    memcpy(p, &xpadbuf[2], unit->payload);
    if (++unit->rp >= RINGBUF_SIZE) {
      unit->rp = 0;
    }
    XBOX360_IN_REPORT *report = (XBOX360_IN_REPORT *)p;
    if ((report->header.command == inReport) && (report->header.size == sizeof(XBOX360_IN_REPORT))) {
      xpad_read_report(unit->number, p);
    }
    unit->rblen--;
  } else {
    *p++ = 0; // count
    memset(p, 0, unit->payload);
  }
  unblock(ringbuf_mutex);
  return(0);
}

static int32_t xpad_set_led(int32_t id, uint8_t led) {
  uint8_t out[3] = {0x01, 0x03, led};

  if (write_xpad(id, out, 3) < 0) {
    return(-1);
  }
  return(CELL_OK);
}

static int32_t xpad_set_rumble(int32_t id, uint8_t lval, uint8_t rval) {
  uint8_t out[8] = {0x00, 0x08, 0x00, lval, rval, 0x00, 0x00, 0x00};

  if (write_xpad(id, out, 8) < 0) {
    return(-1);
  }
  return(CELL_OK);
}
// end of wired controller specific methods 

// start of wireless controller specific methods
static int32_t get_device_desc(int32_t dev_id, void *p) {
  (void) dev_id;
  UsbDeviceDescriptor *desc = (UsbDeviceDescriptor *)p;

  // do nothing
  return(CELL_OK);
}

static int32_t get_configration_desc(int32_t dev_id, void *p) {
  (void) dev_id;
  UsbConfigurationDescriptor *desc = (UsbConfigurationDescriptor *)p;
  uint16_t wTotalLength;
  wTotalLength = SWAP16(desc->wTotalLength);

  // do nothing
  return(CELL_OK);
}

static int32_t get_interface_desc(int32_t dev_id, void *p) {
  (void) dev_id;
  UsbInterfaceDescriptor *desc = (UsbInterfaceDescriptor *)p;

  // do nothing
  return(CELL_OK);
}

static int32_t get_endpoint_desc(int32_t dev_id, void *p) {
  (void) dev_id;
  UsbEndpointDescriptor *edesc = (UsbEndpointDescriptor *)p;
  int32_t payload;
  XPAD_UNIT_t *unit;

  if (edesc->bEndpointAddress == 0x81 || edesc->bEndpointAddress == 0x83 || edesc->bEndpointAddress == 0x85 || edesc->bEndpointAddress == 0x87) {
    payload = SWAP16(edesc->wMaxPacketSize);
    if ((unit = unit_alloc(dev_id, payload, (edesc->bEndpointAddress - 0x01) & 0x0f, 0, XTYPE_XBOX360W)) == NULL) {
      return(CELL_USBD_ATTACH_FAILED);
    }
    if ((unit->c_pipe = cellUsbdOpenPipe(dev_id, NULL)) < 0) {
      unit_free(unit);
      return(CELL_USBD_ATTACH_FAILED);
    }
    if ((unit->i_pipe = cellUsbdOpenPipe(dev_id, edesc)) < 0) {
      unit_free(unit);
      return(CELL_USBD_ATTACH_FAILED);
    }
    edesc->bEndpointAddress &= 0x0f; // XBox controller out endpoint, ex: 0x81 & 0x0f == 0x01
    if ((unit->o_pipe = cellUsbdOpenPipe(dev_id, edesc)) < 0) {
      unit_free(unit);
      return(CELL_USBD_ATTACH_FAILED);
    }

    // endpoint found, set configuration and add to connected controllers list
    cellUsbdSetConfiguration(unit->c_pipe, 1, set_config_done, unit);
    block(xpad_mutex);
    XPAD.n++;
    XPAD.is_connected[unit->number] = 1;
    XPAD.con_unit[unit->number] = unit;
    unblock(xpad_mutex);
  }
  return(CELL_USBD_ATTACH_SUCCEEDED);
}

static int32_t xpadw_probe(int32_t dev_id) {
  uint16_t idVendor, idProduct;
  uint32_t i;
  UsbDeviceDescriptor *ddesc;
  UsbInterfaceDescriptor *idesc;

  block(xpad_mutex);
  if (XPAD.n >= MAX_XPAD_NUM) {
    unblock(xpad_mutex);
    return(CELL_USBD_PROBE_FAILED);
  }
  unblock(xpad_mutex);
  if ((ddesc = (UsbDeviceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, NULL, USB_DESCRIPTOR_TYPE_DEVICE)) == NULL) {
    return(CELL_USBD_PROBE_FAILED);
  }
  idesc = (UsbInterfaceDescriptor *)ddesc;
  if ((idesc = (UsbInterfaceDescriptor *)cellUsbdScanStaticDescriptor(dev_id, idesc, USB_DESCRIPTOR_TYPE_INTERFACE)) == NULL) {
    return(CELL_USBD_PROBE_FAILED);
  }

  // make sure product id and vendor id are valid
  idVendor = SWAP16(ddesc->idVendor);
  idProduct = SWAP16(ddesc->idProduct);
  for (i = 0; i < MAX_XPADW_DEV_NUM; i++) {
    if (xpadw_info[i].vid == idVendor && xpadw_info[i].pid == idProduct) {
      return(CELL_USBD_PROBE_SUCCEEDED);
    }
  }
  return(CELL_USBD_PROBE_FAILED);
}

static int xpadw_attach(int32_t dev_id) {
  uint8_t* desc = 0;
  uint32_t i;

  // Xbox 360 wireless receivers have 4 endpoints (1 per controller)
  // all 4 need to be listened to at all times in case of controller connection/disconnection
  // traverse through its usb device descriptor and find the endpoints
  while (1) {
    if ((desc = cellUsbdScanStaticDescriptor(dev_id, desc, 0)) == 0) {
        break;
    }
    for (i = 0; i < DESCRIPTOR_TABLE_SIZE; i++) {
      if (descriptor_table[i].bDescriptorType == desc[1]) {
        break;
      }
    }
    if (i != DESCRIPTOR_TABLE_SIZE) {
      descriptor_table[i].dump_descriptor(dev_id, desc);
    }
  }
  return(CELL_USBD_ATTACH_SUCCEEDED);
}

static int32_t xpadw_detach(int32_t dev_id) {
  int32_t r;

  // Xbox wireless receiver has been unplugged
  // disconnect all virtual controllers associated to it
  r = xpadw_detach_all();
  return(r);
}

static int32_t xpadw_detach_all(void) {
  int32_t i;
  XPAD_UNIT_t *unit;

  // detach all wireless controllers
  block(xpad_mutex);
  for (i = 0; i < MAX_XPAD_NUM; i++) {
    if (XPAD.is_connected[i]) {
      unit = XPAD.con_unit[i];
      if (unit->xtype == XTYPE_XBOX360) {
        XPAD.n--;
        XPAD.is_connected[unit->number] = 0;
        XPAD.con_unit[unit->number] = NULL;
        unregister_ldd_controller(unit);
        unit_free(unit);
      }
    }
  }
  unblock(xpad_mutex);
  return(CELL_USBD_DETACH_SUCCEEDED);
}

static void xpadw_read_report(int32_t id, uint8_t *readBuf) {
  uint16_t *digit0, *digit1, *digit2,
           *analog_rx, *analog_ry, *analog_lx, *analog_ly,
           *press_l2, *press_r2, *press_l1, *press_r1,
           *press_up, *press_down, *press_left, *press_right,
           *press_cross, *press_square, *press_circle, *press_triangle;
  XBOX360W_IN_REPORT *report;
  CellPadData data;

  report = (XBOX360W_IN_REPORT *)readBuf;
  memset(&data, 0, sizeof(CellPadData));
  data.len = 24;

  // map location of each button in virtual pad's data
  digit0 = &data.button[0];
  digit1 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL1];
  digit2 = &data.button[CELL_PAD_BTN_OFFSET_DIGITAL2];
  analog_rx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X];
  analog_ry = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y];
  analog_lx = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X];
  analog_ly = &data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y];
  press_l2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L2];
  press_r2 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R2];
  press_l1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_L1];
  press_r1 = &data.button[CELL_PAD_BTN_OFFSET_PRESS_R1];
  press_right = &data.button[CELL_PAD_BTN_OFFSET_PRESS_RIGHT];
  press_left = &data.button[CELL_PAD_BTN_OFFSET_PRESS_LEFT];
  press_up = &data.button[CELL_PAD_BTN_OFFSET_PRESS_UP];
  press_down = &data.button[CELL_PAD_BTN_OFFSET_PRESS_DOWN];
  press_cross = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CROSS];
  press_square = &data.button[CELL_PAD_BTN_OFFSET_PRESS_SQUARE];
  press_circle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_CIRCLE];
  press_triangle = &data.button[CELL_PAD_BTN_OFFSET_PRESS_TRIANGLE];

  // set default controller values
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y] = 0x0080;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_X] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Y] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_Z] = 0x0200;
  data.button[CELL_PAD_BTN_OFFSET_SENSOR_G] = 0x0200;

  // read values from Xbox controller and map it to the virtual pad
  *digit0 = (report->buttons & btnXbox) ? *digit0 | CELL_PAD_CTRL_LDD_PS : *digit0 & ~CELL_PAD_CTRL_LDD_PS;
  *digit1 = (report->buttons & btnDigiLeft) ? *digit1 | CELL_PAD_CTRL_LEFT : *digit1 & ~CELL_PAD_CTRL_LEFT;
  *digit1 = (report->buttons & btnDigiDown) ? *digit1 | CELL_PAD_CTRL_DOWN : *digit1 & ~CELL_PAD_CTRL_DOWN;
  *digit1 = (report->buttons & btnDigiRight) ? *digit1 | CELL_PAD_CTRL_RIGHT : *digit1 & ~CELL_PAD_CTRL_RIGHT;
  *digit1 = (report->buttons & btnDigiUp) ? *digit1 | CELL_PAD_CTRL_UP : *digit1 & ~CELL_PAD_CTRL_UP;
  *digit1 = (report->buttons & btnStart) ? *digit1 | CELL_PAD_CTRL_START : *digit1 & ~CELL_PAD_CTRL_START;
  *digit1 = (report->buttons & btnHatRight) ? *digit1 | CELL_PAD_CTRL_R3 : *digit1 & ~CELL_PAD_CTRL_R3;
  *digit1 = (report->buttons & btnHatLeft) ? *digit1 | CELL_PAD_CTRL_L3 : *digit1 & ~CELL_PAD_CTRL_L3;
  *digit1 = (report->buttons & btnBack) ? *digit1 | CELL_PAD_CTRL_SELECT : *digit1 & ~CELL_PAD_CTRL_SELECT;
  *digit2 = (report->buttons & btnX) ? *digit2 | CELL_PAD_CTRL_SQUARE : *digit2 & ~CELL_PAD_CTRL_SQUARE;
  *digit2 = (report->buttons & btnA) ? *digit2 | CELL_PAD_CTRL_CROSS : *digit2 & ~CELL_PAD_CTRL_CROSS;
  *digit2 = (report->buttons & btnB) ? *digit2 | CELL_PAD_CTRL_CIRCLE : *digit2 & ~CELL_PAD_CTRL_CIRCLE;
  *digit2 = (report->buttons & btnY) ? *digit2 | CELL_PAD_CTRL_TRIANGLE : *digit2 & ~CELL_PAD_CTRL_TRIANGLE;
  *digit2 = (report->buttons & btnShoulderRight) ? *digit2 | CELL_PAD_CTRL_R1 : *digit2 & ~CELL_PAD_CTRL_R1;
  *digit2 = (report->buttons & btnShoulderLeft) ? *digit2 | CELL_PAD_CTRL_L1 : *digit2 & ~CELL_PAD_CTRL_L1;
  *digit2 = (report->trigL > 0) ? *digit2 | CELL_PAD_CTRL_L2 : *digit2 & ~CELL_PAD_CTRL_L2;
  *digit2 = (report->trigR > 0) ? *digit2 | CELL_PAD_CTRL_R2 : *digit2 & ~CELL_PAD_CTRL_R2;

  // emulate pressure values except for L2 and R2, button presses correspond to max sensitivity value
  *press_l2 = (report->trigL);
  *press_r2 = (report->trigR);
  *press_l1 = (report->buttons & btnShoulderLeft) ? 0xFF : 0;
  *press_r1 = (report->buttons & btnShoulderRight) ? 0xFF : 0;
  *press_up = (report->buttons & btnDigiUp) ? 0xFF : 0;
  *press_down = (report->buttons & btnDigiDown) ? 0xFF : 0;
  *press_left = (report->buttons & btnDigiLeft) ? 0xFF : 0;
  *press_right = (report->buttons & btnDigiRight) ? 0xFF : 0;
  *press_square = (report->buttons & btnX) ? 0xFF : 0;
  *press_circle = (report->buttons & btnB) ? 0xFF : 0;
  *press_cross = (report->buttons & btnA) ? 0xFF : 0;
  *press_triangle = (report->buttons & btnY) ? 0xFF : 0;

  // PS3 pads use 8 bit values for each axis while Xbox pads use 16 bit
  // convert Xbox analog values for PS3 compatibility 
  *analog_rx = (report->right.x-0x80) & 0x00FF;
  *analog_ry = ((report->right.y ^ 0xFF)-0x80) & 0x00FF;
  *analog_lx = (report->left.x-0x80) & 0x00FF;
  *analog_ly = ((report->left.y ^ 0xFF)-0x80) & 0x00FF;

  // send pad data to virtual pad
  cellPadLddDataInsert(handle[id], &data);
}

static int32_t xpadw_read_input(int32_t id, void *data) {
  unsigned char *p;
  int32_t i, port;
  unsigned char *xpadbuf;
  XBOX360W_IN_REPORT *report;
  XPAD_UNIT_t *unit;

  p = (unsigned char *)data;
  if (id > MAX_XPAD_NUM) {
    return(-1);
  }
  if ((unit = XPAD.con_unit[id]) == NULL) {
    return(-1);
  }

  // get from ringbuffer
  block(ringbuf_mutex);
  if (unit->rblen > 0) {
    xpadbuf = &unit->ringbuf[unit->rp][0];
    *p++ = xpadbuf[0]; // count
    *p++ = xpadbuf[1]; // size
    memcpy(p, &xpadbuf[2], unit->payload);
    if (++unit->rp >= RINGBUF_SIZE) {
      unit->rp = 0;
    }
    if (p[0] == 0x08 && p[1] == 0x80) {

      // controller connected to receiver
      register_ldd_controller(unit);
    } else if (p[0] == 0x08 && p[1] == 0x00) {

      // controller disconnected from receiver
      unregister_ldd_controller(unit);
    }
    report = (XBOX360W_IN_REPORT *)p;
    if ((p[1] == 0x01) && (report->header.command == inReport) && (report->header.size == sizeof(XBOX360W_IN_REPORT))) {
      xpadw_read_report(unit->number, p);
    }
    unit->rblen--;
  } else {
    *p++ = 0; /* count */
    memset(p, 0, unit->payload);
  }
  unblock(ringbuf_mutex);
  return(0);
}

static int32_t xpadw_set_led(int32_t id, uint8_t led) {
  uint8_t out[4] = {0x00, 0x00, 0x08, led | 0x40};

  if (write_xpad(id, out, 4) < 0) {
    return(-1);
  }
  return(CELL_OK);
}

static int32_t xpadw_set_rumble(int32_t id, uint8_t lval, uint8_t rval) {
  uint8_t out[7] = {0x00, 0x01, 0x0f, 0xc0, 0x00, lval, rval};

  if (write_xpad(id, out, 7) < 0) {
    return(-1);
  }
  return(CELL_OK);
}
// end of wireless controller specific methods

static int32_t check_pad_status(void) {
  int32_t i, cr, port, pad;
  XPAD_UNIT_t *unit;
  CellPadInfo2 pad_info2;

  cr = cellPadGetInfo2(&pad_info2);
  if (cr != CELL_PAD_OK) {
    return(-1);
  }
  for (pad = 0; pad < CELL_PAD_MAX_PORT_NUM; ++pad) {
    if (pad_info2.port_status[pad] & CELL_PAD_STATUS_ASSIGN_CHANGES) {

      // controller port changed, assign new led's to all controllers
      for (i = 0; i < MAX_XPAD_NUM; i++) {
        if (XPAD.is_connected[i] > 0) {
          unit = XPAD.con_unit[i];
          port = cellPadLddGetPortNo(handle[i]);
          unit->set_led(unit->number, xpad_led[port%4]);
        }
      }
    }
  }
  return(0);
}

static int32_t init_usb(void) {
  int32_t r, i;
  sys_mutex_attribute_t mutex_attr1;
  sys_mutex_attribute_t mutex_attr2;

  sys_mutex_attribute_initialize(mutex_attr1);
  sys_mutex_attribute_initialize(mutex_attr2);
  if ((r = sys_mutex_create(&ringbuf_mutex, &mutex_attr1)) != CELL_OK) {
    return(r);
  }
  if ((r = sys_mutex_create(&xpad_mutex, &mutex_attr2)) != CELL_OK) {
    return(r);
  }

  // initialize all controller handlers
  memset(handle, -1, sizeof(int32_t) * CELL_PAD_MAX_PORT_NUM);

  // register wired Xbox controller device types
  memset(&XPAD, 0, sizeof(XPAD));
  for (i = 0; i < MAX_XPAD_DEV_NUM; i++) {
    xpad_ops.name = xpad_info[i].name;
    if ((r = cellUsbdRegisterExtraLdd(&xpad_ops, xpad_info[i].vid, xpad_info[i].pid)) != CELL_OK) {
      return(r);
    }
  }

  // register wireless Xbox controller device types
  for (i = 0; i < MAX_XPADW_DEV_NUM; i++) {
    xpadw_ops.name = xpadw_info[i].name;
    if ((r = cellUsbdRegisterExtraLdd(&xpadw_ops, xpadw_info[i].vid, xpadw_info[i].pid)) != CELL_OK) {
      return(r);
    }
  }
  return(CELL_OK);
}

static int32_t shutdown_usb(void) {
  int32_t r;

  if (( r = cellUsbdUnregisterExtraLdd(&xpad_ops)) != CELL_OK) {
    return(r);
  }
  if (( r = cellUsbdUnregisterExtraLdd(&xpadw_ops)) != CELL_OK) {
    return(r);
  }
  if ((r = sys_mutex_destroy(ringbuf_mutex)) != CELL_OK) {
    return(r);
  }
  if ((r = sys_mutex_destroy(xpad_mutex)) != CELL_OK) {
    return(r);
  }
  return(CELL_OK);
}

static int xpadd_thread(uint64_t arg) {
  unsigned char xpad_data[MAX_XPAD_DATA_LEN];
  int32_t i, r;
  XPAD_UNIT_t *unit;

  r = init_usb();
  if (r < 0) {
    sys_ppu_thread_exit(0);
  }

  // wait until we're back in xmb
  sys_timer_sleep(10);
  show_msg((char *)"XPAD Loaded!");
  running = 1;
  while (running) {
    sys_timer_usleep(1000 * 10);
    block(xpad_mutex);
    for (i = 0; i < MAX_XPAD_NUM; i++) {
      if (XPAD.is_connected[i] > 0) {
        unit = XPAD.con_unit[i];
        unit->read_input(i, xpad_data);
      }
    }
    check_pad_status();
    unblock(xpad_mutex);
  }

  // exiting...
  xpad_detach_all();
  xpadw_detach_all();
  shutdown_usb();
  sys_ppu_thread_exit(0);
  return(0);
}

int xpadd_start(uint64_t arg) {
  sys_ppu_thread_create(&thread_id, xpadd_thread, NULL, -0x1d8, 0x2000, SYS_PPU_THREAD_CREATE_JOINABLE, THREAD_NAME);
  _sys_ppu_thread_exit(0);
  return(SYS_PRX_RESIDENT);
}

static void xpadd_stop_thread(uint64_t arg) {
  uint64_t exit_code;

  running = 0;
  sys_timer_usleep(500000);
  if (thread_id != (sys_ppu_thread_t)-1) {
    sys_ppu_thread_join(thread_id, &exit_code);
  }
  show_msg("XPAD Unloaded!");
  sys_ppu_thread_exit(0);
}

int xpadd_stop(void) {
  sys_ppu_thread_t t;
  uint64_t exit_code;

  sys_ppu_thread_create(&t, xpadd_stop_thread, 0, 0, 0x2000, SYS_PPU_THREAD_CREATE_JOINABLE, STOP_THREAD_NAME);
  sys_ppu_thread_join(t, &exit_code);
  _sys_ppu_thread_exit(0);
  return(SYS_PRX_STOP_OK);
}
