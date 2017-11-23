#include "mbed.h"

#define N_CELLS                 (100)
#define N_BMS                   (5)

// 32000 = 3.2V
#define CELL_MIN_VOLTAGE        (27500)
#define CELL_MAX_VOLTAGE        (44000)

// 2200 = 22C
#define CELL_MIN_TEMP           (100)           // 1C
#define CELL_MAX_TEMP           (7000)          // 70C

#define PLAUSIBLE_LOWEST        (50)            // 0.5C
#define PLAUSIBLE_HIGHEST       (9500)          // 95C

#define BMS_TIMEOUT_TICKS       (6)
#define BMS_CHECK_FREQ_MILLISEC (500) // 2Hz, 8/2 = 4 sec max. hyst

DigitalOut amsOk(p7);
DigitalIn enReadings(p20); // Enable BMS, pulled low for charge mode

DigitalOut ledAms(LED1);
DigitalOut rxLed(LED2);
DigitalOut checkLed(LED3);
DigitalOut infoLed(LED4);

// Device i.e. 0x100, 0x200, 0x300 etc
#define BMS_DEVICE_MASK         (7 << 8)
#define BMS_DECODE_DEVICE(val)  ((val) >> 8)

// Category i.e. 0x110, 0x120, 0x130 etc
#define BMS_CAT_MASK            (7 << 4) // 11b << 4
#define BMS_DECODE_CAT(val)     ((val) >> 4)

// Offset from category i.e. 0x110 -> 0, 0x111 -> 1 etc
#define BMS_OFF_MASK            (3 << 0) // 11b, vals 0-2 inc

CAN can3(p9, p10);

/* Format from BMS:
    0xN10, 0xN20 -> Voltage data
        +0: cells 1, 2, 3, 4
        +1: cells 5, 6, 7, 8
        +2: cells 9, 10
        
    0xN30, 0xN40 -> Temperature data
        +0: cells 1, 4, 7, 10
        +1: cells 2, 5, 8
        +2: cells 3, 6, 9
*/

uint16_t cell_voltages[N_CELLS] = {0};
uint16_t cell_temperatures[N_CELLS] = {0};
uint16_t cell_temperature_blacklist[N_CELLS] = {0};
uint32_t recv_timestamps[N_BMS] = {0};

uint8_t voltages_ok = 0;
uint8_t temperatures_ok = 0;
bool timestamps_ok = false;

uint32_t t_now = 0;

bool check_voltages()
{
    for (unsigned i = 0; i < N_CELLS; i++) {
        if (cell_voltages[i] < CELL_MIN_VOLTAGE
          /*|| cell_voltages[i] > CELL_MAX_VOLTAGE*/) {
              return false;
        }
    }
    
    return true;
}

bool temperature_plausible(uint16_t temp)
{
    return (temp >= PLAUSIBLE_LOWEST && temp <= PLAUSIBLE_HIGHEST);
}

bool check_temperatures()
{
    for (unsigned i = 0; i < N_CELLS; i++) {
        if (!cell_temperature_blacklist[i]) {
            // Sometimes the connectors rattle and report false temps
            if (!temperature_plausible(cell_temperatures[i])) {
                continue;
            }
            
            if (/*cell_temperatures[i] < CELL_MIN_TEMP
               ||*/ cell_temperatures[i] > CELL_MAX_TEMP) {
                return false;
            }
        }
    }
    
    return true;
}

bool check_timestamps()
{
    for (unsigned i = 0; i < N_BMS; i++) {
        if (t_now-BMS_TIMEOUT_TICKS > recv_timestamps[i]) {
            return false;
        }
    }
    
    return true;
}

void cell_check_thread()
{
    for (;;) {
        voltages_ok = (voltages_ok << 1) | check_voltages();
        temperatures_ok = (temperatures_ok << 1) | check_temperatures();
        timestamps_ok = check_timestamps();

        bool ok = !(!voltages_ok || !temperatures_ok || !timestamps_ok);
        amsOk = ok || !enReadings;
        ledAms = ok;
        
        checkLed = !checkLed;
        Thread::wait(BMS_CHECK_FREQ_MILLISEC);
        t_now++;
    }
}

void save_bms_voltage(unsigned device, unsigned cat, unsigned offset, char *data, size_t n)
{
    unsigned cell_device = device; // 1 based
    unsigned cell_cat = cat - 1; // 1 based
    unsigned cell_off = (cell_device * 20) + (cell_cat * 10) + (offset * 4);
    unsigned cell_count = (offset == 2) ? 2 : 4;
    uint16_t *cell_data = reinterpret_cast<uint16_t*>(data);
    
    if (n != cell_count * sizeof(uint16_t)) { return; } // safeguard overrun
    
    for (unsigned i = 0; i < cell_count; i++) {
        cell_voltages[cell_off + i] = cell_data[i];
    }
}

void save_bms_temperature(unsigned device, unsigned cat, unsigned offset, char *data, size_t n)
{
    unsigned cell_device = device; // 0 based
    unsigned cell_cat = cat - 3; // 0 based, +2 ignore (voltages)
    unsigned cell_off = (cell_device * 20) + (cell_cat * 10);
    uint16_t *cell_data = reinterpret_cast<uint16_t*>(data);
    
    switch (offset) {
        case 0: // 0 3 6 9, base 0
            if (n != 8) return;
            cell_temperatures[cell_off + 0] = cell_data[0];
            cell_temperatures[cell_off + 3] = cell_data[1];
            cell_temperatures[cell_off + 6] = cell_data[2];
            cell_temperatures[cell_off + 9] = cell_data[3];
            break;
        case 1: // 1 4 7 base 0
            if (n != 6) return;
            cell_temperatures[cell_off + 1] = cell_data[0];
            cell_temperatures[cell_off + 4] = cell_data[1];
            cell_temperatures[cell_off + 7] = cell_data[2];
            break;
        case 2:
            if (n != 6) return;
            cell_temperatures[cell_off + 2] = cell_data[0];
            cell_temperatures[cell_off + 5] = cell_data[1];
            cell_temperatures[cell_off + 8] = cell_data[2];
            break;
    }
}

void save_bms_data(unsigned device, unsigned cat, unsigned offset, char *data, size_t n)
{
    recv_timestamps[device] = t_now;
    
    // We sent this
    if (cat == 0) return;
    
    // Either temperature or voltage
    switch (cat) {
        case 1: case 2: save_bms_voltage(device, cat, offset, data, n); break;
        case 3: case 4: save_bms_temperature(device, cat, offset, data, n); break;
    }
}

// Device i.e. 0x100, 0x200, 0x300 etc
#define BMS_DEVICE_MASK         (7 << 8)
#define BMS_DECODE_DEVICE(val)  ((val) >> 8)

// Category i.e. 0x110, 0x120, 0x130 etc
#define BMS_CAT_MASK            (7 << 4) // 11b << 4
#define BMS_DECODE_CAT(val)     ((val) >> 4)

// Offset from category i.e. 0x110 -> 0, 0x111 -> 1 etc
#define BMS_OFF_MASK            (3 << 0) // 11b, vals 0-2 incl

void can3_recv()
{
    CANMessage msg;
    
    while (can3.read(msg)) {
        rxLed = !rxLed;
        
        unsigned id = msg.id;
        unsigned device = BMS_DECODE_DEVICE(id & BMS_DEVICE_MASK) - 1;
        unsigned cat = BMS_DECODE_CAT(id & BMS_CAT_MASK);
        unsigned off = id & BMS_OFF_MASK;
        
        save_bms_data(device, cat, off, reinterpret_cast<char*>(&msg.data[0]), msg.len);
    }
}

void wakeup_thread_procedure()
{
    for (;;) {
        for (int i = 0; i < N_BMS; i++) {
            unsigned id = (i + 1) * 0x100;
            char data[1] = { enReadings };
            can3.write(CANMessage(id, data, sizeof(data)));
            
            infoLed = !infoLed;
            Thread::wait(100); // ms
        }
    }
}

int main() {
    // Init
    can3.frequency(125000);
    can3.attach(can3_recv);
    amsOk = false;
    
    // Thread to wake up the BMS
    Thread wakeup_thread;
    wakeup_thread.start(wakeup_thread_procedure);
    
    // Blacklist the last in each LTC module, they are sketchy
    /*
    for (unsigned i = 0; i < N_BMS; i++) {
        cell_temperature_blacklist[i*20 + 9] = 1;
        cell_temperature_blacklist[i*20 + 19] = 1;
    } */
    
    // Rest of blacklist
    cell_temperature_blacklist[17 - 1] = 1; // 17
    cell_temperature_blacklist[18 - 1] = 1; // 18
    cell_temperature_blacklist[19 - 1] = 1; // 19
    
    // Wait for everything to settle, capacitors to fill etc
    Thread::wait(250);
    
    // Run the cell checking thread forever
    cell_check_thread();
}

