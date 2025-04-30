from machine import UART
import bluetooth
import time

--- CONFIG ---
target_mac = "DC:0D:30:1F:65:E0"
RSSI_TOLERANCE = 10      # discard values outside +- 10dBm
SCAN_DURATION_MS = 1500  # 1.5 seconds
TX_PIN = 8               # ESP32 TX to Arduino RX (D2)
RX_PIN = 7               # unused, but required
BAUD_RATE = 9600

--- INIT ---
target_mac_bytes = bytes(int(b, 16) for b in target_mac.split(":"))
uart = UART(1, baudrate=BAUD_RATE, tx=TX_PIN, rx=RX_PIN)
ble = bluetooth.BLE()
ble.active(True)

_IRQ_SCAN_RESULT = 5
rssi_list = []

def bt_irq(event, data):
    global rssi_list
    if event == _IRQ_SCAN_RESULT:
        addr_type, addr, adv_type, rssi, adv_data = data
        if addr == target_mac_bytes:
            rssi_list.append(rssi)

ble.irq(bt_irq)

def average(values):
    return int(round(sum(values) / len(values))) if values else -999

def discard_outliers(values, tolerance):
    if not values:
        return []
    avg = sum(values) / len(values)
    return [v for v in values if abs(v - avg) <= tolerance]

--- Main Loop ---
print("Tracking beacon... Sending integer RSSI to Arduino.")

while True:
    rssi_list = []

    ble.gap_scan(None)
    ble.gap_scan(SCAN_DURATION_MS, 30000, 30000)
    time.sleep_ms(SCAN_DURATION_MS + 100)

    filtered = discard_outliers(rssi_list, RSSI_TOLERANCE)
    avg_rssi = average(filtered)

    uart.write(f"{avg_rssi}\n")  # Send as plain integer
    print(f"Sent RSSI: {avg_rssi}, Raw: {rssi_list}, Filtered: {filtered}\n")

    time.sleep(0.2)
