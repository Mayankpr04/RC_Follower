from machine import UART, Pin, Timer
import bluetooth
import time

#--- CONFIG ---
target_mac = "DD:34:02:0A:46:99"
RSSI_TOLERANCE = 10      # discard values outside ±10dBm
SCAN_DURATION_MS = 400   # 500 milliseconds
TX_PIN = 8               # ESP32 TX → Arduino RX (D2)
RX_PIN = 7               # unused, but required
BAUD_RATE = 9600
red_led = Pin(13, Pin.OUT)

#--- INIT ---
target_mac_bytes = bytes(int(b, 16) for b in target_mac.split(":"))
uart = UART(1, baudrate=BAUD_RATE, tx=TX_PIN, rx=RX_PIN)
ble = bluetooth.BLE()
ble.active(True)
red_led.value(1)
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

def reportAvg(timer1):
    global kf_rssi
    print(f"Distance avg = {kf_rssi}, Cycles is {cycles}")

#####KALMAN FILTER TESTING
class KalmanFilter:
    def __init__(self, process_noise=1e-3, measurement_noise=1, estimate_error=1, initial_estimate=0):
        self.q = process_noise          # Process noise covariance (Q)
        self.r = measurement_noise      # Measurement noise covariance (R)
        self.p = estimate_error         # Estimation error covariance (P)
        self.x = initial_estimate       # Value (x)
        #Q - how much we expect the RSSI value to change between readings, can be .001 or large numbers, controls how fast
        #the filter will respond to changes in the system. Think rubber banding or slow stepwise to the new avg. This controls that.
        
        #R - how noisy the RSSI readings are, again help controls the variance calculation and how quickly the system will respond.
        #think of this as like the fine tuning while Q is the hard jumping variable. Lower = faster, higher=smoother
        
        #P - set as the initial uncertainty of the initial ESTIMATE. This should generally be larger initially as the system will change
        #it to a value that makes more sense over time. I assume it is high here since we want the KF to be evolutionary.
        
        #X - our initial estimate of what it might be. This is our initial state we tell the KF to assume. Changing Q and R controls how
        #fast this initial guess will change. Changing P controls how quickly our initial guess will change.
        
    def update(self, measurement):
        # Prediction update
        self.p = self.p + self.q

        # Measurement update
        k = self.p / (self.p + self.r)  # Kalman gain
        self.x = self.x + k * (measurement - self.x) #update estimate
        self.p = (1 - k) * self.p #update estimation error

        return self.x

    
######################


#--- Main Loop ---
#print("Tracking beacon... Sending integer RSSI to Arduino.")
tim0 = Timer(0)
#tim0.init(period=5000,mode=Timer.PERIODIC, callback=reportAvg)
cycles = 0
distance_avg = 0
kf = KalmanFilter(process_noise=0.5, measurement_noise=5, estimate_error=3, initial_estimate=-70)
kf_rssi = 0
while True:
    rssi_list = []
    
    

    ble.gap_scan(None)
    ble.gap_scan(SCAN_DURATION_MS, 30000, 30000)
    
    time.sleep_ms(SCAN_DURATION_MS)

    #filtered = discard_outliers(rssi_list, RSSI_TOLERANCE)
    avg_rssi = average(rssi_list) #allow averaging of readings to happen still since it wont change much
    if avg_rssi !=-999:
        kf_rssi = kf.update(avg_rssi)
        cycles +=1

    uart.write(f"{avg_rssi}\n")  # Send as plain integer
    print(f"Un-KF'd RSSI: {avg_rssi}, Raw: {rssi_list}, Filtered: {kf_rssi}")

    #time.sleep(0.1)