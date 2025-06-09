import smbus2
from datetime import datetime

I2C_BUS = 1
SLAVE_ADDR = 0x28
IDENTIFIER_RTT = 0xB2
IDENTIFIER_DISCONNECT_WATCHDOG = 0xA1
IDENTIFIER_DISCONNECT_NLME = 0xA2
IDENTIFIER_DISCONNECT_DEV_UNAVAILABLE = 0xA3
RTT_OUTPUT_FILE = 'rtt_log.csv'
DISCONNECT_OUTPUT_FILE = 'disconnect_log.csv'

bus = smbus2.SMBus(I2C_BUS)

with open(RTT_OUTPUT_FILE, 'w') as f_rtt, open(DISCONNECT_OUTPUT_FILE, 'w') as f_disc:
    print("Listening...")
    todays_date = datetime.now().strftime("%Y-%m-%d")
    f_rtt.write(f"RTT (ms), Timestamp, {todays_date}\n")
    f_disc.write(f"ESP time since boot (ms), Real time, Reason, {todays_date}\n")
    
    while True:
        try:
            data = bus.read_i2c_block_data(SLAVE_ADDR, 0, 9)
            if data[0] == IDENTIFIER_RTT or data[0] == IDENTIFIER_DISCONNECT_WATCHDOG or data[0] == IDENTIFIER_DISCONNECT_DEV_UNAVAILABLE or data[0] == IDENTIFIER_DISCONNECT_NLME:
                value = int.from_bytes(data[1:9], byteorder='big', signed=False)
                currtime = datetime.now()
                formatted_currtime = currtime.strftime("%H:%M:%S:%f")[:-3]
                print(f"Received value: {value} | {formatted_currtime} | {data[0]}") 
                
                if data[0] == IDENTIFIER_RTT:
                    f_rtt.write(f"{value}, {formatted_currtime}\n")
                    f_rtt.flush
                else:
                    if data[0] == IDENTIFIER_DISCONNECT_DEV_UNAVAILABLE:
                        f_disc.write(f"{value}, {formatted_currtime}, Dev Unavailable\n")
                    if data[0] == IDENTIFIER_DISCONNECT_WATCHDOG:
                        f_disc.write(f"{value}, {formatted_currtime}, Watchdog\n")
                    if data[0] == IDENTIFIER_DISCONNECT_NLME:
                        f_disc.write(f"{value}, {formatted_currtime}, NLME\n")
                    f_disc.flush
        #Ensure program doesnt crash by checking the bus too early before esp starts
        except OSError as e:
            continue
