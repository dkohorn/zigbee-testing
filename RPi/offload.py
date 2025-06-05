import smbus2
from datetime import datetime

I2C_BUS = 1
SLAVE_ADDR = 0x28
IDENTIFIER_RTT = 0xB2
RTT_OUTPUT_FILE = 'rtt_log.csv'

bus = smbus2.SMBus(I2C_BUS)

with open(RTT_OUTPUT_FILE, 'w') as f_rtt:
    print("Listening...")
    f_rtt.write("RTT (ms), Timestamp (ms)\n")
    
    while True:
        try:
            data = bus.read_i2c_block_data(SLAVE_ADDR, 0, 9)
            if data[0] == IDENTIFIER_RTT:
                value = int.from_bytes(data[1:9], byteorder='big', signed=False)
                currtime = datetime.now()
                print(f"Received value: {value} | {currtime}") 
                
                
                f_rtt.write(f"{value}, {currtime}\n")
                f_rtt.flush
        #Ensure program doesnt crash by checking the bus too early before esp starts
        except OSError as e:
            continue
