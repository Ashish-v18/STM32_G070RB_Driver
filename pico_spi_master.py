import machine
import utime

# Pico SPI Master Configuration
# GP0 = MISO (RX)
# GP3 = MOSI (TX)
# GP2 = SCK
# GP1 = CSn

spi_sck = machine.Pin(2)
spi_miso = machine.Pin(0)
spi_mosi = machine.Pin(3)
spi_cs = machine.Pin(1, machine.Pin.OUT)

spi = machine.SPI(0,
                  baudrate=500000, # 500kHz
                  polarity=0,
                  phase=0,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=spi_sck,
                  mosi=spi_mosi,
                  miso=spi_miso)

print("SPI Master Transmitter Started")
spi_cs.value(1) # Deselect

msg = b"Ashish kumar RCB RCB RCB"

while True:
    print("Sending:", msg)
    
    spi_cs.value(0) # Select Slave (Low)
    spi.write(msg)
    spi_cs.value(1) # Deselect Slave (High)
    
    utime.sleep(1) # Send every 1 second
