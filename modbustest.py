import minimalmodbus
import serial
import time

PORT = "/dev/tty.usbserial-210"
SLAVE_ID = 1
BAUDRATE = 38400

inst = minimalmodbus.Instrument(PORT, SLAVE_ID)
inst.serial.baudrate = BAUDRATE
inst.serial.bytesize = 8
inst.serial.parity   = serial.PARITY_NONE
inst.serial.stopbits = 1
inst.serial.timeout  = 0.3
inst.mode = minimalmodbus.MODE_RTU
inst.clear_buffers_before_each_transaction = True
inst.close_port_after_each_call = True

# ---- READ CURRENT VALUES ----
pos_kp = inst.read_register(5, 0)
hold_current = inst.read_register(11, 0)

print("Current position Kp:", pos_kp)
print("Open-loop hold current (100mA):", hold_current)

# ---- WRITE NEW VALUES ----
NEW_HOLD_CURRENT = 30  # 3.0 A

inst.write_register(11, NEW_HOLD_CURRENT, 0)
time.sleep(0.1)

# ---- SAVE TO EEPROM ----
inst.write_register(33, 1, 0)
time.sleep(0.2)

print("Saved. Power-cycle the drive if needed.")