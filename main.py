# MIT License
# 
# Copyright (c) 2024 Bob Harrison
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Stirling EngineTemp RPM Monitor with LCD Display 20241112
#    Thonny 4.0.2 , Python 3.10.9 on Windows 10 64-bit

# Pull-up (47K ohm) required for the 1-Wire input for the DS18x20 (Temperature)
# Pull-up (10K ohm) required for SCL / SDA pins for I2C

# references

#      https://how2electronics.com/interfacing-ds18b20-sensor-with-raspberry-pi-pico/
#      https://randomnerdtutorials.com/raspberry-pi-pico-ds18b20-micropython/
#      https://gemini.google.com/app/44ba9108ea22113b?cros_source=c

# Date / Version 20241112

import machine
import utime
import time

import onewire
import ds18x20
import RGB1602_01
import os


from machine import Pin, I2C
from picozero import pico_temp_sensor

# Initialize rpm variables
interrupt_count = 0
revolutions = 0  # Count number of rotations for the lifetime of program execution (2 interrupts per revolution)

last_time = 0  # used to calculate when next rpm read cycle should begin
rpm = 0  # used to evalute next rpm reading
last_rpm = 0  # used to save last rpm evualted

odd_even = 0  # a loop counter. When odd, display TempF Hot, when even, display TempF Cold on LCD

# Initialize I2C bus
# Required 10K ohm pull up resisters between SCL and VCC & SDA and VCC to remove Err 5

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)


# inspect to observe which I2C devices are on the port

devices = i2c.scan()
if len(devices) == 0:
    print("No i2c devices found")
else:
    print("i2c devices found:", len(devices))
for device in devices:
    print("Hex address: ", hex(device))

# Initialize waveshare RGB1602 sensor and validate output

lcd = RGB1602_01.RGB1602(16, 2, i2c=i2c)
lcd.setColorWhite()
lcd.clear()
lcd.setCursor(0, 0)
lcd.printout("Testing")
lcd.setCursor(0, 1)
lcd.printout("L ... E ... D")

LM393_int_pin = Pin(
    17, Pin.IN, Pin.PULL_UP
)  # interrupt fot he LM393 speed measuring module


onboardLED = machine.Pin(25, machine.Pin.OUT)  # flashes to show app is running

ds_pin = machine.Pin(22)  # pin for ds18x20 Temperature Senor OneWire comms

# inital values

tempF_ob = 0.00  # for onboard temp sensor
tempC_ob = 0.00  # for onboard temp sensor
tempF = 0.00  # for external ds18x20 temp sensor

sensor_temp_ob = machine.ADC(4)  # on board temp sensor
conversion_factor = 3.3 / (65535)

onboardLED.on()


def log_temp():
    # if called, log data to a local Pi Pico file for later analysis
    global tempF
    file_name = "temp_log.csv"
    timestamp = time.localtime()
    timestamp_str = format_timestamp(timestamp)

    temp = tempF

    # Check if CSV file exists, if not create it with a header row
    if not file_name in os.listdir():
        with open(file_name, "w") as file:
            file.write("Timestamp,Temp DegF\n")

    # Append position data to CSV file
    with open(file_name, "a") as file:
        file.write("{},{:.2f}\n".format(timestamp_str, tempF))
        print("{},{:.2f}".format(timestamp_str, tempF))


def format_timestamp(timestamp):
    year, month, day, hour, minute, second, *_ = timestamp
    timestamp_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        year, month, day, hour, minute, second
    )
    return timestamp_str


def LM393_handler(pin):
    global interrupt_count, revolutions, last_time, rpm, last_rpm
    # print ("RPM int")
    # Increment the interrupt count
    interrupt_count += 1
    revolutions += 1

    # Calculate time difference between interrupts
    current_time = time.ticks_ms()
    time_diff = current_time - last_time
    # print(time_diff, current_time, last_time)

    # Calculate RPM based on time difference and number of interrupts
    # Note : 2 interrupts per rotation. Once when sensor led path is blocked, once when it is unblocked

    if time_diff > 5000:  # evaluate new rpm every 5 seconds
        last_time = current_time
        rpm = ((float(interrupt_count) / 2.0) * (60.0 * 1000.0)) / float(
            time_diff
        )  # rpm over measurement period
        # print(rpm)
        last_rpm = rpm  # save current rpm value for use in main loop

        interrupt_count = 0  # reset for next measurement period
        rpm = 0  # reset for next measurement period


LM393_int_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=LM393_handler)

# set up temp sensor ds18x20
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))

roms = ds_sensor.scan()
print("Found DS devices: ", roms)


while True:
    time_now = (
        time.ticks_ms()
    )  # check to see if interrupts for LM323 have stopped, which means RPM is now zero
    if (
        time_now - last_time
    ) > 10000:  # there have been no LM323 interrupt for 10 seconds. Stirling Engine has stopped
        last_rpm = 0

    timestamp = time.localtime()
    timestamp_str = format_timestamp(timestamp)
    reading = sensor_temp_ob.read_u16() * conversion_factor

    tempC_ob = (
        pico_temp_sensor.temp - 1
    )  # or, try this library, calibrate by 1 degC for observed difference with ds18x20 temp sensor
    tempF_ob = round(((tempC_ob * 9 / 5) + 32), 2)
    print(
        last_rpm, ",", tempF_ob, ",", tempF, ",", revolutions / 2
    )  # revolutions/2 since there are two interrupts per revolution
    # print(last_rpm, "'", tempF_ob, ",", tempF, ",", timestamp_str)
    # print("Temp Deg C: ", tempC_ob, " Temp Deg F: ", tempF_ob)

    # for ds18x20 temp sensor
    ds_sensor.convert_temp()
    utime.sleep_ms(10)
    odd_even = odd_even + 1
    for rom in roms:
        # print(rom)
        tempC = ds_sensor.read_temp(rom)
        tempF = tempC * (9 / 5) + 32
        tempF_2f = str(
            "{:.2f}".format(tempF)
        )  # keep just the 2 digits after the decimal point
        lcd.clear()
        lcd.setCursor(8, 0)
        lcd.printout("RPM")
        lcd.setCursor(8, 1)
        lcd.printout(str("{:.2f}".format(last_rpm)))

        lcd.setCursor(0, 0)

        if odd_even % 2 == 0:
            # Even - Display TempF Cold
            lcd.printout("T Cold")
            lcd.setCursor(0, 1)
            lcd.printout(str(tempF_ob))
        else:
            # Odd - Display TempF Hot
            lcd.printout("T Hot")
            lcd.setCursor(0, 1)
            lcd.printout(str(tempF_2f))

        # print("temperature (ºC):", "{:.2f}".format(tempC))
        # print("temperature (ºF):", "{:.2f}".format(tempF))
        # log_temp()

    onboardLED.toggle()

    utime.sleep_ms(5000)

