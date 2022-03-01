from machine import Pin, ADC, UART, SoftI2C, Timer, deepsleep
from time import sleep_ms
import network, urequests, sys, uSGP30, esp32

# ADC Declarations
ADC_TEMP_PIN = 34
ADC_PRES_PIN = 39

# UART Declarations
UART_CHANNEL_CELL = 2
UART_CELL_BR = 115200

# I2C Declarations
I2C_SDA_PIN = 23
I2C_SCL_PIN = 22
SCL_CLK_FREQ = 400000
SGP30_BASE_ADDR = 0x58
SGP30_INNIT_CMD = bytes([0x20, 0x03])
SGP30_MEAS_CMD = bytes([0x20, 0x08])

# GPIO Declarations
EXTERNAL_WAKEUP_PIN = 26
BUZZER_PIN = 25
SOUND_BUZZER_TIME = 200
BLINK_LED_TIME = 500
LED_PIN = 4

# Timer Declarations
SENSOR_TIMER_PERIOD = 1000

# Other Declarations
SLEEP_TIME_THRESH = 10  # End value should be 60
MAX_TEMP_THRESH = 63  # End value should be 90
MAX_CO2_THRESH = 400  # End value should be around 800-1000
TEMP_INVALID_THRESH = 250

# Custom structure to hold sensor ojects and data readings
class Sensor_Readings:
    def __init__(self, temp, pres, cell, co2, ext_wake, buzzer, led):  # Constructor only requires peripheral objects
        self.temp = temp
        self.pres = pres
        self.cell = cell
        self.co2 = co2
        self.ext_wake = ext_wake
        self.buzzer = buzzer
        self.led = led
        self.current_temp = 0
        self.invalid_temp = 0
        self.alarm_temp = 0
        self.current_pres = 0
        self.invalid_pres = 0
        self.no_pres_time = 0
        self.current_co2 = 0
        self.alarm_co2 = 0
       
       
sensors = 0  # Global declaration for Sensor_Readings user defined data structure

# Intialization function: calls individual init functions and detects errors
# Returns all peripheral objects or exits if issue initializing peripherals
def init():
   
    # Initialize peripherals
    temp, pres = ADC_init()
    cell = UART_init()
    co2 = I2C_init()
    ext_wake, buzzer, led = GPIO_init()
   
    # Check periphals success
    # Print error and exit if any failed
    if temp == 0 or pres == 0:
        print("ADC_init() failed!")
        sys.exit()
   
    elif cell == 0:
        print("UART_init() failed!")
        sys.exit()
       
    elif co2 == 0:
        print("I2C_init() failed!")
        sys.exit()
       
    # If successful, return all peripherals    
    return temp, pres, cell, co2, ext_wake, buzzer, led
   

# Function to initialize the temperature and pressure sensor via ADC
# Returns the temperature and pressure adc channels
def ADC_init():
   
    # Intialize temperature ADC connection
    ADC_temp = Pin(ADC_TEMP_PIN, Pin.IN)
    adc_temp = ADC(ADC_temp)
    adc_temp.atten(ADC.ATTN_11DB)

    # Initialize sensor ADC connection
    ADC_pres = Pin(ADC_PRES_PIN, Pin.IN)
    adc_pres = ADC(ADC_pres)
    adc_pres.atten(ADC.ATTN_11DB)
   
    return (adc_temp, adc_pres)


# Function to initialize the cellular module via UART
# Returns the UART object if no errors, or 0 if cannot connect
def UART_init():
    sim800l = UART(UART_CHANNEL_CELL, UART_CELL_BR)
    sim800l.write('AT\r')
    sleep_ms(100)
    if str(sim800l.read()).find("OK") == 0:
        print("SIM800L not properly connected via UART!")
        return 0
   
    return sim800l


# Function to initialize the CO2 sensor via I2C
# Returns the I2C object if no errors, or 0 if no device detected
def I2C_init():
    sda = Pin(I2C_SDA_PIN)
    scl = Pin(I2C_SCL_PIN)
    i2c = SoftI2C(scl = scl, sda = sda, freq = SCL_CLK_FREQ)
    devices = i2c.scan()
    #print(devices) # Debugging
   
    # Case: SGP30 found on I2C bus, returned I2C object
    if SGP30_BASE_ADDR in devices:
        return i2c
   
    # Case: SGP30 not found on I2C bus, return 0 to indicate failure
    return 0


# Function to intialize all GPIO pins
# Returns a list of all the pin objects setup
def GPIO_init():
    ext_wake = Pin(EXTERNAL_WAKEUP_PIN, Pin.IN, Pin.PULL_DOWN)
    esp32.wake_on_ext0(pin = ext_wake, level = esp32.WAKEUP_ANY_HIGH)  # Configures pin as an external wakeup from deep sleep
   
    buzzer = Pin(BUZZER_PIN, Pin.OUT, Pin.PULL_DOWN)
   
    led = Pin(LED_PIN, Pin.OUT, Pin.PULL_DOWN)
   
    return ext_wake, buzzer, led


# Function to construct the Sensor Readings global structure with the periphal objects created in init()
def setup_Sensor_Readings(periph):
    global sensors
    sensors = Sensor_Readings(periph[0], periph[1], periph[2], periph[3], periph[4], periph[5], periph[6])
   
    return


# Function to setup the hardware timer to trigger get_sensor_data() once per second
def setup_timer():
    timer_data = Timer(1)
    timer_data.init(mode=Timer.PERIODIC, period = SENSOR_TIMER_PERIOD, callback = get_sensor_data)
   
    return



# Function to sample each sensor and upload the results to the Sensor Readings data structure
def get_sensor_data(temp_obj): # Temp_obj not used
   
    clear_flags()  # Clear any possible invalidity, or alarm flags before new readings
   
    # Update sensor object with new readings
    sensors.current_temp = get_temp_data()
    sensors.current_pres = get_pres_data()
    sensors.current_co2 = get_co2_data()
   
    # Set alarm flags and trigger alarm data if needed
    process_data()
   
    return


# Function to clear flags before next timer iteration
def clear_flags():
    sensors.invalid_temp = 0
    sensors.invalid_pres = 0
    sensors.alarm_temp = 0
    sensors.alarm_co2 = 0
   
    return


# Function to interpret sensor data to set alarm flags and enter alarm state
def process_data():
   
    print_data_readings() # For debugging
   
    set_alarms()  # Set alarms if needed
   
    # Case: No child in seat, inciment no_pres_time
    if sensors.current_pres == 0:
        sensors.no_pres_time += 1
       
        # Case: No child in seat for 60 seconds, put AlarmMe into deep sleep
        if sensors.no_pres_time >= SLEEP_TIME_THRESH:
            deepsleep()
        return
   
    # Case: Child detected in seat, set no_pres_time to 0
    elif sensors.current_pres > 0:
        sensors.no_pres_time = 0
   
    # Case: Alarm flags have been set, trigger alarm state
    if (sensors.alarm_temp or sensors.alarm_co2) and sensors.current_pres > 0:
        if sensors.invalid_temp == 0 and sensors.invalid_pres == 0:
            alarm()
           
    return


# Function to check current readings and determine if alarm state should be triggered
def set_alarms():
    if sensors.current_temp > MAX_TEMP_THRESH:
        sensors.alarm_temp = 1
       
    if sensors.current_co2 > MAX_CO2_THRESH:
        sensors.alarm_co2 = 1
       
    return
 
 
# Function to act as alarm state
# Triggers buzzer and LED light
# Sends SMS message through cell module
def alarm():
   
    sound_buzzer()
    blink_led()
    print("TODO: ALARM WITH CELL MODULE")
 

# Function to sound buzzer briefly in alarm state
def sound_buzzer():
    sensors.buzzer.value(1)
    sleep_ms(SOUND_BUZZER_TIME)
    sensors.buzzer.value(0)
 
    return
 
 
# Function to blink LED briefly in alarm state
def blink_led():
    sensors.led.value(1)
    sleep_ms(BLINK_LED_TIME)
    sensors.led.value(0)
   
    return
 
   
# Function to sample the ADC value from the temperature sensor and use the reading to calculate the temperature
# Returns a temperature value in Fahrenheit
def get_temp_data():
    temp = ((((1.8663 - ((sensors.temp.read_u16() * 3.3) / 65535)) / .01169) * 9) / 5) + 32 - 20
   
    # This reading is only possible if there is an issue with the sensor
    # So set invalid flag
    if temp > TEMP_INVALID_THRESH:
        sensors.invalid_temp = 1
    return temp


# Function to sample the ADC value from the pressure sensor
# Returns the raw reading from the sensor
def get_pres_data():
    pres = sensors.pres.read_u16()
    return pres


# Function to read the CO2 data from the CO2 sensor
def get_co2_data():
    sensors.co2.writeto(SGP30_BASE_ADDR, SGP30_INNIT_CMD)  # Send INIT command to sensor
    sleep_ms(12)
   
    sensors.co2.writeto(SGP30_BASE_ADDR, SGP30_MEAS_CMD)  # Send MEASURE command to sensor
    sleep_ms(12)
   
    buffer = bytearray(6)
    sensors.co2.readfrom_into(SGP30_BASE_ADDR, buffer)  # Read response from sensor

    result = [buffer[0], buffer[1]]  # Only take the needed CO2 readings
    co2 = result[0] << 8 | result[1]  # Bit shift into decimal, units are ppm

    return co2


# Idle
def run():
    while(1):
        continue


# Debugging purposes to print out the current sensor readings
# Currently just printing out temperature and pressure
def print_data_readings():
    print("Temp: " + str(sensors.current_temp) + "\nPres: " + str(sensors.current_pres) + "\nCO2: " + str(sensors.current_co2) + "\nTime w/ No Pres: " + str(sensors.no_pres_time) + "\n")


# Main control flow of program
periph = init()
setup_Sensor_Readings(periph)
setup_timer()
run()