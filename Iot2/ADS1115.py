import I2C_LCD_driver
from time import sleep
import board
import busio
import adafruit_dht
import RPi.GPIO as GPIO
import blynklib
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

LDR = 17
relay = 27
motorstatus = False  # Initially, the motor is off

BLYNK_AUTH = 'YourAuthToken'  # Put your Auth Token here
blynk = blynklib.Blynk(BLYNK_AUTH)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set the relay pin as output pin
GPIO.setup(relay, GPIO.OUT)
GPIO.output(relay, GPIO.HIGH)  # Relay turns OFF

# Create an object for the LCD
lcd = I2C_LCD_driver.lcd()
# Create an object for the DHT11 sensor
DHT11 = adafruit_dht.DHT11(board.D4, use_pulseio=False)

# Initialize the ADC
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
chan_moisture = AnalogIn(ads, ADS.P0)
chan_rain = AnalogIn(ads, ADS.P1)

# Starting text
lcd.lcd_display_string("System Loading", 1, 1)
for a in range(0, 16):
    lcd.lcd_display_string(".", 2, a)
    sleep(0.1)
lcd.lcd_clear()

def TempHumi():
    try:
        # Get the Temperature and Humidity values
        temperature_c = DHT11.temperature
        humidity = DHT11.humidity

        # Print the values on the LCD display
        lcd.lcd_display_string("T:" + str(temperature_c) + ".0C", 1, 2)
        lcd.lcd_display_string("H:" + str(humidity) + "%", 2, 2)

        # Update temperature and humidity values to Blynk Cloud
        blynk.virtual_write(0, temperature_c)
        blynk.virtual_write(1, humidity)
        
    except RuntimeError as error:
        print(error.args[0])
        sleep(1)
    except Exception as error:
        DHT11.exit()
        raise error

    sleep(1)

def moistureValue():
    value = chan_moisture.value
    value = (value / 65535) * 100  # Convert 16-bit ADC values to percentage
    value = (value - 100) * -1
    value = int(value)
    lcd.lcd_display_string("Moisture:" + str(value) + "%  ", 1, 0)
    
    # Update moisture value to Blynk Cloud
    blynk.virtual_write(2, value)
    return value

def rain():
    value = chan_rain.value
    value = (value / 65535) * 100
    value = (value - 100) * -1
    value = int(value)
    lcd.lcd_display_string("R:" + str(value) + "% ", 2, 8)
    
    # Update rain value to Blynk Cloud
    blynk.virtual_write(4, value)

while True:
    blynk.run()
    TempHumi()
    rain()
    moisture = moistureValue()
    if moisture < 30 and not motorstatus:  # If moisture is low and motor is off
        GPIO.output(relay, GPIO.LOW)  # Relay turns ON
        lcd.lcd_display_string("Motor   :ON ", 2, 0)
        sleep(0.5)
        motorstatus = True
        # Update motor status to Blynk Cloud
        blynk.virtual_write(5, 'ON')
    elif moisture > 60 and motorstatus:  # If moisture is high and motor is on
        GPIO.output(relay, GPIO.HIGH)  # Relay turns OFF
        lcd.lcd_display_string("Motor   :OFF", 2, 0)
        sleep(0.5)
        motorstatus = False
        # Update motor status to Blynk Cloud
        blynk.virtual_write(5, 'OFF')
