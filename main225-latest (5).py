import RPi.GPIO as GPIO
import time
import serial
import adafruit_fingerprint
from picamera import PiCamera
import telepot
import smbus2
import datetime
import requests


# inisiasi pin
RELAY_PIN = 18
BUTTON_PIN = 16
buzzer_pin = 17

# inisiasi konfigurasi
GPIO.setmode(GPIO.BCM)


# Inisiasi pin Relay
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.setup(buzzer_pin, GPIO.OUT)

# Inisiasi  pin Button
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# inisiasi fingerprint
uart = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=1)
finger = adafruit_fingerprint.Adafruit_Fingerprint(uart)

# inisiasi kamera
camera = PiCamera()

# Inisiasi Ubidots
VARIABLE_ID = "64f04ebfe57b852fa067e3e9"
UBIDOTS_URL = f"https://industrial.api.ubidots.com/api/v1.6/variables/{VARIABLE_ID}/values/"
UBIDOTS_TOKEN = "BBFF-wevsahNVjQEKR2COeipycRyXtqLsIa"

bot = telepot.Bot('6461491207:AAEF2tCS5tAyKmNvftLe5Bne1hKwQy_2MuI')

I2C_ADDR = 0x27  # I2C address of the LCD module
LCD_WIDTH = 16   # Number of characters per line

# LCD commands
LCD_CHR = 1  # LCD character mode
LCD_CMD = 0  # LCD command mode
LCD_LINE_1 = 0x80  # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0  # LCD RAM address for the 2nd line

# Initialize I2C bus
bus = smbus2.SMBus(1)  # Use 1 for Raspberry Pi 2 and newer

def lcd_init():
    # Send initialization commands to the display
    lcd_byte(0x33, LCD_CMD)  # Initialize
    lcd_byte(0x32, LCD_CMD)  # Set to 4-bit mode
    lcd_byte(0x06, LCD_CMD)  # Cursor move direction
    lcd_byte(0x0C, LCD_CMD)  # Display ON, Cursor OFF, Blink OFF
    lcd_byte(0x28, LCD_CMD)  # 2 lines and 5x8 character matrix
    lcd_byte(0x01, LCD_CMD)  # Clear display
    time.sleep(0.0005)       # Delay to allow commands to process

def lcd_byte(bits, mode):
    # Send byte to data pins
    # bits = data
    # mode = 1 for character, 0 for command
    bits_high = mode | (bits & 0xF0) | 0x08
    bits_low = mode | ((bits << 4) & 0xF0) | 0x08

    # High bits
    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    # Low bits
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    # Toggle enable bit
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits | 0x04))
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits & ~0x04))
    time.sleep(0.0005)

def lcd_string(message, line):
    # Send string to display
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)

def send_to_ubidots():
    "Kirim data ke IoT Platform misal ubidots"
    pass

def send_to_telegram(photo_path):
    chat_id = '1924627029'
    bot.sendMessage(chat_id, 'IMPORTANT NOTIFICATION')
    photo_file = open(photo_path, 'rb')
    bot.sendMessage(chat_id, 'Smart Door detects threat, Immidiate action is required')
    bot.sendPhoto(chat_id, photo_file)
    
def button_callback(channel):
    "Callback button ketika ditekan"
    print("Button pressed")
    print("Pintu dibuka dari tombol!")
    control_solenoid("UNLOCK")
    # open for certain time then lock again
    time.sleep(2)
    control_solenoid("LOCK")

def beep():
    for _ in range(3):
        GPIO.output(buzzer_pin, GPIO.HIGH)
        time.sleep(1)  # Ubah durasi bunyi sesuai kebutuhan
        GPIO.output(buzzer_pin, GPIO.LOW)
        time.sleep(1)
    
def control_solenoid(state):
    "Untuk kontrol solenoid lock"
    if state == "LOCK":
        # kunci
        GPIO.output(RELAY_PIN, GPIO.HIGH)
    else:
        # membuka kunci
        GPIO.output(RELAY_PIN, GPIO.LOW)

def trigger_alarm(state):
    "Untuk menyalakan alarm"
    pass

def capture_image():
    "Fungsi untuk mengambil foto melalui kamera"
    time_taken = time.strftime('%d-%m-%Y_%H:%M%:%S')
    camera.start_preview()
    #time.sleep(1)
    camera.capture('/home/raspberry225/cctv/image_{}.jpg'.format(time_taken))
    camera.stop_preview()
    #camera.close()
    return '/home/raspberry225/cctv/image_{}.jpg'.format(time_taken)
    
def init_fingerprint():
    "Initialize Fingerprint functions"
    if finger.read_templates() != adafruit_fingerprint.OK:
        #raise RuntimeError("Failed to read templates")
        print("Failed to read templates")
    print("Fingerprint templates: ", finger.templates)
    if finger.count_templates() != adafruit_fingerprint.OK:
        #raise RuntimeError("Failed to read templates")
        print("Failed to read templates")
    print("Number of templates found: ", finger.template_count)
    if finger.read_sysparam() != adafruit_fingerprint.OK:
        #raise RuntimeError("Failed to get system parameters")
        print("Failed to get system parameters")
    print("Size of template library: ", finger.library_size)

def get_fingerprint():
    """Get a finger print image, template it, and see if it matches!"""
    print("Waiting for image...")
    start = time.time()
    while finger.get_image() != adafruit_fingerprint.OK:
        if time.time() - start >= 5:
            return None        
    print("Templating...")
    if finger.image_2_tz(1) != adafruit_fingerprint.OK:
        return False
    print("Searching...")
    if finger.finger_search() != adafruit_fingerprint.OK:
        return False
    return True

# Ubidots functions
def get_latest_ubidots_data():
    header = {
        'X-Auth-Token': UBIDOTS_TOKEN
    }
    param = {
        'start': int(datetime.datetime.utcnow().replace(hour=0, minute=0, second=0).strftime('%s')) * 1000,
        'page_size': 1
    }
    try:
        req = requests.get(UBIDOTS_URL, headers=header, params=param)
        if req.status_code == 200:
            response = req.json()
            values = response['results']
            last_value = None
            if len(values) > 0:
                last_value = values[0]['value']
            return last_value
        else:
            print(req.text)
            return None
            
        
    except Exception as e:
        print(e)
        return None

def main():
    lcd_init()
    lcd_byte(0x01, LCD_CMD)
    time.sleep(0.1)
    #init_mqtt_ubidots()
    # Inisiasi event callback physical button
    GPIO.add_event_detect(BUTTON_PIN, GPIO.RISING, callback=button_callback)
    control_solenoid("LOCK")
    previous_state = None
    while True:
        # Check latest button data
        latest_button_state = get_latest_ubidots_data()
        if latest_button_state != previous_state:
            if latest_button_state == 1.0:
                print("Pintu dibuka dari Ubidots")
                control_solenoid("UNLOCK")
            elif latest_button_state == 0.0:
                print("Pintu dikunci dari Ubidots")
                control_solenoid("LOCK")
            previous_state = latest_button_state
        
        try:
            init_fingerprint()
        except Exception as e:
            print(e)
        lcd_string("  PLEASE INPUT  ", LCD_LINE_1)
        lcd_string("YOUR FINGERPRINT", LCD_LINE_2)
        #1. read fingerprint sensor
        
        is_authenticated = get_fingerprint()
        
        # 2. ambil foto muka
        #capture_image()

        # 3. cek apakah fingerprint sudah tersimpan
        if is_authenticated is not None:
            if not is_authenticated:
                # 4. jika fingerprint belum tersimpan maka --> kirim alarm ke ubidots beserta foto orangnya
                # 5. aksi : menyalakan buzzer/alarm, jika fingerprint cocok membuka solenoid lock
                print("Fingerprint tidak terdaftar")
                lcd_string("   FINGERPRINT   ", LCD_LINE_1)
                lcd_string("  UNRECOGNIZED!  ", LCD_LINE_2)
                photo_path = capture_image()
                trigger_alarm("ON")
                beep()
                send_to_telegram(photo_path)
                
                
            else:
                print("Fingerprint terdaftar, selamat datang tuanku")
                lcd_string("     MATCH!      ", LCD_LINE_1)
                control_solenoid("UNLOCK")
                lcd_string(" HOME SWEET HOME ", LCD_LINE_2)
                time.sleep(5)
                control_solenoid("LOCK")        
            # kirim status pintu apakah terbuka atau terkunci, alarm status
            send_to_ubidots()
            #send_to_telegram()
            time.sleep(3)
            
if __name__ == "__main__":
    main()
    
