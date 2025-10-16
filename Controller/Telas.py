import time
import subprocess
from  st7920 import ST7920
from RPi import GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def get_gpu_temp():
    result = subprocess.run(['vcgencmd', 'measure_temp'], stdout=subprocess.PIPE)
    output = result.stdout.decode('utf-8').replace("temp=", "").replace("'C\n", "")
    return float(output)
lcd = ST7920()
try:
    while True:
        lcd.clear()
        gpu_temp = get_gpu_temp()
        lcd.put_text(f"GPU Temp: {gpu_temp} C", 0, 0)
        lcd.redraw()
        time.sleep(2)
except KeyboardInterrupt:
    lcd.clear()