import machine
import time

# Use I2C0 on Raspberry Pi Pico: SDA=GPIO8, SCL=GPIO9
I2CBUS = 0
I2C_SDA_PIN = 8
I2C_SCL_PIN = 9
I2C_FREQ = 400000  # 400kHz

# LCD constants (commands)
LCD_CLEARDISPLAY = 0x01
LCD_RETURNHOME = 0x02
LCD_ENTRYMODESET = 0x04
LCD_DISPLAYCONTROL = 0x08
LCD_CURSORSHIFT = 0x10
LCD_FUNCTIONSET = 0x20
LCD_SETCGRAMADDR = 0x40
LCD_SETDDRAMADDR = 0x80

LCD_ENTRYLEFT = 0x02
LCD_2LINE = 0x08
LCD_5x8DOTS = 0x00

LCD_BACKLIGHT = 0x08
LCD_NOBACKLIGHT = 0x00

class i2c_device:
    def __init__(self, addr, i2c):
        self.addr = addr
        self.i2c = i2c

    def write(self, data):
        try:
            self.i2c.writeto(self.addr, bytes([data]))
        except Exception:
            pass

    def read(self, nbytes=1):
        try:
            return self.i2c.readfrom(self.addr, nbytes)
        except Exception:
            return b''

class Lcd:
    def __init__(self, i2c_addr=None, en=0x04, rs=0x01, bl=0x08, strobe_us=500):
        """
        en, rs, bl: bit masks for Enable, Register Select and Backlight on the PCF8574 expander.
        strobe_us: pulse width in microseconds for EN strobe (increase if needed).
        """
        self.En = en
        self.Rs = rs
        self.BL = bl
        self.strobe_us = int(strobe_us)

        # init I2C bus 0 with specified pins
        try:
            self.i2c = machine.I2C(I2CBUS, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=I2C_FREQ)
        except Exception:
            self.i2c = None
            self.address = None
            self.lcd_device = None
            self.backlight_state = self.BL
            self._available = False
            return

        addrs = []
        try:
            addrs = self.i2c.scan()
        except Exception:
            addrs = []

        if not addrs:
            print("Aviso: nenhum dispositivo I2C encontrado em SDA=GPIO8 SCL=GPIO9. Modo sem-LCD ativado.")
            self.address = None
            self.lcd_device = None
            self.backlight_state = self.BL
            self._available = False
            return

        if i2c_addr is None:
            self.address = addrs[0]
        else:
            self.address = i2c_addr

        self.lcd_device = i2c_device(self.address, self.i2c)
        self.backlight_state = self.BL
        self._available = True

        # initialization sequence for 4-bit mode (PCF8574 backpack)
        time.sleep_ms(50)
        self._write_init_nibble(0x03)
        time.sleep_ms(5)
        self._write_init_nibble(0x03)
        time.sleep_ms(5)
        self._write_init_nibble(0x03)
        time.sleep_ms(1)
        self._write_init_nibble(0x02)

        self.lcd_write(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS)
        self.lcd_write(LCD_DISPLAYCONTROL | 0x04)  # display on
        self.lcd_clear()
        self.lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT)
        time.sleep_ms(200)

    def _raw_write(self, data):
        if not getattr(self, "_available", False):
            return
        self.lcd_device.write(data | self.backlight_state)

    def lcd_strobe(self, data):
        if not getattr(self, "_available", False):
            return
        # EN high
        self._raw_write(data | self.En)
        time.sleep_us(self.strobe_us)
        # EN low
        self._raw_write(data & ~self.En)
        time.sleep_us(max(100, self.strobe_us // 5))

    def _write_init_nibble(self, nibble):
        # write only the high 4 bits (nibble is 0x02..0x03..0x00..)
        b = (nibble << 4) & 0xF0
        self._raw_write(b)
        self.lcd_strobe(b)

    def lcd_write_four_bits(self, data):
        if not getattr(self, "_available", False):
            return
        self._raw_write(data)
        self.lcd_strobe(data)

    def lcd_write(self, cmd, mode=0):
        if not getattr(self, "_available", False):
            return
        high = (cmd & 0xF0) | mode
        low = ((cmd << 4) & 0xF0) | mode
        self.lcd_write_four_bits(high)
        self.lcd_write_four_bits(low)

    def lcd_write_char(self, charvalue, mode=None):
        if not getattr(self, "_available", False):
            return
        mode = self.Rs if mode is None else mode
        high = (charvalue & 0xF0) | mode
        low = ((charvalue << 4) & 0xF0) | mode
        self.lcd_write_four_bits(high)
        self.lcd_write_four_bits(low)

    def lcd_display_string(self, string, line=1, pos=0):
        if not getattr(self, "_available", False):
            return
        if line == 1:
            pos_new = pos
        elif line == 2:
            pos_new = 0x40 + pos
        elif line == 3:
            pos_new = 0x14 + pos
        elif line == 4:
            pos_new = 0x54 + pos
        else:
            pos_new = pos
        self.lcd_write(LCD_SETDDRAMADDR | pos_new)
        for ch in string:
            self.lcd_write_char(ord(ch))

    def lcd_clear(self):
        if not getattr(self, "_available", False):
            return
        self.lcd_write(LCD_CLEARDISPLAY)
        self.lcd_write(LCD_RETURNHOME)
        time.sleep_ms(2)

    def backlight(self, state):
        if state:
            self.backlight_state = self.BL
        else:
            self.backlight_state = LCD_NOBACKLIGHT
        if not getattr(self, "_available", False):
            return
        try:
            self._raw_write(0x00)  # only update backlight bit
        except Exception:
            pass

    def lcd_load_custom_chars(self, fontdata):
        if not getattr(self, "_available", False):
            return
        self.lcd_write(LCD_SETCGRAMADDR)
        for char in fontdata:
            for line in char:
                self.lcd_write_char(line)

    def lcd_display_string_inverter(self, string, line=1, pos=0):
        if not getattr(self, "_available", False):
            return
        prev = self.backlight_state
        self.backlight(0)
        self.lcd_display_string(string, line, pos)
        self.backlight(1)
        self.backlight_state = prev

    # Diagnostic helper: cycles common En/Rs combos and writes a single letter.
    # Use visually: run lcd.try_mappings() and observe which mapping shows text.
    def try_mappings(self):
        if not getattr(self, "_available", False):
            print("LCD not available")
            return
        en_candidates = [0x04, 0x02, 0x01]
        rs_candidates = [0x01, 0x02, 0x04]
        bl_candidates = [0x08, 0x10, 0x01]
        for en in en_candidates:
            for rs in rs_candidates:
                for bl in bl_candidates:
                    print("Testing En=0x%02X Rs=0x%02X BL=0x%02X" % (en, rs, bl))
                    self.En = en; self.Rs = rs; self.BL = bl
                    self.backlight_state = self.BL
                    self.lcd_clear()
                    time.sleep_ms(100)
                    self.lcd_display_string("T", 1, 0)
                    time.sleep_ms(800)
        print("Mapping test done. Observe which combination displayed 'T'.")

if __name__ == "__main__":
    lcd = Lcd()
    print("available:", getattr(lcd, "_available", False), "addr:", lcd.address)
    if getattr(lcd, "_available", False):
        lcd.backlight(1)
        lcd.lcd_clear()
        lcd.lcd_display_string("Renato Oliveira", 1, 0)
        lcd.lcd_display_string_inverter("Teste Pico I2C0", 2, 0)
        time.sleep(4)
        try:
            while True:
                t = time.localtime()
                lcd.lcd_display_string("Data: %02d/%02d/%02d" % (t[2], t[1], t[0] % 100), 3, 0)
                lcd.lcd_display_string("Hora: %02d:%02d:%02d" % (t[3], t[4], t[5]), 4, 0)
                time.sleep(1)
        except KeyboardInterrupt:
            lcd.lcd_clear()
        print("Fim do programa")