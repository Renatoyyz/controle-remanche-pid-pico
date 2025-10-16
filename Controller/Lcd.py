import machine
import time

# Use I2C0 on Raspberry Pi Pico: SDA=GPIO8, SCL=GPIO9
I2CBUS = 0
I2C_SDA_PIN = 8
I2C_SCL_PIN = 9
I2C_FREQ = 400000  # 400kHz

# LCD constants
LCD_CLEARDISPLAY = 0x01
LCD_RETURNHOME = 0x02
LCD_ENTRYMODESET = 0x04
LCD_DISPLAYCONTROL = 0x08
LCD_CURSORSHIFT = 0x10
LCD_FUNCTIONSET = 0x20
LCD_SETCGRAMADDR = 0x40
LCD_SETDDRAMADDR = 0x80

LCD_ENTRYRIGHT = 0x00
LCD_ENTRYLEFT = 0x02
LCD_ENTRYSHIFTINCREMENT = 0x01
LCD_ENTRYSHIFTDECREMENT = 0x00

LCD_DISPLAYON = 0x04
LCD_DISPLAYOFF = 0x00
LCD_CURSORON = 0x02
LCD_CURSOROFF = 0x00
LCD_BLINKON = 0x01
LCD_BLINKOFF = 0x00

LCD_DISPLAYMOVE = 0x08
LCD_CURSORMOVE = 0x00
LCD_MOVERIGHT = 0x04
LCD_MOVELEFT = 0x00

LCD_8BITMODE = 0x10
LCD_4BITMODE = 0x00
LCD_2LINE = 0x08
LCD_1LINE = 0x00
LCD_5x10DOTS = 0x04
LCD_5x8DOTS = 0x00

LCD_BACKLIGHT = 0x08
LCD_NOBACKLIGHT = 0x00

En = 0b00000100  # Enable bit
Rw = 0b00000010  # Read/Write bit
Rs = 0b00000001  # Register select bit

class i2c_device:
    def __init__(self, addr, i2c):
        self.addr = addr
        self.i2c = i2c

    def write_cmd(self, cmd):
        # write single byte to device
        self.i2c.writeto(self.addr, bytes([cmd]))

    def write_cmd_arg(self, cmd, data):
        # write two bytes (cmd then data)
        self.i2c.writeto(self.addr, bytes([cmd, data]))

    def write_block_data(self, cmd, data):
        # write cmd then block
        self.i2c.writeto(self.addr, bytes([cmd]) + bytes(data))

    def read(self, nbytes=1):
        return self.i2c.readfrom(self.addr, nbytes)

    def read_data(self, cmd, nbytes=1):
        # common I2C expander doesn't support register read; send cmd then read
        try:
            self.i2c.writeto(self.addr, bytes([cmd]))
            return self.i2c.readfrom(self.addr, nbytes)
        except Exception:
            return b''

class Lcd:
    def __init__(self, i2c_addr=None):
        # init I2C bus 0 with specified pins
        self.i2c = machine.I2C(I2CBUS, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN), freq=I2C_FREQ)

        # autodetect address if not provided
        addrs = self.i2c.scan()
        if not addrs:
            raise Exception("I2C bus scan found no devices. Check wiring SDA=GPIO8 SCL=GPIO9.")
        if i2c_addr is None:
            self.address = addrs[0]
        else:
            self.address = i2c_addr
        self.lcd_device = i2c_device(self.address, self.i2c)

        self.backlight_state = LCD_BACKLIGHT

        # initialization sequence for 4-bit mode (PCF8574 I2C backpack)
        time.sleep_ms(50)
        self.lcd_write(0x03)
        time.sleep_ms(5)
        self.lcd_write(0x03)
        time.sleep_ms(5)
        self.lcd_write(0x03)
        time.sleep_ms(1)
        self.lcd_write(0x02)

        self.lcd_write(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE)
        self.lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON)
        self.lcd_write(LCD_CLEARDISPLAY)
        self.lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT)
        time.sleep_ms(200)

    def lcd_strobe(self, data):
        # pulse the enable bit
        self.lcd_device.write_cmd(data | En | self.backlight_state)
        time.sleep_us(500)
        self.lcd_device.write_cmd((data & ~En) | self.backlight_state)
        time.sleep_us(100)

    def lcd_write_four_bits(self, data):
        self.lcd_device.write_cmd(data | self.backlight_state)
        self.lcd_strobe(data)

    def lcd_write(self, cmd, mode=0):
        # send high nibble
        high = mode | (cmd & 0xF0)
        low = mode | ((cmd << 4) & 0xF0)
        self.lcd_write_four_bits(high)
        self.lcd_write_four_bits(low)

    def lcd_write_char(self, charvalue, mode=1):
        high = mode | (charvalue & 0xF0)
        low = mode | ((charvalue << 4) & 0xF0)
        self.lcd_write_four_bits(high)
        self.lcd_write_four_bits(low)

    def lcd_display_string(self, string, line=1, pos=0):
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
            self.lcd_write_char(ord(ch), Rs)

    def lcd_clear(self):
        self.lcd_write(LCD_CLEARDISPLAY)
        self.lcd_write(LCD_RETURNHOME)
        time.sleep_ms(2)

    def backlight(self, state):
        if state:
            self.backlight_state = LCD_BACKLIGHT
        else:
            self.backlight_state = LCD_NOBACKLIGHT
        # update expander with current backlight (no other bits)
        try:
            self.lcd_device.write_cmd(self.backlight_state)
        except Exception:
            pass

    def lcd_load_custom_chars(self, fontdata):
        self.lcd_write(LCD_SETCGRAMADDR)
        for char in fontdata:
            for line in char:
                self.lcd_write_char(line)

    def lcd_display_string_inverter(self, string, line=1, pos=0):
        # simulate inversion by turning off backlight while writing
        prev = self.backlight_state
        self.backlight(0)
        self.lcd_display_string(string, line, pos)
        self.backlight(1)
        self.backlight_state = prev

if __name__ == "__main__":
    lcd = Lcd()
    lcd.lcd_display_string("Renato Oliveira", 1, 1)
    lcd.lcd_display_string_inverter("Teste Pico I2C0", 2, 1)
    time.sleep(4)
    try:
        while True:
            t = time.localtime()
            lcd.lcd_display_string("Data: %02d/%02d/%02d" % (t[2], t[1], t[0] % 100), 3, 1)
            lcd.lcd_display_string("Hora: %02d:%02d:%02d" % (t[3], t[4], t[5]), 4, 1)
            time.sleep(1)
    except KeyboardInterrupt:
        lcd.lcd_clear()
        print("Fim do programa")