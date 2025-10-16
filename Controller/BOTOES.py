import time

class BOTOES:
    def __init__(self, clk_pin=23, dt_pin=22, sw_pin=4, io=None, val_min=1, val_max=10):
        self.clk_pin = clk_pin   # Botão anti-horário (decrementa)
        self.dt_pin = dt_pin     # Botão horário (incrementa)
        self.sw_pin = sw_pin     # Botão de status
        self.io = io
        self.val_min = val_min
        self.val_max = val_max

        self.counter = val_min
        self.sw_status = 1

        self.io.io_rpi.GPIO.setup(self.clk_pin, self.io.io_rpi.GPIO.IN, pull_up_down=self.io.io_rpi.GPIO.PUD_UP)
        self.io.io_rpi.GPIO.setup(self.dt_pin, self.io.io_rpi.GPIO.IN, pull_up_down=self.io.io_rpi.GPIO.PUD_UP)
        self.io.io_rpi.GPIO.setup(self.sw_pin, self.io.io_rpi.GPIO.IN, pull_up_down=self.io.io_rpi.GPIO.PUD_UP)

        self.io.io_rpi.GPIO.add_event_detect(self.clk_pin, self.io.io_rpi.GPIO.FALLING, callback=self._clk_callback, bouncetime=200)
        self.io.io_rpi.GPIO.add_event_detect(self.dt_pin, self.io.io_rpi.GPIO.FALLING, callback=self._dt_callback, bouncetime=200)
        self.io.io_rpi.GPIO.add_event_detect(self.sw_pin, self.io.io_rpi.GPIO.FALLING, callback=self._sw_callback, bouncetime=200)

    def _clk_callback(self, channel):
        # Botão anti-horário (decrementa)
        self.counter -= 1
        if self.counter < self.val_min:
            self.counter = self.val_max

    def _dt_callback(self, channel):
        # Botão horário (incrementa)
        self.counter += 1
        if self.counter > self.val_max:
            self.counter = self.val_min

    def _sw_callback(self, channel):
        # Botão de status
        self.sw_status = self.io.io_rpi.GPIO.input(self.sw_pin)

    def get_counter(self):
        return self.counter

    @property
    def get_sw_status(self):
        return self.sw_status

    def cleanup(self):
        self.io.io_rpi.GPIO.remove_event_detect(self.clk_pin)
        self.io.io_rpi.GPIO.remove_event_detect(self.dt_pin)
        self.io.io_rpi.GPIO.remove_event_detect(self.sw_pin)
        self.io.io_rpi.cleanup()
        print("Exiting...")

# Exemplo de uso
if __name__ == "__main__":
    from IOs import IO_MODBUS
    ios = IO_MODBUS()
    botoes = BOTOES(io=ios)
    try:
        while True:
            print("Counter: ", botoes.get_counter())
            print("SW Status: ", botoes.get_sw_status)
            time.sleep(0.1)
    except KeyboardInterrupt:
        botoes.cleanup()