# ...existing code...
import time
import machine

class KY040:
    def __init__(self, clk_pin=14, dt_pin=15, sw_pin=4, val_min=1, val_max=10, pull=machine.Pin.PULL_UP):
        # pinos do Raspberry Pi Pico (GPIO numbers)
        self.clk_pin = clk_pin
        self.dt_pin = dt_pin
        self.sw_pin = sw_pin

        self.val_min = val_min
        self.val_max = val_max

        # estado do encoder
        self.counter = 0
        self.test_counter = 0
        self.sw_status = 1

        # debounce timestamps (ms)
        self._last_time_clk = time.ticks_ms()
        self._last_time_sw = time.ticks_ms()
        # mínimo entre eventos (ms)
        self._debounce_clk_ms = 2
        self._debounce_sw_ms = 50

        # configurar pinos
        self.clk = machine.Pin(self.clk_pin, machine.Pin.IN, pull)
        self.dt = machine.Pin(self.dt_pin, machine.Pin.IN, pull)
        self.sw = machine.Pin(self.sw_pin, machine.Pin.IN, pull)

        # estado anterior do clock
        self.last_clk_state = self.clk.value()

        # registrar interrupções para ambas as bordas
        self.clk.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=self._clk_callback)
        self.sw.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=self._sw_callback)

    def _clk_callback(self, pin):
        # debounce simples por tempo
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_time_clk) < self._debounce_clk_ms:
            return
        self._last_time_clk = now

        clk_state = self.clk.value()
        dt_state = self.dt.value()

        if clk_state != self.last_clk_state:
            if dt_state != clk_state:
                # sentido horário (por convenção)
                self.test_counter += 1
                if (self.test_counter & 1) == 0:
                    self.counter += 1
                if self.counter > self.val_max:
                    self.counter = self.val_min
            else:
                # sentido anti-horário
                self.test_counter -= 1
                if (self.test_counter & 1) == 0:
                    self.counter -= 1
                if self.counter < self.val_min:
                    self.counter = self.val_max

            self.last_clk_state = clk_state

    def _sw_callback(self, pin):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_time_sw) < self._debounce_sw_ms:
            return
        self._last_time_sw = now
        self.sw_status = self.sw.value()

    def get_counter(self):
        return self.counter

    @property
    def get_sw_status(self):
        return self.sw_status

    def cleanup(self):
        # remover handlers
        try:
            self.clk.irq(handler=None)
            self.sw.irq(handler=None)
        except Exception:
            pass
        # sem cleanup global no Pico; apenas informação
        print("KY040 cleanup complete")