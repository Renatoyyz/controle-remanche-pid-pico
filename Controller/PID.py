import time

# cross-platform thread module + lock factory
try:
    import _thread as _thread_mod  # MicroPython (RP2040)
    allocate_lock = _thread_mod.allocate_lock
    THREAD_MODE = 'micropython'
except Exception:
    import threading as _thread_mod  # CPython
    allocate_lock = _thread_mod.Lock
    THREAD_MODE = 'threading'

from Controller.IOs import IO_MODBUS

class PIDController:
    def __init__(self, kp_list=None, ki_list=None, kd_list=None,
                 setpoint_list=None, io_modbus=None, adr=None):
        # defaults
        if kp_list is None:
            kp_list = [1.0] * 6
        if ki_list is None:
            ki_list = [0.5] * 6
        if kd_list is None:
            kd_list = [0.05] * 6
        if setpoint_list is None:
            setpoint_list = [180] * 6
        if adr is None:
            adr = [1, 2, 3, 4, 5, 6]

        self.kp_list = kp_list
        self.ki_list = ki_list
        self.kd_list = kd_list
        self.setpoint_list = setpoint_list

        # patamares
        self.setpoint_stages = [[sp * 0.25, sp * 0.50, sp * 0.75, sp] for sp in setpoint_list]

        self.value_temp = [0] * len(adr)
        self.io_modbus = io_modbus or IO_MODBUS()
        self.adr = adr

        # PID internal states
        self.integral = [0] * len(adr)
        self.previous_error = [0] * len(adr)
        self.current_stage = [0] * len(setpoint_list)
        self.stage_start_time = [None] * len(setpoint_list)

        # thread control
        self._running = False
        self._control_flag = False
        self._thread_alive = False

        # synchronization
        self._lock = allocate_lock()

    def compute(self, current_value, index):
        error = self.setpoint_stages[index][self.current_stage[index]] - current_value

        if abs(error) < 0.5:
            self.integral[index] = 0
        else:
            self.integral[index] += error
            self.integral[index] = max(-100, min(100, self.integral[index]))

        derivative = error - self.previous_error[index]

        output = (
            self.kp_list[index] * error +
            self.ki_list[index] * self.integral[index] +
            self.kd_list[index] * derivative
        )
        self.previous_error[index] = error

        return output

    def control_pwm(self):
        # chama leitura/atoração para cada canal
        for i, adr in enumerate(self.adr):
            try:
                self.value_temp[i] = self.io_modbus.get_temperature_channel(adr)
            except Exception:
                self.value_temp[i] = 0

            # avançar estágio se mantiver dentro da faixa por 60s
            try:
                target = self.setpoint_stages[i][self.current_stage[i]]
                if abs(self.value_temp[i] - target) <= 5:
                    if self.stage_start_time[i] is None:
                        self.stage_start_time[i] = time.time()
                    elif (time.time() - self.stage_start_time[i]) >= 60:
                        self.current_stage[i] = min(self.current_stage[i] + 1, len(self.setpoint_stages[i]) - 1)
                        self.stage_start_time[i] = None
                else:
                    self.stage_start_time[i] = None
            except Exception:
                pass

            pid_output = self.compute(self.value_temp[i], i)
            pwm_value = max(0, min(100, pid_output))
            # aciona pwm no hardware via IOs
            try:
                self.io_modbus.io_rpi.aciona_pwm(duty_cycle=pwm_value, saida=adr)
            except Exception:
                pass

        # verifica se todos alcançaram último patamar (aprox)
        all_ready = True
        for j, stages in enumerate(self.setpoint_stages):
            target = stages[-1]
            if not (target * 0.92 <= self.value_temp[j] <= target * 1.08):
                all_ready = False
                break
        try:
            # compatibilidade: aciona_maquina_pronta(True) -> ativa (LOW) no InOut
            self.io_modbus.io_rpi.aciona_maquina_pronta(all_ready)
        except Exception:
            pass

    def _run(self, interval):
        self._thread_alive = True
        try:
            while self._running:
                # se controle ativo, executa loop PID, senão zera PWMs
                if self._control_flag:
                    with self._lock:
                        self.control_pwm()
                else:
                    # garante PWMs desligadas
                    for adr in self.adr:
                        try:
                            self.io_modbus.io_rpi.aciona_pwm(duty_cycle=0, saida=adr)
                        except Exception:
                            pass
                time.sleep(interval)
        finally:
            self._thread_alive = False

    def start(self, interval=1.0):
        if self._running:
            return
        self._running = True
        # tenta criar thread (MicroPython: _thread.start_new_thread -> pode usar segundo core na porta que suportar)
        if THREAD_MODE == 'micropython':
            try:
                _thread_mod.start_new_thread(self._run, (interval,))
            except Exception:
                # fallback síncrono (bloqueante)
                self._run(interval)
        else:
            # CPython fallback
            t = _thread_mod.Thread(target=self._run, args=(interval,))
            t.daemon = True
            t.start()

    def stop(self, wait_timeout=2.0):
        if not self._running:
            return
        self._running = False
        # aguarda término (MicroPython não tem join)
        start = time.time()
        while self._thread_alive and (time.time() - start) < wait_timeout:
            time.sleep(0.01)

    def set_control_flag(self, flag):
        with self._lock:
            self._control_flag = bool(flag)
            # atualiza temps ao trocar flag
            for i, adr in enumerate(self.adr):
                try:
                    self.value_temp[i] = self.io_modbus.get_temperature_channel(adr)
                except Exception:
                    self.value_temp[i] = 0
            if not flag:
                # reset estados PID
                self.integral = [0] * len(self.adr)
                self.previous_error = [0] * len(self.adr)
                self.stage_start_time = [None] * len(self.adr)
                self.current_stage = [0] * len(self.adr)
            else:
                # define estágio inicial conforme temperatura atual
                for i, temp in enumerate(self.value_temp):
                    stage_found = 0
                    for stage, sp in enumerate(self.setpoint_stages[i]):
                        if temp >= sp:
                            stage_found = stage
                    self.current_stage[i] = stage_found

# Example usage (kept for testing; on Pico run via REPL or main script)
if __name__ == "__main__":
    io_modbus = IO_MODBUS()

    kp_list = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
    ki_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    kd_list = [0.05, 0.06, 0.07, 0.08, 0.09, 0.1]
    setpoint_list = [180, 180, 180, 180, 180, 180]

    pid_controller = PIDController(kp_list=kp_list, ki_list=ki_list, kd_list=kd_list,
                                   setpoint_list=setpoint_list, io_modbus=io_modbus, adr=[1,2,3,4,5,6])

    pid_controller.start(interval=1)
    pid_controller.set_control_flag(True)

    # exemplo de execução por 5 minutos
    time.sleep(300)

    pid_controller.set_control_flag(False)
    pid_controller.stop()
#