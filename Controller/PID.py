import threading
import time
from Controller.IOs import IO_MODBUS

class PIDController:
    def __init__(self, kp_list=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], ki_list=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5], kd_list=[0.05, 0.05, 0.05, 0.05, 0.05, 0.05], setpoint_list=[180, 180, 180, 180, 180, 180], io_modbus=None, adr=[1, 2, 3, 4, 5, 6]):
        self.kp_list = kp_list
        self.ki_list = ki_list
        self.kd_list = kd_list
        self.setpoint_list = setpoint_list

        # Divide cada setpoint em 4 patamares (25%, 50%, 75%, 100%)
        self.setpoint_stages = [[sp * 0.25, sp * 0.50, sp * 0.75, sp] for sp in setpoint_list]
        # self.setpoint_stages = [[sp , sp , sp , sp] for sp in setpoint_list]

        self.value_temp = [0, 0, 0, 0, 0, 0]

        self.io_modbus = io_modbus
        self.adr = adr
        self.integral = [0] * len(adr)
        self.previous_error = [0] * len(adr)
        self._running = False
        self._control_flag = False
        self._thread = None
        self.current_stage = [0] * len(setpoint_list)  # Patamar atual para cada setpoint
        self.stage_start_time = [None] * len(setpoint_list)  # Tempo de início do patamar para cada setpoint

    # def compute(self, current_value, index):
    #     # Controle baseado no patamar atual
    #     error = self.setpoint_stages[index][self.current_stage[index]] - current_value
    #     self.integral[index] += error
    #     derivative = error - self.previous_error[index]

    #     output = self.kp_list[index] * error + self.ki_list[index] * self.integral[index] + self.kd_list[index] * derivative
    #     self.previous_error[index] = error

    #     return output

    def compute(self, current_value, index):
        error = self.setpoint_stages[index][self.current_stage[index]] - current_value

        # Zera a integral se o erro for muito pequeno (ex: menor que 0.5)
        if abs(error) < 0.5:
            self.integral[index] = 0
        else:
            self.integral[index] += error
            # Limita a integral para evitar windup
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
        if self._control_flag:
            for i, adr in enumerate(self.adr):
                self.value_temp[i] = self.io_modbus.get_temperature_channel(adr)
                
                # Verifica se a temperatura atingiu o patamar atual
                if abs(self.value_temp[i] - self.setpoint_stages[i][self.current_stage[i]]) <= 5:  # Tolerância de 5°C
                    if self.stage_start_time[i] is None:
                        self.stage_start_time[i] = time.time()

                    elapsed_time = time.time() - self.stage_start_time[i]
                    if elapsed_time >= 60:  # 1 minuto por patamar
                        self.current_stage[i] += 1
                        if self.current_stage[i] >= len(self.setpoint_stages[i]):  # Se todos os patamares forem concluídos
                            self.current_stage[i] = len(self.setpoint_stages[i]) - 1  # Fixa no último patamar
                        self.stage_start_time[i] = None  # Reinicia o tempo para o próximo patamar

                pid_output = self.compute(self.value_temp[i], i)
                pwm_value = max(0, min(100, pid_output))  # Ensure PWM value is between 0 and 100
                self.io_modbus.io_rpi.aciona_pwm(duty_cycle=pwm_value, saida=adr)

                # Verifica se todos os canais atingiram o último patamar dentro da faixa permitida
                all_channels_ready = True
                for j, setpoint_stage in enumerate(self.setpoint_stages):
                    if not (setpoint_stage[-1] * 0.92 <= self.value_temp[j] <= setpoint_stage[-1] * 1.08):  # Faixa de 92% a 108% do setpoint final
                        all_channels_ready = False
                        break

                # Aciona a saída de máquina pronta se todos os canais estiverem prontos
                if all_channels_ready:
                    self.io_modbus.io_rpi.aciona_maquina_pronta(False)
                else:
                    self.io_modbus.io_rpi.aciona_maquina_pronta(True)
        else:
            pwm_value = 0  # Set PWM to 0 when control flag is False
            for adr in self.adr:
                self.io_modbus.io_rpi.aciona_pwm(duty_cycle=pwm_value, saida=adr)

    def start(self, interval=1):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._run, args=(interval,))
            self._thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._thread.join()

    def _run(self, interval):
        while self._running:
            self.control_pwm()
            time.sleep(interval)

    def set_control_flag(self, flag):
        self._control_flag = flag

        # Atualiza as temperaturas atuais antes de verificar o flag
        for i, adr in enumerate(self.adr):
            self.value_temp[i] = self.io_modbus.get_temperature_channel(adr)

        if not flag:
            # Resetar estados internos ao desativar o controle
            self.integral = [0] * len(self.adr)
            self.previous_error = [0] * len(self.adr)
            self.stage_start_time = [None] * len(self.adr)  # Reinicia o tempo dos patamares
            self.current_stage = [0] * len(self.adr)  # Reinicia os patamares
        else:
            for i, temp in enumerate(self.value_temp):
                # Procura o maior patamar cujo setpoint seja menor ou igual à temperatura atual
                stage_found = 0
                for stage, setpoint in enumerate(self.setpoint_stages[i]):
                    if temp >= setpoint:
                        stage_found = stage
                self.current_stage[i] = stage_found

# Example usage
if __name__ == "__main__":
    io_modbus = IO_MODBUS()

    kp_list = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
    ki_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    kd_list = [0.05, 0.06, 0.07, 0.08, 0.09, 0.1]
    setpoint_list = [180, 180, 180, 180, 180, 180]

    pid_controller = PIDController(kp_list=kp_list, ki_list=ki_list, kd_list=kd_list, setpoint_list=setpoint_list, io_modbus=io_modbus, adr=[1, 2, 3, 4, 5, 6])

    # Start control loop in a separate thread
    pid_controller.start(interval=1)

    # Activate control
    pid_controller.set_control_flag(True)

    # Run for the duration of the control process
    time.sleep(300)  # Example: wait for 5 minutes

    # Deactivate control
    pid_controller.set_control_flag(False)

    # Stop the control loop
    pid_controller.stop()