# ...existing code...
import time
import machine

class InOut:
    def __init__(self):
        # pinos conforme solicitado (GPIO do Raspberry Pi Pico)
        self.SAIDA_PWM_1 = 12
        self.SAIDA_PWM_2 = 11
        self.SAIDA_PWM_3 = 10
        self.SAIDA_PWM_4 = 7
        self.SAIDA_PWM_5 = 3
        self.SAIDA_PWM_6 = 4
        self.SAIDA_MAQUINA_PRONTA = 6

        self.ENTRADA_ACIONA_MAQUINA = 5

        # PWM / IO inicialização
        self.pwm_period = 1.0  # periodo em segundos (por compatibilidade com código anterior)
        self._pwm_freq = max(1, int(1.0 / self.pwm_period))  # freq em Hz
        self.pwm_pins = {
            self.SAIDA_PWM_1: None,
            self.SAIDA_PWM_2: None,
            self.SAIDA_PWM_3: None,
            self.SAIDA_PWM_4: None,
            self.SAIDA_PWM_5: None,
            self.SAIDA_PWM_6: None,
        }
        self.pwm_duty_cycles = {pin: 0 for pin in self.pwm_pins}

        # configurar pinos como PWM/Pin
        for pin in self.pwm_pins:
            p = machine.Pin(pin, machine.Pin.OUT)
            pwm = machine.PWM(p)
            pwm.freq(self._pwm_freq)
            pwm.duty_u16(0)
            self.pwm_pins[pin] = pwm

        # saída "máquina pronta" (usado como saída ativa baixa no código original)
        self.saida_pronta = machine.Pin(self.SAIDA_MAQUINA_PRONTA, machine.Pin.OUT)
        self.saida_pronta.value(1)  # iniciliza como "desligado" (HIGH)

        # entrada que aciona a máquina (pull-up)
        self.entrada_aciona = machine.Pin(self.ENTRADA_ACIONA_MAQUINA, machine.Pin.IN, machine.Pin.PULL_UP)

    @property
    def get_aciona_maquina(self):
        # retorna 1 quando acionado (entrada em LOW com pull-up)
        return 1 if self.entrada_aciona.value() == 0 else 0

    def set_pwm_period(self, period):
        # atualiza periodo (segundos) e recalcula frequência para PWMs
        if period <= 0:
            return
        self.pwm_period = period
        freq = max(1, int(1.0 / period))
        self._pwm_freq = freq
        for pwm in self.pwm_pins.values():
            try:
                pwm.freq(self._pwm_freq)
            except Exception:
                pass

    def set_pwm_duty_cycle(self, pin, duty_cycle):
        # duty_cycle em 0..100
        if pin in self.pwm_pins:
            duty = max(0, min(100, int(duty_cycle)))
            self.pwm_duty_cycles[pin] = duty
            # duty_u16 aceita 0..65535
            self.pwm_pins[pin].duty_u16(int(duty * 65535 / 100))

    def aciona_pwm(self, duty_cycle, saida):
        mapping = {
            1: self.SAIDA_PWM_1,
            2: self.SAIDA_PWM_2,
            3: self.SAIDA_PWM_3,
            4: self.SAIDA_PWM_4,
            5: self.SAIDA_PWM_5,
            6: self.SAIDA_PWM_6,
        }
        if saida in mapping:
            self.set_pwm_duty_cycle(mapping[saida], duty_cycle)

    def aciona_maquina_pronta(self, status):
        # mantém compatibilidade: status True -> ativa (LOW), False -> desativa (HIGH)
        self.saida_pronta.value(0 if status else 1)

    def cleanup(self):
        # desliga PWMs e reverte pinos
        for pwm in self.pwm_pins.values():
            try:
                pwm.deinit()
            except Exception:
                pass
        try:
            self.saida_pronta.value(1)
        except Exception:
            pass

class IO_MODBUS:
    def __init__(self, dado=None):
        self.dado = dado
        self.fake_modbus = True
        try:
            # UART0: TX=GPIO0, RX=GPIO1
            self.uart = machine.UART(0, baudrate=9600, bits=8, parity=None, stop=1)
            # não é necessário especificar tx/rx se usando UART0 padrão; se preciso:
            # tx_pin = machine.Pin(0)
            # rx_pin = machine.Pin(1)
            # self.uart = machine.UART(0, baudrate=9600, tx=tx_pin, rx=rx_pin)
        except Exception as e:
            print("Erro ao abrir UART0:", e)
            self.uart = None
            return

        self.io_rpi = InOut()

    def crc16_modbus(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if (crc & 0x0001):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def _write_and_read(self, tx_bytes, expected_len, timeout=1.0):
        if not self.uart:
            return b''
        try:
            self.uart.write(bytes(tx_bytes))
            start = time.time()
            while time.time() - start < timeout:
                if self.uart.any() >= expected_len:
                    data = self.uart.read(expected_len)
                    return data if data else b''
                time.sleep(0.01)
        except Exception:
            pass
        return b''

    def _get_adr_PTA(self):
        broadcast = 0xFF
        id_loc = hex(broadcast)[2:].zfill(2).upper()
        hex_text = f"{id_loc}0300020001"
        bytes_hex = bytes.fromhex(hex_text)
        crc_result = self.crc16_modbus(bytes_hex)
        parte_superior = (crc_result >> 8) & 0xFF
        parte_inferior = crc_result & 0xFF

        for i in range(3):
            try:
                tx = [broadcast, 3, 0, 2, 0, 1, parte_inferior, parte_superior]
                resp = self._write_and_read(tx, expected_len=7, timeout=1.0)
                if resp:
                    hex_recv = resp.hex()
                    # valida CRC
                    # reconstrói bytes para calculo CRC (menor parte)
                    bytes_for_crc = bytes.fromhex(hex_recv[0:10])
                    crc_calc = self.crc16_modbus(bytes_for_crc)
                    sup = (crc_calc >> 8) & 0xFF
                    inf = crc_calc & 0xFF
                    superior_crc = int(hex_recv[12:14], 16)
                    inferior_crc = int(hex_recv[10:12], 16)
                    if sup == superior_crc and inf == inferior_crc:
                        dados_recebidos = hex_recv[6:10]
                        return int(dados_recebidos, 16)
                    else:
                        if i > 1:
                            self.reset_serial()
                else:
                    if i > 1:
                        self.reset_serial()
            except Exception as e:
                print("Erro de comunicação:", e)
                return -1
        return -1

    def config_adr_PTA(self, adr):
        adr_target = hex(adr)[2:].zfill(4).upper()
        adr_device = self._get_adr_PTA()
        if adr_device == -1:
            return False
        id_device = hex(adr_device)[2:].zfill(2).upper()
        hex_text = f"{id_device}060002{adr_target}"
        bytes_hex = bytes.fromhex(hex_text)
        crc_result = self.crc16_modbus(bytes_hex)
        parte_superior = (crc_result >> 8) & 0xFF
        parte_inferior = crc_result & 0xFF

        adr_target_int = int(adr_target, 16)
        msb = (adr_target_int >> 8) & 0xFF
        lsb = adr_target_int & 0xFF

        for i in range(3):
            try:
                tx = [adr_device, 6, 0, 2, msb, lsb, parte_inferior, parte_superior]
                resp = self._write_and_read(tx, expected_len=8, timeout=1.0)
                if resp:
                    hex_recv = resp.hex()
                    bytes_for_crc = bytes.fromhex(hex_recv[0:12])
                    crc_calc = self.crc16_modbus(bytes_for_crc)
                    sup = (crc_calc >> 8) & 0xFF
                    inf = crc_calc & 0xFF
                    superior_crc = int(hex_recv[14:16], 16)
                    inferior_crc = int(hex_recv[12:14], 16)
                    if sup == superior_crc and inf == inferior_crc:
                        id_target = int(hex_recv[0:2], 16)
                        id_change = int(hex_recv[8:12], 16)
                        return id_target, id_change
                    else:
                        if i > 1:
                            self.reset_serial()
                else:
                    if i > 1:
                        self.reset_serial()
            except Exception as e:
                print("Erro de comunicação:", e)
                return -1
        return -1

    def get_temperature_channel(self, adr):
        hex_id = hex(adr)[2:].zfill(2).upper()
        hex_text = f"{hex_id}0300000001"
        bytes_hex = bytes.fromhex(hex_text)
        crc_result = self.crc16_modbus(bytes_hex)
        parte_superior = (crc_result >> 8) & 0xFF
        parte_inferior = crc_result & 0xFF

        for i in range(3):
            try:
                tx = [adr, 3, 0, 0, 0, 1, parte_inferior, parte_superior]
                resp = self._write_and_read(tx, expected_len=7, timeout=1.0)
                if resp:
                    hex_recv = resp.hex()
                    bytes_for_crc = bytes.fromhex(hex_recv[0:10])
                    crc_calc = self.crc16_modbus(bytes_for_crc)
                    sup = (crc_calc >> 8) & 0xFF
                    inf = crc_calc & 0xFF
                    superior_crc = int(hex_recv[12:14], 16)
                    inferior_crc = int(hex_recv[10:12], 16)
                    if sup == superior_crc and inf == inferior_crc:
                        value = int(hex_recv[6:10], 16) / 10.0
                        return value
                    else:
                        if i > 1:
                            self.reset_serial()
                else:
                    if i > 1:
                        self.reset_serial()
            except Exception as e:
                print("Erro de comunicação:", e)
                return -1
        return -1

    def reset_serial(self):
        try:
            if self.uart:
                self.uart.deinit()
                time.sleep(0.5)
                self.uart = machine.UART(0, baudrate=9600, bits=8, parity=None, stop=1)
                print("UART0 resetada com sucesso.")
        except Exception as e:
            print("Erro ao resetar UART0:", e)

if __name__ == '__main__':
    # testes simples podem ser feitos interativamente no REPL do Pico
    import time
    io = IO_MODBUS()
    io.config_adr_PTA(3)
    while True:
        print("1. Visualizar endereço do dispositivo")
        print("2. Modificar endereço do dispositivo")
        print("3. Ler temperatura")
        print("4. Sair")
        opcao = input("Escolha uma opção: ")

        if opcao == '1':
            endereco = io._get_adr_PTA()
            if endereco != -1:
                print(f"Endereço do dispositivo: {endereco}")
            else:
                print("Erro ao obter o endereço do dispositivo.")
        elif opcao == '2':
            novo_endereco = int(input("Digite o novo endereço do dispositivo: "))
            resultado = io.config_adr_PTA(novo_endereco)
            if resultado != -1:
                print(f"Endereço do dispositivo alterado para: {novo_endereco}")
            else:
                print("Erro ao alterar o endereço do dispositivo.")
        elif opcao == '3':
            endereco = int(input("Digite o endereço do dispositivo: "))
            temperatura = io.get_temperature_channel(endereco)
            if temperatura != -1:
                print(f"Temperatura do dispositivo: {temperatura}°C")
            else:
                print("Erro ao ler a temperatura do dispositivo.")
        elif opcao == '4':
            break
        else:
            print("Opção inválida. Tente novamente.")