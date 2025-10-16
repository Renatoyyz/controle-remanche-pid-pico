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
    def __init__(self, dado=None, uart_id=0, baudrate=9600, tx_pin=0, rx_pin=1, timeout=1.0):
        self.dado = dado
        self.timeout = timeout
        self.uart = None
        self.io_rpi = None
        # tenta abrir UART0 (TX=GPIO0, RX=GPIO1 por padrão)
        try:
            # máquina MicroPython no Pico aceita parâmetros tx=machine.Pin(...), rx=machine.Pin(...)
            tx = machine.Pin(tx_pin)
            rx = machine.Pin(rx_pin)
            self.uart = machine.UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1, tx=tx, rx=rx)
        except Exception:
            try:
                # fallback: sem especificar tx/rx (algumas portas não aceitam objetos Pin)
                self.uart = machine.UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1)
            except Exception as e:
                print("Erro ao abrir UART0:", e)
                self.uart = None

        # instancia InOut somente se UART ok (permite testes sem HW)
        try:
            self.io_rpi = InOut()
        except Exception:
            self.io_rpi = None

    def crc16_modbus(self, data):
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def _write_and_read(self, tx_bytes, expected_len, timeout=None):
        if self.uart is None:
            return b''
        timeout = timeout if timeout is not None else self.timeout
        try:
            # limpa buffer de leitura antes de enviar
            try:
                while self.uart.any():
                    self.uart.read()
            except Exception:
                pass
            self.uart.write(bytes(tx_bytes))
            start = time.time()
            while (time.time() - start) < timeout:
                if self.uart.any() >= expected_len:
                    data = self.uart.read(expected_len)
                    return data if data else b''
                time.sleep(0.01)
        except Exception:
            pass
        return b''

    def _get_adr_PTA(self):
        # tenta descobrir endereço do dispositivo via broadcast (3, 0x0002, 1)
        broadcast = 0xFF
        tx_payload = bytes([broadcast, 3, 0, 2, 0, 1])
        crc = self.crc16_modbus(tx_payload)
        crc_lo = crc & 0xFF
        crc_hi = (crc >> 8) & 0xFF
        tx = [broadcast, 3, 0, 2, 0, 1, crc_lo, crc_hi]

        for attempt in range(3):
            resp = self._write_and_read(tx, expected_len=7, timeout=1.0)
            if not resp:
                continue
            try:
                # valida CRC do pacote recebido
                if len(resp) < 7:
                    continue
                resp_bytes = bytes(resp)
                resp_crc_calc = self.crc16_modbus(resp_bytes[0:5])
                resp_crc_lo = resp_bytes[5]
                resp_crc_hi = resp_bytes[6]
                if (resp_crc_calc & 0xFF) == resp_crc_lo and ((resp_crc_calc >> 8) & 0xFF) == resp_crc_hi:
                    # dados retornados nos bytes 3..4 (index 3..4 zero-based?)
                    # conforme uso anterior, pega bytes 3..4 (pos 3..4 -> hex_recv[6:10])
                    try:
                        data_high = resp_bytes[3]
                        data_low = resp_bytes[4]
                        value = (data_high << 8) | data_low
                        return value
                    except Exception:
                        return -1
                else:
                    self.reset_serial()
            except Exception:
                self.reset_serial()
        return -1

    def config_adr_PTA(self, adr):
        # escreve novo endereço adr (16-bit) usando função 6 no dispositivo encontrado
        adr_device = self._get_adr_PTA()
        if adr_device == -1:
            return False
        try:
            id_device = adr_device & 0xFF
            msb = (adr >> 8) & 0xFF
            lsb = adr & 0xFF
            tx_payload = bytes([id_device, 6, 0, 2, msb, lsb])
            crc = self.crc16_modbus(tx_payload)
            tx = [id_device, 6, 0, 2, msb, lsb, crc & 0xFF, (crc >> 8) & 0xFF]
            resp = self._write_and_read(tx, expected_len=8, timeout=1.0)
            if not resp:
                return -1
            if len(resp) < 8:
                return -1
            resp_bytes = bytes(resp)
            crc_calc = self.crc16_modbus(resp_bytes[0:6])
            if (crc_calc & 0xFF) == resp_bytes[6] and ((crc_calc >> 8) & 0xFF) == resp_bytes[7]:
                # retorna (id_target, id_change) semelhante à implementação original
                id_target = resp_bytes[0]
                # bytes 4..5 representam o valor escrito (msb, lsb)
                id_change = (resp_bytes[4] << 8) | resp_bytes[5]
                return id_target, id_change
        except Exception:
            pass
        return -1

    def get_temperature_channel(self, adr):
        # lê registrador 0x0000 (função 3, qty 1) e interpreta resultado /10.0
        try:
            payload = bytes([adr & 0xFF, 3, 0, 0, 0, 1])
            crc = self.crc16_modbus(payload)
            tx = [adr & 0xFF, 3, 0, 0, 0, 1, crc & 0xFF, (crc >> 8) & 0xFF]
            resp = self._write_and_read(tx, expected_len=7, timeout=1.0)
            if not resp or len(resp) < 7:
                return -1
            resp_bytes = bytes(resp)
            # valida CRC
            crc_calc = self.crc16_modbus(resp_bytes[0:5])
            if (crc_calc & 0xFF) != resp_bytes[5] or ((crc_calc >> 8) & 0xFF) != resp_bytes[6]:
                self.reset_serial()
                return -1
            # bytes 3..4 contêm valor
            value = (resp_bytes[3] << 8) | resp_bytes[4]
            return value / 10.0
        except Exception:
            return -1

    def reset_serial(self):
        # tenta reinicializar UART para recuperar comunicação
        try:
            if self.uart:
                try:
                    self.uart.deinit()
                except Exception:
                    pass
                time.sleep(0.3)
                # recria uart com parâmetros iniciais (assume UART0)
                try:
                    self.uart = machine.UART(0, baudrate=9600, bits=8, parity=None, stop=1, tx=machine.Pin(0), rx=machine.Pin(1))
                except Exception:
                    try:
                        self.uart = machine.UART(0, baudrate=9600, bits=8, parity=None, stop=1)
                    except Exception as e:
                        print("Erro ao resetar UART0:", e)
        except Exception as e:
            print("Erro ao resetar serial:", e)

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