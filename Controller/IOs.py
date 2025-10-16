import serial
import time
import random
import threading

class FakeRPiGPIO:
    BCM = "BCM"
    PUD_UP = "PUD_UP"
    IN = "IN"
    OUT = "OUT"
    HIGH = 1
    LOW = 0

    def __init__(self):
        self.pins = {}

    def setmode(self, mode):
        self.mode = mode

    def setwarnings(self, state):
        self.warnings = state

    def setup(self, pin, direction, pull_up_down=None):
        self.pins[pin] = {'direction': direction, 'state': self.HIGH if pull_up_down == self.PUD_UP else self.LOW}

    def input(self, pin):
        if pin in self.pins:
            return self.pins[pin]['state']
        raise ValueError(f"Pin {pin} not set up.")

    def output(self, pin, state):
        if pin in self.pins and self.pins[pin]['direction'] == self.OUT:
            self.pins[pin]['state'] = state
        else:
            raise ValueError(f"Pin {pin} not set up or not set as output.")

    def cleanup(self):
        self.pins.clear()


class InOut:
    def __init__(self):
        self.SAIDA_PWM_1 = 20
        self.SAIDA_PWM_2 = 26
        self.SAIDA_PWM_3 = 16
        self.SAIDA_PWM_4 = 19
        self.SAIDA_PWM_5 = 5
        self.SAIDA_PWM_6 = 6
        self.SAIDA_MAQUINA_PRONTA = 13

        self.ENTRADA_ACIONA_MAQUINA = 12
        self.pwm_thread_running = True

        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
        except ImportError:
            print("RPi.GPIO not found. Using fake GPIO.")
            self.GPIO = FakeRPiGPIO()

        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setwarnings(False)

        self.GPIO.setup(self.SAIDA_PWM_1,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_2,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_3,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_4,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_5,  self.GPIO.OUT)
        self.GPIO.setup(self.SAIDA_PWM_6,  self.GPIO.OUT)

        self.GPIO.setup(self.SAIDA_MAQUINA_PRONTA, self.GPIO.OUT)
        self.GPIO.output(self.SAIDA_MAQUINA_PRONTA, self.GPIO.HIGH)  # Inicializa como desligado

        self.GPIO.setup(self.ENTRADA_ACIONA_MAQUINA, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)

        self.pwm_period = 1.0  # Default period in seconds
        self.pwm_duty_cycles = {
            self.SAIDA_PWM_1: 0,
            self.SAIDA_PWM_2: 0,
            self.SAIDA_PWM_3: 0,
            self.SAIDA_PWM_4: 0,
            self.SAIDA_PWM_5: 0,
            self.SAIDA_PWM_6: 0
        }

        self.pwm_threads = {}
        for pin in self.pwm_duty_cycles:
            thread = threading.Thread(target=self._pwm_control, args=(pin,))
            thread.daemon = True
            thread.start()
            self.pwm_threads[pin] = thread

    @property
    def get_aciona_maquina(self):
        if self.GPIO.input(self.ENTRADA_ACIONA_MAQUINA) == self.GPIO.LOW:
            return 1
        else:
            return 0

    def _pwm_control(self, pin):
        while self.pwm_thread_running:
            duty_cycle = self.pwm_duty_cycles[pin]
            on_time = self.pwm_period * (duty_cycle / 100.0)
            off_time = self.pwm_period - on_time
            if on_time > 0:
                self.GPIO.output(pin, self.GPIO.LOW)
                time.sleep(on_time)
            if off_time > 0:
                self.GPIO.output(pin, self.GPIO.HIGH)
                time.sleep(off_time)

    def set_pwm_period(self, period):
        self.pwm_period = period

    def set_pwm_duty_cycle(self, pin, duty_cycle):
        if pin in self.pwm_duty_cycles:
            self.pwm_duty_cycles[pin] = duty_cycle

    def aciona_pwm(self, duty_cycle, saida):
        if saida == 1:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_1, duty_cycle)
        elif saida == 2:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_2, duty_cycle)
        elif saida == 3:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_3, duty_cycle)
        elif saida == 4:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_4, duty_cycle)
        elif saida == 5:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_5, duty_cycle)
        elif saida == 6:
            self.set_pwm_duty_cycle(self.SAIDA_PWM_6, duty_cycle) 

    def cleanup(self):
        self.pwm_thread_running = False
        for pin, thread in self.pwm_threads.items():
            thread.join()
        self.GPIO.cleanup()

    def aciona_maquina_pronta(self, status):
        if status:
            self.GPIO.output(self.SAIDA_MAQUINA_PRONTA, self.GPIO.LOW)
        else:
            self.GPIO.output(self.SAIDA_MAQUINA_PRONTA, self.GPIO.HIGH)

class IO_MODBUS:
    def __init__(self, dado=None):
        self.dado = dado
        self.fake_modbus = True
        try:
            self.ser = serial.Serial(
                                        port='/dev/ttyUSB0',  # Porta serial padrão no Raspberry Pi 4
                                        # port='/dev/tty.URT0',  # Porta serial padrão no Raspberry Pi 4
                                        baudrate=9600,       # Taxa de baud
                                        bytesize=8,
                                        parity="N",
                                        stopbits=1,
                                        timeout=1,            # Timeout de leitura
                                        #xonxoff=False,         # Controle de fluxo por software (XON/XOFF)
                                        #rtscts=True
                                    )
        except Exception as e:
            print(f"Erro ao conectar com a serial: {e}")
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

    def _get_adr_PTA(self):
        broadcast = 0xFF

        id_loc = hex(broadcast)[2:]
        id_loc = id_loc.zfill(2).upper()

        hex_text = f"{id_loc}0300020001"
        bytes_hex = bytes.fromhex(hex_text)  # Transforma em hexa

        crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC16

        parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
        parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

        for i in range(3):
            try:
                # Repete-se os comandos em decimal com os devidos bytes de CRC
                self.ser.write([broadcast,3,0,2,0,1,parte_inferior,parte_superior])
                # self.ser.flush()
                # start_time = time.time()

                while not self.ser.readable():
                    # if time.time() - start_time > self.ser.timeout:
                    #     print("Timeout: Nenhuma resposta do escravo.")
                    #     break
                    time.sleep(0.1)  # Aguarde um curto período antes de verificar novamente

                dados_recebidos = self.ser.read(7)
                self.ser.flushInput()  # Limpa o buffer de entrada após a leitura
                if dados_recebidos != b'':
                    dados_recebidos = dados_recebidos.hex()
                    hex_text = dados_recebidos[0:2]+dados_recebidos[2:4]+dados_recebidos[4:6]+dados_recebidos[6:8]+dados_recebidos[8:10]
                    bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa
                    crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

                    parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
                    parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

                    superior_crc = int(dados_recebidos[12:14],16) # Transforma de hexa para int
                    inferior_crc = int(dados_recebidos[10:12],16) # Transforma de hexa para int

                    if parte_superior == superior_crc and parte_inferior == inferior_crc:
                        dados_recebidos = dados_recebidos[6:10]
                        dados_recebidos = int(dados_recebidos,16)
                        return dados_recebidos
                    else:
                        if i > 1:
                            self.reset_serial()
                else:
                    if i > 1:
                        self.reset_serial()
            except Exception as e:
                print(f"Erro de comunicação: {e}")
                return -1 # Indica erro de alguma natureza....
        return -1

    def config_adr_PTA(self, adr):

        adr_target = hex(adr)[2:]
        adr_target = adr_target.zfill(4).upper()

        adr_device = self._get_adr_PTA()

        if adr_device == -1:
            return False

        id_device = hex(adr_device)[2:]
        id_device = id_device.zfill(2).upper()

        hex_text = f"{id_device}060002{adr_target}"
        bytes_hex = bytes.fromhex(hex_text)  # Transforma em hexa

        crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC16

        parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
        parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

        adr_target_int = int(adr_target, 16)
        msb = (adr_target_int >> 8) & 0xFF
        lsb = adr_target_int & 0xFF

        for i in range(3):
            try:
                # Repete-se os comandos em decimal com os devidos bytes de CRC
                self.ser.write([adr_device,6,0,2,msb,lsb,parte_inferior,parte_superior])
                # self.ser.flush()
                # start_time = time.time()

                while not self.ser.readable():
                    # if time.time() - start_time > self.ser.timeout:
                    #     print("Timeout: Nenhuma resposta do escravo.")
                    #     break
                    time.sleep(0.1)  # Aguarde um curto período antes de verificar novamente

                dados_recebidos = self.ser.read(8)
                self.ser.flushInput()  # Limpa o buffer de entrada após a leitura
                if dados_recebidos != b'':
                    dados_recebidos = dados_recebidos.hex()
                    hex_text = dados_recebidos[0:2]+dados_recebidos[2:4]+dados_recebidos[4:6]+dados_recebidos[6:8]+dados_recebidos[8:10]+dados_recebidos[10:12]
                    bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa
                    crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

                    parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
                    parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

                    superior_crc = int(dados_recebidos[14:16],16) # Transforma de hexa para int
                    inferior_crc = int(dados_recebidos[12:14],16) # Transforma de hexa para int

                    if parte_superior == superior_crc and parte_inferior == inferior_crc:
                        id_target = int(dados_recebidos[0:2], 16)
                        id_change = int(dados_recebidos[8:12],16)
                        return id_target, id_change
                    else:
                        if i > 1:
                            self.reset_serial()
                else:
                    if i > 1:
                        self.reset_serial()
            except Exception as e:
                print(f"Erro de comunicação: {e}")
                return -1 # Indica erro de alguma natureza....
        return -1
        
    # def wp_8026(self, adr, input):
    #     if self.fake_modbus == False:
    #         pass
    #         # return self.wp_8026_(adr=adr, input=input)
    #     else:
    #         # return random.randint(0,1)
    #         return self.dado.passa_condutividade  if input == 8 else self.dado.passa_isolacao
        
    def get_temperature_channel(self, adr):

        id_device = hex(adr)[2:]
        id_device = id_device.zfill(2).upper()

        hex_text = f"{id_device}0300000001"
        bytes_hex = bytes.fromhex(hex_text)  # Transforma em hexa

        crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC16

        parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
        parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

        for i in range(3):
            try:
                # Repete-se os comandos em decimal com os devidos bytes de CRC
                self.ser.write([adr,3,0,0,0,1,parte_inferior,parte_superior])
                # self.ser.flush()
                # start_time = time.time()

                while not self.ser.readable():
                    # if time.time() - start_time > self.ser.timeout:
                    #     print("Timeout: Nenhuma resposta do escravo.")
                    #     break
                    time.sleep(0.1)  # Aguarde um curto período antes de verificar novamente

                dados_recebidos = self.ser.read(7)
                self.ser.flushInput()  # Limpa o buffer de entrada após a leitura
                if dados_recebidos != b'':
                    dados_recebidos = dados_recebidos.hex()
                    hex_text = dados_recebidos[0:2]+dados_recebidos[2:4]+dados_recebidos[4:6]+dados_recebidos[6:8]+dados_recebidos[8:10]
                    bytes_hex = bytes.fromhex(hex_text) # Transforma em hexa
                    crc_result = self.crc16_modbus(bytes_hex) # Retorna o CRC

                    parte_superior = (crc_result >> 8) & 0xFF  # Desloca 8 bits para a direita e aplica a máscara 0xFF
                    parte_inferior = crc_result & 0xFF        # Aplica a máscara 0xFF diretamente

                    superior_crc = int(dados_recebidos[12:14],16) # Transforma de hexa para int
                    inferior_crc = int(dados_recebidos[10:12],16) # Transforma de hexa para int

                    if parte_superior == superior_crc and parte_inferior == inferior_crc:
                        value = int(dados_recebidos[6:10], 16)/10
                        return value
                    else:
                        if i > 1:
                            self.reset_serial()
                else:
                    if i > 1:
                        self.reset_serial()
            except Exception as e:
                print(f"Erro de comunicação: {e}")
                return -1 # Indica erro de alguma natureza....
        return -1

    
    def reset_serial(self):
        try:
            self.ser.close()
            time.sleep(0.5)  # Aguarda um curto período antes de reabrir a porta
            self.ser.open()
            self.ser.flushInput()  # Limpa o buffer de entrada após reabrir a porta
            print("Porta serial resetada com sucesso.")
        except Exception as e:
            print(f"Erro ao resetar a porta serial: {e}")

if __name__ == '__main__':
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