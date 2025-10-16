import time
from Controller.PID import PIDController
from Controller.IOs import IO_MODBUS, InOut
from Controller.Dados import Dado
from Controller.Lcd import Lcd
from Controller.KY040 import KY040
from Controller.BOTOES import BOTOES
import os
import json

def save_setpoint_to_file(setpoint_list, filename="setpoint_list.json"):
    """
    Salva os setpoints de cada canal em um arquivo JSON.
    """
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)

    # Salva a lista de setpoints no arquivo JSON
    with open(file_path, "w") as file:
        json.dump(setpoint_list, file)

def read_setpoint_from_file(filename="setpoint_list.json"):
    """
    Lê os setpoints de cada canal de um arquivo JSON.
    Se o arquivo não existir, cria um com valores padrão.
    """
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)

    # Valores padrão caso o arquivo não exista
    default_setpoint_list = [50, 50, 50, 50, 50, 50]

    if not os.path.exists(file_path):
        # Se o arquivo não existir, cria um com valores padrão
        with open(file_path, "w") as file:
            json.dump(default_setpoint_list, file)
        return default_setpoint_list

    # Se o arquivo existir, carrega os valores
    with open(file_path, "r") as file:
        setpoint_list = json.load(file)

    return setpoint_list

def save_pid_values(kp_list, ki_list, kd_list, filename="pid_values.json"):
    """
    Salva os valores de Kp, Ki e Kd em um arquivo JSON.
    """
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)

    # Cria um dicionário com os valores
    pid_values = {
        "kp": kp_list,
        "ki": ki_list,
        "kd": kd_list
    }

    # Salva o dicionário no arquivo JSON
    with open(file_path, "w") as file:
        json.dump(pid_values, file)

def load_pid_values(filename="pid_values.json"):
    """
    Carrega os valores de Kp, Ki e Kd de um arquivo JSON.
    Se o arquivo não existir, cria um com valores padrão.
    """
    project_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(project_dir, filename)

    # Valores padrão caso o arquivo não exista
    default_kp = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
    default_ki = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    default_kd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    if not os.path.exists(file_path):
        # Se o arquivo não existir, cria um com valores padrão
        pid_values = {
            "kp": default_kp,
            "ki": default_ki,
            "kd": default_kd
        }
        with open(file_path, "w") as file:
            json.dump(pid_values, file)
        return default_kp, default_ki, default_kd

    # Se o arquivo existir, carrega os valores
    with open(file_path, "r") as file:
        pid_values = json.load(file)

    return pid_values["kp"], pid_values["ki"], pid_values["kd"]

if __name__ == "__main__":
    setpoint_list = read_setpoint_from_file()
    kp_list, ki_list, kd_list = load_pid_values()

    dado = Dado()
    lcd = Lcd()
    io = IO_MODBUS(dado=dado)
    #pot = KY040(io=io, val_min=1, val_max=2)
    pot = BOTOES(io=io, val_min=1, val_max=2) # Usando botões em vez do KY040
    pid = PIDController(setpoint_list=setpoint_list, io_modbus=io, kp_list=kp_list, ki_list=ki_list, kd_list=kd_list, adr=[1, 2, 3, 4, 5, 6])
    pid.start(interval=0.5)

    TELA_CONFIGURACAO_PID = 3
    TELA_CONFIGURACAO_TEMP = 4

    try:
        pot.counter = 1
        pot.val_max = 2
        while True:
            if dado.telas == dado.TELA_INICIAL:
                lcd.lcd_display_string("**** QUALIFIX **** ", 1, 1)
                lcd.lcd_display_string("Iniciar", 2, 1)
                lcd.lcd_display_string("Configuracoes", 3, 1)
                if pot.counter == 1:
                    lcd.lcd_display_string(">", 2, 0)
                    lcd.lcd_display_string(" ", 3, 0)
                elif pot.counter == 2:
                    lcd.lcd_display_string(">", 3, 0)
                    lcd.lcd_display_string(" ", 2, 0)
                if pot.get_sw_status == 0 and (pot.counter == 1 or io.io_rpi.get_aciona_maquina == 1 ):
                    dado.set_telas(dado.TELA_EXECUCAO)
                    pid.set_control_flag(True)
                    lcd.lcd_clear()
                    time.sleep(0.3)
                elif pot.get_sw_status == 0 and pot.counter == 2:
                    pot.counter = 1
                    dado.set_telas(dado.TELA_CONFIGURACAO)
                    lcd.lcd_clear()
                    time.sleep(0.3)

            elif dado.telas == dado.TELA_EXECUCAO:
                lcd.lcd_display_string("Execucao", 1, 1)
                lcd.lcd_display_string(f"1:{pid.value_temp[0]} 4:{pid.value_temp[3]}", 2, 1)
                lcd.lcd_display_string(f"2:{pid.value_temp[1]} 5:{pid.value_temp[4]}", 3, 1)
                lcd.lcd_display_string(f"3:{pid.value_temp[2]} 6:{pid.value_temp[5]}", 4, 1)

                if pot.get_sw_status == 0 or io.io_rpi.get_aciona_maquina == 1:
                    io.io_rpi.aciona_maquina_pronta(False) # Desliga a saida que habilita a prensa
                    dado.set_telas(dado.TELA_INICIAL)
                    pid.set_control_flag(False)
                    lcd.lcd_clear()
                    time.sleep(0.3)

            elif dado.telas == dado.TELA_CONFIGURACAO:
                pot.val_max = 3
                lcd.lcd_display_string("Configurar:", 1, 1)
                lcd.lcd_display_string("Temp", 2, 1)
                lcd.lcd_display_string("PID", 3, 1)
                lcd.lcd_display_string("Sair", 4, 1)
                if pot.get_counter() == 1:
                    lcd.lcd_display_string(">", 2, 0)
                    lcd.lcd_display_string(" ", 3, 0)
                    lcd.lcd_display_string(" ", 4, 0)
                    if pot.get_sw_status == 0:
                        dado.set_telas(TELA_CONFIGURACAO_TEMP)
                        lcd.lcd_clear()
                        time.sleep(0.3)
                elif pot.get_counter() == 2:
                    lcd.lcd_display_string(" ", 2, 0)
                    lcd.lcd_display_string(">", 3, 0)
                    lcd.lcd_display_string(" ", 4, 0)
                    if pot.get_sw_status == 0:
                        dado.set_telas(TELA_CONFIGURACAO_PID)
                        lcd.lcd_clear()
                        time.sleep(0.3)
                elif pot.get_counter() == 3:
                    lcd.lcd_display_string(" ", 2, 0)
                    lcd.lcd_display_string(" ", 3, 0)
                    lcd.lcd_display_string(">", 4, 0)
                    if pot.get_sw_status == 0:
                        dado.set_telas(dado.TELA_INICIAL)
                        lcd.lcd_clear()
                        time.sleep(0.3)

            elif dado.telas == TELA_CONFIGURACAO_TEMP:
                pot.val_max = 6  # Limita a quantidade de canais para ajuste de setpoint
                canal = pot.get_counter()
                lcd.lcd_display_string(f"Canal {canal}", 1, 1)
                lcd.lcd_display_string(f"Temp: {setpoint_list[canal-1]}C", 2, 1)
                lcd.lcd_display_string("Sair: ", 3, 1)

                if pot.get_sw_status == 0:
                    time.sleep(0.6)
                    ajt = 1
                    pot.val_max = 300  # Limita o ajuste de temperatura
                    pot.counter = setpoint_list[canal-1]
                    while ajt == 1:
                        setpoint_list[canal-1] = pot.get_counter()
                        lcd.lcd_display_string(f"Temp: {setpoint_list[canal-1]}C", 2, 1)
                        if pot.get_sw_status == 0:
                            save_setpoint_to_file(setpoint_list)  # Salva os setpoints ajustados
                            pid.setpoint_list = setpoint_list  # Atualiza os setpoints no controlador PID
                            ajt = 0
                            pot.val_max = 6
                            pot.counter = 1
                            dado.set_telas(dado.TELA_CONFIGURACAO)
                            lcd.lcd_clear()
                            time.sleep(0.3)

            elif dado.telas == TELA_CONFIGURACAO_PID:
                pot.val_max = 6  # Limita a quantidade de canais para ajuste de PID
                canal = pot.get_counter()
                lcd.lcd_display_string("Ajuste PID", 1, 1)
                lcd.lcd_display_string(f"Canal {canal}", 2, 1)
                lcd.lcd_display_string("Kp: {:.2f} Ki: {:.2f}".format(kp_list[canal-1], ki_list[canal-1]), 3, 1)
                lcd.lcd_display_string("Kd: {:.2f}".format(kd_list[canal-1]), 4, 1)

                if pot.get_sw_status == 0:
                    time.sleep(0.6)
                    pot.val_max = 400  # Limita o ajuste de PID
                    ajt = 1
                    lcd.lcd_clear()
                    pot.counter = int(kp_list[canal-1] * 10)  # Escala para trabalhar com 0.1
                    while ajt == 1:
                        lcd.lcd_display_string("Ajuste Kp", 1, 1)
                        lcd.lcd_display_string(f"Kp: {kp_list[canal-1]:.2f}", 2, 1)
                        kp_list[canal-1] = pot.get_counter() / 10.0
                        if pot.get_sw_status == 0:
                            time.sleep(0.6)
                            ajt = 2
                            lcd.lcd_clear()
                            pot.counter = int(ki_list[canal-1] * 10)  # Escala para trabalhar com 0.1
                            while ajt == 2:
                                lcd.lcd_display_string("Ajuste Ki", 1, 1)
                                lcd.lcd_display_string(f"Ki: {ki_list[canal-1]:.2f}", 2, 1)
                                ki_list[canal-1] = pot.get_counter() / 10.0
                                if pot.get_sw_status == 0:
                                    time.sleep(0.6)
                                    ajt = 3
                                    lcd.lcd_clear()
                                    pot.counter = int(kd_list[canal-1] * 10)  # Escala para trabalhar com 0.1
                                    while ajt == 3:
                                        lcd.lcd_display_string("Ajuste Kd", 1, 1)
                                        lcd.lcd_display_string(f"Kd: {kd_list[canal-1]:.2f}", 2, 1)
                                        kd_list[canal-1] = pot.get_counter() / 10.0
                                        if pot.get_sw_status == 0:
                                            ajt = 0
                                            pot.val_max = 6  # Limita a quantidade de canais para ajuste de PID
                                            pot.counter = 1
                                            save_pid_values(kp_list, ki_list, kd_list)  # Salva os valores ajustados
                                            pid.kp_list = kp_list
                                            pid.ki_list = ki_list
                                            pid.kd_list = kd_list
                                            dado.set_telas(dado.TELA_CONFIGURACAO)
                                            lcd.lcd_clear()
                                            time.sleep(0.3)

    except KeyboardInterrupt:
        lcd.lcd_clear()
        pot.cleanup()
        io.io_rpi.cleanup()
        pid.set_control_flag(False)
        pid.stop()
        print("Saindo do programa")
        exit()