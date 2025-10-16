import time
import json

from Controller.PID import PIDController
from Controller.IOs import IO_MODBUS, InOut
from Controller.Dados import Dado
from Controller.Lcd import Lcd
from Controller.KY040 import KY040

def save_setpoint_to_file(setpoint_list, filename="setpoint_list.json"):
    try:
        with open(filename, "w") as f:
            json.dump(setpoint_list, f)
    except Exception:
        pass

def read_setpoint_from_file(filename="setpoint_list.json"):
    default_setpoint_list = [50, 50, 50, 50, 50, 50]
    try:
        with open(filename, "r") as f:
            return json.load(f)
    except Exception:
        try:
            with open(filename, "w") as f:
                json.dump(default_setpoint_list, f)
        except Exception:
            pass
        return default_setpoint_list

def save_pid_values(kp_list, ki_list, kd_list, filename="pid_values.json"):
    pid_values = {"kp": kp_list, "ki": ki_list, "kd": kd_list}
    try:
        with open(filename, "w") as f:
            json.dump(pid_values, f)
    except Exception:
        pass

def load_pid_values(filename="pid_values.json"):
    default_kp = [30.0] * 6
    default_ki = [0.0] * 6
    default_kd = [0.0] * 6
    try:
        with open(filename, "r") as f:
            pid_values = json.load(f)
            return pid_values.get("kp", default_kp), pid_values.get("ki", default_ki), pid_values.get("kd", default_kd)
    except Exception:
        try:
            pid_values = {"kp": default_kp, "ki": default_ki, "kd": default_kd}
            with open(filename, "w") as f:
                json.dump(pid_values, f)
        except Exception:
            pass
        return default_kp, default_ki, default_kd

def fmt_temp(v):
    if v == -1 or v is None:
        return "--"
    try:
        return "{:.1f}".format(float(v))
    except Exception:
        return "Err"

if __name__ == "__main__":
    setpoint_list = read_setpoint_from_file()
    kp_list, ki_list, kd_list = load_pid_values()

    dado = Dado()
    lcd = Lcd()
    io = IO_MODBUS(dado=dado)
    pot = KY040(val_min=1, val_max=2)
    pid = PIDController(setpoint_list=setpoint_list, io_modbus=io,
                        kp_list=kp_list, ki_list=ki_list, kd_list=kd_list,
                        adr=[1,2,3,4,5,6])

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

                remote_start = False
                if io.io_rpi is not None:
                    try:
                        remote_start = (io.io_rpi.get_aciona_maquina == 1)
                    except Exception:
                        remote_start = False

                if pot.get_sw_status == 0 and (pot.counter == 1 or remote_start):
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
                lcd.lcd_display_string(f"1:{fmt_temp(pid.value_temp[0])} 4:{fmt_temp(pid.value_temp[3])}", 2, 1)
                lcd.lcd_display_string(f"2:{fmt_temp(pid.value_temp[1])} 5:{fmt_temp(pid.value_temp[4])}", 3, 1)
                lcd.lcd_display_string(f"3:{fmt_temp(pid.value_temp[2])} 6:{fmt_temp(pid.value_temp[5])}", 4, 1)

                remote_stop = False
                if io.io_rpi is not None:
                    try:
                        remote_stop = (io.io_rpi.get_aciona_maquina == 1)
                    except Exception:
                        remote_stop = False

                if pot.get_sw_status == 0 or remote_stop:
                    if io.io_rpi is not None:
                        try:
                            io.io_rpi.aciona_maquina_pronta(False)
                        except Exception:
                            pass
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
                pot.val_max = 6
                canal = pot.get_counter()
                lcd.lcd_display_string(f"Canal {canal}", 1, 1)
                lcd.lcd_display_string(f"Temp: {setpoint_list[canal-1]}C", 2, 1)
                lcd.lcd_display_string("Sair: ", 3, 1)
                if pot.get_sw_status == 0:
                    time.sleep(0.6)
                    ajt = 1
                    pot.val_max = 300
                    pot.counter = setpoint_list[canal-1]
                    while ajt == 1:
                        setpoint_list[canal-1] = pot.get_counter()
                        lcd.lcd_display_string(f"Temp: {setpoint_list[canal-1]}C", 2, 1)
                        if pot.get_sw_status == 0:
                            save_setpoint_to_file(setpoint_list)
                            pid.setpoint_list = setpoint_list
                            ajt = 0
                            pot.val_max = 6
                            pot.counter = 1
                            dado.set_telas(dado.TELA_CONFIGURACAO)
                            lcd.lcd_clear()
                            time.sleep(0.3)

            elif dado.telas == TELA_CONFIGURACAO_PID:
                pot.val_max = 6
                canal = pot.get_counter()
                lcd.lcd_display_string("Ajuste PID", 1, 1)
                lcd.lcd_display_string(f"Canal {canal}", 2, 1)
                lcd.lcd_display_string("Kp: {:.2f} Ki: {:.2f}".format(kp_list[canal-1], ki_list[canal-1]), 3, 1)
                lcd.lcd_display_string("Kd: {:.2f}".format(kd_list[canal-1]), 4, 1)
                if pot.get_sw_status == 0:
                    time.sleep(0.6)
                    pot.val_max = 400
                    ajt = 1
                    lcd.lcd_clear()
                    pot.counter = int(kp_list[canal-1] * 10)
                    while ajt == 1:
                        lcd.lcd_display_string("Ajuste Kp", 1, 1)
                        lcd.lcd_display_string(f"Kp: {kp_list[canal-1]:.2f}", 2, 1)
                        kp_list[canal-1] = pot.get_counter() / 10.0
                        if pot.get_sw_status == 0:
                            time.sleep(0.6)
                            ajt = 2
                            lcd.lcd_clear()
                            pot.counter = int(ki_list[canal-1] * 10)
                            while ajt == 2:
                                lcd.lcd_display_string("Ajuste Ki", 1, 1)
                                lcd.lcd_display_string(f"Ki: {ki_list[canal-1]:.2f}", 2, 1)
                                ki_list[canal-1] = pot.get_counter() / 10.0
                                if pot.get_sw_status == 0:
                                    time.sleep(0.6)
                                    ajt = 3
                                    lcd.lcd_clear()
                                    pot.counter = int(kd_list[canal-1] * 10)
                                    while ajt == 3:
                                        lcd.lcd_display_string("Ajuste Kd", 1, 1)
                                        lcd.lcd_display_string(f"Kd: {kd_list[canal-1]:.2f}", 2, 1)
                                        kd_list[canal-1] = pot.get_counter() / 10.0
                                        if pot.get_sw_status == 0:
                                            ajt = 0
                                            pot.val_max = 6
                                            pot.counter = 1
                                            save_pid_values(kp_list, ki_list, kd_list)
                                            pid.kp_list = kp_list
                                            pid.ki_list = ki_list
                                            pid.kd_list = kd_list
                                            dado.set_telas(dado.TELA_CONFIGURACAO)
                                            lcd.lcd_clear()
                                            time.sleep(0.3)

            # small yield to avoid CPU hog
            time.sleep(0.05)

    except KeyboardInterrupt:
        lcd.lcd_clear()
        try:
            pot.cleanup()
        except Exception:
            pass
        try:
            if io.io_rpi is not None:
                io.io_rpi.cleanup()
        except Exception:
            pass
        pid.set_control_flag(False)
        pid.stop()