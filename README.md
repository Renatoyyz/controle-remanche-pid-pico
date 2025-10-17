# Controle de remanche (Raspberry Pi Pico / MicroPython)

Resumo
- Projeto para controle de remanche (aquecimento) usando Raspberry Pi Pico rodando MicroPython.
- Interface por encoder KY040 e display I2C (backpack PCF8574). Leitura de sensores por Modbus RTU via UART (opcional).
- Estrutura:
  - main.py — loop principal / UI
  - Controller/ — drivers (Lcd, IOs, PID, KY040, Dados, ...)

Requisitos de software
- Raspberry Pi Pico (RP2040) com MicroPython (UF2) gravado.
- Ferramenta para transferir arquivos: Thonny (recomendado) ou mpremote / ampy / rshell.
- Editor (opcional): VS Code, Thonny editor, etc.

Instalar MicroPython no Pico (resumo)
1. Baixe UF2 do MicroPython para RP2040 (faça o download no site oficial do MicroPython).
2. Segure o botão BOOTSEL do Pico e conecte via USB — ele aparece como unidade USB.
3. Copie o arquivo UF2 para a unidade do Pico. Pico reinicia com MicroPython.

Transferir o projeto ao Pico
- Usando Thonny:
  - Abra Thonny > selecione "MicroPython (Raspberry Pi Pico)" como intérprete.
  - Arraste/cole a árvore do projeto (main.py e a pasta Controller) para o dispositivo e salve.
- Usando mpremote (terminal macOS):
  - Copiar arquivos:
    - mpremote fs put main.py /main.py
    - mpremote fs put -r Controller /Controller
  - Executar script:
    - mpremote run /main.py
- Alternativa: monte o Pico como massa e copie os arquivos (modo UF2/bootloader não persiste scripts automaticamente; usar Thonny/mpremote recomendado).

Hardware — conexões recomendadas
- I2C LCD (PCF8574 backpack)
  - SDA -> GPIO8 (I2C0 SDA)
  - SCL -> GPIO9 (I2C0 SCL)
  - VCC -> 3.3V
  - GND -> GND (comum)
  - Endereço típico: 0x27 (decimal 39) ou 0x3F. Use i2c.scan() para confirmar.
  - Ajuste contraste com o potenciômetro do módulo; use 3.3V, não 5V.

- Encoder KY040 (exemplo de mapeamento usado no projeto)
  - CLK -> GPIO14
  - DT  -> GPIO15
  - SW  -> GPIO4
  - VCC -> 3.3V
  - GND -> GND (comum)
  - O código usa pull-ups internos; se houver bounce, aumentar debounce no Controller/KY040.py.

- Saídas PWM (saídas do InOut, correspondem ao aquecimento)
  - SAIDA_PWM_1: GPIO12
  - SAIDA_PWM_2: GPIO11
  - SAIDA_PWM_3: GPIO10
  - SAIDA_PWM_4: GPIO7
  - SAIDA_PWM_5: GPIO3
  - SAIDA_PWM_6: GPIO4
  - (Consulte Controller/IOs.py para confirmar ou alterar)

- UART Modbus (opcional)
  - TX/RX: UART0 por padrão (GPIO0 TX / GPIO1 RX). Confirme fiação e níveis (RS485 adaptador se necessário).

Rodando o sistema
1. Certifique-se que todos os arquivos do projeto estejam no Pico.
2. Conecte HW conforme acima e ligue o Pico.
3. Abra REPL (Thonny ou mpremote) e execute main.py ou reinicie o Pico (main.py será executado se salvo como main.py).
4. Testes rápidos:
   - No REPL:
     - import machine
     - i2c = machine.I2C(0, scl=machine.Pin(9), sda=machine.Pin(8), freq=400000)
     - print(i2c.scan())  # mostra endereços I2C
   - I2C sem resposta -> verifique VCC/GND/pull-ups/contraste.

Diagnóstico e dicas rápidas
- LCD não mostra texto, mas backlight funciona:
  - Ajuste contraste no pot do módulo.
  - Confirme endereço com i2c.scan().
  - Use a função Lcd.try_mappings() (se presente) para testar mapeamentos de RS/EN/backlight.
  - Aumente strobe_us / delays se necessário.

- Encoder com bounce:
  - Aumente debounce no Driver KY040.py (ms).
  - Use detecção de borda (press) em vez de ler SW constantemente.

- Leituras Modbus ausentes:
  - get_temperature_channel retorna -1 em erro no código atual. O PID ignora -1 e encerra PWM para segurança.
  - Verifique UART/RS485 e CRC das mensagens.

Boas práticas
- Teste cada bloco HW isoladamente (I2C, encoder, UART).
- Mantenha backup dos arquivos originais antes de editar.
- Se desenvolver no PC, use Thonny para transferir e testar rapidamente.

Arquivos importantes
- main.py — loop principal e UI
- Controller/Lcd.py — driver do display I2C (configurável EN/RS/BL)
- Controller/KY040.py — driver do encoder
- Controller/IOs.py — mapeamento de PWM / I/O
- Controller/PID.py — laço PID e rotina de controle
- Controller/Dados.py — constantes/estados da interface

Se precisar, forneça:
- Saída de i2c.scan()
- Mapeamento físico do encoder
- Logs do REPL (erros) — eu sugiro ajustes mínimos no código e passo a passo