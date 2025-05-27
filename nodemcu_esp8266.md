Você está absolutamente correto! Peço desculpas pela confusão. O NodeMCU ESP8266 será o microcontrolador principal do robô, não um módulo auxiliar. Vou recriar o arquivo `nodemcu_esp8266.md` focado em **MicroPython** como linguagem principal para implementar o robô de entrega autônomo e descentralizado.

# NodeMCU ESP8266 - Robô de Entrega Autônomo com MicroPython

## **Visão Geral**
O NodeMCU ESP8266 é o microcontrolador principal do sistema de entrega descentralizado, executando MicroPython para controle de motores, sensores, comunicação Wi-Fi, integração com Telegram Bot e funcionalidades blockchain para pagamentos em criptomoedas.

---

## **1. Preparação do Ambiente MicroPython**

### **1.1 Instalação do Firmware MicroPython**

```bash
# Instalar esptool
pip install esptool

# Apagar firmware atual
esptool.py --port /dev/ttyUSB0 erase_flash

# Baixar firmware MicroPython para ESP8266
# https://micropython.org/download/esp8266/

# Gravar firmware MicroPython
esptool.py --port /dev/ttyUSB0 --baud 460800 write_flash --flash_size=detect 0 esp8266-20240602-v1.23.0.bin
```

### **1.2 Configuração do Ambiente de Desenvolvimento**

```bash
# Instalar Thonny IDE (recomendado para MicroPython)
sudo apt install thonny

# Ou usar rshell para linha de comando
pip install rshell

# Conectar ao NodeMCU
rshell --port /dev/ttyUSB0

# Acessar REPL
repl
```

---

## **2. Estrutura do Projeto MicroPython**

### **2.1 Organização dos Arquivos**

```
delivery_robot/
├── boot.py              # Configuração inicial e WiFi
├── main.py              # Loop principal do robô
├── config.py            # Configurações e credenciais
├── modules/
│   ├── motor_control.py # Controle dos motores
│   ├── sensors.py       # Leitura de sensores
│   ├── telegram_bot.py  # Integração Telegram
│   ├── crypto_wallet.py # Funcionalidades blockchain
│   ├── qr_scanner.py    # Scanner QR Code
│   └── navigation.py    # Sistema de navegação
└── lib/
    ├── urequests.py     # HTTP requests
    └── umqtt/           # MQTT client
```

---

## **3. Código Principal - boot.py**

```python
# boot.py - Configuração inicial do robô
import gc
import network
import time
from machine import Pin, PWM
import config

# Configuração de pinos
LED_STATUS = Pin(2, Pin.OUT)  # LED interno NodeMCU
LED_DELIVERY = Pin(16, Pin.OUT)  # LED indicador entrega
BUZZER = Pin(14, Pin.OUT)

def connect_wifi():
    """Conectar ao WiFi com retry automático"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print('🌐 Conectando ao WiFi...')
        wlan.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
        
        # Aguardar conexão com timeout
        timeout = 0
        while not wlan.isconnected() and timeout  0:
            self.set_left_motor(left_speed, 'forward')
        elif left_speed  0:
            self.set_right_motor(right_speed, 'forward')
        elif right_speed  0:
                # Calcular distância (cm)
                distance = (duration * 0.034) / 2
                return round(distance, 1)
            else:
                return -1  # Erro na leitura
        except:
            return -1
    
    def read_temperature_humidity(self):
        """Ler temperatura e umidade"""
        if not self.dht_sensor:
            return None, None
        
        try:
            self.dht_sensor.measure()
            temp = self.dht_sensor.temperature()
            humidity = self.dht_sensor.humidity()
            return temp, humidity
        except:
            return None, None
    
    def is_emergency_pressed(self):
        """Verificar se botão de emergência foi pressionado"""
        return not self.emergency_button.value()  # Pull-up invertido
    
    def obstacle_detected(self):
        """Verificar se há obstáculo à frente"""
        distance = self.read_distance()
        if distance > 0:
            return distance Robô de Entrega Ativo!\n\n" \
               f"🆔 ID: {config.ROBOT_NAME}\n" \
               f"📍 Status: Operacional\n\n" \
               f"Comandos disponíveis:\n" + \
               "\n".join([f"{cmd} - {desc}" for cmd, desc in config.TELEGRAM_COMMANDS.items()])
    
    def _cmd_status(self, robot_controller):
        """Comando /status"""
        sensor_data = robot_controller.sensors.get_sensor_data()
        
        status = f"📊 STATUS DO ROBÔ\n\n" \
                f"🤖 Nome: {config.ROBOT_NAME}\n" \
                f"📦 Entrega Ativa: {'Sim' if robot_controller.delivery_active else 'Não'}\n" \
                f"🌡️ Temperatura: {sensor_data['temperature']}°C\n" \
                f"💧 Umidade: {sensor_data['humidity']}%\n" \
                f"📏 Distância: {sensor_data['distance']} cm\n" \
                f"🚨 Emergência: {'Ativada' if sensor_data['emergency_button'] else 'Normal'}\n" \
                f"⏰ Uptime: {time.time()} segundos"
        
        return status
    
    def _cmd_start_delivery(self, robot_controller):
        """Comando /delivery"""
        if robot_controller.delivery_active:
            return "⚠️ Já existe uma entrega em andamento!"
        
        robot_controller.start_delivery()
        return f"🚀 Entrega iniciada!\n📦 ID: {robot_controller.current_delivery_id}"
    
    def _cmd_stop_delivery(self, robot_controller):
        """Comando /stop"""
        if not robot_controller.delivery_active:
            return "ℹ️ Nenhuma entrega ativa para parar."
        
        robot_controller.stop_delivery()
        return "⏹️ Entrega parada!"
    
    def _cmd_emergency(self, robot_controller):
        """Comando /emergency"""
        robot_controller.emergency_stop()
        return "🚨 PARADA DE EMERGÊNCIA ATIVADA!"
    
    def _cmd_location(self):
        """Comando /location"""
        # Implementar GPS ou localização por WiFi
        return f"📍 Localização atual:\nRobô: {config.ROBOT_NAME}\nStatus: Operacional"
    
    def _cmd_sensors(self, robot_controller):
        """Comando /sensors"""
        data = robot_controller.sensors.get_sensor_data()
        
        return f"🌡️ DADOS DOS SENSORES\n\n" \
               f"Temperatura: {data['temperature']}°C\n" \
               f"Umidade: {data['humidity']}%\n" \
               f"Distância: {data['distance']} cm\n" \
               f"Obstáculo: {'Detectado' if data['obstacle_detected'] else 'Livre'}\n" \
               f"Emergência: {'Ativada' if data['emergency_button'] else 'Normal'}"
    
    def _cmd_wallet(self):
        """Comando /wallet"""
        return f"💰 STATUS DA CARTEIRA\n\n" \
               f"Endereço: {config.WALLET_ADDRESS[:10]}...\n" \
               f"Status: Conectada\n" \
               f"Rede: Polygon"
    
    def _cmd_deliver_to(self, destination, robot_controller):
        """Comando /deliver_to_[destino]"""
        if robot_controller.delivery_active:
            return "⚠️ Robô ocupado com outra entrega!"
        
        robot_controller.start_delivery_to(destination)
        return f"🎯 Navegando para: {destination}\n📦 ID: {robot_controller.current_delivery_id}"
```

---

## **8. Loop Principal - main.py**

```python
# main.py - Loop principal do robô
import time
import gc
import machine
from modules.motor_control import MotorController
from modules.sensors import SensorManager
from modules.telegram_bot import TelegramBot
import config

class DeliveryRobot:
    def __init__(self):
        # Inicializar componentes
        self.motors = MotorController()
        self.sensors = SensorManager()
        self.telegram = TelegramBot()
        
        # Estado do robô
        self.delivery_active = False
        self.current_delivery_id = None
        self.robot_status = "IDLE"
        self.emergency_stop_active = False
        
        # Timers
        self.last_sensor_read = 0
        self.last_heartbeat = 0
        self.last_telegram_check = 0
        
        # LEDs de status
        self.led_status = machine.Pin(config.PINS['LED_STATUS'], machine.Pin.OUT)
        self.led_delivery = machine.Pin(config.PINS['LED_DELIVERY'], machine.Pin.OUT)
        self.buzzer = machine.Pin(config.PINS['BUZZER'], machine.Pin.OUT)
        
        print('🤖 Robô de entrega inicializado!')
    
    def start_delivery(self):
        """Iniciar nova entrega"""
        if self.delivery_active:
            return False
        
        self.delivery_active = True
        self.current_delivery_id = f"DEL_{int(time.time())}"
        self.robot_status = "DELIVERING"
        self.led_delivery.on()
        
        # Som de confirmação
        self.play_sound('confirmation')
        
        print(f"🚀 Entrega iniciada: {self.current_delivery_id}")
        return True
    
    def start_delivery_to(self, destination):
        """Iniciar entrega para destino específico"""
        if self.start_delivery():
            self.robot_status = f"NAVIGATING_TO_{destination}"
            print(f"🎯 Navegando para: {destination}")
            return True
        return False
    
    def stop_delivery(self):
        """Parar entrega atual"""
        if not self.delivery_active:
            return False
        
        self.delivery_active = False
        self.robot_status = "IDLE"
        self.led_delivery.off()
        self.motors.stop()
        
        print(f"⏹️ Entrega parada: {self.current_delivery_id}")
        self.current_delivery_id = None
        return True
    
    def emergency_stop(self):
        """Parada de emergência"""
        self.emergency_stop_active = True
        self.delivery_active = False
        self.robot_status = "EMERGENCY_STOP"
        self.motors.emergency_stop()
        self.led_delivery.off()
        
        # Som de emergência
        self.play_sound('emergency')
        
        print("🚨 PARADA DE EMERGÊNCIA ATIVADA!")
    
    def play_sound(self, sound_type):
        """Reproduzir sons de feedback"""
        if sound_type == 'confirmation':
            for _ in range(2):
                self.buzzer.on()
                time.sleep(0.1)
                self.buzzer.off()
                time.sleep(0.1)
        elif sound_type == 'emergency':
            for _ in range(5):
                self.buzzer.on()
                time.sleep(0.05)
                self.buzzer.off()
                time.sleep(0.05)
        elif sound_type == 'obstacle':
            for _ in range(3):
                self.buzzer.on()
                time.sleep(0.2)
                self.buzzer.off()
                time.sleep(0.1)
    
    def autonomous_navigation(self):
        """Sistema básico de navegação autônoma"""
        if not self.delivery_active or self.emergency_stop_active:
            return
        
        # Verificar obstáculos
        if self.sensors.obstacle_detected():
            self.motors.stop()
            self.play_sound('obstacle')
            print("⚠️ Obstáculo detectado!")
            
            # Tentar contornar
            self.motors.turn_right(400)
            time.sleep(1)
            
            if not self.sensors.obstacle_detected():
                self.motors.forward(500)
                time.sleep(0.5)
                self.motors.turn_left(400)
                time.sleep(1)
            else:
                # Caminho bloqueado
                self.motors.stop()
                self.telegram.send_message("🚧 Caminho bloqueado! Aguardando...")
                
                if not self.sensors.wait_for_clear_path(30):
                    self.telegram.send_message("❌ Rota bloqueada por muito tempo. Parando entrega.")
                    self.stop_delivery()
        else:
            # Caminho livre - continuar
            self.motors.forward(600)
    
    def check_emergency_conditions(self):
        """Verificar condições de emergência"""
        # Botão de emergência
        if self.sensors.is_emergency_pressed():
            self.emergency_stop()
            self.telegram.send_message("🚨 Botão de emergência pressionado!")
        
        # Temperatura crítica
        temp, _ = self.sensors.read_temperature_humidity()
        if temp and temp > 50:  # Temperatura muito alta
            self.emergency_stop()
            self.telegram.send_message(f"🌡️ Temperatura crítica: {temp}°C")
    
    def update_status_leds(self):
        """Atualizar LEDs de status"""
        # LED de status pisca se operacional
        if not self.emergency_stop_active:
            current_time = time.ticks_ms()
            if current_time % 2000  2:
                    update = self.telegram.get_updates()
                    if update and 'message' in update:
                        message = update['message']
                        if 'text' in message:
                            command = message['text']
                            self.telegram.process_command(command, self)
                    self.last_telegram_check = current_time
                
                # Ler sensores (a cada 5 segundos)
                if current_time - self.last_sensor_read > config.DELIVERY_SETTINGS['SENSOR_READ_INTERVAL']:
                    sensor_data = self.sensors.get_sensor_data()
                    print(f"📊 Sensores: {sensor_data}")
                    self.last_sensor_read = current_time
                
                # Enviar heartbeat (a cada 30 segundos)
                if current_time - self.last_heartbeat > config.DELIVERY_SETTINGS['HEARTBEAT_INTERVAL']:
                    self.telegram.send_message(f"💓 Heartbeat - Status: {self.robot_status}")
                    self.last_heartbeat = current_time
                
                # Navegação autônoma
                if self.delivery_active and not self.emergency_stop_active:
                    self.autonomous_navigation()
                
                # Atualizar LEDs
                self.update_status_leds()
                
                # Limpeza de memória
                if current_time % 60 == 0:  # A cada minuto
                    gc.collect()
                
                time.sleep(0.1)  # Pequena pausa para não sobrecarregar
                
            except KeyboardInterrupt:
                print("\n🛑 Parando robô...")
                self.motors.stop()
                break
            except Exception as e:
                print(f"❌ Erro no loop principal: {e}")
                time.sleep(1)

# Inicializar e executar robô
if __name__ == "__main__":
    robot = DeliveryRobot()
    robot.run()
```

---

## **9. Scripts de Deploy e Monitoramento**

### **9.1 Script de Upload - upload_files.py**

```python
#!/usr/bin/env python3
# upload_files.py - Script para enviar arquivos para NodeMCU

import os
import serial
import time
import sys

def upload_file(port, filename, content):
    """Enviar arquivo para NodeMCU via REPL"""
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        
        # Entrar no modo REPL
        ser.write(b'\r\n')
        ser.write(b'\x03')  # Ctrl+C
        ser.write(b'\x03')  # Ctrl+C
        time.sleep(1)
        
        # Criar arquivo
        cmd = f"f = open('{filename}', 'w')\n"
        ser.write(cmd.encode())
        time.sleep(0.5)
        
        # Escrever conteúdo linha por linha
        for line in content.split('\n'):
            line = line.replace("'", "\\'")  # Escapar aspas
            cmd = f"f.write('{line}\\n')\n"
            ser.write(cmd.encode())
            time.sleep(0.1)
        
        # Fechar arquivo
        ser.write(b"f.close()\n")
        time.sleep(0.5)
        
        ser.close()
        print(f"✅ {filename} enviado com sucesso!")
        return True
        
    except Exception as e:
        print(f"❌ Erro ao enviar {filename}: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("Uso: python upload_files.py /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    # Lista de arquivos para enviar
    files_to_upload = [
        'config.py',
        'boot.py',
        'main.py',
        'modules/motor_control.py',
        'modules/sensors.py',
        'modules/telegram_bot.py'
    ]
    
    print(f"📤 Enviando arquivos para NodeMCU em {port}")
    
    for filepath in files_to_upload:
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                content = f.read()
            
            filename = os.path.basename(filepath)
            if '/' in filepath:
                # Criar diretório se necessário
                dirname = os.path.dirname(filepath)
                # Implementar criação de diretório no NodeMCU
            
            upload_file(port, filename, content)
        else:
            print(f"⚠️ Arquivo não encontrado: {filepath}")
    
    print("🎉 Upload concluído!")

if __name__ == "__main__":
    main()
```

### **9.2 Script de Monitoramento - monitor_robot.py**

```python
#!/usr/bin/env python3
# monitor_robot.py - Monitoramento do robô via serial

import serial
import time
import json
import requests
from datetime import datetime

class RobotMonitor:
    def __init__(self, port, baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        
    def connect(self):
        """Conectar ao robô via serial"""
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            print(f"📡 Conectado ao robô em {self.port}")
            return True
        except Exception as e:
            print(f"❌ Erro de conexão: {e}")
            return False
    
    def monitor(self):
        """Monitorar saída do robô"""
        if not self.connect():
            return
        
        print("🔍 Iniciando monitoramento...")
        
        try:
            while True:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    timestamp = datetime.now().strftime('%H:%M:%S')
                    
                    # Colorir output baseado no conteúdo
                    if '❌' in line or 'erro' in line.lower():
                        print(f"\033[91m[{timestamp}] {line}\033[0m")  # Vermelho
                    elif '✅' in line or 'sucesso' in line.lower():
                        print(f"\033[92m[{timestamp}] {line}\033[0m")  # Verde
                    elif '⚠️' in line or 'warning' in line.lower():
                        print(f"\033[93m[{timestamp}] {line}\033[0m")  # Amarelo
                    elif '🚨' in line or 'emergency' in line.lower():
                        print(f"\033[95m[{timestamp}] {line}\033[0m")  # Magenta
                    else:
                        print(f"[{timestamp}] {line}")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n🛑 Monitoramento interrompido")
        finally:
            if self.ser:
                self.ser.close()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) != 2:
        print("Uso: python monitor_robot.py /dev/ttyUSB0")
        sys.exit(1)
    
    monitor = RobotMonitor(sys.argv[1])
    monitor.monitor()
```

---

## **10. Comandos Úteis**

### **10.1 Comandos de Desenvolvimento**

```bash
# Conectar ao NodeMCU via rshell
rshell --port /dev/ttyUSB0

# Acessar REPL
repl

# Listar arquivos no NodeMCU
ls /pyboard

# Copiar arquivo para NodeMCU
cp config.py /pyboard/

# Executar arquivo
exec(open('main.py').read())

# Verificar memória livre
import gc; gc.mem_free()

# Reset do NodeMCU
import machine; machine.reset()
```

### **10.2 Troubleshooting**

```python
# Teste básico de conectividade WiFi
import network
wlan = network.WLAN(network.STA_IF)
print(wlan.isconnected())
print(wlan.ifconfig())

# Teste dos motores
from modules.motor_control import MotorController
motors = MotorController()
motors.forward(500)
time.sleep(2)
motors.stop()

# Teste dos sensores
from modules.sensors import SensorManager
sensors = SensorManager()
print(sensors.get_sensor_data())

# Teste do Telegram
from modules.telegram_bot import TelegramBot
bot = TelegramBot()
bot.send_message("Teste de conectividade")
```

Este arquivo `nodemcu_esp8266.md` agora está focado em **MicroPython** como linguagem principal, implementando um robô de entrega autônomo e descentralizado completo com controle de motores, sensores, comunicação Telegram e preparação para integração blockchain.

Citations:
[1] https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/61929226/5b52d6c6-723e-4a3c-bd1e-f937389c354e/paste.txt
[2] https://www.youtube.com/watch?v=lynvr7M1F6g
[3] https://www.youtube.com/watch?v=jKnA_lBr3S8
[4] https://github.com/peterhinch/micropython-iot
[5] https://randomnerdtutorials.com/micropython-esp32-esp8266-dc-motor-l298n/
[6] https://www.instructables.com/Wifi-Controlled-Car-Using-MicroPython-on-ESP8266/
[7] https://www.instructables.com/Getting-Started-With-Python-for-ESP8266-ESP32/
[8] https://forums.raspberrypi.com/viewtopic.php?t=253497
[9] https://community.blynk.cc/t/nodemcu-esp8266-robot-with-stand-alone-library/3550
[10] https://forum.arduino.cc/t/working-with-micropython-which-ecosystem-to-choose-esp-8266-or-esp-32/605727
[11] https://github.com/cpopp/RobotArm
[12] https://www.instructables.com/MicroPython-IoT-Rover-Based-on-WeMos-D1-ESP-8266EX/
[13] https://docs.petoi.com/communication-modules/wifi-esp8266/esp8266-+-python-scripts-implement-wireless-crowd-control
[14] https://www.embedded-robotics.com/esp8266-micropython/
[15] https://www.linkedin.com/pulse/getting-started-micropython-nodemcu-esp8266-aditya-raman
[16] https://docs.micropython.org/en/latest/esp8266/tutorial/intro.html
[17] https://stackoverflow.com/questions/55896613/can-i-use-micropython-in-nodemcu-model-name-ch340-lua-wifi
[18] https://mjrobot.org/tutoriais/
[19] https://www.az-delivery.de/en/blogs/azdelivery-blog-fur-arduino-und-raspberry-pi/nachrichten-versenden-mit-esp32-und-esp8266-in-micropython-teil-1-ifttt
[20] https://www.dfrobot.com/blog-680.html
[21] https://novatec.com.br/livros/iot-micropython-nodemcu/
[22] https://www.instructables.com/ESP8266-Wifi-Controlled-Robot/
[23] https://mjrobot.org/2018/06/06/programando-micropython-no-esp8266/
[24] https://www.instructables.com/Connect-an-ESP8266-to-your-RaspberryPi/
[25] https://randomnerdtutorials.com/esp32-esp8266-raspberry-pi-lamp-server/
[26] https://www.az-delivery.uk/nl/blogs/azdelivery-blog-fur-arduino-und-raspberry-pi/mqtt-auf-dem-raspberry-pi-und-mit-micropython-auf-der-esp-familie-teil-3
[27] https://www.instructables.com/Micropython-on-ESP-Using-Jupyter/
[28] https://randomnerdtutorials.com/micropython-wi-fi-manager-esp32-esp8266/
