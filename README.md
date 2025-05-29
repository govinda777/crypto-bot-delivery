# crypto-bot-delivery

Perplexity: https://www.perplexity.ai/collections/crypto-bot-delivery-nYtHnZVqQG.rG7K.a8EZkw
Video: https://youtu.be/iLiI_IRedhI?si=wKqS-CcKLoUeneRk

> # Robô Autônomo Descentralizado de Entrega via Google Maps

## **Visão Geral do Projeto**

Este tutorial ensina como criar um sistema completo de robô autônomo para entregas descentralizadas usando ROS2, Python e tecnologias de criptografia. O sistema permite cotações, pagamentos seguros, navegação autônoma e confirmação de entrega via QR Code.

## **Arquitetura do Sistema**

### **Componentes Principais**
1. **Sistema de Cotação e Pagamento** (Backend Web)
2. **Sistema de Navegação Autônoma** (ROS2)
3. **Sistema de Criptografia** (Blockchain/Smart Contracts)
4. **Interface de Confirmação** (QR Code + Mobile)
5. **Simulador Gazebo** (Testes)

---

## **Tecnologias Necessárias**

### **1. Sistema Operacional e Ambiente**
- **Ubuntu 22.04 LTS** (Recomendado para ROS2 Humble)
- **ROS2 Humble Hawksbill**
- **Python 3.10+**
- **Gazebo Classic 11** ou **Ignition Gazebo**

### **2. Navegação e Mapeamento**
- **Nav2 Navigation Stack**
- **SLAM Toolbox** (Mapeamento)
- **AMCL** (Localização)
- **Google Maps API** (Rotas externas)

### **3. Criptografia e Blockchain**
- **Web3.py** (Interação com blockchain)
- **Cryptography** (Criptografia local)
- **Ethereum/Polygon** (Smart contracts)
- **IPFS** (Armazenamento descentralizado)

### **4. Interface e Comunicação**
- **FastAPI** (Backend REST)
- **QR Code Libraries** (qrcode, pyzbar)
- **OpenCV** (Processamento de imagem)
- **WebSocket** (Comunicação em tempo real)

---

## **Configuração do Ambiente Linux**

### **1. Instalação do ROS2 Humble**
```bash
# Configurar locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Adicionar repositórios ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS2
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

### **2. Instalação do Nav2 e Dependências**
```bash
# Nav2 Navigation Stack
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-localization

# Gazebo e simulação
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3*
```

### **3. Dependências Python**
```bash
# Criar ambiente virtual
python3 -m venv ~/delivery_robot_env
source ~/delivery_robot_env/bin/activate

# Instalar bibliotecas
pip install fastapi uvicorn
pip install web3 cryptography
pip install qrcode[pil] pyzbar
pip install opencv-python
pip install requests googlemaps
pip install websockets
pip install pydantic sqlalchemy
```

---

## **Estrutura do Projeto**

### **1. Criar Workspace ROS2**
```bash
mkdir -p ~/delivery_robot_ws/src
cd ~/delivery_robot_ws

# Configurar ambiente
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/delivery_robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### **2. Estrutura de Diretórios**
```
delivery_robot_ws/
├── src/
│   ├── delivery_navigation/     # Navegação autônoma
│   ├── delivery_crypto/         # Sistema de criptografia
│   ├── delivery_api/           # Backend API
│   ├── delivery_qr/            # Sistema QR Code
│   └── delivery_simulation/    # Simulação Gazebo
├── backend/                    # Servidor web
├── smart_contracts/           # Contratos blockchain
└── mobile_app/               # App confirmação
```

---

## **Implementação Passo a Passo**

### **Passo 1: Sistema de Cotação e Pagamento**

#### **1.1 Criar Pacote de API**
```bash
cd ~/delivery_robot_ws/src
ros2 pkg create delivery_api --build-type ament_python --dependencies rclpy std_msgs geometry_msgs
```

#### **1.2 Backend FastAPI (~/delivery_robot_ws/backend/main.py)**
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from cryptography.fernet import Fernet
import googlemaps
import json
import uuid

app = FastAPI()

# Configuração Google Maps
gmaps = googlemaps.Client(key='SUA_API_KEY_GOOGLE_MAPS')

# Modelo de dados
class DeliveryRequest(BaseModel):
    origin: str
    destination: str
    package_weight: float
    package_size: str

class PaymentData(BaseModel):
    delivery_id: str
    amount: float
    crypto_address: str

# Sistema de criptografia
encryption_key = Fernet.generate_key()
cipher_suite = Fernet(encryption_key)

@app.post("/quote")
async def create_quote(request: DeliveryRequest):
    try:
        # Calcular distância via Google Maps
        directions = gmaps.directions(
            request.origin,
            request.destination,
            mode="driving"
        )
        
        if not directions:
            raise HTTPException(status_code=400, detail="Rota não encontrada")
        
        distance_km = directions[0]['legs'][0]['distance']['value'] / 1000
        duration_min = directions[0]['legs'][0]['duration']['value'] / 60
        
        # Calcular preço baseado em distância e peso
        base_price = 5.0  # Preço base
        price_per_km = 2.0
        weight_multiplier = 1 + (request.package_weight / 10)
        
        total_price = (base_price + (distance_km * price_per_km)) * weight_multiplier
        
        # Criar ID único para entrega
        delivery_id = str(uuid.uuid4())
        
        # Criptografar dados sensíveis
        delivery_data = {
            "id": delivery_id,
            "origin": request.origin,
            "destination": request.destination,
            "route": directions[0],
            "package_info": {
                "weight": request.package_weight,
                "size": request.package_size
            }
        }
        
        encrypted_data = cipher_suite.encrypt(
            json.dumps(delivery_data).encode()
        )
        
        return {
            "delivery_id": delivery_id,
            "price": round(total_price, 2),
            "distance_km": round(distance_km, 2),
            "estimated_duration_min": round(duration_min),
            "encrypted_data": encrypted_data.decode(),
            "payment_address": "0x..." # Endereço crypto para pagamento
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/confirm_payment")
async def confirm_payment(payment: PaymentData):
    # Verificar pagamento na blockchain
    # Aqui você integraria com Web3 para verificar a transação
    
    # Descriptografar dados da entrega
    # Enviar comando para o robô iniciar entrega
    
    return {
        "status": "payment_confirmed",
        "delivery_id": payment.delivery_id,
        "robot_assigned": "ROBOT_001",
        "estimated_pickup": "15 minutes"
    }
```

### **Passo 2: Sistema de Navegação Autônoma**

#### **2.1 Criar Pacote de Navegação**
```bash
cd ~/delivery_robot_ws/src
ros2 pkg create delivery_navigation --build-type ament_python --dependencies rclpy nav2_msgs geometry_msgs sensor_msgs
```

#### **2.2 Controlador Principal (delivery_navigation/navigation_controller.py)**
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
import json
import requests

class DeliveryNavigationController(Node):
    def __init__(self):
        super().__init__('delivery_navigation_controller')
        
        # Action client para navegação
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher para controle manual
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber para dados do laser
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )
        
        # Estado do robô
        self.current_delivery = None
        self.robot_state = "idle"  # idle, navigating, pickup, delivery
        self.obstacle_detected = False
        
        self.get_logger().info('Delivery Navigation Controller iniciado')

    def laser_callback(self, msg):
        """Detectar obstáculos próximos"""
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance  300:
                    self.get_logger().warning('QR Code expirado')
                    return False
                
                # Validar dados
                if self.validate_qr_data(qr_data):
                    self.get_logger().info('QR Code válido - Ação confirmada')
                    return True
                
        except Exception as e:
            self.get_logger().error(f'Erro ao escanear QR Code: {str(e)}')
            
        return False

    def validate_qr_data(self, qr_data):
        """Validar dados do QR Code"""
        required_fields = ['delivery_id', 'action', 'timestamp', 'robot_id']
        return all(field in qr_data for field in required_fields)

def main(args=None):
    rclpy.init(args=args)
    qr_manager = QRCodeManager()
    rclpy.spin(qr_manager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **Passo 4: Simulação Gazebo**

#### **4.1 Criar Mundo de Simulação**
```bash
cd ~/delivery_robot_ws/src
ros2 pkg create delivery_simulation --build-type ament_cmake
mkdir -p delivery_simulation/worlds
mkdir -p delivery_simulation/launch
```

#### **4.2 Arquivo de Mundo (delivery_simulation/worlds/city_delivery.world)**
```xml


  
    
    
    
      model://ground_plane
    
    
    
      model://sun
    
    
    
    
      10 10 0 0 0 0
      true
      
        
          
            
              5 5 3
            
          
        
        
          
            
              5 5 3
            
          
          
            0.5 0.5 0.5 1
          
        
      
    
    
    
    
      15 5 0 0 0 0
      true
      
        
          
            
              0.5
              0.1
            
          
          
            0 1 0 1
          
        
      
    
    
  

```

#### **4.3 Launch File (delivery_simulation/launch/simulation.launch.py)**
```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Caminhos dos pacotes
    gazebo_ros = get_package_share_directory('gazebo_ros')
    delivery_simulation = get_package_share_directory('delivery_simulation')
    
    # Arquivo do mundo
    world_file = os.path.join(delivery_simulation, 'worlds', 'city_delivery.world')
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'delivery_robot', '-topic', 'robot_description'],
        output='screen'
    )
    
    # Navigation controller
    navigation_controller = Node(
        package='delivery_navigation',
        executable='navigation_controller',
        output='screen'
    )
    
    # QR Code manager
    qr_manager = Node(
        package='delivery_qr',
        executable='qr_generator',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_robot,
        navigation_controller,
        qr_manager
    ])
```

---

## Continuous Integration (CI)

This project uses GitHub Actions for Continuous Integration. The CI pipeline is defined in `.github/workflows/ci.yml` and automatically runs on every push to the `main` branch or when a pull request targets `main`.

The pipeline currently performs the following checks for the Python components located in the `crypto-bot-delivery` directory:

1.  **Linting:**
    *   Checks Python code formatting with [Black](https://github.com/psf/black).
    *   Checks for style issues and errors with [Flake8](https://flake8.pycqa.org/en/latest/).
2.  **Unit Testing:**
    *   Runs Python unit tests using [pytest](https://pytest.org).

Placeholders for future enhancements include:
*   Building and testing ROS2 nodes.
*   Performing security scans (SAST, dependency checking, smart contract analysis).

### Running Checks Locally

To ensure your code passes CI checks before pushing, you can run the linters and tests locally.

**1. Install tools (if not already installed):**
   ```bash
   python -m pip install --upgrade pip
   pip install black flake8 pytest
   ```

**2. Navigate to the Python package directory:**
   ```bash
   cd crypto-bot-delivery
   ```

**3. Run Black (to check formatting):**
   ```bash
   black --check .
   ```
   To automatically format the code, run:
   ```bash
   black .
   ```

**4. Run Flake8:**
   ```bash
   flake8 .
   ```

**5. Run pytest:**
   (Assuming your `pyproject.toml` and test structure like `src/delivery_crypto/tests` are set up)
   ```bash
   pytest src/delivery_crypto/tests 
   ```
   Or simply `pytest` if all tests are discoverable from the `crypto-bot-delivery` directory and configured in `pyproject.toml`.

Configuration for Black can be found in `crypto-bot-delivery/pyproject.toml` (`[tool.black]` section).
Configuration for Flake8 can be found in `crypto-bot-delivery/.flake8`.

## **Compilação e Execução**

### **1. Compilar Workspace**
```bash
cd ~/delivery_robot_ws
colcon build
source install/setup.bash
```

### **2. Executar Sistema Completo**

#### **Terminal 1: Backend API**
```bash
cd ~/delivery_robot_ws/backend
source ~/delivery_robot_env/bin/activate
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

#### **Terminal 2: Simulação**
```bash
source ~/delivery_robot_ws/install/setup.bash
ros2 launch delivery_simulation simulation.launch.py
```

#### **Terminal 3: Navegação**
```bash
source ~/delivery_robot_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py
```

---

## **Próximos Passos**

### **1. Integração Blockchain**
- Implementar smart contracts para pagamentos
- Configurar carteira crypto para o robô
- Sistema de escrow para transações

### **2. Melhorias de Navegação**
- Integração com GPS para navegação externa
- Sistema de evitação de obstáculos avançado
- Planejamento de rotas otimizado

### **3. Interface Mobile**
- App para clientes acompanharem entregas
- Sistema de notificações em tempo real
- Interface para confirmação via QR Code

### **4. Deploy em Robô Real**
- Configuração de hardware (sensores, atuadores)
- Calibração de sensores
- Testes em ambiente real

Este tutorial fornece uma base sólida para desenvolver um sistema completo de robô autônomo de entregas. Cada componente pode ser expandido e refinado conforme as necessidades específicas do projeto.

Citations:
[1] https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/61929226/4292452c-f2b1-4026-bdfe-47886c8650a8/paste.txt
[2] https://repositorio.insper.edu.br/bitstreams/2087121b-59c8-4245-99c9-f752ba1e163d/download
[3] https://www.invenzi.com/qr-code-com-criptografia/
[4] https://www.ufrjnautilus.com/post/ros-uma-breve-introdu%C3%A7%C3%A3o?lang=pt
[5] https://embarcados.com.br/distribuicao-linux-ros/
[6] https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html
[7] https://www.clubedolinux.com.br/como-configurar-um-ambiente-de-desenvolvimento-python-no-linux/
[8] https://www.theconstruct.ai/criacao-de-servicos-personalizados-no-ros2-e-controle-de-movimento-do-robo-em-python-portuguese-ros2-tutorial/
[9] https://blog.casadodesenvolvedor.com.br/ros-robot-operating-system/
[10] https://www.pilz.com/pt-BR/trainings/articles/239239
[11] https://www.tiktok.com/@asimov.academy/video/7298844273925999878
[12] https://pt.linkedin.com/pulse/desvendando-o-c%C3%B3digo-secreto-que-%C3%A9-um-algoritmo-e-ele-lima-mendon%C3%A7a-c3i1f
[13] https://www.tiktok.com/@rodrigo.do.carmo/video/7272754696593411334
[14] https://blog.eletrogate.com/introducao-ao-robot-operating-system-ros/
[15] https://docs.ultralytics.com/pt/guides/ros-quickstart/
[16] https://www.youtube.com/watch?v=ovO9CSS1Y4I
[17] https://repositorio.ufpb.br/jspui/bitstream/tede/9286/2/arquivototal.pdf
[18] https://www.youtube.com/watch?v=3WLhUvvDXTQ
[19] https://www.teachy.com.br/atividades/ensino-medio/1ano/robotica/projeto-super-rescue-bot-desenvolvendo-um-robo-de-resgate-simulado
[20] https://www.youtube.com/watch?v=i2Cs7smMrSg
[21] https://www.xbot.com.br/wp-content/uploads/2018/06/42541995e1dfb1c156a2d54dd1a22f9a.pdf
[22] https://www.em.com.br/emfoco/2025/04/02/nova-tecnologia-promete-deixar-seus-qr-codes-mais-seguros/
[23] https://thunderatz.github.io/ROSGazeboGuide/InstalationGuides/ROSGazeboWSL.html
[24] https://www.enacomp.com.br/2013/anais/pdf/44.pdf
[25] https://www.youtube.com/watch?v=fsoi6faumrw
[26] https://www.youtube.com/watch?v=OFeNDllslJs
[27] https://www.youtube.com/watch?v=4U9woXH2J7g
