# **Criando o Serviço de Entrega Personalizado**

Primeiro, vamos criar um serviço ROS 2 específico para entregas. Crie o arquivo `DeliveryService.srv`:

```
# Requisição
string pickup_location
string delivery_location
string package_id
float64 package_weight
string customer_wallet_address
---
# Resposta
bool delivery_accepted
string estimated_time
float64 delivery_cost
string transaction_hash
```

### **Implementando o Servidor de Entrega**

```python
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import DeliveryService
from std_msgs.msg import String
import hashlib
import time

class DeliveryRobotServer(Node):
    def __init__(self):
        super().__init__('delivery_robot_server')
        self.srv = self.create_service(
            DeliveryService, 
            'request_delivery', 
            self.handle_delivery_request
        )
        
        # Publisher para status da entrega
        self.status_publisher = self.create_publisher(
            String, 
            'delivery_status', 
            10
        )
        
        self.get_logger().info('Robô de entrega pronto para receber pedidos')

    def handle_delivery_request(self, request, response):
        # Validar a solicitação
        if self.validate_delivery_request(request):
            # Calcular custo baseado na distância e peso
            cost = self.calculate_delivery_cost(
                request.pickup_location, 
                request.delivery_location, 
                request.package_weight
            )
            
            # Criar hash da transação para blockchain
            transaction_data = f"{request.package_id}_{request.customer_wallet_address}_{time.time()}"
            transaction_hash = hashlib.sha256(transaction_data.encode()).hexdigest()
            
            response.delivery_accepted = True
            response.estimated_time = "30 minutos"
            response.delivery_cost = cost
            response.transaction_hash = transaction_hash
            
            # Publicar status inicial
            status_msg = String()
            status_msg.data = f"Entrega aceita - Pacote: {request.package_id}"
            self.status_publisher.publish(status_msg)
            
            self.get_logger().info(f'Entrega aceita: {request.package_id}')
        else:
            response.delivery_accepted = False
            response.estimated_time = "N/A"
            response.delivery_cost = 0.0
            response.transaction_hash = ""
            
        return response
    
    def validate_delivery_request(self, request):
        # Validações básicas
        return (len(request.pickup_location) > 0 and 
                len(request.delivery_location) > 0 and
                request.package_weight > 0)
    
    def calculate_delivery_cost(self, pickup, delivery, weight):
        # Cálculo simplificado baseado em peso e distância estimada
        base_cost = 5.0
        weight_cost = weight * 0.5
        return base_cost + weight_cost
```

## **Exemplo 2: Integração com Blockchain**

### **Smart Contract para Entregas (Solidity)**

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract DeliveryContract {
    struct Delivery {
        string packageId;
        address customer;
        address robot;
        string pickupLocation;
        string deliveryLocation;
        uint256 cost;
        uint256 timestamp;
        DeliveryStatus status;
        bool isPaid;
    }
    
    enum DeliveryStatus {
        Requested,
        Accepted,
        PickedUp,
        InTransit,
        Delivered,
        Cancelled
    }
    
    mapping(string => Delivery) public deliveries;
    mapping(address => uint256) public robotBalances;
    
    event DeliveryRequested(string packageId, address customer);
    event DeliveryStatusUpdated(string packageId, DeliveryStatus status);
    event PaymentProcessed(string packageId, uint256 amount);
    
    function requestDelivery(
        string memory _packageId,
        string memory _pickupLocation,
        string memory _deliveryLocation,
        uint256 _cost
    ) public payable {
        require(msg.value >= _cost, "Pagamento insuficiente");
        require(bytes(deliveries[_packageId].packageId).length == 0, "Entrega ja existe");
        
        deliveries[_packageId] = Delivery({
            packageId: _packageId,
            customer: msg.sender,
            robot: address(0),
            pickupLocation: _pickupLocation,
            deliveryLocation: _deliveryLocation,
            cost: _cost,
            timestamp: block.timestamp,
            status: DeliveryStatus.Requested,
            isPaid: true
        });
        
        emit DeliveryRequested(_packageId, msg.sender);
    }
    
    function acceptDelivery(string memory _packageId, address _robotAddress) public {
        require(deliveries[_packageId].status == DeliveryStatus.Requested, "Entrega nao disponivel");
        
        deliveries[_packageId].robot = _robotAddress;
        deliveries[_packageId].status = DeliveryStatus.Accepted;
        
        emit DeliveryStatusUpdated(_packageId, DeliveryStatus.Accepted);
    }
    
    function updateDeliveryStatus(string memory _packageId, DeliveryStatus _status) public {
        require(deliveries[_packageId].robot == msg.sender, "Apenas o robo pode atualizar");
        
        deliveries[_packageId].status = _status;
        
        if (_status == DeliveryStatus.Delivered) {
            // Transferir pagamento para o robô
            robotBalances[msg.sender] += deliveries[_packageId].cost;
            emit PaymentProcessed(_packageId, deliveries[_packageId].cost);
        }
        
        emit DeliveryStatusUpdated(_packageId, _status);
    }
}
```

### **Cliente ROS 2 com Integração Blockchain**

```python
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import DeliveryService
from web3 import Web3
import json

class BlockchainDeliveryClient(Node):
    def __init__(self):
        super().__init__('blockchain_delivery_client')
        
        # Cliente ROS 2
        self.client = self.create_client(DeliveryService, 'request_delivery')
        
        # Conexão com blockchain
        self.w3 = Web3(Web3.HTTPProvider('http://localhost:8545'))
        self.contract_address = "0x..." # Endereço do contrato
        self.contract_abi = self.load_contract_abi()
        self.contract = self.w3.eth.contract(
            address=self.contract_address,
            abi=self.contract_abi
        )
        
        self.wallet_address = "0x..." # Endereço da carteira
        self.private_key = "..." # Chave privada
        
    def request_delivery_with_blockchain(self, package_id, pickup, delivery, weight):
        # 1. Primeiro, solicitar entrega via ROS 2
        request = DeliveryService.Request()
        request.package_id = package_id
        request.pickup_location = pickup
        request.delivery_location = delivery
        request.package_weight = weight
        request.customer_wallet_address = self.wallet_address
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().delivery_accepted:
            # 2. Se aceita, registrar na blockchain
            cost_wei = self.w3.to_wei(future.result().delivery_cost, 'ether')
            
            try:
                # Criar transação na blockchain
                transaction = self.contract.functions.requestDelivery(
                    package_id,
                    pickup,
                    delivery,
                    cost_wei
                ).build_transaction({
                    'from': self.wallet_address,
                    'value': cost_wei,
                    'gas': 200000,
                    'gasPrice': self.w3.to_wei('20', 'gwei'),
                    'nonce': self.w3.eth.get_transaction_count(self.wallet_address)
                })
                
                # Assinar e enviar transação
                signed_txn = self.w3.eth.account.sign_transaction(
                    transaction, 
                    private_key=self.private_key
                )
                tx_hash = self.w3.eth.send_raw_transaction(signed_txn.rawTransaction)
                
                self.get_logger().info(f'Entrega registrada na blockchain: {tx_hash.hex()}')
                return True
                
            except Exception as e:
                self.get_logger().error(f'Erro na blockchain: {str(e)}')
                return False
        else:
            self.get_logger().warning('Entrega rejeitada pelo robô')
            return False
    
    def load_contract_abi(self):
        # Carregar ABI do contrato
        with open('delivery_contract_abi.json', 'r') as f:
            return json.load(f)
```

## **Exemplo 3: Sistema Completo de Monitoramento**

### **Nó de Monitoramento de Entregas**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json

class DeliveryMonitor(Node):
    def __init__(self):
        super().__init__('delivery_monitor')
        
        # Subscribers
        self.status_subscriber = self.create_subscription(
            String,
            'delivery_status',
            self.status_callback,
            10
        )
        
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            'robot_position',
            self.position_callback,
            10
        )
        
        # Publisher para atualizações blockchain
        self.blockchain_publisher = self.create_publisher(
            String,
            'blockchain_updates',
            10
        )
        
        self.current_deliveries = {}
        
    def status_callback(self, msg):
        # Processar atualizações de status
        status_data = json.loads(msg.data)
        package_id = status_data.get('package_id')
        status = status_data.get('status')
        
        self.current_deliveries[package_id] = status
        
        # Enviar atualização para blockchain
        blockchain_msg = String()
        blockchain_msg.data = json.dumps({
            'package_id': package_id,
            'status': status,
            'timestamp': self.get_clock().now().to_msg()
        })
        self.blockchain_publisher.publish(blockchain_msg)
        
        self.get_logger().info(f'Status atualizado: {package_id} -> {status}')
    
    def position_callback(self, msg):
        # Monitorar posição do robô para segurança
        position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        
        # Verificar se está na rota esperada
        self.validate_route(position)
    
    def validate_route(self, position):
        # Implementar validação de rota
        # Se o robô sair da rota, alertar na blockchain
        pass
```

## **Exemplo 4: Launch File Completo**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Nó do servidor de entrega
        Node(
            package='delivery_robot_package',
            executable='delivery_server',
            name='delivery_robot_server',
            parameters=[{'robot_id': 'robot_001'}]
        ),
        
        # Nó de monitoramento
        Node(
            package='delivery_robot_package',
            executable='delivery_monitor',
            name='delivery_monitor'
        ),
        
        # Nó de navegação
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='navigation'
        ),
        
        # Iniciar nó blockchain (simulado)
        ExecuteProcess(
            cmd=['python3', 'blockchain_interface.py'],
            cwd='src/delivery_robot_package/scripts'
        )
    ])
```

## **Como Funciona o Sistema Integrado**

**Fluxo de Entrega Completo:**

1. **Cliente solicita entrega** via aplicativo que se comunica com ROS 2
2. **Robô avalia** a solicitação e calcula custo
3. **Smart contract** registra a entrega na blockchain com pagamento
4. **Robô aceita** e inicia navegação autônoma
5. **Monitoramento contínuo** via ROS 2 com atualizações na blockchain
6. **Confirmação de entrega** libera pagamento automaticamente

**Vantagens da Integração:**

- **Transparência**: Todas as entregas ficam registradas de forma imutável
- **Pagamentos automáticos**: Smart contracts garantem pagamento justo
- **Rastreabilidade**: Posição e status sempre disponíveis
- **Descentralização**: Não depende de uma empresa central
- **Segurança**: Blockchain protege contra fraudes

Este sistema combina a robustez do ROS 2 para controle robótico com a segurança e transparência da blockchain, criando um serviço de entrega verdadeiramente autônomo e descentralizado[1][4].

Citations:
[1] https://www.theconstruct.ai/criacao-de-servicos-personalizados-no-ros2-e-controle-de-movimento-do-robo-em-python-portuguese-ros2-tutorial/
[2] https://books-sol.sbc.org.br/index.php/sbc/catalog/download/29/96/246-1?inline=1
[3] https://www.luiztools.com.br/post/como-criar-seu-primeiro-smart-contract-em-solidity-com-remix/
[4] https://www.youtube.com/watch?v=UqxzZFNAcRI
[5] https://www.voitto.com.br/blog/artigo/startups-que-utilizam-blockchain
[6] https://monografias.ufop.br/bitstream/35400000/4899/6/MONOGRAFIA_AbordagemBlockchainProblema.pdf
[7] https://www.inteli.edu.br/modulos/robo-de-servico-autonomo-com-ia-generativa/
[8] https://www.reddit.com/r/ROS/comments/1i3jogb/what_companies_actually_use_ros2_in_production/?tl=pt-br
[9] https://www.ppgi.unb.br/noticias/456-seminario-09-12-22-14h
[10] https://repositorio.ufmg.br/bitstream/1843/68078/2/CONTRATOS%20INTELIGENTES%20NA%20BLOCKCHAIN_%20VALIDADE%20E%20RESTRI%C3%87%C3%95ES.pdf
[11] https://revistaannep.com.br/index.php/radp/article/download/7/pdf/111
[12] https://lume.ufrgs.br/bitstream/handle/10183/213349/001117332.pdf?sequence
[13] https://attena.ufpe.br/bitstream/123456789/43025/1/TESE%20Carlos%20Alexandre%20Silva%20de%20Melo.pdf
[14] https://101blockchains.com/pt/tecnologia-blockchain-guia/
[15] https://books-sol.sbc.org.br/index.php/sbc/catalog/download/77/329/585?inline=1
[16] https://www.infoteca.cnptia.embrapa.br/infoteca/bitstream/doc/1101384/1/ModelorastreamentoCT130.pdf
[17] https://101blockchains.com/pt/empresas-implementando-tecnologia-blockchain/
[18] https://portal.tcu.gov.br/data/files/58/02/CE/5E/C4854710A7AE4547E18818A8/Blockchain_apendice1.pdf
[19] https://monografias.faculdadebaianadedireito.com.br/wp-content/uploads/2025/04/LEONARDO-EVANGELISTA-DOS-SANTOS.pdf

---
Answer from Perplexity: pplx.ai/share
