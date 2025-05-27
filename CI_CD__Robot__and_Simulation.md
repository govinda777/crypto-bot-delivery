Deploy no Robô e Simulação Completa

# Esclarecimentos sobre Deploy no Robô e Simulação Completa

Você está certo! Vou esclarecer essas partes fundamentais que ficaram vagas no tutorial anterior.

## **1. Como o Código é Instalado e Executado no Robô**

### **Deploy Automático via Git + SSH**

#### **1.1 Configuração do Robô (Raspberry Pi)**
```bash
# No Raspberry Pi do robô
# Instalar dependências
sudo apt update
sudo apt install git python3-pip ros-humble-desktop

# Criar usuário para deploy automático
sudo useradd -m -s /bin/bash robot_deploy
sudo usermod -aG sudo robot_deploy

# Configurar chaves SSH para deploy sem senha
ssh-keygen -t rsa -b 4096 -C "robot_deploy@delivery_robot"
# Copiar chave pública para servidor de desenvolvimento
```

#### **1.2 Script de Deploy Automático (deploy_to_robot.sh)**
```bash
#!/bin/bash

# Script para fazer deploy automático no robô
ROBOT_IP="192.168.1.100"  # IP do seu robô
ROBOT_USER="robot_deploy"
PROJECT_PATH="/home/robot_deploy/delivery_robot_ws"

echo "🚀 Iniciando deploy para o robô..."

# 1. Fazer backup do código atual no robô
ssh $ROBOT_USER@$ROBOT_IP "cd $PROJECT_PATH && git stash"

# 2. Atualizar código via Git
ssh $ROBOT_USER@$ROBOT_IP "cd $PROJECT_PATH && git pull origin main"

# 3. Instalar dependências Python
ssh $ROBOT_USER@$ROBOT_IP "cd $PROJECT_PATH && pip3 install -r requirements.txt"

# 4. Compilar workspace ROS2
ssh $ROBOT_USER@$ROBOT_IP "cd $PROJECT_PATH && source /opt/ros/humble/setup.bash && colcon build"

# 5. Reiniciar serviços do robô
ssh $ROBOT_USER@$ROBOT_IP "sudo systemctl restart delivery_robot.service"

echo "✅ Deploy concluído! Robô atualizado."
```

#### **1.3 Serviço Systemd para Auto-Start (delivery_robot.service)**
```ini
[Unit]
Description=Delivery Robot Service
After=network.target

[Service]
Type=simple
User=robot_deploy
WorkingDirectory=/home/robot_deploy/delivery_robot_ws
Environment=ROS_DOMAIN_ID=42
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch delivery_navigation robot_startup.launch.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

---

## **2. Simulação Completa com Blockchain Local**

### **2.1 Configuração da Blockchain Local**

#### **Instalar Ganache (Blockchain Local)**
```bash
# Instalar Node.js e NPM
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Instalar Ganache CLI
npm install -g ganache-cli

# Instalar Truffle para smart contracts
npm install -g truffle

# Instalar Web3.py para Python
pip3 install web3 eth-account
```

#### **2.2 Smart Contract para Entregas (DeliveryContract.sol)**
```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract DeliveryContract {
    struct Delivery {
        address payable customer;
        address payable robot;
        uint256 amount;
        string encryptedRoute;
        bool isPaid;
        bool isPickedUp;
        bool isDelivered;
        uint256 timestamp;
    }
    
    mapping(string => Delivery) public deliveries;
    address public owner;
    
    event DeliveryCreated(string deliveryId, address customer, uint256 amount);
    event PaymentReceived(string deliveryId, uint256 amount);
    event PickupConfirmed(string deliveryId);
    event DeliveryCompleted(string deliveryId);
    
    constructor() {
        owner = msg.sender;
    }
    
    function createDelivery(
        string memory deliveryId,
        string memory encryptedRoute,
        address payable robotAddress
    ) public payable {
        require(msg.value > 0, "Payment required");
        require(deliveries[deliveryId].customer == address(0), "Delivery already exists");
        
        deliveries[deliveryId] = Delivery({
            customer: payable(msg.sender),
            robot: robotAddress,
            amount: msg.value,
            encryptedRoute: encryptedRoute,
            isPaid: true,
            isPickedUp: false,
            isDelivered: false,
            timestamp: block.timestamp
        });
        
        emit DeliveryCreated(deliveryId, msg.sender, msg.value);
        emit PaymentReceived(deliveryId, msg.value);
    }
    
    function confirmPickup(string memory deliveryId) public {
        Delivery storage delivery = deliveries[deliveryId];
        require(delivery.robot == msg.sender, "Only assigned robot can confirm pickup");
        require(delivery.isPaid, "Payment not received");
        require(!delivery.isPickedUp, "Already picked up");
        
        delivery.isPickedUp = true;
        emit PickupConfirmed(deliveryId);
    }
    
    function confirmDelivery(string memory deliveryId) public {
        Delivery storage delivery = deliveries[deliveryId];
        require(delivery.robot == msg.sender, "Only assigned robot can confirm delivery");
        require(delivery.isPickedUp, "Not picked up yet");
        require(!delivery.isDelivered, "Already delivered");
        
        delivery.isDelivered = true;
        
        // Transferir pagamento para o robô (85%) e taxa para o dono (15%)
        uint256 robotPayment = (delivery.amount * 85) / 100;
        uint256 platformFee = delivery.amount - robotPayment;
        
        delivery.robot.transfer(robotPayment);
        payable(owner).transfer(platformFee);
        
        emit DeliveryCompleted(deliveryId);
    }
}
```

#### **2.3 Script de Inicialização da Blockchain (start_blockchain.py)**
```python
#!/usr/bin/env python3

import subprocess
import time
import json
from web3 import Web3
from eth_account import Account
import requests

class BlockchainManager:
    def __init__(self):
        self.w3 = None
        self.contract = None
        self.accounts = []
        self.ganache_process = None
        
    def start_ganache(self):
        """Iniciar Ganache com contas pré-configuradas"""
        print("🔗 Iniciando blockchain local...")
        
        # Comando para iniciar Ganache
        cmd = [
            "ganache-cli",
            "--accounts", "10",           # 10 contas
            "--defaultBalanceEther", "100", # 100 ETH por conta
            "--gasLimit", "8000000",      # Limite de gas
            "--port", "8545",             # Porta
            "--networkId", "1337",        # Network ID
            "--deterministic"             # Contas determinísticas
        ]
        
        self.ganache_process = subprocess.Popen(
            cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE
        )
        
        # Aguardar Ganache inicializar
        time.sleep(5)
        
        # Conectar ao Web3
        self.w3 = Web3(Web3.HTTPProvider('http://localhost:8545'))
        
        if self.w3.is_connected():
            print("✅ Conectado à blockchain local")
            self.accounts = self.w3.eth.accounts
            print(f"📊 {len(self.accounts)} contas disponíveis")
            
            # Mostrar saldos iniciais
            for i, account in enumerate(self.accounts[:3]):
                balance = self.w3.eth.get_balance(account)
                balance_eth = self.w3.from_wei(balance, 'ether')
                print(f"   Conta {i}: {account} - {balance_eth} ETH")
                
        return self.w3.is_connected()
    
    def deploy_contract(self):
        """Deploy do smart contract"""
        print("📜 Fazendo deploy do smart contract...")
        
        # Compilar contrato (você precisa ter o arquivo compilado)
        with open('build/contracts/DeliveryContract.json', 'r') as f:
            contract_json = json.load(f)
            
        contract_abi = contract_json['abi']
        contract_bytecode = contract_json['bytecode']
        
        # Deploy
        contract = self.w3.eth.contract(
            abi=contract_abi,
            bytecode=contract_bytecode
        )
        
        # Usar primeira conta como deployer
        deployer = self.accounts[0]
        
        # Construir transação
        transaction = contract.constructor().build_transaction({
            'from': deployer,
            'gas': 2000000,
            'gasPrice': self.w3.to_wei('20', 'gwei'),
            'nonce': self.w3.eth.get_transaction_count(deployer)
        })
        
        # Assinar e enviar transação
        signed_txn = self.w3.eth.account.sign_transaction(
            transaction, 
            private_key="0x" + "0" * 63 + "1"  # Chave privada da primeira conta Ganache
        )
        
        tx_hash = self.w3.eth.send_raw_transaction(signed_txn.rawTransaction)
        tx_receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)
        
        self.contract = self.w3.eth.contract(
            address=tx_receipt.contractAddress,
            abi=contract_abi
        )
        
        print(f"✅ Contrato deployado em: {tx_receipt.contractAddress}")
        return tx_receipt.contractAddress
    
    def create_test_wallets(self):
        """Criar carteiras para teste"""
        wallets = {
            'customer': {
                'address': self.accounts[1],
                'private_key': "0x" + "0" * 62 + "02",  # Segunda conta Ganache
                'role': 'Cliente'
            },
            'robot': {
                'address': self.accounts[2], 
                'private_key': "0x" + "0" * 62 + "03",  # Terceira conta Ganache
                'role': 'Robô'
            },
            'destination': {
                'address': self.accounts[3],
                'private_key': "0x" + "0" * 62 + "04",  # Quarta conta Ganache
                'role': 'Destinatário'
            }
        }
        
        print("\n💳 Carteiras de teste criadas:")
        for name, wallet in wallets.items():
            balance = self.w3.eth.get_balance(wallet['address'])
            balance_eth = self.w3.from_wei(balance, 'ether')
            print(f"   {wallet['role']}: {wallet['address']} - {balance_eth} ETH")
            
        return wallets

def main():
    blockchain = BlockchainManager()
    
    # Iniciar blockchain
    if blockchain.start_ganache():
        # Deploy contrato
        contract_address = blockchain.deploy_contract()
        
        # Criar carteiras de teste
        wallets = blockchain.create_test_wallets()
        
        # Salvar configurações para uso posterior
        config = {
            'blockchain_url': 'http://localhost:8545',
            'contract_address': contract_address,
            'wallets': wallets
        }
        
        with open('blockchain_config.json', 'w') as f:
            json.dump(config, f, indent=2)
            
        print("\n🎉 Blockchain configurada e pronta para uso!")
        print("📁 Configurações salvas em blockchain_config.json")
        
        # Manter Ganache rodando
        try:
            blockchain.ganache_process.wait()
        except KeyboardInterrupt:
            print("\n🛑 Parando blockchain...")
            blockchain.ganache_process.terminate()

if __name__ == "__main__":
    main()
```

---

## **3. Simulação Completa End-to-End**

### **3.1 Script de Teste Completo (test_full_simulation.py)**
```python
#!/usr/bin/env python3

import json
import time
import qrcode
import cv2
from web3 import Web3
from eth_account import Account
import requests
import subprocess
import threading

class DeliverySimulation:
    def __init__(self):
        # Carregar configuração da blockchain
        with open('blockchain_config.json', 'r') as f:
            self.config = json.load(f)
            
        self.w3 = Web3(Web3.HTTPProvider(self.config['blockchain_url']))
        
        # Carregar contrato
        with open('build/contracts/DeliveryContract.json', 'r') as f:
            contract_json = json.load(f)
            
        self.contract = self.w3.eth.contract(
            address=self.config['contract_address'],
            abi=contract_json['abi']
        )
        
        self.wallets = self.config['wallets']
        
    def step1_create_delivery_request(self):
        """Passo 1: Cliente cria solicitação de entrega"""
        print("\n🛒 PASSO 1: Cliente criando solicitação de entrega")
        
        delivery_data = {
            'delivery_id': f"DEL_{int(time.time())}",
            'origin': "Rua A, 123 - São Paulo, SP",
            'destination': "Rua B, 456 - São Paulo, SP", 
            'package_weight': 2.5,
            'package_size': "medium",
            'price_eth': 0.1  # 0.1 ETH
        }
        
        print(f"   📦 ID da Entrega: {delivery_data['delivery_id']}")
        print(f"   📍 Origem: {delivery_data['origin']}")
        print(f"   📍 Destino: {delivery_data['destination']}")
        print(f"   💰 Preço: {delivery_data['price_eth']} ETH")
        
        return delivery_data
    
    def step2_process_payment(self, delivery_data):
        """Passo 2: Processar pagamento na blockchain"""
        print("\n💳 PASSO 2: Processando pagamento")
        
        customer_wallet = self.wallets['customer']
        robot_wallet = self.wallets['robot']
        
        # Criar transação de pagamento
        transaction = self.contract.functions.createDelivery(
            delivery_data['delivery_id'],
            json.dumps(delivery_data),  # Dados criptografados (simplificado)
            robot_wallet['address']
        ).build_transaction({
            'from': customer_wallet['address'],
            'value': self.w3.to_wei(delivery_data['price_eth'], 'ether'),
            'gas': 300000,
            'gasPrice': self.w3.to_wei('20', 'gwei'),
            'nonce': self.w3.eth.get_transaction_count(customer_wallet['address'])
        })
        
        # Assinar transação
        signed_txn = self.w3.eth.account.sign_transaction(
            transaction, 
            private_key=customer_wallet['private_key']
        )
        
        # Enviar transação
        tx_hash = self.w3.eth.send_raw_transaction(signed_txn.rawTransaction)
        tx_receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)
        
        print(f"   ✅ Pagamento confirmado! TX: {tx_hash.hex()}")
        print(f"   ⛽ Gas usado: {tx_receipt.gasUsed}")
        
        return tx_hash.hex()
    
    def step3_robot_navigation(self, delivery_data):
        """Passo 3: Robô navega para origem"""
        print("\n🤖 PASSO 3: Robô navegando para origem")
        
        # Simular comando ROS2 para navegação
        navigation_cmd = f"""
        ros2 topic pub /delivery_command std_msgs/String "{{
            'action': 'navigate_to_pickup',
            'delivery_id': '{delivery_data['delivery_id']}',
            'destination': '{delivery_data['origin']}'
        }}" --once
        """
        
        print(f"   🚀 Comando enviado para ROS2:")
        print(f"   {navigation_cmd}")
        
        # Simular tempo de navegação
        print("   🚶 Robô navegando... (simulando 30 segundos)")
        for i in range(6):
            time.sleep(5)
            print(f"   📍 Progresso: {(i+1)*20}%")
            
        print("   ✅ Robô chegou na origem!")
        
    def step4_generate_pickup_qr(self, delivery_data):
        """Passo 4: Gerar QR Code para coleta"""
        print("\n📱 PASSO 4: Gerando QR Code para coleta")
        
        qr_data = {
            'action': 'pickup',
            'delivery_id': delivery_data['delivery_id'],
            'timestamp': int(time.time()),
            'robot_address': self.wallets['robot']['address']
        }
        
        # Gerar QR Code
        qr = qrcode.QRCode(version=1, box_size=10, border=5)
        qr.add_data(json.dumps(qr_data))
        qr.make(fit=True)
        
        qr_image = qr.make_image(fill_color="black", back_color="white")
        qr_filename = f"pickup_qr_{delivery_data['delivery_id']}.png"
        qr_image.save(qr_filename)
        
        print(f"   📱 QR Code gerado: {qr_filename}")
        print(f"   📄 Dados do QR: {json.dumps(qr_data, indent=2)}")
        
        return qr_filename, qr_data
    
    def step5_confirm_pickup_blockchain(self, delivery_data):
        """Passo 5: Confirmar coleta na blockchain"""
        print("\n⛓️ PASSO 5: Confirmando coleta na blockchain")
        
        robot_wallet = self.wallets['robot']
        
        # Transação de confirmação de coleta
        transaction = self.contract.functions.confirmPickup(
            delivery_data['delivery_id']
        ).build_transaction({
            'from': robot_wallet['address'],
            'gas': 100000,
            'gasPrice': self.w3.to_wei('20', 'gwei'),
            'nonce': self.w3.eth.get_transaction_count(robot_wallet['address'])
        })
        
        # Assinar e enviar
        signed_txn = self.w3.eth.account.sign_transaction(
            transaction, 
            private_key=robot_wallet['private_key']
        )
        
        tx_hash = self.w3.eth.send_raw_transaction(signed_txn.rawTransaction)
        tx_receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)
        
        print(f"   ✅ Coleta confirmada na blockchain! TX: {tx_hash.hex()}")
        
    def step6_navigate_to_destination(self, delivery_data):
        """Passo 6: Navegar para destino"""
        print("\n🎯 PASSO 6: Navegando para destino")
        
        print(f"   📍 Destino: {delivery_data['destination']}")
        print("   🚶 Robô navegando para destino... (simulando 45 segundos)")
        
        for i in range(9):
            time.sleep(5)
            print(f"   📍 Progresso: {(i+1)*11}%")
            
        print("   ✅ Robô chegou no destino!")
    
    def step7_generate_delivery_qr(self, delivery_data):
        """Passo 7: Gerar QR Code para entrega"""
        print("\n📱 PASSO 7: Gerando QR Code para confirmação de entrega")
        
        qr_data = {
            'action': 'delivery',
            'delivery_id': delivery_data['delivery_id'],
            'timestamp': int(time.time()),
            'robot_address': self.wallets['robot']['address'],
            'destination_address': self.wallets['destination']['address']
        }
        
        # Gerar QR Code
        qr = qrcode.QRCode(version=1, box_size=10, border=5)
        qr.add_data(json.dumps(qr_data))
        qr.make(fit=True)
        
        qr_image = qr.make_image(fill_color="black", back_color="white")
        qr_filename = f"delivery_qr_{delivery_data['delivery_id']}.png"
        qr_image.save(qr_filename)
        
        print(f"   📱 QR Code de entrega gerado: {qr_filename}")
        
        return qr_filename, qr_data
    
    def step8_scan_qr_and_complete(self, delivery_data, qr_data):
        """Passo 8: Destinatário escaneia QR e libera pagamento"""
        print("\n🔍 PASSO 8: Destinatário escaneando QR Code")
        
        print("   📱 Simulando escaneamento do QR Code...")
        time.sleep(3)
        
        print(f"   ✅ QR Code válido! Dados: {json.dumps(qr_data, indent=2)}")
        
        # Confirmar entrega na blockchain
        robot_wallet = self.wallets['robot']
        
        transaction = self.contract.functions.confirmDelivery(
            delivery_data['delivery_id']
        ).build_transaction({
            'from': robot_wallet['address'],
            'gas': 150000,
            'gasPrice': self.w3.to_wei('20', 'gwei'),
            'nonce': self.w3.eth.get_transaction_count(robot_wallet['address'])
        })
        
        signed_txn = self.w3.eth.account.sign_transaction(
            transaction, 
            private_key=robot_wallet['private_key']
        )
        
        tx_hash = self.w3.eth.send_raw_transaction(signed_txn.rawTransaction)
        tx_receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)
        
        print(f"   💰 Pagamento liberado! TX: {tx_hash.hex()}")
        print(f"   🎉 Entrega concluída com sucesso!")
        
        # Mostrar saldos finais
        self.show_final_balances()
    
    def show_final_balances(self):
        """Mostrar saldos finais das carteiras"""
        print("\n💰 SALDOS FINAIS:")
        
        for name, wallet in self.wallets.items():
            balance = self.w3.eth.get_balance(wallet['address'])
            balance_eth = self.w3.from_wei(balance, 'ether')
            print(f"   {wallet['role']}: {balance_eth:.4f} ETH")
    
    def run_full_simulation(self):
        """Executar simulação completa"""
        print("🚀 INICIANDO SIMULAÇÃO COMPLETA DE ENTREGA")
        print("=" * 50)
        
        try:
            # Passo 1: Criar solicitação
            delivery_data = self.step1_create_delivery_request()
            
            # Passo 2: Processar pagamento
            tx_hash = self.step2_process_payment(delivery_data)
            
            # Passo 3: Robô navega para origem
            self.step3_robot_navigation(delivery_data)
            
            # Passo 4: Gerar QR de coleta
            pickup_qr, pickup_data = self.step4_generate_pickup_qr(delivery_data)
            
            # Passo 5: Confirmar coleta
            self.step5_confirm_pickup_blockchain(delivery_data)
            
            # Passo 6: Navegar para destino
            self.step6_navigate_to_destination(delivery_data)
            
            # Passo 7: Gerar QR de entrega
            delivery_qr, delivery_qr_data = self.step7_generate_delivery_qr(delivery_data)
            
            # Passo 8: Escanear QR e completar
            self.step8_scan_qr_and_complete(delivery_data, delivery_qr_data)
            
            print("\n🎊 SIMULAÇÃO CONCLUÍDA COM SUCESSO!")
            
        except Exception as e:
            print(f"\n❌ Erro na simulação: {str(e)}")

def main():
    # Verificar se blockchain está rodando
    try:
        w3 = Web3(Web3.HTTPProvider('http://localhost:8545'))
        if not w3.is_connected():
            print("❌ Blockchain local não está rodando!")
            print("Execute: python3 start_blockchain.py")
            return
    except:
        print("❌ Erro ao conectar com blockchain!")
        return
    
    # Executar simulação
    simulation = DeliverySimulation()
    simulation.run_full_simulation()

if __name__ == "__main__":
    main()
```

---

## **4. Como Executar a Simulação Completa**

### **Passo a Passo para Rodar Tudo:**

```bash
# Terminal 1: Iniciar blockchain local
python3 start_blockchain.py

# Terminal 2: Iniciar simulação ROS2 (em paralelo)
source ~/delivery_robot_ws/install/setup.bash
ros2 launch delivery_simulation simulation.launch.py

# Terminal 3: Executar simulação completa
python3 test_full_simulation.py
```

### **Resultado Esperado:**
1. ✅ Blockchain local com 10 contas e 100 ETH cada
2. ✅ Smart contract deployado
3. ✅ Cliente paga 0.1 ETH pela entrega
4. ✅ Robô navega para origem (simulado no Gazebo)
5. ✅ QR Code gerado para coleta
6. ✅ Coleta confirmada na blockchain
7. ✅ Robô navega para destino
8. ✅ QR Code gerado para entrega
9. ✅ Destinatário escaneia QR
10. ✅ Pagamento liberado automaticamente (85% para robô, 15% taxa)

Agora está claro como o robô recebe e executa o código, e como toda a simulação funciona end-to-end com blockchain real!

Citations:
[1] https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/61929226/4292452c-f2b1-4026-bdfe-47886c8650a8/paste.txt
[2] https://www.dexterindustries.com/GoPiGo/get-started-with-the-gopigo3-raspberry-pi-robot/3-program-your-raspberry-pi-robot/python-programming-language/
[3] https://github.com/DexterInd/GoPiGo
[4] https://www.youtube.com/watch?v=_zTHUUOZdec
[5] https://github.com/pubkey/eth-crypto/blob/master/tutorials/creating-transactions.md
[6] https://www.bitget.com/wiki/how-to-make-payment-using-qr-code
[7] https://www.instructables.com/Browser-Streaming-Robot-With-the-GoPiGo3/
[8] https://testautomationu.applitools.com/blockchain-testing/chapter2.html
[9] https://docs.kurtosis.com/how-to-local-eth-testnet
[10] https://gopigo3.readthedocs.io/en/master/quickstart.html
[11] https://github.com/DexterInd/GoPiGo3
[12] https://gopigo.io
[13] https://ethereum.org/en/developers/local-environment/
[14] https://www.quicknode.com/guides/ethereum-development/smart-contracts/how-to-setup-local-development-environment-for-solidity
[15] https://stackoverflow.com/questions/73698203/is-it-possible-to-build-a-real-world-private-ethereum-blockchain-without-any-tra
[16] https://github.com/DexterInd/GoPiGo3/blob/master/Projects/BasicRobotControl/keyboarded_robot.py
[17] https://classic.gazebosim.org/tutorials?tut=ros2_installing
[18] https://www.dexterindustries.com/GoPiGo/get-started-with-the-gopigo3-raspberry-pi-robot/test-and-troubleshoot-the-gopigo3-raspberry-pi-robot/
[19] https://gopigo3.readthedocs.io/en/master/api-basic/easygopigo3.html
[20] https://natashaskitchen.com/perfect-chocolate-ganache-how-to-glaze-a-cake/
[21] https://www.reddit.com/r/ethereum/comments/ts5656/what_is_the_absolute_most_secure_way_to_create_an/
