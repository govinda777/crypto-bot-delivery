
# Configuração Completa do Raspberry Pi 4 para Deploy Automático

## **Pré-requisitos**
- Raspberry Pi 4 com Ubuntu Server 22.04 LTS instalado
- Acesso SSH configurado
- Conexão com internet

---

## **1. Configuração Inicial do Raspberry Pi 4**

### **1.1 Primeiro Acesso e Configurações Básicas**

```bash
# Conectar via SSH (do seu computador principal)
ssh ubuntu@IP_DO_RASPBERRY_PI

# Atualizar sistema
sudo apt update && sudo apt upgrade -y

# Instalar ferramentas essenciais
sudo apt install -y git curl wget vim nano htop net-tools

# Configurar timezone
sudo timedatectl set-timezone America/Sao_Paulo

# Verificar informações do sistema
uname -a
free -h
df -h
```

### **1.2 Configurar Usuário para Deploy**

```bash
# Criar usuário específico para deploy
sudo useradd -m -s /bin/bash robot_deploy
sudo usermod -aG sudo robot_deploy

# Definir senha para o usuário
sudo passwd robot_deploy

# Configurar sudo sem senha para operações específicas
sudo visudo

# Adicionar esta linha no final do arquivo:
# robot_deploy ALL=(ALL) NOPASSWD: /bin/systemctl, /usr/bin/docker, /usr/local/bin/docker-compose
```

### **1.3 Configurar Chaves SSH para Deploy Automático**

```bash
# Mudar para usuário robot_deploy
sudo su - robot_deploy

# Gerar chave SSH
ssh-keygen -t ed25519 -C "robot_deploy@delivery_robot" -f ~/.ssh/id_ed25519

# Mostrar chave pública (copie esta chave)
cat ~/.ssh/id_ed25519.pub

# Configurar authorized_keys para acesso remoto
mkdir -p ~/.ssh
chmod 700 ~/.ssh
touch ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys

# Cole aqui a chave pública do seu computador de desenvolvimento
nano ~/.ssh/authorized_keys
```

---

## **2. Instalação do ROS2 Humble no Raspberry Pi**

### **2.1 Configurar Repositórios ROS2**

```bash
# Configurar locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Adicionar repositório ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe

# Adicionar chave GPG do ROS2
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Adicionar repositório à sources.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### **2.2 Instalar ROS2 Humble**

```bash
# Atualizar índice de pacotes
sudo apt update

# Instalar ROS2 Base (versão leve para Raspberry Pi)
sudo apt install -y ros-humble-ros-base

# Instalar ferramentas de desenvolvimento
sudo apt install -y python3-pip python3-rosdep python3-colcon-common-extensions

# Inicializar rosdep
sudo rosdep init
rosdep update

# Configurar ambiente ROS2 permanentemente
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### **2.3 Instalar Dependências Específicas**

```bash
# Navegação e sensores
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-robot-localization

# Drivers de hardware
sudo apt install -y ros-humble-gpio-controllers
sudo apt install -y ros-humble-serial-driver

# Ferramentas de desenvolvimento
sudo apt install -y ros-humble-rqt*
```

---

## **3. Configuração do Sistema de Deploy Automático**

### **3.1 Criar Estrutura de Diretórios**

```bash
# Como usuário robot_deploy
sudo su - robot_deploy

# Criar estrutura do projeto
mkdir -p ~/delivery_robot_ws/{src,scripts,logs,config}
mkdir -p ~/delivery_robot_ws/src/{delivery_navigation,delivery_qr,delivery_crypto}

# Criar diretório para backups
mkdir -p ~/delivery_robot_ws/backups
```

### **3.2 Script de Deploy Automático (no Raspberry Pi)**

```bash
# Criar script de atualização automática
nano ~/delivery_robot_ws/scripts/auto_update.sh
```

**Conteúdo do auto_update.sh:**

```bash
#!/bin/bash

# Script de atualização automática para Raspberry Pi
# Executa a cada 5 minutos via cron

LOG_FILE="/home/robot_deploy/delivery_robot_ws/logs/auto_update.log"
PROJECT_DIR="/home/robot_deploy/delivery_robot_ws"
REPO_URL="git@github.com:seu_usuario/delivery_robot.git"
BRANCH="main"

# Função para log
log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> $LOG_FILE
}

# Função para backup
create_backup() {
    local backup_dir="$PROJECT_DIR/backups/backup_$(date +%Y%m%d_%H%M%S)"
    mkdir -p $backup_dir
    
    if [ -d "$PROJECT_DIR/src" ]; then
        cp -r $PROJECT_DIR/src $backup_dir/
        log_message "Backup criado em: $backup_dir"
    fi
}

# Função para verificar mudanças no repositório
check_for_updates() {
    cd $PROJECT_DIR
    
    # Fetch do repositório remoto
    git fetch origin $BRANCH 2>/dev/null
    
    # Verificar se há commits novos
    local local_commit=$(git rev-parse HEAD)
    local remote_commit=$(git rev-parse origin/$BRANCH)
    
    if [ "$local_commit" != "$remote_commit" ]; then
        return 0  # Há atualizações
    else
        return 1  # Não há atualizações
    fi
}

# Função para atualizar código
update_code() {
    cd $PROJECT_DIR
    
    log_message "Iniciando atualização do código..."
    
    # Criar backup antes da atualização
    create_backup
    
    # Fazer stash das mudanças locais
    git stash push -m "Auto-stash before update $(date)"
    
    # Pull das mudanças
    if git pull origin $BRANCH; then
        log_message "Código atualizado com sucesso"
        return 0
    else
        log_message "ERRO: Falha ao atualizar código"
        return 1
    fi
}

# Função para instalar dependências Python
install_python_deps() {
    if [ -f "$PROJECT_DIR/requirements.txt" ]; then
        log_message "Instalando dependências Python..."
        pip3 install -r $PROJECT_DIR/requirements.txt >> $LOG_FILE 2>&1
    fi
}

# Função para compilar workspace ROS2
build_ros_workspace() {
    cd $PROJECT_DIR
    
    log_message "Compilando workspace ROS2..."
    
    # Source do ROS2
    source /opt/ros/humble/setup.bash
    
    # Instalar dependências ROS2
    rosdep install --from-paths src --ignore-src -r -y >> $LOG_FILE 2>&1
    
    # Compilar
    if colcon build --symlink-install >> $LOG_FILE 2>&1; then
        log_message "Workspace compilado com sucesso"
        return 0
    else
        log_message "ERRO: Falha na compilação do workspace"
        return 1
    fi
}

# Função para reiniciar serviços
restart_services() {
    log_message "Reiniciando serviços..."
    
    # Parar serviços
    sudo systemctl stop delivery_robot.service 2>/dev/null
    sudo systemctl stop delivery_navigation.service 2>/dev/null
    
    # Aguardar um momento
    sleep 5
    
    # Iniciar serviços
    sudo systemctl start delivery_robot.service
    sudo systemctl start delivery_navigation.service
    
    log_message "Serviços reiniciados"
}

# Função principal
main() {
    log_message "=== Iniciando verificação de atualizações ==="
    
    # Verificar se há atualizações
    if check_for_updates; then
        log_message "Atualizações disponíveis. Iniciando processo..."
        
        # Atualizar código
        if update_code; then
            # Instalar dependências
            install_python_deps
            
            # Compilar workspace
            if build_ros_workspace; then
                # Reiniciar serviços
                restart_services
                log_message "Atualização concluída com sucesso!"
            else
                log_message "ERRO: Falha na compilação. Atualização abortada."
            fi
        else
            log_message "ERRO: Falha na atualização do código."
        fi
    else
        log_message "Nenhuma atualização disponível."
    fi
    
    log_message "=== Verificação finalizada ==="
}

# Executar função principal
main
```

### **3.3 Tornar Script Executável e Configurar Cron**

```bash
# Tornar script executável
chmod +x ~/delivery_robot_ws/scripts/auto_update.sh

# Configurar cron para execução automática
crontab -e

# Adicionar estas linhas ao crontab:
# Verificar atualizações a cada 5 minutos
*/5 * * * * /home/robot_deploy/delivery_robot_ws/scripts/auto_update.sh

# Reiniciar serviços automaticamente na inicialização
@reboot sleep 30 && /home/robot_deploy/delivery_robot_ws/scripts/auto_update.sh
```

---

## **4. Configuração dos Serviços Systemd**

### **4.1 Serviço Principal do Robô**

```bash
# Criar arquivo de serviço
sudo nano /etc/systemd/system/delivery_robot.service
```

**Conteúdo do delivery_robot.service:**

```ini
[Unit]
Description=Delivery Robot Main Service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=robot_deploy
Group=robot_deploy
WorkingDirectory=/home/robot_deploy/delivery_robot_ws
Environment=ROS_DOMAIN_ID=42
Environment=RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Comando para iniciar o robô
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch delivery_navigation robot_startup.launch.py"

# Reiniciar automaticamente em caso de falha
Restart=always
RestartSec=10

# Logs
StandardOutput=append:/home/robot_deploy/delivery_robot_ws/logs/robot_service.log
StandardError=append:/home/robot_deploy/delivery_robot_ws/logs/robot_service_error.log

[Install]
WantedBy=multi-user.target
```

### **4.2 Serviço de Navegação**

```bash
# Criar serviço de navegação
sudo nano /etc/systemd/system/delivery_navigation.service
```

**Conteúdo do delivery_navigation.service:**

```ini
[Unit]
Description=Delivery Robot Navigation Service
After=delivery_robot.service
Requires=delivery_robot.service

[Service]
Type=simple
User=robot_deploy
Group=robot_deploy
WorkingDirectory=/home/robot_deploy/delivery_robot_ws
Environment=ROS_DOMAIN_ID=42

ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch nav2_bringup navigation_launch.py"

Restart=always
RestartSec=15

StandardOutput=append:/home/robot_deploy/delivery_robot_ws/logs/navigation_service.log
StandardError=append:/home/robot_deploy/delivery_robot_ws/logs/navigation_service_error.log

[Install]
WantedBy=multi-user.target
```

### **4.3 Habilitar e Iniciar Serviços**

```bash
# Recarregar daemon do systemd
sudo systemctl daemon-reload

# Habilitar serviços para iniciar automaticamente
sudo systemctl enable delivery_robot.service
sudo systemctl enable delivery_navigation.service

# Iniciar serviços
sudo systemctl start delivery_robot.service
sudo systemctl start delivery_navigation.service

# Verificar status
sudo systemctl status delivery_robot.service
sudo systemctl status delivery_navigation.service
```

---

## **5. Configuração do Repositório Git**

### **5.1 Inicializar Repositório no Raspberry Pi**

```bash
# Como usuário robot_deploy
cd ~/delivery_robot_ws

# Inicializar git
git init
git remote add origin git@github.com:seu_usuario/delivery_robot.git

# Configurar usuário git
git config user.name "Robot Deploy"
git config user.email "robot@delivery.com"

# Fazer primeiro clone
git pull origin main
```

### **5.2 Configurar Deploy Key no GitHub**

```bash
# Mostrar chave pública do Raspberry Pi
cat ~/.ssh/id_ed25519.pub
```

**No GitHub:**
1. Vá para **Settings** → **Deploy keys**
2. Clique em **Add deploy key**
3. Cole a chave pública
4. Marque **Allow write access** se necessário

---

## **6. Script de Deploy do Computador de Desenvolvimento**

### **6.1 Script para Enviar Atualizações**

```bash
# No seu computador de desenvolvimento
nano deploy_to_robot.sh
```

**Conteúdo do deploy_to_robot.sh:**

```bash
#!/bin/bash

# Script para fazer deploy para o Raspberry Pi
ROBOT_IP="192.168.1.100"  # IP do seu Raspberry Pi
ROBOT_USER="robot_deploy"
BRANCH="main"

echo "🚀 Iniciando deploy para o robô..."

# Verificar se há mudanças não commitadas
if [[ -n $(git status -s) ]]; then
    echo "⚠️  Há mudanças não commitadas. Fazendo commit automático..."
    git add .
    git commit -m "Auto-commit: $(date '+%Y-%m-%d %H:%M:%S')"
fi

# Push para repositório remoto
echo "📤 Enviando código para repositório..."
git push origin $BRANCH

if [ $? -eq 0 ]; then
    echo "✅ Código enviado com sucesso!"
    echo "🤖 O robô irá atualizar automaticamente em até 5 minutos."
    
    # Opcional: Forçar atualização imediata
    read -p "Deseja forçar atualização imediata? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "⚡ Forçando atualização imediata..."
        ssh $ROBOT_USER@$ROBOT_IP "/home/robot_deploy/delivery_robot_ws/scripts/auto_update.sh"
        echo "✅ Atualização forçada concluída!"
    fi
else
    echo "❌ Erro ao enviar código para repositório!"
    exit 1
fi
```

### **6.2 Tornar Script Executável**

```bash
chmod +x deploy_to_robot.sh
```

---

## **7. Monitoramento e Logs**

### **7.1 Script de Monitoramento**

```bash
# No Raspberry Pi
nano ~/delivery_robot_ws/scripts/monitor.sh
```

**Conteúdo do monitor.sh:**

```bash
#!/bin/bash

# Script de monitoramento do sistema

echo "🤖 STATUS DO ROBÔ DE ENTREGA"
echo "================================"

# Status dos serviços
echo "📋 SERVIÇOS:"
systemctl is-active delivery_robot.service
systemctl is-active delivery_navigation.service

# Uso de recursos
echo -e "\n💻 RECURSOS:"
echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
echo "RAM: $(free | grep Mem | awk '{printf("%.1f%%", $3/$2 * 100.0)}')"
echo "Disco: $(df -h / | awk 'NR==2{printf "%s", $5}')"

# Temperatura
echo -e "\n🌡️  TEMPERATURA:"
vcgencmd measure_temp

# Últimas atualizações
echo -e "\n🔄 ÚLTIMAS ATUALIZAÇÕES:"
tail -n 5 ~/delivery_robot_ws/logs/auto_update.log

# Processos ROS2
echo -e "\n🔧 PROCESSOS ROS2:"
ps aux | grep ros2 | grep -v grep | wc -l
echo "processos ROS2 ativos"
```

### **7.2 Comando para Monitoramento Remoto**

```bash
# No seu computador de desenvolvimento
alias robot_status="ssh robot_deploy@192.168.1.100 '/home/robot_deploy/delivery_robot_ws/scripts/monitor.sh'"

# Usar com:
robot_status
```

---

## **8. Teste do Sistema Completo**

### **8.1 Teste de Deploy**

```bash
# No seu computador de desenvolvimento
echo "print('Hello from robot!')" > test_file.py
git add test_file.py
git commit -m "Teste de deploy automático"
./deploy_to_robot.sh
```

### **8.2 Verificar Logs**

```bash
# Verificar logs no Raspberry Pi
ssh robot_deploy@192.168.1.100

# Ver logs de atualização
tail -f ~/delivery_robot_ws/logs/auto_update.log

# Ver logs dos serviços
sudo journalctl -u delivery_robot.service -f
sudo journalctl -u delivery_navigation.service -f
```

---

## **9. Comandos Úteis para Manutenção**

```bash
# Reiniciar todos os serviços
sudo systemctl restart delivery_robot.service delivery_navigation.service

# Ver status detalhado
systemctl status delivery_robot.service --no-pager -l

# Forçar atualização manual
/home/robot_deploy/delivery_robot_ws/scripts/auto_update.sh

# Ver logs em tempo real
tail -f ~/delivery_robot_ws/logs/*.log

# Verificar conectividade
ping google.com

# Verificar espaço em disco
df -h

# Verificar processos ROS2
ros2 node list
ros2 topic list
```

Agora seu Raspberry Pi 4 está completamente configurado para receber atualizações automáticas do código via Git, com monitoramento completo e serviços que reiniciam automaticamente!

Citations:
[1] https://www.youtube.com/watch?v=U3y7gjy9O9o
[2] https://suda.pl/quick-and-dirty-autodeployment-to-raspberry-pi/
[3] https://pt.linkedin.com/pulse/implantando-o-ubuntu-server-2310-em-um-raspberry-pi-alves-de-aguiar-uco4f
[4] https://linuxuniverse.com.br/linux/auto-update
[5] https://www.tabnews.com.br/xambitow/atualizar-aplicacoes-em-varios-raspberry-pi-automaticamente-arquitetura-e-configuracao-completa
[6] https://www.reddit.com/r/raspberry_pi/comments/1jebz1t/server_forge_automatic_server_configurator_for/?tl=pt-br
[7] https://www.youtube.com/watch?v=tv0GIGP_xlQ
[8] https://www.makerhero.com/blog/modo-quiosque-com-raspberry-pi/
[9] https://www.makerhero.com/guia/raspberry-pi/terminal-shell-raspberry-pi/
[10] https://nodered.org/docs/getting-started/raspberrypi
