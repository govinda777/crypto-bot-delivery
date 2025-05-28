# Crypto-Bot-Delivery

Crypto-Bot-Delivery is a project to develop a decentralized autonomous delivery robot system. It integrates blockchain technology for secure payments, ROS2 for autonomous navigation, and Gazebo for simulation.

## Overview

This system aims to provide an end-to-end solution for autonomous deliveries, from order placement and crypto payment to robot navigation and delivery confirmation via QR codes. The project emphasizes decentralization, security, and autonomy.

## High-Level Features (Target)

*   Automated delivery quotation based on distance and complexity.
*   Secure and transparent payments using cryptocurrencies and smart contracts.
*   Autonomous robot navigation using ROS2 Nav2 stack for path planning and obstacle avoidance.
*   Simulation environment in Gazebo for testing and validation.
*   QR code based pickup and delivery confirmation.
*   (Future) Mobile application for user interaction.

## Repository Structure

*   `src/`: Contains the source code for various components:
    *   `delivery_api/`: Backend FastAPI application.
    *   `delivery_crypto/`: Blockchain interaction scripts and utilities.
    *   `delivery_navigation/`: ROS2 navigation package.
    *   `delivery_qr/`: QR code generation and validation system.
    *   `delivery_simulation/`: Gazebo simulation setup (robot models, worlds).
*   `smart_contracts/`: Solidity smart contracts for the blockchain component.
*   `POC/`: Proof of Concept demonstrations for key functionalities.
*   `docs/`: Detailed documentation for different aspects of the project.
*   `mobile_app/`: Placeholder for the future mobile application.
*   *(Other directories as needed)*

## System Architecture

The following diagram illustrates the high-level architecture of the Crypto-Bot-Delivery system:

```mermaid
graph LR
    subgraph User Interaction
        MobileApp[Mobile App / User Interface]
    end

    subgraph Backend Services
        API[FastAPI Backend API]
    end

    subgraph Blockchain Network
        SmartContract[Escrow Smart Contract]
        Blockchain[Ethereum/Polygon Blockchain]
    end

    subgraph Robot Systems
        ROS2Nav[ROS2 Navigation Stack (Nav2)]
        RobotHW[Robot Hardware/Sensors]
        QRCodeScanner[QR Code Scanner (OpenCV)]
    end
    
    subgraph Simulation
        Gazebo[Gazebo Simulation]
        SimRobot[Simulated Robot Model]
        SimSensors[Simulated Sensors (LiDAR, Camera)]
    end

    MobileApp -- REST API Calls --> API
    API -- Delivery Data Encryption --> API
    API -- Smart Contract Interactions --> SmartContract
    SmartContract -- Transactions/Events --> Blockchain
    
    API -- Task Commands (Start Nav, etc.) --> ROS2Nav
    ROS2Nav -- Control Commands --> SimRobot
    ROS2Nav -- Control Commands --> RobotHW
    
    SimSensors --> ROS2Nav
    RobotHW --> ROS2Nav
    
    SimRobot -- Sensor Data --> SimSensors
    SimRobot -- Physical Interaction --> Gazebo
    Gazebo -- World State --> SimRobot

    QRCodeScanner -- QR Data --> API  // For confirmation
    ROS2Nav -- Robot Status --> API // For monitoring
    
    classDef backend fill:#f9d,stroke:#333,stroke-width:2px;
    classDef blockchain fill:#ccf,stroke:#333,stroke-width:2px;
    classDef robot fill:#9cf,stroke:#333,stroke-width:2px;
    classDef userapp fill:#9f9,stroke:#333,stroke-width:2px;
    classDef simulation fill:#fca,stroke:#333,stroke-width:2px;

    class MobileApp userapp;
    class API backend;
    class SmartContract,Blockchain blockchain;
    class ROS2Nav,RobotHW,QRCodeScanner robot;
    class Gazebo,SimRobot,SimSensors simulation;
```

## Getting Started

To understand and run parts of this project, please refer to the documentation within the `POC/` (Proof of Concept) directories and the detailed guides in the `docs/` directory.

*   **[POC 1: Blockchain Integration](./POC/poc_blockchain/README.md)**
*   **[POC 2: ROS2 Navigation Stack](./POC/poc_ros2_navigation/README.md)**
*   *(Links to other POCs as they are developed)*

## Documentation

*   **[Smart Contract Guide](./docs/smart_contract_guide.md)**
*   **[Blockchain Interaction Guide](./docs/blockchain_interaction_guide.md)**
*   *(Links to other documentation files)*

---
*This project is currently under development.*
```
