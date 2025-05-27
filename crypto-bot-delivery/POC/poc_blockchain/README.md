# POC 1: Blockchain Integration

## Overview

This Proof of Concept (POC) demonstrates the integration of a blockchain-based escrow system for managing payments in the Crypto Bot Delivery system. It showcases:
- An Ethereum smart contract (`Escrow.sol`) to hold funds in escrow.
- A Python script (`blockchain_interface.py`) that interacts with the smart contract using the Web3.py library to simulate a delivery workflow.

The key components involved are:
- **`Escrow.sol`**: The Solidity smart contract defining the escrow logic. Located in `crypto-bot-delivery/smart_contracts/`.
- **`Escrow.json`**: The ABI (Application Binary Interface) of the `Escrow.sol` contract, necessary for interacting with it. Located in `crypto-bot-delivery/smart_contracts/`.
- **`blockchain_interface.py`**: The Python script that simulates client and robot interactions with the smart contract. Located in `crypto-bot-delivery/src/delivery_crypto/`.

## Prerequisites

Before running this POC, ensure you have the following installed:

*   **Python 3.10+**
*   **Node.js and npm:** Required if you need to recompile the smart contract (using `npx solc`) or for running local development tools like Ganache.
*   **Ganache:** A local Ethereum testnet. Download from [Truffle Suite](https://trufflesuite.com/ganache/). Ensure it's installed and running.
    *   When setting up Ganache, note the RPC Server URL (e.g., `http://127.0.0.1:7545`).
    *   Take note of at least two account private keys from Ganache to use for the Client and Robot.
*   **`web3.py` Python library:** Install using pip:
    ```bash
    pip install web3
    ```

## Setup Instructions

### 1. Smart Contract (`Escrow.sol`)

*   **Location:** The smart contract source code `Escrow.sol` is located in the `crypto-bot-delivery/smart_contracts/` directory.
*   **ABI:** The compiled ABI, `Escrow.json`, is also provided in `crypto-bot-delivery/smart_contracts/`. It was generated from `Escrow.sol`. If you modify `Escrow.sol`, you will need to recompile it (see project's main README for compilation instructions).

*   **Deployment to Ganache:**
    The `blockchain_interface.py` script assumes the `Escrow.sol` contract is already deployed on your running Ganache instance. You can deploy it using tools like Remix IDE:
    1.  Open [Remix IDE](https://remix.ethereum.org/) in your browser.
    2.  Load `Escrow.sol` into Remix.
    3.  In the "Solidity Compiler" tab, select a compatible compiler version (e.g., `0.8.0` or higher as per `pragma solidity ^0.8.0;`) and compile `Escrow.sol`.
    4.  In the "Deploy & Run Transactions" tab:
        *   Set the "Environment" to "Injected Provider - Web3" (if using MetaMask connected to Ganache) or "Ganache Provider" (if Remix offers a direct connection, newer versions might call this "Dev - Ganache Provider" or similar, ensure it points to your local Ganache instance e.g. http://127.0.0.1:7545). If using "Injected Provider", ensure MetaMask is connected to your local Ganache network.
        *   The `Escrow` contract should be selected in the "Contract" dropdown.
        *   Click "Deploy".
    5.  Once deployed, copy the "Deployed Contracts" address. This is your `ESCROW_CONTRACT_ADDRESS`.

### 2. Python Script (`blockchain_interface.py`)

1.  **Navigate to the script directory:**
    ```bash
    cd crypto-bot-delivery/src/delivery_crypto/
    ```

2.  **Set Environment Variables:**
    The `blockchain_interface.py` script requires the following environment variables to be set. You can set them in your terminal session or use a `.env` file with a library like `python-dotenv` (not included by default).

    *   `ETH_NODE_URL`: The RPC URL of your Ganache instance.
        ```bash
        export ETH_NODE_URL="http://127.0.0.1:7545" 
        ```
        (Replace with your Ganache RPC server URL if different).

    *   `ESCROW_CONTRACT_ADDRESS`: The address of the `Escrow` contract you deployed on Ganache.
        ```bash
        export ESCROW_CONTRACT_ADDRESS="YOUR_DEPLOYED_CONTRACT_ADDRESS_HERE"
        ```

    *   `CLIENT_PRIVATE_KEY`: The private key of an account from Ganache to act as the client.
        **WARNING: These are for local testing with Ganache only. Never expose real private keys.**
        ```bash
        export CLIENT_PRIVATE_KEY="0xYOUR_GANACHE_CLIENT_ACCOUNT_PRIVATE_KEY"
        ```

    *   `ROBOT_PRIVATE_KEY`: The private key of another account from Ganache to act as the robot/delivery provider.
        ```bash
        export ROBOT_PRIVATE_KEY="0xYOUR_GANACHE_ROBOT_ACCOUNT_PRIVATE_KEY"
        ```
    *   `CONTRACT_ABI_PATH` (Optional): This variable specifies the path to the `Escrow.json` ABI file *relative to the `blockchain_interface.py` script's location*. The script defaults to `../../smart_contracts/Escrow.json` (i.e., `crypto-bot-delivery/smart_contracts/Escrow.json` from the project root). You only need to set this if you move the ABI file.
        Example if you had it in the same directory as the script:
        ```bash
        # export CONTRACT_ABI_PATH="Escrow.json" 
        ```

## Running the POC

1.  Ensure your Ganache instance is running and the `Escrow.sol` contract is deployed.
2.  Ensure all required environment variables (`ETH_NODE_URL`, `ESCROW_CONTRACT_ADDRESS`, `CLIENT_PRIVATE_KEY`, `ROBOT_PRIVATE_KEY`) are set in your terminal session.
3.  Navigate to the script's directory (if not already there):
    ```bash
    cd crypto-bot-delivery/src/delivery_crypto/
    ```
4.  Run the Python script:
    ```bash
    python blockchain_interface.py
    ```

**Expected Output:**

The script will print console logs detailing its actions:
*   Connection to the Ethereum node (Ganache).
*   Loading of client and robot accounts, showing their addresses and initial balances.
*   Loading of the Escrow contract.
*   **Order Creation:** Logs showing the client creating a new delivery order, the transaction details, and the updated order status from the smart contract.
*   **Pickup Confirmation:** Logs showing the robot confirming the pickup, transaction details, and the updated order status.
*   **Delivery Confirmation & Payment Release:** Logs showing the client confirming delivery, transaction details, the updated order status, and the payment being released to the robot.
*   Final balances for the client and robot accounts, illustrating the transfer of funds (minus gas costs).
*   Details of the order fetched from the smart contract at each step.

## Demonstration Points

This POC aims to demonstrate:

*   **Smart contract for escrow of payments:** The `Escrow.sol` contract handles holding and releasing funds based on delivery confirmations.
*   **Creation of crypto wallet for the robot:** While the script doesn't create a new wallet on the fly, it loads a pre-existing robot account from Ganache using its private key, simulating the robot's wallet.
*   **Verification of transactions and automatic liberation of funds:** The script's output will show transaction hashes and receipts. The final step of `confirm_delivery_and_release` triggers the smart contract to automatically transfer the escrowed amount to the robot's account. Balance changes will reflect this.
