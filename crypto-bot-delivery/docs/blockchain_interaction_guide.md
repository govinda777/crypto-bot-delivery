# Blockchain Interaction Guide: `blockchain_interface.py`

## 1. Overview

The `blockchain_interface.py` script is a Python module designed to facilitate interaction with the `Escrow.sol` smart contract from a Python environment. It serves as a bridge between the off-chain components of the Crypto Bot Delivery system and the on-chain Ethereum smart contract.

Its primary roles include:
- **Order Management:** Creating new delivery orders and submitting them to the `Escrow.sol` contract.
- **Delivery Process Steps:** Confirming critical steps in the delivery process, such as pickup by the robot and final delivery confirmation by the client.
- **Wallet Interaction:** Managing Ethereum accounts (simulating client and robot wallets) to sign and send transactions to the smart contract. It does not create new wallets on-the-fly during typical operation but loads them from provided private keys.

This script is crucial for automating and verifying the payment and delivery lifecycle within the system.

## 2. Prerequisites

Before using `blockchain_interface.py`, ensure the following are met:

*   **Python 3.10+:** The script is written using modern Python features.
*   **`web3.py` library:** This is the primary library used for interacting with the Ethereum blockchain. Install it via pip:
    ```bash
    pip install web3
    ```
*   **Access to a running Ethereum Node:** The script needs to connect to an Ethereum node. This can be:
    *   A local testnet like Ganache (recommended for development/testing).
    *   A public testnet node (e.g., Sepolia, Goerli).
    *   An Ethereum mainnet node (for production).
*   **Deployed `Escrow.sol` Contract:** The `Escrow.sol` smart contract must be compiled and deployed on the target Ethereum network.
*   **Contract ABI (`Escrow.json`):** The Application Binary Interface (ABI) of the deployed `Escrow.sol` contract is required. This JSON file allows `web3.py` to understand how to interact with the contract's functions and events.

## 3. Setup & Configuration

*   **Script Location:** The script is located at `crypto-bot-delivery/src/delivery_crypto/blockchain_interface.py`.

*   **Environment Variables:** The script uses environment variables for configuration. These should be set in your terminal session or through a `.env` file management system before running the script.

    *   **`ETH_NODE_URL`**:
        *   **Description:** The HTTP(S) or WebSocket URL of the Ethereum node to connect to.
        *   **Default:** `"http://127.0.0.1:7545"` (standard Ganache local RPC server URL).
        *   **Example:** `export ETH_NODE_URL="http://localhost:7545"`

    *   **`ESCROW_CONTRACT_ADDRESS`**:
        *   **Description:** The hexadecimal address of the deployed `Escrow.sol` smart contract on the Ethereum network.
        *   **Default:** `"0x0000000000000000000000000000000000000000"` (a placeholder; **must be updated**).
        *   **Example:** `export ESCROW_CONTRACT_ADDRESS="0x123Abc..."`

    *   **`CONTRACT_ABI_PATH`**:
        *   **Description:** The file path to the `Escrow.json` ABI file. The path is resolved relative to the `blockchain_interface.py` script's location if not absolute.
        *   **Default:** `"crypto-bot-delivery/smart_contracts/Escrow.json"` (This means the script expects it at `/app/crypto-bot-delivery/smart_contracts/Escrow.json` when run from `/app/src/delivery_crypto/`).
        *   **Note:** The script attempts to construct the full path: `os.path.join(os.path.dirname(__file__), '..', '..', CONTRACT_ABI_PATH)`.

    *   **`CLIENT_PRIVATE_KEY`**:
        *   **Description:** The private key (hexadecimal string, typically prefixed with `0x`) for the Ethereum account acting as the "Client". This account initiates orders and confirms deliveries.
        *   **Default:** `"0xaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"` (a placeholder; **must be updated** with a real private key from your test environment like Ganache).
        *   **WARNING:** Handle private keys with extreme care. For production, use secure key management solutions, not environment variables.

    *   **`ROBOT_PRIVATE_KEY`**:
        *   **Description:** The private key (hexadecimal string) for the Ethereum account acting as the "Robot" or "Delivery Provider". This account confirms pickups.
        *   **Default:** `"0xbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"` (a placeholder; **must be updated**).
        *   **WARNING:** Same security considerations as `CLIENT_PRIVATE_KEY`.

*   **ABI File:**
    *   The `Escrow.json` ABI file is crucial. The script expects this file to be present at the path specified by `CONTRACT_ABI_PATH`. By default, this is `crypto-bot-delivery/smart_contracts/Escrow.json` (relative to the project root). Ensure this file exists and is the correct ABI for your deployed `Escrow.sol` contract.

## 4. Core Functionalities & Key Functions

### Connection
*   **`connect_to_node(node_url: str = ETHEREUM_NODE_URL) -> Web3 | None`**:
    *   Establishes a connection to an Ethereum node using the provided `node_url`.
    *   It uses `Web3.HTTPProvider` for connection.
    *   Checks if the connection is successful using `w3.is_connected()`.
    *   Injects `geth_poa_middleware` which is often required for interacting with Proof-of-Authority (PoA) chains like Ganache.
    *   Returns a `Web3` instance on success, or `None` on failure.

### Contract Loading
*   **`load_contract(w3: Web3, abi_path: str, contract_address: str)`**:
    *   Loads the `Escrow` smart contract into a Python object that can be interacted with.
    *   It reads the ABI from the JSON file specified by `abi_path`.
    *   It uses `w3.to_checksum_address()` to validate and format the `contract_address`.
    *   Returns a `Contract` object from `web3.py`.
    *   Includes error handling for file not found or invalid address.

### Account Management
*   **`create_or_load_account(w3: Web3, private_key: str | None = None)`**:
    *   If a `private_key` is provided, it loads the corresponding Ethereum account using `w3.eth.account.from_key()`.
    *   If no `private_key` is provided (not the typical usage in this script's main flow), it can create a new account (primarily for testing/dev purposes).
    *   Returns an `Account` object.
    *   **Security Note:** The script emphasizes that storing and using private keys directly from environment variables or code is insecure for production. This method is suitable for local development with test accounts (e.g., from Ganache).

### Hashing Utility
*   **`generate_secret_hash(w3: Web3, secret: str) -> bytes`**:
    *   A utility function to compute the Keccak256 hash of a given `secret` string.
    *   This matches the hashing `keccak256(abi.encodePacked(secret_string))` done within the `Escrow.sol` smart contract for verifying pickup and delivery secrets.
    *   Uses `w3.keccak(text=secret)`.

### Transaction Handling
*   **`build_and_send_tx(w3: Web3, contract_function, sender_account, gas_estimate_multiplier=1.2)`**:
    *   A crucial helper function that encapsulates the common steps for sending a transaction to the blockchain:
        1.  **Nonce:** Fetches the correct nonce for the `sender_account` using `w3.eth.get_transaction_count()`.
        2.  **Gas Estimation:** Estimates the gas required for the `contract_function` using `contract_function.estimate_gas()`.
        3.  **Transaction Parameters:** Builds a dictionary of transaction parameters (`from`, `nonce`, `gas`, `value` if applicable).
        4.  **Build Transaction:** Uses `contract_function.build_transaction()` to create the raw transaction.
        5.  **Sign Transaction:** Signs the transaction using the `sender_account.key`.
        6.  **Send Transaction:** Sends the signed transaction using `w3.eth.send_raw_transaction()`.
        7.  **Wait for Receipt:** Waits for the transaction to be mined using `w3.eth.wait_for_transaction_receipt()`, with a timeout.
    *   Returns the transaction receipt if successful and status is 1, otherwise `None`.
    *   Includes basic error logging.

### Smart Contract Interaction Functions

These functions directly call methods on the loaded `Escrow` smart contract. They typically use `build_and_send_tx` for state-changing calls.

*   **`create_delivery_order(w3: Web3, contract, order_id: str, delivery_provider_address: str, amount_wei: int, pickup_secret: str, delivery_secret: str, client_account)`**:
    *   **Parameters:** Web3 instance, contract object, order ID (string, converted to `bytes32`), delivery provider's address, amount in Wei, pickup secret string, delivery secret string, and the client's account object.
    *   **Action:** Calls the `createOrder` function on the `Escrow.sol` contract.
    *   It first generates hashes for the `pickup_secret` and `delivery_secret`.
    *   The `amount_wei` is sent as `msg.value` with the transaction.
    *   Returns the transaction receipt.

*   **`confirm_pickup(w3: Web3, contract, order_id: str, pickup_secret: str, delivery_provider_account)`**:
    *   **Parameters:** Web3 instance, contract object, order ID, the plaintext `pickup_secret`, and the delivery provider's account object.
    *   **Action:** Calls the `confirmPickup` function on the `Escrow.sol` contract.
    *   Returns the transaction receipt.

*   **`confirm_delivery_and_release(w3: Web3, contract, order_id: str, delivery_secret: str, client_account)`**:
    *   **Parameters:** Web3 instance, contract object, order ID, the plaintext `delivery_secret`, and the client's account object.
    *   **Action:** Calls the `confirmDelivery` function on the `Escrow.sol` contract. This function, upon successful verification of the secret by the contract, also triggers the release of funds to the delivery provider.
    *   Returns the transaction receipt.

*   **`get_order_details(w3: Web3, contract, order_id: str)`**:
    *   **Parameters:** Web3 instance, contract object, order ID.
    *   **Action:** Calls the public `orders` mapping (getter function) on the `Escrow.sol` contract to fetch the details of a specific order.
    *   Prints the fetched order details to the console.
    *   Returns the raw order data tuple from the contract.

## 5. Running the Script

1.  **Ensure Prerequisites and Setup:**
    *   Ganache (or other Ethereum node) is running.
    *   `Escrow.sol` is deployed, and you have its address.
    *   `Escrow.json` ABI file is in the correct location.
    *   All required environment variables (`ETH_NODE_URL`, `ESCROW_CONTRACT_ADDRESS`, `CLIENT_PRIVATE_KEY`, `ROBOT_PRIVATE_KEY`) are set in your current terminal session.

2.  **Navigate to Script Directory:**
    ```bash
    cd crypto-bot-delivery/src/delivery_crypto/
    ```

3.  **Execute the Script:**
    ```bash
    python blockchain_interface.py
    ```

*   **Example Flow (`if __name__ == '__main__':`)**:
    The script includes a main block that demonstrates a typical delivery workflow:
    1.  Connects to the Ethereum node.
    2.  Loads the client and robot accounts from the configured private keys.
    3.  Prints initial balances of these accounts.
    4.  Loads the `Escrow` contract.
    5.  **Client creates an order:** Calls `create_delivery_order()` with sample data.
    6.  Fetches and prints order details.
    7.  **Robot confirms pickup:** Calls `confirm_pickup()`.
    8.  Fetches and prints order details.
    9.  **Client confirms delivery:** Calls `confirm_delivery_and_release()`.
    10. Fetches and prints final order details.
    11. Prints final balances of client and robot accounts to show the fund transfer (minus gas costs).

## 6. Error Handling

The script implements basic error handling:
*   **Connection Issues:** `connect_to_node()` prints an error message if connection fails.
*   **Contract Loading:** `load_contract()` prints errors if the ABI file is not found or the contract address is invalid.
*   **Transaction Failures:** `build_and_send_tx()` checks the transaction receipt's `status`. If `status` is 0 (failed), it prints an error. It also has a general `try-except` block for other exceptions during transaction processing (e.g., gas estimation errors, reverts from the contract).
*   The main interaction functions (`create_delivery_order`, etc.) also have `try-except` blocks to catch and print errors.

For more robust applications, error handling would need to be more sophisticated, potentially including retries, more specific error parsing (especially for contract reverts), and integration with a logging framework.

## 7. Customization

The `blockchain_interface.py` script can be adapted and integrated into larger applications:

*   **As a Library:** Import its functions into other Python modules to programmatically interact with the `Escrow` contract. The `if __name__ == '__main__':` block would not run when imported.
*   **Configuration:** Instead of environment variables, configuration could be managed via configuration files (e.g., YAML, INI) or a dedicated configuration management system.
*   **Wallet Management:** For production systems, private key management should be handled by more secure solutions like Hardware Security Modules (HSMs), dedicated wallet services (e.g., MetaMask for user-initiated actions, or server-side wallet solutions), rather than direct key input.
*   **Error Handling and Logging:** Enhance error handling and integrate a robust logging library (e.g., Python's `logging` module) for better monitoring and debugging.
*   **Asynchronous Operations:** For applications requiring high concurrency (e.g., a web server handling multiple requests), consider adapting the functions to be asynchronous using `async` and `await` with `Web3.py`'s async capabilities.
*   **Event Listening:** The script currently focuses on sending transactions. It could be extended to listen for events emitted by the `Escrow` contract for real-time updates.
