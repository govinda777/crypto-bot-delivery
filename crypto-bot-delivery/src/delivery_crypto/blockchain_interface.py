# blockchain_interface.py

import json
import os
import time
from web3 import Web3
from web3.middleware import geth_poa_middleware  # For PoA chains like Ganache

# --- Configuration ---
# For real deployments, consider environment variables or a secure config management system.
ETHEREUM_NODE_URL = os.environ.get(
    "ETH_NODE_URL", "http://127.0.0.1:7545"
)  # Default to Ganache
CONTRACT_ABI_PATH = os.environ.get(
    "CONTRACT_ABI_PATH", "crypto-bot-delivery/smart_contracts/Escrow.json"
)  # Adjusted path
# Placeholder: Replace with actual deployed contract address
DEPLOYED_CONTRACT_ADDRESS = os.environ.get(
    "ESCROW_CONTRACT_ADDRESS", "0x0000000000000000000000000000000000000000"
)

# WARNING: Storing private keys directly in code or environment variables is insecure for production.
# Use a hardware wallet, keystore file, or a dedicated secrets management service.
CLIENT_PRIVATE_KEY = os.environ.get(
    "CLIENT_PRIVATE_KEY",
    "0xaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
)  # Replace with a real private key from Ganache
ROBOT_PRIVATE_KEY = os.environ.get(
    "ROBOT_PRIVATE_KEY",
    "0xbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
)  # Replace with another real private key from Ganache


# --- Web3 Connection ---
def connect_to_node(node_url: str = ETHEREUM_NODE_URL) -> Web3 | None:
    """Connects to an Ethereum node and returns a Web3 instance."""
    try:
        w3 = Web3(Web3.HTTPProvider(node_url))
        if not w3.is_connected():
            print(f"Failed to connect to Ethereum node at {node_url}")
            return None
        # Inject PoA middleware if connected to Ganache or similar PoA chain
        # This is often needed for local testnets like Ganache.
        w3.middleware_onion.inject(geth_poa_middleware, layer=0)
        print(f"Successfully connected to Ethereum node: {node_url}")
        return w3
    except Exception as e:
        print(f"Error connecting to Ethereum node: {e}")
        return None


# --- Contract Loading ---
def load_contract(w3: Web3, abi_path: str, contract_address: str):
    """Loads a smart contract using its ABI and address."""
    try:
        # Adjust ABI path to be relative to the project root if necessary
        # Assuming this script is in crypto-bot-delivery/src/delivery_crypto/
        # and ABI is in crypto-bot-delivery/smart_contracts/
        corrected_abi_path = os.path.join(
            os.path.dirname(__file__), "..", "..", abi_path
        )

        with open(corrected_abi_path, "r") as f:
            abi = json.load(f)

        # Validate contract_address (basic checksum)
        if not w3.is_address(contract_address):
            print(
                f"Invalid contract address: {contract_address}. Not a valid Ethereum address."
            )
            return None

        contract_address = w3.to_checksum_address(contract_address)
        contract = w3.eth.contract(address=contract_address, abi=abi)
        print(f"Contract loaded successfully at address: {contract_address}")
        return contract
    except FileNotFoundError:
        print(
            f"ABI file not found at {corrected_abi_path}. Make sure 'Escrow.json' exists."
        )
        print(f"Current working directory: {os.getcwd()}")
        print(f"Attempted ABI path: {corrected_abi_path}")
        # Try listing files in smart_contracts to help debug
        smart_contracts_dir = os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "crypto-bot-delivery",
            "smart_contracts",
        )
        try:
            print(
                f"Contents of {smart_contracts_dir}: {os.listdir(smart_contracts_dir)}"
            )
        except FileNotFoundError:
            print(f"Could not list contents of {smart_contracts_dir}")

        # Also check the provided CONTRACT_ABI_PATH directly
        try:
            with open(CONTRACT_ABI_PATH, "r") as f:
                pass  # File exists
            print(
                f"ABI file found at raw CONTRACT_ABI_PATH: {CONTRACT_ABI_PATH}, but path correction might be failing."
            )
        except FileNotFoundError:
            print(
                f"ABI file also not found at raw CONTRACT_ABI_PATH: {CONTRACT_ABI_PATH}"
            )

        return None
    except Exception as e:
        print(f"Error loading contract: {e}")
        return None


# --- Wallet Management ---
def create_or_load_account(w3: Web3, private_key: str | None = None):
    """
    Creates a new Ethereum account or loads an existing one from a private key.
    WARNING: For POC/testing only. Managing private keys this way is insecure.
    """
    if private_key:
        try:
            account = w3.eth.account.from_key(private_key)
            print(f"Loaded account: {account.address}")
            return account
        except Exception as e:
            print(f"Error loading account from private key: {e}")
            return None
    else:
        account = w3.eth.account.create()
        print(f"Created new account: {account.address}")
        print(f"Private Key (SAVE THIS SECURELY): {w3.to_hex(account.key)}")
        return account


# --- Helper for Hashes ---
def generate_secret_hash(w3: Web3, secret: str) -> bytes:
    """Computes keccak256(abi.encodePacked(secret_string))."""
    return w3.keccak(text=secret)  # web3.py's keccak directly takes text


# --- Core Interaction Functions ---
def build_and_send_tx(
    w3: Web3, contract_function, sender_account, gas_estimate_multiplier=1.2
):
    """Helper to build, sign, send a transaction and wait for receipt."""
    try:
        tx_params = {
            "from": sender_account.address,
            "nonce": w3.eth.get_transaction_count(sender_account.address),
            # Gas estimation can be tricky. For local testnets, a higher static gas limit might be easier.
            # 'gas': estimated_gas, # Let web3.py estimate, or set manually
            # 'gasPrice': w3.eth.gas_price # Let web3.py set, or set manually
        }

        # Estimate gas
        estimated_gas = contract_function.estimate_gas(
            {"from": sender_account.address, "value": tx_params.get("value", 0)}
        )
        tx_params["gas"] = int(estimated_gas * gas_estimate_multiplier)

        transaction = contract_function.build_transaction(tx_params)
        signed_tx = w3.eth.account.sign_transaction(
            transaction, private_key=sender_account.key
        )
        tx_hash = w3.eth.send_raw_transaction(signed_tx.rawTransaction)
        print(f"Transaction sent with hash: {w3.to_hex(tx_hash)}")

        # Wait for transaction receipt
        tx_receipt = w3.eth.wait_for_transaction_receipt(
            tx_hash, timeout=120
        )  # 120s timeout

        if tx_receipt.status == 1:
            print(
                f"Transaction successful! Block: {tx_receipt.blockNumber}, Gas used: {tx_receipt.gasUsed}"
            )
            return tx_receipt
        else:
            print(f"Transaction failed! Receipt: {tx_receipt}")
            return None
    except Exception as e:
        print(f"Error during transaction: {e}")
        # If it's a revert, the error message might contain details.
        # This part can be complex as error messages from contracts are not always straightforward.
        if hasattr(e, "message") and "revert" in e.message:
            print(
                f"Contract execution reverted. Reason might be in message: {e.message}"
            )
        return None


def create_delivery_order(
    w3: Web3,
    contract,
    order_id: str,
    delivery_provider_address: str,
    amount_wei: int,
    pickup_secret: str,
    delivery_secret: str,
    client_account,
):
    """Creates a new delivery order on the smart contract."""
    try:
        order_id_bytes32 = Web3.to_bytes(text=order_id).ljust(
            32, b"\0"
        )  # Ensure 32 bytes
        pickup_hash = generate_secret_hash(w3, pickup_secret)
        delivery_hash = generate_secret_hash(w3, delivery_secret)

        print(
            f"Creating order '{order_id}' for {delivery_provider_address} with amount {amount_wei} wei."
        )
        print(f"Pickup Hash: {pickup_hash.hex()}, Delivery Hash: {delivery_hash.hex()}")

        contract_function = contract.functions.createOrder(
            order_id_bytes32,
            w3.to_checksum_address(delivery_provider_address),
            pickup_hash,
            delivery_hash,
        )
        # Add value to tx_params for payable function
        tx_params = {
            "from": client_account.address,
            "nonce": w3.eth.get_transaction_count(client_account.address),
            "value": amount_wei,
        }
        estimated_gas = contract_function.estimate_gas(tx_params)
        tx_params["gas"] = int(estimated_gas * 1.2)

        transaction = contract_function.build_transaction(tx_params)
        signed_tx = w3.eth.account.sign_transaction(
            transaction, private_key=client_account.key
        )
        tx_hash = w3.eth.send_raw_transaction(signed_tx.rawTransaction)
        print(f"Create order transaction sent with hash: {w3.to_hex(tx_hash)}")

        tx_receipt = w3.eth.wait_for_transaction_receipt(tx_hash, timeout=120)

        if tx_receipt.status == 1:
            print(
                f"Order '{order_id}' created successfully. Block: {tx_receipt.blockNumber}"
            )
            return tx_receipt
        else:
            print(f"Failed to create order '{order_id}'. Receipt: {tx_receipt}")
            return None
    except Exception as e:
        print(f"Error creating delivery order '{order_id}': {e}")
        return None


def confirm_pickup(
    w3: Web3, contract, order_id: str, pickup_secret: str, delivery_provider_account
):
    """Confirms pickup of an order."""
    try:
        order_id_bytes32 = Web3.to_bytes(text=order_id).ljust(32, b"\0")
        print(
            f"Confirming pickup for order '{order_id}' with secret '{pickup_secret}' by {delivery_provider_account.address}"
        )

        contract_function = contract.functions.confirmPickup(
            order_id_bytes32, pickup_secret
        )
        tx_receipt = build_and_send_tx(w3, contract_function, delivery_provider_account)

        if tx_receipt:
            print(f"Pickup confirmed for order '{order_id}'.")
        else:
            print(f"Failed to confirm pickup for order '{order_id}'.")
        return tx_receipt
    except Exception as e:
        print(f"Error confirming pickup for order '{order_id}': {e}")
        return None


def confirm_delivery_and_release(
    w3: Web3, contract, order_id: str, delivery_secret: str, client_account
):
    """Confirms delivery of an order and releases funds."""
    try:
        order_id_bytes32 = Web3.to_bytes(text=order_id).ljust(32, b"\0")
        print(
            f"Confirming delivery for order '{order_id}' with secret '{delivery_secret}' by {client_account.address}"
        )

        contract_function = contract.functions.confirmDelivery(
            order_id_bytes32, delivery_secret
        )
        tx_receipt = build_and_send_tx(w3, contract_function, client_account)

        if tx_receipt:
            print(f"Delivery confirmed and funds released for order '{order_id}'.")
        else:
            print(f"Failed to confirm delivery for order '{order_id}'.")
        return tx_receipt
    except Exception as e:
        print(f"Error confirming delivery for order '{order_id}': {e}")
        return None


def get_order_details(w3: Web3, contract, order_id: str):
    """Fetches and prints details of a specific order."""
    try:
        order_id_bytes32 = Web3.to_bytes(text=order_id).ljust(32, b"\0")
        print(f"\nFetching details for order: {order_id} ({order_id_bytes32.hex()})")

        order_data = contract.functions.orders(order_id_bytes32).call()

        # Order struct: address client, address deliveryProvider, uint256 amount, Status status, bytes32 ph, bytes32 dh
        status_map = {
            0: "Created",
            1: "PickedUp",
            2: "Delivered",
            3: "Cancelled",
            4: "Refunded",
        }

        print(f"  Client: {order_data[0]}")
        print(f"  Delivery Provider: {order_data[1]}")
        print(f"  Amount (Wei): {order_data[2]}")
        print(f"  Status: {status_map.get(order_data[3], 'Unknown')}")
        print(f"  Pickup Hash: {order_data[4].hex()}")
        print(f"  Delivery Hash: {order_data[5].hex()}")
        return order_data
    except Exception as e:
        print(f"Error fetching order details for '{order_id}': {e}")
        # This can happen if the order_id doesn't exist or is invalid.
        # The contract will likely revert, and `call()` will raise an exception.
        return None


# --- Example Usage ---
if __name__ == "__main__":
    print("--- Blockchain Interface Script Start ---")

    # 1. Connect to Ethereum node
    w3 = connect_to_node()
    if not w3:
        print("Exiting: Could not connect to Ethereum node.")
        exit()

    # Set default gas payer if needed (Ganache often pre-funds accounts)
    # w3.eth.default_account = w3.eth.accounts[0]
    # Better to specify 'from' in transactions or use signed transactions with specific accounts.

    # 2. Create/Load accounts
    # WARNING: These are example private keys. Replace them for any real use.
    # Ensure these accounts have ETH on your testnet (e.g., Ganache).
    client_account = create_or_load_account(w3, CLIENT_PRIVATE_KEY)
    robot_account = create_or_load_account(w3, ROBOT_PRIVATE_KEY)

    if not client_account or not robot_account:
        print("Exiting: Could not create or load accounts.")
        exit()

    print(f"\nClient Account: {client_account.address}")
    print(f"Robot Account: {robot_account.address}")

    initial_client_balance = w3.eth.get_balance(client_account.address)
    initial_robot_balance = w3.eth.get_balance(robot_account.address)
    print(f"Initial Client Balance: {w3.from_wei(initial_client_balance, 'ether')} ETH")
    print(f"Initial Robot Balance: {w3.from_wei(initial_robot_balance, 'ether')} ETH")

    # 3. Load the Escrow contract
    # IMPORTANT: Replace DEPLOYED_CONTRACT_ADDRESS with the actual address after deploying Escrow.sol
    # For this script, we assume it's deployed and the address is known.
    # If DEPLOYED_CONTRACT_ADDRESS is the placeholder 0x0...0, this will fail.
    # You would typically deploy the contract first (e.g., using Remix, Truffle, or another script)
    # and then update this address.

    # Check if ABI exists before trying to load
    # This is a simplified check, load_contract has more robust logging
    abi_full_path = os.path.join(
        os.path.dirname(__file__), "..", "..", CONTRACT_ABI_PATH
    )
    if not os.path.exists(abi_full_path):
        print(f"\nERROR: ABI file not found at {abi_full_path}")
        print(
            "Please ensure 'Escrow.json' exists in 'crypto-bot-delivery/smart_contracts/'."
        )
        print(
            "You might need to compile your Solidity contract and place the ABI JSON there."
        )
        print("--- Script End (due to missing ABI) ---")
        exit()

    if DEPLOYED_CONTRACT_ADDRESS == "0x0000000000000000000000000000000000000000":
        print("\nWARNING: DEPLOYED_CONTRACT_ADDRESS is a placeholder.")
        print(
            "Please deploy your Escrow.sol contract and update this address in the script or via environment variable."
        )
        print("Skipping contract interactions.")
        print("--- Blockchain Interface Script End ---")
        exit()

    escrow_contract = load_contract(w3, CONTRACT_ABI_PATH, DEPLOYED_CONTRACT_ADDRESS)
    if not escrow_contract:
        print(
            "Exiting: Could not load Escrow contract. Ensure ABI is correct and contract is deployed at the address."
        )
        exit()

    # 4. Simulate a delivery flow
    order_id = "ORDER123"
    amount_ether = 0.01  # Example amount in Ether
    amount_wei = w3.to_wei(amount_ether, "ether")

    pickup_secret_phrase = "RobotPickupSecret789"
    delivery_secret_phrase = "ClientDeliverySecretXYZ"

    print(f"\n--- Simulating Delivery Flow for Order ID: {order_id} ---")

    # Step 1: Client creates an order
    print("\nStep 1: Client creates a delivery order...")
    create_tx_receipt = create_delivery_order(
        w3,
        escrow_contract,
        order_id,
        robot_account.address,
        amount_wei,
        pickup_secret_phrase,
        delivery_secret_phrase,
        client_account,
    )
    if not create_tx_receipt:
        print("Failed to create order. Aborting simulation.")
        exit()

    get_order_details(w3, escrow_contract, order_id)
    time.sleep(1)  # Give node some time

    # Step 2: Robot (delivery provider) confirms pickup
    print("\nStep 2: Robot confirms pickup...")
    pickup_tx_receipt = confirm_pickup(
        w3, escrow_contract, order_id, pickup_secret_phrase, robot_account
    )
    if not pickup_tx_receipt:
        print("Failed to confirm pickup. Aborting simulation.")
        exit()

    get_order_details(w3, escrow_contract, order_id)
    time.sleep(1)

    # Step 3: Client confirms delivery
    print("\nStep 3: Client confirms delivery and releases payment...")
    delivery_tx_receipt = confirm_delivery_and_release(
        w3, escrow_contract, order_id, delivery_secret_phrase, client_account
    )
    if not delivery_tx_receipt:
        print("Failed to confirm delivery. Aborting simulation.")
        exit()

    get_order_details(w3, escrow_contract, order_id)
    time.sleep(1)

    # 5. Check balances after the flow
    print("\n--- Balances After Delivery Flow ---")
    final_client_balance = w3.eth.get_balance(client_account.address)
    final_robot_balance = w3.eth.get_balance(robot_account.address)

    print(f"Initial Client Balance: {w3.from_wei(initial_client_balance, 'ether')} ETH")
    print(
        f"Final Client Balance:   {w3.from_wei(final_client_balance, 'ether')} ETH (includes gas costs)"
    )

    print(f"Initial Robot Balance: {w3.from_wei(initial_robot_balance, 'ether')} ETH")
    print(
        f"Final Robot Balance:   {w3.from_wei(final_robot_balance, 'ether')} ETH (received payment, paid gas for pickup)"
    )

    # Note: Exact balance changes will also depend on gas costs for transactions.
    # The client pays for createOrder and confirmDelivery.
    # The robot pays for confirmPickup.

    print("\n--- Blockchain Interface Script End ---")
