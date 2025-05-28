import pytest
from pytest_bdd import scenarios, given, when, then, parsers
from web3 import Web3
from web3.middleware import geth_poa_middleware # For Ganache
import json
import os
from decimal import Decimal # For ETH calculations

# Assuming blockchain_interface.py is in the python path due to editable install
# and pyproject.toml which configures pythonpath for pytest.
# from delivery_crypto.blockchain_interface import generate_secret_hash # Not used directly in GIVEN steps yet
# from delivery_crypto.blockchain_interface import build_and_send_tx # Might be used in WHEN steps

# --- Constants ---
GANACHE_URL = os.environ.get("ETH_NODE_URL", "http://127.0.0.1:7545") # Default to Ganache
GAS_LIMIT = 4700000 # Example gas limit, adjust as needed for contract deployment and transactions

# --- Paths ---
# Calculate BASE_DIR relative to this file's location
# This file is in: crypto-bot-delivery/src/delivery_crypto/tests/step_defs/test_delivery_lifecycle_steps.py
# BASE_DIR should be: crypto-bot-delivery/
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))
CONTRACT_ARTIFACTS_DIR = os.path.join(BASE_DIR, "smart_contracts")
CONTRACT_ABI_PATH = os.path.join(CONTRACT_ARTIFACTS_DIR, "Escrow.json")
CONTRACT_BYTECODE_PATH = os.path.join(CONTRACT_ARTIFACTS_DIR, "Escrow.bin")

# --- Load Scenarios ---
# Assuming the feature file is in ../features/ from this file's location
scenarios('../features/delivery_lifecycle.feature')

# --- Fixtures ---

@pytest.fixture(scope='session')
def web3_connection():
    w3 = Web3(Web3.HTTPProvider(GANACHE_URL))
    if not w3.is_connected():
        # Try one more time after a short delay if Ganache is slow to start
        import time
        time.sleep(2)
        w3 = Web3(Web3.HTTPProvider(GANACHE_URL))
        if not w3.is_connected():
            raise ConnectionError(f"Failed to connect to Ganache at {GANACHE_URL}")
            
    w3.middleware_onion.inject(geth_poa_middleware, layer=0) # For PoA chains like Ganache
    
    # Ensure there are accounts available
    if not w3.eth.accounts:
        pytest.skip("No accounts found in Ganache. Ensure Ganache is running and has accounts.")
        
    w3.eth.default_account = w3.eth.accounts[0] # Set a default account
    return w3

@pytest.fixture(scope='session')
def escrow_contract_abi():
    if not os.path.exists(CONTRACT_ABI_PATH):
        pytest.skip(f"Contract ABI not found at {CONTRACT_ABI_PATH}. Run compilation first.")
    with open(CONTRACT_ABI_PATH, 'r') as f:
        abi = json.load(f)
    return abi

@pytest.fixture(scope='session')
def escrow_contract_bytecode():
    if not os.path.exists(CONTRACT_BYTECODE_PATH):
        pytest.skip(f"Contract bytecode not found at {CONTRACT_BYTECODE_PATH}. Run compilation first (ensure .bin output).")
    with open(CONTRACT_BYTECODE_PATH, 'r') as f:
        bytecode = f.read().strip()
    if not bytecode.startswith("0x"): # web3.py expects bytecode to be hex string
         bytecode = "0x" + bytecode
    return bytecode

@pytest.fixture(scope='function') # Re-deploy for each test function to ensure isolation
def escrow_contract(web3_connection, escrow_contract_abi, escrow_contract_bytecode):
    w3 = web3_connection
    deployer_account = w3.eth.accounts[0] # Assume first Ganache account is deployer

    Escrow = w3.eth.contract(abi=escrow_contract_abi, bytecode=escrow_contract_bytecode)
    
    # Estimate gas for deployment (optional, but good practice)
    # constructor_gas = Escrow.constructor().estimate_gas({'from': deployer_account})
    # print(f"Estimated gas for deployment: {constructor_gas}")

    try:
        tx_hash = Escrow.constructor().transact({'from': deployer_account, 'gas': GAS_LIMIT})
        tx_receipt = w3.eth.wait_for_transaction_receipt(tx_hash, timeout=120) # Increased timeout
    except Exception as e:
        pytest.fail(f"Contract deployment transaction failed: {e}")
    
    if not tx_receipt or not tx_receipt.contractAddress:
        pytest.fail("Contract deployment failed: No contract address in receipt or receipt is None.")
        
    contract_instance = w3.eth.contract(address=tx_receipt.contractAddress, abi=escrow_contract_abi)
    # print(f"Contract deployed at: {contract_instance.address} by {deployer_account}")
    return contract_instance

@pytest.fixture(scope='function')
def accounts(web3_connection):
    w3 = web3_connection
    # Ensure Ganache has at least 3 accounts for these roles
    if len(w3.eth.accounts) < 3:
        pytest.skip("Not enough accounts in Ganache for testing (need at least 3). Skipping.")
    
    # Assign fixed roles. These accounts are expected to be pre-funded by Ganache.
    # Private keys are not directly handled here as transactions will be sent via w3.eth.send_transaction
    # which can use w3.eth.default_account or explicitly set 'from' if Ganache node handles unlocked accounts.
    # For sending signed transactions (if Ganache accounts were locked), we'd need private keys.
    # The blockchain_interface.py and its tests handle private keys for more general use.
    # Here, we assume Ganache accounts are unlocked for direct use via their addresses.
    account_map = {
        "deployer": w3.eth.accounts[0],
        "Alice": w3.eth.accounts[1],    # Client
        "RoboDeliverer": w3.eth.accounts[2] # Robot
    }
    # print(f"Accounts used in test: Deployer: {account_map['deployer']}, Alice: {account_map['Alice']}, Robo: {account_map['RoboDeliverer']}")
    return account_map

@pytest.fixture(scope='function')
def scenario_context():
    # Shared context for steps within a scenario
    # print("Scenario context initialized")
    return {}

# --- Given Steps ---

@given('a connection to the blockchain node')
def given_blockchain_connection(web3_connection, scenario_context):
    assert web3_connection.is_connected()
    scenario_context['w3'] = web3_connection 

@given('the Escrow contract is deployed')
def given_escrow_contract_deployed(escrow_contract, scenario_context):
    assert escrow_contract is not None
    assert escrow_contract.address is not None
    scenario_context['escrow_contract'] = escrow_contract
    # print(f"Escrow contract address in context: {escrow_contract.address}")

@given(parsers.parse('a client account "{account_name}" with sufficient funds'))
def given_client_account(accounts, account_name, web3_connection, scenario_context):
    assert account_name in accounts, f"Account '{account_name}' not found in fixture. Available: {list(accounts.keys())}"
    address = accounts[account_name]
    balance = web3_connection.eth.get_balance(address)
    # print(f"Balance for {account_name} ({address}): {Web3.from_wei(balance, 'ether')} ETH")
    # Check for at least 0.5 ETH for "sufficient funds" (covers gas and order amounts in tests)
    assert balance > Web3.to_wei(0.5, 'ether'), f"{account_name} ({address}) does not have sufficient funds (has {Web3.from_wei(balance, 'ether')} ETH)."
    scenario_context.setdefault('users', {})[account_name] = address

@given(parsers.parse('a robot account "{account_name}"'))
def given_robot_account(accounts, account_name, web3_connection, scenario_context):
    assert account_name in accounts, f"Account '{account_name}' not found in fixture. Available: {list(accounts.keys())}"
    address = accounts[account_name]
    # print(f"Robot account {account_name} ({address}) set up.")
    scenario_context.setdefault('users', {})[account_name] = address


@given(parsers.parse('{user_type:S} wants to create a delivery order with ID "{order_id}" for {provider_type:S}'))
def given_user_wants_to_create_order(scenario_context, user_type, order_id, provider_type):
    # This step primarily sets up context for the 'When' step where the order is actually created.
    # It assumes user_type (e.g. "Alice") and provider_type (e.g. "RoboDeliverer") 
    # have been loaded into scenario_context['users'] by previous steps.
    
    # Check if users exist
    if 'users' not in scenario_context or user_type not in scenario_context['users']:
        pytest.fail(f"User '{user_type}' not found in scenario_context. Ensure a prior step defines this user.")
    if 'users' not in scenario_context or provider_type not in scenario_context['users']:
        pytest.fail(f"Provider '{provider_type}' not found in scenario_context. Ensure a prior step defines this provider.")

    scenario_context['current_order_id'] = order_id
    scenario_context['current_order_client_name'] = user_type
    scenario_context['current_order_provider_name'] = provider_type
    # print(f"Order details set in context: ID={order_id}, Client={user_type}, Provider={provider_type}")


@given(parsers.parse('the order amount is "{amount_eth}" ETH'))
def given_order_amount(scenario_context, amount_eth):
    scenario_context['current_order_amount_eth'] = Decimal(amount_eth)
    scenario_context['current_order_amount_wei'] = Web3.to_wei(Decimal(amount_eth), 'ether')
    # print(f"Order amount set in context: {amount_eth} ETH ({scenario_context['current_order_amount_wei']} Wei)")

@given(parsers.parse('the pickup secret is "{pickup_secret}"'))
def given_pickup_secret(scenario_context, pickup_secret):
    scenario_context['current_pickup_secret'] = pickup_secret
    # print(f"Pickup secret set in context: {pickup_secret}")

@given(parsers.parse('the delivery secret is "{delivery_secret}"'))
def given_delivery_secret(scenario_context, delivery_secret):
    scenario_context['current_delivery_secret'] = delivery_secret
    # print(f"Delivery secret set in context: {delivery_secret}")


# --- Helper ---
def get_eth_balance(w3, address):
    return w3.eth.get_balance(address)

# --- Imports for When/Then steps ---
from delivery_crypto.blockchain_interface import generate_secret_hash
from web3.exceptions import ContractLogicError # For expected reverts

# --- When Steps ---

@when(parsers.parse('{user_role} creates the delivery order "{order_id}"'))
def when_creates_order(web3_connection, escrow_contract, scenario_context, accounts, user_role, order_id):
    w3 = web3_connection
    
    client_name = scenario_context['current_order_client_name']
    if user_role != client_name:
        pytest.fail(f"Step user '{user_role}' does not match context client '{client_name}' for creating order '{order_id}'.")

    client_account = scenario_context['users'][client_name]
    robot_account_address = scenario_context['users'][scenario_context['current_order_provider_name']]
    
    if order_id != scenario_context['current_order_id']:
        pytest.fail(f"Order ID in step '{order_id}' does not match context '{scenario_context['current_order_id']}'.")

    amount_wei = scenario_context['current_order_amount_wei']
    pickup_secret = scenario_context['current_pickup_secret']
    delivery_secret = scenario_context['current_delivery_secret']

    pickup_hash = generate_secret_hash(w3, pickup_secret) 
    delivery_hash = generate_secret_hash(w3, delivery_secret)
    
    order_id_bytes32 = order_id.encode('utf-8').ljust(32, b'\0')

    scenario_context[f'initial_contract_balance_for_{order_id}'] = get_eth_balance(w3, escrow_contract.address)
    scenario_context[f'initial_client_balance_for_{order_id}'] = get_eth_balance(w3, client_account)

    try:
        tx_hash = escrow_contract.functions.createOrder(
            order_id_bytes32, robot_account_address, pickup_hash, delivery_hash
        ).transact({'from': client_account, 'value': amount_wei, 'gas': GAS_LIMIT})
        
        receipt = w3.eth.wait_for_transaction_receipt(tx_hash, timeout=120)
        scenario_context[f'last_tx_receipt_for_{order_id}'] = receipt
        scenario_context[f'last_tx_hash_for_{order_id}'] = tx_hash
        if receipt.status == 0: # Store explicit error if transaction reverted
             scenario_context[f'last_tx_error_for_{order_id}'] = "Transaction reverted (status 0)"
    except Exception as e:
        scenario_context[f'last_tx_error_for_{order_id}'] = e


@when(parsers.parse('{user_role} confirms pickup for order "{order_id}" with secret "{secret}"'))
def when_confirms_pickup(web3_connection, escrow_contract, scenario_context, accounts, user_role, order_id, secret):
    w3 = web3_connection
    actor_account = scenario_context['users'][user_role]
    order_id_bytes32 = order_id.encode('utf-8').ljust(32, b'\0')

    try:
        tx_hash = escrow_contract.functions.confirmPickup(order_id_bytes32, secret).transact({
            'from': actor_account, 'gas': GAS_LIMIT
        })
        receipt = w3.eth.wait_for_transaction_receipt(tx_hash, timeout=120)
        scenario_context[f'last_tx_receipt_for_{order_id}'] = receipt
        scenario_context[f'last_tx_hash_for_{order_id}'] = tx_hash
        if receipt.status == 0:
             scenario_context[f'last_tx_error_for_{order_id}'] = "Transaction reverted (status 0)"
    except Exception as e:
        scenario_context[f'last_tx_error_for_{order_id}'] = e


@when(parsers.parse('{user_role} confirms delivery for order "{order_id}" with secret "{secret}"'))
def when_confirms_delivery(web3_connection, escrow_contract, scenario_context, accounts, user_role, order_id, secret):
    w3 = web3_connection
    actor_account = scenario_context['users'][user_role] 
    
    # Ensure current_order_id is set if this step relies on it, or pass specific order context
    # For balance checks, we need the provider associated with THIS order_id.
    # The Given step for "wants to create order" sets current_order_provider_name,
    # which might be overwritten if multiple orders are set up in Background/Given.
    # It's safer if this step can determine the provider from order_id or has it passed.
    # For now, assume current_order_provider_name is correct for the order_id in question.
    provider_name_key = scenario_context.get('current_order_provider_name', 'RoboDeliverer') # Default if not set by specific order context
    if order_id != scenario_context.get('current_order_id'): # If current_order_id is not this order, this is problematic
         # This implies we need a way to map order_id to its provider if not using 'current_order_id'
         # For the current feature file structure, current_order_id is usually the one being acted upon.
         pass

    robot_account_address = scenario_context['users'][provider_name_key]
    order_id_bytes32 = order_id.encode('utf-8').ljust(32, b'\0')

    scenario_context[f'{provider_name_key.lower()}_balance_before_delivery_for_{order_id}'] = get_eth_balance(w3, robot_account_address)
    scenario_context[f'contract_balance_before_delivery_for_{order_id}'] = get_eth_balance(w3, escrow_contract.address)
    scenario_context[f'{user_role.lower()}_balance_before_confirm_delivery_for_{order_id}'] = get_eth_balance(w3, actor_account)

    try:
        tx_hash = escrow_contract.functions.confirmDelivery(order_id_bytes32, secret).transact({
            'from': actor_account, 'gas': GAS_LIMIT
        })
        receipt = w3.eth.wait_for_transaction_receipt(tx_hash, timeout=120)
        scenario_context[f'last_tx_receipt_for_{order_id}'] = receipt
        scenario_context[f'last_tx_hash_for_{order_id}'] = tx_hash
        if receipt.status == 0:
             scenario_context[f'last_tx_error_for_{order_id}'] = "Transaction reverted (status 0)"
    except Exception as e:
        scenario_context[f'last_tx_error_for_{order_id}'] = e


@when(parsers.parse('{user_role} cancels the order "{order_id}"'))
def when_cancels_order(web3_connection, escrow_contract, scenario_context, accounts, user_role, order_id):
    w3 = web3_connection
    actor_account = scenario_context['users'][user_role] 
    order_id_bytes32 = order_id.encode('utf-8').ljust(32, b'\0')

    scenario_context[f'{user_role.lower()}_balance_before_cancel_for_{order_id}'] = get_eth_balance(w3, actor_account)
    scenario_context[f'contract_balance_before_cancel_for_{order_id}'] = get_eth_balance(w3, escrow_contract.address)

    try:
        tx_hash = escrow_contract.functions.cancelOrder(order_id_bytes32).transact({
            'from': actor_account, 'gas': GAS_LIMIT
        })
        receipt = w3.eth.wait_for_transaction_receipt(tx_hash, timeout=120)
        scenario_context[f'last_tx_receipt_for_{order_id}'] = receipt
        scenario_context[f'last_tx_hash_for_{order_id}'] = tx_hash
        if receipt.status == 0:
             scenario_context[f'last_tx_error_for_{order_id}'] = "Transaction reverted (status 0)"
    except Exception as e:
        scenario_context[f'last_tx_error_for_{order_id}'] = e


# --- Then Steps ---

@then(parsers.parse('the order "{order_id}" should be successfully created on the blockchain'))
def then_order_successfully_created(escrow_contract, scenario_context, order_id, accounts):
    if f'last_tx_error_for_{order_id}' in scenario_context:
        pytest.fail(f"Transaction for creating order '{order_id}' failed with: {scenario_context[f'last_tx_error_for_{order_id}']}")

    receipt = scenario_context.get(f'last_tx_receipt_for_{order_id}')
    assert receipt is not None, f"Transaction receipt not found in context for order '{order_id}'"
    assert receipt.status == 1, f"Transaction to create order '{order_id}' failed (receipt status 0)"
    
    order_id_bytes32 = order_id.encode('utf-8').ljust(32, b'\0')
    order_details = escrow_contract.functions.orders(order_id_bytes32).call()
    
    client_name = scenario_context['current_order_client_name'] # Set by Given step for this order
    provider_name = scenario_context['current_order_provider_name'] # Set by Given step for this order

    assert order_details[0] == accounts[client_name], f"Order client mismatch for '{order_id}'."
    assert order_details[1] == accounts[provider_name], f"Order provider mismatch for '{order_id}'."
    assert order_details[2] == scenario_context['current_order_amount_wei'], f"Order amount mismatch for '{order_id}'."
    assert order_details[3] == 0, f"Order status mismatch for '{order_id}'. Expected 0 (Created)."

@then(parsers.parse('the contract balance should increase by "{amount_eth}" ETH'))
def then_contract_balance_increases(web3_connection, escrow_contract, scenario_context, amount_eth):
    w3 = web3_connection
    order_id = scenario_context['current_order_id'] 
    expected_increase_wei = Web3.to_wei(Decimal(amount_eth), 'ether')
    current_contract_balance = get_eth_balance(w3, escrow_contract.address)
    initial_contract_balance = scenario_context[f'initial_contract_balance_for_{order_id}']
    
    actual_increase = current_contract_balance - initial_contract_balance
    assert actual_increase == expected_increase_wei, \
        f"Contract balance increase mismatch for order '{order_id}'. Expected {expected_increase_wei}, got {actual_increase}"

@then(parsers.parse('the status of order "{order_id}" should be "{expected_status_str}"'))
def then_order_status_is(escrow_contract, order_id, expected_status_str):
    order_id_bytes32 = order_id.encode('utf-8').ljust(32, b'\0')
    order_details = escrow_contract.functions.orders(order_id_bytes32).call()
    
    status_map = {"Created": 0, "PickedUp": 1, "Delivered": 2, "Cancelled": 3, "Refunded": 4} 
    expected_status_enum = status_map[expected_status_str]
    actual_status_enum = order_details[3]
    
    actual_status_str = "Unknown"
    for k, v in status_map.items(): # Find string name for actual enum value for better error messages
        if v == actual_status_enum:
            actual_status_str = k
            break
            
    assert actual_status_enum == expected_status_enum, \
        f"Order status for '{order_id}' is {actual_status_enum} ('{actual_status_str}'), expected {expected_status_enum} ('{expected_status_str}')"

@then(parsers.parse('the pickup for order "{order_id}" should be confirmed'))
def then_pickup_confirmed(scenario_context, order_id):
    if f'last_tx_error_for_{order_id}' in scenario_context:
        pytest.fail(f"Transaction for confirming pickup for order '{order_id}' failed with: {scenario_context[f'last_tx_error_for_{order_id}']}")
    receipt = scenario_context.get(f'last_tx_receipt_for_{order_id}')
    assert receipt is not None, f"Transaction receipt not found for pickup confirmation of order '{order_id}'"
    assert receipt.status == 1, f"Pickup confirmation transaction for order '{order_id}' failed (receipt status 0)"

@then(parsers.parse('the delivery for order "{order_id}" should be confirmed'))
def then_delivery_confirmed(scenario_context, order_id):
    if f'last_tx_error_for_{order_id}' in scenario_context:
        pytest.fail(f"Transaction for confirming delivery for order '{order_id}' failed with: {scenario_context[f'last_tx_error_for_{order_id}']}")
    receipt = scenario_context.get(f'last_tx_receipt_for_{order_id}')
    assert receipt is not None, f"Transaction receipt not found for delivery confirmation of order '{order_id}'"
    assert receipt.status == 1, f"Delivery confirmation transaction for order '{order_id}' failed (receipt status 0)"

@then(parsers.parse("{user_role}'s balance should increase by approximately "{amount_eth}" ETH (minus gas)"))
def then_balance_increases_approx(web3_connection, scenario_context, user_role, amount_eth, accounts):
    w3 = web3_connection
    order_id = scenario_context['current_order_id'] 
    user_account_address = scenario_context['users'][user_role]
    expected_increase_wei = Web3.to_wei(Decimal(amount_eth), 'ether')
    
    balance_after = get_eth_balance(w3, user_account_address)
    balance_before_key = f'{user_role.lower()}_balance_before_delivery_for_{order_id}'
    
    if balance_before_key not in scenario_context:
         pytest.fail(f"Balance before key '{balance_before_key}' not found in scenario_context for user '{user_role}' and order '{order_id}'.")
    balance_before = scenario_context[balance_before_key]

    actual_increase_wei = balance_after - balance_before
    
    # This user (RoboDeliverer) did not pay gas for the confirmDelivery transaction.
    assert actual_increase_wei == expected_increase_wei, \
        f"{user_role}'s balance increase mismatch for order '{order_id}'. Expected {expected_increase_wei}, got {actual_increase_wei}."

@then(parsers.parse('the contract balance for order "{order_id}" should be "0"'))
def then_contract_balance_for_order_is_zero(web3_connection, escrow_contract, scenario_context, order_id):
    w3 = web3_connection
    contract_balance_after_action = get_eth_balance(w3, escrow_contract.address)
    
    # Assuming current_order_amount_wei is set for the specific order_id in a Given step
    order_amount_wei = scenario_context['current_order_amount_wei'] 

    if f'contract_balance_before_delivery_for_{order_id}' in scenario_context:
        initial_relevant_balance = scenario_context[f'contract_balance_before_delivery_for_{order_id}']
        expected_change_wei = -order_amount_wei 
    elif f'contract_balance_before_cancel_for_{order_id}' in scenario_context:
        initial_relevant_balance = scenario_context[f'contract_balance_before_cancel_for_{order_id}']
        expected_change_wei = -order_amount_wei
    else:
        pytest.fail(f"Contract balance context (before delivery/cancel for order {order_id}) not properly set up.")

    actual_change_wei = contract_balance_after_action - initial_relevant_balance
    assert actual_change_wei == expected_change_wei, \
        f"Contract balance change mismatch for order '{order_id}'. Expected change {expected_change_wei}, got {actual_change_wei}"

@then(parsers.parse('the order "{order_id}" should be marked as "{expected_status_str}"'))
def then_order_marked_as_status(escrow_contract, scenario_context, order_id, expected_status_str):
    if expected_status_str not in ["FailedStateOrInvalid"]: 
        if f'last_tx_error_for_{order_id}' in scenario_context:
             pytest.fail(f"Transaction for order '{order_id}' to reach status '{expected_status_str}' failed unexpectedly with: {scenario_context[f'last_tx_error_for_{order_id}']}")
        receipt = scenario_context.get(f'last_tx_receipt_for_{order_id}')
        assert receipt is not None, f"Receipt for order '{order_id}' not found when checking for status '{expected_status_str}'."
        assert receipt.status == 1, f"Transaction for order '{order_id}' failed (status 0) when expecting status '{expected_status_str}'."

    then_order_status_is(escrow_contract, order_id, expected_status_str) # Re-use the status checking logic

@then(parsers.parse("{user_role}'s balance should be refunded approximately "{amount_eth}" ETH (minus gas)"))
def then_balance_refunded_approx(web3_connection, scenario_context, user_role, amount_eth, accounts):
    w3 = web3_connection
    order_id = scenario_context['current_order_id'] 
    user_account_address = scenario_context['users'][user_role] 
    expected_refund_wei = Web3.to_wei(Decimal(amount_eth), 'ether')

    balance_after_cancel = get_eth_balance(w3, user_account_address)
    balance_before_cancel = scenario_context[f'{user_role.lower()}_balance_before_cancel_for_{order_id}']
    
    actual_change_wei = balance_after_cancel - balance_before_cancel
    
    receipt = scenario_context[f'last_tx_receipt_for_{order_id}']
    tx_hash = scenario_context[f'last_tx_hash_for_{order_id}']
    gas_used = receipt.gasUsed
    tx = w3.eth.get_transaction(tx_hash)
    gas_price = tx.gasPrice
    tx_cost_wei = gas_used * gas_price

    expected_net_change_wei = expected_refund_wei - tx_cost_wei

    assert actual_change_wei == expected_net_change_wei, \
        f"{user_role}'s balance change after refund for order '{order_id}' is incorrect. Expected net change: {expected_net_change_wei} Wei (Refund: {expected_refund_wei} - TxCost: {tx_cost_wei}), Actual change: {actual_change_wei} Wei."


# Example of how to run:
# Ensure Ganache is running.
# From the crypto-bot-delivery directory:
# python -m pytest src/delivery_crypto/tests/step_defs/test_delivery_lifecycle_steps.py
# OR, if pyproject.toml is configured for test paths:
# python -m pytest
```
