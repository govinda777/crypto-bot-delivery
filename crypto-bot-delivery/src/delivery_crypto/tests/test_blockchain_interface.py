import pytest
from unittest.mock import patch, MagicMock  # For mocking
from web3 import (
    Web3,
)  # For type hinting if needed, but actual Web3 calls will be mocked

# from eth_account import Account # Account is used internally by web3.py, direct import not always needed for user code.
# The functions in blockchain_interface.py seem to return dicts, not Account objects directly.

# Assuming blockchain_interface.py is structured such that these can be imported
# This should work due to the editable install and pyproject.toml settings.
from delivery_crypto.blockchain_interface import (
    generate_secret_hash,
    create_or_load_account,
    connect_to_node,
    # Import other functions as you add tests for them
)


# Helper to simulate Web3.py's Account object for mocking purposes
# In a real scenario, if the functions returned Account objects, you'd mock those.
# But create_or_load_account returns a dict.
class MockEthAccount:
    def __init__(self, key, address):
        self.key = key
        self.address = address

    @staticmethod
    def create():
        # Simulate Account.create()
        # Return a new instance of our mock account with some dummy data
        return MockEthAccount(
            key=b"a_new_random_private_key_bytes_32", address="0xNewMockAddress"
        )

    @staticmethod
    def from_key(private_key_bytes_hex):
        # Simulate Account.from_key()
        # For simplicity, let's assume the hex string passed in is just used to generate a predictable address
        # and the key is stored as bytes.
        # Remove '0x' if present for byte conversion
        if private_key_bytes_hex.startswith("0x"):
            key_bytes = bytes.fromhex(private_key_bytes_hex[2:])
        else:
            key_bytes = bytes.fromhex(private_key_bytes_hex)
        return MockEthAccount(
            key=key_bytes, address=f"0xAddressFrom{private_key_bytes_hex[:10]}"
        )


def test_generate_secret_hash_consistency():
    # Note: The original blockchain_interface.generate_secret_hash expects a Web3 instance
    # as its first argument. The example test omitted this.
    # For now, let's assume we need to pass a mock Web3 instance.
    mock_w3 = MagicMock(spec=Web3)
    mock_w3.keccak = Web3.keccak  # Use actual keccak for realistic hashing

    secret = "supersecretdeliverycode"
    hash1 = generate_secret_hash(mock_w3, secret)
    hash2 = generate_secret_hash(mock_w3, secret)
    assert hash1 == hash2
    assert isinstance(hash1, bytes)  # Should be bytes for the contract


def test_generate_secret_hash_uniqueness():
    mock_w3 = MagicMock(spec=Web3)
    mock_w3.keccak = Web3.keccak

    secret1 = "supersecretdeliverycode1"
    secret2 = "supersecretdeliverycode2"
    hash1 = generate_secret_hash(mock_w3, secret1)
    hash2 = generate_secret_hash(mock_w3, secret2)
    assert hash1 != hash2


def test_generate_secret_hash_empty_string():
    mock_w3 = MagicMock(spec=Web3)
    mock_w3.keccak = Web3.keccak

    secret = ""
    hash_val = generate_secret_hash(mock_w3, secret)
    assert isinstance(hash_val, bytes)
    assert (
        Web3.keccak(text="") == hash_val
    )  # Compare with direct keccak of empty string


# Patching where 'Account' is used within blockchain_interface.py
# Assuming 'from eth_account import Account' is used in blockchain_interface.py
@patch("delivery_crypto.blockchain_interface.Web3.eth.account.create")  # Corrected path
def test_create_new_account(mock_account_create_w3):
    # The function create_or_load_account takes a w3 instance.
    mock_w3_main = MagicMock(spec=Web3)

    # Mock the account object that Account.create() would return
    mock_new_eth_account = MockEthAccount(
        key=b"testprivatekey_bytes_for_real", address="0xRealNewTestAddress"
    )
    mock_account_create_w3.return_value = mock_new_eth_account

    # Mock w3.to_hex for the private key conversion
    mock_w3_main.to_hex = lambda data: "0x" + data.hex()

    # Call the function under test
    # The function was modified to return the account object directly, not a dict.
    # And it prints, rather than returning a warning in a dict.
    # We should test the returned account object.
    returned_account = create_or_load_account(mock_w3_main)

    mock_account_create_w3.assert_called_once()
    assert returned_account.address == "0xRealNewTestAddress"
    # The function in blockchain_interface.py actually returns the account object, not a dict
    # So, we check properties of the returned account object.
    # The private key is also part of this object.
    # The print statement with the warning is harder to test without capturing stdout.


@patch(
    "delivery_crypto.blockchain_interface.Web3.eth.account.from_key"
)  # Corrected path
def test_load_existing_account(mock_account_from_key_w3):
    mock_w3_main = MagicMock(spec=Web3)
    test_pk_hex = "0x" + b"anotherkey".hex()  # Example private key in hex

    # Mock the account object that Account.from_key() would return
    mock_loaded_eth_account = MockEthAccount(
        key=b"anotherkey", address="0xRealLoadedAddress"
    )
    mock_account_from_key_w3.return_value = mock_loaded_eth_account

    # Call the function under test
    returned_account = create_or_load_account(mock_w3_main, private_key=test_pk_hex)

    mock_account_from_key_w3.assert_called_once_with(test_pk_hex)
    assert returned_account.address == "0xRealLoadedAddress"
    # Again, testing the returned account object directly.


@patch(
    "delivery_crypto.blockchain_interface.Web3.HTTPProvider"
)  # Path to HTTPProvider within Web3 module
@patch(
    "delivery_crypto.blockchain_interface.Web3"
)  # Path to Web3 class itself for instantiation
def test_connect_to_node_success(
    mock_web3_class, mock_http_provider_class
):  # Order of mocks is important
    mock_provider_instance = MagicMock()
    mock_http_provider_class.return_value = mock_provider_instance

    mock_web3_instance = MagicMock(spec=Web3)
    mock_web3_instance.is_connected.return_value = (
        True  # Simulate successful connection
    )
    # Simulate middleware injection if that's part of the success criteria
    mock_web3_instance.middleware_onion = MagicMock()
    mock_web3_class.return_value = mock_web3_instance

    node_url = "http://localhost:8545"
    w3_returned = connect_to_node(node_url)  # connect_to_node from blockchain_interface

    mock_http_provider_class.assert_called_once_with(node_url)
    mock_web3_class.assert_called_once_with(mock_provider_instance)
    assert w3_returned is not None
    assert w3_returned.is_connected()  # Check the mocked return value
    mock_web3_instance.middleware_onion.inject.assert_called_once()  # Check if middleware was injected


@patch("delivery_crypto.blockchain_interface.Web3.HTTPProvider")
@patch("delivery_crypto.blockchain_interface.Web3")
def test_connect_to_node_failure(mock_web3_class, mock_http_provider_class):
    mock_provider_instance = MagicMock()
    mock_http_provider_class.return_value = mock_provider_instance

    mock_web3_instance = MagicMock(spec=Web3)
    mock_web3_instance.is_connected.return_value = False  # Simulate failed connection
    mock_web3_class.return_value = mock_web3_instance

    node_url = "http://localhost:8545"
    w3_returned = connect_to_node(node_url)  # connect_to_node from blockchain_interface

    assert w3_returned is None  # As per the function's logic to return None on failure


# Example of how to run pytest from terminal (from crypto-bot-delivery directory):
# python -m pytest

# To see print statements from tests (if any) and more details:
# python -m pytest -s -v

# To generate a coverage report (if pytest-cov is installed):
# python -m pytest --cov=src/delivery_crypto --cov-report=html
# (This assumes your source code is in src/delivery_crypto and tests are run from project root)

# Note: The original test example for create_or_load_account returned a dict.
# The actual blockchain_interface.py's create_or_load_account returns a Web3 Account object.
# The tests have been adjusted to reflect this and to mock the Account object methods correctly
# by patching them where they are called (Web3.eth.account.create and Web3.eth.account.from_key).
# The MockEthAccount class is a local helper for these tests if we needed to simulate the Account object's structure,
# but patching at the source of the call (e.g. `Web3.eth.account.create`) is more direct.
# The `generate_secret_hash` function in `blockchain_interface.py` also takes `w3` as an argument,
# so tests were updated to pass a mock `w3`.
