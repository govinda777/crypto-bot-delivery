# Smart Contract Guide: Escrow.sol

## 1. Overview

The `Escrow.sol` smart contract is a core component of the Crypto Bot Delivery system. Its primary purpose is to securely manage payments for delivery services. It acts as a neutral third party, holding the client's payment in escrow and releasing it to the delivery provider (the robot) only upon successful confirmation of delivery by the client. This mechanism builds trust and ensures fairness for both parties involved in a transaction.

Key functionalities include:
- Creating new delivery orders with funds locked in escrow.
- Allowing the delivery provider to confirm the pickup of an order.
- Allowing the client to confirm the successful delivery of an order, which triggers payment release.
- Allowing the client to cancel an order before pickup, resulting in a refund.

## 2. Contract Details

*   **Solidity Version:** `^0.8.0`
*   **Main Contract:** `Escrow`

## 3. State Variables

*   **`owner` (address public):**
    *   Stores the address of the account that deployed the smart contract.
    *   Has special privileges, such as withdrawing any Ether accidentally sent to the contract.

*   **`orders` (mapping(bytes32 => DeliveryOrder) public):**
    *   A mapping that stores all delivery orders.
    *   The key is a `bytes32` unique order ID.
    *   The value is a `DeliveryOrder` struct containing the details of that specific order.
    *   Being `public`, Solidity automatically generates a getter function for accessing orders by their ID.

## 4. Structs

### `DeliveryOrder`

This struct holds all the necessary information for a single delivery order.

*   **`client` (address payable):**
    *   The address of the client who initiated the delivery and funded the escrow.
    *   This address will receive a refund if the order is cancelled.
*   **`deliveryProvider` (address payable):**
    *   The address of the delivery robot/provider who will perform the delivery.
    *   This address will receive the payment upon successful delivery.
*   **`amount` (uint256):**
    *   The amount of Ether (in Wei) locked in escrow for the delivery.
*   **`status` (Status):**
    *   An enum representing the current status of the order. Possible values are:
        *   `Created`: The order has been created and funded by the client, but not yet picked up by the delivery provider.
        *   `PickedUp`: The delivery provider has confirmed they have picked up the items for delivery.
        *   `Delivered`: The client has confirmed that the delivery was successful. Payment is released to the delivery provider.
        *   `Cancelled`: The client has cancelled the order (only possible if status was `Created`). Funds are refunded to the client.
        *   `Refunded`: (Not explicitly set as a distinct step but implied post-cancellation) The funds have been processed for refunding. The `Cancelled` status effectively covers this state for fund return.
*   **`pickupConfirmationHash` (bytes32):**
    *   A Keccak256 hash of a secret string known to the delivery provider. Used to verify pickup confirmation.
*   **`deliveryConfirmationHash` (bytes32):**
    *   A Keccak256 hash of a secret string known to the client. Used to verify delivery confirmation.

## 5. Events

Events are emitted by the contract during key state changes. These can be monitored by off-chain applications.

*   **`OrderCreated(bytes32 indexed orderId, address indexed client, address indexed deliveryProvider, uint256 amount)`:**
    *   Emitted when a new delivery order is successfully created.
*   **`OrderPickedUp(bytes32 indexed orderId)`:**
    *   Emitted when the delivery provider successfully confirms the pickup of an order.
*   **`OrderDelivered(bytes32 indexed orderId, address indexed deliveryProvider, uint256 amount)`:**
    *   Emitted when the client successfully confirms the delivery of an order.
*   **`OrderCancelled(bytes32 indexed orderId)`:**
    *   Emitted when a client successfully cancels an order.
*   **`PaymentReleased(bytes32 indexed orderId, address indexed recipient, uint256 amount)`:**
    *   Emitted when the escrowed payment is successfully released to the delivery provider.
*   **`FundsRefunded(bytes32 indexed orderId, address indexed recipient, uint256 amount)`:**
    *   Emitted when the escrowed funds are successfully refunded to the client upon cancellation.

## 6. Modifiers

Modifiers are used to enforce access control and conditions on functions.

*   **`onlyOwner()`:**
    *   Restricts a function to be callable only by the `owner` of the contract.
*   **`onlyClient(bytes32 orderId)`:**
    *   Restricts a function to be callable only by the `client` associated with the specified `orderId`.
*   **`onlyDeliveryProvider(bytes32 orderId)`:**
    *   Restricts a function to be callable only by the `deliveryProvider` associated with the specified `orderId`.

## 7. Functions

### `constructor()`

*   **Visibility:** `public` (implicitly during construction)
*   **Parameters:** None
*   **Description:** This function is executed once when the contract is deployed. It sets the `owner` of the contract to the address that deployed it (`msg.sender`).

### `createOrder(bytes32 orderId, address payable _deliveryProvider, bytes32 _pickupConfirmationHash, bytes32 _deliveryConfirmationHash)`

*   **Visibility:** `public payable`
*   **Parameters:**
    *   `orderId` (bytes32): A unique identifier for the order.
    *   `_deliveryProvider` (address payable): The wallet address of the delivery robot.
    *   `_pickupConfirmationHash` (bytes32): Keccak256 hash of the pickup secret.
    *   `_deliveryConfirmationHash` (bytes32): Keccak256 hash of the delivery secret.
*   **Modifiers:** None
*   **Description:** Allows a client to create a new delivery order. The client sends Ether (payment) along with this call (`msg.value`), which is then locked in the contract.
*   **Key `require` statements:**
    *   `msg.value > 0`: Ensures that the order is funded.
    *   `orders[orderId].client == address(0)`: Ensures the `orderId` is unique.
    *   `_deliveryProvider != address(0)`: Ensures a valid delivery provider address.
*   **Events emitted:** `OrderCreated`

### `confirmPickup(bytes32 orderId, string memory pickupSecret)`

*   **Visibility:** `public`
*   **Parameters:**
    *   `orderId` (bytes32): The ID of the order to confirm pickup for.
    *   `pickupSecret` (string memory): The secret string provided by the robot to confirm pickup.
*   **Modifiers:** `onlyDeliveryProvider(orderId)`
*   **Description:** Allows the designated delivery provider for an order to confirm they have picked up the items. The provided `pickupSecret` is hashed and compared against the stored `pickupConfirmationHash`.
*   **Key `require` statements:**
    *   `orders[orderId].status == Status.Created`: Ensures the order is in the correct state for pickup.
    *   `keccak256(abi.encodePacked(pickupSecret)) == order.pickupConfirmationHash`: Verifies the pickup secret.
*   **Events emitted:** `OrderPickedUp`

### `confirmDelivery(bytes32 orderId, string memory deliverySecret)`

*   **Visibility:** `public`
*   **Parameters:**
    *   `orderId` (bytes32): The ID of the order to confirm delivery for.
    *   `deliverySecret` (string memory): The secret string provided by the client to confirm delivery.
*   **Modifiers:** `onlyClient(orderId)`
*   **Description:** Allows the client for an order to confirm that the delivery has been completed. The provided `deliverySecret` is hashed and compared against the stored `deliveryConfirmationHash`. Upon successful confirmation, the escrowed funds are transferred to the `deliveryProvider`.
*   **Key `require` statements:**
    *   `orders[orderId].status == Status.PickedUp`: Ensures the order has been picked up.
    *   `keccak256(abi.encodePacked(deliverySecret)) == order.deliveryConfirmationHash`: Verifies the delivery secret.
    *   `sent == true` (for fund transfer): Ensures the payment to the delivery provider was successful.
*   **Events emitted:** `OrderDelivered`, `PaymentReleased`

### `cancelOrder(bytes32 orderId)`

*   **Visibility:** `public`
*   **Parameters:**
    *   `orderId` (bytes32): The ID of the order to cancel.
*   **Modifiers:** `onlyClient(orderId)`
*   **Description:** Allows the client to cancel an order and receive a refund. An order can only be cancelled if it is in the `Created` state (i.e., before the delivery provider has confirmed pickup).
*   **Key `require` statements:**
    *   `orders[orderId].status == Status.Created`: Ensures the order is in a cancellable state.
    *   `sent == true` (for fund refund): Ensures the refund to the client was successful.
*   **Events emitted:** `OrderCancelled`, `FundsRefunded`

### `withdrawContractBalance()`

*   **Visibility:** `public`
*   **Parameters:** None
*   **Modifiers:** `onlyOwner`
*   **Description:** Allows the contract `owner` to withdraw any Ether that might have been accidentally sent to the contract's address (outside of order payments). This is a safety measure.

### `receive() external payable {}`

*   **Visibility:** `external payable`
*   **Description:** This is a fallback function that allows the contract to receive Ether directly (e.g., via a simple transfer without a function call).

## 8. Deployment

1.  **Compilation:**
    *   The contract is written for Solidity `^0.8.0`.
    *   It can be compiled using `solc` (the Solidity compiler). For example, using `npx solc@0.8.x --abi --optimize -o <output_directory> Escrow.sol`.
    *   The compilation process will produce the ABI (Application Binary Interface) and bytecode. The ABI is essential for interacting with the contract and is typically saved as a JSON file (e.g., `Escrow.json`).

2.  **Deployment:**
    *   Once compiled, the contract can be deployed to an Ethereum network (e.g., a local testnet like Ganache, a public testnet like Sepolia, or the Ethereum mainnet).
    *   Deployment tools like Remix IDE, Hardhat, or Truffle can be used.
    *   When deploying, you will need the contract's bytecode and ABI. The deployment transaction will require gas and will result in a contract address once mined.

## 9. Interaction

Once deployed, the `Escrow.sol` contract can be interacted with by sending transactions to its functions.

*   **Using Web3 Libraries:** Typically, off-chain applications interact with smart contracts via libraries like:
    *   `web3.py` for Python (as demonstrated in `crypto-bot-delivery/src/delivery_crypto/blockchain_interface.py`)
    *   `web3.js` or `ethers.js` for JavaScript/Node.js
*   These libraries use the contract's ABI (`Escrow.json`) and its deployed address to encode function calls and decode event data.
*   The `blockchain_interface.py` script in this project provides a practical example of how to connect to an Ethereum node, load the `Escrow` contract, and call its functions to simulate the delivery workflow.
