// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract Escrow {
    address public owner;

    enum Status { Created, PickedUp, Delivered, Cancelled, Refunded }

    struct DeliveryOrder {
        address payable client;
        address payable deliveryProvider; // The robot's wallet
        uint256 amount;
        Status status;
        bytes32 pickupConfirmationHash; // Hash of a secret for pickup
        bytes32 deliveryConfirmationHash; // Hash of a secret for delivery
    }

    mapping(bytes32 => DeliveryOrder) public orders;

    event OrderCreated(bytes32 indexed orderId, address indexed client, address indexed deliveryProvider, uint256 amount);
    event OrderPickedUp(bytes32 indexed orderId);
    event OrderDelivered(bytes32 indexed orderId, address indexed deliveryProvider, uint256 amount);
    event OrderCancelled(bytes32 indexed orderId);
    event PaymentReleased(bytes32 indexed orderId, address indexed recipient, uint256 amount);
    event FundsRefunded(bytes32 indexed orderId, address indexed recipient, uint256 amount);

    modifier onlyOwner() {
        require(msg.sender == owner, "Escrow: Not the contract owner");
        _;
    }

    modifier onlyClient(bytes32 orderId) {
        require(msg.sender == orders[orderId].client, "Escrow: Not the client for this order");
        _;
    }

    modifier onlyDeliveryProvider(bytes32 orderId) {
        require(msg.sender == orders[orderId].deliveryProvider, "Escrow: Not the delivery provider for this order");
        _;
    }

    constructor() {
        owner = msg.sender;
    }

    function createOrder(bytes32 orderId, address payable _deliveryProvider, bytes32 _pickupConfirmationHash, bytes32 _deliveryConfirmationHash) public payable {
        require(msg.value > 0, "Escrow: Order amount must be greater than 0");
        require(orders[orderId].client == address(0), "Escrow: Order ID already exists"); // Basic check for uniqueness
        require(_deliveryProvider != address(0), "Escrow: Delivery provider address cannot be zero");

        orders[orderId] = DeliveryOrder({
            client: payable(msg.sender),
            deliveryProvider: _deliveryProvider,
            amount: msg.value,
            status: Status.Created,
            pickupConfirmationHash: _pickupConfirmationHash,
            deliveryConfirmationHash: _deliveryConfirmationHash
        });

        emit OrderCreated(orderId, msg.sender, _deliveryProvider, msg.value);
    }

    function confirmPickup(bytes32 orderId, string memory pickupSecret) public onlyDeliveryProvider(orderId) {
        DeliveryOrder storage order = orders[orderId];
        require(order.status == Status.Created, "Escrow: Order not in Created state");
        require(keccak256(abi.encodePacked(pickupSecret)) == order.pickupConfirmationHash, "Escrow: Invalid pickup secret");

        order.status = Status.PickedUp;
        emit OrderPickedUp(orderId);
    }

    function confirmDelivery(bytes32 orderId, string memory deliverySecret) public onlyClient(orderId) {
        DeliveryOrder storage order = orders[orderId];
        require(order.status == Status.PickedUp, "Escrow: Order not in PickedUp state");
        require(keccak256(abi.encodePacked(deliverySecret)) == order.deliveryConfirmationHash, "Escrow: Invalid delivery secret");

        order.status = Status.Delivered;
        // Transfer funds to delivery provider
        (bool sent, ) = order.deliveryProvider.call{value: order.amount}("");
        require(sent, "Escrow: Failed to send payment to delivery provider");

        emit OrderDelivered(orderId, order.deliveryProvider, order.amount);
        emit PaymentReleased(orderId, order.deliveryProvider, order.amount);
    }

    function cancelOrder(bytes32 orderId) public onlyClient(orderId) {
        DeliveryOrder storage order = orders[orderId];
        require(order.status == Status.Created, "Escrow: Order can only be cancelled if in Created state");

        order.status = Status.Cancelled;
        // Refund payment to client
        (bool sent, ) = order.client.call{value: order.amount}("");
        require(sent, "Escrow: Failed to refund payment to client");
        
        emit OrderCancelled(orderId);
        emit FundsRefunded(orderId, order.client, order.amount);
    }

    // Fallback function to receive Ether
    receive() external payable {}

    // Function to allow owner to withdraw any Ether sent to the contract by mistake
    function withdrawContractBalance() public onlyOwner {
        payable(owner).transfer(address(this).balance);
    }
}
