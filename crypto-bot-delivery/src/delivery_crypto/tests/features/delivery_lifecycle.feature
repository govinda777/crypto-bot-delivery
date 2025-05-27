Feature: Delivery Lifecycle on Escrow Smart Contract
  As a user of the crypto-bot-delivery system,
  I want to ensure the delivery lifecycle (order creation, pickup, delivery, payment)
  is correctly managed by the Escrow smart contract.

  Background:
    Given a connection to the blockchain node
    And the Escrow contract is deployed
    And a client account "Alice" with sufficient funds
    And a robot account "RoboDeliverer"

  Scenario: Successful delivery and payment release
    Given Alice wants to create a delivery order with ID "order123" for RoboDeliverer
    And the order amount is "0.1" ETH
    And the pickup secret is "pickup_secret_123"
    And the delivery secret is "delivery_secret_123"
    When Alice creates the delivery order "order123"
    Then the order "order123" should be successfully created on the blockchain
    And the contract balance should increase by "0.1" ETH
    And the status of order "order123" should be "Created"

    When RoboDeliverer confirms pickup for order "order123" with secret "pickup_secret_123"
    Then the pickup for order "order123" should be confirmed
    And the status of order "order123" should be "PickedUp"

    When Alice confirms delivery for order "order123" with secret "delivery_secret_123"
    Then the delivery for order "order123" should be confirmed
    And RoboDeliverer's balance should increase by approximately "0.1" ETH (minus gas)
    And the status of order "order123" should be "Delivered"
    And the contract balance for order "order123" should be "0"

  Scenario: Client cancels an order before pickup
    Given Alice wants to create a delivery order with ID "order456" for RoboDeliverer
    And the order amount is "0.05" ETH
    And the pickup secret is "pickup_secret_456"
    And the delivery secret is "delivery_secret_456"
    When Alice creates the delivery order "order456"
    Then the order "order456" should be successfully created on the blockchain

    When Alice cancels the order "order456"
    Then the order "order456" should be marked as "Cancelled"
    And Alice's balance should be refunded approximately "0.05" ETH (minus gas)
    And the contract balance for order "order456" should be "0"

  # Add more scenarios:
  # - Attempt to confirm pickup with wrong secret
  # - Attempt to confirm delivery with wrong secret
  # - Attempt to cancel an order after pickup (should fail)
  # - Attempt to create an order with insufficient funds (handled by Web3.py or contract)
