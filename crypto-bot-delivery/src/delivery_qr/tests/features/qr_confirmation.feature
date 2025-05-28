Feature: QR Code Generation and Validation for Delivery Confirmations
  As a part of the delivery system,
  I want to generate secure QR codes for pickup and delivery,
  And validate them to ensure authentic confirmation steps.

  Background:
    Given a unique order ID "order_confirm_789"
    And a temporary directory for QR code images

  Scenario: Successful generation and validation of a Pickup QR Code
    Given the confirmation type is "pickup"
    And a unique token "pickup_token_abc123" is generated for the order
    When a QR code is generated for "order_confirm_789" with type "pickup" and token "pickup_token_abc123"
    Then a QR code image file should be created
    When the generated "pickup" QR code image is scanned
    Then the decoded QR data should contain "order_confirm_789", type "pickup", and token "pickup_token_abc123"

  Scenario: Successful generation and validation of a Delivery QR Code
    Given the confirmation type is "delivery"
    And a unique token "delivery_token_xyz789" is generated for the order
    When a QR code is generated for "order_confirm_789" with type "delivery" and token "delivery_token_xyz789"
    Then a QR code image file should be created
    When the generated "delivery" QR code image is scanned
    Then the decoded QR data should contain "order_confirm_789", type "delivery", and token "delivery_token_xyz789"

  Scenario: Attempt to validate a QR code with tampered/incorrect token
    Given the confirmation type is "pickup"
    And a unique token "actual_pickup_token_456" is generated for the order
    When a QR code is generated for "order_confirm_789" with type "pickup" and token "actual_pickup_token_456"
    Then a QR code image file should be created
    When the generated "pickup" QR code image is scanned
    Then the decoded QR data contains token "actual_pickup_token_456"
    But the system expects token "expected_token_incorrect_789" for "order_confirm_789" type "pickup"
    And the QR data validation should fail due to token mismatch

  Scenario: Attempt to scan a non-QR image or corrupted QR image
    Given a non-QR image file exists
    When the non-QR image file is scanned
    Then the QR scanning should fail or return no usable data

  # Note:
  # The "validation" in these scenarios refers to checking the content of the QR code.
  # The broader system-level validation (e.g., API checking the token against its database)
  # would be part of a higher-level integration test (e.g., API BDD tests).
  # For "QR data validation should fail due to token mismatch", this step in this
  # feature's context means the *test itself* asserts the token is different,
  # not that a specific validation function in delivery_qr fails it (as that logic
  # is currently outside delivery_qr).
```
