Feature: Delivery Management API
  As an API client (e.g., Mobile App, Robot System)
  I want to manage deliveries (create, view, update, list, cancel)
  And ensure data integrity and correct responses.

  Background:
    Given a FastAPI TestClient for the delivery API
    And the mock delivery database is empty
    And a valid JWT access token for user "bdd_test_user"

  Scenario: Successfully create a new delivery
    Given I have delivery creation data:
      | pickup_address      | dropoff_address       | item_description |
      | 100 BDD Pickup St   | 200 BDD Dropoff Ave | BDD Test Package |
    When I make a POST request to "/deliveries/" with the delivery data and token
    Then the API response status code should be 201
    And the response should contain the created delivery details
    And the delivery ID should not be empty
    And the delivery status should be "pending"
    And the delivery user_id should be "bdd_test_user"

  Scenario: Retrieve an existing delivery by ID
    Given a delivery has been created with user "bdd_test_user" and data:
      | pickup_address      | dropoff_address       | item_description     |
      | 101 BDD Pickup St   | 201 BDD Dropoff Ave | BDD Retrieval Test |
    When I make a GET request to "/deliveries/{delivery_id}" for the created delivery
    Then the API response status code should be 200
    And the response should contain the details of the retrieved delivery

  Scenario: Attempt to retrieve a non-existent delivery
    When I make a GET request to "/deliveries/non-existent-uuid-12345"
    Then the API response status code should be 404
    And the response detail should be "Delivery not found"

  Scenario: Update the status of an existing delivery
    Given a delivery has been created with user "bdd_test_user" and data:
      | pickup_address      | dropoff_address       | item_description          |
      | 102 BDD Pickup St   | 202 BDD Dropoff Ave | BDD Status Update Test    |
    When I make a PATCH request to "/deliveries/{delivery_id}/status" for the created delivery with new status "assigned" and token
    Then the API response status code should be 200
    And the delivery status in the response should be "assigned"
    And the delivery version should be incremented

  Scenario: Update details of an existing delivery
    Given a delivery has been created with user "bdd_test_user" and data:
      | pickup_address      | dropoff_address       | item_description        |
      | 103 BDD Pickup St   | 203 BDD Dropoff Ave | BDD Details Update Test |
    When I make a PATCH request to "/deliveries/{delivery_id}" for the created delivery with new details and token:
      | item_description        | robot_id   |
      | Updated BDD Description | robot_bdd_1 |
    Then the API response status code should be 200
    And the delivery item_description in the response should be "Updated BDD Description"
    And the delivery robot_id in the response should be "robot_bdd_1"
    And the delivery version should be incremented

  Scenario: List deliveries for a specific user
    Given a delivery has been created with user "user_list_A" and data:
      | pickup_address | dropoff_address | item_description |
      | List St A1     | List Ave A1     | Item A1          |
    And a delivery has been created with user "user_list_B" and data:
      | pickup_address | dropoff_address | item_description |
      | List St B1     | List Ave B1     | Item B1          |
    And a delivery has been created with user "user_list_A" and data:
      | pickup_address | dropoff_address | item_description |
      | List St A2     | List Ave A2     | Item A2          |
    When I make a GET request to "/deliveries/?user_id=user_list_A"
    Then the API response status code should be 200
    And the response should be a list containing 2 deliveries
    And all deliveries in the list should have user_id "user_list_A"

  Scenario: Cancel an existing delivery
    Given a delivery has been created with user "bdd_test_user" and data:
      | pickup_address      | dropoff_address       | item_description      |
      | 104 BDD Pickup St   | 204 BDD Dropoff Ave | BDD Cancellation Test |
    When I make a DELETE request to "/deliveries/{delivery_id}" for the created delivery with token
    Then the API response status code should be 200
    And the delivery status in the response should be "cancelled"

  Scenario: Attempt to cancel a non-existent delivery
    When I make a DELETE request to "/deliveries/non-existent-uuid-for-cancel" with token
    Then the API response status code should be 404

  Scenario: Attempt to cancel an already delivered delivery (should fail)
    Given a delivery has been created with user "bdd_test_user" and data:
      | pickup_address      | dropoff_address       | item_description      |
      | 105 BDD Pickup St   | 205 BDD Dropoff Ave | BDD Delivered Test    |
    And its status has been updated to "delivered"
    When I make a DELETE request to "/deliveries/{delivery_id}" for this delivery with token
    Then the API response status code should be 400 # Or as per API's error for invalid state transition
    And the response detail should indicate cancellation is not allowed
