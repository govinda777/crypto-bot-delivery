Feature: Delivery API Authentication and Authorization
  As an API client,
  I want to ensure that protected API endpoints require valid authentication
  And that unauthorized access is denied.

  Background:
    Given a FastAPI TestClient for the delivery API
    And the mock delivery database is empty

  Scenario: Accessing a protected endpoint without a token
    Given I have delivery creation data:
      | pickup_address      | dropoff_address       | item_description |
      | Auth Test Pickup    | Auth Test Dropoff   | Auth Package     |
    When I make a POST request to "/deliveries/" with the delivery data without a token
    Then the API response status code should be 401 # Or 403 if FastAPI handles it as Forbidden by default without scheme
    And the response should indicate missing or invalid credentials

  Scenario: Accessing a protected endpoint with an invalid/expired token
    Given I have delivery creation data:
      | pickup_address      | dropoff_address       | item_description |
      | Auth Test Pickup    | Auth Test Dropoff   | Auth Package     |
    And an invalid JWT access token "invalid.jwt.token"
    When I make a POST request to "/deliveries/" with the delivery data and the invalid token
    Then the API response status code should be 401
    And the response should indicate invalid credentials

  Scenario: Accessing a protected endpoint with a valid token
    Given I have delivery creation data:
      | pickup_address      | dropoff_address       | item_description |
      | Auth Test Pickup    | Auth Test Dropoff   | Auth Package     |
    And a valid JWT access token for user "authed_user"
    When I make a POST request to "/deliveries/" with the delivery data and token
    Then the API response status code should be 201
    And the response should contain the created delivery details
    And the delivery user_id should be "authed_user"

  # Future: Add scenarios for different user roles and permissions if implemented
