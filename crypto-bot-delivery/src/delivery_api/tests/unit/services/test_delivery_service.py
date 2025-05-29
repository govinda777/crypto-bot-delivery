import pytest
from datetime import datetime, timezone, timedelta
import uuid
import time # Added for time.sleep()

# Adjust import paths based on actual project structure and how pytest discovers modules.
# Assuming 'app' is discoverable (e.g., project installed with -e ., or PYTHONPATH configured).
from app.models.schemas.delivery import (
    Delivery, DeliveryCreate, DeliveryUpdate, DeliveryStatus
)
from app.services.delivery_service import DeliveryService, _deliveries_db # Import _deliveries_db for clearing

@pytest.fixture(autouse=True) # Automatically use this fixture for every test in this file
def clear_mock_db_between_tests():
    # print(f"Clearing DB before test. Current size: {len(_deliveries_db)}") # For debugging
    _deliveries_db.clear() # Clear before each test
    yield # This is where the test runs
    # print(f"Clearing DB after test. Current size: {len(_deliveries_db)}") # For debugging
    _deliveries_db.clear() # Clear after each test

@pytest.fixture
def delivery_service():
    return DeliveryService()

@pytest.fixture
def sample_user_id():
    return "user_unit_test_123"

@pytest.fixture
def sample_delivery_create_data():
    return DeliveryCreate(
        pickup_address="1 Test Ln, Testville",
        dropoff_address="10 End St, Testburg",
        item_description="Unit Test Item",
        pickup_phone_number="1234567890" # Added example phone
    )

def test_create_delivery(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created_delivery = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)

    assert created_delivery is not None
    assert isinstance(created_delivery, Delivery)
    assert created_delivery.id is not None
    assert len(_deliveries_db) == 1, f"DB should have 1 entry, found {len(_deliveries_db)}"
    assert _deliveries_db[created_delivery.id] == created_delivery

    assert created_delivery.pickup_address == sample_delivery_create_data.pickup_address
    assert created_delivery.dropoff_address == sample_delivery_create_data.dropoff_address
    assert created_delivery.item_description == sample_delivery_create_data.item_description
    assert created_delivery.pickup_phone_number == sample_delivery_create_data.pickup_phone_number
    assert created_delivery.user_id == sample_user_id
    assert created_delivery.status == DeliveryStatus.PENDING
    assert created_delivery.version == 1
    assert isinstance(created_delivery.created_at, datetime)
    assert isinstance(created_delivery.updated_at, datetime)
    assert created_delivery.created_at == created_delivery.updated_at, "created_at and updated_at should be same on creation"

def test_get_delivery_by_id_found(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)
    retrieved = delivery_service.get_delivery_by_id(created.id)
    
    assert retrieved is not None
    assert retrieved.id == created.id
    assert retrieved == created # Compares Pydantic model instances

def test_get_delivery_by_id_not_found(delivery_service: DeliveryService):
    retrieved = delivery_service.get_delivery_by_id(str(uuid.uuid4())) # Use a valid UUID format
    assert retrieved is None

def test_update_delivery_status(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)
    original_updated_at = created.updated_at
    
    time.sleep(0.001) # Ensure timestamp difference
    
    updated = delivery_service.update_delivery_status(created.id, DeliveryStatus.ASSIGNED)
    
    assert updated is not None
    assert updated.id == created.id
    assert updated.status == DeliveryStatus.ASSIGNED
    assert updated.version == 2
    assert updated.updated_at > original_updated_at
    assert _deliveries_db[created.id].status == DeliveryStatus.ASSIGNED
    assert _deliveries_db[created.id].version == 2

def test_update_delivery_status_not_found(delivery_service: DeliveryService):
    updated = delivery_service.update_delivery_status(str(uuid.uuid4()), DeliveryStatus.ASSIGNED)
    assert updated is None

def test_update_delivery_details_full(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)
    original_updated_at = created.updated_at
    time.sleep(0.001)

    update_payload = DeliveryUpdate(
        pickup_address="100 New Pickup Rd",
        dropoff_address="200 New Dropoff Rd",
        item_description="Updated Test Item For Full Update", 
        pickup_phone_number="1110001110",
        dropoff_phone_number="2220002220",
        robot_id="robot_test_full_001",
        status=DeliveryStatus.IN_TRANSIT_PICKUP
    )
    updated = delivery_service.update_delivery_details(created.id, update_payload)

    assert updated is not None
    assert updated.id == created.id
    assert updated.pickup_address == "100 New Pickup Rd"
    assert updated.dropoff_address == "200 New Dropoff Rd"
    assert updated.item_description == "Updated Test Item For Full Update"
    assert updated.pickup_phone_number == "1110001110"
    assert updated.dropoff_phone_number == "2220002220"
    assert updated.robot_id == "robot_test_full_001"
    assert updated.status == DeliveryStatus.IN_TRANSIT_PICKUP
    assert updated.version == 2
    assert updated.updated_at > original_updated_at
    # Check if the update is reflected in the mock DB
    db_entry = _deliveries_db[created.id]
    assert db_entry.item_description == "Updated Test Item For Full Update"
    assert db_entry.robot_id == "robot_test_full_001"
    assert db_entry.status == DeliveryStatus.IN_TRANSIT_PICKUP

def test_update_delivery_details_partial(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)
    original_item_desc = created.item_description
    original_pickup_address = created.pickup_address
    original_updated_at = created.updated_at
    time.sleep(0.001)

    update_payload = DeliveryUpdate(robot_id="robot_partial_002", status=DeliveryStatus.AT_PICKUP) # Only update robot_id and status
    updated = delivery_service.update_delivery_details(created.id, update_payload)

    assert updated is not None
    assert updated.id == created.id
    assert updated.robot_id == "robot_partial_002"
    assert updated.status == DeliveryStatus.AT_PICKUP
    assert updated.item_description == original_item_desc # Ensure other fields are untouched
    assert updated.pickup_address == original_pickup_address # Ensure other fields are untouched
    assert updated.version == 2
    assert updated.updated_at > original_updated_at
    db_entry = _deliveries_db[created.id]
    assert db_entry.robot_id == "robot_partial_002"
    assert db_entry.status == DeliveryStatus.AT_PICKUP


def test_update_delivery_details_not_found(delivery_service: DeliveryService):
    update_payload = DeliveryUpdate(item_description="Doesn't matter for non-existent ID")
    updated = delivery_service.update_delivery_details(str(uuid.uuid4()), update_payload)
    assert updated is None

def test_get_deliveries_by_user(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate):
    user1 = "user_A_test"
    user2 = "user_B_test"
    
    # Create deliveries for user1
    created1_user1 = delivery_service.create_delivery(sample_delivery_create_data, user_id=user1)
    another_data_user1 = DeliveryCreate(pickup_address="p3", dropoff_address="d3", item_description="Item 3 for user A")
    created2_user1 = delivery_service.create_delivery(another_data_user1, user_id=user1)
    
    # Create delivery for user2
    data_user2 = DeliveryCreate(pickup_address="p2", dropoff_address="d2", item_description="Item for user B")
    created_user2 = delivery_service.create_delivery(data_user2, user_id=user2)

    user1_deliveries = delivery_service.get_deliveries_by_user(user1)
    assert len(user1_deliveries) == 2
    # Check if the retrieved deliveries match the ones created for user1
    retrieved_ids_user1 = {d.id for d in user1_deliveries}
    expected_ids_user1 = {created1_user1.id, created2_user1.id}
    assert retrieved_ids_user1 == expected_ids_user1
    for d in user1_deliveries:
        assert d.user_id == user1

    user2_deliveries = delivery_service.get_deliveries_by_user(user2)
    assert len(user2_deliveries) == 1
    assert user2_deliveries[0].id == created_user2.id
    assert user2_deliveries[0].user_id == user2
    
    user3_deliveries = delivery_service.get_deliveries_by_user("user_C_non_existent")
    assert len(user3_deliveries) == 0

def test_cancel_delivery_success_pending(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)
    assert created.status == DeliveryStatus.PENDING # Pre-condition
    original_updated_at = created.updated_at
    time.sleep(0.001)
    
    cancelled = delivery_service.cancel_delivery(created.id)

    assert cancelled is not None
    assert cancelled.id == created.id
    assert cancelled.status == DeliveryStatus.CANCELLED
    assert cancelled.version == 2
    assert cancelled.updated_at > original_updated_at
    assert _deliveries_db[created.id].status == DeliveryStatus.CANCELLED

def test_cancel_delivery_success_assigned(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)
    delivery_service.update_delivery_status(created.id, DeliveryStatus.ASSIGNED) # Set to ASSIGNED
    
    assert _deliveries_db[created.id].status == DeliveryStatus.ASSIGNED # Pre-condition
    assert _deliveries_db[created.id].version == 2 # After status update
    original_updated_at = _deliveries_db[created.id].updated_at
    time.sleep(0.001)

    cancelled = delivery_service.cancel_delivery(created.id)

    assert cancelled is not None
    assert cancelled.id == created.id
    assert cancelled.status == DeliveryStatus.CANCELLED
    assert cancelled.version == 3 # Incremented again
    assert cancelled.updated_at > original_updated_at
    assert _deliveries_db[created.id].status == DeliveryStatus.CANCELLED

def test_cancel_delivery_not_found(delivery_service: DeliveryService):
    cancelled = delivery_service.cancel_delivery(str(uuid.uuid4()))
    assert cancelled is None

def test_cancel_delivery_already_delivered_fails(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)
    # Simulate it being delivered
    delivery_service.update_delivery_status(created.id, DeliveryStatus.DELIVERED)
    
    assert _deliveries_db[created.id].status == DeliveryStatus.DELIVERED # Pre-condition
    original_version = _deliveries_db[created.id].version # Should be 2

    # Current service logic returns the object with original status if not cancelable
    not_cancelled_delivery = delivery_service.cancel_delivery(created.id) 
    
    assert not_cancelled_delivery is not None
    assert not_cancelled_delivery.id == created.id
    assert not_cancelled_delivery.status == DeliveryStatus.DELIVERED # Status should not change
    assert not_cancelled_delivery.version == original_version # Version should not change if status didn't change
    assert _deliveries_db[created.id].status == DeliveryStatus.DELIVERED

def test_cancel_delivery_already_failed_fails(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)
    delivery_service.update_delivery_status(created.id, DeliveryStatus.FAILED)
    
    assert _deliveries_db[created.id].status == DeliveryStatus.FAILED
    original_version = _deliveries_db[created.id].version

    not_cancelled_delivery = delivery_service.cancel_delivery(created.id) 
    assert not_cancelled_delivery is not None
    assert not_cancelled_delivery.status == DeliveryStatus.FAILED
    assert not_cancelled_delivery.version == original_version

def test_cancel_delivery_already_cancelled(delivery_service: DeliveryService, sample_delivery_create_data: DeliveryCreate, sample_user_id: str):
    created = delivery_service.create_delivery(delivery_data=sample_delivery_create_data, user_id=sample_user_id)
    delivery_service.cancel_delivery(created.id) # First cancellation
    
    assert _deliveries_db[created.id].status == DeliveryStatus.CANCELLED
    original_version = _deliveries_db[created.id].version # Should be 2

    already_cancelled_delivery = delivery_service.cancel_delivery(created.id) # Try to cancel again
    assert already_cancelled_delivery is not None
    assert already_cancelled_delivery.status == DeliveryStatus.CANCELLED
    assert already_cancelled_delivery.version == original_version # Should not change if already cancelled
```
