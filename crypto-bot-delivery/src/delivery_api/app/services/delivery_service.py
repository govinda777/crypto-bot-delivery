import uuid
from datetime import datetime, timezone
from typing import List, Dict, Optional

# Assuming models are in app.models.schemas
# Adjust import path if your project structure differs or if using absolute imports
# e.g., from crypto_bot_delivery.src.delivery_api.app.models.schemas.delivery import ...
# For simplicity, assuming direct import works due to PYTHONPATH or package structure
from app.models.schemas.delivery import (
    Delivery,
    DeliveryCreate,
    DeliveryUpdate, 
    DeliveryStatus,
    # DeliveryStatusUpdate # Service method can just take DeliveryStatus directly for now
)

# In-memory database (dictionary) to store delivery objects
# Key: delivery_id (str), Value: Delivery Pydantic model instance
_deliveries_db: Dict[str, Delivery] = {}


class DeliveryService:
    def __init__(self):
        # For this POC, the DB is a simple module-level dict.
        # In a real app, this might receive a database session/client.
        pass

    def create_delivery(self, delivery_data: DeliveryCreate, user_id: str) -> Delivery:
        delivery_id = str(uuid.uuid4())
        current_time = datetime.now(timezone.utc)

        # Conceptual: Simulate interaction with delivery_crypto for escrow
        print(f"INFO: [DeliveryService] Conceptual: Initiating blockchain escrow for order {delivery_id} by user {user_id}.")
        # For POC, we'll assume escrow was successful and proceed.
        
        new_delivery = Delivery(
            id=delivery_id,
            **delivery_data.model_dump(), # Spread fields from DeliveryCreate
            user_id=user_id,
            status=DeliveryStatus.PENDING, # Initial status
            robot_id=None,
            created_at=current_time,
            updated_at=current_time,
            version=1
        )
        _deliveries_db[delivery_id] = new_delivery
        print(f"INFO: [DeliveryService] Delivery created: {delivery_id}")
        return new_delivery

    def get_delivery_by_id(self, delivery_id: str) -> Optional[Delivery]:
        return _deliveries_db.get(delivery_id)

    def update_delivery_status(self, delivery_id: str, new_status: DeliveryStatus) -> Optional[Delivery]:
        delivery = _deliveries_db.get(delivery_id)
        if delivery:
            delivery.status = new_status
            delivery.updated_at = datetime.now(timezone.utc)
            delivery.version += 1
            # _deliveries_db[delivery_id] = delivery # No need, delivery is a mutable object reference
            print(f"INFO: [DeliveryService] Status updated for delivery {delivery_id} to {new_status.value}")
            return delivery
        return None
        
    def update_delivery_details(self, delivery_id: str, delivery_update_data: DeliveryUpdate) -> Optional[Delivery]:
        delivery = _deliveries_db.get(delivery_id)
        if not delivery:
            return None

        update_data_dict = delivery_update_data.model_dump(exclude_unset=True)
        
        for key, value in update_data_dict.items():
            setattr(delivery, key, value)
            
        delivery.updated_at = datetime.now(timezone.utc)
        delivery.version += 1
        # _deliveries_db[delivery_id] = delivery # No need, delivery is a mutable object reference
        print(f"INFO: [DeliveryService] Details updated for delivery {delivery_id}")
        return delivery

    def get_deliveries_by_user(self, user_id: str) -> List[Delivery]:
        return [
            delivery for delivery in _deliveries_db.values() if delivery.user_id == user_id
        ]

    def cancel_delivery(self, delivery_id: str) -> Optional[Delivery]:
        delivery = _deliveries_db.get(delivery_id)
        if delivery:
            # Basic check: Allow cancellation if not already delivered or in a final failed state.
            # More complex state transition logic could be added here.
            if delivery.status not in [DeliveryStatus.DELIVERED, DeliveryStatus.FAILED, DeliveryStatus.CANCELLED]:
                delivery.status = DeliveryStatus.CANCELLED
                delivery.updated_at = datetime.now(timezone.utc)
                delivery.version += 1
                # _deliveries_db[delivery_id] = delivery # No need
                print(f"INFO: [DeliveryService] Conceptual: Initiating blockchain refund/cancellation for order {delivery_id}.")
                print(f"INFO: [DeliveryService] Delivery cancelled: {delivery_id}")
                return delivery
            else:
                print(f"WARN: [DeliveryService] Delivery {delivery_id} cannot be cancelled, status is {delivery.status.value}")
                return delivery 
        return None

    # Helper for testing or admin purposes - clear the mock DB
    def clear_all_deliveries_mock_db(self):
        _deliveries_db.clear()
        print("INFO: [DeliveryService] Mock database cleared.")

if __name__ == '__main__':
    # Example Usage
    service = DeliveryService()
    service.clear_all_deliveries_mock_db() # Clear for fresh run

    print("\n--- Create Deliveries ---")
    delivery_input_1 = DeliveryCreate(
        pickup_address="1 Main St", 
        dropoff_address="10 End Ave", 
        item_description="Books",
        pickup_phone_number="1112223333"
    )
    delivery1 = service.create_delivery(delivery_input_1, user_id="user1")
    
    delivery_input_2 = DeliveryCreate(
        pickup_address="20 Side Rd", 
        dropoff_address="50 Last Stop", 
        item_description="Electronics",
        dropoff_phone_number="4445556666"
    )
    delivery2 = service.create_delivery(delivery_input_2, user_id="user2")
    
    delivery_input_3 = DeliveryCreate(
        pickup_address="30 Broad St", 
        dropoff_address="70 Final Dest", 
        item_description="Gadgets",
        pickup_phone_number="7778889999"
    )
    delivery3 = service.create_delivery(delivery_input_3, user_id="user1") # Another for user1

    print(f"Delivery 1: {delivery1.model_dump_json(indent=2)}")
    print(f"Delivery 2: {delivery2.model_dump_json(indent=2)}")


    print("\n--- Get Delivery by ID ---")
    retrieved_delivery1 = service.get_delivery_by_id(delivery1.id)
    print(f"Retrieved Delivery 1: {retrieved_delivery1.model_dump_json(indent=2) if retrieved_delivery1 else 'Not Found'}")
    retrieved_non_existent = service.get_delivery_by_id("non-existent-id")
    print(f"Retrieved Non-Existent: {retrieved_non_existent}")


    print("\n--- Update Delivery Status ---")
    updated_status_delivery1 = service.update_delivery_status(delivery1.id, DeliveryStatus.ASSIGNED)
    print(f"Updated Status Delivery 1: {updated_status_delivery1.model_dump_json(indent=2) if updated_status_delivery1 else 'Not Found'}")
    assert updated_status_delivery1.status == DeliveryStatus.ASSIGNED
    assert updated_status_delivery1.version == 2

    print("\n--- Update Delivery Details ---")
    details_update = DeliveryUpdate(robot_id="robot_001", item_description="Books and stationary")
    updated_details_delivery1 = service.update_delivery_details(delivery1.id, details_update)
    print(f"Updated Details Delivery 1: {updated_details_delivery1.model_dump_json(indent=2) if updated_details_delivery1 else 'Not Found'}")
    assert updated_details_delivery1.robot_id == "robot_001"
    assert updated_details_delivery1.version == 3


    print("\n--- Get Deliveries by User ---")
    user1_deliveries = service.get_deliveries_by_user("user1")
    print(f"User1 Deliveries ({len(user1_deliveries)}):")
    for d in user1_deliveries:
        print(f"  ID: {d.id}, Status: {d.status.value}")
    assert len(user1_deliveries) == 2

    user3_deliveries = service.get_deliveries_by_user("user3") # Non-existent user
    print(f"User3 Deliveries ({len(user3_deliveries)}):")
    assert len(user3_deliveries) == 0


    print("\n--- Cancel Delivery ---")
    cancelled_delivery2 = service.cancel_delivery(delivery2.id)
    print(f"Cancelled Delivery 2: {cancelled_delivery2.model_dump_json(indent=2) if cancelled_delivery2 else 'Not Found or Cannot Cancel'}")
    assert cancelled_delivery2.status == DeliveryStatus.CANCELLED
    
    # Test cancelling an already delivered order
    service.update_delivery_status(delivery1.id, DeliveryStatus.DELIVERED) # Mark delivery1 as delivered
    cannot_cancel_delivery1 = service.cancel_delivery(delivery1.id)
    print(f"Attempt to cancel delivered Delivery 1: Status is {cannot_cancel_delivery1.status.value if cannot_cancel_delivery1 else 'Not Found'}")
    assert cannot_cancel_delivery1.status == DeliveryStatus.DELIVERED # Should remain DELIVERED


    print(f"\nFinal DB state (length): {len(_deliveries_db)}")
    # for id, item in _deliveries_db.items():
    #    print(item.model_dump_json(indent=2))
```
