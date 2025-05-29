from pydantic import BaseModel, Field, validator, ConfigDict
from enum import Enum
from datetime import datetime
import uuid # For generating default IDs if needed by service layer later

class DeliveryStatus(str, Enum):
    PENDING = "pending"
    ASSIGNED = "assigned" # Assigned to a robot
    IN_TRANSIT_PICKUP = "in_transit_pickup" # Robot en route to pickup
    AT_PICKUP = "at_pickup" # Robot at pickup location
    IN_TRANSIT_DELIVERY = "in_transit_delivery" # Robot en route to delivery
    AT_DELIVERY = "at_delivery" # Robot at delivery location
    DELIVERED = "delivered"
    CANCELLED = "cancelled"
    FAILED = "failed" # Delivery attempt failed

class DeliveryBase(BaseModel):
    pickup_address: str = Field(..., min_length=5, max_length=200, examples=["123 Main St, Anytown, USA"])
    dropoff_address: str = Field(..., min_length=5, max_length=200, examples=["456 Oak Ave, Otherville, USA"])
    item_description: str | None = Field(None, max_length=500, examples=["Box of documents"])
    pickup_phone_number: str | None = Field(None, examples=["+15551234567"])
    dropoff_phone_number: str | None = Field(None, examples=["+15557654321"])

    # Example validator for phone numbers
    @validator('pickup_phone_number', 'dropoff_phone_number', pre=True, always=True)
    def validate_phone_number(cls, v):
        if v is None:
            return v
        # Simple validation: digits only, between 10 and 15 chars.
        # More complex regex could be used: ^\+?[1-9]\d{1,14}$
        # Allow stripping non-digits for flexibility
        cleaned_v = "".join(filter(str.isdigit, v))
        if v and not (10 <= len(cleaned_v) <= 15): # Check length only if original v was not empty
             raise ValueError('Cleaned phone number must be 10-15 digits.')
        if not v and cleaned_v: # If original v was e.g. "abc" -> cleaned_v is "" but original was not None
             raise ValueError('Phone number contains no digits.')
        return cleaned_v if v else None # Return cleaned version, or None if original was None/empty after clean

class DeliveryCreate(DeliveryBase):
    # Potentially add fields specific to creation if not covered by user context (e.g. user_id if not from token)
    # For now, assume DeliveryBase is sufficient for initial user input.
    pass

class DeliveryUpdate(BaseModel): # Not inheriting DeliveryBase to allow all fields to be optional
    pickup_address: str | None = Field(None, min_length=5, max_length=200)
    dropoff_address: str | None = Field(None, min_length=5, max_length=200)
    item_description: str | None = Field(None, max_length=500)
    pickup_phone_number: str | None = Field(None)
    dropoff_phone_number: str | None = Field(None)
    robot_id: str | None = Field(None, examples=["robot_007"])
    status: DeliveryStatus | None = None # Allow status update through general update too

    # Validator for phone numbers, only if provided
    @validator('pickup_phone_number', 'dropoff_phone_number', pre=True, always=True)
    def validate_optional_phone_number(cls, v):
        if v is None:
            return v
        cleaned_v = "".join(filter(str.isdigit, v))
        if v and not (10 <= len(cleaned_v) <= 15):
             raise ValueError('Cleaned phone number must be 10-15 digits.')
        if not v and cleaned_v: 
             raise ValueError('Phone number contains no digits.')
        return cleaned_v if v else None

class DeliveryStatusUpdate(BaseModel):
    status: DeliveryStatus

class Delivery(DeliveryBase):
    # For responses, including fields that are system-generated
    id: str = Field(..., examples=["d1b6e6a5-79e9-4E2F-8A5D-6C8A5C1A0B1E"])
    status: DeliveryStatus = Field(..., examples=[DeliveryStatus.PENDING])
    user_id: str = Field(..., examples=["user_abc_123"]) # User who owns the delivery
    robot_id: str | None = Field(None, examples=["robot_007"])
    created_at: datetime
    updated_at: datetime
    version: int = Field(1, examples=[1]) # For optimistic concurrency control

    # Pydantic V2 way to enable ORM mode / from_attributes
    model_config = ConfigDict(from_attributes=True)

if __name__ == '__main__':
    # Example Usage & Validation Test
    print("--- DeliveryStatus Enum ---")
    for status_member in DeliveryStatus:
        print(f"Status: {status_member.name}, Value: {status_member.value}")
    
    print("\n--- DeliveryBase Validation ---")
    try:
        base_valid = DeliveryBase(
            pickup_address="123 Valid Rd, Anytown", 
            dropoff_address="456 Safe St, Otherville", 
            pickup_phone_number="1234567890" # Valid
        )
        print(f"Base valid: {base_valid.model_dump_json(indent=2)}")
        
        base_valid_plus = DeliveryBase(
            pickup_address="123 Valid Rd, Anytown", 
            dropoff_address="456 Safe St, Otherville", 
            pickup_phone_number="+11234567890" # Valid with +
        )
        print(f"Base valid (with +): {base_valid_plus.model_dump_json(indent=2)}")
        assert base_valid_plus.pickup_phone_number == "11234567890"


        print("\nTesting invalid phone (too short):")
        base_invalid_phone_short = DeliveryBase(
            pickup_address="123 Valid Rd", 
            dropoff_address="456 Safe St", 
            pickup_phone_number="123"
        )
        print(f"This should not print if validation works: {base_invalid_phone_short}") 
    except ValueError as e:
        print(f"Caught Validation Error for Base (short phone): {e}")

    try:
        print("\nTesting invalid phone (non-digits):")
        base_invalid_phone_text = DeliveryBase(
            pickup_address="123 Valid Rd", 
            dropoff_address="456 Safe St", 
            pickup_phone_number="abcdefghij" # 10 chars, but not digits
        )
        print(f"This should not print if validation works: {base_invalid_phone_text}")
    except ValueError as e:
        print(f"Caught Validation Error for Base (text phone): {e}")
        
    try:
        print("\nTesting invalid phone (mixed, too short after clean):")
        base_invalid_phone_mixed_short = DeliveryBase(
            pickup_address="123 Valid Rd", 
            dropoff_address="456 Safe St", 
            pickup_phone_number="123-abc-45" 
        )
        print(f"This should not print if validation works: {base_invalid_phone_mixed_short}")
    except ValueError as e:
        print(f"Caught Validation Error for Base (mixed short phone): {e}")


    print("\n--- DeliveryCreate ---")
    create_data = DeliveryCreate(
        pickup_address="789 New Ave, Creation City", 
        dropoff_address="101 Old Path, Creation City", 
        item_description="Important Package for Creation"
    )
    print(f"Create data: {create_data.model_dump_json(indent=2)}")

    print("\n--- DeliveryUpdate ---")
    update_data_partial = DeliveryUpdate(item_description="Very Important Package", status=DeliveryStatus.ASSIGNED)
    print(f"Update data partial: {update_data_partial.model_dump_json(indent=2)}")
    
    update_data_full = DeliveryUpdate(
        pickup_address="123 Updated Pickup St",
        dropoff_address="456 Updated Dropoff Ave",
        item_description="Updated description",
        pickup_phone_number="0987654321",
        dropoff_phone_number="+11122334455", # Test with +
        robot_id="robot_assigned_01",
        status=DeliveryStatus.IN_TRANSIT_PICKUP
    )
    print(f"Update data full: {update_data_full.model_dump_json(indent=2)}")
    assert update_data_full.dropoff_phone_number == "11122334455"


    print("\n--- Delivery (Response Model) ---")
    current_time = datetime.utcnow()
    response_data = Delivery(
        id=str(uuid.uuid4()),
        pickup_address="Final Dest P, Response City",
        dropoff_address="Final Dest D, Response City",
        item_description="Final Item for Response",
        status=DeliveryStatus.DELIVERED,
        user_id="user_final_123",
        robot_id="robot_final_007",
        created_at=current_time, # Use actual datetime objects
        updated_at=current_time, # Use actual datetime objects
        version=2
    )
    print(f"Response data: {response_data.model_dump_json(indent=2)}")

    print("\n--- DeliveryStatusUpdate ---")
    status_update = DeliveryStatusUpdate(status=DeliveryStatus.AT_PICKUP)
    print(f"Status Update: {status_update.model_dump_json(indent=2)}")

    print("\n--- All Example Model Instantiations Done ---")
```
