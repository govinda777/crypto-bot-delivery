import pytest
from pydantic import ValidationError

# Adjust import path based on project structure and how pytest discovers modules
# Assuming 'app' is discoverable (e.g., project installed with -e ., or PYTHONPATH in pyproject.toml).
from app.models.schemas.delivery import DeliveryBase, DeliveryUpdate

# Tests for DeliveryBase phone number validation
def test_delivery_base_phone_number_valid_simple_digits():
    data = DeliveryBase(pickup_address="1 Valid Pickup", dropoff_address="2 Valid Dropoff", pickup_phone_number="1234567890")
    assert data.pickup_phone_number == "1234567890"

def test_delivery_base_phone_number_valid_with_cleaning_and_plus():
    # Validator cleans non-digits and checks length of cleaned string
    data = DeliveryBase(pickup_address="p", dropoff_address="d", pickup_phone_number="+1 (123) 456-7890") 
    assert data.pickup_phone_number == "11234567890", "Should strip non-digits and keep '+' if it's at the start and then removed by filter"

def test_delivery_base_phone_number_valid_long_with_cleaning():
    data = DeliveryBase(pickup_address="p", dropoff_address="d", pickup_phone_number=" 123-456-7890-1234 ") # 14 digits with spaces/hyphens
    assert data.pickup_phone_number == "12345678901234", "Should strip and keep valid length"

def test_delivery_base_phone_number_none_is_valid():
    data = DeliveryBase(pickup_address="p", dropoff_address="d", pickup_phone_number=None)
    assert data.pickup_phone_number is None

def test_delivery_base_phone_number_invalid_too_short_after_cleaning():
    with pytest.raises(ValidationError) as excinfo:
        DeliveryBase(pickup_address="p", dropoff_address="d", pickup_phone_number="123-456") # 6 digits
    # Pydantic V2 error messages are more structured. Check for relevant part.
    assert any("Cleaned phone number must be 10-15 digits" in err['msg'] for err in excinfo.value.errors())


def test_delivery_base_phone_number_invalid_too_long_after_cleaning():
    with pytest.raises(ValidationError) as excinfo:
        DeliveryBase(pickup_address="p", dropoff_address="d", pickup_phone_number="123-456-7890-123-456-7890") # >15 digits
    assert any("Cleaned phone number must be 10-15 digits" in err['msg'] for err in excinfo.value.errors())
    
def test_delivery_base_phone_number_invalid_only_characters():
    # Validator: "".join(filter(str.isdigit, v)) would result in "" for "abc-def-ghi"
    # Then the `if v and not (10 <= len(cleaned_v) <= 15):` check.
    # If original `v` is not None/empty, but cleaned_v is empty, it should fail.
    # The condition `if not v and cleaned_v:` -> `if not "abc" and ""` is `False and True` -> `False`
    # The condition `if v and not (10 <= len(cleaned_v) <= 15):` -> `if "abc" and not (0 <= 10 <= 15)` -> `if "abc" and True` -> True, raises error.
    with pytest.raises(ValidationError) as excinfo:
        DeliveryBase(pickup_address="p", dropoff_address="d", pickup_phone_number="abc-def-ghij")
    assert any("Cleaned phone number must be 10-15 digits" in err['msg'] for err in excinfo.value.errors())


# Tests for DeliveryUpdate phone number validation (optional fields)
def test_delivery_update_phone_number_valid():
    data = DeliveryUpdate(pickup_phone_number="0987654321")
    assert data.pickup_phone_number == "0987654321"

def test_delivery_update_phone_number_valid_with_cleaning():
    data = DeliveryUpdate(dropoff_phone_number=" (098) 765-43210 ") # 11 digits with cruft
    assert data.dropoff_phone_number == "09876543210"

def test_delivery_update_phone_number_none_is_valid():
    data = DeliveryUpdate(pickup_phone_number=None, dropoff_phone_number=None)
    assert data.pickup_phone_number is None
    assert data.dropoff_phone_number is None

def test_delivery_update_phone_number_invalid_too_short():
    with pytest.raises(ValidationError) as excinfo:
        DeliveryUpdate(pickup_phone_number="123-45") # 5 digits
    assert any("Cleaned phone number must be 10-15 digits" in err['msg'] for err in excinfo.value.errors())

# Tests for Field constraints (min_length, max_length) on address fields

def test_delivery_base_pickup_address_min_length_fail():
    with pytest.raises(ValidationError) as excinfo:
        DeliveryBase(pickup_address="1", dropoff_address="Valid Street Name")
    assert any(err['type'] == 'string_too_short' and 'pickup_address' in err['loc'] for err in excinfo.value.errors()), \
        f"Error details: {excinfo.value.errors()}"


def test_delivery_base_pickup_address_max_length_fail():
    long_addr = "a" * 201 # max_length is 200
    with pytest.raises(ValidationError) as excinfo:
        DeliveryBase(pickup_address=long_addr, dropoff_address="Valid Street Name")
    assert any(err['type'] == 'string_too_long' and 'pickup_address' in err['loc'] for err in excinfo.value.errors()), \
        f"Error details: {excinfo.value.errors()}"

def test_delivery_base_dropoff_address_min_length_fail():
    with pytest.raises(ValidationError) as excinfo:
        DeliveryBase(pickup_address="Valid Pickup Address", dropoff_address="2St") # min_length is 5
    assert any(err['type'] == 'string_too_short' and 'dropoff_address' in err['loc'] for err in excinfo.value.errors()), \
        f"Error details: {excinfo.value.errors()}"

def test_delivery_base_dropoff_address_max_length_fail():
    long_addr = "b" * 201 # max_length is 200
    with pytest.raises(ValidationError) as excinfo:
        DeliveryBase(pickup_address="Valid Pickup Address", dropoff_address=long_addr)
    assert any(err['type'] == 'string_too_long' and 'dropoff_address' in err['loc'] for err in excinfo.value.errors()), \
        f"Error details: {excinfo.value.errors()}"

def test_delivery_base_item_description_max_length_fail():
    long_desc = "c" * 501 # max_length is 500
    with pytest.raises(ValidationError) as excinfo:
        DeliveryBase(pickup_address="Valid Pickup", dropoff_address="Valid Dropoff", item_description=long_desc)
    assert any(err['type'] == 'string_too_long' and 'item_description' in err['loc'] for err in excinfo.value.errors()), \
        f"Error details: {excinfo.value.errors()}"

def test_delivery_base_item_description_valid_none():
    data = DeliveryBase(pickup_address="Valid Pickup", dropoff_address="Valid Dropoff", item_description=None)
    assert data.item_description is None

def test_delivery_base_item_description_valid_string():
    desc = "A reasonable description."
    data = DeliveryBase(pickup_address="Valid Pickup", dropoff_address="Valid Dropoff", item_description=desc)
    assert data.item_description == desc

# Example of how to run these tests from the project root (crypto-bot-delivery):
# python -m pytest -v src/delivery_api/tests/unit/models/test_delivery_schemas.py
```
