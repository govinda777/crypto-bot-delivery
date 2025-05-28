import pytest
from pytest_bdd import scenarios, given, when, then, parsers
import os
import time
import json # For loading decoded data if it's stringified JSON by scanner
from PIL import Image, ImageDraw # For creating dummy non-QR images

# Assuming delivery_qr is installed in editable mode or path is configured
from delivery_qr.generator import generate_qr_code
from delivery_qr.scanner import scan_qr_code_from_file

# --- Load Scenarios ---
# Ensure this path is correct relative to the step_defs directory
scenarios('../features/qr_confirmation.feature')


# --- Fixtures ---

@pytest.fixture
def order_id_fixture():
    return "order_default_bdd_123" # Default, can be overridden by scenario

@pytest.fixture
def qr_image_dir_fixture(tmp_path_factory):
    # Create a unique temporary directory for each test function (scenario)
    return tmp_path_factory.mktemp("qr_images_bdd")

@pytest.fixture
def scenario_context():
    # Shared context for steps within a scenario
    return {}

# --- Given Steps ---

@given(parsers.parse('a unique order ID "{order_id}"'), target_fixture='order_id_fixture')
def given_unique_order_id(order_id):
    return order_id # This new order_id will be used by other fixtures/steps

@given('a temporary directory for QR code images')
def given_temp_qr_dir(qr_image_dir_fixture, scenario_context):
    scenario_context['qr_image_dir'] = str(qr_image_dir_fixture)
    assert os.path.exists(scenario_context['qr_image_dir'])

@given(parsers.parse('the confirmation type is "{conf_type}"'))
def given_confirmation_type(scenario_context, conf_type):
    scenario_context['confirmation_type'] = conf_type

@given(parsers.parse('a unique token "{token}" is generated for the order'))
def given_unique_token(scenario_context, token):
    scenario_context['token'] = token

@given('a non-QR image file exists')
def given_non_qr_image(scenario_context):
    temp_dir = scenario_context['qr_image_dir']
    non_qr_image_path = os.path.join(temp_dir, "not_a_real_qr.png")
    try:
        img = Image.new('RGB', (100, 100), color='blue')
        draw = ImageDraw.Draw(img)
        draw.text((10,10), "Not a QR", fill=(255,255,0))
        img.save(non_qr_image_path)
        scenario_context['non_qr_image_path'] = non_qr_image_path
        assert os.path.exists(non_qr_image_path)
    except Exception as e:
        pytest.fail(f"Failed to create dummy non-QR image: {e}")

# --- When Steps ---

@when(parsers.parse('a QR code is generated for "{order_id}" with type "{conf_type}" and token "{token}"'))
def when_qr_code_generated(scenario_context, order_id_fixture, conf_type, token): # Use order_id_fixture
    data_to_encode = {
        "order_id": order_id_fixture, # Use the one from the fixture, potentially set by scenario
        "type": conf_type,
        "timestamp": time.time(),
        "token": token
    }
    filename = f"{order_id_fixture}_{conf_type}_qr.png"
    image_path = os.path.join(scenario_context['qr_image_dir'], filename)
    
    success = generate_qr_code(data_to_encode, image_path)
    assert success, f"Failed to generate QR code for {order_id_fixture}"
    scenario_context['generated_qr_image_path'] = image_path
    scenario_context['generated_qr_type'] = conf_type # Store for later scanning step
    # Store the data that was encoded for later direct comparison if needed
    scenario_context['generated_qr_data_input'] = data_to_encode


@when(parsers.parse('the generated "{qr_type}" QR code image is scanned'))
def when_generated_qr_scanned(scenario_context, qr_type):
    # This step assumes the QR type matches what was stored, or could be more generic
    assert scenario_context.get('generated_qr_type') == qr_type, "QR type mismatch in test step"
    image_path = scenario_context['generated_qr_image_path']
    decoded_data = scan_qr_code_from_file(image_path)
    scenario_context['decoded_qr_data'] = decoded_data

@when('the non-QR image file is scanned')
def when_non_qr_scanned(scenario_context):
    image_path = scenario_context['non_qr_image_path']
    decoded_data = scan_qr_code_from_file(image_path)
    scenario_context['decoded_qr_data'] = decoded_data # Should be None

# --- Then Steps ---

@then('a QR code image file should be created')
def then_qr_image_file_created(scenario_context):
    image_path = scenario_context['generated_qr_image_path']
    assert os.path.exists(image_path), f"QR image file not found at {image_path}"
    assert os.path.getsize(image_path) > 0, f"QR image file is empty at {image_path}"

@then(parsers.parse('the decoded QR data should contain "{expected_order_id}", type "{expected_type}", and token "{expected_token}"'))
def then_decoded_data_matches(scenario_context, expected_order_id, expected_type, expected_token):
    decoded_data = scenario_context.get('decoded_qr_data')
    assert decoded_data is not None, "QR data was not decoded (None)"
    assert decoded_data.get('order_id') == expected_order_id
    assert decoded_data.get('type') == expected_type
    assert decoded_data.get('token') == expected_token
    # Compare against the input data for timestamp consistency if needed, excluding timestamp if it's highly dynamic
    # For this test, the timestamp in sample_qr_data_for_scanner was fixed.
    # Here, the generated timestamp is dynamic in when_qr_code_generated.
    # So we should check for its presence and type.
    assert 'timestamp' in decoded_data, "Timestamp missing from decoded data"
    assert isinstance(decoded_data['timestamp'], (float, int)), "Timestamp is not a valid number"

    # More robust check: compare against the data that was actually encoded
    original_encoded_data = scenario_context.get('generated_qr_data_input')
    if original_encoded_data:
        assert decoded_data.get('order_id') == original_encoded_data.get('order_id')
        assert decoded_data.get('type') == original_encoded_data.get('type')
        assert decoded_data.get('token') == original_encoded_data.get('token')
        # Timestamps will be slightly different due to float precision and generation time vs. test time.
        # It's better to check if it's close or just check type/presence as above.
        # assert abs(decoded_data['timestamp'] - original_encoded_data['timestamp']) < 0.1 # Example for closeness


@then(parsers.parse('the decoded QR data contains token "{expected_token}"'))
def then_decoded_data_contains_token(scenario_context, expected_token):
    decoded_data = scenario_context.get('decoded_qr_data')
    assert decoded_data is not None, "QR data was not decoded (None)"
    assert decoded_data.get('token') == expected_token

@then(parsers.parse('the system expects token "{expected_token}" for "{order_id}" type "{conf_type}"'))
def then_system_expects_token(scenario_context, expected_token, order_id, conf_type):
    # This step is for setting up the expectation for the next validation step
    # It doesn't perform an action itself but stores the expectation.
    scenario_context['expected_system_token'] = expected_token
    # Could also verify order_id and conf_type match current context if needed for test logic.
    assert scenario_context['order_id_fixture'] == order_id, "Order ID mismatch in expectation step"
    assert scenario_context['confirmation_type'] == conf_type, "Confirmation type mismatch in expectation step"


@then('the QR data validation should fail due to token mismatch')
def then_validation_fails_token_mismatch(scenario_context):
    decoded_data = scenario_context.get('decoded_qr_data')
    assert decoded_data is not None, "QR data was not decoded (None)"
    
    decoded_token = decoded_data.get('token')
    expected_system_token = scenario_context.get('expected_system_token')
    
    assert decoded_token is not None, "Decoded token is missing from QR data"
    assert expected_system_token is not None, "Expected system token was not set in test context"
    assert decoded_token != expected_system_token, \
        f"Token validation should fail, but decoded token '{decoded_token}' matched expected '{expected_system_token}'"

@then('the QR scanning should fail or return no usable data')
def then_scanning_fails_no_data(scenario_context):
    decoded_data = scenario_context.get('decoded_qr_data')
    assert decoded_data is None, "QR scanning was expected to fail or return None, but it returned data."

# To run these tests from the project root (crypto-bot-delivery):
# 1. Ensure pytest, pytest-bdd and other dependencies are installed 
#    (e.g., pip install pytest pytest-bdd Pillow qrcode)
# 2. Ensure project is installed in editable mode (pip install -e .) if not done already.
# 3. Run: python -m pytest
# or for more verbose output: python -m pytest -v src/delivery_qr/tests/step_defs/test_qr_confirmation_steps.py
```
