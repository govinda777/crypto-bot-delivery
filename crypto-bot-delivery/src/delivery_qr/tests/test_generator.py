import pytest
import os
import json
import time
from PIL import Image # For checking image properties or direct decoding
from pyzbar.pyzbar import decode as pyzbar_decode # For content verification
from unittest.mock import patch

# Assuming delivery_qr is installed in editable mode or path is configured
# This should work due to pyproject.toml and editable install.
from delivery_qr.generator import generate_qr_code

@pytest.fixture
def sample_qr_data():
    return {
        "order_id": "test_order_123",
        "type": "pickup",
        "timestamp": time.time(), # Keep timestamp dynamic for realism if desired
        "token": "test_token_abcdef"
    }

def test_generate_qr_code_success(tmp_path, sample_qr_data):
    """Test successful QR code generation and content verification."""
    image_file = tmp_path / "test_qr_success.png"
    image_path_str = str(image_file)

    result = generate_qr_code(sample_qr_data, image_path_str)
    assert result is True, "generate_qr_code should return True on success"
    assert image_file.is_file(), "QR code image file should be created"

    # Verify content by decoding
    try:
        img = Image.open(image_path_str)
        decoded_objects = pyzbar_decode(img)
        assert len(decoded_objects) > 0, "No QR code found in the generated image"
        
        # Assuming only one QR code in the image
        decoded_data_str = decoded_objects[0].data.decode('utf-8')
        decoded_json_content = json.loads(decoded_data_str)
        
        # The generator sorts keys before dumping to JSON, so comparison should be direct
        assert decoded_json_content == sample_qr_data, "Decoded QR content does not match original data"
    except Exception as e:
        pytest.fail(f"Failed to decode or verify QR content: {e}")


def test_generate_qr_code_missing_keys(tmp_path):
    """Test QR code generation failure when required data keys are missing."""
    image_file = tmp_path / "test_qr_missing_keys.png"
    image_path_str = str(image_file)
    
    invalid_data = {
        "order_id": "test_order_456",
        # 'type' is missing
        "timestamp": time.time(),
        "token": "test_token_missing_type"
    }
    
    result = generate_qr_code(invalid_data, image_path_str)
    assert result is False, "generate_qr_code should return False for data with missing keys"
    assert not image_file.exists(), "Image file should not be created when keys are missing"

def test_generate_qr_code_empty_data(tmp_path):
    """Test QR code generation failure with an entirely empty data dictionary."""
    image_file = tmp_path / "test_qr_empty_data.png"
    image_path_str = str(image_file)
    empty_data = {} # Empty dictionary
    
    result = generate_qr_code(empty_data, image_path_str)
    assert result is False, "generate_qr_code should return False for empty data"
    assert not image_file.exists(), "Image file should not be created for empty data"

def test_generate_qr_code_invalid_save_path(sample_qr_data):
    """Test QR code generation failure when provided with an invalid save path."""
    # This path is likely invalid for writing without special permissions or if the parent dir doesn't exist.
    invalid_path = "/this/path/is/hopefully/nonexistent/test_qr_invalid.png"
    
    # A more robust way to test this might be to try to save to a path known to be read-only,
    # or a path that exceeds max length, but that's OS-dependent.
    # For now, we rely on the OS to prevent writing to a deep, non-existent directory.
    
    # Check if the directory exists to prevent false positives if the test environment is unusual
    # For example, if /this/path/is/hopefully/nonexistent/ *does* somehow exist and is writable.
    # This test is more about the function's error handling than the OS filesystem specifics.
    if os.path.exists(os.path.dirname(invalid_path)) and os.access(os.path.dirname(invalid_path), os.W_OK):
         pytest.skip(f"Skipping invalid path test as parent directory of test path '{os.path.dirname(invalid_path)}' unexpectedly exists and is writable.")

    result = generate_qr_code(sample_qr_data, invalid_path)
    assert result is False, "generate_qr_code should return False for an invalid save path"

@patch('json.dumps')
def test_generate_qr_code_json_encode_error(mock_json_dumps, tmp_path, sample_qr_data):
    """Test QR code generation failure if json.dumps raises an error."""
    # Configure the mock to raise JSONEncodeError when json.dumps is called
    mock_json_dumps.side_effect = json.JSONEncodeError("Mocked JSON encoding error", "doc", 0)
    
    image_file = tmp_path / "test_qr_json_error.png"
    image_path_str = str(image_file)
    
    result = generate_qr_code(sample_qr_data, image_path_str)
    assert result is False, "generate_qr_code should return False when json.dumps fails"
    assert not image_file.exists(), "Image file should not be created if JSON encoding fails"

# To run these tests from the project root (crypto-bot-delivery):
# 1. Ensure pytest and other dependencies are installed (e.g., pip install pytest Pillow pyzbar)
# 2. Ensure project is installed in editable mode (pip install -e .) if not done already.
# 3. Run: python -m pytest
# or for more verbose output: python -m pytest -v src/delivery_qr/tests/test_generator.py
```
