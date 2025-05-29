import pytest
import os
import json
import time
from PIL import Image, ImageDraw  # For creating dummy/invalid images
import qrcode  # For direct QR generation in specific tests

# Assuming delivery_qr is installed in editable mode or path is configured
from delivery_qr.scanner import scan_qr_code_from_file
from delivery_qr.generator import (
    generate_qr_code,
)  # To create valid QR images for testing


@pytest.fixture
def sample_data_for_scanner():
    # Using a fixed timestamp for consistent comparison in success test
    return {
        "order_id": "scan_test_789",
        "type": "delivery",
        "timestamp": 1678886400.0,  # Fixed timestamp: e.g., 2023-03-15 12:00:00 UTC
        "token": "scan_token_xyz789",
    }


def test_scan_qr_code_success(tmp_path, sample_data_for_scanner):
    """Test successful scanning of a valid QR code image."""
    qr_image_file = tmp_path / "valid_qr_for_scan.png"
    qr_image_path_str = str(qr_image_file)

    gen_success = generate_qr_code(sample_data_for_scanner, qr_image_path_str)
    assert gen_success, "Setup: Failed to generate QR code for scanner test"
    assert qr_image_file.is_file(), "Setup: QR code image file was not created"

    decoded_data = scan_qr_code_from_file(qr_image_path_str)

    assert (
        decoded_data is not None
    ), "scan_qr_code_from_file returned None for a valid QR"
    assert (
        decoded_data == sample_data_for_scanner
    ), "Decoded data does not match original data"


def test_scan_qr_code_file_not_found(tmp_path):
    """Test scanning a non-existent QR code image file."""
    non_existent_file = tmp_path / "i_do_not_exist.png"
    non_existent_path_str = str(non_existent_file)

    decoded_data = scan_qr_code_from_file(non_existent_path_str)
    assert (
        decoded_data is None
    ), "scan_qr_code_from_file should return None for a non-existent file"


def test_scan_qr_code_not_a_qr_image(tmp_path):
    """Test scanning an image file that does not contain a valid QR code."""
    not_a_qr_file = tmp_path / "not_a_qr.png"
    not_a_qr_path_str = str(not_a_qr_file)

    try:
        img = Image.new("RGB", (200, 200), color="blue")  # Increased size
        draw = ImageDraw.Draw(img)
        draw.text(
            (20, 20), "This is just text, not a QR.", fill=(255, 255, 0)
        )  # Changed text and position
        img.save(not_a_qr_path_str)
    except Exception as e:
        pytest.fail(f"Setup: Failed to create dummy non-QR image: {e}")

    assert not_a_qr_file.is_file(), "Setup: Dummy non-QR image was not created"

    decoded_data = scan_qr_code_from_file(not_a_qr_path_str)
    assert (
        decoded_data is None
    ), "scan_qr_code_from_file should return None for an image without a QR code"


def test_scan_qr_code_qr_image_non_json_data(tmp_path):
    """Test scanning a QR code whose content is not valid JSON."""
    qr_image_file = tmp_path / "non_json_qr.png"
    qr_image_path_str = str(qr_image_file)

    try:
        qr_img_obj = qrcode.make("This is definitely not JSON data")
        qr_img_obj.save(qr_image_path_str)
    except Exception as e:
        pytest.fail(f"Setup: Failed to create non-JSON QR code: {e}")

    assert qr_image_file.is_file(), "Setup: Non-JSON QR image was not created"

    decoded_data = scan_qr_code_from_file(qr_image_path_str)
    assert (
        decoded_data is None
    ), "scan_qr_code_from_file should return None for QR with non-JSON content"


def test_scan_qr_code_empty_json_object_payload(tmp_path):
    """Test scanning a QR code that contains an empty JSON object string '{}'."""
    qr_image_file = tmp_path / "empty_json_object_qr.png"
    qr_image_path_str = str(qr_image_file)

    try:
        qr_img_obj = qrcode.make("{}")  # QR code with string "{}"
        qr_img_obj.save(qr_image_path_str)
        assert (
            qr_image_file.is_file()
        ), "Setup: QR with empty JSON object string was not created"
    except Exception as e:
        pytest.fail(f"Setup: Failed to create QR with empty JSON object string: {e}")

    decoded_data = scan_qr_code_from_file(qr_image_path_str)
    assert (
        decoded_data == {}
    ), "Decoded data for an empty JSON object QR should be an empty dict"


def test_scan_qr_code_qr_with_empty_string_payload(tmp_path):
    """Test scanning a QR code that contains an actual empty string.
    This should lead to a JSONDecodeError as an empty string is not valid JSON.
    """
    qr_image_file = tmp_path / "empty_string_payload_qr.png"
    qr_image_path_str = str(qr_image_file)

    try:
        qr_img_obj = qrcode.make("")  # QR code with an empty string
        qr_img_obj.save(qr_image_path_str)
        assert (
            qr_image_file.is_file()
        ), "Setup: QR with empty string payload was not created"
    except Exception as e:
        pytest.fail(f"Setup: Failed to create QR with empty string: {e}")

    decoded_data = scan_qr_code_from_file(qr_image_path_str)
    assert (
        decoded_data is None
    ), "scan_qr_code_from_file should return None for QR with empty string payload (not valid JSON)"


# To run these tests from the project root (crypto-bot-delivery):
# 1. Ensure pytest and other dependencies are installed (e.g., pip install pytest Pillow pyzbar qrcode)
# 2. Ensure project is installed in editable mode (pip install -e .) if not done already.
# 3. Run: python -m pytest
# or for more verbose output: python -m pytest -v src/delivery_qr/tests/test_scanner.py
