import qrcode
import json
import time  # For default timestamp if needed, or caller provides it
import os  # For example usage

# Ensure 'qrcode' and 'Pillow' are installed: pip install qrcode Pillow


def generate_qr_code(data: dict, image_path: str) -> bool:
    """
    Generates a QR code image from the provided data and saves it to image_path.

    Args:
        data (dict): Data to embed. Expected keys: 'order_id', 'type',
                     'timestamp', 'token'.
        image_path (str): Full path to save the QR code image (e.g., /path/to/qr.png).

    Returns:
        bool: True if QR code was generated and saved successfully, False otherwise.
    """
    required_keys = ["order_id", "type", "timestamp", "token"]
    if not all(key in data for key in required_keys):
        print(f"Error: Data missing one or more required keys: {required_keys}")
        return False

    try:
        # Serialize data to JSON string
        json_data = json.dumps(data, sort_keys=True)

        # Create QR code instance
        qr = qrcode.QRCode(
            version=1,  # Auto-adjusts if None, or set specific size
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=10,  # Size of each box in pixels
            border=4,  # Thickness of the border
        )
        qr.add_data(json_data)
        qr.make(fit=True)

        # Create an image from the QR Code instance
        img = qr.make_image(fill_color="black", back_color="white")

        # Save the image
        img.save(image_path)
        # print(f"QR code successfully generated and saved to {image_path}") # Optional: reduce noise for library use
        return True
    except json.JSONEncodeError as e:
        print(f"Error encoding data to JSON: {e}")
        return False
    except Exception as e:
        print(f"An error occurred during QR code generation or saving: {e}")
        return False


if __name__ == "__main__":
    # Example Usage:
    sample_data_pickup = {
        "order_id": "order_xyz_789",
        "type": "pickup",
        "timestamp": time.time(),
        "token": "pickup_nonce_abcdef123456",
    }
    sample_data_delivery = {
        "order_id": "order_xyz_789",
        "type": "delivery",
        "timestamp": time.time(),
        "token": "delivery_nonce_uvwxyz789012",
    }

    output_dir = "output_qrs_generated_examples"  # Changed dir name to avoid conflict if it exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    pickup_path = os.path.join(output_dir, "pickup_qr_example.png")
    delivery_path = os.path.join(output_dir, "delivery_qr_example.png")

    print(f"Attempting to generate pickup QR to: {os.path.abspath(pickup_path)}")
    if generate_qr_code(sample_data_pickup, pickup_path):
        print(f"Pickup QR generated for {sample_data_pickup['order_id']}")
    else:
        print(f"Failed to generate pickup QR for {sample_data_pickup['order_id']}")

    print(f"Attempting to generate delivery QR to: {os.path.abspath(delivery_path)}")
    if generate_qr_code(sample_data_delivery, delivery_path):
        print(f"Delivery QR generated for {sample_data_delivery['order_id']}")
    else:
        print(f"Failed to generate delivery QR for {sample_data_delivery['order_id']}")

    # To test error for missing keys
    # if not generate_qr_code({"order_id": "test"}, "error_qr.png"):
    #     print("Error QR generation failed as expected due to missing keys.")
