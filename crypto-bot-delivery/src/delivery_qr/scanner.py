from PIL import Image
from pyzbar.pyzbar import decode
import json
import os # For example usage
import time # For example usage, if generating QRs

# Ensure 'pyzbar' and 'Pillow' are installed: pip install pyzbar Pillow

def scan_qr_code_from_file(image_path: str) -> dict | None:
    """
    "Scans" a QR code from an image file and returns the decoded data.

    Args:
        image_path (str): Full path to the QR code image file.

    Returns:
        dict | None: A dictionary containing the decoded data if successful, 
                      None otherwise.
    """
    try:
        # Open the image file
        img = Image.open(image_path)

        # Decode QR codes from the image
        decoded_objects = decode(img)

        if decoded_objects:
            # Assume the first QR code found is the one we want
            qr_data_bytes = decoded_objects[0].data
            
            # Decode bytes to string (assuming UTF-8)
            qr_data_str = qr_data_bytes.decode('utf-8')
            
            # Deserialize JSON string to dictionary
            try:
                data_dict = json.loads(qr_data_str)
                # print(f"Successfully decoded QR data: {data_dict}") # Optional
                return data_dict
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON from QR data: {e}. Data was: '{qr_data_str}'")
                return None
        else:
            print(f"No QR code found in image: {image_path}")
            return None
    except FileNotFoundError:
        print(f"Error: Image file not found at {image_path}")
        return None
    except Exception as e:
        print(f"An error occurred during QR code scanning: {e}")
        return None

if __name__ == '__main__':
    # This example assumes generator.py's example created some QR codes
    # in 'output_qrs_generated_examples' directory.
    # Ensure you run generator.py's example first if you want to test this directly.

    example_dir = "output_qrs_generated_examples" 
    # To make this example runnable standalone, we might need to call the generator
    # or assume the files exist. 
    
    # Attempt to generate sample QR codes for this scanner example if they don't exist
    # This tries to make the __main__ block more self-contained for demonstration.
    pickup_qr_path = os.path.join(example_dir, "scan_pickup_example.png")
    delivery_qr_path = os.path.join(example_dir, "scan_delivery_example.png")

    if not (os.path.exists(pickup_qr_path) and os.path.exists(delivery_qr_path)):
        print("Sample QR files for scanner not found, attempting to generate them...")
        if not os.path.exists(example_dir):
            os.makedirs(example_dir)
        try:
            # Attempt to import the generator function.
            # This works if scanner.py and generator.py are in the same package (delivery_qr)
            # and the package is installed or PYTHONPATH is set up correctly.
            from .generator import generate_qr_code 
            # If running as a script directly from its directory, simple import might fail.
            # Fallback for direct script execution if relative import fails:
        except ImportError:
            # This is a common issue if running a file within a package directly as __main__
            # For a robust __main__ in a package, often more complex path manipulation or
            # running via `python -m delivery_qr.scanner` is needed.
            # For this example, we'll just note if the import fails.
            print("Could not import 'generate_qr_code' for self-contained example. "
                  "Please ensure QR files exist or run generator.py manually.")
            generate_qr_code = None # Ensure it's defined to avoid NameError later

        if generate_qr_code: # Proceed if import was successful
            sample_data_pickup = {
                "order_id": "scan_test_pickup_123", "type": "pickup", 
                "timestamp": time.time(), "token": "scan_token_pickup_xyz"
            }
            if generate_qr_code(sample_data_pickup, pickup_qr_path):
                 print(f"Generated sample: {pickup_qr_path}")
            
            sample_data_delivery = {
                "order_id": "scan_test_delivery_456", "type": "delivery", 
                "timestamp": time.time(), "token": "scan_token_delivery_abc"
            }
            if generate_qr_code(sample_data_delivery, delivery_qr_path):
                print(f"Generated sample: {delivery_qr_path}")
        else:
            print("Skipping sample QR generation as generator could not be imported.")


    non_existent_path = os.path.join(example_dir, "non_existent_qr.png")

    print(f"\nAttempting to scan pickup QR from: {os.path.abspath(pickup_qr_path)}")
    if os.path.exists(pickup_qr_path):
        pickup_data = scan_qr_code_from_file(pickup_qr_path)
        if pickup_data:
            print(f"Decoded Pickup QR Data: {pickup_data}")
            assert pickup_data.get("type") == "pickup" # Basic assertion for example
        else:
            print(f"Failed to decode pickup QR from {pickup_qr_path}")
    else:
        print(f"Pickup QR file not found at {pickup_qr_path}. Run generator.py example first or ensure path is correct.")

    print(f"\nAttempting to scan delivery QR from: {os.path.abspath(delivery_qr_path)}")
    if os.path.exists(delivery_qr_path):
        delivery_data = scan_qr_code_from_file(delivery_qr_path)
        if delivery_data:
            print(f"Decoded Delivery QR Data: {delivery_data}")
            assert delivery_data.get("type") == "delivery" # Basic assertion for example
        else:
            print(f"Failed to decode delivery QR from {delivery_qr_path}")
    else:
        print(f"Delivery QR file not found at {delivery_qr_path}. Run generator.py example first or ensure path is correct.")

    print("\nAttempting to scan a non-existent QR file:")
    scan_qr_code_from_file(non_existent_path) # Expected to fail and print "File not found" error
```
