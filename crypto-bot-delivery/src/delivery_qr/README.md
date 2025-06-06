# Delivery QR Module (`delivery_qr`)

This module is responsible for generating and processing QR codes used in the Crypto-Bot-Delivery system for confirming pickup and delivery events.

## Overview

The `delivery_qr` module provides two main functionalities:
1.  **QR Code Generation:** Creating QR code images that embed essential, secure information for a given delivery step.
2.  **QR Code Scanning (Simulation):** Decoding QR codes from image files to retrieve the embedded data.

The primary goal is to facilitate a secure and verifiable way to confirm key stages of the delivery process.

## Components

*   **`generator.py`:**
    *   Contains the `generate_qr_code(data: dict, image_path: str) -> bool` function.
    *   This function takes a dictionary of data, serializes it to JSON, and then generates a QR code image, saving it to the specified `image_path`.
    *   The embedded `data` typically includes:
        *   `order_id` (str): The unique identifier for the delivery order.
        *   `type` (str): The type of confirmation (e.g., "pickup", "delivery").
        *   `timestamp` (float): The Unix timestamp of when the QR data was generated.
        *   `token` (str): A unique, secure token/nonce associated with this specific confirmation event. This token is the primary means of validating the QR code's authenticity at this stage of the POC.
*   **`scanner.py`:**
    *   Contains the `scan_qr_code_from_file(image_path: str) -> dict | None` function.
    *   This function simulates scanning a QR code by reading an image file, using `pyzbar` to detect and decode any QR codes present, and then deserializing the JSON data found within the QR code back into a Python dictionary.
    *   It returns the dictionary if successful, or `None` if no QR code is found, the image is invalid, or the data cannot be parsed.

## Security Considerations

*   **Token-Based Validation:** In the current implementation, the primary security mechanism for QR code validation relies on the `token` embedded within the QR data.
    *   This token should be unique and cryptographically secure (e.g., a UUID or a random string generated by a trusted service, like the Backend API).
    *   When a QR code is "scanned" (e.g., by the robot or client application), the extracted token is sent to the Backend API.
    *   The API is responsible for validating this token against an expected value stored securely for that specific `order_id` and confirmation `type`.
*   **Data Integrity:** While the JSON serialization provides structure, there is no cryptographic signature embedded directly within the QR code by this module to protect against tampering *of the QR data itself* after generation (if someone were to generate their own QR with different data but a known valid token format). The security relies on the unguessability and server-side validation of the `token`.
*   **Future Enhancements:** For higher security, the data to be embedded in the QR code could be signed by a trusted entity (e.g., the API) before being passed to `generate_qr_code`. The `token` itself could be a signed JWT or a similar construct. The `scanner` module or a subsequent validation step would then also need to verify this signature. This was deferred for simplicity in the initial POC.

## Data Flow for QR Confirmation (Conceptual)

The following Mermaid diagram illustrates the conceptual flow of QR code generation and validation:

```mermaid
graph TD
    A[Backend API: Generate Confirmation Token for OrderID + Type] --> B(Robot/Client App: Has OrderID, Type, Token);
    B -- Uses data --> C[delivery_qr.generator: generate_qr_code()];
    C --> D[QR Code Image (.png) Created];
    
    subgraph Confirmation Event
        E[Physical Scan: Other Party Scans QR Code Image] --> F[Extracted QR Data (JSON string)];
    end

    F -- Simulated by --> G[delivery_qr.scanner: scan_qr_code_from_file()];
    G --> H{Decoded Data (dict with order_id, type, token, timestamp)};
    H -- Send to --> I[Backend API: Validate Token for OrderID & Type];
    I -- Valid? --> J{Decision};
    J -- Yes --> K[API: Proceed with Blockchain Confirmation (e.g., call EscrowContract)];
    J -- No --> L[API: Reject Confirmation];
```

## Usage

The `generator.py` and `scanner.py` scripts include `if __name__ == '__main__':` blocks that demonstrate their basic usage. 

For more comprehensive testing, including unit tests and BDD scenarios, refer to the tests within the `src/delivery_qr/tests/` directory. These can be run using `pytest`.
```
