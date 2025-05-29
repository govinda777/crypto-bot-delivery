from fastapi import APIRouter, Depends, HTTPException, Header, Query
from typing import List

# Assuming models and services are structured as per previous steps
# Adjust import paths if necessary based on actual project structure and PYTHONPATH
# e.g., from ....models.schemas.delivery import ...
# e.g., from ....services.delivery_service import DeliveryService
from app.models.schemas.delivery import (
    Delivery,
    DeliveryCreate,
    DeliveryUpdate,
    DeliveryStatusUpdate,
    DeliveryStatus
)
from app.services.delivery_service import DeliveryService
from app.core.security import get_current_user, User # Added for JWT authentication

router = APIRouter(
    prefix="/deliveries",
    tags=["Deliveries"],
    responses={404: {"description": "Delivery not found"}}
)

# Dependency to get the service instance
def get_delivery_service() -> DeliveryService:
    # In a more complex app, this might involve context managers or other DI frameworks
    return DeliveryService()

@router.post("/", response_model=Delivery, status_code=201, summary="Create a new delivery")
async def create_new_delivery(
    delivery_data: DeliveryCreate,
    service: DeliveryService = Depends(get_delivery_service),
    current_user: User = Depends(get_current_user) # Protected: get user from token
):
    # The service method already handles printing conceptual blockchain interaction
    # User ID now comes from the authenticated user's token subject (username)
    return service.create_delivery(delivery_data=delivery_data, user_id=current_user.username)

@router.get("/{delivery_id}", response_model=Delivery, summary="Get a specific delivery by ID")
async def get_delivery_by_id_endpoint(
    delivery_id: str,
    service: DeliveryService = Depends(get_delivery_service)
):
    delivery = service.get_delivery_by_id(delivery_id=delivery_id)
    if not delivery:
        raise HTTPException(status_code=404, detail=f"Delivery with ID '{delivery_id}' not found")
    return delivery

@router.patch("/{delivery_id}/status", response_model=Delivery, summary="Update the status of a delivery")
async def update_delivery_status_endpoint(
    delivery_id: str,
    status_update: DeliveryStatusUpdate, # Pydantic model for request body
    service: DeliveryService = Depends(get_delivery_service)
):
    updated_delivery = service.update_delivery_status(
        delivery_id=delivery_id, 
        new_status=status_update.status # Extract status from the model
    )
    if not updated_delivery:
        raise HTTPException(status_code=404, detail=f"Delivery with ID '{delivery_id}' not found, cannot update status")
    return updated_delivery

@router.patch("/{delivery_id}", response_model=Delivery, summary="Update details of a delivery")
async def update_delivery_details_endpoint(
    delivery_id: str,
    delivery_update_data: DeliveryUpdate, # Pydantic model for request body
    service: DeliveryService = Depends(get_delivery_service)
):
    updated_delivery = service.update_delivery_details(
        delivery_id=delivery_id,
        delivery_update_data=delivery_update_data
    )
    if not updated_delivery:
        raise HTTPException(status_code=404, detail=f"Delivery with ID '{delivery_id}' not found, cannot update details")
    return updated_delivery

@router.get("/", response_model=List[Delivery], summary="List deliveries, filtered by user_id")
async def list_deliveries_by_user_endpoint(
    user_id: str = Query(..., min_length=1, description="User ID to filter deliveries for"),
    service: DeliveryService = Depends(get_delivery_service)
):
    # Add basic validation for user_id if needed, though Query(..., min_length=1) helps
    return service.get_deliveries_by_user(user_id=user_id)

@router.delete("/{delivery_id}", response_model=Delivery, summary="Cancel a delivery")
async def cancel_delivery_endpoint(
    delivery_id: str,
    service: DeliveryService = Depends(get_delivery_service)
):
    # The service.cancel_delivery method returns the delivery object (possibly updated)
    # or None if not found.
    delivery_after_cancel_attempt = service.cancel_delivery(delivery_id=delivery_id)
    
    if not delivery_after_cancel_attempt:
        raise HTTPException(status_code=404, detail=f"Delivery with ID '{delivery_id}' not found for cancellation attempt")
    
    # If the service method indicates cancellation wasn't possible by not changing status
    if delivery_after_cancel_attempt.status != DeliveryStatus.CANCELLED:
        raise HTTPException(
            status_code=400, # Bad Request
            detail=f"Delivery with ID '{delivery_id}' in status '{delivery_after_cancel_attempt.status.value}' could not be cancelled."
        )
    return delivery_after_cancel_attempt

# Example of how to run this API (from the root of crypto-bot-delivery directory):
# 1. Ensure dependencies are installed: pip install -r src/delivery_api/requirements.txt
# 2. Ensure the project is installed or PYTHONPATH is set for `app` module: pip install -e .
# 3. Run uvicorn: uvicorn src.delivery_api.app.main:app --reload
#    (Adjust path to main:app if your project structure or how uvicorn is run differs)
#
# Then access via e.g. http://127.0.0.1:8000/docs
```
