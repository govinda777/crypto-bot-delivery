# src/delivery_api/app/main.py
from fastapi import FastAPI
# Ensure this import path is correct based on your project structure
# If deliveries.py and auth.py are in app/api/endpoints/, and main.py is in app/
from app.api.endpoints import deliveries, auth 

app = FastAPI(
    title="Crypto-Bot Delivery API", 
    version="0.1.0",
    description="API for managing autonomous delivery operations for Crypto-Bot."
)

@app.get("/", tags=["Health Check"])
async def read_root():
    return {"message": "Welcome to the Crypto-Bot Delivery API"}

app.include_router(auth.router) # Add the auth router
app.include_router(deliveries.router)
# Further routers will be included here later
