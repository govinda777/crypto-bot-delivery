from datetime import datetime, timedelta, timezone
from typing import Optional

from jose import JWTError, jwt
from passlib.context import CryptContext
from pydantic import BaseModel
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer

# Configuration (replace with proper config loading later)
SECRET_KEY = "your-super-secret-key-for-crypto-bot-delivery-0123456789abcdef" # KEEP THIS SAFE AND OUT OF SOURCE CONTROL IN REAL APPS
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# The tokenUrl should point to the endpoint that provides the token.
# This will be /auth/token based on the auth router prefix and endpoint path.
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/token") 

class TokenData(BaseModel):
    username: Optional[str] = None

class User(BaseModel): # Simple User model for dependency response
    username: str
    # email: Optional[str] = None
    # full_name: Optional[str] = None
    # disabled: Optional[bool] = None


def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

async def get_current_user(token: str = Depends(oauth2_scheme)) -> User:
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception
        # For now, we don't fetch user from DB, just return a User model with username
        # In a real app, you'd fetch user details from DB based on username (or user_id in token)
        # token_data = TokenData(username=username) 
    except JWTError:
        raise credentials_exception
    
    # This is where you would normally retrieve the user from your database
    # For this POC, we'll just return a User object with the username from the token
    # user_db_record = get_user_from_db(username=username) # Placeholder for DB lookup
    # if user_db_record is None:
    #     raise credentials_exception
    # return User(**user_db_record.model_dump()) # Or however you convert DB model to Pydantic
    return User(username=username) # Return a User model based on token subject

# Placeholder for a more complete current_active_user if you add roles/disabled status
# async def get_current_active_user(current_user: User = Depends(get_current_user)):
#     if current_user.disabled: # Assuming User model has 'disabled' field
#         raise HTTPException(status_code=400, detail="Inactive user")
#     return current_user
```
