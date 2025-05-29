from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordRequestForm
from datetime import timedelta

# Adjust import paths based on actual project structure
# Assuming 'app' is a top-level directory in PYTHONPATH or part of the package
from app.core.security import create_access_token, verify_password, get_password_hash, ACCESS_TOKEN_EXPIRE_MINUTES
from pydantic import BaseModel

class Token(BaseModel):
    access_token: str
    token_type: str

router = APIRouter(prefix="/auth", tags=["Authentication"])

# Dummy user database (replace with actual database interaction in a real application)
# Ensure get_password_hash is available here or passwords are pre-hashed
DUMMY_USERS_DB = {
    "testuser": {
        "username": "testuser",
        "hashed_password": get_password_hash("testpassword"), # Hash the password during setup
        "email": "testuser@example.com",
        "full_name": "Test User",
        "disabled": False, # Example field for active/inactive users
    },
    "userprod": {
        "username": "userprod",
        "hashed_password": get_password_hash("prodpass"),
        "email": "userprod@example.com",
        "full_name": "Production User",
        "disabled": False,
    }
}

# Simplified user getter for the dummy DB
def get_user_from_dummy_db(db, username: str):
    if username in db:
        user_dict = db[username]
        # If your User model in security.py includes more fields (email, full_name, disabled)
        # ensure they are present here or handle their absence.
        # For now, security.User only expects 'username'.
        return user_dict # Returning the dict, as that's what's expected by current login logic
    return None

@router.post("/token", response_model=Token, summary="Login and get access token")
async def login_for_access_token(form_data: OAuth2PasswordRequestForm = Depends()):
    """
    Standard OAuth2 password flow.
    Provides an access token if credentials are correct.
    """
    user_dict = get_user_from_dummy_db(DUMMY_USERS_DB, form_data.username)
    if not user_dict or not verify_password(form_data.password, user_dict["hashed_password"]):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    # Could check if user_dict["disabled"] here if that was part of a UserInDB model
    
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user_dict["username"]}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

# Example: How to use this in a protected endpoint (in another file like deliveries.py)
# from app.core.security import get_current_user, User
# @router.get("/users/me", response_model=User)
# async def read_users_me(current_user: User = Depends(get_current_user)):
#     return current_user
```
