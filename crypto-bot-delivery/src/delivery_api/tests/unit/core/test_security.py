import pytest
from datetime import datetime, timedelta, timezone
from unittest.mock import patch

from jose import jwt, JWTError
from fastapi import HTTPException, status
# from pydantic import ValidationError # Not directly testing TokenData model validation here

# Adjust import path based on project structure and how pytest discovers modules
# Assuming 'app' is discoverable (e.g., project installed with -e ., or PYTHONPATH in pyproject.toml).
from app.core.security import (
    create_access_token,
    get_current_user,
    verify_password,
    get_password_hash,
    User, # The simple User model returned by get_current_user
    SECRET_KEY,
    ALGORITHM,
    ACCESS_TOKEN_EXPIRE_MINUTES
    # oauth2_scheme # Not directly used in these unit tests, but part of security.py
)

def test_password_hashing_and_verification():
    password = "securepassword123"
    hashed_password = get_password_hash(password)
    
    assert isinstance(hashed_password, str), "Hashed password should be a string"
    assert hashed_password != password, "Hash should not be the same as plain password"
    
    assert verify_password(password, hashed_password) is True, "Password verification failed for correct password"
    assert verify_password("wrongpassword", hashed_password) is False, "Password verification succeeded for incorrect password"

def test_create_access_token_default_expiry():
    username = "testuser_token"
    token_data = {"sub": username} # 'sub' is the standard claim for subject (username)
    
    token = create_access_token(data=token_data)
    assert isinstance(token, str), "Token should be a string"
    
    # Decode for payload inspection (options={"verify_exp": False} to check payload even if test runs near expiry time)
    payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM], options={"verify_exp": False})
    assert payload["sub"] == username, "Token 'sub' claim does not match username"
    assert "exp" in payload, "Token should have an 'exp' (expiration) claim"
    
    expected_expiry_approx = datetime.now(timezone.utc) + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    # Allow a small tolerance (e.g., 5 seconds) for the exact expiry time due to execution delay
    token_expiry_datetime = datetime.fromtimestamp(payload["exp"], tz=timezone.utc)
    assert abs((token_expiry_datetime - expected_expiry_approx).total_seconds()) < 5, \
        f"Token expiry time not within tolerance. Expected around {expected_expiry_approx}, got {token_expiry_datetime}"

def test_create_access_token_custom_expiry():
    username = "testuser_custom_exp"
    custom_delta = timedelta(hours=1)
    token_data = {"sub": username}
    
    token = create_access_token(data=token_data, expires_delta=custom_delta)
    payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM], options={"verify_exp": False})
    
    assert payload["sub"] == username
    expected_expiry_approx = datetime.now(timezone.utc) + custom_delta
    token_expiry_datetime = datetime.fromtimestamp(payload["exp"], tz=timezone.utc)
    assert abs((token_expiry_datetime - expected_expiry_approx).total_seconds()) < 5, \
        f"Custom token expiry time not within tolerance. Expected {expected_expiry_approx}, got {token_expiry_datetime}"

@patch('app.core.security.jwt.decode') # Mocking jwt.decode specifically within the app.core.security module
async def test_get_current_user_valid_token(mock_jwt_decode):
    test_username = "validuser"
    # Simulate the payload that jwt.decode would return for a valid token
    mock_jwt_decode.return_value = {"sub": test_username, "exp": datetime.now(timezone.utc) + timedelta(minutes=15)}
    
    fake_token_string = "this.is.a.mocked.valid.token" # The actual string value doesn't matter much since decode is mocked
    user = await get_current_user(token=fake_token_string) 
    
    assert isinstance(user, User), "get_current_user should return a User model instance"
    assert user.username == test_username, "Returned user's username does not match token subject"
    mock_jwt_decode.assert_called_once_with(fake_token_string, SECRET_KEY, algorithms=[ALGORITHM])

@patch('app.core.security.jwt.decode')
async def test_get_current_user_jwt_error_on_decode(mock_jwt_decode):
    # Simulate jwt.decode raising a JWTError (e.g., signature mismatch, malformed token)
    mock_jwt_decode.side_effect = JWTError("Mocked JWT Decoding Error")
    
    with pytest.raises(HTTPException) as excinfo:
        await get_current_user(token="any.token.string")
    
    assert excinfo.value.status_code == status.HTTP_401_UNAUTHORIZED
    assert "Could not validate credentials" in excinfo.value.detail, "HTTPException detail message mismatch"
    assert excinfo.value.headers == {"WWW-Authenticate": "Bearer"}, "WWW-Authenticate header missing or incorrect"

@patch('app.core.security.jwt.decode')
async def test_get_current_user_missing_sub_claim_in_payload(mock_jwt_decode):
    # Simulate a payload that is validly decoded but lacks the 'sub' claim
    mock_jwt_decode.return_value = {"exp": datetime.now(timezone.utc) + timedelta(minutes=15)} # No 'sub'
    
    with pytest.raises(HTTPException) as excinfo:
        await get_current_user(token="token.missing.sub")
        
    assert excinfo.value.status_code == status.HTTP_401_UNAUTHORIZED
    assert "Could not validate credentials" in excinfo.value.detail # Because username (from sub) will be None

async def test_get_current_user_actually_expired_token():
    # Create a token that is genuinely expired
    username_for_expired_token = "expired_user_test"
    # Expires 1 minute in the past
    expired_delta = timedelta(minutes=-1) 
    expired_token = create_access_token(data={"sub": username_for_expired_token}, expires_delta=expired_delta)

    with pytest.raises(HTTPException) as excinfo:
        # This time, jwt.decode is NOT mocked, so it will process the expired token
        await get_current_user(token=expired_token)
    
    assert excinfo.value.status_code == status.HTTP_401_UNAUTHORIZED
    # The detail might be "Could not validate credentials" because JWTError (specifically ExpiredSignatureError) is caught
    assert "Could not validate credentials" in excinfo.value.detail

# Example of how to run these tests from the project root (crypto-bot-delivery):
# python -m pytest -v src/delivery_api/tests/unit/core/test_security.py
```
