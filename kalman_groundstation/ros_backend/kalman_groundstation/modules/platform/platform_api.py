from fastapi import APIRouter

from .api.state import state_router

router = APIRouter(prefix="/platform")
router.include_router(state_router)


