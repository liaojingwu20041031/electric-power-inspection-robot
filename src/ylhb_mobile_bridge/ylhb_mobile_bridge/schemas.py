from typing import Any, Dict, Optional

from pydantic import BaseModel, Field


class ApiResponse(BaseModel):
    ok: bool
    message: Optional[str] = None
    data: Optional[Any] = None
    error: Optional[str] = None


class VelocityCommand(BaseModel):
    linear_x: float = Field(default=0.0)
    angular_z: float = Field(default=0.0)
    duration_ms: int = Field(default=300, ge=50, le=3000)


class TextCommand(BaseModel):
    text: str


class TaskCommand(BaseModel):
    command: Optional[str] = None
    text: Optional[str] = None
    task_id: Optional[str] = None
    route_id: Optional[str] = None
    checkpoint_id: Optional[str] = None
    metadata: Dict[str, Any] = Field(default_factory=dict)


class ChassisTestRequest(VelocityCommand):
    mode: str


class MappingSaveRequest(BaseModel):
    map_name: str = 'my_map'


class InitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float


class NavigationGoalRequest(BaseModel):
    x: float
    y: float
    yaw: float
    label: Optional[str] = None


class RobotStatus(BaseModel):
    online: bool
    can_status: Optional[str] = None
    zlac_status: Optional[str] = None
    task_status: Optional[str] = None
    mapping_status: Optional[str] = None
    nav2_status: Optional[str] = None
    last_odom_age_sec: Optional[float] = None
    last_scan_age_sec: Optional[float] = None
    battery_percent: Optional[float] = None
    timestamp: float


class DebugStatus(BaseModel):
    online: bool
    topics: Dict[str, bool]
    nodes: Dict[str, bool]
    last_odom_age_sec: Optional[float] = None
    last_scan_age_sec: Optional[float] = None
    zlac_status: Optional[str] = None
    mapping_status: Optional[str] = None
    nav2_status: Optional[str] = None
