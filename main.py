from fastapi import FastAPI, HTTPException, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
from typing import List
import sqlite3
import json
import time
import logging
import threading
import heapq
import redis
import asyncio

# Logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('robot_system.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# App and DB setup
app = FastAPI(title="Wall Robot Control System", version="2.0.0")
app.mount("/static", StaticFiles(directory="static"), name="static")
DB_NAME = "robot_trajectories.db"
db_lock = threading.Lock()

# Redis setup
REDIS_CHANNEL = "robot_updates"
redis_client = redis.Redis(host="localhost", port=6379, decode_responses=True)

def publish_event(message: str):
    try:
        redis_client.publish(REDIS_CHANNEL, message)
        logger.info(f"Published event: {message.encode('ascii', 'ignore').decode()}")
    except Exception as e:
        logger.error(f"Redis publish failed: {str(e)}")

def get_db_connection():
    conn = sqlite3.connect(DB_NAME, check_same_thread=False)
    conn.execute("PRAGMA foreign_keys = ON")
    return conn

def init_db():
    with db_lock:
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS trajectories (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                wall_width REAL NOT NULL,
                wall_height REAL NOT NULL,
                obstacles TEXT,
                path_data TEXT NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                execution_time REAL
            )
        ''')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_name ON trajectories(name)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_created_at ON trajectories(created_at)')
        conn.commit()
        conn.close()

init_db()

# Models
class Obstacle(BaseModel):
    x: float
    y: float
    width: float
    height: float

class WallConfig(BaseModel):
    width: float
    height: float
    obstacles: List[Obstacle] = []

class TrajectoryRequest(BaseModel):
    name: str
    wall_config: WallConfig

class TrajectoryResponse(BaseModel):
    id: int
    name: str
    wall_width: float
    wall_height: float
    obstacles: List[Obstacle]
    path_data: List[List[float]]
    created_at: str
    execution_time: float

# Enhanced Coverage Planner with A* Detour
class CoveragePlannerSmart:
    def __init__(self, wall_width, wall_height, obstacles):
        self.wall_width = wall_width
        self.wall_height = wall_height
        self.obstacles = obstacles
        self.grid_resolution = 0.05
        self.rows = int(wall_height / self.grid_resolution)
        self.cols = int(wall_width / self.grid_resolution)
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        self._mark_obstacles()

    def _mark_obstacles(self):
        for obs in self.obstacles:
            x_start = int(obs.x / self.grid_resolution)
            x_end = int((obs.x + obs.width) / self.grid_resolution)
            y_start = int(obs.y / self.grid_resolution)
            y_end = int((obs.y + obs.height) / self.grid_resolution)
            for y in range(y_start, min(y_end + 1, self.rows)):
                for x in range(x_start, min(x_end + 1, self.cols)):
                    self.grid[y][x] = 1

    def _heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _neighbors(self, node):
        dirs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        result = []
        for dy, dx in dirs:
            ny, nx = node[0] + dy, node[1] + dx
            if 0 <= ny < self.rows and 0 <= nx < self.cols and self.grid[ny][nx] == 0:
                result.append((ny, nx))
        return result

    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            for neighbor in self._neighbors(current):
                tentative = g_score[current] + 1
                if neighbor not in g_score or tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score[neighbor] = tentative + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []

    def generate_path(self):
        path = []
        direction = 1
        current = (0, 0)
        for y in range(self.rows):
            target = (y, self.cols - 1) if direction == 1 else (y, 0)
            sub_path = self.a_star(current, target)
            for cell in sub_path:
                path.append([cell[1] * self.grid_resolution, cell[0] * self.grid_resolution])
            current = target
            direction *= -1
        return path

@app.get("/", response_class=HTMLResponse)
async def root():
    try:
        with open("static/index.html", "r", encoding="utf-8") as f:
            return HTMLResponse(content=f.read())
    except FileNotFoundError:
        return HTMLResponse(content="<h1>index.html not found</h1>", status_code=404)

@app.post("/api/trajectories", response_model=dict)
async def create_trajectory(request: TrajectoryRequest):
    start_time = time.time()
    try:
        planner = CoveragePlannerSmart(request.wall_config.width, request.wall_config.height, request.wall_config.obstacles)
        path_data = planner.generate_path()
        with db_lock:
            db = get_db_connection()
            cursor = db.cursor()
            cursor.execute('''
                INSERT INTO trajectories (name, wall_width, wall_height, obstacles, path_data, execution_time)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (
                request.name,
                request.wall_config.width,
                request.wall_config.height,
                json.dumps([obs.dict() for obs in request.wall_config.obstacles]),
                json.dumps(path_data),
                time.time() - start_time
            ))
            db.commit()
            trajectory_id = cursor.lastrowid
            db.close()
        publish_event(f"🛠️ Trajectory created: {request.name} ({len(path_data)} points)")
        return {
            "id": trajectory_id,
            "message": "Trajectory created successfully",
            "path_points": len(path_data),
            "execution_time": time.time() - start_time
        }
    except Exception as e:
        logger.error(f"Error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.delete("/api/trajectories/{trajectory_id}")
async def delete_trajectory(trajectory_id: int):
    try:
        with db_lock:
            db = get_db_connection()
            cursor = db.cursor()
            cursor.execute("DELETE FROM trajectories WHERE id = ?", (trajectory_id,))
            deleted = cursor.rowcount
            db.commit()
            db.close()
            if deleted == 0:
                raise HTTPException(status_code=404, detail="Trajectory not found")
        publish_event(f"🗑️ Trajectory deleted: ID {trajectory_id}")
        return {"message": "Trajectory deleted successfully"}
    except Exception as e:
        logger.error(f"Delete error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/trajectories", response_model=List[dict])
async def get_trajectories():
    try:
        with db_lock:
            db = get_db_connection()
            cursor = db.cursor()
            cursor.execute('SELECT id, name, wall_width, wall_height, created_at, execution_time FROM trajectories ORDER BY created_at DESC')
            rows = cursor.fetchall()
            db.close()
            return [
                {
                    "id": row[0],
                    "name": row[1],
                    "wall_width": row[2],
                    "wall_height": row[3],
                    "created_at": row[4],
                    "execution_time": row[5]
                } for row in rows
            ]
    except Exception as e:
        logger.error(f"Error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/trajectories/{trajectory_id}", response_model=TrajectoryResponse)
async def get_trajectory(trajectory_id: int):
    try:
        with db_lock:
            db = get_db_connection()
            cursor = db.cursor()
            cursor.execute('SELECT * FROM trajectories WHERE id = ?', (trajectory_id,))
            row = cursor.fetchone()
            db.close()
            if not row:
                raise HTTPException(status_code=404, detail="Trajectory not found")
            return TrajectoryResponse(
                id=row[0],
                name=row[1],
                wall_width=row[2],
                wall_height=row[3],
                obstacles=json.loads(row[4]),
                path_data=json.loads(row[5]),
                created_at=row[6],
                execution_time=row[7]
            )
    except Exception as e:
        logger.error(f"Error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.websocket("/ws/updates")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    pubsub = redis_client.pubsub()
    pubsub.subscribe(REDIS_CHANNEL)
    try:
        while True:
            message = pubsub.get_message()
            if message and message["type"] == "message":
                await websocket.send_text(message["data"])
            await asyncio.sleep(0.2)
    except Exception as e:
        logger.error(f"WebSocket error: {str(e)}")
    finally:
        await websocket.close()




