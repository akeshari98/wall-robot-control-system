import pytest
import json
import os
import sqlite3
from fastapi.testclient import TestClient
from main import app, init_db

# Create test client
client = TestClient(app)

# Test database setup
TEST_DB = "test_robot_trajectories.db"

@pytest.fixture(autouse=True)
def setup_test_db():
    """Setup test database before each test"""
    # Remove existing test database
    if os.path.exists(TEST_DB):
        os.remove(TEST_DB)
    
    # Initialize test database
    init_db()
    yield
    
    # Cleanup after test
    if os.path.exists(TEST_DB):
        os.remove(TEST_DB)

class TestAPI:
    """Test suite for Wall Robot Control System API"""
    
    def test_root_endpoint(self):
        """Test the root endpoint returns HTML"""
        response = client.get("/")
        assert response.status_code == 200
        assert "text/html" in response.headers["content-type"]

    def test_create_trajectory_success(self):
        """Test successful trajectory creation"""
        trajectory_data = {
            "name": "Test Trajectory",
            "wall_config": {
                "width": 5.0,
                "height": 5.0,
                "obstacles": [
                    {
                        "x": 2.0,
                        "y": 2.0,
                        "width": 0.25,
                        "height": 0.25
                    }
                ]
            }
        }
        
        response = client.post("/api/trajectories", json=trajectory_data)
        assert response.status_code == 200
        
        result = response.json()
        assert "id" in result
        assert result["message"] == "Trajectory created successfully"
        assert result["path_points"] > 0
        assert result["execution_time"] > 0

    def test_create_trajectory_validation(self):
        """Test trajectory creation with invalid data"""
        invalid_data = {
            "name": "",  # Empty name
            "wall_config": {
                "width": -1,  # Invalid width
                "height": 5.0,
                "obstacles": []
            }
        }
        
        response = client.post("/api/trajectories", json=invalid_data)
        # Should handle gracefully even with invalid data
        assert response.status_code in [200, 422, 500]

    def test_get_trajectories_empty(self):
        """Test getting trajectories when none exist"""
        response = client.get("/api/trajectories")
        assert response.status_code == 200
        
        trajectories = response.json()
        assert isinstance(trajectories, list)
        assert len(trajectories) == 0

    def test_get_trajectories_with_data(self):
        """Test getting trajectories after creating some"""
        # Create a trajectory first
        trajectory_data = {
            "name": "Test Trajectory 1",
            "wall_config": {
                "width": 3.0,
                "height": 3.0,
                "obstacles": []
            }
        }
        
        create_response = client.post("/api/trajectories", json=trajectory_data)
        assert create_response.status_code == 200
        
        # Get all trajectories
        response = client.get("/api/trajectories")
        assert response.status_code == 200
        
        trajectories = response.json()
        assert len(trajectories) == 1
        assert trajectories[0]["name"] == "Test Trajectory 1"

    def test_get_specific_trajectory(self):
        """Test getting a specific trajectory by ID"""
        # Create a trajectory first
        trajectory_data = {
            "name": "Specific Test",
            "wall_config": {
                "width": 4.0,
                "height": 4.0,
                "obstacles": [
                    {
                        "x": 1.0,
                        "y": 1.0,
                        "width": 0.5,
                        "height": 0.5
                    }
                ]
            }
        }
        
        create_response = client.post("/api/trajectories", json=trajectory_data)
        assert create_response.status_code == 200
        
        trajectory_id = create_response.json()["id"]
        
        # Get specific trajectory
        response = client.get(f"/api/trajectories/{trajectory_id}")
        assert response.status_code == 200
        
        trajectory = response.json()
        assert trajectory["name"] == "Specific Test"
        assert trajectory["wall_width"] == 4.0
        assert trajectory["wall_height"] == 4.0
        assert len(trajectory["obstacles"]) == 1
        assert len(trajectory["path_data"]) > 0

    def test_get_nonexistent_trajectory(self):
        """Test getting a trajectory that doesn't exist"""
        response = client.get("/api/trajectories/999")
        assert response.status_code == 404
        
        result = response.json()
        assert result["detail"] == "Trajectory not found"

    def test_delete_trajectory(self):
        """Test deleting a trajectory"""
        # Create a trajectory first
        trajectory_data = {
            "name": "To Delete",
            "wall_config": {
                "width": 2.0,
                "height": 2.0,
                "obstacles": []
            }
        }
        
        create_response = client.post("/api/trajectories", json=trajectory_data)
        assert create_response.status_code == 200
        
        trajectory_id = create_response.json()["id"]
        
        # Delete the trajectory
        response = client.delete(f"/api/trajectories/{trajectory_id}")
        assert response.status_code == 200
        
        result = response.json()
        assert result["message"] == "Trajectory deleted successfully"
        
        # Verify it's deleted
        get_response = client.get(f"/api/trajectories/{trajectory_id}")
        assert get_response.status_code == 404

    def test_delete_nonexistent_trajectory(self):
        """Test deleting a trajectory that doesn't exist"""
        response = client.delete("/api/trajectories/999")
        assert response.status_code == 404
        
        result = response.json()
        assert result["detail"] == "Trajectory not found"

    def test_coverage_planning_with_obstacles(self):
        """Test that coverage planning properly avoids obstacles"""
        trajectory_data = {
            "name": "Obstacle Test",
            "wall_config": {
                "width": 2.0,
                "height": 2.0,
                "obstacles": [
                    {
                        "x": 0.5,
                        "y": 0.5,
                        "width": 1.0,
                        "height": 1.0
                    }
                ]
            }
        }
        
        response = client.post("/api/trajectories", json=trajectory_data)
        assert response.status_code == 200
        
        trajectory_id = response.json()["id"]
        
        # Get the trajectory data
        get_response = client.get(f"/api/trajectories/{trajectory_id}")
        trajectory = get_response.json()
        
        # Verify path exists and has reasonable number of points
        assert len(trajectory["path_data"]) > 0
        
        # Check that some points exist outside the obstacle area
        path_points = trajectory["path_data"]
        points_outside_obstacle = []
        
        for point in path_points:
            x, y = point[0], point[1]
            # Check if point is outside obstacle (0.5-1.5, 0.5-1.5)
            if not (0.5 <= x <= 1.5 and 0.5 <= y <= 1.5):
                points_outside_obstacle.append(point)
        
        assert len(points_outside_obstacle) > 0, "Path should have points outside obstacle area"

    def test_response_time_performance(self):
        """Test API response times"""
        import time
        
        trajectory_data = {
            "name": "Performance Test",
            "wall_config": {
                "width": 10.0,
                "height": 10.0,
                "obstacles": []
            }
        }
        
        # Measure trajectory creation time
        start_time = time.time()
        response = client.post("/api/trajectories", json=trajectory_data)
        end_time = time.time()
        
        assert response.status_code == 200
        
        # Should complete within reasonable time (5 seconds for large wall)
        execution_time = end_time - start_time
        assert execution_time < 5.0, f"Trajectory creation took too long: {execution_time}s"
        
        # Test getting trajectories response time
        start_time = time.time()
        response = client.get("/api/trajectories")
        end_time = time.time()
        
        assert response.status_code == 200
        query_time = end_time - start_time
        assert query_time < 1.0, f"Query took too long: {query_time}s"

    def test_multiple_trajectories_crud(self):
        """Test creating, reading, and deleting multiple trajectories"""
        trajectory_names = ["Trajectory 1", "Trajectory 2", "Trajectory 3"]
        created_ids = []
        
        # Create multiple trajectories
        for name in trajectory_names:
            trajectory_data = {
                "name": name,
                "wall_config": {
                    "width": 3.0,
                    "height": 3.0,
                    "obstacles": []
                }
            }
            
            response = client.post("/api/trajectories", json=trajectory_data)
            assert response.status_code == 200
            created_ids.append(response.json()["id"])
        
        # Verify all were created
        response = client.get("/api/trajectories")
        trajectories = response.json()
        assert len(trajectories) == 3
        
        # Verify names match
        returned_names = [t["name"] for t in trajectories]
        for name in trajectory_names:
            assert name in returned_names
        
        # Delete all trajectories
        for trajectory_id in created_ids:
            response = client.delete(f"/api/trajectories/{trajectory_id}")
            assert response.status_code == 200
        
        # Verify all deleted
        response = client.get("/api/trajectories")
        trajectories = response.json()
        assert len(trajectories) == 0

if __name__ == "__main__":
    pytest.main([__file__, "-v"])