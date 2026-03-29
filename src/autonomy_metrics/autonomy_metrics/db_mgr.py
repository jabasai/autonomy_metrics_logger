from __future__ import annotations

from datetime import datetime, timezone
import math

from bson import ObjectId
from pymongo import DESCENDING, MongoClient


try:
    import numpy as np
except Exception:  # pragma: no cover
    np = None


class DatabaseMgr:
    """MongoDB helper for session summaries, events, and periodic snapshots."""

    def __init__(
        self,
        database_name: str = "robot_incidents",
        host: str = "localhost",
        port: int = 27017,
    ):
        self.client = MongoClient(
            f"mongodb://{host}:{port}/",
            connectTimeoutMS=1500,
            serverSelectionTimeoutMS=1500,
        )
        self.db = self.client[database_name]
        self.sessions_collection = self.db["sessions"]
        self.events_collection = self.db["session_events"]
        self.snapshots_collection = self.db["session_snapshots"]
        self.session_id = None

        self.sessions_collection.create_index([("session_start_time", DESCENDING)])
        self.events_collection.create_index([("session_id", 1), ("time", DESCENDING)])
        self.snapshots_collection.create_index([("session_id", 1), ("time", DESCENDING)])

    def _bson_safe(self, obj):
        if np is not None:
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if isinstance(obj, np.generic):
                return obj.item()

        if isinstance(obj, float):
            if math.isfinite(obj):
                return obj
            return None

        if isinstance(obj, dict):
            return {key: self._bson_safe(value) for key, value in obj.items()}

        if isinstance(obj, (list, tuple)):
            return [self._bson_safe(value) for value in obj]

        return obj

    def _json_safe(self, obj):
        if isinstance(obj, ObjectId):
            return str(obj)
        if isinstance(obj, datetime):
            return obj.astimezone(timezone.utc).isoformat()
        if isinstance(obj, dict):
            return {key: self._json_safe(value) for key, value in obj.items()}
        if isinstance(obj, list):
            return [self._json_safe(value) for value in obj]
        return obj

    def init_session(self, env_variables: dict, git_repos_info: list[dict], metadata: dict):
        session_start = datetime.now(tz=timezone.utc)
        session_document = self._bson_safe(
            {
                "session_start_time": session_start,
                "session_end_time": None,
                "robot_name": env_variables.get("robot_name", "UNDEFINED"),
                "farm_name": env_variables.get("farm_name", "UNDEFINED"),
                "field_name": env_variables.get("field_name", "UNDEFINED"),
                "application": env_variables.get("application", "UNDEFINED"),
                "scenario_name": env_variables.get("scenario_name", "UNDEFINED"),
                "aoc_repos_info": git_repos_info,
                "summary": metadata,
            }
        )

        result = self.sessions_collection.insert_one(session_document)
        self.session_id = result.inserted_id
        return str(self.session_id)

    def update_session_summary(self, summary: dict):
        if self.session_id is None:
            return False
        summary = self._bson_safe(summary)
        result = self.sessions_collection.update_one(
            {"_id": self.session_id},
            {"$set": {"summary": summary, "last_updated_time": datetime.now(tz=timezone.utc)}},
        )
        return result.acknowledged

    def mark_session_end(self, summary: dict | None = None):
        if self.session_id is None:
            return False
        update = {"session_end_time": datetime.now(tz=timezone.utc)}
        if summary is not None:
            update["summary"] = self._bson_safe(summary)
        result = self.sessions_collection.update_one(
            {"_id": self.session_id},
            {"$set": update},
        )
        return result.acknowledged

    def add_event(self, event: dict):
        if self.session_id is None:
            return False
        document = self._bson_safe(
            {
                "session_id": self.session_id,
                **event,
            }
        )
        result = self.events_collection.insert_one(document)
        return bool(result.inserted_id)

    def add_snapshot(self, snapshot: dict):
        if self.session_id is None:
            return False
        document = self._bson_safe(
            {
                "session_id": self.session_id,
                **snapshot,
            }
        )
        result = self.snapshots_collection.insert_one(document)
        return bool(result.inserted_id)

    def fetch_latest_session(self) -> dict | None:
        document = self.sessions_collection.find_one(
            sort=[("session_start_time", DESCENDING)]
        )
        return self._json_safe(document) if document else None

    def fetch_recent_events(self, limit: int = 20, session_id: str | None = None) -> list[dict]:
        query = {}
        if session_id:
            query["session_id"] = ObjectId(session_id)
        documents = list(
            self.events_collection.find(query).sort("time", DESCENDING).limit(limit)
        )
        return self._json_safe(documents)

    def fetch_recent_snapshots(
        self, limit: int = 10, session_id: str | None = None
    ) -> list[dict]:
        query = {}
        if session_id:
            query["session_id"] = ObjectId(session_id)
        documents = list(
            self.snapshots_collection.find(query).sort("time", DESCENDING).limit(limit)
        )
        return self._json_safe(documents)
