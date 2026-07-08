from pymongo import MongoClient
from datetime import datetime, timezone
from bson import ObjectId

try:
    import numpy as np
except Exception:
    np = None


class DatabaseMgr:
    """
    A thin Mongo session-document manager.

    Reliability contract
    --------------------
    - The constructor MUST NOT raise on connection problems. PyMongo's
      ``MongoClient`` is lazy by default, so this holds as long as we don't
      perform real I/O here.
    - All real I/O happens in ``init_session`` / ``add_event`` / ``update_*``
      and may raise ``pymongo.errors.PyMongoError`` (or subclasses) when the
      server is unreachable. Callers MUST catch those and treat them as
      "DB unhealthy" — they MUST NOT bring down the node.
    - ``ping()`` is a quick liveness probe that never raises.
    - ``server_selection_timeout_ms`` keeps DB ops from hanging forever; the
      default (1000 ms) is intentionally short so a missing server fails fast.

    Session lifecycle
    -----------------
    Calls to ``init_session`` are idempotent: once ``self.session_id`` is
    populated the call is a no-op. This lets the caller retry init from a
    watchdog timer without inserting duplicate session documents.
    """

    def __init__(
        self,
        database_name='robot_incidents',
        host='localhost',
        port=27017,
        server_selection_timeout_ms=1000,
        connect_timeout_ms=1000,
        socket_timeout_ms=2000,
        label='local',
    ):
        self.database_name = database_name
        self.host = host
        self.port = port
        self.label = label
        self.session_id = None

        # MongoClient is lazy: this should not raise even if the server is down.
        self.client = MongoClient(
            f'mongodb://{host}:{port}/',
            serverSelectionTimeoutMS=server_selection_timeout_ms,
            connectTimeoutMS=connect_timeout_ms,
            socketTimeoutMS=socket_timeout_ms,
        )
        self.db = self.client[database_name]
        self.sessions_collection = self.db['sessions']

    def __repr__(self):
        return (
            f"DatabaseMgr(label={self.label}, host={self.host}, "
            f"port={self.port}, has_session={self.session_id is not None})"
        )

    # ------------------------------------------------------------------
    # Liveness
    # ------------------------------------------------------------------
    def ping(self):
        """Quick liveness probe; returns True/False, never raises."""
        try:
            self.client.admin.command('ping')
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------
    # BSON sanitisation
    # ------------------------------------------------------------------
    def _bson_safe(self, obj):
        """Recursively convert non-BSON types (notably numpy) into Mongo-friendly types."""
        if np is not None:
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if isinstance(obj, np.generic):
                return obj.item()

        if isinstance(obj, dict):
            return {k: self._bson_safe(v) for k, v in obj.items()}

        if isinstance(obj, (list, tuple)):
            return [self._bson_safe(v) for v in obj]

        return obj

    # ------------------------------------------------------------------
    # Session management
    # ------------------------------------------------------------------
    def init_session(self, env_variables):
        """
        Insert the session document. Idempotent: a no-op once session_id is set.
        Raises pymongo errors on failure; caller must handle.
        """
        if self.session_id is not None:
            return  # already initialised

        session_start = datetime.now(tz=timezone.utc)
        session_document = {
            "session_start_time": session_start,
            "robot_name": env_variables.get('robot_name'),
            "farm_name": env_variables.get('farm_name'),
            "field_name": env_variables.get('field_name'),
            "application": env_variables.get('application'),
            "scenario_name": env_variables.get('scenario_name'),
            "mdbi": None,
            "incidents": 0,
            "distance": 0,
            "autonomous_distance": 0,
            "manual_distance": 0,
            "collision_incidents": 0,
            "events": [],
        }

        session_document = self._bson_safe(session_document)
        result = self.sessions_collection.insert_one(session_document)
        self.session_id = result.inserted_id

    def _require_session(self):
        if self.session_id is None:
            raise RuntimeError(
                f"DatabaseMgr({self.label}) has no session_id yet; "
                "init_session has not succeeded."
            )

    # ------------------------------------------------------------------
    # Mutations
    # ------------------------------------------------------------------
    def add_event(self, event):
        self._require_session()
        event = self._bson_safe(event)
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$addToSet": {"events": event}},
        )
        return result.modified_count > 0

    def _set_field(self, field, value):
        self._require_session()
        value = self._bson_safe(value)
        result = self.sessions_collection.update_one(
            {"_id": ObjectId(self.session_id)},
            {"$set": {field: value}},
        )
        return result.modified_count > 0

    def update_incidents(self, incidents):
        return self._set_field("incidents", incidents)

    def update_distance(self, distance):
        return self._set_field("distance", distance)

    def update_autonomous_distance(self, autonomous_distance):
        return self._set_field("autonomous_distance", autonomous_distance)

    def update_manual_distance(self, manual_distance):
        return self._set_field("manual_distance", manual_distance)

    def update_mdbi(self, mdbi):
        return self._set_field("mdbi", mdbi)

    def update_collision_incidents(self, collision_incidents):
        return self._set_field("collision_incidents", collision_incidents)
