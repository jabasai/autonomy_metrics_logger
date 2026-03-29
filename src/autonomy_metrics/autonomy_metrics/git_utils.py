"""Helpers for collecting git metadata for the configured repositories."""

from __future__ import annotations

import os
import subprocess


DEFAULT_GIT_REPOS_ROOT = (
    "/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario"
)


def _run_git(repo_path: str, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", repo_path, *args],
        capture_output=True,
        text=True,
        timeout=5.0,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "git command failed")
    return result.stdout.strip()


def get_git_info(repo_path: str) -> dict:
    """Return a git metadata snapshot for one repository path."""
    info = {
        "path": repo_path,
        "exists": False,
        "remote": None,
        "branch": None,
        "commit": None,
        "short_commit": None,
        "commit_message": None,
        "tags": [],
        "describe": None,
        "dirty": None,
        "error": None,
    }

    try:
        if not repo_path or not os.path.isdir(repo_path):
            info["error"] = "Path does not exist or is not a directory"
            return info

        info["exists"] = True

        for key, command in (
            ("remote", ("config", "--get", "remote.origin.url")),
            ("branch", ("rev-parse", "--abbrev-ref", "HEAD")),
            ("commit", ("rev-parse", "HEAD")),
            ("short_commit", ("rev-parse", "--short", "HEAD")),
            ("commit_message", ("log", "-1", "--pretty=%s")),
            ("describe", ("describe", "--tags", "--always")),
        ):
            try:
                info[key] = _run_git(repo_path, *command)
            except Exception:
                info[key] = None

        try:
            tags = _run_git(repo_path, "tag", "--points-at", "HEAD")
            info["tags"] = tags.splitlines() if tags else []
        except Exception:
            info["tags"] = []

        try:
            info["dirty"] = bool(_run_git(repo_path, "status", "--porcelain"))
        except Exception:
            info["dirty"] = None

    except Exception as exc:
        info["error"] = str(exc)

    return info


def collect_git_repos_info(config: dict) -> list[dict]:
    """Collect git metadata for all configured repositories."""
    repos_root = config.get("git_repos_base_path", DEFAULT_GIT_REPOS_ROOT)
    repos_config = config.get("git_repos", {}) or {}

    results = []
    for label, rel_path in repos_config.items():
        repo_path = os.path.join(repos_root, rel_path)
        results.append({label: get_git_info(repo_path)})
    return results
