"""Minimal FastAPI artifact host for the OTA demo."""
from __future__ import annotations

import json
import os
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse


def create_app(artifacts_dir: Path) -> FastAPI:
    app = FastAPI(title="OTA Update Server")
    artifacts_dir = Path(artifacts_dir)

    @app.get("/catalog")
    def catalog() -> list[dict]:
        catalog_file = artifacts_dir / "catalog.json"
        if not catalog_file.exists():
            return []
        return json.loads(catalog_file.read_text())

    @app.get("/artifacts/{filename}")
    def artifact(filename: str) -> FileResponse:
        if "/" in filename or ".." in filename:
            raise HTTPException(status_code=400, detail="invalid filename")
        path = artifacts_dir / filename
        if not path.exists():
            raise HTTPException(status_code=404, detail="not found")
        return FileResponse(path, media_type="application/gzip", filename=filename)

    return app


def run() -> None:
    import uvicorn
    artifacts_dir = Path(os.environ.get("OTA_ARTIFACTS_DIR", "/artifacts"))
    host = os.environ.get("OTA_HOST", "0.0.0.0")
    port = int(os.environ.get("OTA_PORT", "9000"))
    uvicorn.run(create_app(artifacts_dir), host=host, port=port)


__all__ = ["create_app", "run"]
