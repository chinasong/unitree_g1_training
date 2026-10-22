import asyncio
from contextlib import asynccontextmanager, suppress
import hmac
import json
import logging
from pathlib import Path
import re
import time
from uuid import uuid4
from urllib.parse import quote
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.exceptions import RequestValidationError
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware
from .body import BODY
from .config import ConsoleError, ROOT, Settings
from .contracts import Connect, Owner, Acquire, ManualArm, SequenceStart, SequenceContinue, Preflight, Command, Goal, Observation, CameraFrame, Contract
from .service import ConsoleService
from .insight import ObserverControl, Worker, Submission

log = logging.getLogger("g1_console")
COOKIE = "g1_console_session"


class Login(Contract):
    token: str


def authorized(headers, cookies, token):
    value = headers.get("authorization", "").removeprefix("Bearer ") or cookies.get(COOKIE, "")
    return hmac.compare_digest(value, token)


def origin_ok(headers):
    origin = headers.get("origin")
    return origin is None or origin in ("http://" + headers.get("host", ""), "https://" + headers.get("host", ""))


def create_app(settings=None):
    settings = settings or Settings()
    settings.validate()
    service = ConsoleService(settings)

    @asynccontextmanager
    async def lifespan(app):
        task = asyncio.create_task(service.run())
        yield
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
        await service.close()

    app = FastAPI(title="G1 Body Console", version="0.1.0", lifespan=lifespan, docs_url=None, redoc_url=None)
    app.state.service, app.state.settings = service, settings
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=list({settings.host, *settings.allowed_hosts, "localhost", "127.0.0.1", "[::1]", "testserver"}))

    @app.exception_handler(ConsoleError)
    async def domain_error(request, exc):
        error = dict(code=exc.code, message=exc.message)
        if exc.sdk_code is not None:
            error['sdk_code'] = exc.sdk_code
        return JSONResponse(dict(error=error, request_id=getattr(request.state, "request_id", None)), status_code=exc.status)

    @app.exception_handler(RequestValidationError)
    async def invalid(request, exc):
        # Do not echo raw inputs, including tokens, large JPEGs or nonfinite floats.
        fields = [".".join(str(p) for p in err["loc"]) + ": " + err["msg"] for err in exc.errors()]
        return JSONResponse(dict(error=dict(code="VALIDATION_ERROR", message="；".join(fields)[:600])), status_code=422)

    @app.middleware("http")
    async def boundary(request: Request, call_next):
        request.state.request_id = uuid4().hex
        start = time.monotonic()
        path = request.url.path
        if not origin_ok(request.headers):
            response = JSONResponse(dict(error=dict(code="ORIGIN_DENIED", message="拒绝跨站请求")), status_code=403)
        elif request.method not in ("GET", "HEAD", "OPTIONS") and request.headers.get("x-g1-intent") != "operator":
            response = JSONResponse(dict(error=dict(code="INTENT_REQUIRED", message="操作请求缺少意图头")), status_code=403)
        elif path.startswith("/api/") and path != "/api/session" and not authorized(request.headers, request.cookies, settings.token):
            response = JSONResponse(dict(error=dict(code="UNAUTHORIZED", message="需要本机控制台会话或 API token")), status_code=401)
        else:
            try:
                if request.method == "POST":
                    size = 0
                    chunks = []
                    async for chunk in request.stream():
                        size += len(chunk)
                        if size > 900000:
                            raise ConsoleError("BODY_TOO_LARGE", "请求内容超过 900 KB", 413)
                        chunks.append(chunk)
                    request._body = b"".join(chunks)
                response = await call_next(request)
            except ConsoleError as exc:
                response = await domain_error(request, exc)
            except Exception:
                log.exception(json.dumps(dict(event="request_error", request_id=request.state.request_id, path=path)))
                response = JSONResponse(dict(error=dict(code="INTERNAL_ERROR", message="服务异常，请查看本机日志")), status_code=500)
        response.headers.update({"X-Request-ID": request.state.request_id, "X-Content-Type-Options": "nosniff",
            "Referrer-Policy": "no-referrer", "Cache-Control": "no-store",
            "Content-Security-Policy": "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; frame-ancestors 'none'"})
        if request.method != "GET" and path != "/api/session" and (response.status_code >= 400 or path not in (
                "/api/control/heartbeat", "/api/observation", "/api/camera")):
            log.info(json.dumps(dict(event="request", request_id=request.state.request_id, path=path,
                                    status=response.status_code, duration_ms=round((time.monotonic() - start) * 1000))))
        return response

    def session_response():
        response = JSONResponse(dict(ok=True, version="0.1.0"))
        response.set_cookie(COOKIE, settings.token, httponly=True, samesite="strict", max_age=12 * 3600)
        return response

    @app.get("/api/session")
    async def local_session(request: Request):
        if request.client.host not in ("127.0.0.1", "::1", "testclient") and not authorized(request.headers, request.cookies, settings.token):
            raise ConsoleError("TOKEN_REQUIRED", "局域网访问需要 G1_API_TOKEN", 401)
        return session_response()

    @app.post("/api/session")
    async def token_session(login: Login):
        if not hmac.compare_digest(login.token, settings.token):
            raise ConsoleError("INVALID_TOKEN", "连接密钥不正确", 401)
        return session_response()

    @app.get("/health")
    async def health():
        return dict(status="ok", service="g1-console", version="0.1.0")

    @app.get("/ready")
    async def ready():
        return JSONResponse(dict(ready=settings.static_dir.joinpath("index.html").exists(), robot_connected=service.fresh()),
                            status_code=200 if settings.static_dir.joinpath("index.html").exists() else 503)

    @app.get("/api/body")
    async def body():
        return BODY

    @app.get("/api/state")
    async def state():
        return service.snapshot()

    @app.post("/api/control/preflight")
    async def preflight(data: Preflight):
        return await service.preflight(data)

    @app.post('/api/insight/control')
    async def insight_control(data: ObserverControl):
        async with service.lock:
            service.insights.control(data.enabled)
        return dict(ok=True)

    @app.post('/api/insight/heartbeat')
    async def insight_heartbeat(data: Worker):
        async with service.lock:
            service.insights.heartbeat(data)
            return dict(enabled=service.insights.enabled, run_id=service.run_id)

    @app.post('/api/insight/capture')
    async def insight_capture(data: Worker):
        async with service.lock:
            return service.insights.capture(data, service.snapshot(), service.world.camera)

    @app.post('/api/insight/result')
    async def insight_result(data: Submission):
        async with service.lock:
            service.insights.submit(data, service.snapshot())
        return dict(ok=True)

    @app.post("/api/connect")
    async def connect(data: Connect):
        await service.connect(data.mode, data.interface)
        return service.snapshot()

    @app.post("/api/disconnect")
    async def disconnect():
        await service.disconnect()
        return dict(ok=True)

    @app.post("/api/control/acquire")
    async def acquire(data: Acquire):
        await service.acquire(data.owner, data.floor_verified)
        return dict(ok=True)

    @app.post("/api/control/heartbeat")
    async def heartbeat(data: Owner):
        await service.heartbeat(data.owner)
        return dict(ok=True)

    @app.post("/api/control/release")
    async def release(data: Owner):
        await service.release(data.owner)
        return dict(ok=True)

    @app.post("/api/control/manual/arm")
    async def manual_arm(data: ManualArm):
        await service.arm_manual(data)
        return dict(ok=True)

    @app.post("/api/control/manual/disarm")
    async def manual_disarm(data: Owner):
        await service.disarm_manual(data.owner)
        return dict(ok=True)

    @app.post("/api/control/sequence")
    async def sequence_start(data: SequenceStart):
        await service.start_sequence(data)
        return dict(ok=True)

    @app.post("/api/control/sequence/continue")
    async def sequence_continue(data: SequenceContinue):
        await service.continue_sequence(data)
        return dict(ok=True)

    @app.post("/api/control/sequence/abort")
    async def sequence_abort(data: Owner):
        await service.abort_sequence(data.owner)
        return dict(ok=True)

    @app.post("/api/command")
    async def command(data: Command):
        return await service.command(data)

    @app.post("/api/goal")
    async def goal(data: Goal):
        return await service.goal(data)

    @app.post("/api/observation")
    async def observation(data: Observation):
        await service.observation(data)
        return dict(ok=True)

    @app.post("/api/camera")
    async def camera(data: CameraFrame):
        await service.camera(data)
        return dict(ok=True)

    @app.get("/api/camera.jpg")
    async def camera_image():
        if not service.world.camera or service.world.camera_age() > 1:
            raise ConsoleError("NO_CAMERA", "尚无新鲜相机帧", 404)
        status = service.camera_status()
        headers = {"X-G1-Camera-Sequence": str(status["sequence"]),
                   "X-G1-Camera-Source": quote(status["source"], safe="-_."),
                   "X-G1-Camera-Age-Ms": str(status["age_ms"]),
                   "X-G1-Camera-Metadata": json.dumps(status, separators=(",", ":"))}
        return Response(service.world.camera, media_type="image/jpeg", headers=headers)

    @app.get("/api/camera/status")
    async def camera_status():
        return service.camera_status()

    @app.get("/api/model/meshes/{name}")
    async def mesh(name: str):
        if not re.fullmatch(r"[a-zA-Z0-9_]+\.STL", name):
            raise ConsoleError("INVALID_ASSET", "模型文件名称无效", 404)
        file = ROOT / "externals/unitree_rl_gym/resources/robots/g1_description/meshes" / name
        if not file.is_file():
            raise ConsoleError("MISSING_ASSET", "未安装官方模型；请同步锁定的依赖", 404)
        return FileResponse(file, media_type="application/octet-stream")

    @app.websocket("/api/live")
    async def live(socket: WebSocket):
        if not origin_ok(socket.headers) or not authorized(socket.headers, socket.cookies, settings.token):
            await socket.close(code=1008)
            return
        await socket.accept()
        try:
            while True:
                await socket.send_json(service.snapshot())
                await asyncio.sleep(.2)
        except (WebSocketDisconnect, RuntimeError):
            return

    if settings.static_dir.is_dir():
        app.mount("/", StaticFiles(directory=settings.static_dir, html=True), name="console")
    else:
        @app.get("/")
        async def build_required():
            return Response("图形界面尚未构建：请运行 ./scripts/start_g1_console.sh", media_type="text/plain", status_code=503)
    return app
