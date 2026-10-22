"""Read-only semantic observations. No adapter, command or navigation dependencies."""
import base64
import hashlib
import time
from typing import Literal
from uuid import uuid4
from pydantic import Field
from .contracts import Contract
from .config import ConsoleError
from .body import KNOWLEDGE


class Finding(Contract):
    statement: str = Field(min_length=1, max_length=320)
    evidence: Literal['image', 'telemetry', 'both']
    confidence: Literal['high', 'medium', 'low']


class Suggestion(Contract):
    title: str = Field(min_length=1, max_length=120)
    rationale: str = Field(min_length=1, max_length=400)
    requirements: list[str] = Field(max_length=6)
    capability_ids: list[str] = Field(max_length=6)


class Report(Contract):
    summary: str = Field(min_length=1, max_length=800)
    scene_facts: list[Finding] = Field(max_length=8)
    body_findings: list[Finding] = Field(max_length=8)
    uncertainties: list[str] = Field(max_length=8)
    suggestions: list[Suggestion] = Field(max_length=5)
    change_summary: str = Field(max_length=500)


class ObserverControl(Contract):
    enabled: bool


class Worker(Contract):
    worker: str = Field(pattern=r'^[a-f0-9]{32}$')
    model: str = Field(min_length=1, max_length=80)


class Submission(Worker):
    ticket: str = Field(pattern=r'^[a-f0-9]{32}$')
    report: Report | None
    error: Literal['', 'CODEX_FAILED', 'CODEX_RATE_LIMITED', 'CODEX_TIMEOUT', 'INVALID_REPORT', 'INPUT_EXPIRED']


class Insights:
    LEASE = 20
    JOB_TTL = 150
    REPORT_TTL = 120

    def __init__(self):
        self.enabled = False
        self.worker = None
        self.model = None
        self.seen = 0.
        self.job = None
        self.report = None
        self.error = None
        self.completed = 0

    def invalidate(self):
        self.job, self.report = None, None

    def control(self, enabled):
        self.enabled = enabled
        self.job = None  # Pausing invalidates any in-flight inference.
        self.error = None

    def heartbeat(self, worker):
        if self.worker != worker.worker and time.monotonic() - self.seen < self.LEASE:
            raise ConsoleError('OBSERVER_BUSY', '另一台 Mac 的观察进程仍在线')
        if self.worker != worker.worker:
            self.job = None
        self.worker, self.model, self.seen = worker.worker, worker.model, time.monotonic()

    def require_worker(self, worker):
        if worker.worker != self.worker or time.monotonic() - self.seen >= self.LEASE or worker.model != self.model:
            raise ConsoleError('OBSERVER_LEASE_EXPIRED', '观察进程心跳失效，请重新注册')

    def capture(self, worker, state, raw):
        self.require_worker(worker)
        if not self.enabled:
            raise ConsoleError('OBSERVER_PAUSED', '自主观察已暂停')
        if self.job and time.monotonic() - self.job['started'] < self.JOB_TTL:
            raise ConsoleError('OBSERVER_BUSY', '已有一份观察正在分析')
        if state['mode'] != 'robot' or not state['read_only'] or not state['connected']:
            raise ConsoleError('OBSERVER_INPUT_UNAVAILABLE', '需要真实 G1 的新鲜身体反馈和只读模式')
        if state['camera']['status'] != 'live' or state['camera']['backend'] != 'unitree' or not raw:
            raise ConsoleError('OBSERVER_INPUT_UNAVAILABLE', '需要新鲜的官方相机画面')
        ticket = uuid4().hex
        evidence = dict(captured_at=state['at'], run_id=state['run_id'], camera=state['camera'],
                        image_sha256=hashlib.sha256(raw).hexdigest(), telemetry_age_ms=state['telemetry_age_ms'],
                        tick=state['telemetry'].get('tick'), joints=len(state['telemetry']['joints']),
                        foundation_id=KNOWLEDGE['foundation_id'])
        caps = [{k: c[k] for k in ('id', 'label', 'availability', 'compatibility') if k in c} for c in state['capabilities']]
        self.job = dict(ticket=ticket, started=time.monotonic(), evidence=evidence,
                        capability_ids={c['id'] for c in caps})
        self.error = None
        return dict(ticket=ticket, evidence=evidence, telemetry=state['telemetry'], hardware=state['hardware'],
                    foundation=KNOWLEDGE, assessment=state.get('assessment'),
                    capabilities=caps, geometry=dict(source=state['world']['source'], age_ms=state['world']['age_ms'],
                    pose_available=state['world']['pose'] is not None, scan_points=len(state['world']['scan'])),
                    previous=self.report['report'] if self.report else None,
                    jpeg_base64=base64.b64encode(raw).decode())

    def submit(self, submission, state):
        self.require_worker(submission)
        job = self.job
        if not self.enabled or not job or job['ticket'] != submission.ticket:
            raise ConsoleError('OBSERVER_TICKET_EXPIRED', '观察已暂停或输入批次已失效')
        evidence = job['evidence']
        if (time.monotonic() - job['started'] >= self.JOB_TTL or not state['connected']
                or state['camera']['status'] != 'live' or evidence['run_id'] != state['run_id']
                or evidence['camera']['source'] != state['camera']['source']):
            self.job = None
            raise ConsoleError('OBSERVER_INPUT_EXPIRED', '输入来源已改变或断流，丢弃本次分析')
        if submission.report:
            if submission.error or any(cap not in job['capability_ids'] for s in submission.report.suggestions for cap in s.capability_ids):
                raise ConsoleError('INVALID_REPORT', '建议引用了未知能力或报告状态冲突')
            # Bound arbitrary strings too, rather than accepting unbounded model prose.
            if any(len(s) > 500 for s in submission.report.uncertainties) or any(len(s) > 500 for v in submission.report.suggestions for s in v.requirements):
                raise ConsoleError('INVALID_REPORT', '观察报告过长')
            self.completed += 1
            self.report = dict(report=submission.report.model_dump(), evidence=evidence, model=self.model,
                               completed_at=time.time(), analysis_ms=round((time.monotonic()-job['started'])*1000))
            self.error = None
        elif not submission.error:
            raise ConsoleError('INVALID_REPORT', '报告和错误不能同时为空')
        else:
            self.error = submission.error
        self.job = None

    def snapshot(self, connected, camera):
        worker_online = time.monotonic() - self.seen < self.LEASE
        age = max(0, time.time()-self.report['evidence']['captured_at']) if self.report else None
        current = bool(self.enabled and worker_online and connected and camera['status'] == 'live' and self.report
                       and age < self.REPORT_TTL and self.report['evidence']['camera']['source'] == camera['source'])
        return dict(enabled=self.enabled, worker_online=worker_online, model=self.model, error=self.error,
                    status='paused' if not self.enabled else 'waiting_worker' if not worker_online else
                    'waiting_input' if not connected or camera['status'] != 'live' else
                    'analyzing' if self.job and time.monotonic()-self.job['started'] < self.JOB_TTL else
                    'error' if self.error else 'observing',
                    report=self.report, report_age_s=round(age, 1) if age is not None else None,
                    current=current, completed=self.completed, report_ttl_s=self.REPORT_TTL)
