"""Durable allowlisted experiment records. Unknown physical facts stay null."""
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import threading
import time


METADATA_FIELDS = ('operator', 'observer', 'robot_wall_time_raw', 'entrypoint', 'code_identity',
                   'source_manifest_identity', 'config_identity', 'layout_id', 'map_identity',
                   'alignment_id', 'waypoint_identity', 'calibration_id', 'camera_mount_id',
                   'tool_id', 'speed_profile', 'hardware_ready', 'data_origin', 'recording_policy',
                   'execution_backend','code_files','config','resource_files')


def file_identity(path):
    if not path:
        return None
    candidate = Path(path).expanduser().resolve()
    if not candidate.exists():
        return {'path':str(candidate),'exists':False}
    stat=candidate.stat()
    return {'path':str(candidate),'size':stat.st_size,'modified_ns':stat.st_mtime_ns}


def validate_topics(topics):
    if (not isinstance(topics, (list, tuple)) or not topics or len(set(topics)) != len(topics)
            or any(not isinstance(topic, str) or not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_/]*', topic)
                   for topic in topics)):
        raise ValueError('record_topics must be an explicit nonempty unique absolute topic whitelist')
    return tuple(topics)


class RunRecorder:
    def __init__(self, root, run_id, scope, topics, metadata=None, clock=time.monotonic):
        if not isinstance(run_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', run_id):
            raise ValueError('invalid run directory ID')
        if scope not in ('cake-only', 'full'):
            raise ValueError('invalid recording scope')
        self.topics = validate_topics(topics)
        self.clock = clock
        self.lock = threading.RLock()
        self.path = Path(root).expanduser().resolve() / run_id
        self.path.mkdir(parents=True, exist_ok=False)
        self.error = None
        self.started_at = None
        self.finished = False
        self.interventions = []
        self.meta = {key: (metadata or {}).get(key) for key in METADATA_FIELDS}
        self.meta.update(schema=1, record_kind='automatic_runtime', run_id=run_id, scope=scope,
                         client_date=datetime.now(timezone.utc).isoformat(),
                         record_topics=list(self.topics), start_event_monotonic=None,
                         original_deadline_monotonic=None)
        self._json('meta.json', self.meta)
        self.event('prepared', {'scope': scope, 'topics': list(self.topics)})

    def _json(self, name, value):
        temporary = self.path / (name + '.pending')
        try:
            with temporary.open('w', encoding='utf-8') as stream:
                json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(str(temporary), str(self.path / name))
        except Exception as exc:
            self.error = str(exc)
            raise

    def _append(self, name, value):
        try:
            target = self.path / name
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
                stream.flush()
                os.fsync(stream.fileno())
        except Exception as exc:
            self.error = str(exc)
            raise

    def event(self, kind, details=None):
        with self.lock:
            if self.finished:
                raise ValueError('run recording is already finished')
            self._append('events.jsonl', {'kind': kind, 'monotonic': self.clock(), 'details': details or {}})

    def start(self, started_at, deadline, budget):
        with self.lock:
            if self.started_at is not None or self.finished:
                raise ValueError('start event is single-use')
            if (not all(type(value) in (int, float) and math.isfinite(value) for value in (started_at, deadline))
                    or deadline <= started_at):
                raise ValueError('invalid monotonic start or deadline')
            self.started_at = started_at
            self.meta.update(start_event_monotonic=started_at, original_deadline_monotonic=deadline,
                             time_budget=dict(budget))
            self._json('meta.json', self.meta)
            self.event('start', {'started_at': started_at, 'deadline': deadline})

    def intervention(self, reason):
        with self.lock:
            item = {'reason': reason, 'monotonic': self.clock()}
            self.interventions.append(item)
            self.event('intervention', item)

    def topic(self, topic, payload):
        with self.lock:
            if topic not in self.topics:
                raise ValueError('topic is outside the recording whitelist')
            if not self.finished:
                self._append('data/topics.jsonl', {'topic': topic, 'monotonic': self.clock(), 'payload': payload})

    def finish(self, report, final_state=None):
        with self.lock:
            if self.finished:
                raise ValueError('result is single-use')
            if report.get('run_id') != self.meta['run_id']:
                raise ValueError('result run_id does not match the prepared record')
            now = self.clock()
            result = dict(schema=1, record_kind='automatic_runtime', run_id=self.meta['run_id'],
                          scope=self.meta['scope'], execution=report.get('status', 'unknown'),
                          acceptance='not_evaluated', validation_type=self.meta.get('data_origin'),
                          exit_reason=report.get('error') or report.get('status', 'unknown'),
                          delivered_confirmed=report.get('delivered'), report=dict(report),
                          manual_intervention=bool(self.interventions), interventions=list(self.interventions),
                          duration_monotonic_s=None if self.started_at is None else now - self.started_at,
                          physical_motion_observed=None, base_stationary_confirmed_at=None,
                          field_judged_score=None, software_score_estimate=None,
                          arm_hold_state='unknown', pump_state='unknown', final_pose_observed=None,
                          final_state=final_state or {}, recording_error=self.error)
            self.event('finished', {'status': result['execution']})
            self._json('result.json', result)
            self.finished = True
            return result
