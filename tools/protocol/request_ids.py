"""Local durable fixed64 allocator. Shared state prevents ID reuse, not network reordering."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import tempfile
import time

DEFAULT_STATE = Path(__file__).resolve().parent / '.state' / 'request_ids.json'
MAX_ID = (1 << 64) - 1


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'Duplicate JSON field: {key}')
        result[key] = value
    return result


def atomic_write(path, data):
    """Flush contents before replacing a file in the same directory."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as file:
            name = file.name
            file.write(data)
            file.flush()
            os.fsync(file.fileno())
        os.replace(name, path)
        name = None
        if os.name != 'nt':
            fd = os.open(path.parent, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
    finally:
        if name is not None:
            os.unlink(name)


@contextmanager
def state_lock(path):
    """OS-released lock survives process crashes without a stale lock-file protocol."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix(path.suffix + '.lock').open('a+b') as lock:
        if lock.seek(0, os.SEEK_END) == 0:
            lock.write(b'\0')
            lock.flush()
        deadline = time.monotonic() + 5
        while True:
            lock.seek(0)
            try:
                if os.name == 'nt':
                    import msvcrt
                    msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Request-ID state is locked by another client')
                time.sleep(0.02)
        try:
            yield
        finally:
            lock.seek(0)
            if os.name == 'nt':
                msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def allocate_request_id(path=DEFAULT_STATE, *, clock_ns=None, floor=0):
    """Reserve before publish: max(persisted+1, clock, optional observed floor+1).

    Missing state starts a new local history. Corrupt state fails closed. Use a
    single publisher; separate stores/machines and arrival order are not coordinated.
    """
    if type(floor) is not int or not 0 <= floor <= MAX_ID:
        raise ValueError('ID floor must be uint64')
    path = Path(path)
    with state_lock(path):
        previous = 0
        if path.exists():
            state = json.loads(path.read_text(encoding='utf8'), object_pairs_hook=unique_object)
            if (not isinstance(state, dict) or set(state) != {'format_version', 'last_id'} or
                    type(state['format_version']) is not int or state['format_version'] != 1 or
                    not isinstance(state['last_id'], str) or not state['last_id'].isascii() or
                    not state['last_id'].isdecimal() or len(state['last_id']) > 20):
                raise ValueError('Invalid request-ID state; do not silently reset it')
            previous = int(state['last_id'])
            if not 1 <= previous <= MAX_ID:
                raise ValueError('Invalid persisted request ID')
        now = time.time_ns() if clock_ns is None else clock_ns()
        if type(now) is not int:
            raise ValueError('Clock must return integer nanoseconds')
        value = max(1, now, previous + 1, floor + 1)
        if value > MAX_ID:
            raise ValueError('fixed64 request-ID space exhausted')
        atomic_write(path, (json.dumps({'format_version': 1, 'last_id': str(value)}) + '\n').encode())
        return value
