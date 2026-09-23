"""GUI-free IPC and pacing checks for the isolated native observer."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
import fcntl
import time

import numpy as np
import pytest

from scripts import dispatch_native_process as native


def test_mailbox_copy_is_coherent_between_independent_descriptors(tmp_path):
    path = tmp_path / 'state'
    writer = native.StateMailbox(path, 8, create=True)
    reader = native.StateMailbox(path, 8)
    try:
        def publish():
            for seq in range(1, 1001):
                def write(array):
                    array[native.HEADER:] = seq
                    array[native.SEQ] = seq
                    return True
                while writer.transact(write) is None:
                    pass

        def inspect():
            seen = 0
            for _ in range(2000):
                def read(array):
                    return int(array[native.SEQ]), array[native.HEADER:].copy()
                snapshot = reader.transact(read)
                if snapshot is not None:
                    seq, state = snapshot
                    assert np.all(state == seq)
                    seen = max(seen, seq)
            return seen

        with ThreadPoolExecutor(max_workers=2) as executor:
            written = executor.submit(publish)
            observed = executor.submit(inspect)
            written.result()
            observed.result()
        sequence, state = reader.transact(lambda a: (int(a[native.SEQ]), a[native.HEADER:].copy()))
        assert sequence == 1000
        np.testing.assert_array_equal(state, np.full(8, 1000.))
    finally:
        reader.close()
        writer.close()


def test_busy_descriptor_returns_immediately_without_blocking_physics(tmp_path):
    path = tmp_path / 'state'
    mailbox = native.StateMailbox(path, 4, create=True)
    locker = path.open('r+b')
    try:
        fcntl.flock(locker.fileno(), fcntl.LOCK_EX)
        started = time.perf_counter()
        assert mailbox.transact(lambda _: pytest.fail('busy mailbox was mutated')) is None
        assert time.perf_counter() - started < .1
    finally:
        fcntl.flock(locker.fileno(), fcntl.LOCK_UN)
        locker.close()
        mailbox.close()


def test_pause_counter_quit_and_graceful_stop_protocol(tmp_path, monkeypatch):
    path = tmp_path / 'state'
    mailbox = native.StateMailbox(path, 1, create=True)
    view = native.IsolatedDispatchNativeView.__new__(native.IsolatedDispatchNativeView)
    view.scene = SimpleNamespace(deadline=None, out=tmp_path)
    view.mailbox = mailbox
    view.process = SimpleNamespace(poll=lambda: None)
    view.paused = view.rebase_pace = False
    view.pause_seen = 0
    view.next_sync = 1e9
    view.stats = {'mode': 'test'}
    view.closed = False
    try:
        mailbox.transact(lambda a: a.__setitem__(native.PAUSES, 2))
        view.poll()  # Two queued Space events cancel, even if no intermediate read.
        assert not view.paused and view.pause_seen == 2

        mailbox.transact(lambda a: a.__setitem__(native.PAUSES, 3))
        real_sleep = time.sleep
        calls = []
        def resume(_seconds):
            calls.append(True)
            mailbox.transact(lambda a: a.__setitem__(native.PAUSES, 4))
        monkeypatch.setattr(native.time, 'sleep', resume)
        view.poll()
        assert calls and not view.paused and view.rebase_pace
        monkeypatch.setattr(native.time, 'sleep', real_sleep)

        mailbox.transact(lambda a: a.__setitem__(native.QUIT, 1))
        with pytest.raises(KeyboardInterrupt, match='operator quit'):
            view.poll()
        mailbox.transact(lambda a: a.__setitem__(native.QUIT, 0))

        events = []
        class Process:
            def poll(self): return None
            def wait(self, timeout):
                events.append(('wait', timeout))
                assert mailbox.transact(lambda a: int(a[native.STOP])) == 1
                return 0
            def terminate(self): pytest.fail('graceful child was terminated')
            def kill(self): pytest.fail('graceful child was killed')
        view.process = Process()
        view.scratch = SimpleNamespace(cleanup=lambda: events.append('cleanup'))
        view.log = SimpleNamespace(close=lambda: events.append('logclose'))
        view.close()
        assert ('wait', 3) in events
        assert 'cleanup' in events and 'logclose' in events
        assert (tmp_path / 'native-observer.json').exists()
        view.close()
        assert events.count(('wait', 3)) == 1
    finally:
        if view.mailbox.array is not None:
            mailbox.close()


@pytest.mark.parametrize('factor,expected_wall', [(1., 1.), (2., .5)])
def test_inherited_pacing_factor_and_oversleep_rebase(monkeypatch, factor, expected_wall):
    from scripts import dispatch_native_view

    class Clock:
        now = 0.
        sleeps = []
        def monotonic(self): return self.now
        def sleep(self, seconds):
            self.sleeps.append(seconds)
            self.now += seconds + .002

    clock = Clock()
    monkeypatch.setattr(native, 'time', clock)
    monkeypatch.setattr(dispatch_native_view, 'time', clock)
    sim = [0.]
    view = native.IsolatedDispatchNativeView.__new__(native.IsolatedDispatchNativeView)
    view.scene = SimpleNamespace(time=lambda: sim[0], deadline=None)
    view.factor, view.paused = factor, False
    view.pace_sim = view.pace_wall = view.last_tick_wall = 0.
    view.rebase_pace, view.next_poll, view.next_sync = False, 0., float('inf')
    view.process = SimpleNamespace(poll=lambda: None)
    view.pause_seen = 0
    view._exchange = lambda _publish: np.zeros(native.HEADER)
    for _ in range(4000):
        view.tick()
        sim[0] += .00025
        clock.now += .00001
    assert abs(clock.now - expected_wall) < .012
    assert len(clock.sleeps) < 220
    clock.now += 10.  # Inference pause must not cause wall-time catch-up.
    view.tick()
    assert view.pace_wall >= 10.
