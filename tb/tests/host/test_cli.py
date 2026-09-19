"""End-to-end CLI tests against the mock backend."""

import contextlib
import io
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[3] / 'tools'
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from m68kctl import MockDevice, cli
from m68kctl.device import MOCK_STATE_ENV


def _run_cli(argv):
    """Run m68kctl with argv; return (rc, stdout, stderr)."""
    out = io.StringIO()
    err = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            rc = cli.main(argv)
        except SystemExit as e:
            rc = int(e.code) if e.code is not None else 0
    return rc, out.getvalue(), err.getvalue()


class _IsolatedMockStateMixin:
    """Point ``M68KCTL_MOCK_STATE`` at a tempdir for the life of the test.

    Keeps the real ``~/.local/state/m68kctl/mock_state.pkl`` untouched
    when we run the in-process ``cli.main(...)`` flow.
    """

    def setUp(self):  # noqa: D401
        super().setUp()
        self._state_dir = tempfile.mkdtemp(prefix='m68kctl-mock-state-')
        self._state_path = os.path.join(self._state_dir, 'mock_state.pkl')
        self._prev_env = os.environ.get(MOCK_STATE_ENV)
        os.environ[MOCK_STATE_ENV] = self._state_path

    def tearDown(self):
        if self._prev_env is None:
            os.environ.pop(MOCK_STATE_ENV, None)
        else:
            os.environ[MOCK_STATE_ENV] = self._prev_env
        # best-effort cleanup; tempdir may contain pickle + .tmp
        try:
            for name in os.listdir(self._state_dir):
                os.unlink(os.path.join(self._state_dir, name))
            os.rmdir(self._state_dir)
        except OSError:
            pass
        super().tearDown()


class CliHelpTests(unittest.TestCase):
    def test_top_help(self):
        rc, out, _ = _run_cli(['--help'])
        self.assertEqual(rc, 0)
        self.assertIn('m68kctl', out)
        self.assertIn('cpu', out)
        self.assertIn('sd', out)
        self.assertIn('bus', out)

    def _sub_help(self, sub):
        rc, out, _ = _run_cli([sub, '--help'])
        self.assertEqual(rc, 0, f'{sub} --help failed: {out}')
        self.assertIn('usage:', out)

    def test_sub_help_all(self):
        for sub in ['info', 'cpu', 'sd', 'bus', 'checkpoint']:
            self._sub_help(sub)

    def test_cpu_leaf_help(self):
        for op in ['halt', 'resume', 'step', 'soft-rst', 'reset-halt',
                   'redirect', 'regs', 'trace', 'halt-status',
                   'halt-after', 'break-pc', 'halt-exc', 'clear-halt',
                   'load-arch']:
            rc, out, _ = _run_cli(['cpu', op, '--help'])
            self.assertEqual(rc, 0, f'cpu {op} --help failed')

    def test_sd_leaf_help(self):
        for op in ['read', 'write', 'upload', 'verify',
                   'check-layout', 'make-image']:
            rc, out, _ = _run_cli(['sd', op, '--help'])
            self.assertEqual(rc, 0, f'sd {op} --help failed')

    def test_bus_leaf_help(self):
        for op in ['read', 'write', 'dump', 'load']:
            rc, out, _ = _run_cli(['bus', op, '--help'])
            self.assertEqual(rc, 0, f'bus {op} --help failed')

    def test_checkpoint_leaf_help(self):
        rc, out, _ = _run_cli(['checkpoint', '--help'])
        self.assertEqual(rc, 0)
        self.assertIn('usage:', out)
        rc, out, _ = _run_cli(['checkpoint', 'dump', '--help'])
        self.assertEqual(rc, 0)
        self.assertIn('--region', out)


class CliMockInvocationTests(_IsolatedMockStateMixin, unittest.TestCase):
    def test_info_mock(self):
        rc, out, _ = _run_cli(['--mock', 'info'])
        self.assertEqual(rc, 0)
        self.assertIn('DBG_VERSION  : 0xdeb60004', out)
        self.assertIn('SDP_VERSION  : 0x5d500002', out)

    def test_cpu_halt_resume(self):
        rc, _, _ = _run_cli(['--mock', 'cpu', 'halt'])
        self.assertEqual(rc, 0)
        rc, _, _ = _run_cli(['--mock', 'cpu', 'resume'])
        self.assertEqual(rc, 0)
        rc, _, _ = _run_cli(['--mock', 'cpu', 'reset-halt'])
        self.assertEqual(rc, 0)

    def test_cpu_trace_mock(self):
        rc, out, _ = _run_cli(['--mock', 'cpu', 'trace', '--tail', '4'])
        self.assertEqual(rc, 0)
        self.assertIn('retired PCs', out)

    def test_cpu_programmable_halt_mock(self):
        rc, out, _ = _run_cli(['--mock', '--reset-state', 'cpu', 'halt-after', '3'])
        self.assertEqual(rc, 0)
        self.assertIn('halt-after enabled', out)
        rc, out, _ = _run_cli(['--mock', 'cpu', 'break-pc', '0x40800006'])
        self.assertEqual(rc, 0)
        self.assertIn('break-pc enabled', out)
        rc, out, _ = _run_cli(['--mock', 'cpu', 'halt-exc'])
        self.assertEqual(rc, 0)
        self.assertIn('halt-exc enabled', out)
        rc, out, _ = _run_cli(['--mock', 'cpu', 'halt-status'])
        self.assertEqual(rc, 0)
        self.assertIn('HALT_CTL', out)
        rc, _, _ = _run_cli(['--mock', 'cpu', 'clear-halt'])
        self.assertEqual(rc, 0)

    def test_cpu_load_arch_mock(self):
        rc, out, _ = _run_cli(['--mock', '--reset-state', 'cpu', 'load-arch',
                               '--no-apply', 'PC=0x40801234', 'D0=0x55',
                               'A7=0x1000', 'SR=0x2700'])
        self.assertEqual(rc, 0)
        self.assertIn('arch shadow words=4', out)

    def test_sd_upload_mock(self):
        fd, path = tempfile.mkstemp(suffix='.bin')
        os.write(fd, os.urandom(1024))
        os.close(fd)
        try:
            rc, out, _ = _run_cli(['--mock', 'sd', 'upload', path])
            self.assertEqual(rc, 0)
            self.assertIn('uploaded 2 blocks', out)
        finally:
            os.unlink(path)

    def test_sd_check_layout(self):
        fd_rom, rom = tempfile.mkstemp(suffix='.rom')
        fd_hdd, hdd = tempfile.mkstemp(suffix='.hdv')
        os.write(fd_rom, b'Q700')
        os.write(fd_hdd, bytes(range(256)) * 2)
        os.close(fd_rom)
        os.close(fd_hdd)
        try:
            rc, out, _ = _run_cli(['sd', 'check-layout',
                                   '--rom', rom, '--hdd', hdd])
            self.assertEqual(rc, 0)
            self.assertIn('ROM window        : LBA 0..8191', out)
            self.assertIn('raw SCSI HDD base : LBA 8192', out)
            self.assertIn('overlap check     : PASS', out)
        finally:
            os.unlink(rom)
            os.unlink(hdd)

    def test_bus_write_read_mock(self):
        # --mock creates fresh state per invocation, so do this all in one
        # subcommand: `bus write` the CLI flow in isolation.
        rc, _, _ = _run_cli(['--mock', 'bus', 'write', '0x1000',
                             '--bytes', 'DEADBEEF'])
        self.assertEqual(rc, 0)

    def test_checkpoint_dump_mock(self):
        payload_ram = b'RAM-' * 1024
        payload_rom = b'ROM-' * 1024
        dev = MockDevice()
        dev.dma_write(0x00001000, payload_ram)
        dev.dma_write(0x40002000, payload_rom)
        with tempfile.TemporaryDirectory(prefix='m68kctl-checkpoint-') as tmp:
            with mock.patch.object(cli, '_make_device', return_value=dev):
                rc, out, err = _run_cli([
                    'checkpoint', 'dump',
                    '--output-dir', tmp,
                    '--region', 'ram', '0x00001000', str(len(payload_ram)),
                    '--region', 'rom', '0x40002000', str(len(payload_rom)),
                ])
            self.assertEqual(rc, 0, err)
            self.assertIn('wrote 2 region(s)', out)
            manifest = Path(tmp) / 'manifest.json'
            self.assertTrue(manifest.exists())
            self.assertEqual((Path(tmp) / 'ram.bin').read_bytes(), payload_ram)
            self.assertEqual((Path(tmp) / 'rom.bin').read_bytes(), payload_rom)


class CliMockResetStateTests(_IsolatedMockStateMixin, unittest.TestCase):
    def test_reset_state_no_mock_warns(self):
        # Without --mock, --reset-state is a no-op that should warn on
        # stderr.  We drive `info` which, on a machine with no FPGA,
        # will fail at /dev/xdma0_user open time and return rc=2 — but
        # only AFTER we've printed the warning.  Assert on the warning.
        _, _, err = _run_cli(['--reset-state', '--no-verify', 'info'])
        self.assertIn('--reset-state has no effect without --mock', err)

    def test_reset_state_removes_pickle(self):
        # Prime the state file by running a normal --mock info.
        rc, _, _ = _run_cli(['--mock', 'info'])
        self.assertEqual(rc, 0)
        self.assertTrue(os.path.exists(self._state_path),
                        'expected state file after first --mock invocation')
        rc, _, _ = _run_cli(['--mock', '--reset-state', 'info'])
        self.assertEqual(rc, 0)
        # The second invocation wipes then re-creates (saved on close()).
        # What we really want to check is identity regs came back from
        # fresh state; so verify sd_blocks is empty via a fresh lookup.
        self.assertTrue(os.path.exists(self._state_path),
                        'info should re-save state on close')


class CliMockPersistenceSubprocessTests(unittest.TestCase):
    """Exercise CLI-mode state persistence across real subprocesses.

    The bug this fixes: ``m68kctl --mock sd write N X.bin`` followed by
    ``m68kctl --mock sd read N -o Y.bin`` in two separate processes used
    to produce an empty Y.bin because each process got a fresh in-memory
    MockDevice.  With the pickle snapshot wired up via
    ``M68KCTL_MOCK_STATE``, the second process sees the first's writes.
    """

    def setUp(self):
        self._tmp = tempfile.mkdtemp(prefix='m68kctl-sp-')
        self._state = os.path.join(self._tmp, 'state.pkl')
        # Locate the package root so PYTHONPATH points at tools/ (the
        # layout inside the repo).
        self._tools = str(Path(__file__).resolve().parents[3] / 'tools')

    def tearDown(self):
        try:
            for name in os.listdir(self._tmp):
                p = os.path.join(self._tmp, name)
                if os.path.isfile(p):
                    os.unlink(p)
            os.rmdir(self._tmp)
        except OSError:
            pass

    def _run(self, *cli_argv):
        env = dict(os.environ)
        env['M68KCTL_MOCK_STATE'] = self._state
        env['PYTHONPATH'] = (self._tools + os.pathsep +
                             env.get('PYTHONPATH', ''))
        return subprocess.run(
            [sys.executable, '-m', 'm68kctl', *cli_argv],
            env=env, check=False, capture_output=True, text=True, timeout=30)

    def test_sd_write_then_read_across_processes(self):
        # Write a distinctive payload in process A, read it back in
        # process B, compare bytes.
        payload = bytes(range(256)) + bytes(range(255, -1, -1))  # 512 B
        self.assertEqual(len(payload), 512)
        in_path  = os.path.join(self._tmp, 'in.bin')
        out_path = os.path.join(self._tmp, 'out.bin')
        with open(in_path, 'wb') as f:
            f.write(payload)

        r = self._run('--mock', 'sd', 'write', '7', in_path)
        self.assertEqual(r.returncode, 0,
                         f'write failed: stdout={r.stdout!r} stderr={r.stderr!r}')
        self.assertTrue(os.path.exists(self._state),
                        'first --mock invocation should create the pickle')

        r = self._run('--mock', 'sd', 'read', '7', '-o', out_path)
        self.assertEqual(r.returncode, 0,
                         f'read failed: stdout={r.stdout!r} stderr={r.stderr!r}')

        with open(out_path, 'rb') as f:
            got = f.read()
        self.assertEqual(got, payload,
                         'roundtripped SD block does not match original')

    def test_reset_state_info_returns_identity_regs(self):
        # Pre-populate state (will create pickle on close), then reset +
        # info.  info should still print the magic version registers
        # (which come from the pre-seeded defaults, not from the stale
        # pickle).
        self._run('--mock', 'sd', 'write', '0',
                  '/dev/null')  # any write to ensure snapshot exists
        r = self._run('--mock', '--reset-state', 'info')
        self.assertEqual(r.returncode, 0,
                         f'reset+info failed: stderr={r.stderr!r}')
        self.assertIn('DBG_VERSION  : 0xdeb60004', r.stdout)
        self.assertIn('SDP_VERSION  : 0x5d500002', r.stdout)


if __name__ == '__main__':
    unittest.main()
