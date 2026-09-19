"""Unit tests for the low-level ``device.py`` backends — currently
only covers ``MockDevice`` persistence, since ``XdmaDevice`` needs a
real FPGA to exercise meaningfully.
"""

import os
import pickle
import sys
import tempfile
import unittest
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[3] / 'tools'
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from m68kctl import regs
from m68kctl.device import (
    MOCK_STATE_ENV,
    MockDevice,
    default_mock_state_path,
    reset_mock_state,
)


class _TempStateMixin:
    def setUp(self):  # noqa: D401
        super().setUp()
        self._tmp = tempfile.mkdtemp(prefix='m68kctl-dev-')
        self._path = os.path.join(self._tmp, 'state.pkl')

    def tearDown(self):
        for name in os.listdir(self._tmp):
            try: os.unlink(os.path.join(self._tmp, name))
            except OSError: pass
        try: os.rmdir(self._tmp)
        except OSError: pass
        super().tearDown()


class MockDeviceInMemoryOnlyTests(unittest.TestCase):
    """Without ``state_path`` or env var, MockDevice stays in-memory."""

    def setUp(self):
        # Ensure the env var isn't leaking in from a parent shell.
        self._prev = os.environ.pop(MOCK_STATE_ENV, None)

    def tearDown(self):
        if self._prev is not None:
            os.environ[MOCK_STATE_ENV] = self._prev

    def test_no_state_file_side_effects(self):
        dev = MockDevice()
        dev.mmio_write32(1, 0x2000, 0xCAFEBABE)
        dev.sd_blocks[5] = b'\xAA' * 512
        dev.close()
        # If MockDevice secretly created a default state file, fail.
        self.assertFalse(os.path.exists(default_mock_state_path()) and
                         self._prev is None and
                         os.path.getmtime(default_mock_state_path()) > 0,
                         'close() with no state_path should NOT write')


class MockDevicePersistenceTests(_TempStateMixin, unittest.TestCase):
    def test_empty_state_file_roundtrip(self):
        # First MockDevice: no file yet → starts clean.
        dev = MockDevice(state_path=self._path)
        dev.mmio_write32(1, 0x3000, 0xDEADBEEF)
        dev.sd_blocks[9] = bytes(range(256)) * 2
        dev.close()
        self.assertTrue(os.path.exists(self._path))

        # Second MockDevice: loads snapshot, sees prior writes.
        dev2 = MockDevice(state_path=self._path)
        self.assertEqual(dev2.mmio_read32(1, 0x3000), 0xDEADBEEF)
        self.assertEqual(dev2.sd_blocks[9], bytes(range(256)) * 2)
        dev2.close()

    def test_env_var_opt_in(self):
        os.environ[MOCK_STATE_ENV] = self._path
        try:
            dev = MockDevice()
            self.assertEqual(dev._state_path, self._path)
            dev.mmio_write32(1, 0x4000, 0x12345678)
            dev.close()
            # Reopen without explicit path: env still points here
            dev2 = MockDevice()
            self.assertEqual(dev2.mmio_read32(1, 0x4000), 0x12345678)
            dev2.close()
        finally:
            os.environ.pop(MOCK_STATE_ENV, None)

    def test_explicit_path_overrides_env(self):
        os.environ[MOCK_STATE_ENV] = os.path.join(self._tmp, 'bogus.pkl')
        try:
            dev = MockDevice(state_path=self._path)
            self.assertEqual(dev._state_path, self._path)
            dev.close()
            self.assertTrue(os.path.exists(self._path))
            self.assertFalse(os.path.exists(os.environ[MOCK_STATE_ENV]))
        finally:
            os.environ.pop(MOCK_STATE_ENV, None)

    def test_missing_file_starts_clean(self):
        # Not there yet — should silently start empty.
        dev = MockDevice(state_path=self._path)
        # version magics still seeded from defaults
        self.assertEqual(dev.mmio_read32(1, regs.OFF_DBG_VERSION),
                         regs.DBG_VERSION_MAGIC)
        dev.close()

    def test_corrupt_pickle_starts_clean(self):
        with open(self._path, 'wb') as f:
            f.write(b'this is not a valid pickle')
        dev = MockDevice(state_path=self._path)
        # Still works, just empty SD and reads version magic.
        self.assertEqual(dev.mmio_read32(1, regs.OFF_DBG_VERSION),
                         regs.DBG_VERSION_MAGIC)
        self.assertEqual(dev.sd_blocks, {})
        dev.close()

    def test_non_dict_snapshot_starts_clean(self):
        with open(self._path, 'wb') as f:
            pickle.dump(['not', 'a', 'dict'], f)
        dev = MockDevice(state_path=self._path)
        self.assertEqual(dev.sd_blocks, {})
        dev.close()

    def test_close_is_idempotent(self):
        dev = MockDevice(state_path=self._path)
        dev.sd_blocks[1] = b'\xFF' * 512
        dev.close()
        mtime1 = os.path.getmtime(self._path)
        dev.close()  # second call must not re-save or error
        mtime2 = os.path.getmtime(self._path)
        self.assertEqual(mtime1, mtime2)

    def test_context_manager_saves_on_exit(self):
        with MockDevice(state_path=self._path) as dev:
            dev.sd_blocks[13] = b'\x13' * 512
        self.assertTrue(os.path.exists(self._path))
        with MockDevice(state_path=self._path) as dev2:
            self.assertEqual(dev2.sd_blocks[13], b'\x13' * 512)

    def test_partial_snapshot_keys_ok(self):
        # A forward-compat snapshot that only has mmio should work.
        with open(self._path, 'wb') as f:
            pickle.dump({'mmio': {1: {0x1234: 0xAABBCCDD}}}, f)
        dev = MockDevice(state_path=self._path)
        self.assertEqual(dev.mmio_read32(1, 0x1234), 0xAABBCCDD)
        self.assertEqual(dev.sd_blocks, {})
        dev.close()

    def test_version_magics_survive_load(self):
        # If a snapshot was written before the version magic was seeded,
        # load MUST still expose the correct magic values (merge, not
        # replace semantics).
        with open(self._path, 'wb') as f:
            pickle.dump({'mmio': {1: {0x6000: 0xFEEDFACE}}}, f)
        dev = MockDevice(state_path=self._path)
        self.assertEqual(dev.mmio_read32(1, 0x6000), 0xFEEDFACE)
        self.assertEqual(dev.mmio_read32(1, regs.OFF_DBG_VERSION),
                         regs.DBG_VERSION_MAGIC)
        dev.close()


class ResetMockStateTests(_TempStateMixin, unittest.TestCase):
    def test_reset_deletes_file(self):
        with open(self._path, 'wb') as f:
            pickle.dump({'mmio': {}, 'sd_blocks': {}}, f)
        self.assertTrue(reset_mock_state(self._path))
        self.assertFalse(os.path.exists(self._path))

    def test_reset_on_missing_file_is_ok(self):
        self.assertFalse(reset_mock_state(self._path))


if __name__ == '__main__':
    unittest.main()
