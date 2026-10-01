import tempfile
import unittest
from pathlib import Path
import parallel_chase_rom as parallel


class ParallelSequenceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'bus'
        links = parallel.ring()
        nodes = [c * parallel.PER_CHAIN for c in range(4)]
        events = []
        tick = 2
        for step in range(4 * parallel.PER_CHAIN):
            order = [(step+c) % 4 for c in range(4)]
            for chain in order:
                address = parallel.single.RING_BASE + nodes[chain] * 64
                events.append(f'{tick} AR {chain} {address:08x} 0 4 1')
                nodes[chain] = links[nodes[chain]]
            tick += 2
            for chain in reversed(order):
                events.append(f'{tick} RV {chain} 0 1 1')
            tick += 2
        self.events = events
        self.timing = dict(start_tick=0, end_tick=tick)

    def check_events(self, events):
        self.path.write_text('\n'.join(events)+'\n')
        return parallel.validate_requests(self.path, self.timing)

    def test_independent_chains_allow_different_interleavings(self):
        result = self.check_events(self.events)
        self.assertEqual(result['pointer_reads_per_chain'], [1024]*4)
        self.assertEqual(result['max_outstanding_hot_reads'], 4)

    def test_wrong_next_pointer_is_rejected(self):
        events = self.events.copy()
        parts = events[8].split()
        parts[3] = f'{parallel.single.RING_BASE + parallel.PER_CHAIN * 64:08x}'
        events[8] = ' '.join(parts)
        with self.assertRaisesRegex(ValueError, 'skipped or repeated'):
            self.check_events(events)

    def test_missing_iteration_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            self.check_events(self.events[:-8])

    def test_misaligned_pointer_is_rejected(self):
        events = self.events.copy()
        events[0] = f'2 AR 0 {parallel.single.RING_BASE+4:08x} 0 4 1'
        with self.assertRaisesRegex(ValueError, 'aligned'):
            self.check_events(events)


if __name__ == '__main__':
    unittest.main()
