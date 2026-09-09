"""Check update authorization and restart ordering without contacting a printer."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import auto_update
import apply_update


class UpdateTests(unittest.TestCase):
    def status(self, **changes):
        info = dict(is_valid=True, is_dirty=False,
                    remote_url='https://github.com/maevebaksa/Klipper-Polar-Support.git',
                    remote_hash='a' * 40, current_hash='b' * 40)
        info.update(changes)
        return dict(busy=False, version_info={'polar-support': info})

    def test_busy_printer_is_never_queried_for_update(self):
        with patch.object(auto_update, 'idle', return_value=False), \
                patch.object(auto_update, 'call') as call:
            auto_update.main()
            call.assert_not_called()

    def test_invalid_dirty_unknown_and_unchanged_candidates_are_skipped(self):
        for changes in [dict(is_valid=False), dict(is_dirty=True),
                        dict(remote_url='https://github.com/other/repo.git'),
                        dict(remote_hash='?'), dict(current_hash='a' * 40)]:
            with self.subTest(changes=changes), \
                    patch.object(auto_update, 'idle', return_value=True), \
                    patch.object(auto_update, 'call', return_value=self.status(**changes)) as call:
                auto_update.main()
                self.assertEqual(call.call_count, 1)

    def test_only_polar_plugin_is_updated(self):
        with patch.object(auto_update, 'idle', return_value=True), \
                patch.object(auto_update, 'call', return_value=self.status()) as call:
            auto_update.main()
            self.assertEqual(call.call_args.args,
                             ('/machine/update/client', {'name': 'polar-support'}))

    def test_becoming_busy_cancels_update(self):
        with patch.object(auto_update, 'idle', side_effect=[True, False]), \
                patch.object(auto_update, 'call', return_value=self.status()) as call:
            auto_update.main()
            self.assertEqual(call.call_count, 1)

    def test_hot_heater_is_not_idle(self):
        replies = [{'klippy_state': 'ready'}, {'status': {
            'print_stats': {'state': 'complete'}, 'idle_timeout': {'state': 'Idle'},
            'heaters': {'available_heaters': ['extruder']}}},
            {'status': {'extruder': {'target': 200}}}]
        with patch.object(auto_update, 'call', side_effect=replies):
            self.assertFalse(auto_update.idle())

    def test_managed_service_never_stops_busy_klipper(self):
        with patch.object(sys, 'argv', ['apply_update.py', '--user', 'printer', '--klipper', '/test']), \
                patch.object(apply_update, 'idle', return_value=False), \
                patch.object(apply_update.subprocess, 'run') as run:
            with self.assertRaises(SystemExit):
                apply_update.main()
            self.assertEqual(run.call_count, 1)
            self.assertEqual(run.call_args.args[0][-1], '--check')

    def test_managed_service_stops_before_relinking(self):
        with patch.object(sys, 'argv', ['apply_update.py', '--user', 'printer', '--klipper', '/test']), \
                patch.object(apply_update, 'idle', return_value=True), \
                patch.object(apply_update.subprocess, 'run') as run:
            apply_update.main()
            calls = [c.args[0] for c in run.call_args_list]
            self.assertEqual(calls[1], ['systemctl', 'stop', 'klipper.service'])
            self.assertEqual(calls[2][-1], '--sync')
            self.assertEqual(calls[3], ['systemctl', 'start', 'klipper.service'])
