"""Exercise clean-checkout install and legacy-patch restoration."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.home = Path(self.temp.name)
        self.klipper = self.home / "klipper"
        shutil.copytree(Path(os.environ["KLIPPER_PATH"]), self.klipper,
                        ignore=shutil.ignore_patterns(".git", "out", "__pycache__", "c_helper.so"))
        # The recovered local test checkout contains the legacy untracked
        # plugin file; official Klipper does not.
        (self.klipper / "klippy/kinematics/polar_center.py").unlink(missing_ok=True)
        subprocess.run(["git", "init", "-q", str(self.klipper)], check=True)
        subprocess.run(["git", "-C", str(self.klipper), "add", "."], check=True)
        subprocess.run(["git", "-C", str(self.klipper), "-c", "user.name=Test",
                        "-c", "user.email=test@example.invalid", "commit", "-qm", "baseline"], check=True)

    def run_install(self):
        return subprocess.run([sys.executable, str(ROOT / "install.py"), "--klipper",
                               str(self.klipper), "--sync", '--state-root',
                               str(self.home / '.local/share/klipper-polar-support')],
                              capture_output=True, text=True)

    def test_repeat_install_leaves_tracked_klipper_clean(self):
        for _ in range(2):
            result = self.run_install()
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        link = self.klipper / "klippy/kinematics/polar_center.py"
        self.assertTrue(link.is_symlink())
        self.assertEqual(link.resolve(), ROOT / "klippy/kinematics/polar_center.py")
        status = subprocess.check_output(["git", "-C", str(self.klipper), "status",
                                          "--porcelain"], text=True)
        self.assertEqual(status, "")

    def test_restores_legacy_tracked_changes(self):
        legacy = self.klipper / ".polar-center-upgrade"
        originals = legacy / "original"
        records = {}
        for name in ("klippy/chelper/kin_polar.c", "klippy/chelper/__init__.py"):
            target = self.klipper / name
            backup = originals / name
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target, backup)
            original = digest(target)
            target.write_text(target.read_text() + "\n/* legacy patch */\n")
            records[name] = {"original_sha256": original,
                             "installed_sha256": digest(target), "mode": 0o644}
        old_py = self.klipper / "klippy/kinematics/polar_center.py"
        old_py.write_text("# legacy")
        records["klippy/kinematics/polar_center.py"] = {
            "original_sha256": None, "installed_sha256": digest(old_py), "mode": 0o644}
        (legacy / "installed.json").write_text(json.dumps(records))
        result = self.run_install()
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertEqual(subprocess.check_output(
            ["git", "-C", str(self.klipper), "status", "--porcelain"], text=True), "")
        self.assertTrue(list((self.home / ".local/share/klipper-polar-support").iterdir()))

    def test_unknown_tracked_edit_fails_preflight_without_mutation(self):
        target = self.klipper / 'klippy/chelper/kin_polar.c'
        target.write_text(target.read_text() + '\n// independent user edit\n')
        before = target.read_bytes()
        result = subprocess.run([sys.executable, str(ROOT / 'install.py'),
                                 '--klipper', str(self.klipper), '--check'],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('independently modified', result.stderr)
        self.assertEqual(target.read_bytes(), before)
        self.assertFalse((self.klipper / 'klippy/kinematics/polar_center.py').exists())

    def test_staged_changes_are_preserved(self):
        target = self.klipper / 'klippy/chelper/kin_polar.c'
        target.write_text(target.read_text() + '\n// staged user edit\n')
        subprocess.run(['git', '-C', str(self.klipper), 'add', str(target)], check=True)
        result = self.run_install()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('staged Klipper changes', result.stderr)
        self.assertIn('staged user edit', target.read_text())

    def test_unrelated_untracked_file_remains_visible(self):
        target = self.klipper / 'user-notes.txt'
        target.write_text('keep this')
        result = self.run_install()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('user-notes.txt', result.stdout)
        self.assertEqual(target.read_text(), 'keep this')

    def test_tracked_plugin_is_not_hidden_or_removed(self):
        target = self.klipper / 'klippy/kinematics/polar_center.py'
        target.write_text('# user-tracked implementation')
        subprocess.run(['git', '-C', str(self.klipper), 'add', str(target)], check=True)
        result = self.run_install()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('is tracked', result.stderr)
        self.assertEqual(target.read_text(), '# user-tracked implementation')


if __name__ == "__main__":
    unittest.main()
