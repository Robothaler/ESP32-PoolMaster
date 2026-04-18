"""
PlatformIO extra_script: upload SPIFFS filesystem only when data/ files changed.

Runs as a pre-upload action. Compares the newest mtime in data/ against a
stamp file (.pio/spiffs_upload.stamp). If data is newer, uploadfs is triggered
before the normal firmware upload and the stamp is updated.
"""
import os
import glob
Import("env")  # noqa: F821 — PlatformIO injects this

DATA_DIR   = os.path.join(env.subst("$PROJECT_DIR"), "data")
STAMP_FILE = os.path.join(env.subst("$PROJECT_DIR"), ".pio",
                          env.subst("$PIOENV"), "spiffs_upload.stamp")

def _newest_mtime(folder):
    files = glob.glob(os.path.join(folder, "**", "*"), recursive=True)
    files = [f for f in files if os.path.isfile(f)]
    return max((os.path.getmtime(f) for f in files), default=0)

def _stamp_mtime():
    try:
        return os.path.getmtime(STAMP_FILE)
    except OSError:
        return 0

def upload_fs_if_changed(source, target, env):  # noqa: ARG001
    if not os.path.isdir(DATA_DIR):
        print("[uploadfs] No data/ directory — skipping.")
        return

    newest = _newest_mtime(DATA_DIR)
    stamp  = _stamp_mtime()

    if newest <= stamp:
        print("[uploadfs] data/ unchanged — skipping SPIFFS upload.")
        return

    print("[uploadfs] data/ changed — uploading SPIFFS filesystem...")
    rc = env.Execute("$PYTHONEXE -m platformio run "
                     f"--project-dir {env.subst('$PROJECT_DIR')} "
                     f"-e {env.subst('$PIOENV')} "
                     "--target uploadfs")
    if rc == 0:
        os.makedirs(os.path.dirname(STAMP_FILE), exist_ok=True)
        with open(STAMP_FILE, "w") as f:
            f.write("ok\n")
        print("[uploadfs] SPIFFS upload done — stamp updated.")
    else:
        print("[uploadfs] SPIFFS upload FAILED (rc={rc}) — firmware upload continues.")

env.AddPreAction("upload", upload_fs_if_changed)
