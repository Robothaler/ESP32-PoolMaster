"""
PlatformIO extra_script: upload SPIFFS filesystem only when needed.

Runs as a pre-action of the firmware "upload" target.  Triggers `uploadfs`
if any of the following is true:

  * the stamp file (.pio/<env>/spiffs_upload.stamp) is missing
    → first build / fresh checkout: we can't know what's on the device,
      so always upload to be safe;
  * any file in data/ is newer than the stamp;
  * partitions.csv is newer than the stamp (partition layout changed →
    SPIFFS must be re-flashed regardless of data/ content).

The stamp is updated only on a successful uploadfs run, so a failed
upload will be retried on the next attempt.
"""
import os
import glob

Import("env")  # noqa: F821 — PlatformIO injects this

PROJECT_DIR = env.subst("$PROJECT_DIR")
DATA_DIR    = os.path.join(PROJECT_DIR, "data")
PARTITIONS  = os.path.join(PROJECT_DIR, "partitions.csv")
STAMP_FILE  = os.path.join(PROJECT_DIR, ".pio",
                           env.subst("$PIOENV"), "spiffs_upload.stamp")


def _newest_mtime(folder):
    files = glob.glob(os.path.join(folder, "**", "*"), recursive=True)
    files = [f for f in files if os.path.isfile(f)]
    return max((os.path.getmtime(f) for f in files), default=0)


def _mtime(path):
    try:
        return os.path.getmtime(path)
    except OSError:
        return 0


def upload_fs_if_changed(source, target, env):  # noqa: ARG001
    if not os.path.isdir(DATA_DIR):
        print("[uploadfs] No data/ directory — skipping.")
        return

    stamp        = _mtime(STAMP_FILE)
    data_newest  = _newest_mtime(DATA_DIR)
    parts_mtime  = _mtime(PARTITIONS)

    if stamp == 0:
        reason = "first run / no stamp file"
    elif data_newest > stamp:
        reason = "data/ changed"
    elif parts_mtime > stamp:
        reason = "partitions.csv changed"
    else:
        print("[uploadfs] data/ and partitions.csv unchanged — "
              "skipping SPIFFS upload.")
        return

    print(f"[uploadfs] Triggering SPIFFS upload ({reason}) ...")
    rc = env.Execute("$PYTHONEXE -m platformio run "
                     f"--project-dir {PROJECT_DIR} "
                     f"-e {env.subst('$PIOENV')} "
                     "--target uploadfs")
    if rc == 0:
        os.makedirs(os.path.dirname(STAMP_FILE), exist_ok=True)
        with open(STAMP_FILE, "w") as f:
            f.write("ok\n")
        print("[uploadfs] SPIFFS upload done — stamp updated.")
    else:
        print(f"[uploadfs] SPIFFS upload FAILED (rc={rc}) — "
              "ABORTING firmware upload so the stale state is obvious.")
        env.Exit(1)


env.AddPreAction("upload", upload_fs_if_changed)
