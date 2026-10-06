"""A deadline for instrument connects: a pyvisa-py open can hang inside libusb (see ../README.md)."""

import threading


class ConnectHung(Exception):
    """A connect attempt blew its deadline, or an earlier one is still stuck."""


class ConnectGuard:
    def __init__(self, deadline_s):
        self.deadline_s = float(deadline_s)
        self._lock = threading.Lock()
        self._hung = None          # the abandoned worker thread, while alive

    def hung(self):
        """True while an abandoned attempt is still stuck."""
        with self._lock:
            return self._hung is not None and self._hung.is_alive()

    def run(self, attempt, discard):
        """Return attempt()'s result, re-raise its exception, or raise ConnectHung past the deadline."""
        if self.hung():
            raise ConnectHung(
                "an earlier connect attempt is still stuck inside the USB stack; "
                "not starting another on the same device. Unplug and replug the "
                "instrument (or restart the container) to free it.")

        box = {}
        done = threading.Event()
        gate = threading.Lock()
        abandoned = [False]

        def work():
            try:
                box["result"] = attempt()
            except BaseException as exc:          # delivered to the caller below
                box["error"] = exc
            with gate:
                late = abandoned[0]
                done.set()
            if late and "result" in box:
                try:
                    discard(box["result"])
                except Exception:
                    pass

        worker = threading.Thread(target=work, daemon=True, name="instrument-connect")
        worker.start()
        if not done.wait(self.deadline_s):
            with gate:
                if not done.is_set():             # still running: give up on it
                    abandoned[0] = True
                    with self._lock:
                        self._hung = worker
                    raise ConnectHung(
                        f"connect did not return within {self.deadline_s:.0f} s -- "
                        "stuck inside the USB stack (on this Pi: pyvisa-py fighting "
                        "the kernel usbtmc driver for the device). Abandoned it.")
        if "error" in box:
            raise box["error"]
        return box["result"]
