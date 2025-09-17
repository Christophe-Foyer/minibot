import threading
import time
from .DRV8825 import DRV8825

class StepperMotor:
    def __init__(self, dir_pin, step_pin, enable_pin,
                 microstep_pins=None, step_delay=0.002, timeout=0.5):
        self.motor = DRV8825(dir_pin=dir_pin,
                             step_pin=step_pin,
                             enable_pin=enable_pin,
                             mode_pins=microstep_pins)
        self.motor.SetMicroStep('softward', 'fullstep')
        self.step_delay = step_delay
        self.timeout = timeout  # seconds until auto-stop

        self._speed = 0
        self._thread = None
        self._stop_flag = threading.Event()
        self._last_cmd_time = time.time()

    def run(self, speed):
        self._speed = speed
        self._last_cmd_time = time.time()
        if self._thread is None or not self._thread.is_alive():
            self._stop_flag.clear()
            self._thread = threading.Thread(target=self._worker, daemon=True)
            self._thread.start()

    def _worker(self):
        while not self._stop_flag.is_set():
            # Timeout check
            if time.time() - self._last_cmd_time > self.timeout:
                self._speed = 0

            if self._speed == 0:
                time.sleep(0.01)
                continue

            direction = 'forward' if self._speed > 0 else 'backward'
            delay = max(0.0005, self.step_delay / abs(self._speed))
            self.motor.TurnStep(dir=direction, steps=1, stepdelay=delay)

    def stop(self):
        self._speed = 0
        self._stop_flag.set()
        if self._thread:
            self._thread.join()
            self._thread = None
        self.motor.Stop()
