"""Utility helpers: rate recording, logging."""

import logging
import time


class RateRecorder:
    """Track and report loop iteration frequency."""

    def __init__(
        self,
        name: str | None = None,
        report_interval: float = 10,
        min_required_frequency: float | None = None,
    ):
        self.report_interval = report_interval
        self.last_report_time: float | None = None
        self.iteration_count = 0
        self.name = name
        self.min_required_frequency = min_required_frequency

    def __enter__(self):
        return self.start()

    def start(self) -> "RateRecorder":
        self.last_report_time = time.time()
        self.iteration_count = 0
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.last_report_time is not None:
            self._report_rate()

    def _report_rate(self) -> float:
        assert self.last_report_time is not None, "RateRecorder must be started before reporting."
        elapsed_time = time.time() - self.last_report_time
        rate = self.iteration_count / elapsed_time if elapsed_time > 0 else 0
        logging.info(f"{self.name} rate: {rate:.2f} Hz over {elapsed_time:.2f}s")
        return rate

    def track(self) -> None:
        """Call once per loop iteration to track and periodically report rate."""
        self.iteration_count += 1
        current_time = time.time()
        assert self.last_report_time is not None, "RateRecorder must be started before tracking."

        if current_time - self.last_report_time >= self.report_interval:
            interval_rate = self._report_rate()
            if self.min_required_frequency is not None and interval_rate < self.min_required_frequency:
                raise RuntimeError(
                    f"{self.name} frequency too low: {interval_rate:.2f} Hz "
                    f"(required: {self.min_required_frequency:.2f} Hz)"
                )
            self.last_report_time = current_time
            self.iteration_count = 0


def override_log_level(level: int = logging.INFO) -> None:
    """Reset root logger to a specific level."""
    logger = logging.getLogger()
    logger.setLevel(level)
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)
    logging.basicConfig(level=level)
