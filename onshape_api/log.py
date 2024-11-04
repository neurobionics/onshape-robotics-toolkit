"""
Logging module for opensourceleg library.

Module Overview:

This module defines a custom logger class, `Logger`, designed to log attributes
from class instances to a CSV file. It extends the `logging.Logger` class.

Key Classes:

- `LogLevel`: Enum class that defines the log levels supported by the `Logger` class.
- `Logger`: Logs attributes of class instances to a CSV file. It supports
setting different logging levels for file and stream handlers.
- `LOGGER`: Global instance of the `Logger` class that can be used throughout

Usage Guide:

1. Create an instance of the `Logger` class.
2. Optionally, set the logging levels for file and stream handlers using
   `set_file_level` and `set_stream_level` methods.
3. Add class instances and attributes to log using the `track_variable` method.
4. Start logging data using the `update` method.
5. PLEASE call the `close` method before exiting the program to ensure all data is written to the log file.

"""

import csv
import logging
import os
from collections import deque
from datetime import datetime
from enum import Enum
from logging.handlers import RotatingFileHandler
from typing import Any, Callable, Optional

__all__ = ["LOGGER", "LOG_LEVEL", "Logger"]


class LogLevel(Enum):
    DEBUG = logging.DEBUG
    INFO = logging.INFO
    WARNING = logging.WARNING
    ERROR = logging.ERROR
    CRITICAL = logging.CRITICAL


class Logger(logging.Logger):
    _instance = None

    def __new__(cls, *args: Any, **kwargs: Any) -> "Logger":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(
        self,
        log_path: str = "./",
        log_format: str = "[%(asctime)s] %(levelname)s: %(message)s",
        file_level: LogLevel = LogLevel.DEBUG,
        stream_level: LogLevel = LogLevel.INFO,
        file_max_bytes: int = 0,
        file_backup_count: int = 5,
        file_name: Optional[str] = None,
        buffer_size: int = 1000,
    ) -> None:
        if not hasattr(self, "_initialized"):
            super().__init__(__name__)
            self._log_path = log_path
            self._log_format = log_format
            self._file_level = file_level
            self._stream_level = stream_level
            self._file_max_bytes = file_max_bytes
            self._file_backup_count = file_backup_count
            self._user_file_name = file_name

            self._file_path: str = ""
            self._csv_path: str = ""
            self._file: Optional[Any] = None
            self._writer: Optional[csv.writer] = None
            self._is_logging = False
            self._header_written = False

            self._tracked_vars: dict[int, Callable[[], Any]] = {}
            self._var_names: dict[int, str] = {}
            self._buffer: deque[list[str]] = deque(maxlen=buffer_size)
            self._buffer_size: int = buffer_size

            self._setup_logging()
            self._initialized: bool = True
        else:
            self.set_file_name(file_name)
            self.set_file_level(file_level)
            self.set_stream_level(stream_level)
            self.set_format(log_format)
            self._file_max_bytes = file_max_bytes
            self._file_backup_count = file_backup_count
            self.set_buffer_size(buffer_size)

            self._log_path = log_path

    def _setup_logging(self) -> None:
        self.setLevel(level=self._file_level.value)
        self._std_formatter = logging.Formatter(self._log_format)

        self._stream_handler = logging.StreamHandler()
        self._stream_handler.setLevel(level=self._stream_level.value)
        self._stream_handler.setFormatter(fmt=self._std_formatter)
        self.addHandler(hdlr=self._stream_handler)

    def _setup_file_handler(self) -> None:
        if self._file_path == "":
            self._generate_file_paths()

        self._file_handler = RotatingFileHandler(
            filename=self._file_path,
            mode="w",
            maxBytes=self._file_max_bytes,
            backupCount=self._file_backup_count,
        )
        self._file_handler.setLevel(level=self._file_level.value)
        self._file_handler.setFormatter(fmt=self._std_formatter)
        self.addHandler(hdlr=self._file_handler)

    def _ensure_file_handler(self) -> None:
        if not hasattr(self, "_file_handler"):
            self._setup_file_handler()

    def track_variable(self, var_func: Callable[[], Any], name: str) -> None:
        var_id = id(var_func)
        self._tracked_vars[var_id] = var_func
        self._var_names[var_id] = name

    def untrack_variable(self, var_func: Callable[[], Any]) -> None:
        var_id = id(var_func)
        self._tracked_vars.pop(var_id, None)
        self._var_names.pop(var_id, None)

    def __repr__(self) -> str:
        return f"Logger(file_path={self._file_path})"

    def set_file_name(self, file_name: Optional[str]) -> None:
        self._user_file_name = file_name
        self._file_path = ""
        self._csv_path = ""

    def set_file_level(self, level: LogLevel) -> None:
        self._file_level = level
        if hasattr(self, "_file_handler"):
            self._file_handler.setLevel(level=level.value)

    def set_stream_level(self, level: LogLevel) -> None:
        self._stream_level = level
        self._stream_handler.setLevel(level=level.value)

    def set_format(self, log_format: str) -> None:
        self._log_format = log_format
        self._std_formatter = logging.Formatter(log_format)
        if hasattr(self, "_file_handler"):
            self._file_handler.setFormatter(fmt=self._std_formatter)
        self._stream_handler.setFormatter(fmt=self._std_formatter)

    def set_buffer_size(self, buffer_size: int) -> None:
        self._buffer_size = buffer_size
        self._buffer = deque(self._buffer, maxlen=buffer_size)

    def update(self) -> None:
        if not self._tracked_vars:
            return

        data = []
        for _var_id, get_value in self._tracked_vars.items():
            value = get_value()
            data.append(str(value))

        self._buffer.append(data)

        if len(self._buffer) >= self._buffer_size:
            self.flush_buffer()

    def flush_buffer(self) -> None:
        if not self._buffer:
            return

        self._ensure_file_handler()

        if self._file is None:
            self._file = open(self._csv_path, "w", newline="")
            self._writer = csv.writer(self._file)

        if not self._header_written:
            self._write_header()

        self._writer.writerows(self._buffer)
        self._buffer.clear()
        self._file.flush()

    def _write_header(self) -> None:
        header = list(self._var_names.values())

        self._writer.writerow(header)  # type: ignore[assignment]
        self._header_written = True

    def _generate_file_paths(self) -> None:
        now = datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S")
        script_name = os.path.basename(__file__).split(".")[0]

        base_name = self._user_file_name if self._user_file_name else f"{script_name}_{timestamp}"

        file_path = os.path.join(self._log_path, base_name)
        self._file_path = file_path + ".log"
        self._csv_path = file_path + ".csv"

    def __enter__(self) -> "Logger":
        return self

    def __exit__(
        self, exc_type: Optional[type[BaseException]], exc_val: Optional[BaseException], exc_tb: Optional[Any]
    ) -> None:
        self.flush_buffer()
        self.close()

    def reset(self) -> None:
        self._buffer.clear()
        self._tracked_vars.clear()
        self._var_names.clear()
        self._header_written = False
        if hasattr(self, "_file_handler"):
            self._file_handler.close()
            del self._file_handler

    def close(self) -> None:
        if self._file:
            self._file.close()
            self._file = None
            self._writer = None

    def debug(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._ensure_file_handler()
        super().debug(msg, *args, **kwargs)

    def info(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._ensure_file_handler()
        super().info(msg, *args, **kwargs)

    def warning(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._ensure_file_handler()
        super().warning(msg, *args, **kwargs)

    def error(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._ensure_file_handler()
        super().error(msg, *args, **kwargs)

    def critical(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self._ensure_file_handler()
        super().critical(msg, *args, **kwargs)

    def log(self, level: int, msg: str, *args: Any, **kwargs: Any) -> None:
        self._ensure_file_handler()
        super().log(level, msg, *args, **kwargs)

    @property
    def file_path(self) -> str:
        if self._file_path == "":
            self._generate_file_paths()
        return self._file_path

    @property
    def buffer_size(self) -> int:
        return self._buffer_size

    @property
    def file_level(self) -> LogLevel:
        return self._file_level

    @property
    def stream_level(self) -> LogLevel:
        return self._stream_level

    @property
    def file_max_bytes(self) -> int:
        return self._file_max_bytes

    @property
    def file_backup_count(self) -> int:
        return self._file_backup_count


# Initialize a global logger instance to be used throughout the library
LOGGER = Logger()
LOG_LEVEL = dict(enumerate(LogLevel.__members__.values()))
