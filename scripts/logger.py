import logging
import datetime

class Logger:
    COLOR_CODES = {
        'DEBUG': '\033[36m',
        'INFO': '\033[97m',
        'WARNING': '\033[93m',
        'ERROR': '\033[91m',
        'CRITICAL': '\033[41m\033[37m',
    }
    def __init__(self, name):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        handler.setFormatter(self.ColorFormatter())
        self.logger.addHandler(handler)

    class ColorFormatter(logging.Formatter):
        def format(self, record):
            color_code = Logger.COLOR_CODES.get(record.levelname, '')
            reset_code = '\033[0m'
            message = super().format(record)
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            return f'{color_code}{timestamp} {message}{reset_code}'

    def info(self, msg, *args, **kwargs):
        self.logger.info("INFO: " + msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self.logger.warning("WARNING: " + msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self.logger.error("ERROR: " + msg, *args, **kwargs)
