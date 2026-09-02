import logging

class ColorFormatter(logging.Formatter):
    COLORS = {
        logging.DEBUG: "\x1b[36m",    # cyan
        logging.INFO: "\x1b[32m",     # green
        logging.WARNING: "\x1b[33m",  # yellow
        logging.ERROR: "\x1b[31m",    # red
        logging.CRITICAL: "\x1b[35m", # magenta
    }
    RESET = "\x1b[0m"

    def format(self, record: logging.LogRecord) -> str:
        color = self.COLORS.get(record.levelno, "")
        level = record.levelname
        if color:
            record.levelname = f"{color}{level}{self.RESET}"
        try:
            return super().format(record)
        finally:
            record.levelname = level

def create_logger(name=__name__, level=logging.INFO) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(level)
    formatter = ColorFormatter("%(asctime)s  [%(levelname)s] %(message)s")
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    return logger
