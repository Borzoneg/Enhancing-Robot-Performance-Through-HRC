import logging
import os

class CustomLogger(logging.Logger):
    def __init__(self, name, level=logging.NOTSET): 
        
        super().__init__(name, level)
        self.filename = 'log_file.log'
        import os



        # Create console handler and set level to WARNING
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.WARNING)

        # Create rotating file handler and set level to DEBUG
        # log_file = 'log_file.log'
        # max_bytes = 1024 * 1024 * 10  # 10 MB
        # backup_count = 3
        # file_handler = RotatingFileHandler(filename=log_file, maxBytes=max_bytes, backupCount=backup_count)
        # file_handler.setLevel(logging.DEBUG)

        # Create file handler and set level to DEBUG
        if os.path.exists(self.filename):
            if os.path.getsize(self.filename) > 100e3: # the size is in B
                os.remove(self.filename)
        file_handler = logging.FileHandler('log_file.log')
        file_handler.setLevel(logging.DEBUG)

        # Create formatter and add it to the handlers
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        file_handler.setFormatter(formatter)

        # Add the handlers to the logger
        self.addHandler(console_handler)
        self.addHandler(file_handler)
        
        self.info("NEW RUN")