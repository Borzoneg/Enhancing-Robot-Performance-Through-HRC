import logging
import os

class CustomLogger(logging.Logger):
    def __init__(self, name, filename, level=logging.NOTSET, overwrite=False): 
        
        super().__init__(name, level)
        self.filename = filename
        
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.WARNING)

        # Create file handler and set level to DEBUG
        if os.path.exists(self.filename):
            if os.path.getsize(self.filename) > 100e3: # the size is in B
                os.remove(self.filename)
        mode = 'a' if not overwrite else 'w'
        file_handler = logging.FileHandler(self.filename, mode=mode, encoding='utf-8')
        file_handler.setLevel(logging.DEBUG)

        # Create formatter and add it to the handlers
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        file_handler.setFormatter(formatter)

        # Add the handlers to the logger
        self.addHandler(console_handler)
        self.addHandler(file_handler)
        
        self.info("NEW RUN")