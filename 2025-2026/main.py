import time
from config.config import button, switch




while True:
    if button.value() == 0:
        switch = True
        break
    else:
        pass
    time.sleep(0.1)
