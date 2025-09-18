import time
from machine import Pin, Timer
from config import ENCODER_A1 ,ENCODER_B1, ENCODER_A2, ENCODER_B2, PULSES_PER_REV,DEGREES_PER_COUNT, encoder_mode

#depended on encoders/motors
#might need to add something with time

encoder_pin_a1 = Pin(ENCODER_A1, Pin.IN, Pin.PULL_UP)
encoder_pin_b1 = Pin(ENCODER_B1, Pin.IN, Pin.PULL_UP)
encoder_pin_a2 = Pin(ENCODER_A2, Pin.IN, Pin.PULL_UP)
encoder_pin_b2 = Pin(ENCODER_B2, Pin.IN, Pin.PULL_UP)

class Encoder:
    def __init__(self, pin_a, pin_b, mode="2x"):
        
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.mode = mode
        self.count = 0

        if mode == "1x":
            self.pin_a.irq(trigger=Pin.IRQ_RISING, handler=self._isr_1x)
        elif mode == "2x":
            self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                        handler=self._isr_2x)
        elif mode == "4x":
            self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                        handler=self._isr_4x)
            self.pin_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                        handler=self._isr_4x)

    def _isr_1x(self, pin):
        if self.pin_b.value() == 0:
            self.count += 1
        else:
            self.count -= 1

    def _isr_2x(self, pin):
        
        if self.pin_a.value() == self.pin_b.value():
            self.count += 1
        else:
            self.count -= 1

    def _isr_4x(self, pin):
        
        a_state = self.pin_a.value()
        b_state = self.pin_b.value()

        if a_state == b_state:
            if pin is self.pin_a:
                self.count += 1
            else:
                self.count -= 1
        else:
            if pin is self.pin_a:
                self.count -= 1
            else:
                self.count += 1

    def read(self):
        return self.count

    def reset(self):
        self.count = 0

encoder_right = Encoder(encoder_pin_a1,encoder_pin_b1,mode=encoder_mode)
encoder_left = Encoder(encoder_pin_a2,encoder_pin_b2,mode=encoder_mode)