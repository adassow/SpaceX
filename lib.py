import pcf8574
from machine import Pin

class NeoPixelRange:
    def __init__(self, np, start, end, reverse=False):
        self.np = np
        self.n = end - start + 1
        self.start = start
        self.end = end
        self.reverse = reverse

    def __setitem__(self, index, val):
        if index <= self.end - self.start:
            if self.reverse:
                self.np[self.end - index - 1] = val
            else: 
                self.np[index + self.start] = val

    def __getitem__(self, index):
        if self.reverse:
            return self.np[self.end - index]    
        return self.np[index+self.start]

    def fill(self, color):
        for i in range(self.n):
            self[i] = color

    def write(self):
        self.np.write()

class PixelSet:
    def __init__(self, np):
        self.np = np
        self.color = [1,1,1]
        self.brightness = 255
        self.mode = 'inc' 
        self.off()
        self.current=-1
    def on(self):
        self.np.fill([self.brightness * x for x in self.color])
        self.np.write()
    def off(self):
        self.np.fill((0,0,0))
        self.np.write()
    def inc(self):
        self.np.fill((0,0,0))
        self.current += 1
        self.np[self.current] = [self.brightness * x for x in self.color]
        self.np.write()
    def dec(self):
        self.np.fill((0,0,0))
        self.current -= 1
        self.np[self.current] = [self.brightness * x for x in self.color]
        self.np.write()

class Switches:
    def __init__(self, interrupt, i2c, address, np, buzzer, colors = None):
        self.pcf = pcf8574.PCF8574(i2c, address)
        self.np = np
        self.pins = {}
        self.callbacks = {}
        self.colors = colors
        self.buzzer = buzzer
        self.callback(1)
        interrupt.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.callback)

    def register_callback(self, pin, callback):
        self.callbacks.setdefault(pin,[])
        self.callbacks[pin].append(callback)
    def callback(self, p):
        default_color = [255,255,255]
        for i in range(8):
            val = (self.pcf.port >> i) & 0x01
            if i in self.pins and self.pins[i] == val:
                continue
            self.pins[i] = val
            # print('Sw ', i, ' set to ', val)
            if self.colors is None or len(self.colors) <= i:
                color = default_color
            else:
                color = self.colors[i]
            self.np[i] = [val * x for x in color]
            self.buzzer.bip()
            for call in self.callbacks.get(i,[]):
                call(val)
        self.np.write()