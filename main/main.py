from time import sleep_ms, ticks_ms
from machine import I2C, Pin, ADC, Timer
import neopixel
import utime
from main.lcd import esp32
import main.pcf8574
import main.lib
import math
from main.rotary_irq_esp import RotaryIRQ

class Buzzer:
    def __init__(self):
        self.buzzer = Pin(33, Pin.OUT)
    def bip(self):
        self.buzzer.on()
        sleep_ms(100)
        self.buzzer.off()

class Animation:
    def __init__(self, delay, timer, np, potentiometers, rotary, lcd):
        self.delay = delay
        self.timer = timer
        self.i = 0
        self.np = np
        self.potentiometers = potentiometers
        self.rotary = rotary
        self.rotary_value = 0
        self.lcd = lcd

    def start(self):
        self.timer.init(period=self.delay, mode=Timer.PERIODIC, callback=self.next_step)
    
    def next_step(self, t):
        self.np.fill((0,0,0))
        factor = 255/4095
        color = (factor,factor,factor)
        brightness = self.potentiometers[3].read()/4095
        foo = [int(brightness * self.potentiometers[i].read() * x) for i, x in enumerate(color)]
        self.np[self.i] = foo
        self.i += 1
        self.i = self.i % 12
        self.np.write()
        if  self.rotary.value() != self.rotary_value:
            self.lcd.clear()
            self.lcd.putstr("Speed {}".format(self.rotary.value()))
            self.rotary_value = self.rotary.value()
            self.timer.deinit()
            self.timer.init(period=int(math.pow(2,int(self.rotary_value/2))), mode=Timer.PERIODIC, callback=self.next_step)
        
    def stop(self):
        self.timer.deinit()

class SpaceX:
    def __init__(self):
        r = RotaryIRQ(pin_num_clk=16, 
                    pin_num_dt=17, 
                    min_val=10, 
                    max_val=20, 
                    reverse=False, 
                    range_mode=RotaryIRQ.RANGE_BOUNDED)

        potentiometers = [
            ADC(Pin(34)),
            ADC(Pin(35)),
            ADC(Pin(36)),
            ADC(Pin(39)),
        ]
        for potentiometer in potentiometers:
            potentiometer.atten(ADC.ATTN_11DB)

        buzzer = Buzzer()
        np = neopixel.NeoPixel(Pin(14),50)
        np[35] = [255,0,0]
        np.write()
        circle = main.lib.NeoPixelRange(np,22,33)
        square = main.lib.NeoPixelRange(np,0,12)
        line = main.lib.NeoPixelRange(np,12,21)

        i2c = I2C(scl=Pin(22), sda=Pin(21))
        lcd = esp32.I2cLcd(i2c, 0x3f, 2, 16)
        animation = Animation(100, Timer(1), circle, potentiometers, r, lcd)
        animation2 = Animation(100, Timer(2), square, potentiometers, r, lcd)
        animation3 = Animation(100, Timer(3), line, potentiometers, r, lcd)
        pin26 = Pin(26, Pin.IN, Pin.PULL_UP)
        pin27 = Pin(27, Pin.IN, Pin.PULL_UP)
        switches1 = main.lib.Switches(pin26, i2c, 0x20, main.lib.NeoPixelRange(np,42,50,True), buzzer, [
            [255,255,0],
            [255,255,0],
            [255,255,0],
            [255,0,0],
            [0,255,0],
            [255,0,0],
            [0,255,0],
            [255,0,0],
        ])
        switches2 = main.lib.Switches(pin27, i2c, 0x21,main.lib.NeoPixelRange(np,41,42,True), buzzer, [
            [0,255,0],
            [0,255,0],
        ])
        def callback_1(p):
            if p == 1:
                animation.start()
            else: 
                animation.stop()
        switches1.register_callback(0, callback_1)
        def callback_2(p):
            if p == 1:
                animation2.start()
            else: 
                animation2.stop()
        switches1.register_callback(1, callback_2)
        def callback_3(p):
            if p == 1:
                animation3.start()
            else: 
                animation3.stop()
        switches1.register_callback(2, callback_3)