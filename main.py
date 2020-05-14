from time import sleep_ms, ticks_ms
from machine import I2C, Pin, ADC, Timer
from lcd import esp32
import neopixel
import utime
import pcf8574
import lib
import math
from rotary_irq_esp import RotaryIRQ

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
        self.rotary_value = self.rotary.value()
        self.lcd = lcd

    def start(self):
        self.timer.init(period=self.delay, mode=Timer.PERIODIC, callback=self.next_step)
    
    def next_step(self, t):
        self.np.fill((0,0,0))
        factor = 255/4095
        color = (factor,factor,factor)
        brightness = self.potentiometers[3].read()/4095
        # for i, potentiometer in enumerate(potentiometers):
        #     print("Pot {} val {} brightness {}".format(i, potentiometer.read(), brightness))
        foo = [int(brightness * self.potentiometers[i].read() * x) for i, x in enumerate(color)]
        self.np[self.i] = foo
        self.i += 1
        self.i = self.i % 12
        self.np.write()
        if  self.rotary.value() != self.rotary_value:
            print("val {}".format(self.rotary.value()))
            lcd.clear()
            lcd.putstr("Speed {}".format(self.rotary.value()))
            self.rotary_value = self.rotary.value()
            self.timer.deinit()
            self.timer.init(period=int(math.pow(2,int(self.rotary_value/2))), mode=Timer.PERIODIC, callback=self.next_step)
        #     self.timer.deinit()
        #self.timer = Timer(-1)
        # print("tick {}".format(self.i))
        

    def stop(self):
        self.timer.deinit()

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
circle = lib.NeoPixelRange(np,22,33)
square = lib.NeoPixelRange(np,0,12)
line = lib.NeoPixelRange(np,12,21)





# bar = PixelSet(np,0,10)
# def callback(p):
#     np[0]=[p.value()*255,0,0]
#     buzzer.value(not p.value())
#     np.write()
#     print(p.value())
i2c = I2C(scl=Pin(22), sda=Pin(21))
lcd = esp32.I2cLcd(i2c, 0x3f, 2, 16)
animation = Animation(100, Timer(1), circle, potentiometers, r, lcd)
animation2 = Animation(100, Timer(2), square, potentiometers, r, lcd)
animation3 = Animation(100, Timer(3), line, potentiometers, r, lcd)
pin26 = Pin(26, Pin.IN, Pin.PULL_UP)
pin27 = Pin(27, Pin.IN, Pin.PULL_UP)
switches1 = lib.Switches(pin26, i2c, 0x20, lib.NeoPixelRange(np,42,50,True), buzzer, [
    [255,255,0],
    [255,255,0],
    [255,255,0],
    [255,0,0],
    [0,255,0],
    [255,0,0],
    [0,255,0],
    [255,0,0],
])
switches2 = lib.Switches(pin27, i2c, 0x21,lib.NeoPixelRange(np,41,42,True), buzzer, [
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
# pcf1 = PCF8574(i2c, 0x20)
# pcf2 = PCF8574(i2c, 0x21)

# def callback_pcf1(p):
#     color = [
#         [255,255,0],
#         [255,255,0],
#         [255,255,0],
#         [255,0,0],
#         [0,255,0],
#         [255,0,0],
#         [0,255,0],
#         [255,0,0],
#     ]
#     for i in range(8):
#         val = (pcf1.port >> i) & 0x01
#         np[49 - i] = [val * x for x in color[i]]
#     np.write()

# def callback_pcf2(p):
#     color = [255,255,255]
#     for i in range(2):
#         val = (pcf2.port >> i) & 0x01
#         np[41 - i] = [val * x for x in color]
#     np.write()

# pin = Pin(27, Pin.IN, Pin.PULL_UP)
# pin.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=callback_pcf2)

# pin = Pin(26, Pin.IN, Pin.PULL_UP)
# pin.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=callback_pcf1)


    

# while True:
# for i in range(50):
#     np[i]=[255,255,255]
#     np.write()
    #np[i]=[0,0,0]
    #utime.sleep_ms(500)



# The PCF8574 has a jumper selectable address: 0x20 - 0x27
DEFAULT_I2C_ADDR = 0x3f

def test_main():
    """Test function for verifying basic functionality."""
    print("Running test_main")
    i2c = I2C(scl=Pin(22), sda=Pin(21), freq=400000)
    lcd = esp32.I2cLcd(i2c, DEFAULT_I2C_ADDR, 2, 16)
    lcd.putstr("It Works!\nSecond Line")
    sleep_ms(3000)
    lcd.clear()
    count = 0
    while True:
        lcd.move_to(0, 0)
        lcd.putstr("%7d" % (ticks_ms() // 1000))
        sleep_ms(1000)
        count += 1
        if count % 10 == 3:
            print("Turning backlight off")
            lcd.backlight_off()
        if count % 10 == 4:
            print("Turning backlight on")
            lcd.backlight_on()
        if count % 10 == 5:
            print("Turning display off")
            lcd.display_off()
        if count % 10 == 6:
            print("Turning display on")
            lcd.display_on()
        if count % 10 == 7:
            print("Turning display & backlight off")
            lcd.backlight_off()
            lcd.display_off()
        if count % 10 == 8:
            print("Turning display & backlight on")
            lcd.backlight_on()
            lcd.display_on()

def demo(np):
    n = np.n

    # cycle
    for i in range(4 * n):
        for j in range(n):
            np[j] = (0, 0, 0)
        np[i % n] = (255, 255, 255)
        np.write()
        sleep_ms(25)

    # bounce
    for i in range(4 * n):
        for j in range(n):
            np[j] = (0, 0, 128)
        if (i // n) % 2 == 0:
            np[i % n] = (0, 0, 0)
        else:
            np[n - 1 - (i % n)] = (0, 0, 0)
        np.write()
        sleep_ms(60)

    # fade in/out
    for i in range(0, 4 * 256, 8):
        for j in range(n):
            if (i // 256) % 2 == 0:
                val = i & 0xff
            else:
                val = 255 - (i & 0xff)
            np[j] = (val, 0, 0)
        np.write()

    # clear
    for i in range(n):
        np[i] = (0, 0, 0)
    np.write()
