import pygame
from bisect import bisect_left
from PIL import Image
from bresenham import bresenham
import numpy as np
import delegator
import math
from threading import Thread
from subprocess import Popen, PIPE
from pyo import *
from time import time
from collections import namedtuple


class WacomValues:

    def __init__(self):
        "docstring"
        self.tilt_x = 0
        self.tilt_y = 0
        self.pressure = 0
        self.thread = Thread(target=lambda : self.update_wacom_stuff())
        self.thread.start()

    def update_wacom_stuff(self):
        libin = Popen(["unbuffer", "libinput", "debug-events"], stdout=PIPE)
        while True:
            l = libin.stdout.readline().decode("utf-8")
            # tilt 3
            # pressure 4
            try:
                if "TABLET_TOOL" in l:
                    self.tilt_x, self.tilt_y = [min(max((float(num.strip())+60)/120, -1.0), 1.0) for num in l.split("\t")[3].split("tilt:")[1].replace("*","").split("/")]
                    if "pressure" in l:
                        self.pressure = float(l.split("\t")[4].split("pressure:")[1].split(" ")[1].replace("*",""))
            except:
                pass

class Brush:
    def preview(self, points):
        pyo_sine_pan.setPan(points[0].pan)
        for point in points:
            faderidx = int((1-point.y)*current_surface.get_height()*preview_quality_factor)
            if faderidx < len(pyo_sine_faders):
                fader = pyo_sine_faders[faderidx]
                fader.mul = fader_loudness*(float(point.amp))
                fader.play()

class PixelBrush(Brush):
    def draw(self, x, y):
        x = x/current_surface.get_width()
        y = y/current_surface.get_height()
        points = [Point(y, 1, x)]
        current_spec_unit.modify_or_insert_line(points, x)

class HarmonicBrush(Brush):
    def draw(self, x, y):
        x = x/current_surface.get_width()
        y = y/current_surface.get_height()
        points = [Point(y, 1, x)]
        ratio = 2
        bandwidth = math.log(max_freq/min_freq,2)
        next_y = (bandwidth*y - math.log(ratio,2)) / bandwidth
        while next_y > 0 and ratio < 12:
            points.append(Point(next_y, 1/(ratio/1.5), x))
            ratio+=1
            next_y = (bandwidth*y - math.log(ratio,2)) / bandwidth

        current_spec_unit.modify_or_insert_line(points, x)

class WSampleLoopBrush(Brush):
    def __init__(self):

        loadt = Thread(target=lambda:self.load())
        loadt.start()

    def load(self):
        self.loaded = False
        bpo = (current_surface.get_height()-1)/math.log(max_freq/min_freq,2)
        self.sndTable = SndTable('sample.wav')
        delegator.run("arss sample.wav sample.bmp --min-freq {} --max-freq {} --pps 150 --bpo {}".format(min_freq, max_freq, bpo))
        self.sample = np.asarray(Image.open("sample.bmp"))
        self.idx = 0
        print("loaded")
        self.loaded = True

    def preview(self, offset):
        if pyo_sample_reader.table != self.sndTable:
            pyo_sample_reader.setTable(self.sndTable)
            pyo_sample_reader.setFreq(self.sndTable.getRate())

        # number of octaves
        bandwidth = math.log(max_freq/min_freq,2)
        # number of semitones in the bandwidth
        note_num = bandwidth*12
        transpo = offset * note_num
        pyo_sample_transpo.setTranspo(transpo)
        pyo_sample_fader.mul = wacom.pressure
        pyo_sample_reader.setPhase(self.idx/len(self.sample[0]))
        pyo_sample_pan.setPan(wacom.tilt_x)
        pyo_sample_fader.play()

    def draw(self, x, y):
        if not self.loaded:
            return
        x = x/current_surface.get_width()
        offset = y/current_surface.get_height() - 0.5
        increment = 1/current_surface.get_height()
        points = []
        for i in range(len(self.sample)):
            pointy = i * increment + offset
            if wacom.pressure * (self.sample[i][self.idx][0]) <=3:
                continue
            if pointy >= 1 or pointy < 0:
                continue
            points.append(Point(pointy, wacom.pressure * (self.sample[i][self.idx][0])/255, wacom.tilt_x))
        # advance one point in the self.sample and loop
        self.preview((1 - y/current_surface.get_height())-0.5)
        current_spec_unit.modify_or_insert_line(points, x)
        self.idx+=1
        self.idx%=len(self.sample[0])


class WHarmonicBrush(Brush):
    def draw(self, x, y):
        x = x/current_surface.get_width()
        y = y/current_surface.get_height()
        points = [Point(y, wacom.pressure, wacom.tilt_x)]
        ratio = 2
        # frequency bandwidth in number of octaves
        bandwidth = math.log(max_freq/min_freq,2)
        next_y = (bandwidth*y - math.log(ratio,2)) / bandwidth
        while next_y > 0 and ratio < 12:
            multip = 1
            if ratio %2 == 0:
                multip = wacom.tilt_y * 2
            else:
                multip = (1 - wacom.tilt_y) * 2
            amp = wacom.pressure/max(((ratio/1.5) * multip),1)
            points.append(Point(next_y, amp, wacom.tilt_x))
            ratio+=1
            next_y = (bandwidth*y - math.log(ratio,2)) / bandwidth
        self.preview(points)
        current_spec_unit.modify_or_insert_line(points, x)

class WSineBrush(Brush):
    def draw(self, x, y):
        x = x/current_surface.get_width()
        y = y/current_surface.get_height()
        points = [Point(y, wacom.pressure, wacom.tilt_x)]
        self.preview(points)
        current_spec_unit.modify_or_insert_line(points, x)


class WNoiseBrush(Brush):
    def draw(self, x, y):
        x = x/current_surface.get_width()
        y = y/current_surface.get_height()
        increment = 1/current_surface.get_height()
        points = []
        h = int(wacom.tilt_y*(current_surface.get_height()/3))
        highest = 1 - max(y - (h*increment),0)
        lowest = 1 - min(y + (h*increment),1)
        for i in range(-h,h+1):
            points.append(Point(min(max(y+i*increment, 0), 1 - increment), wacom.pressure, wacom.tilt_x))
        self.preview(highest, lowest, wacom.pressure, wacom.tilt_x)
        current_spec_unit.modify_or_insert_line(points, x)

    def preview(self, highest, lowest, amp, pan):
        # number of octaves
        bandwidth = math.log(max_freq/min_freq,2)
        # number of midi note in the bandwidth
        note_num = bandwidth*12
        # the target midi note is the normalized location in y + the base midi note offset

        highest * bandwidth
        pyo_noise_hp.setFreq(midi_to_freq((note_num * lowest) + freq_to_midi(min_freq)))
        pyo_noise_lp.setFreq(midi_to_freq((note_num * highest) + freq_to_midi(min_freq)))
        pyo_noise_pan.setPan(pan)
        pyo_noise_fader.mul = amp
        pyo_noise_fader.play()



class NoiseBrush(Brush):
    def draw(self, x, y):
        x = x/current_surface.get_width()
        y = y/current_surface.get_height()
        increment = 1/current_surface.get_height()
        points = []
        for i in range(-10,11):
            points.append(Point(y+i*increment, 1, x))
        current_spec_unit.modify_or_insert_line(points, x)

class Point:
    def __init__(self, y, amp, pan):
        self.y = y
        self.amp = amp
        self.pan = pan
        self.dirty = True
        self.gesture_id = current_gesture

    def render(self, x_value):
        if not self.dirty:
            return
        self.dirty = False
        y_value = round(self.y * current_surface.get_height())
        l_amp, r_amp = [self.amp * math.sin(self.pan*math.pi/2), self.amp * math.cos(self.pan*math.pi/2)]
        pixel_color = [l_amp*255,0 , r_amp*255]

        canvas_l[y_value][x_value] += pixel_color[2]
        canvas_l[y_value][x_value] = min(canvas_l[y_value][x_value], 255)
        canvas_r[y_value][x_value] += pixel_color[0]
        canvas_r[y_value][x_value] = min(canvas_r[y_value][x_value], 255)
        try:
            current_surface.fill([canvas_r[y_value][x_value],0,canvas_l[y_value][x_value]], ((x_value,y_value),(1,1)))
        except:
            pass

class Line:
    def __init__(self, x, width):
        self.x = x
        self.width = 1
        self.points = []
        self.points_y = []
        self.dirty = True
        self.gesture_id = current_gesture

    def render(self):
        if not self.dirty:
            return
        self.dirty = False

        x_value = round(self.x * current_surface.get_width())

        for point in self.points:
            point.render(x_value)


    def add_point(self, point):
        self.dirty = True
        idx = bisect_left(self.points_y, point.y)
        adjacent_point = self.points[idx] if idx < len(self.points) else None
        if adjacent_point and adjacent_point.y == point.y:
            adjacent_point.pan = point.pan
            adjacent_point.amp = point.amp
            adjacent_point.dirty = True
        else:
            self.points.insert(idx, point)
            self.points_y.insert(idx, point.y)

    def add_points(self, points):
        for point in points:
            self.add_point(point)

class SpectrumUnit:
    def __init__(self):
        self.lines = []
        # keeping this alows us to not build this list every time we need to bissect
        self.lines_x = []

    def modify_or_insert_line(self, points, x):
        line = None
        # right now, can only add one line at a time
        idx = bisect_left(self.lines_x, x)
        adjacent_line = self.lines[idx] if idx < len(self.lines) else None
        if adjacent_line and adjacent_line.x == x:
            line = adjacent_line
        else:
            line = Line(x, 1/current_surface.get_width())
            self.lines.insert(idx, line)
            self.lines_x.insert(idx, x)
        line.add_points(points)

    def invalidate_canvas(self):
        global canvas_l
        global canvas_r
        canvas_l = [[0 for i in range(w)] for i in range(h)]
        canvas_r = [[0 for i in range(w)] for i in range(h)]
        current_surface.fill([0,0,0])
        for line in self.lines:
            line.dirty = True
            for point in line.points:
                point.dirty = True

    def remove_last_gesture(self):
        gesture_to_remove = gesture_stack.pop()
        for line in self.lines:
            line.points = [point for point in line.points if point.gesture_id != gesture_to_remove]
        self.invalidate_canvas()

h = 1000
w = 1800

max_freq = 20000
min_freq = 27.5

canvas_l = [[0 for i in range(w)] for i in range(h)]
canvas_r = [[0 for i in range(w)] for i in range(h)]

gesture_stack = []
current_spec_unit = SpectrumUnit()
current_surface = None
current_brush = None
wacom = WacomValues()
current_gesture = 0
pyo_server = Server(audio="jack").boot()
pyo_server.start()
def midi_to_freq(note_num):
    return (440 / 32) * (2 ** ((note_num - 9) / 12))

def freq_to_midi(freq):
    return math.log(freq/440.0,2) * 12 + 69


preview_quality_factor = 0.3
bpo = ((preview_quality_factor*h)-1)/math.log(max_freq/min_freq,2)
midi_notes_increment = 12/bpo
fader_loudness = 0.1

pyo_sine_faders = [Fader(fadeout=0.03, dur=0.04, mul=fader_loudness) for i in range(int(h*preview_quality_factor))]
pyo_sines = FastSine(mul=pyo_sine_faders,freq=[midi_to_freq(freq_to_midi(min_freq)+i*midi_notes_increment) for i in range(int(h*preview_quality_factor))])
pyo_sine_pan = Pan(pyo_sines).out()

pyo_noise_fader = Fader(fadeout=0.03, dur=0.04, mul=1)
pyo_noise = Noise(mul=pyo_noise_fader)
pyo_noise_hp = Biquadx(pyo_noise, q=1, type=1)
pyo_noise_lp = Biquadx(pyo_noise_hp, q=1)
pyo_noise_pan = Pan(pyo_noise_lp).out()

pyo_sample_fader = Fader(fadeout=0.03, dur=0.04, mul=1)
pyo_sample_reader = Osc(HarmTable([1]), mul=pyo_sample_fader)
pyo_sample_transpo = Harmonizer(pyo_sample_reader, transpo=0, winsize=0.03)
pyo_sample_pan = Pan(pyo_sample_transpo).out()

last_grid_len = 33
grid_len = 33

def main():

    global last_grid_len
    global grid_len
    pressing = False

    pygame.init()
    pygame.display.set_caption("dessin")
    global current_gesture
    screen = pygame.display.set_mode((w,h))
    global current_surface
    current_surface = screen
    brushes = [WSineBrush(), WHarmonicBrush(), WNoiseBrush(), WSampleLoopBrush()]
    current_brush = brushes[0]
    last_mouse_pos = None
    running = True
    framenum=0
    ctrl= False
    playing_start = False
    playing = False
    last_x = 0
    grid_changed = True
    while running:
        for line in current_spec_unit.lines:
            line.render()

        if grid_changed:
            for i in range(int(current_surface.get_width()/last_grid_len)):
                current_surface.fill([0,40,0,0.1], ((i * last_grid_len,0),(1,current_surface.get_height())), pygame.BLEND_RGBA_SUB)

            for i in range(int(current_surface.get_width()/grid_len)):
                current_surface.fill([0,40,0,0.1], ((i * grid_len,0),(1,current_surface.get_height())), pygame.BLEND_RGBA_ADD)

            last_grid_len = grid_len
            grid_changed = False

        if playing:
            num_seconds = current_surface.get_width()/150
            x = int(((playing-playing_start)/num_seconds) * current_surface.get_width())
            if x >= current_surface.get_width():
                playing = False
                playing_start = False
                last_x=0
                current_surface.fill([0,100,0,0.3], ((x - (x-last_x),0),(x-last_x,current_surface.get_height())), pygame.BLEND_RGBA_SUB)
                continue
            if x-last_x >=1:
                current_surface.fill([0,100,0,0.3], ((x - (x-last_x),0),(x-last_x,current_surface.get_height())), pygame.BLEND_RGBA_ADD)
            playing = time()
            last_x = x

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONUP:
                pressing = False
                gesture_stack.append(current_gesture)
                current_gesture += 1
                last_mouse_pos = None
                grid_changed = True
            if event.type == pygame.MOUSEBUTTONDOWN:
                pressing = True
                last_mouse_pos = [-1,-1]
            if event.type == pygame.MOUSEMOTION and pressing:

                if last_mouse_pos != [-1,-1]:
                    points = bresenham(last_mouse_pos[0], last_mouse_pos[1], event.pos[0], event.pos[1])
                else:
                    points = [[event.pos[0], event.pos[1]]]
                for p in list(points)[0:-1]:
                    current_brush.draw(p[0], p[1])
                last_mouse_pos = event.pos
            if event.type == pygame.KEYDOWN:
                num = -1
                try:
                    num = int(event.unicode)
                except:
                    pass
                if num > -1:
                    current_brush = brushes[num-1]
                elif event.key == 122 and pygame.key.get_mods() & pygame.KMOD_CTRL:
                    current_spec_unit.remove_last_gesture()

                elif event.unicode == "+":
                    last_grid_len = grid_len
                    grid_len += 1
                    grid_changed = True

                elif event.unicode == "-":
                    last_grid_len = grid_len
                    grid_len -= 1
                    grid_changed = True


                elif event.unicode == " ":
                    if playing:
                        continue
                    il = Image.fromarray(np.asarray(canvas_l, np.dtype('uint8')))
                    il.convert("RGB").save("out_l.bmp")
                    ir = Image.fromarray(np.asarray(canvas_r, np.dtype('uint8')))
                    ir.convert("RGB").save("out_r.bmp")
                    delegator.run("arss out_l.bmp o_l.wav --sine --min-freq 27.5 --max-freq 20000 -r 44000 -f 16 --pps 150")
                    delegator.run("arss out_r.bmp o_r.wav --sine --min-freq 27.5 --max-freq 20000 -r 44000 -f 16 --pps 150")
                    delegator.run("sox o_l.wav o_r.wav --channels 2 --combine merge o.wav")
                    delegator.run("play o.wav", block=False)
                    playing_start = time()
                    playing = time()
        pygame.display.update()



if __name__=="__main__":
    main()
