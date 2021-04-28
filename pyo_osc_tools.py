from pyo import *
from collections import deque
import os
import shutil
import json

_l = locals

class _OSCNode:
    def __init__(self, idle_timer=0.01, ramp=0.01, sig_size=3, idle_ramp=1, address="/"):
        self._child_nodes = {}
        self.address = address
        self.sig = SigTo([0]*sig_size, time=ramp)
        self.idle = True
        self.idle_timer = idle_timer
        self.ramp = ramp
        def set_to_zero():
            if self.idle:
                self.setValue([-400]*sig_size, reset_idle=False, ramp=idle_ramp)
            elif not self.idle:
                self.idle = True
        if idle_timer:
            self.idle_pattern = Pattern(set_to_zero, idle_timer)
            self.idle_pattern.play()

    def setValue(self, vals, reset_idle=True, ramp=False):
        if ramp:
            self.sig.time = ramp
        else:
            self.sig.time = self.ramp
        self.sig.value = vals
        if reset_idle:
            self.idle = False

    def _get_eff(self, key):
        if key not in self._child_nodes:
            self._child_nodes[key] = _OSCNode(self.idle_timer, self.ramp, address=key)
        return self._child_nodes[key]

    def __getitem__(self, key):
        key = str(key)
        return self._get_eff(key)

class OSCToSig:
    """Converts any osc messages sent to that port and osc address to a tree of pyo sigs"""
    def __init__(self, port=13001, idle_timer=0.1, ramp=0.01, osc_address_pattern="/*", sig_size=1):
        self._get_eff = self.__getitem__
        self._root_node = _OSCNode(idle_timer, ramp)
        # values of y between these ranges will go from 0 to 1
        def receive_msg(address, *args):
            addresses = address.split("/")[1:]
            current_node = self._root_node
            for a in addresses:
                # navigate to the targeted node
                current_node = current_node._get_eff(a)
            # set the values of all the sigs
            current_node.setValue(list(args))

        self.osc_rx = OscDataReceive(port, osc_address_pattern, receive_msg)
    def to_dict(self):
        """returns a dict representation of the current values of everything
        this will not work properly with osc streams that assigns values to non leaf addresses.
        """
        val_obj = {}
        def build_val_obj(current_node, obj_ptr):
            if len(current_node._child_nodes.keys()) == 0:
                obj_ptr[current_node.address] = current_node.sig.value
            else:
                obj_ptr[current_node.address] = {}
                a = {}
                for ck in current_node._child_nodes:
                    build_val_obj(current_node._child_nodes[ck], obj_ptr[current_node.address])
        build_val_obj(self._root_node, val_obj)
        return val_obj

    def __getitem__(self, key):
        key = str(key)
        return self._root_node[key]

class OSCRecord:
    """Records a stream of OSC event as json files"""
    def __init__(self, foldername, framerate=24):
        self.o = OSCToSig(idle_timer=False, sig_size=3)
        try:
            os.mkdir(foldername)
        except:
            shutil.rmtree(foldername)
            os.mkdir(foldername)
        def dump_frame():
            with open("{}/frame{}.json".format(foldername, self.framenum),"w") as f:
                json.dump(self.o.to_dict(), f)
            self.framenum += 1

        self.framenum = 0
        self.pattern = Pattern(dump_frame, 1/framerate)

    def start(self):
        self.pattern.play()

class OSCRecordReader:
    """reads a recording made by OSCRecord"""
    def __init__(self, foldername, buffer_size, loop_playback=True):
        self.current_index = 0
        self.foldername = foldername
        self.loop_playback = loop_playback
        self.current_data = None
        self.framebuffer = deque()
        self.nbfiles = len(next(os.walk(foldername))[2])
        self.buffer_size = buffer_size
        for i in range(buffer_size - 1):
            self._pre_load_next()
        self.next()

    def get(address):
        """address : an osc address string ex: "/whatever/osc/address" This will always get something interesting. It's going to be 0 if it doesn't exist but will never crash for a keyerror. This is to deal with the fact that every frame might not have the same data adresses"""
        parts = address.split("/")[1:]
        current_node = self.current_data.get("/",{})
        for part in parts:
            current_node = self.current_data.get(part, {})
        return 0 if current_node == {} else current_node

    def _pre_load_next(self):
        try :
            with open("{}/frame{}.json".format(self.foldername, self.current_index)) as f:
                self.framebuffer.append(json.load(f))
        except:
            pass
        self.current_index += 1
        if self.loop_playback:
            self.current_index %= self.nbfiles

    def next(self):
        """loads the next frame in memory"""
        try:
            self.current_data = self.framebuffer.popleft()
        except:
            return "ended"
        self._pre_load_next()

class OSCRecordSigReader:
    """Reads a recording made by OSCRecord as a sig tree with the same API as the OSCTosig class.
    Reads at a steady 24 fps by default but supports external frame advance triggers."""
    def __init__(self, foldername, framerate=24, trigsource=None, buffersize=None):
        buffersize = framerate if buffersize is None else buffersize
        # if this is constructed with an external trigsource, bypass the start method
        self._needs_play = trigsource is None
        self.trigsource = Metro(1/framerate) if trigsource is None else trigsource
        self._root_node = _OSCNode(idle_timer=False, ramp=1/framerate)
        self._reader = OSCRecordReader(foldername, buffersize)
        def set_tree():
            node = self._root_node
            current_branch = self._reader.current_data
            def set_node(branch, node):
                if type(branch) in [int, list, float] :
                    node.setValue(branch)
                else:
                    for k in branch:
                        set_node(branch[k], node[k])
            set_node(current_branch, node)
            if self._reader.next() == "ended":
                self.end_trig.play()

        self.tf = TrigFunc (self.trigsource, set_tree)
        self.end_trig = Trig()

    def start(self):
        if self._needs_play:
            self.trigsource.play()

    def __getitem__(self, key):
        key = str(key)
        return self._root_node["/"][key]

if __name__ == "__main__":
    s = Server().boot()
    o = OSCToSig(idle_timer=0.01, sig_size=3, ramp=0.001)
    h1fs = o["hand"][1]["finger"]
    s.start()
    sls = Pan([
        SineLoop(
            freq=MToF(Scale(h1fs[i]["pos"].sig[0], -400.4, 1144.4, 0, 100)),
            mul=DBToA(Scale(h1fs[i]["pos"].sig[1], 0, 473, -76, -13)),
            feedback=Scale(h1fs[i]["pos"].sig[2], -178, 473, 0, 0.6)
        )
        for i in range(5)
    ])
    sls.out()

    s.gui(_l)
