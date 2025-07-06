"""
IVCP MicroPython Library
Provides Tx and Rx classes for the IVCP single-wire protocol using RP2040 PIO.

The Integrated Value Communication Protocol was fully designed by me, Franklín, and vibe coded
by our almighty AI-overlords.

API:
    Tx(data_pin, t_value=8000)
        transmit(value, vid)   # start/update continuous streaming (background thread)
        blip(value, vid)       # send one frame immediately
        stop()                 # stop continuous streaming

    Rx(data_pin, t_value=8000)
        listen()     # start background decode thread
        grab()       # return (value, vid) of last valid frame
        status()     # return status code of last decode
        stop()       # stop background thread

Status codes for Rx:
    VID_FAULT = 0   # last frame had VID=0
    OK        = 1   # last frame valid
    CRC_FAIL  = 2   # CRC-8 check failed
    RZ_FAIL   = 3   # framing (RZ) check failed
    NO_MSG    = 4   # no frame received yet

Continuous TX uses a busy-loop for true microsecond timing and cleanly exits on stop().
"""

import array, utime, _thread
from machine import Pin
from rp2 import asm_pio, PIO, StateMachine

# --- PIO Programs ---
@asm_pio(out_init=(PIO.OUT_LOW,), autopull=True, pull_thresh=32, out_shiftdir=PIO.SHIFT_LEFT)
def _ivcp_pio_tx():
    out(pins, 1)

@asm_pio(in_shiftdir=PIO.SHIFT_LEFT, autopush=True, push_thresh=32)
def _ivcp_pio_rx():
    in_(pins, 1)

# --- CRC-8 Helper ---
def _crc8_poly7(payload, bitlen=28):
    crc = 0
    for i in range(bitlen-1, -1, -1):
        b = (payload >> i) & 1
        crc ^= b << 7
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

# --- Transmitter Class ---
class Tx:
    """IVCP transmitter using PIO. Continuous via transmit(), single via blip()."""

    def __init__(self, data_pin, t_value=12500):
        self.pin     = Pin(data_pin, Pin.OUT)
        self.t_value = t_value               # desired frame interval in µs
        total_ticks = 82
        pio_freq = int((1_000_000 / t_value) * total_ticks)
        self.sm = StateMachine(0, _ivcp_pio_tx, freq=pio_freq, out_base=self.pin)
        self.sm.active(1)
        self._running = False
        self._words   = None

    def _build_frame_words(self, value, vid):
        sign = 1 if value >= 0 else 0
        absV = abs(value)
        best_err, best = float('inf'), (0,0,0)
        for es in (0,1):
            for E in range(8):
                cand = round(absV * 10**E) if es == 0 else round(absV / 10**E)
                if 0 <= cand <= 0x7FFFF:
                    recon = cand / 10**E if es == 0 else cand * 10**E
                    err = abs(recon - absV)
                    if err < best_err:
                        best_err, best = err, (cand, E, es)
        cand, exp, es = best
        payload = (vid<<24) | (sign<<23) | (cand<<4) | (es<<3) | exp
        crc = _crc8_poly7(payload)
        msg = (0b11<<38) | (payload<<10) | (crc<<2)
        bitstr = f"{msg:040b}"
        sched = [1]*4 + [0]*2
        for b in bitstr[2:]: sched += [1,0] if b=='1' else [0,0]
        words = []
        for i in range(0, len(sched), 32):
            chunk = sched[i:i+32]
            w = 0
            for bit in chunk: w = (w<<1) | bit
            w <<= (32 - len(chunk))
            words.append(w)
        return array.array('I', words)

    def _stream(self):
        period_us = self.t_value
        try:
            while self._running:
                start = utime.ticks_us()
                self.sm.put(self._words)
                ms = period_us // 1000
                if ms > 1:
                    utime.sleep_ms(ms - 1)
                while utime.ticks_diff(utime.ticks_us(), start) < period_us:
                    utime.sleep_ms(0)
        finally:
            self._running = False

    def transmit(self, value, vid):
        self._words = self._build_frame_words(value, vid)
        if not self._running:
            self._running = True
            _thread.start_new_thread(self._stream, ())

    def blip(self, value, vid):
        words = self._build_frame_words(value, vid)
        self.sm.put(words)

    def stop(self):
        """Stop any ongoing streaming cleanly."""
        self._running = False

# --- Receiver Class ---
class Rx:
    """IVCP receiver using PIO sampler + background decode."""
    VID_FAULT = 0
    OK        = 1
    CRC_FAIL  = 2
    RZ_FAIL   = 3
    NO_MSG    = 4

    def __init__(self, data_pin, t_value=12500):
        self.pin = Pin(data_pin, Pin.IN)
        total_ticks = 82
        pio_freq = int((1_000_000 / t_value) * total_ticks)
        self.sm = StateMachine(1, _ivcp_pio_rx, freq=pio_freq, in_base=self.pin)
        self.sm.active(1)
        self._buffer = []
        self._running = False
        self._last_frame = (0.0, 0)
        self._last_status = Rx.NO_MSG

    def _run(self):
        while self._running:
            while self.sm.rx_fifo():
                w = self.sm.get()
                for i in range(32): self._buffer.append((w>>(31-i))&1)
                if len(self._buffer)>164: self._buffer=self._buffer[-164:]
            if len(self._buffer)>=82:
                handled=False
                for i in range(len(self._buffer)-81):
                    win=self._buffer[i:i+82]
                    if win[:4]==[1,1,1,1] and win[4:6]==[0,0]:
                        bits = '11' + ''.join(
    '1' if (win[j] == 1 and win[j+1] == 0) else '0'
    for j in range(6, 82, 2)
)
                        if len(bits)!=40:
                            self._last_status=Rx.RZ_FAIL
                        else:
                            mi=int(bits,2)
                            payload=(mi>>10)&((1<<28)-1)
                            crc_rcv=(mi>>2)&0xFF
                            crc_calc=_crc8_poly7(payload)
                            if crc_calc!=crc_rcv:
                                self._last_status=Rx.CRC_FAIL
                            else:
                                vid=(payload>>24)&0xF
                                if vid==0:
                                    self._last_status=Rx.VID_FAULT
                                else:
                                    sign=(payload>>23)&1
                                    cand=(payload>>4)&((1<<19)-1)
                                    es=(payload>>3)&1
                                    exp=payload&0x7
                                    val=0.0 if cand==0 else((cand*10**exp) if es else(cand/10**exp))
                                    if sign==0:val=-val
                                    self._last_frame=(val,vid)
                                    self._last_status=Rx.OK
                        self._buffer=self._buffer[i+82:]
                        handled=True
                        break
                if not handled: self._buffer.pop(0)
            utime.sleep_ms(1)

    def listen(self):
        if not self._running:
            self._running=True
            _thread.start_new_thread(self._run,())

    def grab(self): return self._last_frame
    def status(self):return self._last_status
    def stop(self): self._running=False
