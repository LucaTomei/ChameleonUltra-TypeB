#!/usr/bin/env python3
"""
Flasher Nordic Secure DFU su USB-CDC per macOS.

Perché esiste: su macOS `nrfutil nrf5sdk-tools dfu usb-serial` fallisce con
`TypeError: cannot convert 'NoneType' object to bytearray` in __get_mtu, e
`nrfutil device program --traits nordicDfu` non agisce perché il CU in app mode
usa un VID non-Nordic (0x6868) e non espone il DFU trigger interface standard.
Causa reale: il bootloader (CDC 0x1915:0x521f) risponde solo con **DTR asserito**;
il tool apre la porta con DTR basso e non riceve risposta.

Questo script apre la porta con dtr=True e parla il Secure DFU seriale (SLIP).
Uso: python3 dfu_flash_macos.py <pacchetto-dfu.zip>
Il device va messo in DFU prima (enter_dfu.py) oppure ci pensa lo script.
"""
import sys, time, zlib, struct, zipfile, json
import serial
import serial.tools.list_ports as lp

DFUCMD = b'\x11\xef\x03\xf2\x00\x00\x00\x00\x0b\x00'  # comando app->DFU (vedi enter_dfu.py)

def find(vid, pid):
    return next((p.device for p in lp.comports() if p.vid == vid and p.pid == pid), None)

def enter_dfu():
    if find(0x1915, 0x521f):
        return
    port = find(0x6868, 0x8686)
    if port:
        try:
            s = serial.Serial(port=port, baudrate=115200); s.dtr = 1; s.timeout = 0
            s.write(DFUCMD); s.close()
        except Exception as e:
            print(f"enter_dfu warn: {e}")
    t0 = time.time()
    while time.time() - t0 < 10:
        if find(0x1915, 0x521f):
            return
        time.sleep(0.2)

def slip_encode(data):
    out = bytearray()
    for c in data:
        if c == 0xC0: out += b'\xDB\xDC'
        elif c == 0xDB: out += b'\xDB\xDD'
        else: out.append(c)
    out.append(0xC0)
    return bytes(out)

class Dfu:
    def __init__(self, port):
        s = serial.Serial(); s.port = port; s.baudrate = 115200
        s.dtr = True; s.rts = False          # <-- DTR True: indispensabile su macOS
        s.open(); time.sleep(0.1)
        try: s.reset_input_buffer()
        except Exception: pass
        self.s = s
        self.mtu = None

    def _read_frame(self, timeout=5.0):
        self.s.timeout = 0.05
        buf = bytearray(); esc = False; t0 = time.time()
        while time.time() - t0 < timeout:
            ch = self.s.read(1)
            if not ch: continue
            c = ch[0]
            if c == 0xC0:
                if buf: return bytes(buf)
                continue
            if esc: buf.append(0xC0 if c == 0xDC else 0xDB); esc = False
            elif c == 0xDB: esc = True
            else: buf.append(c)
        raise TimeoutError("nessuna risposta dal bootloader")

    def _request(self, opcode, payload=b'', expect_resp=True, timeout=5.0):
        self.s.write(slip_encode(bytes([opcode]) + payload))
        if not expect_resp:
            return None
        r = self._read_frame(timeout)
        if len(r) < 3 or r[0] != 0x60 or r[1] != opcode:
            raise RuntimeError(f"risposta inattesa op={opcode}: {r.hex()}")
        if r[2] != 0x01:
            raise RuntimeError(f"errore DFU op={opcode} result={r[2]} ({r.hex()})")
        return r[3:]

    def get_mtu(self):
        d = self._request(0x07)
        self.mtu = struct.unpack('<H', d[:2])[0]
        return self.mtu

    def set_prn(self, prn=0):
        self._request(0x02, struct.pack('<H', prn))

    def select(self, obj_type):
        d = self._request(0x06, bytes([obj_type]))
        max_size, offset, crc = struct.unpack('<III', d[:12])
        return max_size, offset, crc

    def create(self, obj_type, size):
        self._request(0x01, bytes([obj_type]) + struct.pack('<I', size))

    def write_data(self, data):
        # payload per pacchetto limitato dall'MTU (caso peggiore: ogni byte escapato)
        chunk = max(1, (self.mtu - 1) // 2 - 1)
        for i in range(0, len(data), chunk):
            self.s.write(slip_encode(bytes([0x08]) + data[i:i+chunk]))

    def calc_crc(self):
        d = self._request(0x03)
        offset, crc = struct.unpack('<II', d[:8])
        return offset, crc

    def execute(self):
        self._request(0x04)

    def transfer_object(self, obj_type, data, running_crc, running_off):
        """Trasferisce `data` come uno o più oggetti di dimensione <= max_size."""
        max_size, offset, crc = self.select(obj_type)
        step = max_size
        for i in range(0, len(data), step):
            part = data[i:i+step]
            self.create(obj_type, len(part))
            self.write_data(part)
            running_crc = zlib.crc32(part, running_crc) & 0xFFFFFFFF
            running_off += len(part)
            off, dev_crc = self.calc_crc()
            if off != running_off or dev_crc != running_crc:
                raise RuntimeError(f"CRC mismatch: host off={running_off} crc={running_crc:08x} / dev off={off} crc={dev_crc:08x}")
            self.execute()
        return running_crc, running_off

    def close(self):
        try: self.s.close()
        except Exception: pass


def main():
    if len(sys.argv) < 2:
        print("uso: dfu_flash_macos.py <pacchetto-dfu.zip>"); sys.exit(2)
    pkg = sys.argv[1]
    with zipfile.ZipFile(pkg) as z:
        manifest = json.loads(z.read('manifest.json'))
        app = manifest['manifest']['application']
        dat = z.read(app['dat_file'])
        bin_ = z.read(app['bin_file'])
    print(f"init packet={len(dat)}B  firmware={len(bin_)}B")

    enter_dfu()
    port = find(0x1915, 0x521f)
    if not port:
        print("ERRORE: device non in DFU (0x1915:0x521f)"); sys.exit(1)
    print("porta DFU:", port)

    dfu = Dfu(port)
    try:
        mtu = dfu.get_mtu(); print("MTU:", mtu)
        dfu.set_prn(0)
        # 1) init packet -> command object (type 0x01)
        crc, off = dfu.transfer_object(0x01, dat, 0, 0)
        print("init packet OK")
        # 2) firmware -> data object (type 0x02), CRC cumulativo
        crc, off = dfu.transfer_object(0x02, bin_, 0, 0)
        print(f"firmware OK ({off}B, crc {crc:08x})")
        print("FLASH COMPLETATO")
    finally:
        dfu.close()

if __name__ == '__main__':
    main()
