# vim: set ts=4 sw=4 sts=4 et :
from construct import *
from . import cmd
import traceback

def checksum8(data: bytes) -> int:
    return sum(data) & 0xff

def checksum16(data: bytes) -> int:
    crc = 0
    def add_byte(byte):
        nonlocal crc
        for _ in range(8):
            # The difference here from the usual implementation is that
            # input is processed from LSB to MSB so we get better results
            # by doing the whole calculation with crc bits inverted
            if crc >> 15 != byte & 1:
                crc = (crc << 1) ^ 0x8005
            else:
                crc <<= 1
            byte >>= 1
            crc &= 0xffff

    for byte in data:
        add_byte(byte)

    return crc

class FlatRawCopy(RawCopy):
    def _build(self, obj, stream, context, path):
        # Accept inner value directly
        return super()._build(Container(value=obj), stream, context, path)

    def _parse(self, stream, context, path):
        result = super()._parse(stream, context, path)
        # Flatten: merge value fields into the container
        if isinstance(result.value, bytes):
            flat = Container()
        else:
            flat = Container(result.value)
        flat.data = result.data
        flat.offset1 = result.offset1
        flat.offset2 = result.offset2
        return flat

Payload = GreedyBytes # Replace with cmd.RequestPayload or cmd.ResponsePayload

FrameHeader = Struct(
    "cmd_id"  / cmd.CmdId,
    "size"    / Int8ul,
    "hdrcrc"  / Checksum(
        Int8ul,
        checksum8,
        lambda ctx: bytes([int(ctx.cmd_id), ctx.size])
    ),
)

FrameV1 = Struct(
    "start"   / Const(b'>'),
    "hdr"     / FrameHeader,
    "pld"     / FlatRawCopy(FixedSized(this.hdr.size, Payload)),
    "pldcrc"  / Checksum(Int8ul, checksum8, this.pld.data),
)

FrameV2 = Struct(
    "start"   / Const(b'$'),
    "hdr"     / FlatRawCopy(FrameHeader),
    "pld"     / FlatRawCopy(FixedSized(this.hdr.size, Payload)),
    "pldcrc"  / Checksum(Int16ul, checksum16, this.hdr.data + this.pld.data),
)

Frame = Select(FrameV1, FrameV2)

class InStream:
    def __init__(self, text_cb=None, line_cb=None, frame_cb=None, debug_cb=None):
        self.text_cb = text_cb
        self.line_cb = line_cb
        self.frame_cb = frame_cb
        self.debug_cb = debug_cb
        self.line = b''
        self.buf = b''

    def feed(self, data):
        self.buf += data
        text = b''

        while len(self.buf):
            c = self.buf[0]

            if c in b'>$':
                if len(self.buf) < 5:
                    break

                crclen = 1 if c == ord('>') else 2
                payloadlen = self.buf[2]
                framelen = 4 + payloadlen + crclen

                if len(self.buf) < framelen:
                    break

                if text:
                    if self.text_cb is not None:
                        self.text_cb(text.decode('utf-8'))
                    text = ''

                cmdstr = str(cmd.CmdId.parse(self.buf[1:2]))
                frame = None
                if self.debug_cb is not None:
                    self.debug_cb(f'Trying parse {str(self.buf[:framelen])} as cmd {cmdstr}')
                try:
                    frame = Frame.parse(self.buf[:framelen])
                except Exception as e:
                    if self.debug_cb is not None:
                        self.debug_cb(f'Parser raised "{e}":\n{traceback.format_exc()}')
                if frame and self.frame_cb is not None:
                    try:
                        self.frame_cb(frame)
                    except Exception as e:
                        print(e)
                        if self.debug_cb is not None:
                            self.debug_cb(f'frame_cb raised "{e}":\n{traceback.format_exc()}')

                self.buf = self.buf[framelen:]
                continue

            cbytes = self.buf[:1]
            self.buf = self.buf[1:]
            text += cbytes

            if c in b'\r\n':
                if self.line and self.line_cb is not None:
                    self.line_cb(self.line.decode('utf-8'))
                self.line = b''
            else:
                self.line += cbytes

        if text:
            if self.text_cb is not None:
                self.text_cb(text.decode('utf-8'))
