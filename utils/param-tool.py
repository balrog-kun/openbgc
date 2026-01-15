#! /usr/bin/python3
# vim: set ts=4 sw=4 sts=4 et :
import sbgcserialapi.cmd as cmd
import sbgcserialapi.cmd_obgc as cmd_obgc
import sbgcserialapi.frame as fr

import param_defs
import param_utils

import math, time, sys, codecs, argparse
import serial, serial.tools.list_ports
import selectors
import construct

description = '''Get or set OpenBGC parameters over serial
Pass <value> to set a parameter, skip to read current value.'''

lst = 'Known parameters (<num-id>: <name>) are:\n'
for num in param_defs.params:
    pdef = param_defs.params[num]
    lst += f'{pdef.id}: {pdef.name}\n'

parser = argparse.ArgumentParser(add_help=False,
        formatter_class=argparse.RawTextHelpFormatter, description=description, epilog=lst)
parser.add_argument('param-id', help='Can be a <num-id> or <name>, see below')
parser.add_argument('value', nargs='?', help='Optional, must be a hexstring', default=None)
parser.add_argument('-h', '-l', '--help', action='help', help='Show help and list known <param-id>s')
parser.add_argument('-p', help='Serial port path, defaults to /dev/ttyUSB0, pass /dev/rfcommN for Bluetooth', default='/dev/ttyUSB0')
parser.add_argument('-v', '-d', action='store_true', help='Enable verbose mode')

if len(sys.argv) <= 1:
    parser.print_help()
    sys.exit(0)

args = parser.parse_args()
pid = getattr(args, 'param-id')
try:
    pdef = param_defs.params[int(pid, 0)]
except:
    by_name = {pdef.name: pdef for pdef in param_defs.params.values()}
    if pid not in by_name:
        print('Unknown param-id')
        sys.exit(-1)

    pdef = by_name[pid]

param_type_cls = param_utils.ctype_to_construct(pdef.typ, pdef.size)

if args.value is None:
    op_set = False
    out_payload = cmd_obgc.GetParamRequest.build(dict(param_id=pdef.id))
else:
    op_set = True
    # TODO: maybe parse as json, then pass to param_type_cls.build()?
    vbytes = codecs.decode(args.value, 'hex')
    if pdef.size != len(vbytes):
        print(f'{pdef.name} value must be {pdef.size} bytes, got {len(vbytes)}')
        sys.exit(-1)
    out_payload = cmd_obgc.SetParamRequest.build(dict(param_id=pdef.id, value=vbytes))

out_frame = fr.FrameV1.build(dict(hdr=dict(cmd_id=int(cmd_obgc.CmdId.CMD_OBGC), size=len(out_payload)), pld=out_payload))

sbgc_port_path = args.p
sbgc_port = serial.Serial(sbgc_port_path, baudrate=115200, timeout=0, write_timeout=1)

sbgc_port.write(out_frame)
running = True
exit_code = 0

def list_convert(val):
    if isinstance(val, list):
        return [list_convert(elem) for elem in val]
    return val

def frame_cb(in_frame):
    global running
    running = False

    if in_frame.hdr.cmd_id == cmd.CmdId.CMD_ERROR:
        in_payload = cmd.ErrorResponse.parse(in_frame.pld.data)
        print(repr(in_payload)) # TODO: ensure ERR_OPERATION_FAILED doesn't print as ERR_FS_WRONG_PARAMS
        exit_code = -1
        return

    if op_set:
        pld = cmd.ConfirmResponse.build(dict(cmd_id=int(cmd_obgc.CmdId.CMD_OBGC), data=bytes([int(cmd_obgc.SubcmdId.SET_PARAM)])))
        if in_frame.hdr.cmd_id != cmd.CmdId.CMD_CONFIRM or in_frame.pld.data != pld:
            print("Unexpected response", in_frame)
            exit_code = -1
            return

        val = list_convert(param_type_cls.parse(vbytes))
        print("Type:", pdef.typ)
        print("New value:", val)
    else:
        if in_frame.hdr.cmd_id != int(cmd_obgc.CmdId.CMD_OBGC):
            print("Unexpected response", in_frame)
            exit_code = -1
            return

        in_payload = cmd_obgc.GetParamResponse.parse(in_frame.pld.data)
        if pdef.size != len(in_payload):
            print("Received parameter value but size doesn't match", in_payload)
            exit_code = -1
            return

        ### TODO: undefined params requesting???
        val = list_convert(param_type_cls.parse(in_payload))
        print("Type:", pdef.typ)
        print("Value:", val)

def line_cb(line):
    print(f'{sbgc_port_path}: {line}')

def debug_cb(info):
    print('dbg:', info)

sbgc_stream = fr.InStream(line_cb=line_cb, frame_cb=frame_cb, debug_cb=debug_cb if args.v else None)

def sbgc_read_cb(port, mask):
    """Callback triggered when data is ready to be read."""
    if port.in_waiting <= 0:
        return

    sbgc_stream.feed(port.read(port.in_waiting))

def mainloop(timeout):
    sel = selectors.DefaultSelector()
    sel.register(sbgc_port, selectors.EVENT_READ, sbgc_read_cb)
    end_ts = time.monotonic() + timeout

    try:
        while running:
            left = end_ts - time.monotonic()
            if left <= 0:
                break

            events = sel.select(timeout=left)
            for key, mask in events:
                callback = key.data
                callback(key.fileobj, mask)
    except KeyboardInterrupt:
        print("\nExiting...\n")

    sel.unregister(sbgc_port)
    sel.close()

mainloop(1)
sbgc_port.close()

if running:
    print("Response timeout")
    exit_code = -1

sys.exit(exit_code)
