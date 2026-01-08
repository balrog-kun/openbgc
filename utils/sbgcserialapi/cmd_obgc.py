# vim: set ts=4 sw=4 sts=4 et :
from construct import *

from .cmd import *

#
# Non-standard command request and response payloads
#

CmdId = Enum(Int8ul,
    CMD_OBGC = 240,
)

SubcmdId = Enum(Int8ul,
    GET_PARAM = 0,
    SET_PARAM = 1,
    READ_CONFIG = 2,
    SAVE_CONFIG = 3,
)

GetParamRequest = Struct(
    "subcmd_id" / Const(int(SubcmdId.GET_PARAM), Int8ul),
    "param_id"  / Int16ul,
)

GetParamResponse = GreedyBytes

SetParamRequest = Struct(
    "subcmd_id" / Const(int(SubcmdId.SET_PARAM), Int8ul),
    "param_id"  / Int16ul,
    "value"     / GreedyBytes,
)
