# Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from struct import *
from .transport import Transport
import sys
import os
import select

failure = -1
success = 0

PROTO_PSER_TLV_T_EPNAME = b'\x01'
PROTO_PSER_TLV_T_DATA   = b'\x02'

# Open `/dev/esps0` in read-write mode
flags = os.O_RDWR

# All users can read and write
mode = 0o666

# Timeout for select
timeout = 20

if sys.version_info >= (3, 0):
    def get_val(string):
        return bytes([string])
else:
    def get_val(string):
        return str(string)

class Transport_pserial(Transport):
    def __init__(self, devname):
        self.f = os.open(devname, mode, flags)

    def get_len(self, ep_name, in_buf):
        if get_val(in_buf[0]) == PROTO_PSER_TLV_T_EPNAME:
            if in_buf[1:3] == bytearray(pack('<H',len(ep_name))):
                length = 3 + len(ep_name)
                if in_buf[3:length].decode('ASCII') == ep_name :
                    if get_val(in_buf[length]) == PROTO_PSER_TLV_T_DATA:
                        length = length + 1
                        length = in_buf[length:length+2]
                        length = unpack('<H',length)
                        return success, length[0]
                    else:
                        print("Data type not matched")
                else:
                    print("Endpoint name not matched")
            else:
                print("Endpoint length not matched")
        else:
            print("Endpoint type not matched")
        return failure, -1

    def read_data(self, ep_name):
#
# Read fixed length of received data in below format:
# ----------------------------------------------------------------------------
# Endpoint Type | Endpoint Length | Endpoint Value  | Data Type | Data Length
#  ----------------------------------------------------------------------------
#
#  Bytes used per field as follows:
#  ---------------------------------------------------------------------------
#      1         |       2         | Endpoint Length |     1     |     2     |
#  ---------------------------------------------------------------------------
#
        read_len = 1 + 2 + len(ep_name) + 1 + 2
        self.data = os.read(self.f, read_len)
# Read variable length of received data. Variable length is obtained after
# parsing of previously read data.
        data_len = self.get_len(ep_name, self.data)
        if data_len[0] != success:
            return failure
        self.data = os.read(self.f, data_len[1])
        return success

    def send_data(self, ep_name, data):
        buf = bytearray(PROTO_PSER_TLV_T_EPNAME)
        buf.extend(pack('<H', len(ep_name)))
        buf.extend(map(ord,ep_name))
        buf.extend(PROTO_PSER_TLV_T_DATA)
        buf.extend(pack('<H', len(data)))
        buf.extend(bytearray(data))
        s = os.write(self.f, buf)
        reads,_,_ = select.select([self.f], [], [], timeout)
        if reads:
            s = self.read_data(ep_name)
            if s != success:
                return failure, ""
            return success, self.data
        else:
            print("Failed to read data within "+str(timeout)+" seconds")
        return failure, ""
