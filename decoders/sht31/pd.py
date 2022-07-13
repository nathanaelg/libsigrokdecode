##
# Copyright (C) 2022 Nathanael Gray <nathanael.gray@fphcare.co.nz>
##
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
##
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
##
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
import crc


class Decoder(srd.Decoder):
    api_version = 3
    id = 'sht31'
    name = 'SHT31'
    longname = 'Sensirion SHT31'
    desc = 'Sensirion SHT31 humidity and temperature sensor.'
    license = 'gplv2+'
    inputs = ['i2c']
    outputs = []
    tags = ['Sensor']
    options = (
    )
    annotations = (
        ('celsius', 'Temperature / °C'),
        ('relative-humidity', 'Relative Humidity / %'),
        ('text-verbose', 'Text (verbose)'),
        ('text', 'Text'),
        ('warning', 'Warning'),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 'IDLE'
        self.databytes = []

        width = 8
        poly = 0x31
        init_value = 0xFF
        final_xor_value = 0x00
        reverse_input = False
        reverse_output = False

        configuration = crc.Configuration(
            width, poly, init_value, final_xor_value, reverse_input, reverse_output)

        use_table = True
        self.crc_calculator = crc.CrcCalculator(configuration, use_table)

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putx(self, data):
        # Helper for annotations which span exactly one I²C packet.
        self.put(self.ss, self.es, self.out_ann, data)

    def putb(self, data):
        # Helper for annotations which span a block of I²C packets.
        self.put(self.ss_block, self.es_block, self.out_ann, data)

    def warn_upon_invalid_slave(self, addr):
        # SHT31 devices have a 7-bit I²C slave address either 0x44 or 0x45 (depending on whether ADDR pin is pulled high or low).
        if addr not in range(0x44, 0x45 + 1):
            s = 'Warning: I²C slave address 0x%02x not an SHT31 sensor.'
            self.putx([4, [s % addr]])

    def warn_upon_crc_error(self, data, expected_crc):
        crc = self.crc_calculator.calculate_checksum(data)
        if self.crc_calculator.verify_checksum(data, expected_crc):
            s = 'CRC: %02X'
            self.putx([3, [s % crc]])
        else:
            s = 'Warning: CRC: %02X'
            self.putx([4, [s % crc]])

    def output_temperature(self):
        raw = (self.databytes[0] << 8) | self.databytes[1]
        celsius = -45 + 175 * float(raw) / (2**16 - 1)
        self.putb([0, ['%s: %.1f °C' % ('Temperature', celsius)]])

    def output_relative_humidity(self):
        raw = (self.databytes[2] << 8) | self.databytes[3]
        relative_humidity = 100 * float(raw) / (2**16 - 1)
        self.putb([1, ['%s: %.1f %%' % ('Relative Humidity', relative_humidity)]])

    def handle_periodic_data(self, b, rw):
        self.databytes.append(b)
        if len(self.databytes) == 1 or len(self.databytes) == 4:
            self.ss_block = self.ss
            return
        if len(self.databytes) == 2:
            self.es_block = self.es
            self.output_temperature()
            return
        if len(self.databytes) == 3:
            self.ss_block = self.ss
            self.es_block = self.es
            self.warn_upon_crc_error(self.databytes[0:2], self.databytes[2])
            return
        if len(self.databytes) == 5:
            self.es_block = self.es
            self.output_relative_humidity()
        if len(self.databytes) == 6:
            self.ss_block = self.ss
            self.es_block = self.es
            self.warn_upon_crc_error(self.databytes[3:5], self.databytes[5])
            self.databytes = []
            return

    def decode(self, ss, es, data):
        cmd, databyte = data

        # Store the start/end samples of this I²C packet.
        self.ss, self.es = ss, es

        # State machine.
        if self.state == 'IDLE':
            # Wait for an I²C START condition.
            if cmd != 'START':
                return
            self.state = 'GET SLAVE ADDR'
        elif self.state == 'GET SLAVE ADDR':
            # Wait for an address read/write operation.
            if cmd == 'ADDRESS READ':
                self.warn_upon_invalid_slave(databyte)
                self.state = 'READ PERIODIC DATA'
        elif self.state == 'READ PERIODIC DATA':
            if cmd == 'DATA READ':
                self.handle_periodic_data(databyte, cmd[5:])
            elif cmd == 'STOP':
                self.state = 'IDLE'
            else:
                # self.putx([0, ['Ignoring: %s (data=%s)' % (cmd, databyte)]])
                pass
