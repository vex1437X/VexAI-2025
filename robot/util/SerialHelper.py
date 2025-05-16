import serial
import struct
import time
from robot.util.Constants import *
from robot.util.Constants import DataTag
import threading

from dataclasses import dataclass, field


class VarState:
    value: float = 0.0  # last value we saw
    updated: bool = False  # did we see this tag this tick?
    ts: float = field(default_factory=time.time)  # (optional) wall‑clock of last update


class SerialHelper:
    # Special byte values for packet framing and escaping
    START = 0xAA
    END = 0xAB
    ESCAPE = 0xAC

    def __init__(
        self,
        serial_port: str = "/dev/ttyACM1",
        baud_rate: int = 9600,
        timeout: float = 0.01,
    ):
        self.ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
        self.rx_buffer = bytearray()
        # NEW ➜ tag → VarState
        self.state: dict[DataTag, VarState] = {tag: VarState() for tag in DataTag}

    def send_command(self, command: bytes):
        # Send a command over the serial connection
        # print(f"Sending command: {command}")
        try:
            self.ser.write(command)
            self.ser.flush()
        except serial.SerialException as e:
            print(f"Serial error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def build_doubles(self, doubles):
        # Convert a list of doubles into a byte array
        raw = bytearray()
        for val in doubles:
            packed = struct.pack(">d", val)  # Pack each double in big-endian format
            raw.extend(packed)
        return raw

    def escape_bytes(self, data: bytes) -> bytes:
        # Escape special bytes in the data to avoid conflicts with START, END, and ESCAPE
        escaped = bytearray()
        for b in data:
            if b == self.START:
                escaped.append(self.ESCAPE)
                escaped.append(0x00)
            elif b == self.END:
                escaped.append(self.ESCAPE)
                escaped.append(0x01)
            elif b == self.ESCAPE:
                escaped.append(self.ESCAPE)
                escaped.append(0x02)
            else:
                escaped.append(b)
        return bytes(escaped)

    def encode_instruction(self, instr: Instruction, operands: list[float]) -> bytes:
        # Encode an instruction with its operands into a packet
        packet = bytearray()
        packet.append(self.START)  # Add start byte
        packet.append(instr.value)  # Add instruction byte

        # Convert operands to bytes and escape special characters
        raw_operands = self.build_doubles(operands)
        packet.extend(self.escape_bytes(raw_operands))

        packet.append(self.END)  # Add end byte
        return bytes(packet)

    def decode_vex_data_packet(self, packet_bytes: bytes) -> dict:
        # Decode a VEX data packet into a dictionary of data tags and values
        idx = 0
        data = {}
        length = len(packet_bytes)

        while idx < length:
            # Read the data tag
            tag_val = packet_bytes[idx]
            idx += 1
            tag = DataTag(tag_val) if tag_val in (t.value for t in DataTag) else None
            if tag is None:
                break

            # Ensure there are enough bytes for a double value
            if idx + 8 > length:
                break

            # Read the double value
            double_bytes = packet_bytes[idx : idx + 8]
            idx += 8
            value = struct.unpack(">d", double_bytes)[
                0
            ]  # Unpack the double in big-endian format
            data[tag] = value

        return data

    def read_vex_data(self) -> list[dict]:
        # Read and decode VEX data packets from the serial buffer
        packets_found = []

        # Read incoming data from the serial port
        incoming = self.ser.read(1024)
        if incoming:
            self.rx_buffer.extend(incoming)

        i = 0
        while i < len(self.rx_buffer):
            if self.rx_buffer[i] == self.START:  # Look for the start byte
                j = i + 1
                escaped_data = bytearray()
                i_advance = 0

                while j < len(self.rx_buffer):
                    if self.rx_buffer[j] == self.END:  # Look for the end byte
                        i_advance = j + 1 - i
                        break
                    elif self.rx_buffer[j] == self.ESCAPE:  # Handle escaped bytes
                        if j + 1 < len(self.rx_buffer):
                            escaped_code = self.rx_buffer[j + 1]
                            if escaped_code == 0x00:
                                escaped_data.append(self.START)
                            elif escaped_code == 0x01:
                                escaped_data.append(self.END)
                            elif escaped_code == 0x02:
                                escaped_data.append(self.ESCAPE)
                            j += 2
                        else:
                            j += 1
                        continue
                    else:
                        escaped_data.append(self.rx_buffer[j])
                        j += 1

                if i_advance > 0:
                    # Decode the escaped data into a packet dictionary
                    packet_dict = self.decode_vex_data_packet(escaped_data)
                    packets_found.append(packet_dict)
                    i += i_advance
                else:
                    break
            else:
                i += 1

        # Remove processed bytes from the buffer
        if i > 0:
            self.rx_buffer = self.rx_buffer[i:]

        return packets_found

    def periodic(self) -> None:
        """Read every complete packet in the UART RX buffer
        and update `self.state` in‑place.

        Call this exactly once per control‑loop iteration.
        """
        #  1) clear all updated flags
        for vs in self.state.values():
            vs.updated = False

        #  2) pull every finished packet off the wire
        packets: list[dict] = self.read_vex_data()

        #  3) fold into the state table
        now = time.time()
        for pkt in packets:
            for tag, val in pkt.items():
                vs = self.state[tag]
                vs.value = val
                vs.updated = True
                vs.ts = now  # optional – handy for timeouts

    def value(self, tag: DataTag) -> float:
        return self.state[tag].value

    def was_updated(self, tag: DataTag) -> bool:
        return self.state[tag].updated
