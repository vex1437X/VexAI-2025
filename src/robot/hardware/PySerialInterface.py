import serial
import struct
from src.core.config.ConfigModel import ConfigModel
from src.core.hardware.SerialInterface import SerialInterface
from src.core.config.util import Instruction, DataTag
from src.core.logging.logger_config import get_logger

logger = get_logger(__name__)


class PySerialInterface(SerialInterface):
    START = 0xAA
    END = 0xAB
    ESCAPE = 0xAC

    _instance = None

    @classmethod
    def get_instance(cls, config: ConfigModel = None) -> "PySerialInterface":
        if cls._instance is None:
            if config is None:
                raise RuntimeError(
                    "First call to get_instance() must pass a ConfigModel"
                )
            # create, open, and store the one instance
            cls._instance = cls(config)
            cls._instance.open()
        return cls._instance

    def __init__(self, config: ConfigModel):
        self.serial_port = config.serial_port
        self.baud_rate = config.baud_rate
        self.timeout = config.timeout
        self.ser = None
        self.rx_buffer = bytearray()

    def open(self) -> None:
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout)

    def read(self, size: int = 1024) -> bytes:
        incoming = self.ser.read(size)
        if incoming:
            self.rx_buffer.extend(incoming)
        packets = self._extract_packets()
        # Flatten to raw escaped data or return list of decoded dicts
        return packets

    def write(self, data: bytes) -> None:
        if self.ser and self.ser.is_open:
            self.ser.write(data)
            self.ser.flush()
            # logger.info(f"Sent data: {data}")

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def build_doubles(self, doubles: list[float]) -> bytearray:
        raw = bytearray()
        for val in doubles:
            raw.extend(struct.pack(">d", float(val)))
        return raw

    def escape_bytes(self, data: bytes) -> bytearray:
        escaped = bytearray()
        for b in data:
            if b == self.START:
                escaped.extend([self.ESCAPE, 0x00])
            elif b == self.END:
                escaped.extend([self.ESCAPE, 0x01])
            elif b == self.ESCAPE:
                escaped.extend([self.ESCAPE, 0x02])
            else:
                escaped.append(b)
        return escaped

    def encode_instruction(self, instr: Instruction, operands: list[float]) -> bytes:
        packet = bytearray([self.START, instr.value])
        packet.extend(self.escape_bytes(self.build_doubles(operands)))
        packet.append(self.END)
        logger.info(f"Encoded instruction {instr.name} with operands {operands}")
        return bytes(packet)

    def _extract_packets(self) -> list[dict]:
        i = 0
        packets_found = []
        while i < len(self.rx_buffer):
            if self.rx_buffer[i] == self.START:
                j = i + 1
                escaped = bytearray()
                while j < len(self.rx_buffer) and self.rx_buffer[j] != self.END:
                    if self.rx_buffer[j] == self.ESCAPE and j + 1 < len(self.rx_buffer):
                        code = self.rx_buffer[j + 1]
                        if code == 0x00:
                            escaped.append(self.START)
                        elif code == 0x01:
                            escaped.append(self.END)
                        elif code == 0x02:
                            escaped.append(self.ESCAPE)
                        j += 2
                    else:
                        escaped.append(self.rx_buffer[j])
                        j += 1
                if j < len(self.rx_buffer) and self.rx_buffer[j] == self.END:
                    packet_dict = self.decode_vex_data_packet(escaped)
                    packets_found.append(packet_dict)
                    i = j + 1
                else:
                    break
            else:
                i += 1
        # trim processed
        self.rx_buffer = self.rx_buffer[i:]
        return packets_found

    def decode_vex_data_packet(self, packet_bytes: bytes) -> dict:
        idx = 0
        data = {}
        while idx + 1 < len(packet_bytes):
            tag_val = packet_bytes[idx]
            idx += 1
            try:
                tag = DataTag(tag_val)
            except ValueError:
                break
            if idx + 8 > len(packet_bytes):
                break
            value = struct.unpack(">d", packet_bytes[idx : idx + 8])[0]
            idx += 8
            data[tag] = value
        return data
