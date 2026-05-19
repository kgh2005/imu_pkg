import serial


class EbimuDriver:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser = None

    def connect(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=1.0
        )

    def setup(self):
        commands = [
            "<sof2>",   # Quaternion ON
            "<sog1>",   # Gyro ON
            "<soa1>",   # Acceleration ON
            "<sem0>",   # Magnetometer OFF
            "<sot0>",   # Temperature OFF
            "<sod0>",   # Distance OFF
            "<sor10>",  # 100Hz output
        ]

        for command in commands:
            self.ser.write(command.encode())
            self.ser.readline()

    def read(self):
        line = self.ser.read_until(
            expected=b'\n'
        ).decode(
            'utf-8',
            errors='ignore'
        ).strip()

        return self.parse_line(line)

    def parse_line(self, line):
        if not line or ',' not in line:
            return None

        parts = line.split(',')

        if '*' in parts[0]:
            parts[0] = parts[0].replace('*', '')

        if len(parts) < 10:
            return None

        try:
            values = [float(x) for x in parts[:10]]
        except ValueError:
            return None

        return {
            "qz": values[0],
            "qy": values[1],
            "qx": values[2],
            "qw": values[3],
            "gx": values[4],
            "gy": values[5],
            "gz": values[6],
            "ax": values[7],
            "ay": values[8],
            "az": values[9],
        }

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()