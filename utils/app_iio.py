# vim: set ts=4 sw=4 sts=4 et :
import os
import struct
import glob
import re

class IIODevice:
    def __init__(self, sysfs_path):
        self.sysfs_path = sysfs_path
        self.dev_num = os.path.basename(sysfs_path).replace("iio:device", "")
        self.dev_node = f"/dev/iio:device{self.dev_num}"
        self.name = self._read_sysfs("name")

        self.channels = []
        scan_dir = os.path.join(sysfs_path, "scan_elements")
        if os.path.exists(scan_dir):
            en_files = glob.glob(os.path.join(scan_dir, "in_*_en"))
            self.channels = sorted([os.path.basename(f)[3:-3] for f in en_files])

        # Assuming this is constant...
        self.channels_by_idx = sorted(self.channels,
            key=lambda ch: int(self._read_sysfs(f"scan_elements/in_{ch}_index")))

        self.selected_channels = {} # Either dict or list works

    def set_frequency(self, channel, freq_hz):
        while channel:
            path = os.path.join(self.sysfs_path, f"in_{channel}_sampling_frequency")
            if os.path.exists(path):
                with open(path, "w") as f:
                    f.write(str(freq_hz))
                    return
            # Try all prefixes of the channel name until one has the right node
            channel = channel[:-1]
        raise Exception('Sampling frequency node not found')

    def _parse_type(self, channel):
        """Parses nodes like 'le:s16/16>>0' or 'be:u32/32>>0'."""
        t = self._read_sysfs(f"scan_elements/in_{channel}_type")

        # Regex to extract: endian, sign, bits, storage_bits
        match = re.match(r"(le|be):([su])(\d+)/(\d+)>>(\d+)", t)
        if not match:
            raise ValueError(f"Unsupported type format: {t}")

        endian, sign, bits, storage, shift = match.groups()

        prefix = "<" if endian == "le" else ">"
        # Map storage bits to struct format characters
        fmt_map = {"16": "h" if sign == "s" else "H",
                   "32": "i" if sign == "s" else "I",
                   "64": "q" if sign == "s" else "Q"}

        # TODO: handle "bits"
        return prefix + fmt_map[storage], int(storage) // 8, int(shift)

    def _get_metadata(self, channel, key):
        """Reads scale or offset; returns 1.0 or 0.0 if missing."""
        while channel:
            path = os.path.join(self.sysfs_path, f"in_{channel}_{key}")
            if os.path.exists(path):
                with open(path, "r") as f:
                    return float(f.read().strip())
            # Try all prefixes of the channel name until one has the right node
            channel = channel[:-1]

        return 1.0 if key == "scale" else 0.0

    def start_buffering(self, selected_channels=None, sample_count=2):
        self.stop_buffering()

        if selected_channels is None:
            selected_channels = self.selected_channels

        # Note: with some drivers (not clear if all) the buffer will contain
        # all of the channels regardless of which ones we enabled, or a
        # continuous range of channel indices from 0 to the maximum index
        # among those that are enabled.  This contradicts
        # https://docs.kernel.org/driver-api/iio/buffers.html but we could
        # deal with it if we knew what the driver is going to do... but we
        # don't.  So enable all channels until the one from selected_channels
        # that has the highest index.  This way we don't care whether the
        # driver is compliant or not with the downside of potentially
        # requesting unneeded channels when working with compliant drivers.
        count = max([self.channels_by_idx.index(ch) for ch in selected_channels]) + 1

        self.fmt = ""
        self.sample_size = 0
        self.active_channels = []

        for i, ch in enumerate(self.channels_by_idx[:count]):
            self._write_sysfs(f"scan_elements/in_{ch}_en", "1")

            ch_fmt, ch_size, ch_shift = self._parse_type(ch)
            self.fmt += ch_fmt[1 if self.fmt else 0:] # Set endianness from 1st channel
            self.sample_size += ch_size

            if ch in selected_channels:
                self.active_channels.append((
                    i, ch, self._get_metadata(ch, "scale") / (1 << ch_shift),
                    self._get_metadata(ch, "offset")))
        for ch in self.channels_by_idx[count:]:
            self._write_sysfs(f"scan_elements/in_{ch}_en", "0")

        self._write_sysfs("buffer/length", str(sample_count))
        self._write_sysfs("buffer/enable", "1")

        self.readings = {}

    def stop_buffering(self):
        self._write_sysfs("buffer/enable", "0")
        if hasattr(self, 'active_channels'):
            del self.active_channels

    def get_latest(self):
        data = None
        try:
            fd = os.open(self.dev_node, os.O_RDONLY | os.O_NONBLOCK)
            while True:
                try:
                    chunk = os.read(fd, self.sample_size)
                    if not chunk or len(chunk) < self.sample_size: break
                    data = chunk
                except OSError: break
            os.close(fd)
        except Exception: return None

        if not data: return None

        raw_values = struct.unpack(self.fmt, data)
        # Calibrated Value = (Raw + Offset) * Scale
        self.readings = {ch: (raw_values[i] + offset) * scale for i, ch, scale, offset in self.active_channels}
        return self.readings

    def _read_sysfs(self, subpath):
        with open(os.path.join(self.sysfs_path, subpath), "r") as f:
            return f.read().strip()

    def _write_sysfs(self, subpath, value):
        with open(os.path.join(self.sysfs_path, subpath), "w") as f:
            f.write(value)

    def check_access(self):
        # Check /dev and /sys permissions, not all the files in there but
        # assume permissions on other files are consistent
        return (os.access(self.dev_node, os.R_OK) and
            os.access(os.path.join(self.sysfs_path, 'buffer/enable'), os.W_OK))

def detect_iio_devices():
    return [IIODevice(d) for d in glob.glob("/sys/bus/iio/devices/iio:device*")]

# --- Usage Example ---
if __name__ == "__main__":
    devices = detect_iio_devices()
    print(f'{len(devices)} devices:')
    for d in devices:
        print(f'  "{d.name}" channels: {", ".join(d.channels)}')
        print(f'    access {"" if d.check_access() else "NOT "}Ok')
    gyro = next((d for d in devices if "gyro" in d.name.lower() or "ish" in d.name.lower()), None)

    if gyro:
        print(f"Found: {gyro.name} with channels: {gyro.channels}")
        #gyro.set_frequency('anglvel', 10)
        gyro.start_buffering(["anglvel_x", "anglvel_y", "anglvel_z"], 3)

        # Read a sample
        import time
        for i in range(20):
            time.sleep(0.2) # Give buffer a moment to fill
            print(f"Latest Values: {gyro.get_latest()}")
        gyro.stop_buffering()
