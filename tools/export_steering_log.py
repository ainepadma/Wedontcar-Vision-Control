"""Stopped-car CSV export. Requires pyserial; never sends a motor command."""
import argparse
from pathlib import Path
import time


def receive(port, output, timeout=600):
    output = Path(output)
    partial = output.with_suffix(output.suffix + '.partial')
    if output.exists() or partial.exists():
        raise FileExistsError(f'Refusing overwrite: {output} / {partial}')
    deadline = time.monotonic() + timeout
    last_data = time.monotonic()
    pending = bytearray()
    session = count = None
    rows = 0
    checksum = 2166136261
    header_seen = False
    columns = 0
    # Reserve partial first. Failures retain evidence; only validated data is renamed.
    with partial.open('xb') as f:
        port.write(b'\nLOG\n')
        while time.monotonic() < deadline:
            chunk = port.read(4096)
            if chunk:
                last_data = time.monotonic()
                pending.extend(chunk)
            elif time.monotonic() - last_data > 20:
                raise TimeoutError('No data for 20s; keep power, enable C12 and retry.')
            while b'\n' in pending:
                line, _, tail = pending.partition(b'\n')
                pending = bytearray(tail)
                line = bytes(line) + b'\n'
                if session is None:
                    # A cancelled camera frame may precede the first marker.
                    if line.startswith(b'#LOG ERROR'):
                        raise RuntimeError(line.decode('ascii').strip())
                    if not line.startswith(b'#LOG BEGIN '):
                        continue
                    fields = line.decode('ascii').split()
                    session, count, overwritten = map(int, fields[2:5])
                    if not 1 <= count <= 21845:
                        raise ValueError('Invalid record count')
                    print(f'Session {session}: {count} records, overwritten {overwritten}, {fields[5]}')
                    continue
                if line.startswith(b'#LOG END '):
                    end_session, end_count, expected_hash = map(int, line.split()[2:5])
                    if not header_seen or (end_session, end_count, expected_hash) != (session, rows, checksum) or rows != count:
                        raise ValueError('Incomplete/corrupt export: count or checksum mismatch')
                    f.flush()
                    break
                if line.startswith(b'#LOG'):
                    raise RuntimeError(f'Export interrupted: {line!r}')
                if not header_seen:
                    columns = len(line.rstrip().split(b','))
                    if not line.startswith(b'ms,frame,frame_age_ms,') or columns not in (25, 33):
                        raise ValueError('Invalid CSV header')
                    header_seen = True
                else:
                    values = line.rstrip().split(b',')
                    if len(values) != columns:
                        raise ValueError('Invalid CSV row')
                    for value in values:
                        int(value)
                    for value in line:
                        checksum = ((checksum ^ value) * 16777619) & 0xffffffff
                    rows += 1
                    if rows > count:
                        raise ValueError('Too many records')
                f.write(line)
            else:
                if len(pending) > 65536:
                    raise ValueError('Unframed serial stream')
                continue
            break  # validated END
        else:
            raise TimeoutError('Export deadline exceeded')
    partial.rename(output)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='e.g. COM6')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    import serial
    with serial.Serial(args.port, args.baud, timeout=0.25, write_timeout=2) as port:
        try:
            rows = receive(port, args.output)
        finally:
            port.write(b'\nLOG CANCEL\n')
    print(f'Saved {rows} verified rows: {args.output.resolve()}')


if __name__ == '__main__':
    main()
