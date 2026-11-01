"""Rebuild a disposable Keil project with explicit local toolchain overrides.

Never flashes, edits the source project, or changes global Keil configuration.
All copied project files and outputs stay under ignored demo/mdk/Objects.
"""
import argparse
from datetime import datetime
import hashlib
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uv4', type=Path, required=True)
    parser.add_argument('--compiler-id', help='Example: 6210000::V6.21::ARMCLANG')
    parser.add_argument('--pack-id', help='Example: NXP.MIMXRT1064_DFP.19.0.0')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = root / 'demo/mdk/rt1064.uvprojx'
    if not args.uv4.is_file():
        parser.error('UV4 executable does not exist.')
    destination = source.parent / 'Objects' / ('local-build-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    destination.mkdir(parents=True)
    tree = ET.parse(source)
    project = tree.getroot()
    for tag in ('FilePath', 'ScatterFile', 'IncludePath'):
        for node in project.iter(tag):
            if not node.text:
                continue
            node.text = ';'.join(str((source.parent / p).resolve()) if p.startswith('.') else p
                                 for p in node.text.split(';'))
    for tag in ('OutputDirectory', 'ListingPath'):
        for node in project.iter(tag):
            node.text = str(destination) + '\\'
    if args.compiler_id:
        for node in project.iter('pCCUsed'):
            node.text = args.compiler_id
    if args.pack_id:
        for node in project.iter('PackID'):
            node.text = args.pack_id
    copied = destination / 'rt1064-local.uvprojx'
    tree.write(copied, encoding='utf-8', xml_declaration=True)
    logfile = destination / 'rebuild.log'
    source_digest = hashlib.sha256(source.read_bytes()).hexdigest()
    print('BUILD_DIRECTORY=' + str(destination), flush=True)
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    temporary = destination / 'temp'
    temporary.mkdir()
    build_environment = os.environ.copy()
    build_environment['TEMP'] = str(temporary)
    build_environment['TMP'] = str(temporary)
    result = subprocess.run([str(args.uv4), '-r', str(copied), '-t', 'nor_sdram_zf_dtcm',
                             '-o', str(logfile)], cwd=destination, startupinfo=startup,
                            env=build_environment)
    if hashlib.sha256(source.read_bytes()).hexdigest() != source_digest:
        raise RuntimeError('Source project changed during build; inspect concurrent edits.')
    if logfile.exists():
        text = logfile.read_text(encoding='utf-8', errors='replace')
        print('\n'.join(text.splitlines()[-16:]))
    else:
        raise RuntimeError('Keil did not produce a build log.')
    axf = destination / 'rt1064.axf'
    if result.returncode not in (0, 1) or not axf.exists() or '0 Error(s)' not in text:
        raise RuntimeError(f'Keil build failed (exit={result.returncode}); see {logfile}')
    print('AXF_SHA256=' + hashlib.sha256(axf.read_bytes()).hexdigest())


if __name__ == '__main__':
    main()
