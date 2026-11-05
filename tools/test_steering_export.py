"""Exercise actual firmware export service with host UART/IRQ stubs and PC parser."""
from pathlib import Path
import subprocess
import tempfile
from contextlib import contextmanager
import unittest
import csv
import io
import export_steering_log as exporter

ROOT = Path(__file__).resolve().parent.parent


@contextmanager
def artifact_directory():
    path = tempfile.mkdtemp(dir=ROOT/'demo/mdk/Objects/steering-log-tests')
    yield path  # Retain test evidence inside ignored build outputs.


class Port:
    def __init__(self, data):
        self.data = data
        self.sent = b''

    def write(self, value):
        self.sent += value

    def read(self, size):
        # Split markers, CRLF and CSV values across reads.
        chunk, self.data = self.data[:17], self.data[17:]
        return chunk


class ExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        main = (ROOT/'demo/user/src/main.c').read_text(encoding='utf-8')
        service = main[main.index('static void log_queue('):main.index('void assistant_uart_tx_irq_handler(')]
        dest = ROOT/'demo/mdk/Objects/steering-log-tests'
        dest.mkdir(exist_ok=True)
        code = r'''
#include "steering_log.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <io.h>
#include <fcntl.h>
static uint8_t camera_tx_packet[50000], screen_enabled=1, motor_enabled;
static uint32_t camera_tx_position,camera_tx_length;
static uint8_t log_export_state;
static uint32_t log_export_session,log_export_count,log_export_index,log_export_hash;
static const char *rx="";
static uint32_t debug_read_ring_buffer(uint8_t *b,uint32_t n){uint32_t k=(uint32_t)strlen(rx);if(k>n)k=n;memcpy(b,rx,k);rx+=k;return k;}
static uint32_t interrupt_global_disable(void){return 0;}
static void interrupt_global_enable(uint32_t m){(void)m;}
static void assistant_transport_cancel(void){log_export_state=0;camera_tx_length=0;}
#define __DMB() ((void)0)
#define LPUART1 0
#define kLPUART_TxDataRegEmptyInterruptEnable 0
static void LPUART_EnableInterrupts(int a,int b){(void)a;(void)b;}
''' + service + r'''
int main(void){
 _setmode(_fileno(stdout),_O_BINARY);
 rx="LOG\n";steering_log_service();assert(strstr((char*)camera_tx_packet,"ERROR EMPTY"));camera_tx_length=0;steering_log_service();
 steering_log_start();steering_log_record_t r={0};r.running=1;r.ms=10;r.servo10=870;r.ring_stage=7;r.ring_source=4;steering_log_tick(&r);
 motor_enabled=1;rx="LOG\n";steering_log_service();assert(strstr((char*)camera_tx_packet,"ERROR RUNNING"));camera_tx_length=0;steering_log_service();
 r.ms=30;r.error=-100;r.ring_stage=6;r.ring_guide=19;r.ring_source=1;r.ring_direction=1;
 r.bridge_seen=1;r.bridge_passed=1;r.exit_armed=0;r.transition_frames=2;
 steering_log_tick(&r);r.running=0;r.ms=40;r.stop_reason=1;steering_log_tick(&r);motor_enabled=0;
 rx="LOG\n";steering_log_service();
 while(log_export_state){if(camera_tx_length){fwrite(camera_tx_packet,1,camera_tx_length,stdout);camera_tx_length=0;}steering_log_service();}
 /* Export is repeatable; restart interrupts instead of mixing sessions. */
 rx="LOG\n";steering_log_service();assert(log_export_state==1);camera_tx_length=0;
 steering_log_start();steering_log_service();assert(strstr((char*)camera_tx_packet,"ABORT RESTART"));
 rx="LOG CANCEL\n";steering_log_service();assert(!log_export_state && !camera_tx_length);
 screen_enabled=0;rx="LOG\n";steering_log_service();assert(!log_export_state);
 return 0;
}
'''
        source = dest/'transport_test.c'
        source.write_text(code, encoding='utf-8')
        exe = dest/'transport_test.exe'
        subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-O2',
                        '-DSTEERING_LOG_HOST_TEST','-I',str(ROOT/'demo/user/inc'),str(source),
                        str(ROOT/'demo/user/src/steering_log.c'),'-o',str(exe)],check=True)
        cls.stream = subprocess.check_output([str(exe)])

    def test_actual_firmware_stream(self):
        with artifact_directory() as d:
            output = Path(d)/'run.csv'
            port = Port(b'old camera garbage\n'+self.stream)
            self.assertEqual(exporter.receive(port,output),3)
            self.assertEqual(len(output.read_text().splitlines()),4)
            rows = list(csv.DictReader(io.StringIO(output.read_text())))
            self.assertEqual(len(rows[-1]),33)
            self.assertEqual((rows[0]['ring_stage'],rows[0]['ring_source'],rows[0]['servo10']),('7','4','870'))
            self.assertEqual([rows[-1][k] for k in ('ring_stage','ring_guide','ring_source','ring_direction',
                             'bridge_seen','bridge_passed','exit_armed','transition_frames')],
                             ['6','19','1','1','1','1','0','2'])
            self.assertIn(b'LOG\n',port.sent)
            self.assertFalse(output.with_suffix('.csv.partial').exists())
            with self.assertRaises(FileExistsError):
                exporter.receive(port,output)

    def test_corruption_and_abort_retain_partial(self):
        for data in (self.stream.replace(b'-100',b'-101',1),
                     self.stream[:self.stream.index(b'#LOG END')]+b'#LOG ABORT RESTART\r\n'):
            with artifact_directory() as d:
                output = Path(d)/'run.csv'
                with self.assertRaises((ValueError,RuntimeError)):
                    exporter.receive(Port(data),output)
                self.assertFalse(output.exists())
                self.assertTrue(output.with_suffix('.csv.partial').exists())

    def test_reject_running(self):
        with artifact_directory() as d:
            with self.assertRaisesRegex(RuntimeError,'RUNNING'):
                exporter.receive(Port(b'#LOG ERROR RUNNING\r\n'),Path(d)/'run.csv')

    def test_legacy_25_columns(self):
        lines = self.stream.lstrip(b'\r\n').splitlines(keepends=True)
        header = b','.join(lines[1].rstrip().split(b',')[:25])+b'\r\n'
        data = [b','.join(line.rstrip().split(b',')[:25])+b'\r\n' for line in lines[2:-1]]
        checksum = 2166136261
        for value in b''.join(data):
            checksum = ((checksum ^ value) * 16777619) & 0xffffffff
        session = lines[0].split()[2]
        end = b'#LOG END '+session+b' 3 '+str(checksum).encode()+b'\r\n'
        with artifact_directory() as d:
            output = Path(d)/'legacy.csv'
            self.assertEqual(exporter.receive(Port(lines[0]+header+b''.join(data)+end),output),3)
            self.assertEqual(len(output.read_text().splitlines()[0].split(',')),25)

    def test_column_count_must_match_header(self):
        lines = self.stream.lstrip(b'\r\n').splitlines(keepends=True)
        lines[2] = b','.join(lines[2].rstrip().split(b',')[:-1])+b'\r\n'
        with artifact_directory() as d:
            with self.assertRaisesRegex(ValueError,'Invalid CSV row'):
                exporter.receive(Port(b''.join(lines)),Path(d)/'bad.csv')


if __name__ == '__main__':
    unittest.main()
