import sys
import time
import select
import termios
import tty
from datetime import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LOG_FILE = 'waktu_gerak.txt'

# kunci -> (linear, angular, nama arah)
COMMANDS = {
    'w': (1.5, 0.0, 'MAJU'),
    's': (-1.5, 0.0, 'MUNDUR'),
    'a': (0.0, 0.5, 'PUTAR KIRI'),
    'd': (0.0, -0.5, 'PUTAR KANAN'),
    ' ': (0.0, 0.0, 'BERHENTI'),
}


class ManualControl(Node):

    def __init__(self):
        super().__init__('manual_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher_.publish(msg)


def get_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)

    if ready:
        key = sys.stdin.read(1)

        if key == '\x1b':
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if ready and sys.stdin.read(1) == '[':
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if ready:
                    arrow = sys.stdin.read(1)
                    return {'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}.get(arrow, '')
            return ''

        return key.lower()

    return ''


def log_line(text):
    """Tulis satu baris ke file langsung (flush), supaya aman kalau program mati."""
    with open(LOG_FILE, 'a') as f:
        f.write(text + '\n')


def main(args=None):

    if not sys.stdin.isatty():
        print('Jalankan program melalui terminal interaktif.')
        return

    old_settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = ManualControl()

    print("""
=================================
 KENDALI MANUAL ROBOT
=================================
 W / Panah atas   : Maju
 S / Panah bawah  : Mundur
 A / Panah kiri   : Putar kiri
 D / Panah kanan  : Putar kanan
 Spasi            : Berhenti (robot)
 M                : Stop robot + pause timer
 Q                : Keluar
=================================
Timer mulai/lanjut saat perintah
gerak dimasukkan. Log: waktu_gerak.txt
""")

    # --- status timer ---
    accumulated = 0.0       # total waktu dari segmen yang sudah selesai (detik)
    segment_start = 0.0     # kapan segmen berjalan sekarang dimulai
    timer_running = False
    last_key = ''           # perintah terakhir yang dicatat (anti-duplikat)

    def elapsed():
        if timer_running:
            return accumulated + (time.monotonic() - segment_start)
        return accumulated

    log_line(f'=== Sesi {datetime.now():%Y-%m-%d %H:%M:%S} ===')

    try:
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():

            key = get_key()

            if key in COMMANDS:
                linear, angular, name = COMMANDS[key]
                node.send_command(linear, angular)

                # Lanjutkan timer jika sedang pause / belum mulai
                # (hanya oleh perintah gerak, bukan spasi)
                if key != ' ' and not timer_running:
                    timer_running = True
                    segment_start = time.monotonic()

                # Catat hanya jika perintah BERUBAH (tahan tombol = banyak
                # repeat, tidak dicatat berkali-kali)
                if key != last_key:
                    t = elapsed()
                    log_line(f'{t:7.1f} s | perintah: {key.upper() if key != " " else "SPASI"}'
                             f' | arah: {name}')
                    last_key = key

            elif key == 'm':
                # Hentikan robot SELALU, lalu pause timer jika sedang jalan
                node.send_command(0.0, 0.0)

                if timer_running:
                    accumulated = elapsed()
                    timer_running = False
                    log_line(f'{accumulated:7.1f} s | perintah: M'
                             f' | arah: BERHENTI + PAUSE')
                    last_key = ''   # perintah sama setelah pause tetap dicatat
                    print(f'\rRobot berhenti, timer pause di {accumulated:.1f} s'
                          f'          ')

            elif key == 'q':
                break

            if timer_running:
                print(f'\rTimer: {elapsed():6.1f} s', end='', flush=True)

            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass

    finally:
        total = elapsed()
        log_line(f'--- selesai, total waktu berjalan: {total:.1f} s ---')

        for _ in range(5):
            node.send_command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print('\nProgram kendali manual selesai.')


if __name__ == '__main__':
    main()