import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


R_ARC = 1.533 

PLAN = [
    ("line", 1.400),                      
    ("turn", +90),                         
    ("line", 0.660),                       
    ("turn", -90),                         
    ("line", 1.215),                      
    ("turn", -90),                         
    ("line", 2.240),                       
    ("turn", +90),                         
    ("line", 4.350),                       
    ("turn", +90),                         
    ("line", 5.029),                       
    ("arc", R_ARC, +180),                  
    ("line", 1.554),                       
    ("turn", -90),                         
    ("line", 0.910),                       
    ("turn", -90),                         
    ("line", 2.515),                       
    ("turn", +90),                         
    ("line", 1.635),                      
    ("turn", -90),                        
    ("line", 0.760),                       
    ("turn", +90),                        
    ("line", 2.025),                      
    ("turn", +90),                         
    ("line", 2.645),                      
]

APPROACH_FROM_ORIGIN = [
    ("turn", +90),
    ("line", 2.375),
    ("turn", -90),
    ("line", 1.250),
]


def trapezoid(total, vmax, amax):
    """Profil kecepatan |v|(t) yang total integralnya = total. Return (T, f)."""
    total = abs(total)
    if total < 1e-9:
        return 0.0, (lambda t: 0.0)
    if total >= vmax * vmax / amax:
        ta = vmax / amax
        tc = (total - vmax * vmax / amax) / vmax
        vp = vmax
    else:                                   
        vp = math.sqrt(total * amax)
        ta = vp / amax
        tc = 0.0
    T = 2.0 * ta + tc

    def f(t):
        if t <= 0.0 or t >= T:
            return 0.0
        if t < ta:
            return amax * t
        if t < ta + tc:
            return vp
        return amax * (T - t)

    return T, f


class Segment:
    """Satu primitif gerak. rate(t) -> (v_real m/s, w_real rad/s) pada skala dunia nyata."""

    def __init__(self, kind, args, p):
        self.kind = kind
        if kind == "line":
            d = args[0]
            self.T, f = trapezoid(d, p["lin_speed"], p["lin_acc"])
            s = math.copysign(1.0, d)
            self.rate = lambda t: (s * f(t), 0.0)
        elif kind == "turn":
            a = math.radians(args[0])
            self.T, f = trapezoid(a, p["ang_speed"], p["ang_acc"])
            s = math.copysign(1.0, a)
            self.rate = lambda t: (0.0, s * f(t))
        elif kind == "arc":
            r, a = args[0], math.radians(args[1])
            self.T, f = trapezoid(r * abs(a), p["lin_speed"], p["lin_acc"])
            s = math.copysign(1.0, a)
            self.rate = lambda t: (f(t), s * f(t) / r)
        else:
            raise ValueError(f"segmen tidak dikenal: {kind}")


def build_timeline(plan, p):
    """Gabungkan semua segmen + jeda diam (settle) antar segmen. Return list (t_mulai, Segment)."""
    tl, t = [], 0.0
    for item in plan:
        seg = Segment(item[0], item[1:], p)
        tl.append((t, seg))
        t += seg.T
        if p["settle"] > 0.0:
            tl.append((t, Segment("line", (0.0,), p)))   
            tl[-1][1].T = p["settle"]
            t += p["settle"]
    return tl, t


class ArenaOpenLoop(Node):
    def __init__(self):
        super().__init__("arena_open_loop")

        defaults = {
            "lin_speed": 0.25,    
            "lin_acc": 0.25,     
            "ang_speed": 1.0,    
            "ang_acc": 2.0,       
            "lin_scale": 0.3,     
            "ang_scale": 0.7941,  
            "settle": 0.4,        
            "start_delay": 3.0,   
            "rate_hz": 50.0,
            "approach_from_origin": False,
            "cmd_topic": "/cmd_vel",
        }
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}

        plan = (APPROACH_FROM_ORIGIN if self.p["approach_from_origin"] else []) + PLAN
        self.timeline, self.total = build_timeline(plan, self.p)

        self.pub = self.create_publisher(Twist, self.p["cmd_topic"], 10)
        self.t0 = None
        self.finished = False
        self.timer = self.create_timer(1.0 / self.p["rate_hz"], self.tick)

        use_sim = self.get_parameter("use_sim_time").value
        self.get_logger().info(
            f"{len(plan)} langkah, durasi gerak {self.total:.1f} s "
            f"(use_sim_time={use_sim}, lin_scale={self.p['lin_scale']}, "
            f"ang_scale={self.p['ang_scale']})"
        )
        if not use_sim:
            self.get_logger().warn(
                "use_sim_time=false: waktu mengikuti jam dinding. Jika Gazebo RTF < 1, "
                "jarak tempuh akan kurang. Jalankan dengan -p use_sim_time:=true."
            )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, v_real, w_real):
        m = Twist()
        m.linear.x = v_real / self.p["lin_scale"]
        m.angular.z = w_real / self.p["ang_scale"]
        self.pub.publish(m)

    def tick(self):
        if self.finished:
            return
        if self.t0 is None:
            if self.pub.get_subscription_count() == 0 or self.now() <= 0.0:
                return
            self.t0 = self.now() + self.p["start_delay"]
            self.get_logger().info("mulai dalam beberapa detik.")
            return

        t = self.now() - self.t0
        if t < 0.0:
            return
        if t >= self.total:
            self.stop_and_finish()
            return

        for t_start, seg in reversed(self.timeline):
            if t >= t_start:
                v, w = seg.rate(t - t_start)
                self.send(v, w)
                break

    def stop_and_finish(self):
        for _ in range(5):
            self.send(0.0, 0.0)
        self.finished = True
        self.get_logger().info("Selesai: robot berhenti di titik akhir.")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ArenaOpenLoop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.send(0.0, 0.0)     
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()