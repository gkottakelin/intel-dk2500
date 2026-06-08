"""Auto-detect a bus-servo directional limit with motor mode.

The tester commands a low motor speed, polls the servo position, and treats the
direction as limited when the position remains stable for a configured duration.
It is intentionally conservative: hardware motion requires ``--confirm`` and a
stop command is sent in a ``finally`` block.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from typing import Callable, Iterable, Optional

try:
    from .bus_servo import DEFAULT_BAUDRATE, BusServoController
except ImportError:
    from bus_servo import DEFAULT_BAUDRATE, BusServoController  # type: ignore


DEFAULT_COM_PORT = "COM15"
DEFAULT_DUTY_SPEED = 80
DEFAULT_FIXED_SPEED = 10


@dataclass(frozen=True)
class LimitTestResult:
    servo_id: int
    direction: str
    signed_speed: int
    start_position: int
    limit_position: int
    elapsed_s: float
    stable_s: float
    samples: int
    reason: str
    timed_out: bool

    def to_dict(self) -> dict:
        return asdict(self)


class PositionStabilityTracker:
    """Track whether position has stayed within tolerance long enough."""

    def __init__(self, start_position: int, start_time: float, *, tolerance: int) -> None:
        self.reference_position = start_position
        self.reference_since = start_time
        self.tolerance = tolerance

    def update(self, position: int, now: float) -> float:
        if abs(position - self.reference_position) > self.tolerance:
            self.reference_position = position
            self.reference_since = now
        return now - self.reference_since


class ServoLimitTester:
    def __init__(
        self,
        controller,
        servo_id: int,
        *,
        speed: int,
        fixed_speed_mode: bool = False,
        stable_duration: float = 2.0,
        stable_tolerance: int = 2,
        poll_interval: float = 0.1,
        max_duration: float = 20.0,
        return_servo_mode: bool = True,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        progress: Optional[Callable[[str], None]] = print,
    ) -> None:
        if servo_id < 0 or servo_id > 253:
            raise ValueError("servo_id must be in 0..253")
        if speed <= 0:
            raise ValueError("speed must be a positive magnitude; direction controls the sign")
        if stable_duration <= 0:
            raise ValueError("stable_duration must be positive")
        if stable_tolerance < 0:
            raise ValueError("stable_tolerance must be non-negative")
        if poll_interval <= 0:
            raise ValueError("poll_interval must be positive")
        if max_duration <= 0:
            raise ValueError("max_duration must be positive")
        if fixed_speed_mode and speed > 50:
            raise ValueError("fixed speed mode requires speed <= 50")
        if not fixed_speed_mode and speed > 1000:
            raise ValueError("duty speed mode requires speed <= 1000")

        self.controller = controller
        self.servo_id = servo_id
        self.speed = speed
        self.fixed_speed_mode = fixed_speed_mode
        self.stable_duration = stable_duration
        self.stable_tolerance = stable_tolerance
        self.poll_interval = poll_interval
        self.max_duration = max_duration
        self.return_servo_mode = return_servo_mode
        self.monotonic = monotonic
        self.sleep = sleep
        self.progress = progress

    def run_direction(self, direction: str) -> LimitTestResult:
        direction = normalize_direction(direction)
        signed_speed = self.speed if direction == "positive" else -self.speed
        start_position = self.controller.read_position(self.servo_id)
        start_time = self.monotonic()
        tracker = PositionStabilityTracker(
            start_position,
            start_time,
            tolerance=self.stable_tolerance,
        )
        samples = 0
        last_position = start_position
        last_stable_s = 0.0

        self._log(
            f"direction={direction} servo_id={self.servo_id} "
            f"start_position={start_position} signed_speed={signed_speed}"
        )

        self.controller.set_motor_speed(
            self.servo_id,
            signed_speed,
            fixed_speed_mode=self.fixed_speed_mode,
        )

        try:
            while True:
                now = self.monotonic()
                elapsed = now - start_time
                if elapsed > self.max_duration:
                    return LimitTestResult(
                        servo_id=self.servo_id,
                        direction=direction,
                        signed_speed=signed_speed,
                        start_position=start_position,
                        limit_position=last_position,
                        elapsed_s=elapsed,
                        stable_s=last_stable_s,
                        samples=samples,
                        reason="max_duration",
                        timed_out=True,
                    )

                position = self.controller.read_position(self.servo_id)
                samples += 1
                stable_s = tracker.update(position, now)
                last_position = position
                last_stable_s = stable_s
                self._log(
                    f"direction={direction} pos={position} "
                    f"elapsed={elapsed:.2f}s stable={stable_s:.2f}s"
                )

                if stable_s >= self.stable_duration:
                    return LimitTestResult(
                        servo_id=self.servo_id,
                        direction=direction,
                        signed_speed=signed_speed,
                        start_position=start_position,
                        limit_position=position,
                        elapsed_s=elapsed,
                        stable_s=stable_s,
                        samples=samples,
                        reason="stable_position",
                        timed_out=False,
                    )

                self.sleep(self.poll_interval)
        finally:
            self.stop()

    def stop(self) -> None:
        self.controller.set_motor_speed(
            self.servo_id,
            0,
            fixed_speed_mode=self.fixed_speed_mode,
        )
        if self.return_servo_mode:
            self.controller.set_servo_mode(self.servo_id)

    def _log(self, message: str) -> None:
        if self.progress is not None:
            self.progress(message)


def normalize_direction(direction: str) -> str:
    value = direction.lower().strip()
    if value in {"positive", "+", "pos"}:
        return "positive"
    if value in {"negative", "-", "neg"}:
        return "negative"
    raise ValueError("direction must be positive or negative")


def expand_directions(direction: str) -> list[str]:
    if direction == "both":
        return ["positive", "negative"]
    return [normalize_direction(direction)]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Detect a bus-servo limit by driving motor mode until position "
            "stays stable for the configured duration."
        )
    )
    parser.add_argument("--com-port", default=DEFAULT_COM_PORT, help="serial port, default COM15")
    parser.add_argument("--servo-id", type=int, required=True, help="servo ID, 0..253")
    parser.add_argument("--direction", choices=["positive", "negative", "both"], required=True)
    parser.add_argument("--speed", type=int, default=None, help="positive speed magnitude")
    parser.add_argument("--fixed-speed-mode", action="store_true", help="use fixed-speed motor mode")
    parser.add_argument("--stable-duration", type=float, default=2.0, help="seconds at same position before limit")
    parser.add_argument("--stable-tolerance", type=int, default=2, help="position units treated as unchanged")
    parser.add_argument("--poll-interval", type=float, default=0.1, help="seconds between position reads")
    parser.add_argument("--max-duration", type=float, default=20.0, help="maximum seconds per direction")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--timeout", type=float, default=0.2, help="serial read timeout")
    parser.add_argument("--leave-motor-mode", action="store_true", help="do not switch back to servo mode after stop")
    parser.add_argument("--json", action="store_true", help="print final results as JSON")
    parser.add_argument("--confirm", action="store_true", help="required to move real hardware")
    return parser


def default_speed(fixed_speed_mode: bool, speed: Optional[int]) -> int:
    if speed is not None:
        return speed
    return DEFAULT_FIXED_SPEED if fixed_speed_mode else DEFAULT_DUTY_SPEED


def planned_run_text(args: argparse.Namespace, speed: int) -> str:
    directions = ", ".join(expand_directions(args.direction))
    return (
        f"Planned limit test: com_port={args.com_port}, servo_id={args.servo_id}, "
        f"directions={directions}, speed={speed}, fixed_speed_mode={args.fixed_speed_mode}, "
        f"stable_duration={args.stable_duration}s, stable_tolerance={args.stable_tolerance}, "
        f"max_duration={args.max_duration}s"
    )


def print_summary(results: Iterable[LimitTestResult]) -> None:
    print("")
    print("Limit test results:")
    for result in results:
        status = "TIMEOUT" if result.timed_out else "LIMIT"
        print(
            f"- {result.direction}: {status} position={result.limit_position} "
            f"elapsed={result.elapsed_s:.2f}s stable={result.stable_s:.2f}s "
            f"samples={result.samples}"
        )


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    speed = default_speed(args.fixed_speed_mode, args.speed)

    try:
        directions = expand_directions(args.direction)
        planned = planned_run_text(args, speed)
        print(planned)
        if not args.confirm:
            print("Dry run only. Add --confirm to move the servo.")
            return 0

        results: list[LimitTestResult] = []
        with BusServoController(args.com_port, baudrate=args.baudrate, timeout=args.timeout) as controller:
            tester = ServoLimitTester(
                controller,
                args.servo_id,
                speed=speed,
                fixed_speed_mode=args.fixed_speed_mode,
                stable_duration=args.stable_duration,
                stable_tolerance=args.stable_tolerance,
                poll_interval=args.poll_interval,
                max_duration=args.max_duration,
                return_servo_mode=not args.leave_motor_mode,
            )
            for direction in directions:
                results.append(tester.run_direction(direction))

        print_summary(results)
        if args.json:
            print(json.dumps([result.to_dict() for result in results], indent=2))
        return 0
    except KeyboardInterrupt:
        print("Interrupted by user.")
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
