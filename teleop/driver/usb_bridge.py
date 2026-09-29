"""O10 LEFT USB bridge. Bind localhost; access from Mac through SSH forwarding.
Starts read-only unless --allow-motion is explicitly supplied. No reset/calibration.
"""
import argparse
import json
import math
import secrets
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path


def limited_target(target, actual, limits, step):
    if len(target) != 10 or len(actual) != 10:
        raise ValueError('Expected 10 active joints')
    if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in target + actual):
        raise ValueError('Invalid joint value')
    if not all(lo - .05 <= q <= hi + .05 for q, (lo, hi) in zip(actual, limits)):
        raise ValueError('Readback outside model limits; check model / SDK convention')
    bad = [(i, q, lo, hi) for i, (q, (lo, hi)) in enumerate(zip(target, limits)) if not lo-1e-6 <= q <= hi+1e-6]
    if bad:
        raise ValueError('Target outside model limits: ' + repr(bad))
    target = [max(lo, min(hi, q)) for q, (lo, hi) in zip(target, limits)]
    return [max(lo, min(hi, q + max(-step, min(step, t-q))))
            for t, q, (lo, hi) in zip(target, actual, limits)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='/dev/ttyACM0')
    p.add_argument('--http-port', type=int, default=8890)
    p.add_argument('--allow-motion', action='store_true')
    p.add_argument('--speed-limit', type=float, default=.3, help='rad/s; 0 disables slew limit, preserves 0.05 rad tracking envelope')
    p.add_argument('--max-lead', type=float, default=.05, help='Maximum target-readback gap in radians; 0 sends full valid targets')
    args = p.parse_args()
    if not math.isfinite(args.max_lead) or args.max_lead < 0:
        p.error('max-lead must be finite and nonnegative')
    if not math.isfinite(args.speed_limit) or args.speed_limit < 0:
        p.error('speed-limit must be finite and nonnegative')
    from omnihand import OmniHand2025, HandType
    cfg = json.loads(Path(__file__).with_name('left_limits.json').read_text())
    hand = OmniHand2025.create_hand_by_usb(hand_type=HandType.LEFT,
        hand_device_id=OmniHand2025.kDefaultHandDeviceId, uart_port=args.port, baudrate=460800)
    hand.set_request_interval(0)
    hand.set_frame_recv_timeout(100)
    if not hand.init():
        raise RuntimeError('O10 init failed; check USB passthrough and power')
    info = hand.get_vendor_info()
    if info.dof != 10:
        raise RuntimeError('Expected O10 with 10 active joints')
    print('O10 LEFT initialized; allow_motion=' + str(args.allow_motion), flush=True)
    state = {'nonce': None, 'issued': 0., 'last_write': 0., 'command': None}

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, data):
            raw = json.dumps(data, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path != '/state':
                self.reply(404, {'error': 'not found'}); return
            try:
                errors = hand.get_all_error_reports()
                if len(errors) != 10 or any(any(getattr(e, k) for k in ('stalled','overheat','over_current','motor_except','commu_except')) for e in errors):
                    raise ValueError('Motor fault or missing fault report')
                actual = list(hand.get_all_active_joint_angles())
                # Measured encoder values may slightly exceed nominal model bounds.
                checked_target = [max(lo, min(hi, q)) for q, (lo, hi) in zip(actual, cfg['limits_rad'])]
                limited_target(checked_target, actual, cfg['limits_rad'], 0)
                state.update(nonce=secrets.token_hex(16), issued=time.monotonic())
                self.reply(200, dict(hand='left', usb_port=args.port, units='rad', angles=actual,
                    joint_names=cfg['joint_names'], allow_motion=args.allow_motion,
                    nonce=state['nonce'], speed_limit=args.speed_limit, max_lead=args.max_lead))
            except Exception as e:
                state['nonce'] = None
                self.reply(503, {'error': str(e)})

        def do_POST(self):
            try:
                if self.path != '/target':
                    raise ValueError('Unknown route')
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 4096:
                    raise ValueError('Invalid payload size')
                data = json.loads(self.rfile.read(size))
                now = time.monotonic()
                nonce, state['nonce'] = state['nonce'], None
                if not nonce or data.get('nonce') != nonce or now-state['issued'] > .25:
                    raise ValueError('Expired or reused command')
                if not args.allow_motion or data.get('enabled') is not True:
                    raise ValueError('Motion disabled')
                if data.get('hand') != 'left' or data.get('joint_names') != cfg['joint_names']:
                    raise ValueError('Hand / joint order mismatch')
                actual = list(hand.get_all_active_joint_angles())
                dt = min(.05, max(0., now-state['last_write']))
                # Limit both motion away from readback and change from previous setpoint.
                lead = args.max_lead if args.max_lead > 0 else float('inf')
                if args.speed_limit > 0 and (state['command'] is None or now-state['last_write'] >= .25):
                    lead = min(lead, args.speed_limit * dt)
                command = limited_target(data['angles'], actual, cfg['limits_rad'], lead)
                if args.speed_limit > 0 and state['command'] is not None and now-state['last_write'] < .25:
                    command = limited_target(command, state['command'], cfg['limits_rad'], args.speed_limit * dt)
                if time.monotonic()-state['issued'] > .25:
                    raise ValueError('Readback too slow; target expired')
                hand.set_all_active_joint_angles(command)
                state.update(last_write=now, command=command)
                self.reply(200, {'sent': command, 'measured': actual})
            except Exception as e:
                self.reply(400, {'error': str(e)})

        def log_message(self, *args):
            pass

        def setup(self):
            super().setup()
            self.connection.settimeout(.5)

    # No background trajectory queue: absent valid requests means no further writes.
    server = HTTPServer(('127.0.0.1', args.http_port), Handler)
    try:
        server.serve_forever()
    finally:
        server.server_close()

if __name__ == '__main__':
    main()
