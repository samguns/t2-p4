import argparse
import socketio
import eventlet
import eventlet.wsgi
from flask import Flask

from pid import PID


sio = socketio.Server()
app = Flask(__name__)
pid = PID()


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    return

@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        #print(data)
        #cte = data["cte"]
        speed = data["speed"]
        angle = data["steering_angle"]
        print(" steer " + angle + " speed " + speed)
        send_control(0, 0.3)
    else:
        sio.emit('manual', data={}, skip_sid=True)

    return


def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': str(steering_angle),
            'throttle': str(throttle)
        },
        skip_sid=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('Kp', type=float)
    parser.add_argument('Ki', type=float)
    parser.add_argument('Kd', type=float)
    args = parser.parse_args()

    pid.Init(args.Kp, args.Ki, args.Kd)
    app = socketio.Middleware(sio, app)
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)