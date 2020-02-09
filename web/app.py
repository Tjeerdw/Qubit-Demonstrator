#!/usr/bin/python3
from flask import Flask, render_template, send_file
import serial,sys
import numpy as np
import time
app = Flask(__name__)

OPT_TURNS = {1 : 3, 2 : 7, 3 : 3, 4 : 5}

class Orb:
    rotations = {
        'x' : ([1,0,0], np.pi),
        'y' : ([0,1,0], np.pi),
        'z' : ([0,0,1], np.pi),
        'h' : ([1/np.sqrt(2), 0, 1/np.sqrt(2)], np.pi),
        's' : ([0,0,1], .5*np.pi),
        'S' : ([0,0,1], -.5*np.pi),
        't' : ([0,0,1], .25*np.pi),
        'T' : ([0,0,1], -.25*np.pi)
    }

    def __init__(self, comm = True):
        self.comm = comm
        self.state = np.array([0,0,1])
        self.goal = None
        if self.comm:
            try:
                self.ser = serial.Serial('/dev/ttyUSB0', 115200)
            except serial.serialutil.SerialException:
                print("Could not connect to the orb.", file = sys.stderr)
                print("Connect the orb to /dev/ttyUSB0 and restart the server.", file = sys.stderr)
                print("Run as ./app.py test if you want to run without communicating with the orb.", file = sys.stderr)
                exit()
        self.update_status()

    def rotate(self, n, theta):
        assert abs(np.sum(np.square(np.array(n))) - 1) < 1e-6
        matrix = np.array([
            [
                np.cos(theta) + n[0]**2 * (1-np.cos(theta)),
                n[0]*n[1]*(1-np.cos(theta)) - n[2]*np.sin(theta),
                n[0]*n[2]*(1-np.cos(theta)) + n[1]*np.sin(theta)
            ],[
                n[0]*n[1]*(1-np.cos(theta)) + n[2]*np.sin(theta),
                np.cos(theta) + n[1]**2 * (1-np.cos(theta)),
                n[1]*n[2]*(1-np.cos(theta)) - n[0]*np.sin(theta)
            ],[
                n[0]*n[2]*(1-np.cos(theta)) - n[1]*np.sin(theta),
                n[1]*n[2]*(1-np.cos(theta)) + n[0]*np.sin(theta),
                np.cos(theta) + n[2]**2 * (1-np.cos(theta))
            ]
        ])
        self.state = matrix @ self.state

    def communicate(self, charcode, blocking = False, extra_data = None):
        if self.comm:
            self.ser.write(charcode.encode('ascii'))
            if extra_data is not None:
                self.ser.write(bytes(extra_data))
            if blocking:
                while self.ser.read().decode('ascii') != 'd':
                    pass
        else:
            time.sleep(1)

    def set_goal(self,x,y,z):
        assert abs(np.sqrt(x*x + y*y + z*z) - 1) <= 1e-6
        self.goal = [x,y,z]

    def dist_to_goal(self):
        if self.goal == None: return 0
        return 2 * (1 - np.sum(self.state * self.goal))

    def get_status(self):
        return self.dist_to_goal() < 1e-4

    def update_status(self):
        if self.get_status():
            if self.comm:
                self.communicate('g')
        else:
            if self.comm:
                self.communicate('r')

    def perform_rotation(self, n, theta):
        charcodes = list(map(lambda e : int((e+1)*127), [*n, theta/np.pi]))
        self.rotate(n,theta)
        self.communicate('u', blocking = True, extra_data = charcodes)
        self.update_status()

    def perform_operation(self, charcode):
        assert charcode in Orb.rotations
        n,theta = Orb.rotations[charcode]
        self.rotate(n,theta)
        self.communicate(charcode, blocking = True)
        self.update_status()

    def reset(self):
        self.state = np.array([0,0,1])
        self.update_status()

    def __str__(self):
        d = self.dist_to_goal()
        s = []
        s.append("This is the current status of the orb:")
        s.append(f"Position: [{self.state[0]:.3f}, {self.state[1]:.3f}, {self.state[2]:.3f}]")
        if self.goal == None: s.append("Goal: not set.")
        else: s.append(f"Goal: [{self.goal[0]:.3f}, {self.goal[1]:.3f}, {self.goal[2]:.3f}]")
        s.append(f"Distance to goal: {d:.3f} ==> {'green' if d < 1e-4 else 'red'}")
        return '\n'.join(s)

@app.route('/')
def splash():
    return render_template('splash.html')

@app.route('/<string:lang>/')
def main(lang):
    return render_template(f'{lang}/main.html')

@app.route('/favicon.ico')
def favicon():
    return send_file('favicon.ico')

@app.route('/<string:lang>/interface')
def interface(lang):
    return render_template(f'{lang}/interface.html')

@app.route('/<string:lang>/problem/<id>')
def problem(lang, id):
    return render_template(f'{lang}/problem{id}.html')

@app.route('/<string:lang>/congratulations/<int:id>/<int:num_turns>')
def congratz(lang, id, num_turns):
    return render_template(f'{lang}/congratulations.html', id = id, num_turns = num_turns, opt_turns = OPT_TURNS[id])

@app.route('/set_goal/<string:x>/<string:y>/<string:z>')
def set_goal(x,y,z):
    orb.set_goal(float(x), float(y), float(z))
    print(orb)
    return ''

@app.route('/reset')
def reset():
    orb.reset()
    print(orb)
    return ''

@app.route('/status')
def status():
    status = orb.get_status()
    return str(status)

@app.route('/operation/<param>')
def perform_operation(param):
    orb.perform_operation(param)
    print(orb)
    return ''

@app.route('/rotation/<string:x>/<string:y>/<string:z>/<string:theta>')
def perform_rotation(x,y,z,theta):
    orb.perform_rotation([float(x), float(y), float(z)], float(theta)*np.pi)
    print(orb)
    return ''

if __name__ == '__main__':
    if len(sys.argv) == 1:
        orb = Orb(comm = True)
    else:
        orb = Orb(comm = False)
    app.run(port = 5001)
