import os
import signal

from flask import Flask
from flask import Response
from flask import render_template
from flask import request
from flask import jsonify

import struct

from robotModule import robotInit
from robotModule import robotShutdown
from robotModule import robotCmd

app = Flask(__name__)
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0

@app.route("/")
def main():
    return render_template('main.html')

@app.route("/start")
def start():
    app.logger.debug('ROBOT START')
    data = robotInit(app.logger)
    app.logger.debug('ROBOT START OK')
    if data is not None:
        return Response("{'status': '0', 'cmd': 'start', 'cameraWidth': %d, 'cameraHeight': %d}" %
                         (data.CameraWidth, data.CameraHeight), content_type='text/plain; charset=utf-8')
    else:
        return Response("{'status': '-1', 'cmd': 'start'}", content_type='text/plain; charset=utf-8')
    

@app.route("/shutdown")
def shutdown():
    app.logger.debug('SHUTDOWN START')
    robotShutdown()
    app.logger.debug('SHUTDOWN OK')
    shutdown_server = request.environ.get('werkzeug.server.shutdown')
    if shutdown_server is not None:
        shutdown_server()
    else:
        os.kill(os.getpid(), signal.SIGINT)
    return Response("{'status': 'ok'}", content_type='text/plain; charset=utf-8')

@app.route("/command/<cmd>/<arg>")
def command(arg,cmd):
    status = robotCmd(cmd, arg)
    app.logger.debug(f"{cmd}/{arg} status={status}")
    return Response("{'status': '%d', 'cmd': '%s', 'arg': '%s'}" % (status, cmd, arg) , content_type='text/plain; charset=utf-8')

@app.route("/getdepthdump")
def getdepthdump():
    with open(r"/home/pi/git/sprobot/static/depth.jpg.dump", mode='rb') as file:
        fileContent = file.read()
    data = []
    for v in struct.iter_unpack("H", fileContent):
        data.append(v[0])
    return jsonify(data)
        

app.run(host='0.0.0.0', port=8000, debug=True)
