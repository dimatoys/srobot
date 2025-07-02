import sys

from flask import Flask
from flask import Response
from flask import render_template
from flask import request

import json

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
    params = robotInit(app.logger)
    app.logger.debug('ROBOT START OK')
    return Response(json.dumps(params), content_type='text/plain; charset=utf-8')

@app.route("/shutdown")
def shutdown():
    app.logger.debug('SHUTDOWN START')
    robotShutdown()
    app.logger.debug('SHUTDOWN OK')
    shutdown_server = request.environ.get('werkzeug.server.shutdown')
    if shutdown_server is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    shutdown_server()
    return Response("{'status': 'ok'}", content_type='text/plain; charset=utf-8')

@app.route("/command/<cmd>/<arg>")
def command(arg,cmd):
    status = robotCmd(cmd, arg)
    return Response("{'status': '%d'}" % status , content_type='text/plain; charset=utf-8')

app.run(host='0.0.0.0', port=8000, debug=True)
