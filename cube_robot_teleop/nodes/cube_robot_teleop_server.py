#!/usr/bin/env python

from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
import json
import cgi
import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)


class Server(BaseHTTPRequestHandler):
    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()

    def do_HEAD(self):
        self._set_headers()

    # GET sends back a Hello world message
    def do_GET(self):
        self._set_headers()
        self.wfile.write(json.dumps({'hello': 'world', 'received': 'ok'}))

    # POST echoes the message adding a JSON field
    def do_POST(self):
        ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))

        # refuse to receive non-json content
        if ctype != 'application/json':
            self.send_response(400)
            self.end_headers()
            return

        # read the message and convert it into a python dictionary
        length = int(self.headers.getheader('content-length'))
        message = json.loads(self.rfile.read(length))
        print(message)
        twist = Twist()
        twist.linear.x = message['lin']
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = message['ang']
        pub.publish(twist)
        print vels(twist.linear.x, twist.angular.z)
        # add a property to the object, just to mess with data
        message['received'] = 'ok'

        # send the message back
        self._set_headers()
        self.wfile.write(json.dumps(message))


def run(server_class=HTTPServer, handler_class=Server, port=8008):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)

    print 'Starting httpd on port %d...' % port
    httpd.serve_forever()


if __name__ == "__main__":
    from sys import argv

    rospy.init_node('cube_robot_teleop_server')

    if len(argv) == 2:
        run(port=int(argv[1]))
    else:
        run()
