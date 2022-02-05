#!/usr/bin/env python
from naoqi import ALProxy
from optparse import OptionParser
from ros_pepper_pkg.srv import *
import rospy
import socket

class TabletNode:

    def __init__(self, ip, port, http_server):
        self.ip = ip
        self.port = port
        self.http_server = http_server
        self.tablet = ALProxy("ALTabletService", ip, port)
        self.show_web_page()     
        

    def show_web_page(self):
        try:
            self.tablet.loadUrl(self.http_server)
            self.tablet.showWebview()
        except:
            self.tablet = ALProxy("ALTabletService", self.ip, self.port)
            self.tablet.loadUrl(self.http_server)
            self.tablet.showWebview()

    def reload_web_page(self, msg):
        self.tablet.reloadPage(True)
        return "ACK"
    
    def start(self,service_name):
        rospy.init_node(service_name)
        rospy.Service(service_name, Html, self.reload_web_page)

        rospy.spin()

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.230")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    localhost = socket.gethostbyname(socket.gethostname())
    http = 'http://'+localhost+':8000'

    try:
        ttsnode = TabletNode(options.ip, int(options.port), http)
        ttsnode.start("tablet_server")
    except rospy.ROSInterruptException:
        pass