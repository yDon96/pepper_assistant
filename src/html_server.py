#!/usr/bin/env python3
from selenium import webdriver
from ros_pepper_pkg.srv import *
import rospy
import socket
import os
import yaml

class HtmlNode:

    def __init__(self, http_server):
        self.browser = webdriver.Firefox()
        self.http_server = http_server   
        self.show_web_page()     

    def show_web_page(self):
        self.browser.get(self.http_server)


    def reload_web_page(self,msg):
        self.browser.refresh()
        return "ACK"
    
    def start(self, service_name):
        rospy.init_node(service_name)
        rospy.Service(service_name, Html, self.reload_web_page)

        rospy.spin()

if __name__ == "__main__":

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    localhost = socket.gethostbyname(socket.gethostname())
    http = 'http://'+localhost+':8000'

    service_name = config['nodes']['htmlServer']

    try:
        htmlnode = HtmlNode(http)
        htmlnode.start(service_name)
    except rospy.ROSInterruptException:
        pass