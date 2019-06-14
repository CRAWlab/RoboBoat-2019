#! /usr/bin/env python

###############################################################################
# TD_TCP_statusServer.py
#
# Script to make a TCP connection with the Technical Director's (TD) network for 
# the 2019 International RoboBoat Competition. It should be run as a node on 
# a shore-based computer running ROS on the same network as the TD server. 
# 
# The protocol is based on the NMEA sentences described at:
#   https://robonationforum.vbulletin.net/filedata/fetch?id=2030
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 06/18/16
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   *
#
###############################################################################


# Imports for "pure" TCP communcation with TD server
import socket
import socketserver
import threading

# ROS related imports
import rospy
from std_msgs.msg import String


import time

# Change this address and port to match those provided by the technical direcotrs
TD_IP_ADDRESS = '192.168.0.20'
TD_PORT = 2390


class ThreadedTCPRequestHandler(socketserver.StreamRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        # self.rfile is a file-like object created by the handler;
        # we can now use e.g. readline() instead of raw recv() calls
        self.data = self.rfile.readline().strip()
        print("{} wrote:".format(self.client_address[0]))
        print(self.data)
        # Likewise, self.wfile is a file-like object used to write back
        # to the client
        #self.wfile.write(self.data.upper())

class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    pass


class TD_communication(object):
    """ 
    Class to handled forming and sending of status packets 
    

    
    """

    def __init__(self, ip_address, port):
        self.ip_address = ip_address
        self.port = port
        self.data_packet = None
        
        self.TEAM_ID = "ULLAF"  # Update this to reflect ID given by TDs
        
        
        
    def send_data_and_wait(self, message):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ip, port))

        try:
            sock.sendall(message)
            response = sock.recv(1024)
            print("Received: {}".format(response))

        finally:
            sock.close()
            
    
    def calculate_checksum(self, message):
        """ 
        Calculate the checksum based on the: 

            the bitwise exclusive OR of ASCII codes 
            of all characters between the $ and *
        
        Arguments:
            message: the message to calculate the checksum for
        
        Returns:
            the checksum
        """
        checksum = 0
        
        for c in message:
            checksum ^= ord(c)
        
        return hex(checksum)
        


    def generate_heartbeat_message(self):
        """ 
        Generates a heartbeat message. This message should be sent at 1Hz
        
        The message contains these elements in a comma separated form:
        
          $RDHRB    |  protocol header
          ddmmyy    |  date in EST
          hhmmss    |  24 hour formatted time in EST
          lat       |  Latitude in decimal degrees
          N/S       |  North/South indicator for latitude
          long      |  longitude in decimal degrees
          E/W       |  East/West indicator for longitude
          TEAMID    |  5 character team ID assgined to us
          MODE      |  System mode 1=Remote, 2=Autonomous, 3=E-stop
          *         |  Indicate end of message
          Checksum  |  Bitwise XOR of message between $ and *
          \r\n      |  Carrige returna and linefeed end message
          
          
        Example Message: 
            $RBHRB,101218,161229,21.31198,N,157.88972,W,AUVSI,2*01
        """
        
        
    def generate_dock_message(self):
        """ 
        Generates a Identify the Dock message. This message should be sent 
        during identify the dock challenge, reporting the dock in which the 
        acoustic pinger is located
        
        The message contains these elements in a comma separated form: 
        
          $RDDOK    |  protocol header
          ddmmyy    |  date in EST
          hhmmss    |  24 hour formatted time in EST
          TEAMID    |  5 character team ID assgined to us
          DOCKNUM   |  Number of the dock with the pinger
          *         |  Indicate end of message
          Checksum  |  Bitwise XOR of message between $ and *
          \r\n      |  Carrige returna and linefeed end message
          
        Example Message:
            $RBDOK,101218,161229,AUVSI,2*3E
        """
    
    def generate_flag_message(self):
        """ 
        Generates a Identify Flag message. This message should be sent 
        during identify the flag challenge, reporting the flag number
        
        The message contains these elements in a comma separated form: 
        
          $RDFLG    |  protocol header
          ddmmyy    |  date in EST
          hhmmss    |  24 hour formatted time in EST
          TEAMID    |  5 character team ID assgined to us
          FLAGNUM   |  Number of the flag
          *         |  Indicate end of message
          Checksum  |  Bitwise XOR of message between $ and *
          \r\n      |  Carrige returna and linefeed end message
          
        Example Message:
            $RBFLG,101218,161229,AUVSI,3*32
        """
    
    
if __name__ == "__main__":
    # Depending the setup of your computer and the network, you may need to 
    # change the IP address and port below
    # Port 0 means to select an arbitrary unused port
    LOCAL_IP, LOCAL_PORT = "0.0.0.0", 0

    server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)
    ip, port = server.server_address

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()
    print("Server loop running in thread:", server_thread.name)


    try:
        start_time = time.time()
        while True:
            dt = time.time() - start_time
            signal = 25 * np.sin(0.5 * np.pi * dt) + 25

            data = '{}\r\n'.format(int(signal))
        
            client(TD_IP_ADDRESS, TD_PORT, data.encode('utf-8'))
            print('Sending to {}:{} \t Message: {}'.format(CLIENT_1_ADDRESS, CLIENT_1_PORT, data))
            time.sleep(1)
        
            # client(CLIENT_2_ADDRESS, CLIENT_2_PORT, data.encode('utf-8'))
            # print('Sending: {} to {}:{}'.format(data, CLIENT_2_ADDRESS, CLIENT_2_PORT))
            # time.sleep(0.04)
        
    except (KeyboardInterrupt, SystemExit):
        server.shutdown()
