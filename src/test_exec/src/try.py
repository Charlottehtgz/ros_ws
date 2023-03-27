#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import socket
 
rospy.init_node('topic_publisher')
pub = rospy.Publisher('speech_direction',Int32)
rate = rospy.Rate(2)
 
server = socket.socket()
#print(socket.getaddrinfo("bitbucket.org", 1146))
server.bind(('172.20.10.6', 1160))
#server.bind(('192.168.1.101', 1600))
server.listen(5)
 
exit = ''
while True:
    con, addr = server.accept()
    print('connect_addr: ', addr)
    while con:
        msg = con.recv(1024).decode('utf-8')
        if msg[0:2] == 'ab' and msg[-2:] == 'cd':
    #if msg[0:2] == 'ab':
        #while not rospy.is_shutdown():
            leng=len(msg)
            print(leng)
            posi=leng-2
            pub.publish(int(msg[2:posi]))
            rate.sleep()
 
            # if msg == 'break':
            #     con.close()
            #     exit = 'break'
            #     break
    if exit == 'break':
        break
server.close() 

