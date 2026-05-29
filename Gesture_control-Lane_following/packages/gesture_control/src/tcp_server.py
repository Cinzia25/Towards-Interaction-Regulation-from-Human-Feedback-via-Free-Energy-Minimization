#!/usr/bin/env python3
import socket
import rospy
import sys
from std_msgs.msg import String

def tcp_server():
    rospy.init_node('gesture_commander', anonymous=False)
    pub = rospy.Publisher('/gesture_command', String, queue_size=10)

    HOST = ''       # Escucha en todas las interfaces dentro del contenedor
    PORT = 5005

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((HOST, PORT))
            sock.listen(1)
            rospy.loginfo(f"Listening on port {PORT}...")
        except Exception as e:
            rospy.logerr(f"Failed to bind socket: {e}")
            return

        try:
            while not rospy.is_shutdown():
                conn, addr = sock.accept()
                rospy.loginfo(f"Connection from {addr}")
                with conn:
                    while not rospy.is_shutdown():
                        data = conn.recv(1)
                        if not data:
                            break
                        cmd = data.decode('utf-8').strip()
                        #rospy.loginfo(f"Received command: {cmd}")
                        pub.publish(cmd)
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down TCP server (Ctrl+C pressed).")
            sys.exit(0)
        except Exception as e:
            rospy.logwarn(f"Connection error: {e}")

if __name__ == '__main__':
    try:
        tcp_server()
    except rospy.ROSInterruptException:
        pass


