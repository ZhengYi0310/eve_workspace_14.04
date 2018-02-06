# first of all import the socket library
import socket

ip = socket.gethostbyname('bml-ALL-SERIES')
print "The client hostname is %s." %(ip)


# next create a socket object
s_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print "socket on eve successfully created."

# reserve a port on the pc, just set it to 12345
port = 9999

# Next bind to the port
# we have not typed any ip in the ip field
# instead we have inputted an empty string
# this makes the server listen to requests
# coming from other computers on the network
s_server.bind(("eve", port))
print "socket binded to %s." %(port)
s_server.listen(10)


s_client, address = s_server.accept()
print "socket is listening."
print s_client.recv(8192)
s_client.send('Hello Joy !')
print "handshake ! now starting transfer file !"
f = open("test.json", "wb")

l = s_client.recv(8192)
while (l):
    f.write(l)
    l = s_client.recv(1024)
        #print "receiving"
s_client.send('file received by Eve!')
print "file received !"
f.close()
s_client.close()
s_server.close()





