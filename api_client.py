import zmq

context = zmq.Context()

#  Socket to talk to server
print("Connecting to server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.100:5555")

def stand_up():
    send_request(b"stand")

def sit_down():
    send_request(b"sit")

def command_velocity(x, yaw, time):
    send_request(b"move_{}_{}_{}".format(x, yaw, time))


def send_request(request):
    print(f"Sending request {request} …")
    socket.send(request)

    #  Get the reply.
    message = socket.recv()
    print(f"Received reply {request} [ {message} ]")