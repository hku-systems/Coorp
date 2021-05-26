import threading
import socket
from bh_controller import MessageStream
import pickle
import queue
import logging
logger = logging.getLogger('BH Controller')
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter('%(asctime)s - %(name)s: %(message)s'))
logger.addHandler(ch)

TIME_SLICE = 5

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

sock.bind(('0.0.0.0', 9999))
sock.listen()

request_queue = queue.Queue()
token = 0
token_lock = threading.Lock()
token_cv = threading.Condition()


def serve_task():
    while True:
        requester = request_queue.get()
       # bookkeeping
        token_lock.acquire()
        global token
        token = int((token+1) % 1e9)  # simply works
        token_lock.release()
        # send token to the requester
        requester.send(pickle.dumps(('permit', token)))
        logger.info(f'token {token} acquired by {requester.sock.getpeername()}')
        # wait for token to be returned
        with token_cv:
            token_returned = token_cv.wait(timeout=TIME_SLICE)
        if token_returned:
            logger.info(f'token {token} returned in time')
        else:
            logger.info(f'token {token} timeout, ignore')
serve_thread = threading.Thread(target=serve_task)
serve_thread.start()


while True:
    client_sock, addr = sock.accept()
    logger.info(f'connection from {addr}')
    msg_stream = MessageStream(client_sock, use_controller=False)
    def f(msg_stream):
        while True:
            msg, tok = pickle.loads(msg_stream.recv())

            if msg == 'acquire':
                request_queue.put(msg_stream)
            elif msg == 'release':
                logger.info(f'token {tok} returned by {msg_stream.sock.getpeername()}')
                # notify if applicable
                token_lock.acquire()
                if token == tok:
                    with token_cv:
                        token_cv.notify()
                token_lock.release()
    thread = threading.Thread(target=f, args=(msg_stream,))
    thread.start()
