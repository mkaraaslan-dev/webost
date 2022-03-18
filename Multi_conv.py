from multiprocessing import Process
import cv2
import io
import socket
import struct
# import time
import pickle
import numpy as np
from PIL import Image
from  threading import Thread



class pro:
    def __init__(self):
        self.framae = 1
        
    def func2(self):
        HOST=''
        PORT=4000
        global frame
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        print('Socket created')
        
        s.bind((HOST,PORT))
        print('Socket bind complete')
        s.listen(10)
        print('Socket now listening')
        
        conn,addr=s.accept()
        
        data = b""
        payload_size = struct.calcsize(">L")
        # print("payload_size: {}".format(payload_size))
        while True:
            while len(data) < payload_size:
                # print("Recv: {}".format(len(data)))
                # data += conn.recv(1228800)
                data += conn.recv(1330425)
        
            # print("Done Recv: {}".format(len(data)))
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack(">L", packed_msg_size)[0]
            # print("msg_size: {}".format(msg_size))
            while len(data) < msg_size:
                # data += conn.recv(1228800)
                data += conn.recv(1330425)
            frame_data = data[:msg_size]
            data = data[msg_size:]
        
            frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
            # frame = np.fromstring(frame.getvalue(), dtype='uint8')
            # print(type(frame)) 
            
            # frame = frame.astype(np.uint8)
            # frame = Image.frombytes("L", (3, 2), frame)
            #frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            # stream = StringIO(frame)
    
            # image = Image.open(stream).convert("RGBA")  
            # print(frame)
            # stream.close()
    
            
    def convert_show(self):
        
            while True:
                print("a")
                try:
                    img = Image.frombytes("RGBA", (640, 480), frame)
                    # stream = BytesIO(bytes(frame))
                    img = np.array(img)
                    # image = Image.open(stream).convert("RGBA")
                    # stream.close()
                    # image.show()
                    cv2.imshow('ImageWindow',img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        cv2.destroyAllWindows()
                        break
                except:
                    pass
    
    def starup(self):
        
        p1 = Thread(target=self.func2)
        p1.start()
        p2 = Thread(target=self.convert_show)
        p2.start()
        p1.join() # Process1'in tamamlanmasını beklemek için kullanılır
        p2.join()

    
if __name__ == '__main__':
        cla=pro()
        cla.starup()