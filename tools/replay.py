# This script replays recorded debugging messages
# It is meant to be used with the testing framework
import struct
import asyncio
from websockets import broadcast, serve
import json


CONNECTIONS = set()
async def register(websocket):
    print("client connected")
    CONNECTIONS.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        print("client disconnected")
        CONNECTIONS.remove(websocket)


async def main():
    async with serve(register, "localhost", 5678):
        file = open("debug_output.raw", "rb")
        while (header := file.read(1)):
            while len(CONNECTIONS) < 1:
                await asyncio.sleep(0.5)
            header = header[0]
            if header == 0:
                length = file.read(1)[0]
                msg = file.read(length).decode("utf-8")
                broadcast(CONNECTIONS, json.dumps({'msgs':[msg]}))
            elif header == 1:
                tdir = struct.unpack("f",file.read(4))[0]
                broadcast(CONNECTIONS, json.dumps({'trot': tdir}))
                await asyncio.sleep(0.016)
            elif header == 2:
                cdir = struct.unpack("f",file.read(4))[0]
                broadcast(CONNECTIONS, json.dumps({'rot': cdir}))
                await asyncio.sleep(0.016)
            elif header == 3:
                battery = struct.unpack("f",file.read(4))[0]
                broadcast(CONNECTIONS, json.dumps({'bat': battery}))
            elif header == 4:
                pos_x = struct.unpack("f",file.read(4))[0]
                pos_y = struct.unpack("f",file.read(4))[0]
                broadcast(CONNECTIONS, json.dumps({'pos': [pos_x,pos_y]}))
                await asyncio.sleep(0.016)

asyncio.run(main())
