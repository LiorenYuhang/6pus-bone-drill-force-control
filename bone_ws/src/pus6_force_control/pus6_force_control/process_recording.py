"""Bounded asynchronous experiment records; never performs hardware operations."""
import json
import queue
import threading
from datetime import datetime
import time

from .six_axis_tare_experiment import SerialTrace


class AsyncRecords:
    def __init__(self, directory, capacity=8192, transform=None):
        self.directory=directory;self.queue=queue.Queue(capacity);self.transform=transform
        self.dropped=0;self.error=None
        self.thread=threading.Thread(target=self.work,name='experiment-recording',daemon=True)
        self.thread.start()

    def submit(self,name,row):
        try:self.queue.put_nowait((name,row))
        except queue.Full:self.dropped+=1

    def work(self):
        files={}
        try:
            while True:
                item=self.queue.get()
                if item is None:break
                name,row=item
                if self.transform is not None:row=self.transform(name,row)
                if name not in files:files[name]=(self.directory/name).open('a',buffering=65536)
                files[name].write(json.dumps(row)+'\n')
        except Exception as exc:self.error=f'{type(exc).__name__}: {exc}'
        finally:
            for stream in files.values():stream.close()

    def close(self):
        # Only terminal cleanup may wait for storage; UART has already stopped.
        if self.thread.is_alive():
            try:self.queue.put(None,timeout=1.)
            except queue.Full:return dict(dropped=self.dropped,error='recording queue did not drain',closed=False)
        self.thread.join(timeout=10.)
        return dict(dropped=self.dropped,error=self.error,closed=not self.thread.is_alive())


class AsyncSerialTrace(SerialTrace):
    def __init__(self,serial,directory,records):
        super().__init__(serial,directory)
        object.__setattr__(self,'records',records)
        object.__setattr__(self,'write_count',0)

    def record(self,direction,raw):
        self.records.submit('serial_bytes.jsonl',dict(time_iso=datetime.now().astimezone().isoformat(),
            ns=time.monotonic_ns(),direction=direction,raw_hex=bytes(raw).hex(' ')))

    def write(self,data):
        try:return super().write(data)
        finally:
            if self.write_commands:
                object.__setattr__(self,'write_count',self.write_count+len(self.write_commands))
                self.write_commands.clear()

    def read(self,size=1):
        data=self._serial.read(size)
        if data:
            self.record('RX',data)
            for frame in self._parser.feed(data):
                self.records.submit('rx_frames.jsonl',dict(ns=time.monotonic_ns(),motor_id=frame.motor_id,
                    command=frame.command,register=frame.register_addr,raw_hex=frame.raw.hex(' ')))
        return data
