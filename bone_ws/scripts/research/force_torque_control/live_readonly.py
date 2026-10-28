"""Bounded real-sensor source; real actuator register writes are forbidden."""
import os
import subprocess
import threading
import time
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from pus6_motor_driver.uart_bus import UartBus
from pus6_motor_driver.protocol import parse_frame, CMD_RD_STATUS
from pus6_research_msgs.msg import RawMotorStatus
from status_bridge import ReadOnlyStatusSource
from status_node import StatusSourcePublisher

BASE='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00'
PLATFORM='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00'

def check_free(path):
    if not os.path.exists(path): raise RuntimeError('missing device: '+path)
    result=subprocess.run(['fuser','-s',path],capture_output=True)
    if result.returncode==0: raise RuntimeError('device already in use: '+path)
    if result.returncode!=1: raise RuntimeError('unable to determine device ownership: '+path)

class StatusOnlyBus:
    def __init__(self,bus): self.bus=bus
    def request(self,frame,**kwargs):
        if parse_frame(frame).command != CMD_RD_STATUS: raise ValueError('only status reads permitted')
        return self.bus.request(frame,**kwargs)

class LiveReadonly:
    def __init__(self,node,log_dir,port='/dev/ttyUSB0'):
        self.node=node; self.log_dir=Path(log_dir); self.port=port
        self.bus=None; self.thread=None; self.closed=threading.Event(); self.error=None
        self.children=[]; self.streams=[]; self.samples=0
    def start(self):
        for path in (self.port,BASE,PLATFORM): check_free(path)
        import serial
        ser=serial.Serial(port=self.port,baudrate=921600,timeout=.05,write_timeout=.05,exclusive=True)
        try: self.bus=UartBus(serial_instance=ser,timeout=.05,retries=0,read_only=True)
        except Exception: ser.close(); raise
        pub=StatusSourcePublisher(self.node,ReadOnlyStatusSource(StatusOnlyBus(self.bus)),RawMotorStatus.SOURCE_HARDWARE)
        binary=Path(get_package_prefix('h30_imu'))/'lib/h30_imu/h30_imu_node'
        for role,path in (('base',BASE),('platform',PLATFORM)):
            stream=(self.log_dir/('h30_'+role+'.log')).open('x'); self.streams.append(stream)
            cmd=[str(binary),'--ros-args','-r','__node:=h30_readonly_'+role,
                 '-p','port:='+path,'-p','imu_topic:=/research_imu_lengths/'+role+'_imu',
                 '-p','frame_id:=readonly_'+role+'_imu','-p','publish_euler:=false']
            self.children.append(subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT))
        self.thread=threading.Thread(target=self.poll,args=(pub,),daemon=True); self.thread.start()
    def poll(self,pub):
        try:
            while not self.closed.is_set():
                begin=time.monotonic()
                for axis in range(1,7):
                    if self.closed.is_set(): return
                    pub.poll_and_publish(axis); self.samples+=1
                self.closed.wait(max(0,.05-(time.monotonic()-begin)))
        except Exception as exc: self.error=f'{type(exc).__name__}: {exc}'
    def close(self):
        self.closed.set()
        if self.thread: self.thread.join(timeout=2.)
        if self.thread and self.thread.is_alive(): raise RuntimeError('status polling did not stop')
        for child in self.children:
            if child.poll() is None: child.terminate()
        for child in self.children:
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired: child.kill(); child.wait()
        for stream in self.streams: stream.close()
        audit={'source':'hardware_status_reads','samples':self.samples,'register_writes':len(self.bus.write_audit) if self.bus else 0,
               'tx_command_counts':self.bus.tx_cmd_counts if self.bus else {},'error':self.error}
        if self.bus: self.bus.close()
        return audit
