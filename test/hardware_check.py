#!/usr/bin/env python3
"""Explicit hardware integration check. Does not store camera frames.
Requires source ROS environment; starts and owns one node in caller's domain.
"""
import json
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

root = Path(__file__).resolve().parents[1]
rclpy.init()
client = rclpy.create_node('endoscope_hardware_check')
want_raw = os.environ.get("ENDOSCOPE_TEST_RAW", "1") == "1"
counts = dict(raw=0, compressed=0, info=0)
capture_rates = []
first_last = {}

def receive(kind, message):
    if kind == 'raw':
        assert (message.width, message.height, message.encoding) == (1920, 1080, 'bgr8')
        assert len(message.data) == 1920 * 1080 * 3
    elif kind == 'compressed':
        assert bytes(message.data[:2]) == b'\xff\xd8'
    else:
        assert (message.width, message.height) == (1920, 1080)
    now = time.monotonic()
    first_last.setdefault(kind, [now, now])[1] = now
    counts[kind] += 1

subscriptions = [client.create_subscription(t, '/endoscope/' + topic, lambda m,k=k: receive(k,m), qos_profile_sensor_data)
                 for t,topic,k in [(Image,'image_raw','raw'),(CompressedImage,'image_raw/compressed','compressed'),(CameraInfo,'camera_info','info')] if want_raw or k != 'raw']

from diagnostic_msgs.msg import DiagnosticArray

def diagnostic(message):
    for status in message.status:
        for item in status.values:
            if item.key == 'capture_fps': capture_rates.append(float(item.value))
subscriptions.append(client.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, 10))

def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(client,timeout_sec=0.05)

def call(service, typ, request):
    c=client.create_client(typ,service)
    assert c.wait_for_service(timeout_sec=10), service
    f=c.call_async(request)
    rclpy.spin_until_future_complete(client,f,timeout_sec=10)
    assert f.done() and f.result() is not None, service
    return f.result()

def transition(identifier):
    req=ChangeState.Request();req.transition.id=identifier
    assert call('/endoscope/change_state',ChangeState,req).success

def set_param(name, value):
    req=SetParameters.Request()
    pv=ParameterValue()
    if isinstance(value,bool):pv.type=ParameterType.PARAMETER_BOOL;pv.bool_value=value
    elif isinstance(value,str):pv.type=ParameterType.PARAMETER_STRING;pv.string_value=value
    else:pv.type=ParameterType.PARAMETER_INTEGER;pv.integer_value=value
    req.parameters=[Parameter(name=name,value=pv)]
    return call('/endoscope/set_parameters',SetParameters,req).results[0]

log = Path('/tmp/endoscope_hardware_check.log')
report = {}
with log.open('w') as out:
    if os.environ.get('ENDOSCOPE_TEST_LAUNCH') == '1':
        command = ['ros2', 'launch', 'endoscope_device', 'endoscope.launch.py',
                   'config:=' + str(root / 'config' / 'endoscope.yaml')]
    else:
        command = ['ros2','run','endoscope_device','endoscope_device_node','--ros-args','--params-file',str(root/'config/endoscope.yaml')]
    proc=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        spin(8)
        assert call('/endoscope/get_state',GetState,GetState.Request()).current_state.id==3, log.read_text()
        if os.environ.get('ENDOSCOPE_TEST_MANUAL') == '1':
            assert set_param('controls.auto_exposure',False).successful
            assert set_param('controls.exposure_time_us',10000).successful
            spin(2)
        counts.update(raw=0,compressed=0,info=0);first_last.clear();capture_rates.clear()
        spin(float(os.environ.get('ENDOSCOPE_TEST_SECONDS','120')))
        report['counts']=dict(counts)
        report['capture_fps_mean']=sum(capture_rates)/len(capture_rates) if capture_rates else None
        report['fps']={k:(counts[k]-1)/(v[1]-v[0]) for k,v in first_last.items()}
        assert (not want_raw or counts['raw']>20) and counts['compressed']>20, report
        assert not set_param('controls.brightness',999).successful
        assert not set_param('controls.brightness',2**32+128).successful
        assert not set_param('width',640).successful
        assert set_param('controls.auto_exposure',False).successful
        assert set_param('controls.exposure_time_us',10000).successful
        spin(2)
        assert not set_param('controls.exposure_time_us',60000).successful
        assert set_param('controls.auto_exposure',True).successful
        assert set_param('controls.auto_white_balance',False).successful
        assert set_param('controls.white_balance_temperature',4600).successful
        assert set_param('controls.auto_white_balance',True).successful
        transition(4);spin(1)
        before=dict(counts);spin(2);assert counts==before
        transition(3);spin(3);assert counts['compressed']>before['compressed']
        report['parameter_and_lifecycle_checks']='passed'
        transition(4);transition(2)
        assert set_param('device','/dev/endoscope_missing_test').successful
        req=ChangeState.Request();req.transition.id=1
        assert not call('/endoscope/change_state',ChangeState,req).success
        assert call('/endoscope/get_state',GetState,GetState.Request()).current_state.id==1
        assert set_param('device','/dev/v4l/by-id/usb-DP-251024-J_HD_WebCam-video-index0').successful
        transition(1);transition(3);spin(2)
        report['missing_device_and_reconfigure']='passed'
    finally:
        # Restore default automatic modes when the node is still responding.
        try:
            set_param('controls.auto_exposure',True)
            set_param('controls.auto_white_balance',True)
        except Exception:
            pass
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        client.destroy_node();rclpy.shutdown()
Path('/tmp/endoscope_last_result.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
