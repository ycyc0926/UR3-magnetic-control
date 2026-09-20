"""Real Firefox clicks against an isolated, hardware-free preview server.

Start a SEPARATE headless Firefox profile with --marionette, then set
H_UI_TEST_MARIONETTE_PORT to its loopback port. Never attach to the user's profile.
No selenium dependencies; Mozilla's length-prefixed Marionette protocol is used.
"""
import json
import os
import socket
import time
import unittest

import cv2
import numpy as np

from ur3_magnetic_control.h_preview_web import LocalPreview, PAGE


class FirefoxClient:
    def __init__(self, port):
        self.socket = socket.create_connection(('127.0.0.1', port), timeout=5)
        self.sequence = 0
        self.receive()
        self.call('WebDriver:NewSession', {'pageLoadStrategy': 'eager'})

    def receive(self):
        prefix = b''
        while not prefix.endswith(b':'):
            chunk = self.socket.recv(1)
            if not chunk:raise ConnectionError('Firefox connection closed')
            prefix += chunk
        size = int(prefix[:-1])
        data = b''
        while len(data) < size:
            chunk = self.socket.recv(size-len(data))
            if not chunk:raise ConnectionError('Firefox response interrupted')
            data += chunk
        return json.loads(data)

    def call(self, method, parameters):
        self.sequence += 1
        data = json.dumps([0, self.sequence, method, parameters]).encode()
        self.socket.sendall(str(len(data)).encode()+b':'+data)
        response = self.receive()
        if response[1] != self.sequence or response[2]:
            raise RuntimeError(response)
        result = response[3]
        return result.get('value', result) if isinstance(result, dict) else result

    def script(self, source, args=None):
        return self.call('WebDriver:ExecuteScript', {
            'script': source, 'args': args or [], 'newSandbox': False, 'sandbox': None})

    def click(self, element_id):
        element = self.call('WebDriver:FindElement', {'using': 'css selector', 'value': '#'+element_id})
        self.call('WebDriver:ElementClick', {'id': element['element-6066-11e4-a52e-4f735466cecf']})

    def close(self):
        try:self.call('WebDriver:DeleteSession', {})
        finally:self.socket.close()


def until(function, seconds=4):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        result = function()
        if result:return result
        time.sleep(.05)
    raise AssertionError('Timed out waiting for browser/server condition')


@unittest.skipUnless(os.environ.get('H_UI_TEST_MARIONETTE_PORT'), 'isolated Firefox not requested')
class BrowserTests(unittest.TestCase):
    def setUp(self):
        self.web = LocalPreview(port=0)
        self.addCleanup(self.web.close)
        self.web.jpeg = cv2.imencode('.jpg', np.full((1024, 1224, 3), 220, np.uint8))[1].tobytes()
        self.web.status = json.dumps({'detected': False, 'recording': {'active': False, 'frames': 0}}).encode()
        self.browser = FirefoxClient(int(os.environ['H_UI_TEST_MARIONETTE_PORT']))
        self.addCleanup(self.browser.close)
        url = f'http://127.0.0.1:{self.web.server.server_address[1]}/'
        self.browser.call('WebDriver:Navigate', {'url': url})
        until(lambda: self.browser.script("return !!document.getElementById('start-recording')"))
        self.browser.script("window.testErrors=[];window.addEventListener('error',e=>window.testErrors.push(e.message));")

    def test_each_button_reaches_backend_with_expected_payload(self):
        expected = [
            ('reset-target', ('reset', None)), ('clear-trail', ('clear', None)),
            ('start-recording', ('record_start', 120)), ('stop-recording', ('record_stop', None)),
            ('mark-motor-started', ('event', 'motor_started')),
            ('mark-motor-stopped', ('event', 'motor_stopped')),
            ('path-negative-y', ('path', ('line_negative_y', 10))),
            ('path-positive-y', ('path', ('line', 10))),
            ('path-square', ('path', ('square', 24))), ('hide-path', ('path', ('clear', 20))),
        ]
        for element_id, command in expected:
            self.browser.click(element_id)
            self.assertEqual(until(lambda: list(self.web.pop_commands())), [command])
            until(lambda: self.browser.script("return document.getElementById('feedback').textContent.startsWith('已提交')"))
        self.assertEqual(self.browser.script('return window.testErrors'), [])
        self.assertNotIn('onclick=', PAGE)

    def test_click_image_still_selects_and_feedback_survives_poll(self):
        self.browser.click('view')
        received = until(lambda: list(self.web.pop_commands()))
        self.assertEqual(received[0][0], 'select')
        self.assertTrue(0 <= received[0][1][0] < 1224)
        self.assertTrue(0 <= received[0][1][1] < 1024)
        until(lambda: self.browser.script("return document.getElementById('feedback').textContent.startsWith('已提交')"))
        self.web.status = json.dumps({'command_error': '请先锁定静止的 H，再设置参考路径'}).encode()
        until(lambda: self.browser.script("return document.getElementById('commanderror').textContent.includes('请先锁定')"))
        self.assertIn('已提交', self.browser.script("return document.getElementById('feedback').textContent"))
        self.assertEqual(self.browser.script('return window.testErrors'), [])

    def test_http_error_remains_visible_during_status_polling(self):
        for _ in range(32):self.web.commands.put_nowait(('clear', None))
        self.browser.click('clear-trail')
        until(lambda: self.browser.script("return document.getElementById('feedback').textContent.includes('429')"))
        time.sleep(1.1)
        self.assertIn('429', self.browser.script("return document.getElementById('feedback').textContent"))
        self.assertFalse(self.browser.script("return document.getElementById('clear-trail').disabled"))


if __name__ == '__main__':
    unittest.main()
