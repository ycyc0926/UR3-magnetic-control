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
        self.web.status = json.dumps({'command_error': '相机无新图像，未开始录像'}).encode()
        until(lambda: self.browser.script("return document.getElementById('commanderror').textContent.includes('相机无新图像')"))
        self.assertIn('已提交', self.browser.script("return document.getElementById('feedback').textContent"))
        self.assertEqual(self.browser.script('return window.testErrors'), [])

    def test_http_error_remains_visible_during_status_polling(self):
        for _ in range(32):self.web.commands.put_nowait(('clear', None))
        self.browser.click('clear-trail')
        until(lambda: self.browser.script("return document.getElementById('feedback').textContent.includes('429')"))
        time.sleep(1.1)
        self.assertIn('429', self.browser.script("return document.getElementById('feedback').textContent"))
        self.assertFalse(self.browser.script("return document.getElementById('clear-trail').disabled"))

    def test_reacquisition_progress_and_ambiguity_keep_coordinates_hidden(self):
        for reason, expected in [('reacquiring', '1/3'), ('ambiguous', '等待唯一目标')]:
            self.web.status = json.dumps({
                'detected': False, 'tracking_reason': reason,
                'tracking_diagnostics': {'confirmation_frames': 1, 'reacquire_frames': 3},
            }).encode()
            until(lambda: expected in self.browser.script("return document.getElementById('diagnostic').textContent"))
            self.assertEqual(self.browser.script("return document.getElementById('xy').textContent"), '—')
        self.web.status = json.dumps({
            'detected': True, 'tracking_reason': 'reacquired',
            'utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'world_xy_mm': [123, 45], 'displacement_from_reset_mm': [23, 5],
        }).encode()
        until(lambda: self.browser.script("return document.getElementById('xy').textContent") == '123.00 , 45.00')
        self.assertEqual(self.browser.script('return window.testErrors'), [])


if __name__ == '__main__':
    unittest.main()
