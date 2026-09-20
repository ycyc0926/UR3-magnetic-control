"""Isolated browser integration; ephemeral HTTP gates have no robot consumer."""
import os
from pathlib import Path
import sys
import unittest

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / 'ros2_ws/src/ur3_magnetic_control/test'))
import test_h_preview_browser as preview_browser
from explicit_probe_trigger import ExplicitProbeTrigger
from ur3_magnetic_control.h_preview_web import LocalPreview

until = preview_browser.until


@unittest.skipUnless(os.environ.get('H_UI_TEST_MARIONETTE_PORT'), 'isolated Firefox not requested')
class EmbeddedProbeTests(unittest.TestCase):
    setUp = preview_browser.BrowserTests.setUp

    def test_embedded_button_single_click_and_disconnect(self):
        # The normal ephemeral preview never connects to the live arm panel.
        self.assertTrue(self.browser.script("return document.getElementById('arm-waiting-button').disabled"))
        self.assertEqual(self.browser.script("return document.getElementById('arm-panel').getAttribute('src')"), None)
        gate = ExplicitProbeTrigger(port=0)
        gate_closed = False
        try:
            arm_url = f'http://127.0.0.1:{gate.server.server_address[1]}/'
            web = LocalPreview(port=0, arm_panel_url=arm_url)
            self.addCleanup(web.close)
            web.jpeg = self.web.jpeg
            web.status = self.web.status
            url = f'http://127.0.0.1:{web.server.server_address[1]}/#arm-control'
            self.browser.call('WebDriver:Navigate', {'url': url})
            until(lambda: self.browser.script("return document.getElementById('arm-panel').style.display==='block'"))
            self.assertIsNone(gate.start_event)
            self.browser.click('mark-motor-started')
            self.assertEqual(until(lambda: list(web.pop_commands())), [('event', 'motor_started')])
            self.assertIsNone(gate.start_event)

            def enter_panel():
                self.browser.script("document.getElementById('arm-panel').scrollIntoView({block:'start'})")
                element = self.browser.call('WebDriver:FindElement', {'using': 'css selector', 'value': '#arm-panel'})
                self.browser.call('WebDriver:SwitchToFrame', {'element': element['element-6066-11e4-a52e-4f735466cecf']})

            enter_panel()
            until(lambda: self.browser.script("return !document.getElementById('start').disabled"))
            self.browser.script("window.testErrors=[];window.addEventListener('error',e=>window.testErrors.push(e.message));")
            self.browser.click('start')
            try:
                until(lambda: gate.start_event)
            except AssertionError:
                self.fail(str(self.browser.script("return {status:document.getElementById('status').textContent,errors:window.testErrors,disabled:document.getElementById('start').disabled}")))
            stamp = gate.start_event['wall_time_ns']
            self.assertTrue(self.browser.script("return document.getElementById('start').disabled"))
            self.assertEqual(list(web.pop_commands()), [])
            self.browser.call('WebDriver:SwitchToFrame', {'id': None})
            self.browser.call('WebDriver:Navigate', {'url': url})
            until(lambda: self.browser.script("return document.getElementById('arm-panel').style.display==='block'"))
            enter_panel()
            until(lambda: self.browser.script("return document.getElementById('start').disabled"))
            self.assertEqual(gate.start_event['wall_time_ns'], stamp)
            self.browser.call('WebDriver:SwitchToFrame', {'id': None})
            gate.close(); gate_closed = True
            until(lambda: self.browser.script("return document.getElementById('arm-unavailable').style.display==='block'"))
            self.assertTrue(self.browser.script("return document.getElementById('arm-waiting-button').disabled"))
            self.assertEqual(list(web.pop_commands()), [])
        finally:
            if not gate_closed: gate.close()


if __name__ == '__main__':
    unittest.main()
