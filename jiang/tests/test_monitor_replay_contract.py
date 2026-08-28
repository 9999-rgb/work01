"""Behavioral Web contracts for recording, playback and task replay."""

from __future__ import annotations

import re
import shutil
import subprocess
import textwrap
import unittest
from pathlib import Path


MONITOR_PATH = Path(__file__).resolve().parents[1] / "app" / "monitor.html"


def _inline_script() -> str:
    html = MONITOR_PATH.read_text(encoding="utf-8")
    scripts = re.findall(r"<script>(.*?)</script>", html, flags=re.DOTALL)
    if not scripts:
        raise AssertionError("monitor.html does not contain an inline script")
    return scripts[-1]


def _source_block(start_marker: str, end_marker: str) -> str:
    source = _inline_script()
    start = source.index(start_marker)
    end = source.index(end_marker, start)
    return source[start:end]


@unittest.skipUnless(shutil.which("node"), "Node.js is required for Web tests")
class MonitorReplayContractTest(unittest.TestCase):
    maxDiff = None

    def run_node(self, source: str) -> None:
        result = subprocess.run(
            ["node", "-"],
            input=source,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(
            0,
            result.returncode,
            msg=(
                "Node test failed:\n"
                f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
            ),
        )

    def test_recording_and_replay_request_bodies(self) -> None:
        actions = _source_block(
            "async function startRecording()",
            "\n// ─── Autonomous task control",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            const elements = {{
              recordingName: {{value: '  press_5n_regression  '}},
              recordingIncludeSensors: {{checked: true}}
            }};
            const $ = id => elements[id] || (elements[id] = {{}});
            const calls = [];
            let replayStatusSnapshot = {{
              playback: {{status: 'playing'}}
            }};
            function selectedRecordingId() {{ return 'rec_001'; }}
            function replayPartState(part) {{
              return String(part && (part.status || part.state) || 'idle');
            }}
            function validReplayRate() {{ return 2.5; }}
            function clearDirectionInputs() {{ return Promise.resolve(); }}
            function toast() {{}}
            async function runReplayAction(action, path, payload, options) {{
              calls.push({{action, path, payload, options}});
            }}
            {actions}

            (async () => {{
              await startRecording();
              await stopRecording();
              await startDataReplay();
              await toggleDataReplayPause();
              await applyReplayRate();
              await startTaskReplay();
              await cancelReplay();

              assert.deepEqual(calls.map(call => call.path), [
                '/recording/start',
                '/recording/stop',
                '/replay/data/start',
                '/replay/data/pause',
                '/replay/data/rate',
                '/replay/task/start',
                '/replay/cancel'
              ]);
              assert.deepEqual(calls[0].payload, {{
                name: 'press_5n_regression', include_sensors: true
              }});
              assert.deepEqual(calls[1].payload, {{}});
              assert.deepEqual(calls[2].payload, {{
                recording_id: 'rec_001', rate: 2.5
              }});
              assert.deepEqual(calls[3].payload, {{}});
              assert.deepEqual(calls[4].payload, {{rate: 2.5}});
              assert.deepEqual(calls[5].payload, {{recording_id: 'rec_001'}});
              assert.deepEqual(calls[6].payload, {{}});

              calls.length = 0;
              elements.recordingName.value = '  ';
              elements.recordingIncludeSensors.checked = false;
              replayStatusSnapshot.playback.status = 'paused';
              await startRecording();
              await toggleDataReplayPause();
              assert.deepEqual(calls[0].payload, {{include_sensors: false}});
              assert.equal(calls[1].path, '/replay/data/resume');
            }})().catch(error => {{
              console.error(error);
              process.exitCode = 1;
            }});
        """))

    def test_all_33_cabinet_controls_still_flow_through_catalog(self) -> None:
        apply_controls = _source_block(
            "function applyCabinetControls(payload)",
            "\nfunction renderCabinetTarget()",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            class Element {{
              constructor(tag = '') {{
                this.tagName = tag;
                this.children = [];
                this.value = '';
                this.textContent = '';
                this.disabled = false;
              }}
              appendChild(child) {{ this.children.push(child); }}
              replaceChildren(...children) {{ this.children = children; }}
            }}
            const document = {{createElement: tag => new Element(tag)}};
            const cabinetTarget = new Element('select');
            cabinetTarget.value = 'robot_limited';
            const cabinetControlTypeOrder = [0, 1, 2, 3];
            const cabinetControlTypeLabels = {{
              0: '按钮', 1: '旋钮', 2: '总开关', 3: '柜门'
            }};
            const cabinetCommandSupport = {{press: 1}};
            let cabinetControls = [];
            let cabinetCatalogReceived = false;
            let cabinetAvailableFromHealth = false;
            let cabinetOperationAvailable = false;
            let cabinetResetAvailable = false;
            let cabinetStatus = null;
            function isCabinetOperationActive() {{ return false; }}
            function renderCabinetTarget() {{}}
            function selectedCabinetName() {{ return 'cabinet_a'; }}
            {apply_controls}

            const controls = Array.from({{length: 33}}, (_, index) => ({{
              control_id: index === 12 ? 'robot_limited' : `control_${{index}}`,
              display_name: `控件 ${{index}}`,
              control_type: index % 4,
              supported_commands: 1,
              operable: index !== 12,
              unavailable_reason: index === 12 ? '机械臂不可达' : ''
            }}));
            assert.equal(applyCabinetControls({{
              cabinet: 'cabinet_a',
              catalog_received: true,
              controls
            }}), true);
            assert.equal(cabinetControls.length, 33);
            assert.equal(cabinetTarget.value, 'robot_limited');
            const options = cabinetTarget.children.flatMap(group => group.children);
            assert.equal(options.length, 33);
            assert.equal(
              options.find(option => option.value === 'robot_limited').disabled,
              false
            );
        """))

    def test_playback_read_only_interlocks_every_write_control(self) -> None:
        interlocks = _source_block(
            "function updateControlInterlocks()",
            "\nconst manualTakeoverRequestTimeoutMs",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            class Element {{
              constructor() {{ this.disabled = false; this.value = ''; }}
            }}
            const elements = new Map();
            const $ = id => {{
              if (!elements.has(id)) elements.set(id, new Element());
              return elements.get(id);
            }};
            const dpadButtons = Array.from({{length: 5}}, () => new Element());
            const jointInputs = Array.from({{length: 3}}, () => new Element());
            const document = {{
              querySelectorAll: selector => selector.includes('#dpad')
                ? dpadButtons
                : selector.includes('#jointSliders') ? jointInputs : []
            }};
            const cabinetTarget = new Element();
            const cabinetInstance = new Element();
            const navCabinetInstance = new Element();
            let replayControlReadOnly = true;
            let ctrlConnected = true;
            let cabinetCatalogReceived = true;
            let cabinetOperationAvailable = true;
            let cabinetResetAvailable = true;
            let cabinetInterlockWasActive = false;
            let manualTakeoverState = 'idle';
            const navigationStatus = {{available: true, mode: false}};
            const cabinetStatus = null;
            const cabinetInstances = [{{name: 'cabinet_a'}}];
            const manualJointSpecs = [{{name: 'joint_1'}}];
            function selectedCabinetControl() {{
              return {{control_id: 'box_10_button_1', control_type: 0, operable: true}};
            }}
            function selectedCabinetName() {{ return 'cabinet_a'; }}
            function isCabinetAvailable() {{ return true; }}
            function isCabinetOperationActive() {{ return false; }}
            function isNavigationActive() {{ return false; }}
            function hasRetiringNavigationGoals() {{ return false; }}
            function isManualControlReady() {{ return true; }}
            function clearDirectionInputs() {{}}
            function renderBaseControlStatus() {{}}
            function updateReplayControls() {{}}
            {interlocks}

            updateControlInterlocks();
            [
              'btnNavSend', 'btnNavCancel', 'btnManualMode',
              'btnArmZero',
              'btnCabinetPress', 'btnCabinetCancel', 'btnCabinetReset',
              'spdLinear', 'spdAngular', 'cabinetTargetState',
              'cabinetTargetPosition', 'cabinetForce'
            ].forEach(id => assert.equal($(id).disabled, true, id));
            dpadButtons.forEach(button => assert.equal(button.disabled, true));
            jointInputs.forEach(input => assert.equal(input.disabled, true));
            assert.equal(cabinetTarget.disabled, true);
            assert.equal(cabinetInstance.disabled, true);
            assert.equal(navCabinetInstance.disabled, true);

            // Recording itself is deliberately not a robot-control lock.
            replayControlReadOnly = false;
            updateControlInterlocks();
            assert.equal($('btnCabinetPress').disabled, false);
            assert.equal(dpadButtons[0].disabled, false);
        """))

    def test_replay_status_is_fail_closed_and_never_trusts_false_alone(self) -> None:
        read_only = _source_block(
            "function replayStatusReadOnly(status)",
            "\nfunction formatReplaySeconds(",
        )
        refresh = _source_block(
            "async function refreshReplayStatus(",
            "\nfunction startReplayPolling(",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            function playbackIsActive(status) {{
              return status && status.playback && status.playback.status === 'playing';
            }}
            function taskReplayIsActive(status) {{
              return status && status.task_replay && status.task_replay.status === 'running';
            }}
            {read_only}
            assert.equal(replayStatusReadOnly({{
              read_only: false,
              playback: {{status: 'playing'}},
              task_replay: {{status: 'idle'}}
            }}), true);

            let ctrlConnected = true;
            let replayStatusPollGeneration = 4;
            let replayStatusPollInFlight = false;
            let replayControlReadOnly = false;
            let frozen = 0;
            const replayState = {{}};
            const replayDetail = {{textContent: ''}};
            function freezeManualOutputsForReplay() {{ frozen += 1; }}
            function setAutonomyState() {{}}
            function updateReplayControls() {{}}
            function updateControlInterlocks() {{}}
            function applyReplayStatus() {{ throw new Error('unexpected'); }}
            async function controlRequestWithTimeout() {{
              throw new Error('status unavailable');
            }}
            {refresh}

            (async () => {{
              await refreshReplayStatus(4);
              assert.equal(replayControlReadOnly, true);
              assert.equal(frozen, 1);
              assert.match(replayDetail.textContent, /保持只读锁定/);
              assert.equal(replayStatusPollInFlight, false);
            }})().catch(error => {{
              console.error(error);
              process.exitCode = 1;
            }});
        """))

    def test_control_request_blocks_live_writes_but_allows_replay_cancel(self) -> None:
        request = _source_block(
            "async function controlRequest(",
            "\nconst autonomyPollIntervalMs",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            let ctrlConnected = true;
            let replayControlReadOnly = true;
            let authToken = '';
            const fetched = [];
            async function fetch(url, options) {{
              fetched.push({{url, options}});
              return {{ok: true, json: async () => ({{accepted: true}})}};
            }}
            const ctrlBaseUrl = 'http://localhost:8090';
            {request}

            (async () => {{
              await assert.rejects(
                controlRequest('/task/navigate', 'POST', {{cabinet: 'cabinet_a'}}),
                /只读/
              );
              await assert.rejects(
                controlRequest('/task/reset', 'POST', {{cabinet: 'cabinet_a'}}),
                /只读/
              );
              await assert.rejects(
                controlRequest('/joint_trajectory', 'POST', {{positions: [0]}}),
                /只读/
              );
              assert.equal(fetched.length, 0);
              await controlRequest('/replay/cancel', 'POST', {{}});
              assert.equal(fetched.length, 1);
              assert.equal(fetched[0].url, 'http://localhost:8090/replay/cancel');
            }})().catch(error => {{
              console.error(error);
              process.exitCode = 1;
            }});
        """))

    def test_progress_and_timeline_render_current_task_context(self) -> None:
        render_status = _source_block(
            "function renderReplayStatus()",
            "\nfunction markReplayDisconnected(",
        )
        timeline = _source_block(
            "function timelineEventSummary(event)",
            "\nasync function refreshReplayTimeline(",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            class Element {{
              constructor() {{
                this.textContent = '';
                this.hidden = true;
                this.disabled = false;
                this.style = {{}};
                this.children = [];
                this.scrollTop = 0;
                this.scrollHeight = 42;
              }}
              append(...children) {{ this.children.push(...children); }}
              replaceChildren(...children) {{ this.children = children; }}
            }}
            const elements = new Map();
            const $ = id => {{
              if (!elements.has(id)) elements.set(id, new Element());
              return elements.get(id);
            }};
            const document = {{createElement: () => new Element()}};
            const replayState = new Element();
            const replayDetail = new Element();
            const replayTimeline = new Element();
            let ctrlConnected = true;
            let replayControlReadOnly = true;
            let replayActionError = '';
            let replayStatusSnapshot = {{
              mode: 'data_playback',
              read_only: true,
              recording: {{status: 'idle'}},
              playback: {{
                status: 'playing',
                recording_id: 'rec_001',
                rate: 2,
                progress: 0.375,
                position_seconds: 12.5,
                duration_seconds: 40,
                message: '正在播放隔离话题'
              }},
              task_replay: {{status: 'idle'}}
            }};
            function activeReplayPart(status) {{
              return {{kind: 'playback', value: status.playback}};
            }}
            function replayPartState(part) {{
              return String(part && (part.status || part.state) || 'idle');
            }}
            function replayFailureMessage() {{ return ''; }}
            function recordingIsActive() {{ return false; }}
            function replayModeLabel() {{ return '只读数据回放'; }}
            function replayStateLabel() {{ return '播放中'; }}
            function clampReplayProgress(value) {{ return value; }}
            function formatReplaySeconds(value) {{ return Number(value).toFixed(1); }}
            function setAutonomyState(element, text, cls) {{
              element.textContent = text;
              element.className = cls;
            }}
            function updateReplayControls() {{}}
            function updateControlInterlocks() {{}}
            function formatReplayTimestamp(value) {{ return String(value); }}
            {render_status}
            {timeline}

            renderReplayStatus();
            assert.equal($('replayProgressFill').style.width, '38%');
            assert.equal($('replayProgressText').textContent, '38%');
            assert.equal($('replayPosition').textContent, '12.5 / 40.0 s');
            assert.equal($('replayMode').textContent, '只读数据回放');
            assert.equal($('replayReadOnly').hidden, false);
            assert.match(replayDetail.textContent, /安全互锁/);
            assert.match(replayDetail.textContent, /rec_001/);

            renderReplayTimeline([{{
              event: 'task_completed',
              elapsed_seconds: 8.25,
              data: {{task: {{
                task_id: 'operate_1',
                status: 'failed',
                phase: 'operation_pressing',
                request: {{
                  cabinet: 'cabinet_a',
                  control_id: 'box_10_button_1',
                  force: 4
                }},
                result: {{estimated_force: 3.8}},
                failure_reason: '力度不足'
              }}}}
            }}], 'rec_001');
            assert.equal(replayTimeline.children.length, 1);
            const row = replayTimeline.children[0];
            assert.equal(row.children[0].textContent, '+8.25s');
            assert.equal(row.children[1].textContent, 'task_completed');
            assert.match(row.children[2].textContent, /cabinet=cabinet_a/);
            assert.match(row.children[2].textContent, /control=box_10_button_1/);
            assert.match(row.children[2].textContent, /force=4.00N/);
            assert.match(row.children[2].textContent, /力度不足/);
        """))

    def test_timeline_fetches_every_bounded_page(self) -> None:
        refresh = _source_block(
            "async function refreshReplayTimeline(",
            "\nfunction validReplayRate()",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            let ctrlConnected = true;
            let replayTimelineRequestGeneration = 0;
            let replayTimelineRecordingId = null;
            let replayTimelineRequestInFlightId = null;
            let replayTimelineLastRefreshAt = 0;
            const replayTimeline = {{textContent: ''}};
            const paths = [];
            let rendered = null;
            function selectedRecordingId() {{ return 'rec_001'; }}
            function renderReplayTimeline(events) {{ rendered = events; }}
            function toast() {{}}
            async function controlRequestWithTimeout(path) {{
              paths.push(path);
              if (paths.length === 1) return {{
                events: [{{event: 'one'}}], has_more: true, next_offset: 1
              }};
              return {{
                events: [{{event: 'two'}}], has_more: false, next_offset: 2
              }};
            }}
            {refresh}

            (async () => {{
              await refreshReplayTimeline('rec_001');
              assert.deepEqual(paths, [
                '/recordings/rec_001/timeline?offset=0&limit=500',
                '/recordings/rec_001/timeline?offset=1&limit=500'
              ]);
              assert.deepEqual(rendered.map(event => event.event), ['one', 'two']);
              assert.equal(replayTimelineRequestInFlightId, null);
            }})().catch(error => {{
              console.error(error);
              process.exitCode = 1;
            }});
        """))


if __name__ == "__main__":
    unittest.main()
