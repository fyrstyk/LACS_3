#include "web_server.h"
#include "motor_control.h"
#include "sun_tracking.h"
#include "storage.h"
#include "ota.h"
#include "hardware_config.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>

static const char *TAG = "WEB_SERVER";

// Global web server handle
httpd_handle_t web_server = NULL;

// External references
extern float sun_az, sun_el;
extern bool time_synced;
extern bool auto_tracking;
extern bool sync_mode;
extern float latitude;
extern float longitude;
extern float home_bearing;
extern bool tracking_in_progress;
extern bool deploy_in_progress;
extern bool stow_in_progress;
extern bool extensions_deployed;
extern motor_group_t active_group;
extern uint16_t active_motors;
extern uint8_t expansion_relay_state;
extern motor_state_t motor_states[11];
extern motor_config_t motors[11];
extern sync_state_t sync_state;
extern bool Deploy_All_API_Enable_Flag;
extern TaskHandle_t auto_tracking_handle;

// HTML content split into parts for chunked response
static const char html_part1[] = 
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"<meta charset='UTF-8'>\n"
"<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
"<title>LACS2</title>\n"
"<style>\n"
"* { box-sizing: border-box; }\n"
"body { font-family: Arial, sans-serif; max-width: 1200px; margin: 0 auto; padding: 20px; background: #f0f0f0; }\n"
"h1, h2 { color: #333; }\n"
".container { background: #fff; border-radius: 10px; padding: 20px; margin-bottom: 20px; box-shadow: 0 2px 10px rgba(0,0,0,.1); }\n"
".controls { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; }\n"
".control-group { background: #f8f8f8; padding: 15px; border-radius: 5px; }\n"
"button { background: #2196f3; color: #fff; border: none; padding: 10px 20px; margin: 5px; border-radius: 5px; cursor: pointer; font-size: 16px; }\n"
"button:hover { background: #1976d2; }\n"
"button.stop { background: #f44336; }\n"
"button.stop:hover { background: #d32f2f; }\n"
"button.active { background: #4caf50; }\n"
"button:disabled { background: #ccc; cursor: not-allowed; }\n"
"input { width: 100%; padding: 8px; margin: 5px 0; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }\n"
"input[type='range'] { -webkit-appearance: none; height: 25px; background: #ddd; border-radius: 5px; outline: none; }\n"
"input[type='range']::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 25px; height: 25px; background: #2196f3; cursor: pointer; border-radius: 50%; }\n"
"input[type='range']::-moz-range-thumb { width: 25px; height: 25px; background: #2196f3; cursor: pointer; border-radius: 50%; }\n"
".status { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px; margin-bottom: 20px; }\n"
".status-item { text-align: center; background: #f8f8f8; padding: 10px; border-radius: 5px; cursor: pointer; transition: all 0.3s; border: 2px solid transparent; box-sizing: border-box; }\n"
".status-item:hover { background: #e0e0e0; }\n"
".status-item.inactive { opacity: 0.5; }\n"
".status-item.open-loop { background: #fff3cd; border-color: #ffeaa7; }\n"
".status-item.moving { border-color: #2196f3; animation: pulse 1s infinite; }\n"
"@keyframes pulse { 0% { border-color: #2196f3; } 50% { border-color: #64b5f6; } 100% { border-color: #2196f3; } }\n"
".status-value { font-size: 24px; font-weight: bold; color: #2196f3; }\n"
".status-value small { font-size: 12px; font-weight: normal; color: #666; }\n"
".manual-btn { touch-action: none; user-select: none; width: 100%; }\n"
".manual-btn:disabled { opacity: 0.6; background: #999 !important; }\n"
".motor-select { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin: 10px 0; }\n"
".motor-checkbox { display: flex; align-items: center; gap: 5px; }\n"
".bearing-info { background: #e3f2fd; padding: 10px; border-radius: 5px; margin: 10px 0; }\n"
".bearing-value { font-weight: bold; color: #1976d2; }\n"
".sync-indicator { display: inline-block; margin-left: 10px; padding: 2px 8px; border-radius: 3px; font-size: 12px; background: #4caf50; color: white; }\n"
".sync-error { background: #ff9800; }\n"
"label { display: flex; align-items: center; margin: 10px 0; font-weight: normal; }\n"
"label input[type='checkbox'] { margin-right: 8px; }\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<h1>LACS2 - Unit-011 OTA Updates with Leveling and Control System 2</h1>\n"
"<div class='container'>\n"
"<h2>System Status</h2>\n"
"<div class='status' id='status'></div>\n"
"<div id='sync-status' style='margin-top: 15px;'></div>\n"
"</div>\n"
"<div class='container'>\n"
"<h2>Motor Control</h2>\n"
"<div style='display: flex; gap: 10px; margin-bottom: 15px; flex-wrap: wrap;' id='group-buttons'>\n"
"<button onclick='selectGroup(0)'>None</button>\n"
"<button onclick='selectGroup(1)'>Legs</button>\n"
"<button onclick='selectGroup(2)'>Elevation / Panels</button>\n"
"<button onclick='selectGroup(3)'>Slew</button>\n"
"</div>\n"
"<div id='motor-select-container'>\n"
"<h3>Individual Motor Selection</h3>\n"
"<div class='motor-select' id='motor-select'></div>\n"
"</div>\n"
"</div>\n";

static const char html_part2[] = 
"<div class='container'>\n"
"<h2>Manual Control</h2>\n"
"<div class='controls'>\n"
"<div class='control-group'>\n"
"<h3>Speed Control</h3>\n"
"<label for='speed-slider' style='margin-bottom: 5px; display: block;'>Manual Speed: <span id='speed-value' style='font-weight: bold; color: #2196f3; font-size: 18px;'>150</span></label>\n"
"<input type='range' id='speed-slider' min='20' max='250' value='150' style='width: 100%; margin-bottom: 15px;' oninput='updateSpeedDisplay()' title='Speed for manual movements'>\n"
"<button class='manual-btn' id='up-btn'>UP / EXTEND</button>\n"
"<button class='manual-btn' id='down-btn'>DOWN / RETRACT</button>\n"
"<button class='stop' onclick='stopAll()' style='background: #f44336 !important;' id='stop-btn'>STOP ALL</button>\n"
"</div>\n"
"<div class='control-group'>\n"
"<h3>Position Control (Closed-Loop Only)</h3>\n"
"<input type='number' id='target-pos' placeholder='Target position (mm or degrees)' step='10'>\n"
"<label for='goto-speed-slider' style='margin-bottom: 5px; display: block;'>Movement Speed: <span id='goto-speed-value' style='font-weight: bold; color: #2196f3; font-size: 18px;'>150</span></label>\n"
"<input type='range' id='goto-speed-slider' min='20' max='250' value='150' style='width: 100%; margin-bottom: 15px;' oninput='updateGotoSpeedDisplay()' title='Speed for position movements'>\n"
"<button onclick='goToPosition()'>Go To Position</button>\n"
"<button onclick='syncLift()' style='background: #4CAF50;'>Sync Lift (Selected Legs)</button>\n"
"<button onclick='zeroSelectedPositions()'>Zero Selected Motors</button>\n"
"<label style='display: flex; align-items: center; margin-top: 10px;'>\n"
"<input type='checkbox' id='sync-mode' onchange='setSyncMode()' style='margin-right: 8px;'>\n"
"<span>Enable Synchronized Movement</span>\n"
"</label>\n"
"</div>\n"
"<div class='control-group'>\n"
"<h3>Calibration (Closed-Loop Only)</h3>\n"
"<input type='number' id='cal-motor' placeholder='Motor ID (0-6)' min='0' max='6'>\n"
"<input type='number' id='cal-value' placeholder='Pulses per unit' step='0.1'>\n"
"<button onclick='setCalibration()'>Set Calibration</button>\n"
"</div>\n"
"<div class='control-group'>\n"
"<h3>Solar Tracking</h3>\n"
"<div class='bearing-info'>Location: <span class='bearing-value' id='location-display'>52.77°N, -0.38°E</span></div>\n"
"<input type='number' id='latitude' placeholder='Latitude (-90 to 90°)' min='-90' max='90' step='0.01'>\n"
"<input type='number' id='longitude' placeholder='Longitude (-180 to 180°)' min='-180' max='180' step='0.01'>\n"
"<button onclick='setLocation()'>Set Location</button>\n"
"<div class='bearing-info'>Home Bearing: <span class='bearing-value' id='home-bearing-display'>0°</span> from North</div>\n"
"<input type='number' id='home-bearing' placeholder='Home bearing (0-360°)' min='0' max='360' step='0.1'>\n"
"<button onclick='setHomeBearing()'>Set Home Bearing</button>\n"
"<div id='sun-info'></div>\n"
"<button onclick='syncTime()'>Sync Time</button>\n"
"<div style='display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 10px;'>\n"
"<button id='deploy-btn' onclick='deploy()' style='background: #4CAF50;'>Deploy</button>\n"
"<button id='stow-btn' onclick='stow()' style='background: #FF9800;'>Stow</button>\n"
"</div>\n"
"<button id='track-btn' onclick='toggleTracking()' style='width: 100%; margin-top: 10px;'>Enable Tracking</button>\n"
"</div>\n"
"</div>\n"
"</div>\n";

static const char html_part3[] = 
"<script>\n"
"let activeGroup = 0;\n"
"let activeMotors = 0x0000;\n"
"let autoTracking = false;\n"
"let motorData = {};\n"
"let homeBearing = 0;\n"
"let latitude = 52.77;\n"
"let longitude = -0.38;\n"
"let trackingInProgress = false;\n"
"let deployInProgress = false;\n"
"let stowInProgress = false;\n"
"let syncMode = false;\n"
"let syncMaxError = 0;\n"
"let manualSpeed = 150;\n"
"let gotoSpeed = 150;\n"
"let sync_state = {is_moving: false, target_position: 0};\n"
"\n"
"const groupNames = ['None', 'Legs', 'Elevation / Panels', 'Slew'];\n"
"\n"
"function updateSpeedDisplay() {\n"
"    const slider = document.getElementById('speed-slider');\n"
"    manualSpeed = parseInt(slider.value);\n"
"    document.getElementById('speed-value').textContent = manualSpeed;\n"
"}\n"
"\n"
"function updateGotoSpeedDisplay() {\n"
"    const slider = document.getElementById('goto-speed-slider');\n"
"    gotoSpeed = parseInt(slider.value);\n"
"    document.getElementById('goto-speed-value').textContent = gotoSpeed;\n"
"}\n"
"\n"
"function cmd(command, params = {}) {\n"
"    fetch('/command', {\n"
"        method: 'POST',\n"
"        headers: {'Content-Type': 'application/json'},\n"
"        body: JSON.stringify({cmd: command, params: params})\n"
"    }).catch(e => console.error(e));\n"
"}\n"
"\n"
"function selectGroup(g) {\n"
"    if (!trackingInProgress && !deployInProgress && !stowInProgress) {\n"
"        cmd('select_group', {group: g});\n"
"    } else {\n"
"        alert('Cannot change group - operation in progress');\n"
"    }\n"
"}\n"
"\n"
"function updateMotorSelect() {\n"
"    const container = document.getElementById('motor-select-container');\n"
"    const select = document.getElementById('motor-select');\n"
"    \n"
"    if (activeGroup === 0) {\n"
"        container.style.display = 'none';\n"
"        return;\n"
"    }\n"
"    \n"
"    container.style.display = 'block';\n"
"    select.innerHTML = '';\n"
"    \n"
"    let motors = [];\n"
"    if (activeGroup === 1) motors = [0, 1, 2, 3];\n"
"    else if (activeGroup === 2) motors = [4, 5, 7, 8, 9, 10];\n"
"    else if (activeGroup === 3) motors = [6];\n"
"    \n"
"    motors.forEach(m => {\n"
"        const div = document.createElement('div');\n"
"        div.className = 'motor-checkbox';\n"
"        \n"
"        const cb = document.createElement('input');\n"
"        cb.type = 'checkbox';\n"
"        cb.id = `motor${m}`;\n"
"        cb.checked = !!(activeMotors & (1 << m));\n"
"        cb.onchange = updateMotorMask;\n"
"        \n"
"        const label = document.createElement('label');\n"
"        label.htmlFor = `motor${m}`;\n"
"        label.textContent = motorData.names ? motorData.names[m] : `Motor ${m}`;\n"
"        \n"
"        div.appendChild(cb);\n"
"        div.appendChild(label);\n"
"        select.appendChild(div);\n"
"    });\n"
"}\n"
"\n"
"function updateMotorMask() {\n"
"    let mask = 0;\n"
"    for (let i = 0; i < 11; i++) {\n"
"        const cb = document.getElementById(`motor${i}`);\n"
"        if (cb && cb.checked) mask |= (1 << i);\n"
"    }\n"
"    activeMotors = mask;\n"
"    cmd('select_motors', {mask: mask});\n"
"}\n"
"\n"
"function stopAll() {\n"
"    cmd('stop_all');\n"
"    setTimeout(updateStatus, 100);\n"
"}\n"
"\n"
"function syncLift() {\n"
"    if (trackingInProgress || deployInProgress || stowInProgress) return;\n"
"    const h = parseFloat(prompt('Height in mm (0-615):'));\n"
"    if (!isNaN(h)) cmd('sync_lift', {height: h, speed: gotoSpeed});\n"
"}\n"
"\n"
"function goToPosition() {\n"
"    if (trackingInProgress || deployInProgress || stowInProgress) return;\n"
"    const p = parseFloat(document.getElementById('target-pos').value);\n"
"    if (isNaN(p)) return;\n"
"    \n"
"    for (let i = 0; i < 11; i++) {\n"
"        if (activeMotors & (1 << i) && motorData.types && motorData.types[i] === 'closed') {\n"
"            cmd('go_to', {motor: i, position: p, speed: gotoSpeed});\n"
"        }\n"
"    }\n"
"    \n"
"    if (motorData.types) {\n"
"        const hasOpenLoop = [...Array(11)].some((v, i) => (activeMotors & (1 << i)) && motorData.types[i] === 'open');\n"
"        if (hasOpenLoop) alert('Note: Open-loop motors do not support position control');\n"
"    }\n"
"}\n"
"\n"
"function zeroSelectedPositions() {\n"
"    for (let i = 0; i < 7; i++) {\n"
"        if (activeMotors & (1 << i)) cmd('zero_position', {motor: i});\n"
"    }\n"
"}\n"
"\n"
"function setCalibration() {\n"
"    const m = parseInt(document.getElementById('cal-motor').value);\n"
"    const v = parseFloat(document.getElementById('cal-value').value);\n"
"    if (!isNaN(m) && !isNaN(v) && m < 7) cmd('set_calibration', {motor: m, value: v});\n"
"}\n"
"\n"
"function setHomeBearing() {\n"
"    const b = parseFloat(document.getElementById('home-bearing').value);\n"
"    if (!isNaN(b) && b >= 0 && b <= 360) {\n"
"        cmd('set_home_bearing', {bearing: b});\n"
"        homeBearing = b;\n"
"        document.getElementById('home-bearing-display').textContent = b.toFixed(1) + '°';\n"
"    }\n"
"}\n"
"\n"
"function setLocation() {\n"
"    const lat = parseFloat(document.getElementById('latitude').value);\n"
"    const lon = parseFloat(document.getElementById('longitude').value);\n"
"    if (!isNaN(lat) && !isNaN(lon) && lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180) {\n"
"        cmd('set_location', {latitude: lat, longitude: lon});\n"
"        latitude = lat;\n"
"        longitude = lon;\n"
"        document.getElementById('location-display').textContent = \n"
"            `${lat.toFixed(2)}°${lat >= 0 ? 'N' : 'S'}, ${lon.toFixed(2)}°${lon >= 0 ? 'E' : 'W'}`;\n"
"    }\n"
"}\n"
"\n"
"function syncTime() {\n"
"    cmd('sync_time', {timestamp: Math.floor(Date.now() / 1000)});\n"
"}\n"
"\n"
"function setSyncMode() {\n"
"    const cb = document.getElementById('sync-mode');\n"
"    syncMode = cb.checked;\n"
"    cmd('set_sync_mode', {enable: syncMode});\n"
"}\n"
"\n"
"function deploy() {\n"
"    if (!deployInProgress && !stowInProgress) {\n"
"        if (!motorData.time_synced) {\n"
"            cmd('sync_time', {timestamp: Math.floor(Date.now() / 1000)});\n"
"            setTimeout(() => cmd('deploy'), 100);\n"
"        } else {\n"
"            cmd('deploy');\n"
"        }\n"
"        deployInProgress = true;\n"
"    }\n"
"}\n"
"\n"
"function stow() {\n"
"    if (!deployInProgress && !stowInProgress) {\n"
"        cmd('stow');\n"
"        stowInProgress = true;\n"
"    }\n"
"}\n"
"\n"
"function toggleTracking() {\n"
"    autoTracking = !autoTracking;\n"
"    cmd('auto_track', {enable: autoTracking});\n"
"    document.getElementById('track-btn').textContent = autoTracking ? 'Disable Tracking' : 'Enable Tracking';\n"
"}\n";

static const char html_part4[] = 
"\n"
"async function updateStatus() {\n"
"    try {\n"
"        const r = await fetch('/status');\n"
"        const d = await r.json();\n"
"        \n"
"        motorData = {\n"
"            names: d.motor_names,\n"
"            types: d.motor_types,\n"
"            time_synced: d.time_synced,\n"
"            speeds: d.speeds || []\n"
"        };\n"
"        \n"
"        homeBearing = d.home_bearing || 0;\n"
"        latitude = d.latitude || 52.77;\n"
"        longitude = d.longitude || -0.38;\n"
"        syncMode = d.sync_mode || false;\n"
"        syncMaxError = d.sync_max_error || 0;\n"
"        sync_state.is_moving = d.sync_is_moving || false;\n"
"        sync_state.target_position = d.sync_target_position || 0;\n"
"        \n"
"        document.getElementById('sync-mode').checked = syncMode;\n"
"        document.getElementById('home-bearing-display').textContent = homeBearing.toFixed(1) + '°';\n"
"        \n"
"        const hbInput = document.getElementById('home-bearing');\n"
"        const latInput = document.getElementById('latitude');\n"
"        const lonInput = document.getElementById('longitude');\n"
"        \n"
"        if (hbInput !== document.activeElement) hbInput.value = homeBearing.toFixed(1);\n"
"        if (latInput !== document.activeElement) latInput.value = latitude.toFixed(2);\n"
"        if (lonInput !== document.activeElement) lonInput.value = longitude.toFixed(2);\n"
"        \n"
"        document.getElementById('location-display').textContent = \n"
"            `${latitude.toFixed(2)}°${latitude >= 0 ? 'N' : 'S'}, ${longitude.toFixed(2)}°${longitude >= 0 ? 'E' : 'W'}`;\n"
"        \n"
"        // Update motor status display\n"
"        const units = ['mm', 'mm', 'mm', 'mm', '°', '°', '°', '', '', '', ''];\n"
"        let html = '';\n"
"        \n"
"        d.positions.forEach((p, i) => {\n"
"            const active = !!(d.active_motors & (1 << i));\n"
"            const isOpenLoop = d.motor_types[i] === 'open';\n"
"            const speed = d.speeds ? d.speeds[i] : 0;\n"
"            const isMoving = speed !== 0;\n"
"            \n"
"            html += `<div class='status-item ${active ? '' : 'inactive'} ${isOpenLoop ? 'open-loop' : ''} ${isMoving ? 'moving' : ''}' onclick='toggleMotor(${i})'>`;\n"
"            html += `<div>${d.motor_names[i]}</div>`;\n"
"            html += `<div class='status-value'>`;\n"
"            if (isOpenLoop) {\n"
"                html += 'Open Loop';\n"
"            } else {\n"
"                html += `${p.toFixed(1)}${units[i]}`;\n"
"            }\n"
"            if (isMoving) html += `<br><small>Speed: ${speed}</small>`;\n"
"            html += `</div></div>`;\n"
"        });\n"
"        \n"
"        document.getElementById('status').innerHTML = html;\n"
"        \n"
"        // Update sun info\n"
"        if (d.time_synced && d.sun_azimuth !== undefined) {\n"
"            let sunInfo = `Sun: ${d.sun_azimuth.toFixed(1)}° / ${d.sun_elevation.toFixed(1)}°<br>Time: ${d.time}`;\n"
"            if (d.target_azimuth !== undefined) {\n"
"                sunInfo += `<br>Target: ${d.target_azimuth.toFixed(1)}° / ${d.target_tilt.toFixed(1)}°`;\n"
"            }\n"
"            if (d.tracking_in_progress) {\n"
"                sunInfo += `<br><span style='color:#ff9800;font-weight:bold'>🔄 Tracking update in progress...</span>`;\n"
"            }\n"
"            if (d.deploy_in_progress) {\n"
"                sunInfo += `<br><span style='color:#2196F3;font-weight:bold'>🚀 Deployment in progress...</span>`;\n"
"            }\n"
"            if (d.stow_in_progress) {\n"
"                sunInfo += `<br><span style='color:#FF9800;font-weight:bold'>📦 Stowing in progress...</span>`;\n"
"            }\n"
"            if (d.extensions_deployed) {\n"
"                sunInfo += `<br><span style='color:#4CAF50;font-weight:bold'>📐 Extensions deployed</span>`;\n"
"            }\n"
"            document.getElementById('sun-info').innerHTML = sunInfo;\n"
"        } else {\n"
"            document.getElementById('sun-info').innerHTML = 'Time not synced';\n"
"        }\n"
"        \n"
"        // Update sync status display\n"
"        const syncStatusEl = document.getElementById('sync-status');\n"
"        if (syncMode) {\n"
"            let syncHtml = `<div style='padding:15px;background:#e3f2fd;border-radius:5px;border:1px solid #2196f3'>`;\n"
"            syncHtml += `<h4 style='margin:0 0 10px 0'>Synchronized Movement: <span style='color:#4caf50'>ENABLED</span></h4>`;\n"
"            \n"
"            if (sync_state.is_moving && syncMaxError > 0) {\n"
"                const errorClass = syncMaxError > 1.0 ? 'color:#ff5722' : 'color:#4caf50';\n"
"                syncHtml += `<p style='margin:5px 0'>Position Error: <span style='${errorClass};font-weight:bold'>${syncMaxError.toFixed(2)}${activeGroup === 1 ? 'mm' : '°'}</span></p>`;\n"
"                syncHtml += `<p style='margin:5px 0'>Target: ${sync_state.target_position?.toFixed(1) || '--'}${activeGroup === 1 ? 'mm' : '°'}</p>`;\n"
"            } else if (sync_state.is_moving) {\n"
"                syncHtml += `<p style='margin:5px 0;color:#666'>Waiting for movement...</p>`;\n"
"            } else {\n"
"                syncHtml += `<p style='margin:5px 0;color:#666'>No synchronized movement active</p>`;\n"
"            }\n"
"            \n"
"            syncHtml += `</div>`;\n"
"            syncStatusEl.innerHTML = syncHtml;\n"
"        } else {\n"
"            syncStatusEl.innerHTML = '';\n"
"        }\n"
"        \n"
"        activeGroup = d.active_group;\n"
"        activeMotors = d.active_motors;\n"
"        autoTracking = d.auto_tracking;\n"
"        trackingInProgress = d.tracking_in_progress || false;\n"
"        deployInProgress = d.deploy_in_progress || false;\n"
"        stowInProgress = d.stow_in_progress || false;\n"
"        \n"
"        document.getElementById('track-btn').textContent = autoTracking ? 'Disable Tracking' : 'Enable Tracking';\n"
"        \n"
"        // Update button states\n"
"        const anyInProgress = trackingInProgress || deployInProgress || stowInProgress;\n"
"        const groupBtns = document.querySelectorAll('#group-buttons button');\n"
"        groupBtns.forEach((btn, i) => {\n"
"            btn.disabled = anyInProgress;\n"
"            btn.classList.toggle('active', i === activeGroup);\n"
"        });\n"
"        \n"
"        document.getElementById('up-btn').disabled = anyInProgress;\n"
"        document.getElementById('down-btn').disabled = anyInProgress;\n"
"        document.getElementById('stop-btn').disabled = false;\n"
"        document.getElementById('deploy-btn').disabled = deployInProgress || stowInProgress;\n"
"        document.getElementById('stow-btn').disabled = deployInProgress || stowInProgress;\n"
"        \n"
"        updateMotorSelect();\n"
"    } catch (e) {\n"
"        console.error(e);\n"
"    }\n"
"}\n"
"\n"
"function toggleMotor(m) {\n"
"    const cb = document.getElementById(`motor${m}`);\n"
"    if (cb) {\n"
"        cb.checked = !cb.checked;\n"
"        updateMotorMask();\n"
"    }\n"
"}\n"
"\n"
"// Manual movement controls\n"
"const upBtn = document.getElementById('up-btn');\n"
"const downBtn = document.getElementById('down-btn');\n"
"let moveInterval = null;\n"
"\n"
"function startMove(dir) {\n"
"    if (trackingInProgress || deployInProgress || stowInProgress) return;\n"
"    \n"
"    const motors = [];\n"
"    for (let i = 0; i < 11; i++) {\n"
"        if (activeMotors & (1 << i)) motors.push(i);\n"
"    }\n"
"    \n"
"    motors.forEach(motor => cmd('set_speed', {motor: motor, speed: dir * manualSpeed}));\n"
"    moveInterval = setInterval(() => {\n"
"        motors.forEach(motor => cmd('set_speed', {motor: motor, speed: dir * manualSpeed}));\n"
"    }, 500);\n"
"}\n"
"\n"
"function stopMove() {\n"
"    if (moveInterval) {\n"
"        clearInterval(moveInterval);\n"
"        moveInterval = null;\n"
"    }\n"
"    \n"
"    const motors = [];\n"
"    for (let i = 0; i < 11; i++) {\n"
"        if (activeMotors & (1 << i)) motors.push(i);\n"
"    }\n"
"    \n"
"    motors.forEach(motor => cmd('set_speed', {motor: motor, speed: 0}));\n"
"}\n"
"\n"
"// Touch and mouse event handlers\n"
"['mousedown', 'touchstart'].forEach(e => {\n"
"    upBtn.addEventListener(e, ev => {\n"
"        ev.preventDefault();\n"
"        startMove(1);\n"
"    });\n"
"    downBtn.addEventListener(e, ev => {\n"
"        ev.preventDefault();\n"
"        startMove(-1);\n"
"    });\n"
"});\n"
"\n"
"['mouseup', 'touchend', 'mouseleave', 'touchcancel'].forEach(e => {\n"
"    upBtn.addEventListener(e, ev => {\n"
"        ev.preventDefault();\n"
"        stopMove();\n"
"    });\n"
"    downBtn.addEventListener(e, ev => {\n"
"        ev.preventDefault();\n"
"        stopMove();\n"
"    });\n"
"});\n"
"\n"
"// Initialize on load\n"
"setInterval(updateStatus, 1000);\n"
"updateStatus();\n"
"updateSpeedDisplay();\n"
"updateGotoSpeedDisplay();\n"
"\n"
"</script>\n"
"</body>\n"
"</html>";

// Index handler - serves the main HTML page
esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    
    // Send HTML in chunks
    httpd_resp_send_chunk(req, html_part1, strlen(html_part1));
    httpd_resp_send_chunk(req, html_part2, strlen(html_part2));
    httpd_resp_send_chunk(req, html_part3, strlen(html_part3));
    httpd_resp_send_chunk(req, html_part4, strlen(html_part4));
    
    // Send final chunk to indicate end
    httpd_resp_send_chunk(req, NULL, 0);
    
    return ESP_OK;
}

// API status handler
esp_err_t api_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    char *json_str = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

// API debug handler
esp_err_t api_debug_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON *logs = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "logs", logs);
    cJSON_AddNumberToObject(root, "latest", 0);
    char *json_str = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

// Get status handler - returns system status as JSON
esp_err_t get_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON *positions = cJSON_CreateArray();
    cJSON *speeds = cJSON_CreateArray();
    cJSON *calibrations = cJSON_CreateArray();
    cJSON *motor_names = cJSON_CreateArray();
    cJSON *motor_types = cJSON_CreateArray();
    
    for (int i = 0; i < 11; i++) {
        cJSON_AddItemToArray(positions, cJSON_CreateNumber(motor_states[i].position));
        cJSON_AddItemToArray(speeds, cJSON_CreateNumber(motor_states[i].speed));
        cJSON_AddItemToArray(calibrations, cJSON_CreateNumber(motor_states[i].pulses_per_unit));
        cJSON_AddItemToArray(motor_names, cJSON_CreateString(motors[i].name));
        cJSON_AddItemToArray(motor_types, cJSON_CreateString(motors[i].type == MOTOR_TYPE_CLOSED_LOOP ? "closed" : "open"));
    }
    
    cJSON_AddItemToObject(root, "positions", positions);
    cJSON_AddItemToObject(root, "speeds", speeds);
    cJSON_AddItemToObject(root, "calibrations", calibrations);
    cJSON_AddItemToObject(root, "motor_names", motor_names);
    cJSON_AddItemToObject(root, "motor_types", motor_types);
    cJSON_AddNumberToObject(root, "active_group", active_group);
    cJSON_AddNumberToObject(root, "active_motors", active_motors);
    cJSON_AddBoolToObject(root, "auto_tracking", auto_tracking);
    cJSON_AddBoolToObject(root, "tracking_in_progress", tracking_in_progress);
    cJSON_AddBoolToObject(root, "deploy_in_progress", deploy_in_progress);
    cJSON_AddBoolToObject(root, "stow_in_progress", stow_in_progress);
    cJSON_AddBoolToObject(root, "time_synced", time_synced);
    cJSON_AddBoolToObject(root, "extensions_deployed", extensions_deployed);
    cJSON_AddBoolToObject(root, "sync_mode", sync_mode);
    cJSON_AddNumberToObject(root, "sync_max_error", sync_state.max_error);
    cJSON_AddBoolToObject(root, "sync_is_moving", sync_state.is_moving);
    cJSON_AddNumberToObject(root, "sync_target_position", sync_state.target_position);
    cJSON_AddNumberToObject(root, "expansion_relay_state", expansion_relay_state);
    cJSON_AddNumberToObject(root, "home_bearing", home_bearing);
    cJSON_AddNumberToObject(root, "latitude", latitude);
    cJSON_AddNumberToObject(root, "longitude", longitude);
    
    // Add sun position if time synced
    if (time_synced) {
        calculate_sun_position(&sun_az, &sun_el);
        cJSON_AddNumberToObject(root, "sun_azimuth", sun_az);
        cJSON_AddNumberToObject(root, "sun_elevation", sun_el);
        
        if (sun_el > 10.0f) {
            float target_azimuth = sun_az - home_bearing;
            while (target_azimuth > 270.0f) target_azimuth -= 360.0f;
            while (target_azimuth < -270.0f) target_azimuth += 360.0f;
            
            float optimal_tilt = 90.0f - sun_el;
            cJSON_AddNumberToObject(root, "target_azimuth", target_azimuth);
            cJSON_AddNumberToObject(root, "target_tilt", optimal_tilt);
        }
    }
    
    // Add time
    char time_str[32];
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
    cJSON_AddStringToObject(root, "time", time_str);
    
    char *json_str = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    free(json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

// Command handler - processes control commands
esp_err_t command_handler(httpd_req_t *req) {
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) return ESP_FAIL;
    
    cJSON *cmd = cJSON_GetObjectItem(json, "cmd");
    cJSON *params = cJSON_GetObjectItem(json, "params");
    
    if (cJSON_IsString(cmd)) {
        const char *command = cmd->valuestring;
        
        if (strcmp(command, "select_group") == 0 && params) {
            cJSON *group = cJSON_GetObjectItem(params, "group");
            if (cJSON_IsNumber(group)) {
                if (!tracking_in_progress && !deploy_in_progress && !stow_in_progress) {
                    select_motor_group((motor_group_t)group->valueint);
                } else {
                    ESP_LOGW(TAG, "Cannot change group - operation in progress");
                }
            }
        } else if (strcmp(command, "select_motors") == 0 && params) {
            cJSON *mask = cJSON_GetObjectItem(params, "mask");
            if (cJSON_IsNumber(mask)) {
                active_motors = mask->valueint & 0x07FF;
                
                for (int i = 0; i < 11; i++) {
                    motor_states[i].is_active = !!(active_motors & (1 << i));
                }
                
                if (active_group == GROUP_ELEVATION_PANELS) {
                    uint8_t relay_mask = 0;
                    if (active_motors & 0x0780) relay_mask |= 0x07;
                    set_expansion_relay(relay_mask);
                }
                
                ESP_LOGI(TAG, "Motor selection mask: 0x%04X", active_motors);
            }
        } else if (strcmp(command, "set_speed") == 0 && params) {
            cJSON *motor = cJSON_GetObjectItem(params, "motor");
            cJSON *speed = cJSON_GetObjectItem(params, "speed");
            if (cJSON_IsNumber(motor) && cJSON_IsNumber(speed)) {
                motor_states[motor->valueint].target_speed = speed->valueint;
            }
        } else if (strcmp(command, "go_to") == 0 && params) {
            cJSON *motor = cJSON_GetObjectItem(params, "motor");
            cJSON *position = cJSON_GetObjectItem(params, "position");
            cJSON *speed = cJSON_GetObjectItem(params, "speed");
            if (cJSON_IsNumber(motor) && cJSON_IsNumber(position)) {
                int m = motor->valueint;
                if (is_motor_active(m) && motors[m].type == MOTOR_TYPE_CLOSED_LOOP) {
                    motor_states[m].target_position = position->valuedouble;
                    motor_states[m].is_moving = true;
                    if (cJSON_IsNumber(speed)) {
                        int custom_speed = speed->valueint;
                        if (custom_speed > 20 && custom_speed <= 250) {
                            motor_states[m].max_speed = custom_speed;
                        }
                    } else {
                        motor_states[m].max_speed = MAX_SPEED;
                    }
                }
            }
        } else if (strcmp(command, "stop_all") == 0) {
            for (int i = 0; i < 11; i++) {
                set_motor_speed(i, 0);
                motor_states[i].is_moving = false;
                motor_states[i].target_speed = 0;
                motor_states[i].max_speed = MAX_SPEED;
            }
            sync_state.enabled = false;
            sync_state.is_moving = false;
            tracking_in_progress = false;
            deploy_in_progress = false;
            stow_in_progress = false;
            ESP_LOGI(TAG, "Emergency stop - all operations cancelled");
        } else if (strcmp(command, "zero_position") == 0 && params) {
            cJSON *motor = cJSON_GetObjectItem(params, "motor");
            if (cJSON_IsNumber(motor)) {
                int m = motor->valueint;
                if (m < 7 && motors[m].type == MOTOR_TYPE_CLOSED_LOOP) {
                    motor_states[m].position = 0;
                    motor_states[m].pulse_count = 0;
                    motor_states[m].pulse_offset = 0;
                    if (motors[m].pcnt_unit) {
                        pcnt_unit_clear_count(motors[m].pcnt_unit);
                    }
                    ESP_LOGI(TAG, "Motor %d (%s) position zeroed", m, motors[m].name);
                }
            }
        } else if (strcmp(command, "set_calibration") == 0 && params) {
            cJSON *motor = cJSON_GetObjectItem(params, "motor");
            cJSON *value = cJSON_GetObjectItem(params, "value");
            if (cJSON_IsNumber(motor) && cJSON_IsNumber(value)) {
                int m = motor->valueint;
                if (m < 7 && motors[m].type == MOTOR_TYPE_CLOSED_LOOP) {
                    motor_states[m].pulses_per_unit = value->valuedouble;
                    save_calibration();
                }
            }
        } else if (strcmp(command, "set_home_bearing") == 0 && params) {
            cJSON *bearing = cJSON_GetObjectItem(params, "bearing");
            if (cJSON_IsNumber(bearing)) {
                home_bearing = bearing->valuedouble;
                while (home_bearing < 0) home_bearing += 360.0f;
                while (home_bearing >= 360.0f) home_bearing -= 360.0f;
                save_home_bearing();
            }
        } else if (strcmp(command, "set_location") == 0 && params) {
            cJSON *lat = cJSON_GetObjectItem(params, "latitude");
            cJSON *lon = cJSON_GetObjectItem(params, "longitude");
            if (cJSON_IsNumber(lat) && cJSON_IsNumber(lon)) {
                latitude = lat->valuedouble;
                longitude = lon->valuedouble;
                if (latitude > 90.0f) latitude = 90.0f;
                if (latitude < -90.0f) latitude = -90.0f;
                while (longitude > 180.0f) longitude -= 360.0f;
                while (longitude < -180.0f) longitude += 360.0f;
                save_location();
            }
        } else if (strcmp(command, "sync_time") == 0 && params) {
            cJSON *timestamp = cJSON_GetObjectItem(params, "timestamp");
            if (cJSON_IsNumber(timestamp)) {
                struct timeval tv = {
                    .tv_sec = timestamp->valueint,
                    .tv_usec = 0
                };
                settimeofday(&tv, NULL);
                time_synced = true;
                ESP_LOGI(TAG, "Time synced");
            }
        } else if (strcmp(command, "auto_track") == 0 && params) {
            cJSON *enable = cJSON_GetObjectItem(params, "enable");
            if (cJSON_IsBool(enable)) {
                auto_tracking = cJSON_IsTrue(enable);
                ESP_LOGI(TAG, "Auto tracking %s", auto_tracking ? "enabled" : "disabled");
            }
        } else if (strcmp(command, "deploy") == 0) {
            if (!deploy_in_progress && !stow_in_progress && !tracking_in_progress) {
                xTaskCreate(deploy_task, "deploy", 4096, NULL, 4, NULL);
            } else {
                ESP_LOGW(TAG, "Cannot deploy - operation already in progress");
            }
        } else if (strcmp(command, "stow") == 0) {
            if (!deploy_in_progress && !stow_in_progress && !tracking_in_progress) {
                xTaskCreate(stow_task, "stow", 4096, NULL, 4, NULL);
            } else {
                ESP_LOGW(TAG, "Cannot stow - operation already in progress");
            }
        } else if (strcmp(command, "sync_lift") == 0 && params) {
            cJSON *height = cJSON_GetObjectItem(params, "height");
            cJSON *speed = cJSON_GetObjectItem(params, "speed");
            if (cJSON_IsNumber(height)) {
                select_motor_group(GROUP_LEGS);
                sync_state.enabled = true;
                sync_state.motor_mask = active_motors & 0x000F;
                sync_state.target_position = height->valuedouble;
                sync_state.tolerance = SYNC_TOLERANCE_LEG;
                sync_state.is_moving = true;
                
                if (cJSON_IsNumber(speed)) {
                    int custom_speed = speed->valueint;
                    if (custom_speed > 20 && custom_speed <= 250) {
                        for (int i = 0; i < 4; i++) {
                            motor_states[i].max_speed = custom_speed;
                        }
                    }
                }
                
                ESP_LOGI(TAG, "Synchronized lift to %.1fmm", height->valuedouble);
            }
        } else if (strcmp(command, "raise_legs") == 0 && params) {
            cJSON *target_distance = cJSON_GetObjectItem(params, "target_distance");
            cJSON *speed = cJSON_GetObjectItem(params, "speed");
            
            if (cJSON_IsNumber(target_distance)) {
                float distance_mm = target_distance->valuedouble;
                int motor_speed = 150;
                
                if (cJSON_IsNumber(speed)) {
                    motor_speed = speed->valueint;
                    if (motor_speed < 20) motor_speed = 20;
                    if (motor_speed > 250) motor_speed = 250;
                }
                
                if (tracking_in_progress || deploy_in_progress || stow_in_progress) {
                    ESP_LOGW(TAG, "Cannot raise legs - operation in progress");
                } else {
                    ESP_LOGI(TAG, "Raise legs command: distance=%.1fmm, speed=%d", distance_mm, motor_speed);
                    xTaskCreate(raise_legs_task, "raise_legs", 4096, 
                               (void*)((uint32_t)(distance_mm * 100) | (motor_speed << 16)), 4, NULL);
                }
            } else {
                ESP_LOGW(TAG, "Raise legs command missing target_distance parameter");
            }
        } else if (strcmp(command, "drop_legs") == 0 && params) {
            cJSON *target_distance = cJSON_GetObjectItem(params, "target_distance");
            cJSON *speed = cJSON_GetObjectItem(params, "speed");
            
            if (cJSON_IsNumber(target_distance)) {
                float distance_mm = target_distance->valuedouble;
                int motor_speed = -150;
                
                if (cJSON_IsNumber(speed)) {
                    motor_speed = speed->valueint;
                    if (motor_speed > -20) motor_speed = -20;
                    if (motor_speed < -250) motor_speed = -250;
                }
                
                if (tracking_in_progress || deploy_in_progress || stow_in_progress) {
                    ESP_LOGW(TAG, "Cannot drop legs - operation in progress");
                } else {
                    ESP_LOGI(TAG, "Drop legs command: distance=%.1fmm, speed=%d", distance_mm, motor_speed);
                    xTaskCreate(drop_legs_task, "drop_legs", 4096, 
                               (void*)((uint32_t)(distance_mm * 100) | (motor_speed << 16)), 4, NULL);
                }
            } else {
                ESP_LOGW(TAG, "Drop legs command missing target_distance parameter");
            }
        } else if (strcmp(command, "set_sync_mode") == 0 && params) {
            cJSON *enable = cJSON_GetObjectItem(params, "enable");
            if (cJSON_IsBool(enable)) {
                sync_mode = cJSON_IsTrue(enable);
                if (!sync_mode) {
                    sync_state.enabled = false;
                    sync_state.is_moving = false;
                }
                ESP_LOGI(TAG, "Sync mode %s", sync_mode ? "enabled" : "disabled");
            }
        } else if (strcmp(command, "deploy_with_time") == 0 && params) {
            cJSON *timestamp = cJSON_GetObjectItem(params, "timestamp");
            
            if (!deploy_in_progress && !stow_in_progress && !tracking_in_progress) {
                if (cJSON_IsNumber(timestamp)) {
                    struct timeval tv = {
                        .tv_sec = timestamp->valueint,
                        .tv_usec = 0
                    };
                    settimeofday(&tv, NULL);
                    time_synced = true;
                    
                    time_t received_time = timestamp->valueint;
                    struct tm timeinfo;
                    localtime_r(&received_time, &timeinfo);
                    char time_str[64];
                    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
                    ESP_LOGI(TAG, "✓ Time synced from deploy command: %s (timestamp: %d)", time_str, timestamp->valueint);
                    
                    if (Deploy_All_API_Enable_Flag) {
                        sync_mode = true;
                        ESP_LOGI(TAG, "✓ Synchronized movement mode enabled");
                    }
                } else {
                    ESP_LOGW(TAG, "Deploy command received without timestamp");
                }
                
                ESP_LOGI(TAG, "Starting deploy task from deploy_with_time command");
                xTaskCreate(deploy_task, "deploy", 4096, NULL, 4, NULL);
            } else {
                ESP_LOGW(TAG, "Cannot deploy - operation already in progress");
            }
        } else if (strcmp(command, "stop_button") == 0) {
            ESP_LOGI(TAG, "=== EMERGENCY STOP INITIATED ===");
            
            for (int i = 0; i < 11; i++) {
                emergency_stop_motor(i);
            }
            
            sync_state.enabled = false;
            sync_state.is_moving = false;
            tracking_in_progress = false;
            deploy_in_progress = false;
            stow_in_progress = false;
            
            for (int i = 0; i < 11; i++) {
                motor_states[i].is_moving = false;
                motor_states[i].target_speed = 0;
                motor_states[i].max_speed = MAX_SPEED;
                motor_states[i].last_direction = 0;
            }
            
            mcp_portb &= ~0x87;
            mcp23017_write(MCP23017_OLATB, mcp_portb);
            
            mcp_porta &= ~0x10;
            mcp23017_write(MCP23017_OLATA, mcp_porta);
            
            ESP_LOGI(TAG, "=== EMERGENCY STOP COMPLETED - ALL MOTORS STOPPED ===");
            ESP_LOGI(TAG, "All relays disabled for safety");
        }
    }
    
    cJSON_Delete(json);
    httpd_resp_send(req, "{\"ok\":true}", 11);
    return ESP_OK;
}

// Initialize web server
void web_server_init(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    ESP_ERROR_CHECK(httpd_start(&web_server, &config));
    
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
    };
    httpd_register_uri_handler(web_server, &index_uri);
    
    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = get_status_handler,
    };
    httpd_register_uri_handler(web_server, &status_uri);
    
    httpd_uri_t command_uri = {
        .uri = "/command",
        .method = HTTP_POST,
        .handler = command_handler,
    };
    httpd_register_uri_handler(web_server, &command_uri);
    
    httpd_uri_t api_status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = api_status_handler,
    };
    httpd_register_uri_handler(web_server, &api_status_uri);
    
    httpd_uri_t api_debug_uri = {
        .uri = "/api/debug",
        .method = HTTP_GET,
        .handler = api_debug_handler,
    };
    httpd_register_uri_handler(web_server, &api_debug_uri);
    
    // OTA handlers
    httpd_uri_t ota_upload_uri = {
        .uri = "/api/ota/upload",
        .method = HTTP_POST,
        .handler = ota_upload_handler,
        .user_ctx = NULL
    };

    httpd_uri_t ota_page_uri = {
        .uri = "/ota",
        .method = HTTP_GET,
        .handler = ota_page_handler,
        .user_ctx = NULL
    };

    httpd_uri_t ota_status_uri = {
        .uri = "/api/ota/upload/status",
        .method = HTTP_GET,
        .handler = ota_upload_status_handler,
        .user_ctx = NULL
    };

    if (web_server) {
        httpd_register_uri_handler(web_server, &ota_upload_uri);
        httpd_register_uri_handler(web_server, &ota_page_uri);
        httpd_register_uri_handler(web_server, &ota_status_uri);
    } else {
        ESP_LOGE(TAG, "Web server failed to start — OTA handlers not registered");
    }
    
    ESP_LOGI(TAG, "Web server initialized on port 80");
}