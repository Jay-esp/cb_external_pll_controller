const char index_html[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>CB Master</title>
  <link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&display=swap" rel="stylesheet">
  <style>
  @font-face {
      font-family: 'DSEG7';
         src: url('data:font/woff2;base64,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') format('woff2');
        font-weight: normal;
      font-style: normal;
    }
    body { margin: 20px; background: #000; color: #fff; font-family: sans-serif; }
    /* Main layout */
    .main-container {
      display: flex;
      max-width: 800px;
      margin: 0 auto;
      gap: 30px;
    }
    .left-panel {
      flex: 3;
      text-align: center;
    }
    .right-panel {
      flex: 1;
      min-width: 200px;
      display: flex;
      flex-direction: column;
    }

    /* Boven en onder display */
    .upper-display {
      font-family: 'DSEG7', 'Orbitron', monospace;
      font-size: 120px;
      font-weight: normal;
      color: #ff0000;
      margin: 20px 0;
     /* letter-spacing: 5px;*/
    }
    .lower-display {
 font-family: 'DSEG7', 'Orbitron', monospace;
      font-size: 65px;
      font-weight: normal;
      color: #ffffff;
      margin: 10px 0;
     /* letter-spacing: 3px; */
    }

    /* Button styling */
    button {
      margin: 5px;
      padding: 15px 10px;
      font-size: 16px;
      background: #333;
      color: white;
      border: none;
      border-radius: 5px;
      box-sizing: border-box;
      -webkit-user-select: none;
      -moz-user-select: none;
      -ms-user-select: none;
      user-select: none;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    button:hover { background: #555; }

    /* Arrow buttons */
    .arrow-btn {
      width: 80px;
      height: 80px;
      font-size: 40px;
    }
    .arrow-up::after {
      content: "▲";
    }
    .arrow-down::after {
      content: "▼";
    }

    /* Preset buttons */
    .preset-btn {
      width: 140px;
      height: 55px;
    }

    /* Toggle mode button */
    .toggle-btn {
      width: 140px;
      height: 55px;
    }

    /* S-meter styling */
    .smeter-container { margin: 10px auto 30px auto; width: 362px; }
    .smeter-scale {
      position: relative;
      width: 362px;
      height: 36px;
      border: 2px solid white;
      background: black;
      margin-bottom: 38px;
    }
    .smeter-fill {
      position: absolute;
      left: 1px;
      top: 1px;
      height: 34px;
      width: 0%;
      transition: width 0.2s ease;
    }
    .smeter-markers {
      position: absolute;
      bottom: -14px;
      left: 0;
      width: 100%;
      height: 14px;
    }
    .smeter-marker {
      position: absolute;
      width: 2px;
      height: 14px;
      background: #ccc;
      transform: translateX(-50%);
    }
    .smeter-labels {
      display: flex;
      justify-content: space-between;
      width: 362px;
      margin-top: -6px;
      font-size: 12px;
      position: relative;
    }
    .smeter-label {
      transform: translateX(-50%);
      color: #ccc;
      position: absolute;
    }

    /* Button containers */
    .channel-control {
      display: flex;
      justify-content: center;
      gap: 20px;
      margin: 20px 0;
    }
    .preset-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 8px;
      margin-top: 60px; /* PRESETS HOGER - was 85px */
    }
    .toggle-container {
      display: flex;
      justify-content: center;
      margin-top: 15px;
    }
  </style>
</head>
<body>
  <div class="main-container">
    <!-- Linker paneel - Hoofd content -->
    <div class="left-panel">
      <h1>CB Master Control</h1>

      <!-- S-meter -->
      <div class="smeter-container">
        <div class="smeter-scale">
          <div class="smeter-markers" id="smeter-markers"></div>
          <div class="smeter-fill" id="smeter-fill"></div>
        </div>
        <div class="smeter-labels" id="smeter-labels"></div>
      </div>

      <!-- Boven en onder displays -->
      <div class="upper-display"><span id="boven-text">20</span></div>
      <div class="lower-display"><span id="onder-text">27.205</span></div>

      <!-- Channel control buttons -->
      <div class="channel-control">
        <button class="arrow-btn arrow-down"
                onmousedown="(function(cmd){window.cmd=cmd;sendCommand(cmd);window.rt=setInterval(function(){sendCommand(cmd)},500)})(this.getAttribute('data-cmd'))"
                onmouseup="clearInterval(window.rt)"
                ontouchstart="(function(cmd){event.preventDefault();window.cmd=cmd;sendCommand(cmd);window.rt=setInterval(function(){sendCommand(cmd)},500)})(this.getAttribute('data-cmd'))"
                ontouchend="clearInterval(window.rt)"
                data-cmd="CHAN_DOWN"></button>

        <button class="arrow-btn arrow-up"
                onmousedown="(function(cmd){window.cmd=cmd;sendCommand(cmd);window.rt=setInterval(function(){sendCommand(cmd)},500)})(this.getAttribute('data-cmd'))"
                onmouseup="clearInterval(window.rt)"
                ontouchstart="(function(cmd){event.preventDefault();window.cmd=cmd;sendCommand(cmd);window.rt=setInterval(function(){sendCommand(cmd)},500)})(this.getAttribute('data-cmd'))"
                ontouchend="clearInterval(window.rt)"
                data-cmd="CHAN_UP"></button>
      </div>
    </div>

    <!-- Rechter paneel - Presets + Toggle button -->
    <div class="right-panel">
      <div class="preset-container">
        <button class="preset-btn" id="preset0">STD</button>
        <button class="preset-btn" id="preset1">P1</button>
        <button class="preset-btn" id="preset2">P2</button>
        <button class="preset-btn" id="preset3">P3</button>
        <button class="preset-btn" id="preset4">P4</button>
      </div>

      <!-- NIEUW: Toggle mode button -->
      <div class="toggle-container">
        <button class="toggle-btn" onclick="sendCommand('TOGGLE_MODE')" id="toggle-btn">C/F</button>
      </div>
    </div>
  </div>

  <script>
    function sendCommand(cmd) {
      console.log('Verzenden:', cmd);
      if (window.ws && window.ws.readyState === WebSocket.OPEN) {
        window.ws.send(cmd);
      } else {
        console.error('WebSocket niet verbonden');
      }
    }

    function initSMeter() {
      const labels = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "+10", "+20", "+30"];
      const markersContainer = document.getElementById('smeter-markers');
      const labelsContainer = document.getElementById('smeter-labels');
      const scaleWidth = 362;

      for (let i = 0; i < labels.length; i++) {
        const position = (i / (labels.length - 1)) * scaleWidth;

        const marker = document.createElement('div');
        marker.className = 'smeter-marker';
        marker.style.left = position + 'px';
        markersContainer.appendChild(marker);

        const label = document.createElement('span');
        label.className = 'smeter-label';
        label.textContent = labels[i];
        label.style.left = position + 'px';
        labelsContainer.appendChild(label);
      }
    }

    function updateSMeter(sValue) {
      const scaleWidth = 362;
      const totalWidth = scaleWidth - 2;
      const s9Position = (9 / 12) * totalWidth;

      const totalNeededWidth = (sValue / 12) * totalWidth;
      const fillElement = document.getElementById('smeter-fill');

      fillElement.style.width = totalNeededWidth + 'px';

      if (sValue <= 9) {
        fillElement.style.background = 'green';
      } else {
        fillElement.style.background = `linear-gradient(to right, green 0%, green ${s9Position}px, red ${s9Position}px, red 100%)`;
      }
    }

    function initPresetButtons() {
      for (let i = 0; i < 5; i++) {
        document.getElementById('preset' + i).addEventListener('click', function() {
          sendCommand('PRESET_' + i);
        });
      }
    }

    function connectWebSocket() {
      console.log('Starting WebSocket connection...');
      window.ws = new WebSocket('ws://'+location.host+'/ws');

      window.ws.onopen = function() {
        console.log('✅ WebSocket verbonden');
      };

      window.ws.onmessage = function(e) {
        console.log('Ontvangen:', e.data);
        try {
          var d = JSON.parse(e.data);

          // Update boven/onder displays
          document.getElementById('boven-text').textContent = d.boven;
          document.getElementById('onder-text').textContent = d.onder;

          // Update S-meter
          updateSMeter(d.s);

          // Update preset button teksten (als ze meegegeven worden)
          if (d.presets && Array.isArray(d.presets)) {
            for (let i = 0; i < 5; i++) {
              const btn = document.getElementById('preset' + i);
              if (btn && d.presets[i]) {
                btn.textContent = d.presets[i];
              }
            }
          }

          // Update toggle button tekst op basis van mode (optioneel)
          if (d.mode !== undefined) {
            const toggleBtn = document.getElementById('toggle-btn');
            if (toggleBtn) {
              toggleBtn.textContent = d.mode === 1 ? 'C/F' : 'F/C';
            }
          }

        } catch (error) {
          console.error('Fout bij verwerken data:', error);
        }
      };

      window.ws.onerror = function(error) {
        console.error('WebSocket error:', error);
      };

      window.ws.onclose = function(event) {
        console.log('WebSocket closed, reconnecting in 2s...', event.code, event.reason);
        setTimeout(connectWebSocket, 2000);
      };
    }

    document.addEventListener('DOMContentLoaded', function() {
      console.log('DOM geladen, initialiseren...');
      initSMeter();
      initPresetButtons();

      // Start WebSocket verbinding NA DOM ready
      setTimeout(connectWebSocket, 100);
    });
  </script>
</body>
</html>
)=====";
