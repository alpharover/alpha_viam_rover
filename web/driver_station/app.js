import { RoverWS } from './ws.js';
import { VirtualJoystick } from './joystick.js';

function clampNumber(value, lo, hi) {
    const v = Number(value);
    if (Number.isNaN(v)) return lo;
    return Math.min(hi, Math.max(lo, v));
}

function fmtNumber(value, digits, fallback = '--') {
    if (typeof value !== 'number' || Number.isNaN(value)) return fallback;
    return value.toFixed(digits);
}

class DriverStation {
    constructor() {
        this.ws = new RoverWS();
        this.joystick = new VirtualJoystick('joystick-zone');

        this.state = {
            armed: false,
            seq: 0,
            limits: {
                max_speed: 1.0,
                max_turn: 6.5,
                deadband: 0.08,
                expo: 1.0,
            },
            optic: {
                view: 'video',
                raw: false,
                mjpeg_direct_url: null,
                mjpeg_proxy_url: '/mjpeg',
                ascii_cols: 120,
                ascii_fps: 10,
            },
        };

        this.elements = {
            statusLink: document.getElementById('status-link'),
            statusArmed: document.getElementById('status-armed'),
            video: document.getElementById('video-stream'),
            asciiStream: document.getElementById('ascii-stream'),
            hudVideo: document.querySelector('.hud-video'),
            btnOpticRaw: document.getElementById('btn-optic-raw'),
            btnOpticMode: document.getElementById('btn-optic-mode'),
            btnHold: document.getElementById('btn-drive-hold'),
            cmdLin: document.getElementById('cmd-lin'),
            cmdAng: document.getElementById('cmd-ang'),

            // Stats
            valBusV: document.getElementById('val-bus-v'),
            valCurr: document.getElementById('val-curr'),
            valPower: document.getElementById('val-power'),
            valCpu: document.getElementById('val-cpu-temp'),
            valWifi: document.getElementById('val-wifi-sig'),
            valMem: document.getElementById('val-mem'),
            valOdomVx: document.getElementById('val-odom-vx'),
            valOdomWz: document.getElementById('val-odom-wz'),
            valWheelL: document.getElementById('val-wheel-l'),
            valWheelR: document.getElementById('val-wheel-r'),
            valWheelDiff: document.getElementById('val-wheel-diff'),

            // Sliders
            sliderSpeed: document.getElementById('slider-speed'),
            sliderTurn: document.getElementById('slider-turn'),
            dispSpeed: document.getElementById('disp-speed'),
            dispTurn: document.getElementById('disp-turn'),
        };

        this._asciiTimer = null;
        this._asciiCanvas = null;
        this._asciiCtx = null;

        this.lastSendMs = 0;

        this.init();
    }

    init() {
        // Setup WS listeners
        this.ws.addEventListener('open', () => {
            this.elements.statusLink.textContent = 'CONNECTED';
            this.elements.statusLink.className = 'value online';

            // Push current limits when we connect.
            this.sendLimits();
        });

        this.ws.addEventListener('close', () => {
            this.elements.statusLink.textContent = 'DISCONNECTED';
            this.elements.statusLink.className = 'value offline';
            this.setArmed(false);
        });

        this.ws.addEventListener('message', (e) => this.handleMessage(e.detail));

        // Connect WS
        this.ws.connect();

        // Setup Safety Button (dead-man switch)
        this.setupHoldToDrive();

        // Setup sliders
        this.setupSliders();

        this.setupOpticControls();

        if (!this.state.optic.mjpeg_direct_url) {
            this.state.optic.mjpeg_direct_url = `http://${window.location.hostname}:8080/stream`;
        }
        this.applyOpticSource();

        // Start loop
        this.loop();
    }

    setupHoldToDrive() {
        const btn = this.elements.btnHold;

        const toggleArm = (e) => {
            e.preventDefault();
            this.setArmed(!this.state.armed);
        };

        btn.addEventListener('click', toggleArm);

        btn.oncontextmenu = (e) => {
            e.preventDefault();
            e.stopPropagation();
            return false;
        };
    }

    setupSliders() {
        this.elements.sliderSpeed.addEventListener('input', () => this.updateLimitsFromUI());
        this.elements.sliderTurn.addEventListener('input', () => this.updateLimitsFromUI());

        this.updateLimitsFromUI();
    }

    setupOpticControls() {
        const container = this.elements.hudVideo;
        const btnRaw = this.elements.btnOpticRaw;
        const btnMode = this.elements.btnOpticMode;

        if (!container || !btnRaw || !btnMode) return;

        btnRaw.addEventListener('click', (e) => {
            e.preventDefault();
            if (this.state.optic.view !== 'video') return;
            this.setOpticRaw(!this.state.optic.raw);
        });

        btnMode.addEventListener('click', (e) => {
            e.preventDefault();
            this.setOpticView(this.state.optic.view === 'ascii' ? 'video' : 'ascii');
        });

        this.applyOpticState();
        this.applyOpticSource();
    }

    setOpticRaw(raw) {
        this.state.optic.raw = Boolean(raw);
        this.applyOpticState();
    }

    setOpticView(view) {
        const next = view === 'ascii' ? 'ascii' : 'video';
        if (this.state.optic.view === next) return;
        this.state.optic.view = next;

        if (next === 'ascii') {
            this.state.optic.raw = false;
            this.startAscii();
        } else {
            this.stopAscii();
        }

        this.applyOpticState();
        this.applyOpticSource();
    }

    applyOpticState() {
        const container = this.elements.hudVideo;
        if (!container) return;

        const isAscii = this.state.optic.view === 'ascii';
        container.classList.toggle('is-ascii', isAscii);
        container.classList.toggle('is-raw', this.state.optic.raw && !isAscii);

        const btnRaw = this.elements.btnOpticRaw;
        if (btnRaw) {
            btnRaw.disabled = isAscii;
            btnRaw.textContent = this.state.optic.raw ? 'RAW: ON' : 'RAW: OFF';
        }

        const btnMode = this.elements.btnOpticMode;
        if (btnMode) {
            btnMode.textContent = isAscii ? 'MODE: ASCII' : 'MODE: VIDEO';
        }
    }

    applyOpticSource() {
        const img = this.elements.video;
        if (!img) return;

        const wantAscii = this.state.optic.view === 'ascii';
        const direct = this.state.optic.mjpeg_direct_url;
        const proxy = this.state.optic.mjpeg_proxy_url;

        const target = wantAscii ? proxy : direct;
        if (typeof target === 'string' && target && img.src !== target) {
            img.src = target;
        }
    }

    startAscii() {
        const pre = this.elements.asciiStream;
        if (pre) pre.textContent = 'LOADING...';

        if (this._asciiTimer) return;

        this._asciiCanvas = this._asciiCanvas || document.createElement('canvas');
        this._asciiCtx = this._asciiCanvas.getContext('2d');

        if (!this._asciiCtx) return;

        const fps = Math.max(1, this.state.optic.ascii_fps);
        const intervalMs = Math.max(50, Math.round(1000 / fps));
        this._asciiTimer = window.setInterval(() => this.renderAsciiFrame(), intervalMs);
    }

    stopAscii() {
        if (this._asciiTimer) {
            window.clearInterval(this._asciiTimer);
            this._asciiTimer = null;
        }
    }

    renderAsciiFrame() {
        if (this.state.optic.view !== 'ascii') return;

        const img = this.elements.video;
        const pre = this.elements.asciiStream;
        const ctx = this._asciiCtx;

        if (!img || !pre || !ctx || !this._asciiCanvas) return;
        if (!img.naturalWidth || !img.naturalHeight) return;

        const cols = Math.max(20, Math.floor(this.state.optic.ascii_cols));
        const aspect = img.naturalWidth / img.naturalHeight;
        const rows = Math.max(10, Math.round((cols / aspect) * 0.55));

        const container = pre.parentElement;
        const containerWidth = container ? container.clientWidth : 800;
        const containerHeight = container ? container.clientHeight : 600;
        const fontSize = Math.min(containerWidth / (cols * 0.6), containerHeight / rows);
        pre.style.fontSize = fontSize + 'px';
        pre.style.lineHeight = fontSize + 'px';

        this._asciiCanvas.width = cols;
        this._asciiCanvas.height = rows;

        try {
            ctx.drawImage(img, 0, 0, cols, rows);
            const data = ctx.getImageData(0, 0, cols, rows).data;

            const ramp = '@%#*+=-:. ';
            const rampLen = ramp.length;

            const lines = new Array(rows);
            let p = 0;
            for (let y = 0; y < rows; y++) {
                let row = '';
                for (let x = 0; x < cols; x++) {
                    const r = data[p];
                    const g = data[p + 1];
                    const b = data[p + 2];
                    p += 4;

                    const l = 0.2126 * r + 0.7152 * g + 0.0722 * b;
                    const idx = Math.max(0, Math.min(rampLen - 1, Math.round((rampLen - 1) * (l / 255))));
                    row += ramp[idx];
                }
                lines[y] = row;
            }
            pre.textContent = lines.join('\n');
        } catch (e) {
            pre.textContent = 'ASCII_UNAVAILABLE';
            this.stopAscii();
        }
    }

    updateLimitsFromUI() {
        const speed = parseFloat(this.elements.sliderSpeed.value);
        const turn = parseFloat(this.elements.sliderTurn.value);

        // These are physical limits (m/s, rad/s).
        this.state.limits.max_speed = Math.max(0, speed);
        this.state.limits.max_turn = Math.max(0, turn);

        this.elements.dispSpeed.textContent = this.state.limits.max_speed.toFixed(2);
        this.elements.dispTurn.textContent = this.state.limits.max_turn.toFixed(2);

        this.sendLimits();
    }

    applyHelloConfig(cfg) {
        if (!cfg) return;

        if (cfg.video) {
            if (cfg.video.mjpeg_stream_url) {
                this.state.optic.mjpeg_direct_url = cfg.video.mjpeg_stream_url;
            }
            if (cfg.video.mjpeg_proxy_url) {
                this.state.optic.mjpeg_proxy_url = cfg.video.mjpeg_proxy_url;
            }
            this.applyOpticSource();
        }

        // Limits
        const minSpeed = typeof cfg.min_speed === 'number' ? cfg.min_speed : 0.0;
        const minTurn = typeof cfg.min_turn === 'number' ? cfg.min_turn : 0.0;

        const curMaxSpeed = typeof cfg.max_speed === 'number' ? cfg.max_speed : this.state.limits.max_speed;
        const curMaxTurn = typeof cfg.max_turn === 'number' ? cfg.max_turn : this.state.limits.max_turn;

        this.state.limits.deadband = typeof cfg.deadband === 'number' ? cfg.deadband : this.state.limits.deadband;
        this.state.limits.expo = typeof cfg.expo === 'number' ? cfg.expo : this.state.limits.expo;

        // UI ranges: keep sane caps so we don't accidentally create a "missile" UI.
        const speedMaxUi = clampNumber(Math.max(curMaxSpeed, minSpeed) * 2.0, minSpeed, 2.0);
        const turnMaxUi = clampNumber(Math.max(curMaxTurn, minTurn) * 2.0, minTurn, 12.0);

        this.elements.sliderSpeed.min = String(minSpeed);
        this.elements.sliderSpeed.max = String(speedMaxUi);
        this.elements.sliderSpeed.step = '0.01';
        this.elements.sliderSpeed.value = String(clampNumber(curMaxSpeed, minSpeed, speedMaxUi));

        this.elements.sliderTurn.min = String(minTurn);
        this.elements.sliderTurn.max = String(turnMaxUi);
        this.elements.sliderTurn.step = '0.01';
        this.elements.sliderTurn.value = String(clampNumber(curMaxTurn, minTurn, turnMaxUi));

        // Pull values from sliders to state + send to backend.
        this.updateLimitsFromUI();
    }

    sendLimits() {
        this.ws.send({
            type: 'set_limits',
            max_speed: this.state.limits.max_speed,
            max_turn: this.state.limits.max_turn,
            deadband: this.state.limits.deadband,
            expo: this.state.limits.expo,
        });
    }

    setArmed(armed) {
        if (this.state.armed === armed) return;
        this.state.armed = armed;

        const btnText = this.elements.btnHold.querySelector('.btn-text');

        if (armed) {
            this.elements.statusArmed.textContent = 'ARMED';
            this.elements.statusArmed.className = 'value armed blink';
            this.elements.btnHold.classList.add('active');
            if (btnText) btnText.textContent = 'DISARM SYSTEM';
            this.ws.send({ type: 'arm', armed: true });
        } else {
            this.elements.statusArmed.textContent = 'SAFE';
            this.elements.statusArmed.className = 'value disarmed';
            this.elements.btnHold.classList.remove('active');
            if (btnText) btnText.textContent = 'ARM SYSTEM';
            this.ws.send({ type: 'arm', armed: false });
            // Stop immediately (extra safety; backend also enforces deadman).
            this.ws.send({ type: 'cmd', lin: 0.0, ang: 0.0, seq: ++this.state.seq });
        }
    }

    handleMessage(msg) {
        if (!msg || !msg.type) return;

        if (msg.type === 'hello') {
            if (msg.config) {
                this.applyHelloConfig(msg.config);
            }
            return;
        }

        if (msg.type === 'stats') {
            this.updateStats(msg.stats);
            return;
        }
    }

    updateStats(stats) {
        if (!stats) return;

        const p = stats.power || {};
        this.elements.valBusV.textContent = fmtNumber(p.bus_voltage, 2, '--.--');
        this.elements.valCurr.textContent = fmtNumber(p.current, 2, '--.--');
        this.elements.valPower.textContent = fmtNumber(p.power, 2, '--.--');

        const s = stats.system || {};
        this.elements.valCpu.textContent = fmtNumber(s.cpu_temp_c, 1, '--');
        this.elements.valMem.textContent = fmtNumber(s.mem_used_mb, 0, '--');

        const w = stats.wifi || {};
        if (typeof w.signal_dbm === 'number') {
            this.elements.valWifi.textContent = w.signal_dbm.toFixed(0);
        } else {
            this.elements.valWifi.textContent = '--';
        }

        // If backend reports actual published command, display it.
        const c = stats.control || {};
        if (typeof c.cmd_linear === 'number') {
            this.elements.cmdLin.textContent = c.cmd_linear.toFixed(2);
        }
        if (typeof c.cmd_angular === 'number') {
            this.elements.cmdAng.textContent = c.cmd_angular.toFixed(2);
        }

        const od = stats.odom || {};
        if (this.elements.valOdomVx) {
            this.elements.valOdomVx.textContent = fmtNumber(od.linear_x, 2, '--.--');
        }
        if (this.elements.valOdomWz) {
            this.elements.valOdomWz.textContent = fmtNumber(od.angular_z, 2, '--.--');
        }

        const d = stats.drive || {};
        if (this.elements.valWheelL) {
            this.elements.valWheelL.textContent = fmtNumber(d.wheel_left_vel_rad_s, 2, '--.--');
        }
        if (this.elements.valWheelR) {
            this.elements.valWheelR.textContent = fmtNumber(d.wheel_right_vel_rad_s, 2, '--.--');
        }
        if (this.elements.valWheelDiff) {
            this.elements.valWheelDiff.textContent = fmtNumber(d.wheel_vel_diff_rad_s, 2, '--.--');
        }
    }

    loop() {
        requestAnimationFrame(() => this.loop());

        if (!this.state.armed) return;

        // Limit send rate to ~30Hz.
        const now = Date.now();
        if (now - this.lastSendMs < 33) return;
        this.lastSendMs = now;

        const pos = this.joystick.getPosition();

        // Normalized commands in [-1, 1].
        // Y: + is forward. X: + is right, but ROS angular.z + is left.
        const linNorm = clampNumber(pos.y, -1.0, 1.0);
        const angNorm = clampNumber(-pos.x, -1.0, 1.0);

        // Show desired input briefly until stats arrives.
        if (Number.isFinite(linNorm)) this.elements.cmdLin.textContent = linNorm.toFixed(2);
        if (Number.isFinite(angNorm)) this.elements.cmdAng.textContent = angNorm.toFixed(2);

        this.ws.send({
            type: 'cmd',
            lin: linNorm,
            ang: angNorm,
            seq: ++this.state.seq,
        });
    }
}

window.addEventListener('DOMContentLoaded', () => {
    new DriverStation();
});
