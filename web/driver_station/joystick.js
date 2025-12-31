export class VirtualJoystick {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.knob = this.container.querySelector('.joystick-knob');
        this.base = this.container.querySelector('.joystick-base');
        
        this.active = false;
        this.pointerId = null;
        
        // Coordinates normalized -1 to 1
        // Y is inverted (up is -1 in screen coords usually, but for robot drive forward:
        // Let's standard: 
        //   Forward (Up) -> +1.0 
        //   Backward (Down) -> -1.0
        //   Left -> +1.0 (Turn Rate) ? Usually Left Turn is +Z angular velocity.
        // Let's stick to standard math: 
        //   X: -1 (Left) to 1 (Right)
        //   Y: -1 (Down) to 1 (Up)
        this.valX = 0;
        this.valY = 0;

        this.rect = null;
        this.maxRadius = 0;

        this.init();
    }

    init() {
        // Prevent default touch actions on the container
        this.container.addEventListener('pointerdown', this.onDown.bind(this));
        this.container.addEventListener('pointermove', this.onMove.bind(this));
        this.container.addEventListener('pointerup', this.onUp.bind(this));
        this.container.addEventListener('pointercancel', this.onUp.bind(this));
        
        // Recalculate dimensions on resize
        window.addEventListener('resize', () => this.updateDimensions());
        // Initial calc
        setTimeout(() => this.updateDimensions(), 100);
    }

    updateDimensions() {
        this.rect = this.base.getBoundingClientRect();
        // knob moves within the base. Max travel is radius of base - radius of knob
        // But for visual simplicity, let's say max travel is half width of base.
        this.maxRadius = this.rect.width / 2;
        this.centerX = this.rect.left + this.rect.width / 2;
        this.centerY = this.rect.top + this.rect.height / 2;
    }

    onDown(e) {
        this.active = true;
        this.pointerId = e.pointerId;
        this.container.setPointerCapture(this.pointerId);
        this.updateDimensions(); // Ensure correct rect
        this.processMove(e.clientX, e.clientY);
    }

    onMove(e) {
        if (!this.active) return;
        if (e.pointerId !== this.pointerId) return;
        this.processMove(e.clientX, e.clientY);
    }

    onUp(e) {
        if (!this.active) return;
        if (e.pointerId !== this.pointerId) return;
        
        this.active = false;
        this.pointerId = null;
        this.container.releasePointerCapture(e.pointerId);
        this.reset();
    }

    processMove(clientX, clientY) {
        let dx = clientX - this.centerX;
        let dy = clientY - this.centerY;
        
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        // Clamp to max radius
        if (distance > this.maxRadius) {
            const ratio = this.maxRadius / distance;
            dx *= ratio;
            dy *= ratio;
        }

        // Update visuals
        this.knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

        // Normalize values (-1 to 1)
        // Y: Up on screen is negative pixels, so we invert it to make Up positive
        this.valX = dx / this.maxRadius;
        this.valY = -(dy / this.maxRadius); 

        // Deadband (small)
        if (Math.abs(this.valX) < 0.05) this.valX = 0;
        if (Math.abs(this.valY) < 0.05) this.valY = 0;
    }

    reset() {
        this.valX = 0;
        this.valY = 0;
        this.knob.style.transform = `translate(-50%, -50%)`;
    }

    getPosition() {
        return { x: this.valX, y: this.valY };
    }
}
