export class RoverWS extends EventTarget {
    constructor() {
        super();
        this.socket = null;
        this.reconnectInterval = 2000;
        const wsProto = window.location.protocol === 'https:' ? 'wss' : 'ws';
        this.url = `${wsProto}://${window.location.host}/ws`;
        // For development/testing if served from file or different port
        if (window.location.protocol === 'file:') {
            console.warn("WS: Running from file, defaulting to localhost:8080");
            this.url = 'ws://localhost:8090/ws';
        }
    }

    connect() {
        console.log(`WS: Connecting to ${this.url}...`);
        this.socket = new WebSocket(this.url);

        this.socket.onopen = () => {
            console.log("WS: Connected");
            this.dispatchEvent(new Event('open'));
        };

        this.socket.onclose = () => {
            console.log("WS: Disconnected");
            this.dispatchEvent(new Event('close'));
            setTimeout(() => this.connect(), this.reconnectInterval);
        };

        this.socket.onerror = (err) => {
            console.error("WS: Error", err);
        };

        this.socket.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                this.dispatchEvent(new CustomEvent('message', { detail: data }));
            } catch (e) {
                console.error("WS: Failed to parse message", event.data);
            }
        };
    }

    send(data) {
        if (this.socket && this.socket.readyState === WebSocket.OPEN) {
            this.socket.send(JSON.stringify(data));
        }
    }
}
