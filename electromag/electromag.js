const net = require('net');
const SLAVE_PORT = 8084;

module.exports = class {

    constructor() {
        this._socket = net.connect(SLAVE_PORT, 'localhost');
    }

    on() {
        return new Promise((resolve, reject) => {
            console.log('setting electromag on');
            this._socket.once('data', data => resolve(data.toString()));
            this._socket.write('1');
            setTimeout(reject, 1000);
        })
    }

    off() {
        return new Promise((resolve, reject) => {
            console.log('setting electromag off');
            this._socket.once('data', data => resolve(data.toString()));
            this._socket.write('0');
            setTimeout(reject, 1000);
        })
    }

};

if (require.main === module) {
    const electromag = new module.exports();
    setInterval(async () => {
        console.log(await electromag.on());
    }, 1000);
}
