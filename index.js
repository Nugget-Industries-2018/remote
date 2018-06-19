/**
 * Nugget Industries
 * 2017
 *
 * index.js
 * for running on the robot
 *
 * COMMAND LINE ARGUMENTS:
 * -d | --debug:
 *   runs the script without actually trying to read from sensors, sends made up values instead
 * -l | --local:
 *   runs the robot on localhost, should be run with --debug
 */

// dependencies
const net = require('net');
const EventEmitter = require('events');
const yargs = require('yargs');
const { nugLog, levels } = require('nugget-logger');
const { tokenTypes, responseTypes, responseToken } = require('nugget-comms').protocol;
const { Pca9685Driver } = require("pca9685");
const util = require('util');
const fs = require('fs');
const args = yargs
    .usage('Usage: $0 [options]')
    .version(false)
    .option('d', {
        alias: 'debug',
        desc: 'use fake sensor values instead of real onez',
        type: 'boolean'
    })
    .option('l', {
        alias: 'local',
        desc: 'run the server on localhost',
        type: 'boolean'
    })
    .option('L', {
        alias: 'log-level',
        desc: 'specify the logging level to use',
        type: 'string',
        choices: levels,
        default: 'INFO'
    })
    .alias('h', 'help')
    .argv;
if (args.local) args.debug = true;
const depthSlave = !(args.debug || args.local) ? new (require('./depth/depth'))() : {};
const electromagSlave = !(args.debug || args.local) ? new (require('./electromag/electromag'))() : {};
const i2cbus = !args.debug ? require('i2c-bus') : { openSync: () => 69 };
// global constants
const hostAddress = '0.0.0.0';
const hostPort = 8080;
const motorChannels = {
    LF: 8,
    RF: 1,
    LB: 9,
    RB: 2,
    F: 3,
    B: 11,
    manip: 0,
    picam: 10,
    leveler: 7
};
const motorChannelGroups = {
    vector: [ motorChannels.LF, motorChannels.RF, motorChannels.LB, motorChannels.RB ],
    depth: [ motorChannels.F, motorChannels.B ],
};
const LEDChannels = [5, 6];
const vectorMapMatrix = [
    // F/B, Turn, Strafe
    [ 1,  1,  1 ], // LF
    [ 1, -1, -1 ], // RF
    [ 1,  1, -1 ], // LB
    [ 1, -1,  1 ], // RB
];
const depthMapMatrix = [
    // Pitch, Depth
    [  1, -1 ], // F
    [ -1, -1 ]  // B
];
// # of turbines in the vector drive
const intervals = {};

// set up logger
const logger = new nugLog(args.logLevel, 'remote.log', () => {
    console.log(`logging at level ${args.logLevel}`)
});
if (args.debug) logger.i('startup', 'running in debug mode');
const tokenTypeEmitter = new EventEmitter();
const specialEmitter = new EventEmitter();
const pca = args.debug ? undefined : new Pca9685Driver({
    i2c: i2cbus.openSync(1),
    address: 0x40,
    frequency: 50,
    debug: false
}, error => {
    if (error) {
        logger.e('PCA Init', 'wow some serious shit happened trying to initialize the PCA. here\'s some more on that:\n');
        throw error;
    }
    logger.i('PCA Init', 'PCA Initialized successfully');
    Object.keys(motorChannels).map(async key => {
        if (key === 'vector' || key === 'depth') return;
        console.log(`setting ${key} to 1550us`);
        pca.setPulseLength(motorChannels[key], 1550);
    });
});

// global not-constants
let _client, depthLockToggle, targetPressure;

// exit on any message from parent process (if it exists)
process.on('message', process.exit);

//////////////////////////////////
// server logic & listener shit //
//////////////////////////////////

const server = net.createServer();
server.on('listening', onServerListening);
server.on('error', onServerError);
server.on('connection', onServerConnection);
server.listen({
    host: hostAddress,
    port: hostPort,
    exclusive: true
});

// listening listener (heh)
function onServerListening() {
    logger.i('listening', `server is listening at ${hostAddress}:${hostPort}`);
    try {
        process.send('listening');
    }
    catch(error) {
        logger.v('listening', 'process was not spawned as a child process')
    }
}

// error listener
function onServerError(error) {
    logger.e('server error', error);
}

// connection logic
function onServerConnection(client) {
    logger.i('connection', 'client connected');
    _client = client;

    client.on('data', onClientData);
    client.on('close', onClientDisconnect);
    client.on('error', onClientError);
}

function onClientData(data) {
    data.toString().replace(/}{/g, '}|{').split('|').forEach(datum => {
        logger.d('message', `Hey I got this: ${datum}`);
        try {
            datum = JSON.parse(datum);
        }
        catch(error) {
            logger.e('token parse', `couldn't parse this: ${datum}`);
            datum = 'tokenerror';
        }
        tokenTypeEmitter.emit(datum.type, datum);
    })
}

function onClientDisconnect() {
    logger.i('connection', 'client disconnected');
    Object.values(intervals).map(interval => clearInterval(interval));
    _client = null;
}

function onClientError(error) {
    logger.e('client error', error);
}

/*
 * EMITTER LOGIC
 *
 * Instead of having a big disgusting switch statement to handle commands
 * like two years ago, we're using an event emitter now.
 *
 * This server gets data from the surface in the form of stringified botProtocol tokens.
 * botProtocol tokens look like this:
 * {
 *   type: botProtocol.tokenType
 *   headers: {
 *     transactionID: UUIDv1
 *   }
 *   body: message body
 * }
 * When the server gets one of these tokens, [emitter] will emit an event with the token's
 * 'type' as the event name and with the whole token itself as the callback parameter.
 * The token is recieved as a string and is parsed to an object before it is emitted.
 */
tokenTypeEmitter.on(tokenTypes.ECHO, echo);
tokenTypeEmitter.on(tokenTypes.READMAG, readMag);
tokenTypeEmitter.on(tokenTypes.STARTMAGSTREAM, startMagStream);
tokenTypeEmitter.on(tokenTypes.STOPMAGSTREAM, stopMagStream);
tokenTypeEmitter.on(tokenTypes.CONTROLLERDATA, data => setMotors(data, true));
tokenTypeEmitter.on(tokenTypes.READPITEMP, readPiTemp);
tokenTypeEmitter.on(tokenTypes.STARTPITEMPSTREAM, startPiTempStream);
tokenTypeEmitter.on(tokenTypes.STOPPITEMPSTREAM, stopPiTempStream);
tokenTypeEmitter.on(tokenTypes.SETDEPTHLOCK, setDepthLock);
tokenTypeEmitter.on(tokenTypes.LEDTEST, setLEDBrightness);
tokenTypeEmitter.on(tokenTypes.PIDTUNE, tunePIDLoop);
tokenTypeEmitter.on(tokenTypes.SPECIALTOKEN, data => {
    logger.d('special', `emitting type ${data.headers.specialType}`);
    specialEmitter.emit(data.headers.specialType, data)
});

specialEmitter.on('test', data => {
    logger.d('special delivery', 'IT FUCKING WORKED');
    sendToken(new responseToken('WOOOOO', data.headers.transactionID));
});
specialEmitter.on('electromag', async data => {
    if (data.body) {
        logger.d('electromag', 'setting electromag on');
        await electromagSlave.on();
    }
    else {
        logger.d('electromag', 'setting electromag off');
        electromagSlave.off();
    }
    sendToken(new responseToken({}, data.headers.transactionID));
});

// respond with the same body as the request
function echo(data) {
    const response = new responseToken(data.body, data.headers.transactionID);
    sendToken(response);
}

function readMag(data) {
    // if we're in debug mode, send back random values from [-pi - pi) radians
    if (args.debug) {
        const response = new responseToken({
            heading: Math.random() * 2 * Math.PI - Math.PI,
            pitch: Math.random() * 2 * Math.PI - Math.PI,
            roll: Math.random() * 2 * Math.PI - Math.PI
        }, data.headers.transactionID);
        sendToken(response);
        return;
    }
    // Chris' sensor library call would go here
}

function startMagStream(data) {
    clearInterval(intervals['mag']);
    /*
     * The third setInterval parameter is used as a fake token to pass to the readMag function.
     * Since readMag only ever looks at the token's transactionID, we can
     * trick it into sending a response token with the a pre-determined
     * transactionID. The surface station will then emit an event of that
     * pre-determined type, and we can handle that event knowing that it's
     * a response from the magStream.
     */
    intervals['mag'] = setInterval(readMag, data.body.interval, {
        headers: {
            transactionID: responseTypes.MAGDATA
        }
    });
    sendToken(new responseToken({}, data.headers.transactionID));
}

function stopMagStream(data) {
    clearInterval(intervals['mag']);
    sendToken(new responseToken({}, data.headers.transactionID));
}

/**
 * Maps degrees of freedom to motor values and sends the motor values in the body of a response token.
 * @param data - token recieved from the surface
 */
// old joystick values
let oldDOFValues = {
    FB: 0,
    turn: 0,
    strafe: 0,
    pitch: 0,
    depth: 0,
    manip: 0,
    leveler: 0,
    picam: 0
};
function setMotors(data, fromController = false) {
    logger.d('motor values token', JSON.stringify(data));
    const DOFValues = data.body;
    const motorValues = {};
    if (DOFValues.hasOwnProperty('FB') || DOFValues.hasOwnProperty('turn') || DOFValues.hasOwnProperty('strafe'))
        motorValues.vector = calcMotorValues([
            DOFValues.hasOwnProperty('FB') ? DOFValues.FB : oldDOFValues.FB,
            DOFValues.hasOwnProperty('turn') ? DOFValues.turn : oldDOFValues.turn,
            DOFValues.hasOwnProperty('strafe') ? DOFValues.strafe : oldDOFValues.strafe
        ], vectorMapMatrix)
            .map((value, index) => setChannel(motorChannelGroups.vector[index], value));
    if (DOFValues.hasOwnProperty('pitch') || (DOFValues.hasOwnProperty('depth') && !depthLockToggle && fromController))
        motorValues.depth = calcMotorValues([
            DOFValues.hasOwnProperty('pitch') ? DOFValues.pitch : oldDOFValues.pitch,
            DOFValues.hasOwnProperty('depth') ? DOFValues.depth : oldDOFValues.depth
        ], depthMapMatrix)
            .map((value, index) => {
                if (depthLockToggle && index === 1) return;
                setChannel(motorChannelGroups.depth[index], value);
            });
    if (DOFValues.hasOwnProperty('manip'))
        motorValues.manip = setChannel(
            motorChannels.manip,
            calcMotorValue(DOFValues.manip)
        );
    if (DOFValues.hasOwnProperty('picam'))
        motorValues.picam = setChannel(
            motorChannels.picam,
            calcMotorValue(DOFValues.picam)
        );
    if (DOFValues.hasOwnProperty('leveler'))
        motorValues.leveler = setChannel(
            motorChannels.leveler,
            calcMotorValue(DOFValues.leveler, 500)
        );

    logger.d('motor values', JSON.stringify(motorValues));

    const response = new responseToken(motorValues, data.headers.transactionID);
    sendToken(response);
    Object.assign(oldDOFValues, DOFValues);
}

/**
 * Just set one motor but do all the things too
 * @param data
 */
function setMotor(data) {
    setMotors({
        headers: {
            transactionID: responseTypes.MOTORDATA
        },
        body: data
    });
}

/**
 * Just set the fuckin thing and make sure it doesn't fail too hard
 * @param value -
 * @param channel
 */
function setChannel(channel, value) {
    if (args.debug) return value;
    try {
        pca.setPulseLength(channel, value);
    }
    catch (error) {
        console.error(`YOU GOT AN ERROR BITCH ${value} CHANNEL ${channel} DON'T FLY`);
        console.error(error);
    }
    return value;
}

/**
 * Wrapper for calcMotorValues for when you only have to set one motor value (like the manipulator)
 * @param data - DOF data
 * @param diff - diff for calcMotorValues
 * @param mid - mid for calcMotorValues
 * @returns {Object} - actually a number
 */
function calcMotorValue(data, diff = 400, mid = 1550) {
    return calcMotorValues([[data]], [[1]])[0];
}

/**
 * Map the input DOF data to motor values
 * @param data - Array of DOF values
 * @param matrix - Matrix mapping DOFs to motor values
 * @param diff - the difference in pulse width microseconds between mid and min/max
 * @param mid - the exact middle of the pulse width range of the thing we're controlling
 * @returns {Array<Object>}
 */
function calcMotorValues(data, matrix, diff = 400, mid = 1550) {
    const rawValues = matrix.map(row =>
        /*
         * Each row in the matrix vectorMapMatrix is a motor, and each column is a degree of freedom.
         *
         * With the reduce function we're applying the values of the 5 degrees of freedom to each row
         * in the matrix, where each value represents weather the turbine represented by that row should
         * go forwards or backwards based on the value of the degr.ee of freedom in that column.
         *
         * Then we take that and divide it by the number of motors there are in the vector drive (4 in our case)
         * so that no motor's value will never go over 1 or below -1.
         *
         * THEN we take that value and add 1 & divide by 2 so the number's range becomes [0 -> 1] instead of [-1 -> 1].
         *
         * FINALLY because sometimes the buttons hiccup and joystick-mapper adds more degrees of freedom together than
         * it needs to, we set the duty cycle in a try/catch.
         *
         * TODO I'd like to fix the last one if we have time, it's really a nitpicky thing though.
         */
        row.reduce((sum, dir, index) => sum + dir * data[index], 0)
    );

    return rawValues.map(element =>
        // divide all elements by max
        // calculate max using reduce
        element / rawValues.reduce((accum, val) => Math.abs(val) > accum ? Math.abs(val) : accum, 1) * diff + mid
    )
}

/**
 * Respond with Pi's CPU temp in degrees Celcius
 * @param data
 */
async function readPiTemp(data) {
    if (args.debug)
        return sendToken(new responseToken('6 bajillion degrees', data.headers.transactionID));

    sendToken(new responseToken(
        (await util.promisify(fs.readFile)('/sys/class/thermal/thermal_zone0/temp', 'utf8'))/1000,
        data.headers.transactionID)
    );
}

/**
 * Start streaming Pi CPU temp at the interval sent as data.body
 * @param data
 */
function startPiTempStream(data) {
    intervals['piTemp'] = setInterval(readPiTemp, data.body, {
        headers: {
            transactionID: responseTypes.PITEMPDATA
        }
    });
    sendToken(new responseToken({}, data.headers.transactionID));
}

/**
 * Stop streaming Pi CPU temp data to the surface
 * @param data
 */
function stopPiTempStream(data) {
    clearInterval(intervals['piTemp']);
    sendToken(new responseToken({}, data.headers.transactionID));
}

/**
 * Toggles depth lock to be enabled or disabled
 * @param data - token sent from robot, body has boolean to lock depth or not
 */
async function setDepthLock(data) {
    // clear interval first
    clearInterval(intervals['depthLock']);
    if (!data.body) {
        // depth lock off
        logger.i('depth lock', 'depth lock disabled');
        depthLockToggle = false;
        return;
    }

    // depth lock on!!
    depthLockToggle = true;
    targetPressure = await depthSlave.getPressure();
    logger.i('depth lock', `depth lock enabled, setting to ${targetPressure}`);
    intervals['depthLock'] = setInterval(depthLoop, 100);

    // respond like a good boy
    sendToken(new responseToken({}, data.headers.transactionID));
}

// Constants are here for now, will be set by the user through the dashboard later.
let zKp = 1;
let zKi = 1;
let zKd = 0;

async function depthLoop() {
    // difference between target and current depth
    const error = targetPressure - await depthSlave.getPressure();
    const dofValue = Math.min(Math.max(-1, error / 30), 1);
    setMotor({
        depth: -dofValue
    });

    logger.v('depth lock', dofValue);
}

function setLEDBrightness(data) {
    if (args.debug)
        return;

    const dutyCycle = (data.body + 1) / 2;
    LEDChannels.map(channel => {
        logger.d('LEDTest', `${channel} ${dutyCycle}`);
        pca.setDutyCycle(channel, dutyCycle);
    });

    sendToken(new responseToken({}, data.headers.transactionID));
}

function tunePIDLoop(data) {
    if (data.hasOwnProperty('zKp')) zKp = data.body.zKp;
    if (data.hasOwnProperty('zKi')) zKi = data.body.zKi;
    if (data.hasOwnProperty('zKd')) zKd = data.body.zKd;

    sendToken(new responseToken({
        zKp: zKp,
        zKi: zKi,
        zKd: zKd
    }, data.headers.transactionID));
}

function sendToken(token) {
    // be careful with verbose logging in local mode
    // this can crash the script if too much is being logged
    logger.d('message', 'sending it: ' + token.stringify());
    _client.write(token.stringify());
}
