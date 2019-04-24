
"use strict";

let pickup = require('./pickup.js');
let drop_command = require('./drop_command.js');
let grabber_motors = require('./grabber_motors.js');
let drop_motors = require('./drop_motors.js');

module.exports = {
  pickup: pickup,
  drop_command: drop_command,
  grabber_motors: grabber_motors,
  drop_motors: drop_motors,
};
