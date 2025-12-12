
"use strict";

let Sound = require('./Sound.js');
let ControllerInfo = require('./ControllerInfo.js');
let PowerSystemEvent = require('./PowerSystemEvent.js');
let DigitalInputEvent = require('./DigitalInputEvent.js');
let ButtonEvent = require('./ButtonEvent.js');
let KeyboardInput = require('./KeyboardInput.js');
let BumperEvent = require('./BumperEvent.js');
let ExternalPower = require('./ExternalPower.js');
let Led = require('./Led.js');
let DigitalOutput = require('./DigitalOutput.js');
let RobotStateEvent = require('./RobotStateEvent.js');
let DockInfraRed = require('./DockInfraRed.js');
let VersionInfo = require('./VersionInfo.js');
let ScanAngle = require('./ScanAngle.js');
let SensorState = require('./SensorState.js');
let CliffEvent = require('./CliffEvent.js');
let MotorPower = require('./MotorPower.js');
let WheelDropEvent = require('./WheelDropEvent.js');
let AutoDockingResult = require('./AutoDockingResult.js');
let AutoDockingActionGoal = require('./AutoDockingActionGoal.js');
let AutoDockingAction = require('./AutoDockingAction.js');
let AutoDockingActionFeedback = require('./AutoDockingActionFeedback.js');
let AutoDockingGoal = require('./AutoDockingGoal.js');
let AutoDockingFeedback = require('./AutoDockingFeedback.js');
let AutoDockingActionResult = require('./AutoDockingActionResult.js');

module.exports = {
  Sound: Sound,
  ControllerInfo: ControllerInfo,
  PowerSystemEvent: PowerSystemEvent,
  DigitalInputEvent: DigitalInputEvent,
  ButtonEvent: ButtonEvent,
  KeyboardInput: KeyboardInput,
  BumperEvent: BumperEvent,
  ExternalPower: ExternalPower,
  Led: Led,
  DigitalOutput: DigitalOutput,
  RobotStateEvent: RobotStateEvent,
  DockInfraRed: DockInfraRed,
  VersionInfo: VersionInfo,
  ScanAngle: ScanAngle,
  SensorState: SensorState,
  CliffEvent: CliffEvent,
  MotorPower: MotorPower,
  WheelDropEvent: WheelDropEvent,
  AutoDockingResult: AutoDockingResult,
  AutoDockingActionGoal: AutoDockingActionGoal,
  AutoDockingAction: AutoDockingAction,
  AutoDockingActionFeedback: AutoDockingActionFeedback,
  AutoDockingGoal: AutoDockingGoal,
  AutoDockingFeedback: AutoDockingFeedback,
  AutoDockingActionResult: AutoDockingActionResult,
};
