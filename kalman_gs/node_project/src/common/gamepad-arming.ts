import { GamepadInput } from './gamepad-compat';
import { readGamepads } from './gamepads';
import { ros } from './ros';
import { ArmFkCommand } from './ros-interfaces';
import { Topic } from 'roslib';

const RATE = 10.0;

enum AxisType {
  AXIS,
  TWO_BUTTONS,
  TWO_AXES
}

interface AxisInput {
  axisId: GamepadInput;
}

interface TwoAxesInput {
  positiveAxisId: GamepadInput;
  negativeAxisId: GamepadInput;
}

interface TwoButtonsInput {
  positiveButtonId: GamepadInput;
  negativeButtonId: GamepadInput;
}

type MappingFunction = (input: number) => number;

type JointBind = {
  type: AxisType;
  input: AxisInput | TwoAxesInput | TwoButtonsInput;
  mapping: MappingFunction;
};

type JointMapping = {
  joint_1: JointBind;
  joint_2: JointBind;
  joint_3: JointBind;
  joint_4: JointBind;
  joint_5: JointBind;
  joint_6: JointBind;
  gripper: JointBind;
};

const noMapping: MappingFunction = (input) => input;
const invertedMapping: MappingFunction = (input) => -input;

const jointInputMapping: JointMapping = {
  joint_1: {
    type: AxisType.AXIS,
    input: { axisId: 'left-x' },
    mapping: invertedMapping
  },
  joint_2: {
    type: AxisType.AXIS,
    input: { axisId: 'left-y' },
    mapping: noMapping
  },
  joint_3: {
    type: AxisType.AXIS,
    input: { axisId: 'right-y' },
    mapping: noMapping
  },
  joint_4: {
    type: AxisType.AXIS,
    input: { axisId: 'right-x' },
    mapping: noMapping
  },
  joint_5: {
    type: AxisType.TWO_BUTTONS,
    input: { positiveButtonId: 'dpad-up', negativeButtonId: 'dpad-down' },
    mapping: noMapping
  },
  joint_6: {
    type: AxisType.TWO_AXES,
    input: { positiveAxisId: 'right-trigger', negativeAxisId: 'left-trigger' },
    mapping: noMapping
  },
  gripper: {
    type: AxisType.TWO_BUTTONS,
    input: { positiveButtonId: 'y-button', negativeButtonId: 'a-button' },
    mapping: noMapping
  }
};

function translateGamepadToCommand({ type, input, mapping }: JointBind) {
  switch (type) {
    case AxisType.AXIS:
      if (!('axisId' in input)) {
        return 0;
      }
      return mapping(readGamepads(input.axisId, 'arm'));

    case AxisType.TWO_AXES:
      if (!('positiveAxisId' in input && 'negativeAxisId' in input)) {
        return 0;
      }
      return mapping(
        readGamepads(input.positiveAxisId, 'arm') -
          readGamepads(input.negativeAxisId, 'arm')
      );
    case AxisType.TWO_BUTTONS:
      if (!('positiveButtonId' in input && 'negativeButtonId' in input)) {
        return 0;
      }
      const pos = input.positiveButtonId;
      return mapping(
        readGamepads(input.positiveButtonId, 'arm') -
          readGamepads(input.negativeButtonId, 'arm')
      );
  }
}

window.addEventListener('ros-connect', () => {
  const fkTopic = new Topic<ArmFkCommand>({
    ros: ros,
    name: '/station/arm/fk/command',
    messageType: 'kalman_interfaces/ArmFkCommand'
  });

  setInterval(() => {
    let msg: ArmFkCommand = {
      joint_1: translateGamepadToCommand(jointInputMapping.joint_1),
      joint_2: translateGamepadToCommand(jointInputMapping.joint_2),
      joint_3: translateGamepadToCommand(jointInputMapping.joint_3),
      joint_4: translateGamepadToCommand(jointInputMapping.joint_4),
      joint_5: translateGamepadToCommand(jointInputMapping.joint_5),
      joint_6: translateGamepadToCommand(jointInputMapping.joint_6),
      gripper: translateGamepadToCommand(jointInputMapping.gripper)
    };

    fkTopic.publish(msg);
  }, 1000 / RATE);
});