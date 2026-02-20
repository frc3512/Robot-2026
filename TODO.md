# TODO for Correcting ConveyorIO_SIM.java

1. [x] Update class fields: Replace single DCMotorSim with hopperSim and feederSim, add hopperSpeed and feederSpeed double fields.
2. [x] Update constructor: Initialize two DCMotorSim instances for hopper and feeder.
3. [x] Implement setHopper(double speed) method.
4. [x] Implement setFeeder(double speed) method.
5. [x] Update updateInputs method: Simulate both motors, set input voltages based on speeds, update sim, and populate inputs with feederVelocity, hopperVelocity, feederVolts, hopperVolts.
