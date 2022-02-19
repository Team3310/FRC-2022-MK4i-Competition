package com.swervedrivespecialties.swervelib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    void setOpenLoopVoltageRamp(double timeTo12VSec);
}
