package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void resetAbsoluteSteerAngle();

    void setEncoderAutoResetIterations(int iterations);
}
