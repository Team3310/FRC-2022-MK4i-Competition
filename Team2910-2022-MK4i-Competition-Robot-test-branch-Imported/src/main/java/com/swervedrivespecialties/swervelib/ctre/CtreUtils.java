package com.swervedrivespecialties.swervelib.ctre;

//import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenixpro.CtrCode;

public final class CtreUtils {
    private CtreUtils() {
    }

    // public static void checkCtreError(ErrorCode errorCode, String message) {
    //     if (errorCode != ErrorCode.OK) {
    //         System.out.println(String.format("%s: %s", message, errorCode.toString()));
    //     }
    // }

    public static void checkCtreError(CtrCode errorCode, String message) {
        if (errorCode != CtrCode.OK) {
            System.out.println(String.format("%s: %s", message, errorCode.toString()));
        }
    }
}
