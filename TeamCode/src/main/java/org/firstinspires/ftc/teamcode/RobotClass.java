package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotClass {
    // Drivetrain motors as CRServos
    public CRServo frontLeft, frontRight, backLeft, backRight;

    // Public arrays to hold all gun motor and servo objects
    public DcMotorEx[] gunFireMotors = new DcMotorEx[6];
    public DcMotorEx[] aimTurret = new DcMotorEx[2];

    public RobotClass(HardwareMap hardwareMap) {
        // Initialize Drivetrain
        frontLeft = hardwareMap.get(CRServo.class, "frontLeftSparkMax");
        frontRight = hardwareMap.get(CRServo.class, "frontRightSparkMax");
        backLeft = hardwareMap.get(CRServo.class, "backLeftSparkMax");
        backRight = hardwareMap.get(CRServo.class, "backRightSparkMax");

        // Use a loop to initialize all gun hardware components by name
        for (int i = 0; i < 6; i++) {
            String motorName = "gun" + (i + 1) + "Fire";
            gunFireMotors[i] = hardwareMap.get(DcMotorEx.class, motorName);
        }

        // Initialize turret aiming motors
        aimTurret[0] = hardwareMap.get(DcMotorEx.class, "aimTurret1");
        aimTurret[1] = hardwareMap.get(DcMotorEx.class, "aimTurret2");
    }
}
