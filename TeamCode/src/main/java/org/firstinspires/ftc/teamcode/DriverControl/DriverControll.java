package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.RobotClass;
import java.util.List;

@TeleOp(name = "ONTOS Combined", group = "driving")
public class DriverControll extends OpMode {

    RobotClass robot;
    List<LynxModule> allHubs;

    // Use arrays to manage the state for each gun individually
    boolean[] valveOpen = new boolean[6];
    boolean[] moving = new boolean[6];
    int[] targetPosition = new int[6];
    double[] movePower = new double[6];
    boolean[] servoAt90 = new boolean[6];

    // Turret hold state
    boolean turretHolding = false;

    // Individual target encoder values for each gun
    static final int[] GUN_TARGETS = {
            6 * 28, // Gun 1
            6 * 28, // Gun 2
            6 * 28, // Gun 3
            6 * 28, // Gun 4
            6 * 28, // Gun 5
            6 * 28  // Gun 6
    };

    @Override
    public void init() {
        telemetry.addData("Init Status", "Phase 1: Starting...");
        telemetry.update();

        robot = new RobotClass(hardwareMap);

        telemetry.addData("Init Status", "Phase 2: Hardware Mapped");
        telemetry.update();

        // Reverse one side of the drivetrain for tank drive
        robot.frontRight.setDirection(CRServo.Direction.REVERSE);
        robot.backRight.setDirection(CRServo.Direction.REVERSE);

        // Reverse one turret motor if they are facing opposite directions
        robot.aimTurret[1].setDirection(DcMotor.Direction.REVERSE);

        // Set turret motors to brake when power is zero to resist gravity
        robot.aimTurret[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.aimTurret[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Init Status", "Phase 3: Directions Set");
        telemetry.update();

        // Enable Bulk Caching to reduce RS-485 bus traffic
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addData("Init Status", "Phase 4: Caching Enabled");
        telemetry.update();

        telemetry.addData("Init Status", "Complete - Awaiting Start");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Start Status", "Staggered Power Up...");
        telemetry.update();

        // Re-enable PWM for the drivetrain one by one to prevent current overdraw
        CRServo[] drivetrain = {robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight};
        String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};

        for (int i = 0; i < drivetrain.length; i++) {
            if (drivetrain[i] instanceof PwmControl) {
                ((PwmControl) drivetrain[i]).setPwmEnable();
                telemetry.addData("Powering Up", names[i]);
                telemetry.update();

                try {
                    Thread.sleep(500); // 500ms delay to stabilize current
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }

        telemetry.addData("Start Status", "Phase 2: Resetting Gun Encoders...");
        telemetry.update();

        // Reset and set the run mode for all gun motors
        for (int i = 0; i < 6; i++) {
            robot.gunFireMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.gunFireMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Start Status", "Phase 3: Configuring Turret...");
        telemetry.update();

        // Turret Encoder Setup
        robot.aimTurret[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.aimTurret[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        robot.aimTurret[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.aimTurret[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Start Status", "Complete - Running");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Safety Watchdog ---
        // Check if Servo Hub (Address 3) is still responding
        boolean servoHubHealthy = true;
        for (LynxModule module : allHubs) {
            if (module.getModuleAddress() == 3 && module.isNotResponding()) {
                servoHubHealthy = false;
                break;
            }
        }

        // --- Drivetrain Logic ---
        // Use gamepad sticks for tank drive. 
        // Note: gamepad stick Y is typically -1 at the top, so we negate it for forward motion.
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        if (!servoHubHealthy) {
            // EMERGENCY STOP: Servo Hub disconnected, stop drivetrain to prevent runaway
            robot.frontLeft.setPower(-leftPower);
            robot.backLeft.setPower(-leftPower);
            robot.frontRight.setPower(-rightPower);
            robot.backRight.setPower(-rightPower);
            telemetry.addData("!!! ERROR !!!", "SERVO HUB DISCONNECTED - DRIVETRAIN STOPPED");
        } else {
            robot.frontLeft.setPower(-leftPower);
            robot.backLeft.setPower(-leftPower);
            robot.frontRight.setPower(-rightPower);
            robot.backRight.setPower(-rightPower);
        }

        // --- Gun Logic ---
        boolean openSafety = gamepad1.left_trigger > 0.5;
        boolean closeSafety = gamepad1.right_trigger > 0.5;

        // Individual Gun states based on new mapping
        // Gun 1: A, 2: B, 3: Y, 4: X, 5: RB, 6: LB
        boolean[] gunButtons = {
                gamepad1.a,              // Gun 1
                gamepad1.b,              // Gun 2
                gamepad1.y,              // Gun 3
                gamepad1.x,              // Gun 4
                gamepad1.right_bumper,   // Gun 5
                gamepad1.left_bumper     // Gun 6
        };

        boolean[] currentGunState = new boolean[6];
        for (int i = 0; i < 6; i++) {
            if (openSafety && gunButtons[i]) {
                currentGunState[i] = true;
            } else if (closeSafety && gunButtons[i]) {
                currentGunState[i] = false;
            } else {
                // Keep the current valve state if no action is taken
                currentGunState[i] = valveOpen[i];
            }
        }

        // Group Logic: D-pad Left fires/closes all
        if (gamepad1.dpad_left) {
            if (openSafety) {
                for (int i = 0; i < 6; i++) currentGunState[i] = true;
            } else if (closeSafety) {
                for (int i = 0; i < 6; i++) currentGunState[i] = false;
            }
        }

        for (int i = 0; i < 6; i++) {
            if (currentGunState[i] != valveOpen[i]) {
                fireGunGroup(new int[]{i}, currentGunState[i]);
                // fireGunGroup already sets valveOpen[i] = open
            }
        }

        // --- Turret Aiming Logic ---
        double turretPower = 0;
        if (gamepad1.dpad_up) {
            turretPower = 0.6;
        } else if (gamepad1.dpad_down) {
            turretPower = -0.6;
        }
        
        int currentPos = robot.aimTurret[0].getCurrentPosition();

        // Block downward movement if limit is reached

        if (Math.abs(turretPower) > 0.05) {
            // Manual control
            robot.aimTurret[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.aimTurret[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.aimTurret[0].setPower(turretPower);
            robot.aimTurret[1].setPower(turretPower);
            turretHolding = false;
        } else {
            // Hold position logic
            if (!turretHolding) {
                // Just released the trigger, lock both at current position
                robot.aimTurret[0].setTargetPosition(robot.aimTurret[0].getCurrentPosition());
                robot.aimTurret[1].setTargetPosition(robot.aimTurret[1].getCurrentPosition());
                
                robot.aimTurret[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.aimTurret[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                robot.aimTurret[0].setPower(0.8); // Power limit for holding
                robot.aimTurret[1].setPower(0.8);
                turretHolding = true;
            }
        }

        // Movement and telemetry logic for each gun
        for (int i = 0; i < 6; i++) {
            int position = robot.gunFireMotors[i].getCurrentPosition();

            if (moving[i]) {
                if (valveOpen[i]) {
                    if (position < targetPosition[i]) {
                        robot.gunFireMotors[i].setPower(movePower[i]);
                    } else {
                        robot.gunFireMotors[i].setPower(0);
                        moving[i] = false;
                    }
                } else {
                    if (position > targetPosition[i]) {
                        robot.gunFireMotors[i].setPower(movePower[i]);
                    } else {
                        robot.gunFireMotors[i].setPower(0);
                        moving[i] = false;
                        robot.gunFireMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.gunFireMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }
            } else {
                robot.gunFireMotors[i].setPower(0);
            }

            telemetry.addData("Gun " + (i + 1) + " Encoder Position", position);
            telemetry.addData("Gun " + (i + 1) + " Status", valveOpen[i] ? "Open" : "Closed");
        }

        // Add drivetrain telemetry
        telemetry.addData("Open Safety (LT)", openSafety);
        telemetry.addData("Close Safety (RT)", closeSafety);
        telemetry.addData("Left Drivetrain Power", leftPower);
        telemetry.addData("Right Drivetrain Power", rightPower);
        telemetry.addData("Turret Power", turretPower);
        telemetry.addData("Turret Position", currentPos);

        telemetry.update();
    }

    private void fireGunGroup(int[] gunIndices, boolean open) {
        for (int i : gunIndices) {
            valveOpen[i] = open;
            targetPosition[i] = open ? GUN_TARGETS[i] : 0;
            movePower[i] = open ? 1.0 : -1.0;
            moving[i] = true;
        }
    }
}