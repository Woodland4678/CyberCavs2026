// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LEDStrip.LEDModes;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private LEDStrip ledStrip;
    private final RobotContainer m_robotContainer;
    private boolean hasGameData;
    private boolean wonAuto = false;
    private boolean isHubActive = true;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        double matchTime = DriverStation.getMatchTime();
        double shiftTimer = 0;
        String shiftName = "Transition";
        if (matchTime > 130) {
            shiftName = "Transition";
            shiftTimer = matchTime - 130;
            isHubActive = true;
        }
        else if (matchTime > 105) {
            shiftName = "Shift 1";
            shiftTimer = matchTime - 105;
            if (wonAuto == true){
                isHubActive = false;
            }
            else{
                isHubActive = true;
            }
        }
        else if (matchTime > 80) {
            shiftName = "Shift 2";
            shiftTimer = matchTime - 80;
            if (wonAuto == false){
                isHubActive = false;
            }
            else{
                isHubActive = true;
            }
        }
        else if (matchTime > 55) {
            shiftName = "Shift 3";
            shiftTimer = matchTime - 55;
            if (wonAuto == true){
                isHubActive = false;
            }
            else{
                isHubActive = true;
            }
        }
        else if (matchTime > 30) {
            shiftName = "Shift 4";
            shiftTimer = matchTime - 30;
            if (wonAuto == false){
                isHubActive = false;
            }
            else{
                isHubActive = true;
            }
        }
        else  {
            shiftName = "End Game";
            shiftTimer = matchTime;
            isHubActive = true;
        }
        ledStrip.periodic();  // SDW uncomment when ready to use
        SmartDashboard.putNumber("Match Timer", matchTime);
        SmartDashboard.putString("Shift #", shiftName);
        SmartDashboard.putNumber("Shift Timer", shiftTimer);
        SmartDashboard.putBoolean("Hub Active", isHubActive);
        

        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {
        ledStrip = LEDStrip.getInstance();
        ledStrip.setLEDMode(LEDModes.SOLIDBLUE);  
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.updateAutoPreview();
        var diagState = 0; //diagnostic state 
   
        ledStrip = LEDStrip.getInstance();
        diagState += LEDStrip.unused; //add unused segment as is always true
        if (m_robotContainer.isShooterMotorsReady()){
        diagState += LEDStrip.shooterMotorDiag; 
        }
        if (m_robotContainer.isShooterHoodReady()){
        diagState += LEDStrip.shooterHoodDiag; 
        }
        if (m_robotContainer.isClimberReady()){
        diagState += LEDStrip.climberDiag; 
        }
        if (m_robotContainer.isFrontLeftSwerveReady()){
        diagState += LEDStrip.swerve1Diag; 
        }
        if (m_robotContainer.isFrontRightSwerveReady()){
        diagState += LEDStrip.swerve2Diag; 
        }
        if (m_robotContainer.isBackLeftSwerveReady()){
        diagState += LEDStrip.swerve3Diag; 
        }
        if (m_robotContainer.isBackRightSwerveReady()){
        diagState += LEDStrip.swerve4Diag; 
        }
        if (m_robotContainer.isGyroReady()){
        diagState += LEDStrip.gyroDiag;
        }  
        if (m_robotContainer.isAprilTagCameraReady()){
        diagState += LEDStrip.apriltagDiag;
        }  
        if (m_robotContainer.isFloorReady()){
        diagState += LEDStrip.hopperFloorDiag;
        }  
        if (m_robotContainer.isIntakeDeployReady()){
        diagState += LEDStrip.intakeDeployDiag;
        }
        if (m_robotContainer.isIntakeWheelsReady()){
        diagState += LEDStrip.intakeWheelsDiag;
        }  

        // force the spare led segment to be green
        diagState += LEDStrip.spareDiag;

        SmartDashboard.putNumber("diagState", diagState);
        ledStrip.setDiagnosticPattern(diagState);
        ledStrip.diagnosticLEDmode(); // SDW uncomment when ready to use

       // ledStrip.setLEDMode(LEDModes.SOLIDBLUE); // SDW

        ledStrip.periodic();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        hasGameData = false;
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        
    }

    @Override
    public void teleopPeriodic() {
        String gameData;
       
        
        Optional<Alliance> ally = DriverStation.getAlliance();
        
       

        // which alliance is inactive first?
        // boolean isRedInactiveFirst = false;
        // gameData = DriverStation.getGameSpecificMessage();
        
        // if(!hasGameData && gameData.length() > 0) {
        //     switch (gameData.charAt(0)) {
        //         case 'B':
        //             isRedInactiveFirst = false;
        //             break;
        //         case 'R':
        //             isRedInactiveFirst = true;
        //             break;
        //         default:
        //             break;
        //     }
        // }

        // ******** add code to check if our hub is active currently *********
        // This should likely be a function accessible from where we need to use it

        
        gameData = DriverStation.getGameSpecificMessage();
        
        if(!hasGameData && gameData.length() > 0) {
           // Optional<Alliance> ally = DriverStation.getAlliance();
            switch (gameData.charAt(0)) {
                case 'B':
                    if(ally.isPresent()) {
                        if (ally.get() == Alliance.Blue) {
                            ledStrip = LEDStrip.getInstance();
                            ledStrip.setLEDMode(LEDModes.SOLIDGREEN);
                            wonAuto = true;  
                            SmartDashboard.putString("Won or Lost Auto", "Won");
                        }

                    }
                    if(ally.isPresent()) {
                        if (ally.get() == Alliance.Red) {
                            ledStrip = LEDStrip.getInstance();
                            ledStrip.setLEDMode(LEDModes.SOLIDPURPLE);
                            wonAuto = false;
                            SmartDashboard.putString("Won or Lost Auto", "Lost");
                        }

                    }
                    hasGameData = true;

                    break;

                case 'R':
                    if(ally.isPresent()) {
                        if (ally.get() == Alliance.Red) {
                            ledStrip = LEDStrip.getInstance();
                            ledStrip.setLEDMode(LEDModes.SOLIDGREEN);
                            wonAuto = true;
                            SmartDashboard.putString("Won or Lost Auto", "Won");
                        }

                    }
                    if(ally.isPresent()) {
                        if (ally.get() == Alliance.Blue) {
                            ledStrip = LEDStrip.getInstance();
                            ledStrip.setLEDMode(LEDModes.SOLIDPURPLE);
                            wonAuto = false;
                            SmartDashboard.putString("Won or Lost Auto", "Lost");
                        }

                    }
                    hasGameData = true;

                    break;
                default:
                    break;
            }
        }
        
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
