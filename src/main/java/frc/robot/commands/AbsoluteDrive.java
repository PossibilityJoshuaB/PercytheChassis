package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drive.Swerve;

public class AbsoluteDrive extends Command {
    
    private Swerve swervy;
    private DoubleSupplier X, Y;
    private DoubleSupplier  headingAdjust;
    private BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
    private boolean resetHeading = false;
    
    public void AbsoluteDriveAdv(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                          BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                          BooleanSupplier lookRight) {
    this.swervy = swerve;
    this.X = vX;
    this.Y = vY;
    this.headingAdjust = headingAdjust;
    this.lookAway = lookAway;
    this.lookTowards = lookTowards;
    this.lookLeft = lookLeft;
    this.lookRight = lookRight;
    addRequirements(swervy);
    }
 // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        resetHeading = true;
    }

 // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double headingX = 0;
        double headingY = 0;
        
        //Snap to commands
        if (lookAway.getAsBoolean()){
         headingY = -1;
        }
        if (lookTowards.getAsBoolean()){
         headingY = 1;
        }
        if (lookLeft.getAsBoolean()){
         headingX = -1;
        }
        if (lookRight.getAsBoolean()){
         headingX = 1;
        }

        
    }

 // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

 // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return false;
    }
}
