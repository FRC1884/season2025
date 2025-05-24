package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class BoardOperatorMap extends CommandGenericHID implements OperatorMap {
  public BoardOperatorMap(int port) {
    super(port);
  }
}
