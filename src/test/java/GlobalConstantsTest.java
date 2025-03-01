import frc.robot.GlobalConstants;

public class GlobalConstantsTest {
  // @Test
  public void main() {
    assert (GlobalConstants.FieldMap.Coordinates.REEF_1.getPose().getRotation().getDegrees()
        == 180);
  }
}
