package frc.robot.Controller;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class ReyannController extends GenericHID implements Sendable {
  /** Represents a digital button on a Reyann Controller. */
  public enum Button {
    /** L1 coral button. */
    kL1(12),
    /** L2 coral button. */
    kL2(11),
    /** L3 coral button. */
    kL3(10),
    /** L4 coral button. */
    kL4(9);

    /** Button value. */
    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into (0-5).
   */
  public ReyannController(final int port) {
    super(port);
    HAL.report(tResourceType.kResourceType_Button, port + 1);
  }

  /**
   * Read the value of the L1 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getL1Button() {
    return getRawButton(Button.kL1.value);
  }

  /**
   * Whether the L1 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getL1ButtonPressed() {
    return getRawButtonPressed(Button.kL1.value);
  }

  /**
   * Whether the L1 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getL1ButtonReleased() {
    return getRawButtonReleased(Button.kL1.value);
  }

  /**
   * Constructs an event instance around the L1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the L1 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent L1(EventLoop loop) {
    return button(Button.kL1.value, loop);
  }

  /**
   * Read the value of the L2 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getL2Button() {
    return getRawButton(Button.kL2.value);
  }

  /**
   * Whether the L2 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getL2ButtonPressed() {
    return getRawButtonPressed(Button.kL2.value);
  }

  /**
   * Whether the L2 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getL2ButtonReleased() {
    return getRawButtonReleased(Button.kL2.value);
  }

  /**
   * Constructs an event instance around the L2 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the L2 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent L2(EventLoop loop) {
    return button(Button.kL2.value, loop);
  }

  /**
   * Read the value of the L3 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getL3Button() {
    return getRawButton(Button.kL3.value);
  }

  /**
   * Whether the L3 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getL3ButtonPressed() {
    return getRawButtonPressed(Button.kL3.value);
  }

  /**
   * Whether the L3 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getL3ButtonReleased() {
    return getRawButtonReleased(Button.kL3.value);
  }

  /**
   * Constructs an event instance around the L3 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the L3 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent L3(EventLoop loop) {
    return button(Button.kL3.value, loop);
  }

  /**
   * Read the value of the L4 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getL4Button() {
    return getRawButton(Button.kL4.value);
  }

  /**
   * Whether the L4 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getL4ButtonPressed() {
    return getRawButtonPressed(Button.kL4.value);
  }

  /**
   * Whether the L4 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getL4ButtonReleased() {
    return getRawButtonReleased(Button.kL4.value);
  }

  /**
   * Constructs an event instance around the L4 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the L4 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent L4(EventLoop loop) {
    return button(Button.kL4.value, loop);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "Reyann");
    builder.addBooleanProperty("L1", this::getL1Button, null);
    builder.addBooleanProperty("L2", this::getL2Button, null);
    builder.addBooleanProperty("L3", this::getL3Button, null);
    builder.addBooleanProperty("L4", this::getL4Button, null);
  }
}
