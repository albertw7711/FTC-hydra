/*
 * Copyright 2016 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor.Mode;

/**
 * A class that provides JavaScript access to a {@link IrSeekerSensor}.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
class IrSeekerSensorAccess extends HardwareAccess<IrSeekerSensor> {
  private final IrSeekerSensor irSeekerSensor;

  IrSeekerSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
    super(blocksOpMode, hardwareItem, hardwareMap, IrSeekerSensor.class);
    this.irSeekerSensor = hardwareDevice;
  }

  // Properties

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "setSignalDetectedThreshold")
  public void setSignalDetectedThreshold(double threshold) {
    try {
      startBlockExecution(BlockType.SETTER, ".SignalDetectedThreshold");
      irSeekerSensor.setSignalDetectedThreshold(threshold);
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "getSignalDetectedThreshold")
  public double getSignalDetectedThreshold() {
    try {
      startBlockExecution(BlockType.GETTER, ".SignalDetectedThreshold");
      return irSeekerSensor.getSignalDetectedThreshold();
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "setMode")
  public void setMode(String modeString) {
    try {
      startBlockExecution(BlockType.SETTER, ".Mode");
      Mode mode = checkArg(modeString, Mode.class, "");
      if (mode != null) {
        irSeekerSensor.setMode(mode);
      }
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "getMode")
  public String getMode() {
    try {
      startBlockExecution(BlockType.GETTER, ".Mode");
      Mode mode = irSeekerSensor.getMode();
      if (mode != null) {
        return mode.toString();
      }
      return "";
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "signalDetected")
  public boolean getIsSignalDetected() {
    try {
      startBlockExecution(BlockType.GETTER, ".IsSignalDetected");
      return irSeekerSensor.signalDetected();
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "getAngle")
  public double getAngle() {
    try {
      startBlockExecution(BlockType.GETTER, ".Angle");
      return irSeekerSensor.getAngle();
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "getStrength")
  public double getStrength() {
    try {
      startBlockExecution(BlockType.GETTER, ".Strength");
      return irSeekerSensor.getStrength();
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "setI2cAddress")
  public void setI2cAddress7Bit(int i2cAddr7Bit) {
    try {
      startBlockExecution(BlockType.SETTER, ".I2cAddress7Bit");
      irSeekerSensor.setI2cAddress(I2cAddr.create7bit(i2cAddr7Bit));
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "getI2cAddress")
  public int getI2cAddress7Bit() {
    try {
      startBlockExecution(BlockType.GETTER, ".I2cAddress7Bit");
      I2cAddr i2cAddr = irSeekerSensor.getI2cAddress();
      if (i2cAddr != null) {
        return i2cAddr.get7Bit();
      }
      return 0;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "setI2cAddress")
  public void setI2cAddress8Bit(int i2cAddr8Bit) {
    try {
      startBlockExecution(BlockType.SETTER, ".I2cAddress8Bit");
      irSeekerSensor.setI2cAddress(I2cAddr.create8bit(i2cAddr8Bit));
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = "getI2cAddress")
  public int getI2cAddress8Bit() {
    try {
      startBlockExecution(BlockType.GETTER, ".I2cAddress8Bit");
      I2cAddr i2cAddr = irSeekerSensor.getI2cAddress();
      if (i2cAddr != null) {
        return i2cAddr.get8Bit();
      }
      return 0;
    } catch (Throwable e) {
      blocksOpMode.handleFatalException(e);
      throw new AssertionError("impossible", e);
    } finally {
      endBlockExecution();
    }
  }
}
