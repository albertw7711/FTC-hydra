/*
 * Copyright (c) 2014, 2015 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.qualcomm.hardware;

import android.content.Context;
import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.AnalogSensorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.BuiltInConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.ConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.DigitalIoDeviceConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.LynxI2cDeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LynxModuleConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LynxUsbDeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ReadXMLFileHandler;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.I2cDeviceConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.hardware.lynx.EmbeddedControlHubModule;
import org.xmlpull.v1.XmlPullParser;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Populates the hardware map.
 */
@SuppressWarnings("unused")
public class HardwareFactory {

  //------------------------------------------------------------------------------------------------
  // State
  //------------------------------------------------------------------------------------------------

  public static final String TAG = "HardwareFactory";

  private Context       context;
  private XmlPullParser xmlPullParser = null;

  //------------------------------------------------------------------------------------------------
  // Construction
  //------------------------------------------------------------------------------------------------

  public HardwareFactory(Context context) {
    this.context = context;
  }

  //------------------------------------------------------------------------------------------------
  // Hardware management
  //------------------------------------------------------------------------------------------------

  /**
   * Create a hardware map
   *
   * @return HardwareMap
   */
  public @NonNull HardwareMap createHardwareMap(SyncdDevice.Manager manager) throws RobotCoreException, InterruptedException  {

    // We synchronize with scanning so that there's only one thread trying to open *new* FTDI devices at a time
    synchronized (HardwareDeviceManager.scanDevicesLock) {

      RobotLog.vv(TAG, "createHardwareMap()");

      // Clear notion of embedded lynx module that we currently have. We'll discover a new one,
      // if he's there, when we go through the below.
      EmbeddedControlHubModule.clear();

      HardwareMap map = new HardwareMap(context);

      if (xmlPullParser != null) {
        DeviceManager deviceMgr = new HardwareDeviceManager(context, manager);

        ReadXMLFileHandler readXmlFileHandler = new ReadXMLFileHandler(deviceMgr);

        List<ControllerConfiguration> ctrlConfList = readXmlFileHandler.parse(xmlPullParser);

        for (ControllerConfiguration ctrlConf : ctrlConfList) {
          mapControllerConfiguration(map, deviceMgr, ctrlConf);
        }

      } else {
        // no XML to parse, just return empty map
        RobotLog.vv(TAG, "no xml to parse: using empty map");
      }
      return map;
    }
  }

  public void instantiateConfiguration(HardwareMap hardwareMap, ControllerConfiguration controllerConfiguration, SyncdDevice.Manager manager) throws RobotCoreException, InterruptedException {
    synchronized (HardwareDeviceManager.scanDevicesLock) {
      DeviceManager deviceMgr = new HardwareDeviceManager(context, manager);
      mapControllerConfiguration(hardwareMap, deviceMgr, controllerConfiguration);
    }
  }

  protected void mapControllerConfiguration(HardwareMap map, DeviceManager deviceMgr, ControllerConfiguration ctrlConf) throws RobotCoreException, InterruptedException {
    ConfigurationType type = ctrlConf.getConfigurationType();
    if (type==BuiltInConfigurationType.LYNX_USB_DEVICE) {
      mapLynxUsbDevice(map, deviceMgr, (LynxUsbDeviceConfiguration)ctrlConf);
    }
    else if (type==BuiltInConfigurationType.WEBCAM) {
      mapWebcam(map, deviceMgr, (WebcamConfiguration)ctrlConf);
    }
    else {
      RobotLog.ee(TAG, "unexpected controller configuration type: %s", type);
    }
  }

  public void setXmlPullParser(XmlPullParser xmlPullParser) {
    this.xmlPullParser = xmlPullParser;
  }

  public XmlPullParser getXmlPullParser() {
    return xmlPullParser;
  }

  private void mapMotor(HardwareMap map, DeviceManager deviceMgr, DeviceConfiguration motorConf, DcMotorController dcMotorController) {
    if (!motorConf.isEnabled()) return;
    DcMotor dcMotor = deviceMgr.createDcMotor(dcMotorController, motorConf.getPort(), (MotorConfigurationType) motorConf.getConfigurationType(), motorConf.getName());
    map.dcMotor.put(motorConf.getName(), dcMotor);
  }

  private void mapServoDevice(HardwareMap map, DeviceManager deviceMgr, DeviceConfiguration servoConf, ServoController servoController) {
    if (!servoConf.isEnabled() || !servoConf.getConfigurationType().isDeviceFlavor(ConfigurationType.DeviceFlavor.SERVO)) {
        return;
    }
    HardwareDevice servoDevice = null;

    ServoConfigurationType servoType = (ServoConfigurationType) servoConf.getConfigurationType();
    if (servoType.getServoFlavor() == ServoFlavor.STANDARD) {
      servoDevice = deviceMgr.createServo(servoController, servoConf.getPort(), servoConf.getName());
    } else if (servoType.getServoFlavor() == ServoFlavor.CONTINUOUS) {
      servoDevice = deviceMgr.createCRServo(servoController, servoConf.getPort(), servoConf.getName());
    } else {
      servoDevice = deviceMgr.createCustomServoDevice(servoController, servoConf.getPort(), servoType);
    }
    if (servoDevice != null) {
      addUserDeviceToMap(map, servoConf, servoDevice);
    }
  }

  private void mapLynxServoDevice(HardwareMap map, DeviceManager deviceMgr, DeviceConfiguration servoConf, ServoControllerEx servoController) {
    if (!servoConf.isEnabled() || !servoConf.getConfigurationType().isDeviceFlavor(ConfigurationType.DeviceFlavor.SERVO)) {
      return;
    }
    HardwareDevice servoDevice = null;

    ServoConfigurationType servoType = (ServoConfigurationType) servoConf.getConfigurationType();
    if (servoType.getServoFlavor() == ServoFlavor.STANDARD) {
      servoDevice = deviceMgr.createServoEx(servoController, servoConf.getPort(), servoConf.getName(), servoType);
    } else if (servoType.getServoFlavor() == ServoFlavor.CONTINUOUS) {
      servoDevice = deviceMgr.createCRServoEx(servoController, servoConf.getPort(), servoConf.getName(), servoType);
    } else {
      servoDevice = deviceMgr.createLynxCustomServoDevice(servoController, servoConf.getPort(), servoType);
    }
    if (servoDevice != null) {
      addUserDeviceToMap(map, servoConf, servoDevice);
    }
  }

  private void buildLynxDevices(List<DeviceConfiguration> list, HardwareMap map, DeviceManager deviceMgr, AnalogInputController lynxModule) {
    for (DeviceConfiguration deviceConfiguration : list) {
      ConfigurationType devType = deviceConfiguration.getConfigurationType();
      if (devType.isDeviceFlavor(ConfigurationType.DeviceFlavor.ANALOG_SENSOR)) {
        mapAnalogSensor(map, deviceMgr, lynxModule, deviceConfiguration);
      }
    }
  }

  private void buildLynxDevices(List<DeviceConfiguration> list, HardwareMap map, DeviceManager deviceMgr, DigitalChannelController lynxModule) {
    for (DeviceConfiguration deviceConfiguration : list) {
      ConfigurationType devType = deviceConfiguration.getConfigurationType();
      if (devType.isDeviceFlavor(ConfigurationType.DeviceFlavor.DIGITAL_IO)) {
        mapDigitalDevice(map, deviceMgr, lynxModule, deviceConfiguration);
      }
    }
  }

  /* @see EditLynxModuleActivity#editI2cChannel */
  private void buildLynxI2cDevices(List<LynxI2cDeviceConfiguration> list, HardwareMap map, DeviceManager deviceMgr, LynxModule module) {
    for (LynxI2cDeviceConfiguration deviceConfiguration : list) {
      ConfigurationType devType = deviceConfiguration.getConfigurationType();
      if (devType==BuiltInConfigurationType.IR_SEEKER_V3) {
        mapIrSeekerV3Device(map, deviceMgr, module, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.ADAFRUIT_COLOR_SENSOR) {
        mapAdafruitColorSensor(map, deviceMgr, module, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.LYNX_COLOR_SENSOR) {
        mapLynxColorSensor(map, deviceMgr, module, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.COLOR_SENSOR) {
        mapModernRoboticsColorSensor(map, deviceMgr, module, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.GYRO) {
        mapModernRoboticsGyro(map, deviceMgr, module, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.NOTHING) {
        // nothing to do
      }
      else if (devType.isDeviceFlavor(ConfigurationType.DeviceFlavor.I2C)) {
        if (devType instanceof I2cDeviceConfigurationType) {
          mapUserI2cDevice(map, deviceMgr, module, deviceConfiguration);
        }
      }
      else {
        RobotLog.w("Unexpected device type connected to I2c Controller while parsing XML: " + devType.toString());
      }
    }
  }

  private void mapIrSeekerV3Device(HardwareMap map, DeviceManager deviceMgr, LynxModule module, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    IrSeekerSensor irSeekerSensor = deviceMgr.createMRI2cIrSeekerSensorV3(module, devConf.getI2cChannel(), devConf.getName());
    map.irSeekerSensor.put(devConf.getName(), irSeekerSensor);
  }

  private void mapDigitalDevice(HardwareMap map, DeviceManager deviceMgr, DigitalChannelController digitalChannelController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    HardwareDevice digitalDevice = null;

    if (devConf.getConfigurationType() == BuiltInConfigurationType.TOUCH_SENSOR) {
      digitalDevice = deviceMgr.createMRDigitalTouchSensor(digitalChannelController, devConf.getPort(), devConf.getName());
    } else if (devConf.getConfigurationType().isDeviceFlavor(ConfigurationType.DeviceFlavor.DIGITAL_IO)) {
      DigitalIoDeviceConfigurationType digitalDeviceType = (DigitalIoDeviceConfigurationType) devConf.getConfigurationType();
      digitalDevice = deviceMgr.createDigitalDevice(digitalChannelController, devConf.getPort(), digitalDeviceType);
    }
    if (digitalDevice != null) {
      addUserDeviceToMap(map, devConf, digitalDevice);
    }
  }

  private void mapAnalogSensor(HardwareMap map, DeviceManager deviceMgr, AnalogInputController analogInputController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    if (devConf.getConfigurationType().isDeviceFlavor(ConfigurationType.DeviceFlavor.ANALOG_SENSOR)) {
      AnalogSensorConfigurationType analogSensorType = (AnalogSensorConfigurationType) devConf.getConfigurationType();
      HardwareDevice analogSensorDevice = deviceMgr.createAnalogSensor(analogInputController, devConf.getPort(), analogSensorType);
      if (analogSensorDevice != null) {
        addUserDeviceToMap(map, devConf, analogSensorDevice);
      }
    }
  }

  private void mapPwmOutputDevice(HardwareMap map, DeviceManager deviceMgr, PWMOutputController pwmOutputController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    PWMOutput pwmOutput = deviceMgr.createPwmOutputDevice(pwmOutputController, devConf.getPort(), devConf.getName());
    map.pwmOutput.put(devConf.getName(), pwmOutput);
  }

  private void mapI2cDeviceSynch(HardwareMap map, DeviceManager deviceMgr, LynxModule module, DeviceConfiguration devConf) {
    I2cDeviceSynch i2cDeviceSynch = deviceMgr.createI2cDeviceSynch(module, devConf.getI2cChannel(), devConf.getName());
    map.i2cDeviceSynch.put(devConf.getName(), i2cDeviceSynch);
  }

  private void addUserDeviceToMap(HardwareMap map, DeviceConfiguration deviceConf, HardwareDevice deviceInstance) {
    map.put(deviceConf.getName(), deviceInstance);

    // Put in the appropriate device-specific mappings
    for (HardwareMap.DeviceMapping mapping : map.allDeviceMappings) {
      if (mapping.getDeviceTypeClass().isInstance(deviceInstance)) {
        maybeAddToMapping(mapping, deviceConf.getName(), mapping.cast(deviceInstance));
      }
    }
  }

  private <T extends HardwareDevice> void maybeAddToMapping(HardwareMap.DeviceMapping<T> mapping, String name, T device) {
    if (!mapping.contains(name)) {
      mapping.putLocal(name, device);
    }
  }

  private void mapUserI2cDevice(HardwareMap map, DeviceManager deviceMgr, LynxModule lynxModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    I2cDeviceConfigurationType userType = (I2cDeviceConfigurationType)devConf.getConfigurationType();
    HardwareDevice hardwareDevice = deviceMgr.createUserI2cDevice(lynxModule, devConf.getI2cChannel(), userType, devConf.getName());
    if (hardwareDevice != null) {
      addUserDeviceToMap(map, devConf, hardwareDevice);
    }
  }

  private void mapWebcam(HardwareMap map, DeviceManager deviceManager, WebcamConfiguration webcamConfiguration) throws RobotCoreException, InterruptedException {
    if (!webcamConfiguration.isEnabled()) return;
    SerialNumber serialNumber = webcamConfiguration.getSerialNumber();
    if (webcamConfiguration.getAutoOpen()) {
      RobotLog.ee(TAG, "support for auto-opening webcams is not yet implemented: %s", serialNumber);
      return;
    }
    WebcamName webcamName = deviceManager.createWebcamName(serialNumber, webcamConfiguration.getName());
    if (webcamName != null) {
      map.put(serialNumber, webcamConfiguration.getName(), webcamName);
    }
  }

  private void mapLynxUsbDevice(HardwareMap map, DeviceManager deviceMgr, LynxUsbDeviceConfiguration lynxUsbDeviceConfiguration) throws RobotCoreException, InterruptedException {
    if (!lynxUsbDeviceConfiguration.isEnabled()) return;
    // Make a new LynxUsbDevice
    SerialNumber serialNumber = lynxUsbDeviceConfiguration.getSerialNumber();
    LynxUsbDevice lynxUsbDevice = (LynxUsbDevice)deviceMgr.createLynxUsbDevice(serialNumber, lynxUsbDeviceConfiguration.getName());
    try {
      // If the system made up this device, let the live device know that too
      if (lynxUsbDeviceConfiguration.isSystemSynthetic()) {
        lynxUsbDevice.setSystemSynthetic(true);
      }

      // Are we the first USB-attached (as opposed to embedded) LynxUsbDevice?
      boolean isFirstLynxUsbDevice = !LynxConstants.isEmbeddedSerialNumber(serialNumber);
      for (LynxUsbDevice usbDevice : map.getAll(LynxUsbDevice.class)) {
        if (!LynxConstants.isEmbeddedSerialNumber(usbDevice.getSerialNumber())) {
          isFirstLynxUsbDevice = false;
          break;
        }
      }

      // Make all the modules first, since we need to ping the parent before pinging anyone else,
      // and so we need to figure out who that is.
      //
      List<LynxModule> potentialModules = new ArrayList<LynxModule>();
      Map<Integer, String> moduleNames = new HashMap<Integer, String>();
      final int parentModuleAddress = lynxUsbDeviceConfiguration.getParentModuleAddress();
      for (DeviceConfiguration moduleConfiguration : lynxUsbDeviceConfiguration.getModules()) {
        int moduleAddress = moduleConfiguration.getPort();
        moduleNames.put(moduleAddress, moduleConfiguration.getName());
        //
        LynxModule module = (LynxModule)deviceMgr.createLynxModule(lynxUsbDevice, moduleAddress, parentModuleAddress==moduleAddress, moduleConfiguration.getName());
        potentialModules.add(module);

        // If the system made up this device, let the live device know that too
        if (((LynxModuleConfiguration)moduleConfiguration).isSystemSynthetic()) {
          module.setSystemSynthetic(true);
        }
      }

      // Attach all the LynxModules to that LynxUsbDevice, parents first, so that parents get pinged first. Note that if some
      // modules aren't actually there, or are there but are wedged, these may throw exceptions.
      Map<Integer, LynxModule> connectedModules = new HashMap<Integer, LynxModule>();
      for (LynxModule module : potentialModules) {
        if (module.isParent()) { // nb: there should be only one parent
         connectModule(lynxUsbDevice, module, moduleNames, connectedModules, isFirstLynxUsbDevice && connectedModules.isEmpty());
        }
      }
      for (LynxModule module : potentialModules) {
        if (!module.isParent()) {
          connectModule(lynxUsbDevice, module, moduleNames, connectedModules, false /*non-parents are connected over 485, so they can't charge us*/);
        }
      }

      // Finish adding all the pieces to the modules we successfully talked to
      mapLynxModuleComponents(map, deviceMgr, lynxUsbDeviceConfiguration, lynxUsbDevice, connectedModules);

      // For the things that worked, remember the module names in the hwmap
      for (Map.Entry<Integer, LynxModule> pair : connectedModules.entrySet()) {
        int moduleAddress = pair.getKey();
        LynxModule lynxModule = pair.getValue();
        map.put(lynxModule.getModuleSerialNumber(), moduleNames.get(moduleAddress), lynxModule);
      }

      // Remember the LynxUsbDevice too, so we we'll only enable one to charge the RC battery
      map.put(serialNumber, lynxUsbDeviceConfiguration.getName(), lynxUsbDevice);

    } catch (LynxNackException e) {
      throw e.wrap();
    } catch (RobotCoreException|RuntimeException e) {
      lynxUsbDevice.close();
      map.remove(serialNumber, lynxUsbDeviceConfiguration.getName(), lynxUsbDevice);
      throw e;
    }
  }

  /**
   * If this method returns without throwing an exception, you can be sure that an equivalent
   * LynxModule instance is registered with the LynxUsbDevice. However, it is not guaranteed that
   * the exact LynxModule instance passed in is the one registered to the LynxUsbDevice. ALWAYS use
   * the one connected to the connectedModules map parameter, not one originally passed in.
   */
  private void connectModule(LynxUsbDevice lynxUsbDevice, LynxModule module, Map<Integer,String> moduleNames, Map<Integer,LynxModule> connectedModules, boolean enableCharging) throws InterruptedException {
    try {
      // addConfiguredModule may return a different LynxModule instance than we passed in, and if it
      // does that, that's the one we should be using.
      module = lynxUsbDevice.addConfiguredModule(module);    // this will throw if there's a problem
      if (enableCharging) {
        // When we first connect, we enable phone charging because we KNOW that the modules
        // aren't connected to a PC where that will be a problem. Note that if for some reason
        // the firmware auto-disables the charging, we won't in the interests of safety turn
        // it back on again unless we go through another arming cycle.
        //
        // Note that we only ever enable charging for the first module attached so as to avoid
        // voltage regulators pushing and pulling at each other (our caller guarantees that
        // enableCharging will only be true for that first module).
        module.enablePhoneCharging(true);
      }
      connectedModules.put(module.getModuleAddress(), module);
    } catch (RobotCoreException|LynxNackException|RuntimeException e) {
      lynxUsbDevice.noteMissingModule(module, moduleNames.get(module.getModuleAddress()));
    }
  }

  private void mapLynxModuleComponents(HardwareMap map, DeviceManager deviceMgr, LynxUsbDeviceConfiguration lynxUsbDeviceConfiguration, LynxUsbDevice lynxUsbDevice, Map<Integer, LynxModule> connectedModules) throws LynxNackException, RobotCoreException, InterruptedException {

    // Hook up the pieces to each module
    for (DeviceConfiguration moduleConfiguration : lynxUsbDeviceConfiguration.getModules()) {
      LynxModule module = connectedModules.get(moduleConfiguration.getPort());

      // Ignore modules that ultimately didn't connect
      if (module==null) continue;

      LynxModuleConfiguration lynxModuleConfiguration = (LynxModuleConfiguration)moduleConfiguration;

      // For each module, hook up motor controller and motors
      LynxDcMotorController mc = new LynxDcMotorController(context, module);
      map.dcMotorController.put(moduleConfiguration.getName(), mc);
      for (DeviceConfiguration motorConf : lynxModuleConfiguration.getMotors()) {
        if (motorConf.isEnabled()) {
          DcMotor m = deviceMgr.createDcMotorEx(mc, motorConf.getPort(), (MotorConfigurationType) motorConf.getConfigurationType(), motorConf.getName());
          map.dcMotor.put(motorConf.getName(), m);
        }
      }

      // And hook up servo controller and servos
      LynxServoController sc = new LynxServoController(context, module);
      map.servoController.put(moduleConfiguration.getName(), sc);
      for (DeviceConfiguration servoConf : lynxModuleConfiguration.getServos()) {
        mapLynxServoDevice(map, deviceMgr, servoConf, sc);
      }

      // And a voltage sensor
      LynxVoltageSensor voltageSensor = new LynxVoltageSensor(context, module);
      map.voltageSensor.put(moduleConfiguration.getName(), voltageSensor);

      // Also an AnalogInputController
      LynxAnalogInputController analogInputController = new LynxAnalogInputController(context, module);
      map.put(moduleConfiguration.getName(), analogInputController);
      buildLynxDevices(lynxModuleConfiguration.getAnalogInputs(), map, deviceMgr, analogInputController);

      // And a digital channel controller
      LynxDigitalChannelController digitalChannelController = new LynxDigitalChannelController(context, module);
      map.put(moduleConfiguration.getName(), digitalChannelController);
      buildLynxDevices(lynxModuleConfiguration.getDigitalDevices(), map, deviceMgr, digitalChannelController);

      // And I2c devices
      buildLynxI2cDevices(lynxModuleConfiguration.getI2cDevices(), map, deviceMgr, module);
    }
  }

  private void mapAdafruitColorSensor(HardwareMap map, DeviceManager deviceMgr, LynxModule module, DeviceConfiguration devConf) {
    ColorSensor colorSensor = deviceMgr.createAdafruitI2cColorSensor(module, devConf.getI2cChannel(), devConf.getName());
    map.colorSensor.put(devConf.getName(), colorSensor);
  }

  private void mapLynxColorSensor(HardwareMap map, DeviceManager deviceMgr, LynxModule module, DeviceConfiguration devConf) {
    ColorSensor sensor = deviceMgr.createLynxColorRangeSensor(module, devConf.getI2cChannel(), devConf.getName());
    map.colorSensor.put(devConf.getName(), sensor);
    map.opticalDistanceSensor.put(devConf.getName(), (OpticalDistanceSensor)sensor);
  }

  private void mapModernRoboticsColorSensor(HardwareMap map, DeviceManager deviceMgr, LynxModule lynxModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    ColorSensor colorSensor = deviceMgr.createModernRoboticsI2cColorSensor(lynxModule, devConf.getI2cChannel(), devConf.getName());
    map.colorSensor.put(devConf.getName(), colorSensor);
  }

  private void mapModernRoboticsGyro(HardwareMap map, DeviceManager deviceMgr, LynxModule module, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    GyroSensor gyroSensor = deviceMgr.createModernRoboticsI2cGyroSensor(module, devConf.getI2cChannel(), devConf.getName());
    map.gyroSensor.put(devConf.getName(), gyroSensor);
  }

  //------------------------------------------------------------------------------------------------
  // Serial number display name management
  //------------------------------------------------------------------------------------------------

  public static void noteSerialNumberType(Context context, SerialNumber serialNumber, String typeName) {
    SerialNumber.noteSerialNumberType(serialNumber, typeName);
  }

  public static String getDeviceDisplayName(Context context, SerialNumber serialNumber) {
    return SerialNumber.getDeviceDisplayName(serialNumber);
  }
}
