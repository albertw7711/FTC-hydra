/*
 * Copyright 2018 Google LLC
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

package org.firstinspires.ftc.robotcore.external.tfod;

import android.app.Activity;
import androidx.annotation.IdRes;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import android.view.ViewGroup;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import java.util.List;

/**
 * Interface for TensorFlow Object Detector.
 *
 * @author Vasu Agrawal
 * @author lizlooney@google.com (Liz Looney)
 */
public interface TFObjectDetector extends CameraStreamSource {
  /**
   * Loads a TFLite model from the indicated application asset, which must be of
   * type .tflite.
   *
   * @param assetName the name of the .tflite model asset to load
   * @param labels the labels of the objects in the model
   */
  void loadModelFromAsset(String assetName, String... labels);

  /**
   * Loads a TFLite model from the indicated file, which must be a .tflite file and contain the
   * full file path.
   *
   * @param fileName either the name of the file that was uploaded using the Manage page or the full path to the .tflite model file
   * @param labels the labels of the objects in the model
   */
  void loadModelFromFile(String fileName, String... labels);

  /**
   * Activates this TFObjectDetector so it starts recognizing objects.
   */
  void activate();

  /**
   * Deactivates this TFObjectDetector so it stops recognizing objects.
   */
  void deactivate();

  /**
   * Sets the number of pixels to obscure on the left, top, right, and bottom edges of each image
   * passed to the TensorFlow object detector. The size of the images are not changed, but the
   * pixels in the margins are colored black.
   */
  void setClippingMargins(int left, int top, int right, int bottom);

  /**
   * Indicates that only the zoomed center area of each image will be passed to the TensorFlow
   * object detector. For no zooming, set magnification to 1.0. For best results, the aspect ratio
   * should match the aspect ratio of the images that were used to train the TensorFlow model
   * (1.7777 for 16/9).
   */
  void setZoom(double magnification, double aspectRatio);

  /**
   * Returns the list of recognitions, but only if they are different than the last call to {@link #getUpdatedRecognitions()}.
   */
  List<Recognition> getUpdatedRecognitions();

  /**
   * Returns the list of recognitions.
   */
  List<Recognition> getRecognitions();

  /**
   * Perform whatever cleanup is necessary to release all acquired resources.
   */
  void shutdown();

  /**
   * {@link Parameters} provides configuration information for instantiating the TFObjectDetector
   */
  class Parameters {
    /** @deprecated Use minResultConfidence instead */
    @Deprecated
    public double minimumConfidence = 0.4;

    public boolean useObjectTracker = true;

    public boolean isModelTensorFlow2 = false;

    // Additional configuration requested in
    // https://github.com/FIRST-Tech-Challenge/SkyStone/issues/210.
    public boolean isModelQuantized = true;
    public int inputSize = 300; // px
    public int numInterpreterThreads = 1;
    public int numExecutorThreads = 2;
    public int maxNumDetections = 10;

    @Deprecated
    public int timingBufferSize = 10;

    @Deprecated
    public double maxFrameRate = 30;

    public float minResultConfidence = 0.4f;
    public float trackerMaxOverlap = 0.2f;
    public float trackerMinSize = 16.0f;
    public float trackerMarginalCorrelation = 0.75f;
    public float trackerMinCorrelation = 0.3f;

    /**
     * The resource id of the view within {@link #activity} that will be used
     * as the parent for a live monitor which provides feedback as to what objects
     * are detected. If both {@link #tfodMonitorViewIdParent} and {@link #tfodMonitorViewParent}
     * are specified, {@link #tfodMonitorViewParent} is used and {@link #tfodMonitorViewIdParent}
     * is ignored. Optional: if no view monitor parent is indicated, then no detector
     * monitoring is provided. The default is zero, which does not indicate a view parent.
     * @see #tfodMonitorViewParent
     */
    public @IdRes int tfodMonitorViewIdParent = 0;

    /**
     * The view that will be used as the parent for a live monitor which provides
     * feedback as to what objects are detected. If both {@link #tfodMonitorViewIdParent}
     * and {@link #tfodMonitorViewParent} are specified, {@link #tfodMonitorViewParent} is used
     * and {@link #tfodMonitorViewIdParent} is ignored. Optional: if no view monitor parent is
     * indicated, then no detector monitoring is provided. The default is null.
     * @see #tfodMonitorViewIdParent
     */
    public ViewGroup tfodMonitorViewParent = null;

    /**
     * The activity in which the detector is to run. May be null, in which case
     * the contextually current activity will be used.
     */
    public Activity activity = null;

    public Parameters() {
    }

    public Parameters(@IdRes int tfodMonitorViewIdParent) {
      this.tfodMonitorViewIdParent = tfodMonitorViewIdParent;
    }
  }
}
