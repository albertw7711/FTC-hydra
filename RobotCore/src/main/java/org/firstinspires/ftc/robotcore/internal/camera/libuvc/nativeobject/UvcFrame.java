/*
Copyright (c) 2017 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.internal.camera.libuvc.nativeobject;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.libuvc.constants.UvcFrameFormat;
import org.firstinspires.ftc.robotcore.internal.system.NativeObject;

import java.nio.ByteBuffer;

/**
 * The Java manifestation of a native uvc_frame
 */
@SuppressWarnings("WeakerAccess")
public class UvcFrame extends NativeObject<UvcContext>
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public UvcFrame(long pointer, MemoryAllocator memoryAllocator, UvcContext uvcContext)
        {
        super(pointer, memoryAllocator, TraceLevel.VeryVerbose);
        setParent(uvcContext);
        }

    public long getPointer()
        {
        return pointer;
        }

    public UvcFrame copy()
        {
        return new UvcFrame(checkAlloc(nativeCopyFrame(pointer)), MemoryAllocator.EXTERNAL, getParent());
        }

    @Override protected void destructor()
        {
        if (memoryAllocator == MemoryAllocator.EXTERNAL) // some frames might use malloc
            {
            if (pointer != 0)
                {
                nativeFreeFrame(pointer);
                clearPointer();
                }
            }
        super.destructor();
        }

    //----------------------------------------------------------------------------------------------
    // Conversion
    //----------------------------------------------------------------------------------------------

    public void copyToBitmap(Bitmap bitmap)
        {
        switch (getFrameFormat())
            {
            case YUY2:  yuy2ToBitmap(bitmap);
            default:    break; // throw?
            }
        }

    /*
     * https://msdn.microsoft.com/en-us/library/windows/desktop/dd206750(v=vs.85).aspx
     * formulas: https://msdn.microsoft.com/en-us/library/ms893078.aspx
     * https://github.com/yigalomer/Yuv2RgbRenderScript/blob/master/src/com/example/yuv2rgbrenderscript/RenderScriptHelper.java
     * C:\Android\410c\build\frameworks\rs\cpu_ref\rsCpuIntrinsicYuvToRGB.cpp
     */
    protected void yuy2ToBitmap(Bitmap bitmap)
        {
        if (bitmap.getConfig() != Bitmap.Config.ARGB_8888)
            {
            RobotLog.ee(getTag(), "conversion to %s not yet implemented; ignored", bitmap.getConfig());
            return;
            }

        nativeCopyFrameToBmp(pointer, bitmap);
        }

    //----------------------------------------------------------------------------------------------
    // Accessing
    //----------------------------------------------------------------------------------------------

    public UvcContext getContext()
        {
        return getParent();
        }

    public int getWidth()
        {
        return getInt(Fields.width.offset());
        }

    public int getHeight()
        {
        return getInt(Fields.height.offset());
        }

    public UvcFrameFormat getFrameFormat()
        {
        return UvcFrameFormat.from(getInt(Fields.frameFormat.offset()));
        }

    public int getStride()
        {
        return getInt(Fields.stride.offset());
        }

    /** Frame number. May skip if we drop frames. Is strictly monotonically increasing. */
    public long getFrameNumber()
        {
        return getUInt(Fields.frameNumber.offset());
        }

    public long getCaptureTime()
        {
        return getLong(Fields.captureTime.offset());
        }

    public ByteBuffer getImageByteBuffer()
        {
        ByteBuffer result = (ByteBuffer) nativeGetImageByteBuffer(pointer);
        result.order(this.byteOrder);
        return result;
        }

    public byte[] getImageData()
        {
        return getImageData(new byte[getImageSize()]);
        }

    public byte[] getImageData(byte[] byteArray)
        {
        int cbNeeded = getImageSize();
        if (byteArray.length != cbNeeded)
            {
            byteArray = new byte[cbNeeded];
            }
        nativeCopyImageData(pointer, byteArray, byteArray.length);
        return byteArray;
        }

    public int getImageSize()
        {
        return getSizet(Fields.cbData.offset());
        }

    public long getImageBuffer()
        {
        return getLong(Fields.pbData.offset());
        }

    //----------------------------------------------------------------------------------------------
    // Native
    //----------------------------------------------------------------------------------------------

    protected static int[] fieldOffsets = nativeGetFieldOffsets(Fields.values().length);

    protected int getStructSize()
        {
        return fieldOffsets[Fields.sizeof.ordinal()];
        }

    protected enum Fields
        {
        sizeof,
        pbData,
        cbData,
        cbAllocated,
        width,
        height,
        frameFormat,
        stride,
        frameNumber,
        pts,
        captureTime,
        sourceClockReference,
        pContext;
        public int offset() { return fieldOffsets[this.ordinal()]; }
        }

    protected native static int[] nativeGetFieldOffsets(int cFieldExpected);
    protected native static Object nativeGetImageByteBuffer(long pointer);
    protected native static void nativeCopyImageData(long pointer, byte[] byteArray, int byteArrayLength);
    protected native static void nativeCopyFrameToBmp(long pointer, Bitmap bmp);
    protected native static long nativeCopyFrame(long pointer);
    protected native static void nativeFreeFrame(long pointer);
    }
