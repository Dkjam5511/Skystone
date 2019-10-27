package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Matrix;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DbgLog;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createScaledBitmap;

public class VuforiaStuff {

    VuforiaLocalizer vuforia;

    public VuforiaStuff(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
    }

    public enum skystonePos {
        LEFT, CENTER, RIGHT;
    }

    public skystonePos vuforiascan() {
        Image rgbImage = null;
        int rgbTries = 0;
        double colorcountL = 0;
        double colorcountC = 0;
        double colorcountR = 0;

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaLocalizer.CloseableFrame closeableFrame = null;
        this.vuforia.setFrameQueueCapacity(1);
        while (rgbImage == null) {
            try {
                closeableFrame = this.vuforia.getFrameQueue().take();
                long numImages = closeableFrame.getNumImages();

                for (int i = 0; i < numImages; i++) {
                    if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgbImage = closeableFrame.getImage(i);
                        if (rgbImage != null) {
                            break;
                        }
                    }
                }
            } catch (InterruptedException exc) {

            } finally {
                if (closeableFrame != null) closeableFrame.close();
            }
        }

        if (rgbImage != null) {

            // copy the bitmap from the Vuforia frame
            Bitmap bitmap = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(rgbImage.getPixels());
/*
            String path = Environment.getExternalStorageDirectory().toString();
            FileOutputStream out = null;
            try {
                File file = new File(path, "Bitmap.png");
                out = new FileOutputStream(file);
                bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                try {
                    if (out != null) {
                        out.flush();
                        out.close();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
*/
            bitmap = createScaledBitmap(bitmap, 320, 180, true);
/*
            try {
                File file = new File(path, "BitmapCropped.png");
                out = new FileOutputStream(file);
                bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                try {
                    if (out != null) {
                        out.flush();
                        out.close();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
*/
            int height;
            int width;
            int pixel;
            int r;
            int g;
            int b;
            int bitmapWidth = bitmap.getWidth();
            int bitmapHeight = bitmap.getHeight();

            for (height = 0; height < bitmapHeight; ++height) {
                for (width = 0; width < (bitmapWidth / 3); ++width) {
                    pixel = bitmap.getPixel(width, height);
                    r = Color.red(pixel);
                    g = Color.green(pixel);
                    b = Color.blue(pixel);
                    colorcountL += Color.rgb(r, g, b);
                }
                for (width = (bitmapWidth / 3); width < (bitmapWidth / 3) * 2; ++width) {
                    pixel = bitmap.getPixel(width, height);
                    r = Color.red(pixel);
                    g = Color.green(pixel);
                    b = Color.blue(pixel);
                    colorcountC += Color.rgb(r, g, b);
                }

                for (width = (bitmapWidth / 3) * 2; width < bitmapWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);
                    r = Color.red(pixel);
                    g = Color.green(pixel);
                    b = Color.blue(pixel);
                    colorcountR += Color.rgb(r, g, b);
                }
            }
        }

        skystonePos pos;

        DbgLog.msg("color L: ", colorcountL);
        DbgLog.msg("color C: ", colorcountC);
        DbgLog.msg("color R: ", colorcountR);

        if (colorcountL < colorcountC && colorcountL < colorcountR) {
            pos = skystonePos.LEFT;
        } else if (colorcountC < colorcountL && colorcountC < colorcountR) {
            pos = skystonePos.CENTER;
        } else {
            pos = skystonePos.RIGHT;
        }

        return pos;
    }
}