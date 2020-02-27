package org.firstinspires.ftc.teamcode;


import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Date;


/**
 * Provide utility methods for debug logging
 */
public class DbgLog {
    public DbgLog() {
    }

    /**
     * Tag used by logcat
     */
    public static final String TAG = "FIRST";

    public static final String ERROR_PREPEND = "### ERROR: ";

    private File file;
    private FileOutputStream output;
    private OutputStreamWriter outputStreamWriter;
    private SimpleDateFormat dateFormat = new SimpleDateFormat("MM/dd/yy hh:mm:ss:SSS ");
    private Date date = new Date();


    public void openLog() {
        String path = Environment.getExternalStorageDirectory().toString();
        file = new File(path, "10435Log.txt");
        {
            try {
                output = new FileOutputStream(file);
                outputStreamWriter = new OutputStreamWriter(output);
            } catch (FileNotFoundException e) {
                Log.e("Unable to open ", file.getName());
            }
        }
    }

    public void msg(String message) {
        try {
            date = new Date();
            outputStreamWriter.append(dateFormat.format(date)).append(message).append("\n");
        } catch (IOException e) {
            Log.e("Unable to write to ", file.getName());
        }
    }

    public void close(){
        try {
            outputStreamWriter.close();
            output.close();
        } catch (IOException e) {
            Log.e("Failed to close ", file.getName());
        }
    }

    /*public static void msg(String message) {
            android.util.Log.i(TAG, message);
    }

    public static void msg(String format, Object... args) {
        msg(String.format(format, args));
    }
*/

    /**
     * Log an error message
     * <p>
     * Messages will be prepended with the ERROR_PREPEND string
     *
     * @param message
     */
    public static void error(String message) {
        android.util.Log.e(TAG, ERROR_PREPEND + message);
    }

    public static void error(String format, Object... args) {
        error(String.format(format, args));
    }

    /*
    public static void logStacktrace(Exception e) {
        msg(e.toString());
        for (StackTraceElement el : e.getStackTrace()) {
            msg(el.toString());
        }
    }

     */
}