package org.firstinspires.ftc.teamcode.Barry.Util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Small CSV logger for reviewing runs offline. Writes to /sdcard/FIRST/barry_logs/<name>_<time>.csv
 * on the Control Hub; copy with:  adb pull /sdcard/FIRST/barry_logs
 *
 * Logging never throws: if the file cannot be written, the OpMode keeps running and error() says why.
 */
public class CsvLog {

    private static final int FLUSH_EVERY_ROWS = 25;

    private BufferedWriter writer;
    private String fileName = "";
    private String error = "";
    private int rows = 0;

    public CsvLog(String name, String header) {
        try {
            File dir = new File(AppUtil.FIRST_FOLDER, "barry_logs");
            if (!dir.exists() && !dir.mkdirs()) {
                throw new IOException("cannot create " + dir);
            }
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            fileName = name + "_" + stamp + ".csv";
            writer = new BufferedWriter(new FileWriter(new File(dir, fileName)));
            writer.write(header);
            writer.newLine();
        } catch (IOException e) {
            writer = null;
            error = e.getMessage();
        }
    }

    /** Writes one row; values are joined with commas (numbers use '.' decimals). */
    public void row(Object... values) {
        if (writer == null) {
            return;
        }
        StringBuilder line = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) line.append(',');
            Object v = values[i];
            if (v instanceof Double) {
                line.append(String.format(Locale.US, "%.3f", (Double) v));
            } else if (v != null) {
                // Keep the CSV simple: no commas or newlines inside a value
                line.append(v.toString().replace(',', ';').replace('\n', ' '));
            }
        }
        try {
            writer.write(line.toString());
            writer.newLine();
            if (++rows % FLUSH_EVERY_ROWS == 0) {
                writer.flush();
            }
        } catch (IOException e) {
            error = e.getMessage();
            close();
        }
    }

    public void close() {
        if (writer == null) {
            return;
        }
        try {
            writer.close();
        } catch (IOException e) {
            error = e.getMessage();
        }
        writer = null;
    }

    public String fileName() {
        return fileName;
    }

    public String error() {
        return error;
    }
}
