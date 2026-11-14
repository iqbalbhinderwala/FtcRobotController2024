package org.firstinspires.ftc.teamcode.John.Util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Small CSV logger for reviewing runs offline. Writes to
 * /sdcard/FIRST/john_v2_logs/r<run>_<name>_<time>.csv on the Control Hub; copy with:
 *   adb pull /sdcard/FIRST/john_v2_logs
 *
 * John v2 (the rebuild for the second competition, 2026-11) logs to its own folder, FOLDER, with run
 * numbers from FIRST_RUN, so its logs never mix with v1's (/sdcard/FIRST/john_logs, r0001-r0288):
 * the shooter, intakes and wiring changed, and v1 data (e.g. Shot Calibration coverage) does not
 * apply. A v3 would get the next folder and FIRST_RUN 2001.
 *
 * The Control Hub has no battery-backed clock: its time restarts at 1969-12-31 on every boot, so
 * times repeat across boots. The run number does not: it is one more than the highest run number
 * already in the folder, so sorting the file names puts the runs in order and the last is the latest.
 *
 * Logging never throws: if the file cannot be written, the OpMode keeps running and error() says why.
 */
public class CsvLog {

    /** This robot version's log folder under /sdcard/FIRST. */
    public static final String FOLDER = "john_v2_logs";
    /** Its first run number (v1 ran r0001-r0288). */
    public static final int FIRST_RUN = 1001;

    private static final int FLUSH_EVERY_ROWS = 25;
    private static final Pattern RUN_NUMBER = Pattern.compile("^r(\\d+)_");

    private BufferedWriter writer;
    private String fileName = "";
    private String error = "";
    private int rows = 0;

    public CsvLog(String name, String header) {
        this(name, header, nextRunNumber());
    }

    /** With a given run number, so several logs of one run share it (see nextRunNumber()). */
    public CsvLog(String name, String header, int run) {
        try {
            File dir = logDir();
            if (!dir.exists() && !dir.mkdirs()) {
                throw new IOException("cannot create " + dir);
            }
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            fileName = String.format(Locale.US, "r%04d_%s_%s.csv", run, name, stamp);
            writer = new BufferedWriter(new FileWriter(new File(dir, fileName)));
            writer.write(header);
            writer.newLine();
        } catch (IOException e) {
            writer = null;
            error = e.getMessage();
        }
    }

    /** The log folder of this robot version. */
    public static File logDir() {
        return new File(AppUtil.FIRST_FOLDER, FOLDER);
    }

    /** One more than the highest run number among the files already in the log folder (FIRST_RUN if none). */
    public static int nextRunNumber() {
        File dir = logDir();
        int max = 0;
        String[] names = dir.list();
        if (names != null) {
            for (String n : names) {
                Matcher m = RUN_NUMBER.matcher(n);
                if (m.find()) {
                    try {
                        max = Math.max(max, Integer.parseInt(m.group(1)));
                    } catch (NumberFormatException ignored) {
                    }
                }
            }
        }
        return Math.max(FIRST_RUN, max + 1);
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
                // 3 decimals, but small values (gains, coefficients) keep 5 so they are not rounded away
                double d = (Double) v;
                line.append(String.format(Locale.US, d != 0 && Math.abs(d) < 0.1 ? "%.5f" : "%.3f", d));
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

    /** Writes the buffered rows to storage now (for logs that must survive the app being killed). */
    public void flush() {
        if (writer == null) {
            return;
        }
        try {
            writer.flush();
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
