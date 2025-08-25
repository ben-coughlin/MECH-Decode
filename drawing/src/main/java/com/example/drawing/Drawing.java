package com.example.drawing;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;
import android.util.Log;
import android.view.View;

import androidx.annotation.NonNull;

import java.util.ArrayList;

public class Drawing extends View {
    private final ArrayList<CurvePoint> points;
    private int robotIndex = 0;

    private final Paint pathPaint = new Paint();
    private final Paint robotPaint = new Paint();
    private final Bitmap fieldBitmap;

    private double startX = 0;
    private double startY = 20;
    private boolean startSet = false;

    private float robotPixelsToCornerX = 8.124F * 6;
    private float robotPixelsToCornerY = 8.75F * 6;
   

    public Drawing(Context context, ArrayList<CurvePoint> points) {
        super(context);
        this.points = points;

        fieldBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.ftc_field);


        pathPaint.setColor(Color.BLUE);
        pathPaint.setStrokeWidth(5);

        robotPaint.setColor(Color.RED);
        Paint bgPaint = new Paint();
        bgPaint.setColor(Color.LTGRAY);

        postDelayed(animator, 200);
    }

    private Runnable animator = new Runnable() {
        @Override
        public void run() {
            if (robotIndex < points.size() - 1) robotIndex++;
            invalidate();
            postDelayed(this, 200);
        }
    };

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        int width = MeasureSpec.getSize(widthMeasureSpec);
        int height = MeasureSpec.getSize(heightMeasureSpec);

        // Use the smaller of width and height to make a square
        int size = Math.min(width, height);

        // Set both width and height to this size
        setMeasuredDimension(size, size);
    }


    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        float scaleX = getWidth() / 72f;   // FTC field width is 72 inches
        float scaleY = getHeight() / 72f;  // FTC field height is 72 inches



        // Draw field background
        if (fieldBitmap != null) {
            Rect src = new Rect(0, 0, fieldBitmap.getWidth(), fieldBitmap.getHeight());
            Rect dst = new Rect(0, 0, getWidth(), getHeight());
            canvas.drawBitmap(fieldBitmap, src, dst, null);
        }


        for (int i = 0; i < points.size() - 1; i++) {
            CurvePoint p1 = points.get(i);
            CurvePoint p2 = points.get(i + 1);

            float x1 = (float) ((p1.x + startX) * scaleX);
            float y1 = getHeight() - (float) ((p1.y + startY) * scaleY); // flip Y
            float x2 = (float) ((p2.x + startX) * scaleX);
            float y2 = getHeight() - (float) ((p2.y + startY) * scaleY);

            canvas.drawLine(x1, y1, x2, y2, pathPaint);
        }

        // Draw robot
        CurvePoint r = points.get(robotIndex);
        float robotCenterX = (float) ((r.x + startX) * scaleX);
        float robotCenterY = getHeight() - (float) ((r.y + startY) * scaleY);

            RectF rect = new RectF(robotCenterX - robotPixelsToCornerX, robotCenterY - robotPixelsToCornerY, robotCenterX + robotPixelsToCornerX, robotCenterY + robotPixelsToCornerY);

            float rotation = Float.isNaN((float) Math.toDegrees(r.slowDownTurnRadians)) ?
                    0f : (float) Math.toDegrees(r.slowDownTurnRadians);


            canvas.save();
            canvas.rotate(rotation, robotCenterX, robotCenterY);
            canvas.drawRect(rect, robotPaint);
            canvas.restore();
        }
    }

