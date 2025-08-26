package com.example.drawing;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;
import android.view.View;

import androidx.annotation.NonNull;

import java.util.ArrayList;

public class Drawing extends View {
    private final ArrayList<CurvePoint> points;
    private int robotIndex = 0;

    private final Paint pathPaint = new Paint();
    private final Paint robotPaint = new Paint();
    private Bitmap fieldBitmap;
    private final Rect backgroundRect = new Rect();
    private final Rect fieldImgRect = new Rect();
    private final RectF robotDrawing = new RectF();


    public void initDrawing()
    {

        fieldBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.ftc_field);


        pathPaint.setColor(Color.BLUE);
        pathPaint.setStrokeWidth(5);

        robotPaint.setColor(Color.RED);
        Paint bgPaint = new Paint();
        bgPaint.setColor(Color.LTGRAY);

        //change this delay millis to speed up/slow down animation
        postDelayed(animator, 700);
    }


    public Drawing(Context context, ArrayList<CurvePoint> points) {
        super(context);
        this.points = points;
        initDrawing();
    }

    //android gets upset without a default constructor so this is here but probably won't ever be needed
    public Drawing(Context context)
    {
        super(context);
        this.points = new ArrayList<>();
        initDrawing();
    }


    private final Runnable animator = new Runnable() {
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
    protected void onDraw(@NonNull Canvas canvas) {
        super.onDraw(canvas);

        float scaleX = getWidth() / 72f;   // FTC field width is 72 inches
        float scaleY = getHeight() / 72f;  // FTC field height is 72 inches



        // Draw field background
        if (fieldBitmap != null) {
            fieldImgRect.set(0, 0, fieldBitmap.getWidth(), fieldBitmap.getHeight());
            backgroundRect.set(0, 0, getWidth(), getHeight());
            canvas.drawBitmap(fieldBitmap, fieldImgRect, backgroundRect, null);
        }


        double startX = 0;
        double startY = 20;
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

        float robotPixelsToCornerX = 8.124F * 6;
        float robotPixelsToCornerY = 10F * 6;
        robotDrawing.set(robotCenterX - robotPixelsToCornerX, robotCenterY - robotPixelsToCornerY, robotCenterX + robotPixelsToCornerX, robotCenterY + robotPixelsToCornerY);

            float rotation = Float.isNaN((float) Math.toDegrees(r.slowDownTurnRadians)) ?
                    0f : (float) Math.toDegrees(r.slowDownTurnRadians);


            canvas.save();
            canvas.rotate(rotation, robotCenterX, robotCenterY);
            canvas.drawRect(robotDrawing, robotPaint);
            canvas.restore();
        }


    }

