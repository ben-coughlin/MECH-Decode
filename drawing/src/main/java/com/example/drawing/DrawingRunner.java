package com.example.drawing;

import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import java.util.ArrayList;

public class DrawingRunner extends AppCompatActivity {


    protected void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);

            //starter variables
            ArrayList<CurvePoint> points = new ArrayList<>();

            double SCALE_FACTOR = 1;

        int stateStartingX = 0;
        int stateStartingY = 0;



        // put points between here - - - - - - - - - - - - - - -  >
        points.add(new CurvePoint(stateStartingX, stateStartingY,
                0, 0, 0, 0, 0, 0));

        points.add(new CurvePoint(20, 0,
                0.4 * SCALE_FACTOR, 0.3 * SCALE_FACTOR, 20, 10, // changed move speed from .35 to .45
                Math.toRadians(0), 0.6));

        points.add(new CurvePoint(25.75, 15,
                0.3 * SCALE_FACTOR, 0.2 * SCALE_FACTOR, 20, 10, // changed move speed from .2 to .25
                Math.toRadians(60), 0.6));
        points.add(new CurvePoint(25.75, 20,
                0.3 * SCALE_FACTOR, 0.2 * SCALE_FACTOR, 20, 10, // changed move speed from .2 to .25
                Math.toRadians(0), 0.6));



        // and here - - - - - - - - - - - - - - - - - - - - - - - - >
        setContentView(new Drawing(this, points));
        }
    }

