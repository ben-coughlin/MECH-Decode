package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kickstand {
   private final Servo kickstand;

   private double lowered = 0;
   private double raised = .9;

   public Kickstand(HardwareMap hwMap)
   {
       kickstand = hwMap.get(Servo.class, "kickstand");
       raiseKickstand();
   }

   public void lowerKickstand()
   {
       //kickstand.setPosition(lowered);
   }
   public void raiseKickstand()
   {
       //kickstand.setPosition(raised);
   }




}
