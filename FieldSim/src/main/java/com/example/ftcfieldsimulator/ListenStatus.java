package com.example.ftcfieldsimulator;

public enum ListenStatus {
    NOT_STARTED,
    LISTENING,
    COMPLETED_SUCCESSFULLY, // Received "end"
    TIMED_OUT,              // 10-second activity timeout
    ERROR,
    STOPPED                 // Explicitly stopped by user
}