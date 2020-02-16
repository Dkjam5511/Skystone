package org.firstinspires.ftc.teamcode.Autonomous;

import java.util.ArrayList;

public class ServoEventManager {
    ArrayList<ServoEvent> events = new ArrayList<>();

    public void addEvent(ServoEvent event){
        events.add(event);
    }

    public void removeEvent(int index){
        events.remove(index);
    }

    public void removeAllEvents(){
        events.clear();
    }

    public ArrayList<ServoEvent> getEvents() {
        return events;
    }
}
