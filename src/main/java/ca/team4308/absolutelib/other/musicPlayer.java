package ca.team4308.absolutelib.other;

import com.ctre.phoenix.music.Orchestra;

import java.lang.reflect.Array;
import java.nio.file.Path;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import ca.team4308.absolutelib.wrapper.MotorWrapper;


/**
 * musicPlayer class to manage an Orchestra of TalonFX motors to play music files. 
 * Supports loading, playing, queuing, and stopping songs.
 
 */
public class musicPlayer {

    Orchestra orchestra = new Orchestra();

    ArrayList<String> songs = new ArrayList<String>();
    public musicPlayer(Orchestra orchestra) {
        this.orchestra = orchestra;
    }

    public void addInstruments(MotorWrapper[] motors) {
        for (MotorWrapper motor : motors) {
            if (motor.isTalonFX()) {
                orchestra.addInstrument((TalonFX) motor.get());
            }
        }
    }

    
    public void loadSong(String songPath) {
        orchestra.loadMusic(songPath);
    }
    public void playSong() {
        orchestra.play();
    }

    public void queueSong(String songPath) {
        songs.add(songPath);
    }

    public void playNextSong() {
        if (songs.size() > 0) {
            String nextSong = songs.remove(0);
            orchestra.loadMusic(nextSong);
            orchestra.play();
        }
    }

    public void stopSong() {
        orchestra.stop();
    }

    public void removeSong(int i) {
        if (songs.size() > 0) {
            songs.remove(i);
        }
    }

    public ArrayList getPlaylist() {
        return songs;
    }


}
