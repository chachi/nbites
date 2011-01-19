/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package remote;

import ch.ethz.ssh2.Connection;
import ch.ethz.ssh2.Session;
import ch.ethz.ssh2.StreamGobbler;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JOptionPane;
import javax.swing.JPasswordField;

/**
 *
 * @author jgmorris
 */
public class RemoteModel {
    public static final String NAO_USERNAME = "nao";

    // Model State variables
    private Connection connection;
    private Session sesh;
    private BufferedWriter stdinWriter;

    // Robot identifiers
    private String host;
    private String robotName;
    private String username;

    private boolean hasConnection;

    public RemoteModel(){
        hasConnection = false;
    }

    public void connectTo() {
        this.host = host;
        if (host == null)
            return;
        if (hasConnection && connection.getHostname().equals(host)) {
            return;
        }

        String password = askForPassword();
        if (hasConnection = connectToHost(password)){
            startSession();
        }
    }

    private boolean connectToHost(String password){
        closeOldConnection();

        try {
            connection = new Connection(host);
            connection.connect();

            boolean isAuthenticated = connection.authenticateWithPassword(username, password);

            if (isAuthenticated == false) {
                throw new IOException("Authentication failed.");
            }

        } catch (IOException e) {
            System.out.println(e.getMessage());
            return false;
        }
        return true;
    }

    private void closeOldConnection(){
        if (connection != null){
            connection.close();
        }
        if (sesh != null){
            sesh.close();
        }
        hasConnection = false;
    }

    private String askForPassword(){
        JPasswordField passField = new JPasswordField();
        JOptionPane.showMessageDialog(null,
                passField,
                "Please enter Nao password",
                JOptionPane.QUESTION_MESSAGE);
        String password = new String(passField.getPassword());
        if (password == null || password.length() == 0){
            System.out.println("You must enter a password.");
        }
        return password;
    }

    private void startSession(){
        try {
            sesh = connection.openSession();
            sesh.requestDumbPTY();
            sesh.startShell();

            stdinWriter = new BufferedWriter(new OutputStreamWriter(sesh.getStdin()));

        } catch (IOException ex) {
            Logger.getLogger(RemoteModel.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void restartNaoQi(String host){
        setHost(host);
        connectTo();
        if (hasConnection){
            runRemoteCommand("/etc/init.d/naoqi restart");
        }
    }

    public void stopNaoQi(String host){
        setHost(host);
        connectTo();
        if (hasConnection){
            runRemoteCommand("/etc/init.d/naoqi stop");
        }
    }

    public void shutdown(String host){
        setHost(host);
        connectTo();
        if (hasConnection){
            int meantToDoThat = JOptionPane.showConfirmDialog(null, "Did you mean to shutdown " + host + "?", "Just checking", JOptionPane.YES_NO_OPTION);
            if (meantToDoThat == JOptionPane.YES_OPTION){
                runRemoteCommand("shutdown -h 0");
            }
        }
    }

    public void runRemoteCommand(String command){
        try {
            stdinWriter.write(command);
            stdinWriter.newLine();
            stdinWriter.flush();

            StreamGobbler stdout = new StreamGobbler(sesh.getStdout());
            
            /* Show exit status, if available (otherwise "null") */
            if (sesh.getExitStatus() != null && sesh.getExitStatus() != 0) {
                System.out.println("Command exited with code: " + sesh.getExitStatus());
            }
        } catch (IOException ex) {
            System.out.println(ex.getMessage());
            Logger.getLogger(RemoteModel.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public InputStream getStdout(){
        return sesh.getStdout();
    }

    public boolean isConnected(){
        return hasConnection && sesh != null;
    }

    public void setRobot(String robot){
        this.robotName = robot;
    }

    public void setUsername(String username){
        this.username = username;
    }

    public void setHost(String host){
        this.host = host;
    }
}

