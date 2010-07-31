package TOOL.WorldController;

import TOOL.TOOL;
import TOOL.Image.WorldControllerViewer;
/*import TOOL.WorldController.Field;
import TOOL.WorldController.UDPServer;
*/
import TOOL.Data.Field;
import TOOL.Data.LabField2009;
import TOOL.Data.NaoField2009;
import TOOL.Net.RobotViewModule;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.event.*;
import java.util.Date;
import java.util.Vector;
import java.util.Iterator;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.io.*;
import java.lang.Process;

//import java.util.*;

public class WorldController extends JPanel implements KeyListener,
						       ActionListener,
						       ChangeListener
{
    public TOOL t;
    //@todo integrate lines into simulation filter
    //@todo force the simulator to restart when a new simulation made is
    // selected
    // DEBUG SWITCHES
    private boolean DEBUG_RECV_ODOMETRY = false;
    private boolean DEBUG_RECV_VISION = false;
    private boolean DEBUG_RECV_LOCALIZATION = false;

    // Instance of the physical field
    Field the_field;
    LabField2009 labField;
    NaoField2009 naoField;

    // Paints on the field
    WorldControllerPainter painter;
    // Displays the field + the painted info
    WorldControllerViewer viewer;

    // Instance of the DebugViewer Class
    DebugViewer debugViewer;

    // To handle recorder log input, parsing, and passing to display
    LogHandler log;

    // Listen to live UDP data, used to visualize multiple robots' world states
    // simultaneously
    UDPServer udp_server;

	// Stream object information from a robot live
	TCPStreamHandler tcp_handler;

	// Robot view module for purpose of retrieving selected robot
	RobotViewModule robotModule;

    // Modes
    public static final int DO_NOTHING = -1;
    public static final int SIMULATE_OFFLINE = 0;
    public static final int VIEW_UDP_PACKETS = 1;
    public static final int VIEW_MCL_LOG = 2;
    public static final int VIEW_EKF_LOG = 3;
	public static final int VIEW_TCP_STREAM = 4;

    // Speed paramaters
    public static final int ROBOT_FPS = 30;
    public static final int DRAWING_FPS = ROBOT_FPS;
    public static final int LOG_FPS = 25;
    public static final int IO_FPS = 400;
    private static final int CHECK_INTERVAL = 100; // in milliseconds
    private static final int NUM_TICKS_FOR_UNSUCCESSFUL = 5;

    // slider for playback constants
    public static final int FPS_SLIDE_MIN = 1;
    public static final int FPS_SLIDE_MAX = 60;
    public static final int FPS_SLIDE_INIT = 25;

    // TCP Constants
    // data coming off robot via tcp.
    static final int TCP_NUM_LOC_DATA = 21;

    // Loc Debug Data Indexes
    public final static int TCP_LOC_MY_X = 0;
    public final static int TCP_LOC_MY_Y = 1;
    public final static int TCP_LOC_MY_H = 2;
    public final static int TCP_LOC_MY_UNCERT_X = 3;
    public final static int TCP_LOC_MY_UNCERT_Y = 4;
    public final static int TCP_LOC_MY_UNCERT_H = 5;
    public final static int TCP_LOC_BALL_X = 6;
    public final static int TCP_LOC_BALL_Y = 7;
    public final static int TCP_LOC_BALL_UNCERT_X = 8;
    public final static int TCP_LOC_BALL_UNCERT_Y = 9;
    public final static int TCP_LOC_BALL_VEL_X = 10;
    public final static int TCP_LOC_BALL_VEL_Y = 11;
    public final static int TCP_LOC_BALL_UNCERT_VEL_X = 12;
    public final static int TCP_LOC_BALL_UNCERT_VEL_Y = 13;
    public final static int TCP_LOC_BALL_DIST = 14;
    public final static int TCP_LOC_BALL_BEARING = 15;
    public final static int TCP_LOC_ODO_X = 16;
    public final static int TCP_LOC_ODO_Y = 17;
    public final static int TCP_LOC_ODO_H = 18;
    public final static int TCP_LOC_TEAM_COLOR = 19;
    public final static int TCP_LOC_PLAYER_NUMBER = 20;


    static final int NUM_LANDMARKS = 21;

    // Landmark Constants
    public final static int LANDMARK_CORNER = -1;
    public final static int LANDMARK_MY_GOAL_LEFT_POST = 0;
    public final static int LANDMARK_MY_GOAL_RIGHT_POST = 1;
    public final static int LANDMARK_OPP_GOAL_LEFT_POST = 2;
    public final static int LANDMARK_OPP_GOAL_RIGHT_POST = 3;
    public final static int LANDMARK_LEFT_BEACON = 4;
    public final static int LANDMARK_RIGHT_BEACON = 5;
    public final static int LANDMARK_BALL = 6;
    public final static int LANDMARK_MY_CORNER_LEFT_L = 7;
    public final static int LANDMARK_MY_CORNER_RIGHT_L = 8;
    public final static int LANDMARK_MY_GOAL_LEFT_T = 9;
    public final static int LANDMARK_MY_GOAL_RIGHT_T = 10;
    public final static int LANDMARK_MY_GOAL_LEFT_L = 11;
    public final static int LANDMARK_MY_GOAL_RIGHT_L = 12;
    public final static int LANDMARK_CENTER_LEFT_T = 13;
    public final static int LANDMARK_CENTER_RIGHT_T = 14;
    public final static int LANDMARK_OPP_CORNER_LEFT_L = 15;
    public final static int LANDMARK_OPP_CORNER_RIGHT_L = 16;
    public final static int LANDMARK_OPP_GOAL_LEFT_T = 17;
    public final static int LANDMARK_OPP_GOAL_RIGHT_T = 18;
    public final static int LANDMARK_OPP_GOAL_LEFT_L = 19;
    public final static int LANDMARK_OPP_GOAL_RIGHT_L = 20;

    // Landmark Tcp Message Format
    public final static int NUM_LANDMARK_VALUES = 5;
    public final static int LANDMARK_ID_INDEX = 0;
    public final static int LANDMARK_X_INDEX = 1;
    public final static int LANDMARK_Y_INDEX = 2;
    public final static int LANDMARK_DIST_INDEX = 3;
    public final static int LANDMARK_BEARING_INDEX = 4;

    // Button Strings
    public final static String PROGRAM_LABEL_STRING = "Options:";
    public final static String PROGRAM_SPECIFIC_LABEL_STRING="Option Specific:";
    public final static String SIMULATION_LABEL_STRING = "Simulation:";
    public final static String UDP_LABEL_STRING = "UDP Viewer:";
    public final static String CONNECTION_LABEL_STRING = "Connection:";
    public final static String FPS_LABEL_STRING = "FPS Playback Speed:";
    public final static String SWITCH_FIELDS_STRING = "Switch Fields";
    public final static String SWITCH_COLORS_STRING = "Switch Colors";
    public final static String SWITCH_TEAMS_STRING = "Switch Teams";
    public final static String CLEAR_FIELD_STRING = "Clear Field";
    public final static String RELOAD_LOG_STRING = "Reload Log";
    public final static String QUIT_LOG_STRING = "Quit Log";
    public final static String DISCONNECT_STRING = "Disconnect";
    public final static String LIMIT_TEAM_STRING = "Restrict to Team:";
    public final static String DRAW_EST_STRING = "Draw Estimates";
	public final static String START_TCP_STRING = "Start TCP Stream";
	public final static String STOP_TCP_STRING = "Stop TCP Stream";

    // Button Action Commands
    public static final String SWITCH_FIELDS_ACTION = "switchfields";
    public static final String SWITCH_COLORS_ACTION = "switchcolors";
    public static final String SWITCH_TEAMS_ACTION = "switchteams";
    public static final String DRAW_EST_ACTION = "drawest";
    public static final String RELOAD_MCL_LOG_ACTION = "reloadmcllog";
    public static final String QUIT_MCL_LOG_ACTION = "quitekflog";
    public static final String RELOAD_EKF_LOG_ACTION = "reloadekflog";
    public static final String QUIT_EKF_LOG_ACTION = "quitekflog";
    public static final String CONNECT_ACTION = "connectme";
    public static final String DISCONNECT_ACTION = "disconnectme";
    public static final String LIMIT_TEAM_ACTION = "limitteam";
    public static final String CLEAR_FIELD_ACTION = "clearfield";
	public static final String STOP_TCP_ACTION = "stoptcpstream";
	public static final String START_TCP_ACTION = "starttcpstream";

    // Menu Strings
    public static final String VIEW_ROBOT_EKF_STRING = "View Robot EKF";
    public static final String VIEW_ROBOT_MCL_STRING = "View Robot Particles";
    public static final String VIEW_ROBOT_LOG_STRING = "View Robot Log";
    public static final String VIEW_EKF_LOG_STRING = "View EKF Log";
    public static final String VIEW_MCL_LOG_STRING = "View MCL Log";
    public static final String VIEW_UDP_PACKETS_STRING = "View UDP Packets";
    public static final String VIEW_TCP_STRING = "View TCP Stream";

    // Menu Action Commands
    public static final String VIEW_ROBOT_EKF_ACTION = "view robot ekfs";
    public static final String VIEW_ROBOT_MCL_ACTION = "view robot mcl";
    public static final String VIEW_ROBOT_LOG_ACTION = "viewrobotlog";
    public static final String VIEW_EKF_LOG_ACTION = "viewekflog";
    public static final String VIEW_MCL_LOG_ACTION = "viewmcllog";
    public static final String VIEW_UDP_PACKETS_ACTION = "viewudpaction";
	public static final String VIEW_TCP_ACTION = "viewtcpaction";
    public static final String CIRCLE_LINE_GOALIE_ACTION = "circlelinegoalie";
    public static final String GO_TO_XY_NORMAL_ACTION = "gotoxy";
    public static final String GO_TO_XY_LOOK_AT_POINT_ACTION =
        "gotoxy lookatpoint";

    // Values for the grid bag layout -- affects proportions of field and button
    // area
    final static int FIELD_WEIGHT = 10;
    final static int BUTTON_AREA_WEIGHT = 1;
    final static int BUTTON_AREA_WIDTH = 75;

    // Connect thread
    private Thread connect_thread;
    private JDialog connect_dialog;

    // Viewing robot EKF thread
    private Thread viewRobotEKFThread;

    // simulation getters
    private double thinks_looking_at_x, thinks_looking_at_y;
    private double real_x,real_y,real_heading;
    private double real_looking_at_x, real_looking_at_y;

    // for streamed vision
    private double received_pb_dist, received_pb_heading;
    private double received_bp_dist, received_bp_heading;
    private double received_yg_dist, received_yg_heading;
    private double received_bg_dist, received_bg_heading;

    // for streamed odometry
    private double x_change, y_change, a_change;

    // controls total playback_fps
    private int playback_fps;

    // for the connection counter we need to remember how many ticks have passed
    // since we started waiting for a successful robot connection
    private Timer waitToConnect;
    private int numWaitTimerTicks;

    // button declarations
    private JPanel button_area;
    private JLabel program_label;
    private JLabel program_specific_label;
    private JLabel connection_label;
    private JLabel fps_label;
    private JButton button_switch_fields;
    private JButton button_view_robot_ekf;
    private JButton button_view_robot_mcl;
    private JButton button_view_udp_packets;
	private JButton button_view_tcp;
    private JButton button_view_robot_log;
    private JButton button_view_ekf_log;
    private JButton button_view_mcl_log;
    private JButton button_one;
    private JButton button_two;
    private JButton button_three;
    private JTextField field_one;
    private JButton connect_button;
    private JButton draw_real_button;
    private JButton draw_est_button;
    private JSlider fps_slide;

    // for key listener
    private boolean key_pressed;

    private int mode;

    public WorldController(TOOL t, RobotViewModule robot_mod)
    {
        this.t = t;
        labField = new LabField2009();
        naoField = new NaoField2009();

        the_field = naoField;
        viewer = new WorldControllerViewer(the_field);
        painter = viewer.getPainter();

        debugViewer = new DebugViewer(this, (int)(the_field.getPreferredSize().
                                                  getWidth() +
                                                  BUTTON_AREA_WIDTH + 150), 0);
        log = new LogHandler(this, painter, debugViewer);
        udp_server = new UDPServer();
        udp_server.addRobotListener(painter);

		robotModule = robot_mod;

		tcp_handler = new TCPStreamHandler(robotModule, debugViewer, painter);

        setSize((int) (the_field.getPreferredSize().getWidth() +
                       BUTTON_AREA_WIDTH),
                (int) (the_field.getPreferredSize().getHeight()));

        // The GUI will use a GridBag layout to manipulate the visible objects
        GridBagLayout gridbag = new GridBagLayout();
        GridBagConstraints c = new GridBagConstraints();
        setLayout(gridbag);

        // Field will fill all the space alotted
        c.fill = GridBagConstraints.BOTH;
        // And take up all vertical space
        c.gridheight = GridBagConstraints.REMAINDER;
        // Field will be weighted in the horizontal greater than button area
        c.weightx = FIELD_WEIGHT;
        c.weighty = FIELD_WEIGHT;

        // Set the constraints for the Field and add it to the frame
        gridbag.setConstraints(viewer,c);
        add(viewer);

        // Create a new panel to hold the buttons, dubbed the buttonArea
        button_area = new JPanel();
        button_area.setLayout(new GridLayout(20,1));
        // Button area will be weighted less than the Field
        //c.weightx = BUTTON_AREA_WEIGHT;
        //c.weighty = BUTTON_AREA_WEIGHT;

        c.weightx = GridBagConstraints.REMAINDER;
        c.weighty = GridBagConstraints.REMAINDER;

        initButtons();
        // Set the constraints for the button area and add it to the frame
        add(button_area);
        gridbag.setConstraints(button_area,c);

        playback_fps = ROBOT_FPS;

        setFocusable(true);
        requestFocusInWindow();

        addKeyListener(this);

        setVisible(true);

        // Initialize mode
        mode = DO_NOTHING;

    } // end constructor

    /**
     * Method switches between Lab and Nao fields dimensions and landmarks
     */
    public void switchFields()
    {
        if (the_field == labField) {
            the_field = naoField;
        }
        else if (the_field == naoField) {
            the_field = labField;
        }
        // Takes care of sending info to the painter as well
        viewer.setField(the_field);
        debugViewer.swapField();
    }

    /**
     * Method required by ActionListener to deal with the various actions
     * invoked by users
     *
     * @param ActionEvent e The action event associated with the current user
     * action
     */
    public void actionPerformed(ActionEvent e)
    {
        String cmd = e.getActionCommand();

        // BUTTON ACTIONS
        if (cmd.equals(SWITCH_FIELDS_ACTION)) {
            switchFields();
        } else if (cmd.equals(CLEAR_FIELD_ACTION)) {
            painter.clearSimulationHistory();
        } else if (cmd.equals(QUIT_MCL_LOG_ACTION)) {
            log.quitMCLLog();
            startDoNothing();
        } else if (cmd.equals(RELOAD_MCL_LOG_ACTION)) {
            reloadMCLLog();
        } else if (cmd.equals(VIEW_MCL_LOG_ACTION)) {
            startMCLLog();
        } else if (cmd.equals(QUIT_EKF_LOG_ACTION)) {
            log.quitMCLLog();
            startDoNothing();
        } else if (cmd.equals(RELOAD_EKF_LOG_ACTION)) {
            reloadEKFLog();
        } else if (cmd.equals(VIEW_EKF_LOG_ACTION)) {
            startEKFLog();
        } else if (cmd.equals(VIEW_UDP_PACKETS_ACTION)) {
            startRobotUDP();
		} else if (cmd.equals(VIEW_TCP_ACTION)) {
			viewTCPStream();
		} else if (cmd.equals(START_TCP_ACTION)) {
			startTCPStream();
		} else if (cmd.equals(STOP_TCP_ACTION)) {
			stopTCPStream();
        } else if (cmd.equals(DISCONNECT_ACTION)) {
            udp_server.setReceiving(false);
            startDoNothing();
        } else if (cmd.equals(LIMIT_TEAM_ACTION)) {
            try {
                udp_server.limitToTeam(Integer.parseInt(field_one.getText()));
                painter.clearSimulationHistory();
            } catch (NumberFormatException asdf) {
                System.out.println("Excepted an integer");
            }
        }

        // keeps keyboard focus
        setFocusable(true);
        requestFocusInWindow();
    }

    ///###### JAVA FRAME METHODS %%%%%%%/

    public void initButtons()
    {
        /* BUTTON AREA */

        // program label
        program_label = new JLabel(PROGRAM_LABEL_STRING, JLabel.CENTER);
        program_label.setAlignmentX(Component.CENTER_ALIGNMENT);
        button_area.add(program_label);


        button_switch_fields = new JButton(SWITCH_FIELDS_STRING);
        button_switch_fields.setActionCommand(SWITCH_FIELDS_ACTION);
        button_switch_fields.addActionListener(this);
        button_area.add(button_switch_fields);

        // setup udp packets button
        button_view_udp_packets = new JButton(VIEW_UDP_PACKETS_STRING);
        button_view_udp_packets.setActionCommand(VIEW_UDP_PACKETS_ACTION);
        button_view_udp_packets.addActionListener(this);
        button_area.add(button_view_udp_packets);

		// setup tcp stream button
		button_view_tcp = new JButton(VIEW_TCP_STRING);
		button_view_tcp.setActionCommand(VIEW_TCP_ACTION);
		button_view_tcp.addActionListener(this);
		button_area.add(button_view_tcp);

        // setup MCL log button
        button_view_mcl_log = new JButton(VIEW_MCL_LOG_STRING);
        button_view_mcl_log.setActionCommand(VIEW_MCL_LOG_ACTION);
        button_view_mcl_log.addActionListener(this);
        button_area.add(button_view_mcl_log);

        // setup EKF log button
        button_view_ekf_log = new JButton(VIEW_EKF_LOG_STRING);
        button_view_ekf_log.setActionCommand(VIEW_EKF_LOG_ACTION);
        button_view_ekf_log.addActionListener(this);
        button_area.add(button_view_ekf_log);

        // program specific label
        program_specific_label = new JLabel(PROGRAM_SPECIFIC_LABEL_STRING,
					    JLabel.CENTER);
        program_specific_label.setAlignmentX(Component.CENTER_ALIGNMENT);
        button_area.add(program_specific_label);

        // Program specific buttons
        // Nulled out to start with
        button_one = new JButton(DISCONNECT_STRING);
        button_one.addActionListener(this);
        button_area.add(button_one);
        button_one.setVisible(false);
        button_two = new JButton(DISCONNECT_STRING);
        button_two.addActionListener(this);
        button_area.add(button_two);
        button_two.setVisible(false);
        button_three = new JButton(DISCONNECT_STRING);
        button_three.addActionListener(this);
        button_area.add(button_three);
        button_three.setVisible(false);
        field_one = new JTextField();
        field_one.addActionListener(this);
        button_area.add(field_one);
        field_one.setVisible(false);

        // fps playback
        fps_label = new JLabel(FPS_LABEL_STRING, JLabel.CENTER);
        fps_label.setAlignmentX(Component.CENTER_ALIGNMENT);
        fps_label.setVisible(false);
        button_area.add(fps_label);
        fps_slide = new JSlider(JSlider.HORIZONTAL, FPS_SLIDE_MIN,
                                FPS_SLIDE_MAX,
                                FPS_SLIDE_INIT);
        fps_slide.addChangeListener(this);
        button_area.add(fps_slide);
        fps_slide.setVisible(false);

    }

    public void nothingButtons()
    {
        button_one.setVisible(false);
        button_two.setVisible(false);
        button_three.setVisible(false);
        field_one.setVisible(false);
        fps_label.setVisible(false);
        fps_slide.setVisible(false);
    }

    public void mclLogButtons()
    {
        button_one.setText(DRAW_EST_STRING);
        button_one.setActionCommand(DRAW_EST_ACTION);
        button_one.setVisible(true);
        button_two.setText(RELOAD_LOG_STRING);
        button_two.setActionCommand(RELOAD_MCL_LOG_ACTION);
        button_two.setVisible(true);
        button_three.setText(QUIT_LOG_STRING);
        button_three.setActionCommand(QUIT_MCL_LOG_ACTION);
        button_three.setVisible(true);
        fps_label.setVisible(true);
        fps_slide.setVisible(true);
    }

    public void ekfLogButtons()
    {
        button_one.setText(DRAW_EST_STRING);
        button_one.setActionCommand(DRAW_EST_ACTION);
        button_one.setVisible(true);
        button_two.setText(RELOAD_LOG_STRING);
        button_two.setActionCommand(RELOAD_EKF_LOG_ACTION);
        button_two.setVisible(true);
        button_three.setText(QUIT_LOG_STRING);
        button_three.setActionCommand(QUIT_EKF_LOG_ACTION);
        button_three.setVisible(true);
        fps_label.setVisible(true);
        fps_slide.setVisible(true);
    }

    public void udpButtons()
    {
        button_one.setText(DISCONNECT_STRING);
        button_one.setActionCommand(DISCONNECT_ACTION);
        button_one.setVisible(true);
        button_two.setText(CLEAR_FIELD_STRING);
        button_two.setActionCommand(CLEAR_FIELD_ACTION);
        button_two.setVisible(true);
        button_three.setText(LIMIT_TEAM_STRING);
        button_three.setActionCommand(LIMIT_TEAM_ACTION);
        button_three.setVisible(true);
        field_one.setVisible(true);
    }

	public void tcpButtons()
	{
		// Set buttons appropriately for whether
		// TCP Streaming is active or not
		if (tcp_handler.isReceiving()) {
			button_one.setText(STOP_TCP_STRING);
			button_one.setActionCommand(STOP_TCP_ACTION);
			button_one.setVisible(true);
			debugViewer.setVisible(true);
		} else {
			button_one.setText(START_TCP_STRING);
			button_one.setActionCommand(START_TCP_ACTION);
			button_one.setVisible(true);
			debugViewer.setVisible(false);
		}
	}

	// Sets up the streaming object info from robot
	// public void startRobotObjectStream()
	// {

	// }

    // master method for getting robot udp working
    public void startRobotUDP()
    {
        nothingButtons();
        udpButtons();
        //painter.prepareForUDP();
        udp_server.setReceiving(true);
        mode = VIEW_UDP_PACKETS;
        painter.setPositionsToDraw(1);
    }

	public void viewTCPStream()
	{
		nothingButtons();
		tcpButtons();
		mode = VIEW_TCP_STREAM;
	}

	public void startTCPStream()
	{
		tcp_handler.setReceiving(true);
		tcpButtons();
	}

	public void stopTCPStream()
	{
		tcp_handler.setReceiving(false);
		painter.clearSimulationHistory();
		tcpButtons();
	}

    public void startDoNothing()
    {
        nothingButtons(); // clears buttons
        painter.clearSimulationHistory(); // clears field display
        mode = DO_NOTHING;
    }

    public String openDialog(String title, String dir, int mode)
    {
        FileDialog fd = new FileDialog(new Frame(),title,mode);
        fd.setDirectory(dir);
        fd.setVisible(true);
        String fullpath = fd.getDirectory() + fd.getFile();
        t.wcLastDirectory = fd.getDirectory();
        fd.dispose();
        if (fd.getFile() == null) return null;
        return fullpath;
    }

    public void delayFor(long fps)
    {
        // slows down updates to a DRAWING_FPS.
        try {
            Thread.sleep((long)(1000./fps));
        }catch (InterruptedException e) {
        }
    }

    // returns millis from start up
    public long getTime() { return new Date().getTime(); }


    public boolean checkConnecting() {
        boolean result = isConnecting();
        if (result)
            JOptionPane.showMessageDialog(this,"The previous connection " +
                    "attempt's Socket has not closed yet");
        return result;
    }

    public boolean isConnecting() {
        if (connect_thread!=null && connect_thread.isAlive())
            return true;
        return false;
    }

    public void keyPressed(KeyEvent k)
    {
        int key = k.getKeyCode();
    }

    /** Listen to the slider. */
    public void stateChanged(ChangeEvent e)
    {
        JSlider source = (JSlider)e.getSource();
        if (!source.getValueIsAdjusting() && source == fps_slide) {
            int value = (int)source.getValue();
            System.out.println("fps set to: " + value);
            setPlaybackFps(value);
        }
    }

    //keyboard methods
    public void keyTyped(KeyEvent e) {}
    public void keyReleased(KeyEvent e)
    {
    }

    public void setPlaybackFps(int _fps)
    {
        playback_fps = _fps;
        // maybe the redraw timer is currently running, so we have to
        // manually inform it that the fps has changed
        log.setPlaybackFps(_fps);
    }
    public int getPlaybackFps() { return playback_fps; }
    public void setMode(int _mode) { mode = _mode; }
    public int getMode() { return mode; }

    /**
     * Method to test the output system of the mcl...
     */
    public void startMCLLog()
    {
        mode = VIEW_MCL_LOG;
        if(!log.initLog()) {
            mode = DO_NOTHING;
            nothingButtons();
            return;
        }
        nothingButtons();
        mclLogButtons();
    }
    /**
     * Method to test the output system of the mcl...
     */
    public void startEKFLog()
    {
        mode = VIEW_EKF_LOG;
        if(!log.initLog()) {
            mode = DO_NOTHING;
            nothingButtons();
            return;
        }
        nothingButtons();
        ekfLogButtons();
    }

    /**
     * Method to test the output system of the mcl...
     */
    public void reloadMCLLog()
    {
        if(!log.reloadLog()) {
            mode = DO_NOTHING;
            nothingButtons();
            return;
        }
    }
    /**
     * Method to test the output system of the mcl...
     */
    public void reloadEKFLog()
    {
        if(!log.reloadLog()) {
            mode = DO_NOTHING;
            nothingButtons();
            return;
        }
    }

}




