package TOOL.Misc;
// This is a placeholder for the C++ estimate we receive out of pixEstimate.
// Everything is public for convenience. There no need for setters and getters
// ever

public class Estimate {
    public double dist;
    public double elevation;
    public double bearing;
    public double x;
    public double y;
    public double distance_variance;
    public double bearing_variance;

    public Estimate(double _dist, double _elevation, double _bearing,
                    double _x, double _y,
                    double _distance_variance, double _bearing_variance) {
        dist = _dist;
        elevation = _dist;
        bearing = _bearing;
        x = _x;
        y = _y;
        distance_variance = _distance_variance;
        bearing_variance = _bearing_variance;
    }
}