public class FTCRobot
{
    private int numMatches;
    private int numAutoMatches;
    private boolean autoClimbers;
    private int climberAttempts;
    private int autoMountainAttempts;
    private boolean climberCapability;
    private int numAutoClimbers;
    private int startToBoxSpeed;
    private boolean beacon;
    private int numBeacons;
    private boolean autoHigh;
    private int autoNumHighs;
    private boolean autoMid;
    private int autoNumMids;
    private boolean autoFullLow;
    private int autoNumFullLows;
    private boolean autoPartialLow;
    private int autoNumPartialLows;
    
    private boolean lowGoalCapable;
    private int [] numLowGoalsPerMatch;
    private boolean midGoalCapable;
    private int [] numMidGoalsPerMatch;
    private boolean highGoalCapable;
    private int [] numHighGoalsPerMatch;
    private boolean zipLineCapable;
    private int zipLineAttempts;
    private int [] numZipLinesPerMatch;
    
    private String [] endMountainAttempts;
    private boolean endHigh;
    private int endNumHighs;
    private boolean endMid;
    private int endNumMids;
    private boolean hang;
    private int numHangs;
    private boolean allClear;
    private int numAllClears;
    
    public FTCRobot (int numMatches, int numAutoMatches, boolean autoClimbers, int climberAttempts, int autoMountainattempts, boolean climberCapability,
    int numAutoClimbers, int startToBoxSpeed, boolean beacon, int numBeacons, boolean autoHigh, int autoNumHighs, boolean autoMid, int autoNumMids,
    boolean autoFullLows, int autoNumFullLows, boolean autoPartialLow, int autoNumPartialLows, boolean lowGoalCapable,
    int [] numLowGoalsPerMatch, boolean midGoalCapable, int [] numMidGoalsPerMatch, boolean highGoalCapable, 
    int [] numHighGoalsPerMatch, boolean zipLineCapable,int zipLineAttempts, int [] numZipLinesPerMatch, String [] endMountainAttempts,
    boolean endHigh, int endNumHighs, boolean endMid, int endNumMids, boolean hang, int numHangs, boolean allClear, int numAllClears)
    {
        this.numMatches=numMatches;
        this.autoClimbers=autoClimbers;
        this.climberCapability=climberCapability;
        this.numAutoClimbers=numAutoClimbers;
        this.startToBoxSpeed=startToBoxSpeed;
        this.beacon=beacon;
        this.numBeacons=numBeacons;
        this.autoHigh=autoHigh;
        this.autoNumHighs=autoNumHighs;
        this.autoMid=autoMid;
        this.autoNumMids=autoNumMids;
        this.autoFullLow=autoFullLow;
        this.autoNumFullLows=autoNumFullLows;
        this.autoPartialLow=autoPartialLow;
        this.autoNumPartialLows=autoNumPartialLows;
        this.lowGoalCapable=lowGoalCapable;
        this.numLowGoalsPerMatch=numLowGoalsPerMatch;
        this.midGoalCapable=midGoalCapable;
        this.numMidGoalsPerMatch=numMidGoalsPerMatch;
        this.highGoalCapable=highGoalCapable;
        this.numHighGoalsPerMatch=numHighGoalsPerMatch;
        this.zipLineCapable=zipLineCapable;
        this.numZipLinesPerMatch=numZipLinesPerMatch;
        this.endHigh=endHigh;
        this.endNumHighs=endNumHighs;
        this.endMid=endMid;
        this.endNumMids=endNumMids;
        this.hang=hang;
        this.numHangs=numHangs;
        this.allClear=allClear;
        this.numAllClears=numAllClears;
        
        
    }



    public double AutoPowerRank ()
    {
        count=0;
        if (autoClimbers)
        {
           count+=auto 
    
    
    
    
    
    
    
    
