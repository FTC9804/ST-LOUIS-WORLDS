public class FTCRobot
{
    private int numMatches;
    private int numAutoMatches;
    private boolean autoClimbers;
    private int climberAttempts;
    private boolean autoMountainCapability;
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
    
    private boolean 2climbersAndBeacon;
    private int num2climbersAndBeacon
    
    private boolean 2climbersAndLow;
    private int num2climbersAndLow;
    private boolean 2climbersAndMid;
    private int num2climbersAndMid;
    private boolean 2climbersAndHigh;
    private int num2climbersAndHigh;
    
    private boolean beaconAndLow;
    private int numBeaconAndLow;
    private boolean beaconAndMid;
    private int numBeaconAndMid;
    private boolean beaconAndHigh;
    private int numBeaconAndHigh;
    
    private boolean 2climbersAndBeaconAndLow;
    private int num2climbersAndBeaconAndLow;
    private boolean 2climbersAndBeaconAndMid;
    private int num2climbersAndBeaconAndMid;
    private boolean 2climbersAndBeaconAndHigh;
    private int num2climbersAndBeaconAndHigh;
    
    
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
    
    public FTCRobot (int numMatches, int numAutoMatches, boolean autoClimbers, int climberAttempts, boolean autoMountainCapablility, int autoMountainattempts, boolean climberCapability,
    int numAutoClimbers, int startToBoxSpeed, boolean beacon, int numBeacons, boolean autoHigh, int autoNumHighs, boolean autoMid, int autoNumMids,
    boolean autoFullLows, int autoNumFullLows, boolean twoClimbersAndBeacon, int numTwoClimbersAndBeacon,   boolean lowGoalCapable,
    int [] numLowGoalsPerMatch, boolean midGoalCapable, int [] numMidGoalsPerMatch, boolean highGoalCapable, 
    int [] numHighGoalsPerMatch, boolean zipLineCapable,int zipLineAttempts, int [] numZipLinesPerMatch, String [] endMountainAttempts,
    boolean endHigh, int endNumHighs, boolean endMid, int endNumMids, boolean hang, int numHangs, boolean allClear, int numAllClears)
    {
        this.numMatches=numMatches;
        this.autoClimbers=autoClimbers;
        this.climberCapability=climberCapability;
        this.numAutoClimbers=numAutoClimbers;
        this.autoMountainCapability=autoMountainCapability
        this.startToBoxSpeed=startToBoxSpeed;
        this.beacon=beacon;
        this.numBeacons=numBeacons;
        this.autoHigh=autoHigh;
        this.autoNumHighs=autoNumHighs;
        this.autoMid=autoMid;
        this.autoNumMids=autoNumMids;
        this.autoFullLow=autoFullLow;
        this.autoNumFullLows=autoNumFullLows;
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
        double count=0;
        if (autoClimbers)
        {
            count+=20
            int autoClimberConsistency=(numAutoClimbers/(climberAttempts*2))
            count+=AutoClimberConsistency*135
        }
        
        if (beacon)
        {
            count+=16;
            int beaconConsistency=numBeacons/climberAttempts;
            count+=beaconConsistency*105;
        }
    
        
        if (autoMountainCapability)
        {
            count+= (count*.06)
            if (autoFullLow)
            {
                count+=10;
                int lowConsistency= autoFullLows/autoMountainAttempts;
                count+=lowConsistency*20
                count-=((autoMatches-autoMountainAttempts)/autoMatches)*lowConsistency*20);
                if (twoClimbersAndLow)
                {
                    int climbersAndLowConsistency= numClimbersAndLow/(autoMatches-beaconButNoClimbers);
                    count+=(5+climbersAndLowConsistency*100/16);
                }
                
                if (beaconAndLow)
                {
                    int beaconAndLowConsistency = numBeaconAndLow/(autoMatches-climbersButNoBeacon);
                    count+=(4+beaconAndLowConsistency*100/18);
                }
                
                if (twoClimbersAndBeaconAndLow)
                {
                    int twoClimbersAndBeaconAndLowConsistency= numClimbersAndBeaconAndLow/autoMatches;
                    count+= (10+twoClimbersAndBeaconAndLowConsistency*100/9);
                    count+=count*(.02*twoClimbersAndBeaconAndLowConsistency);
                }
            }
            
            if (autoMid)
            {
                count+=15
                int midConsistency= autoMids/autoMountainAttempts;
                count+=midConsistency*40;
                count-=((autoMatches-autoMountainAttempts)/autoMatches)*midConsistency*40);
                if (twoClimbersAndMid)
                {
                    int climbersAndMidConsistency= numClimbersAndMid/(autoMatches);
                    count+=(10+climbersAndLowConsistency*100/11);
                }
                
                if (beaconAndMid)
                {
                    int beaconAndMidConsistency = numBeaconAndMid/(autoMatches);
                    count+=(8+beaconAndLowConsistency*100/12.3);
                }
                
                if (twoClimbersAndBeaconAndMid)
                {
                    int twoClimbersAndBeaconAndMidConsistency= numClimbersAndBeaconAndMid/autoMatches;
                    count+= (15+twoClimbersAndBeaconAndLowConsistency*100/6.4);
                    count+=count*(.032*twoClimbersAndBeaconAndMidConsistency);
                }
            }
            
            if (autoHigh)
            {
                count+=25
                int highConsistency= autoHighs/autoMountainAttempts;
                count+=highConsistency*75;
                count-=((autoMatches-autoMountainAttempts)/autoMatches)*highConsistency*75);
                if (twoClimbersAndHigh)
                {
                    int climbersAndHighConsistency= numClimbersAndHigh/(autoMatches);
                    count+=(20+climbersAndLowConsistency*100/8);
                }
                
                if (beaconAndHigh)
                {
                    int beaconAndHighConsistency = numBeaconAndHigh/(autoMatches);
                    count+=(16+beaconAndLowConsistency*100/9.6);
                }
                
                if (twoClimbersAndBeaconAndHigh)
                {
                    int twoClimbersAndBeaconAndHighConsistency= numClimbersAndBeaconAndMid/autoMatches;
                    count+= (28+twoClimbersAndBeaconAndLowConsistency*100/5.3);
                    count+=count*(.045*twoClimbersAndBeaconAndHighConsistency);
                }
            }
            
            if (startToBoxSpeed>=20 && startToBoxSpeed<=25)
            {
                count+=5;
            }
            
            if (startToBoxSpeed>=15 && startToBoxSpeed<20)
            {
                count+=10
            }
            
            if (startToBoxSpeed>=10 && startToBoxSpeed<15)
            {
                count+=20;
            }
            
            if (startToBoxSpeed>=6 && startToBoxSpeed<10)
            {
                count+=27.5
            }
            
            if (startToBoxSpeed<6)
            {
                count+=35;
            }
            
        }
    }
