using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;


namespace Gravity
{
    partial class Program : MyGridProgram
    {

        // Automatic Gravity Alignment Script    
        //    
        // Version: 1.0
        //
        // This is an independant addition for HASCS
        //
        // Link for HASCS: https://steamcommunity.com/sharedfiles/filedetails/?id=2921178770

        //=============================================================
        //Start copying the code into the program block after this line:
        //=============================================================

        // === Most useful variables ===
        const string gyroExcludeName = "[Exclude]";
        const string statusScreenName = "[Alignment]"; //(Optional) Name of status screen
        const double angleTolerance = 0.01; //How many degrees the code will allow before it overrides user control


        bool shouldAlign = true; //If the script should attempt to stabalize by default
        bool referenceOnSameGridAsProgram = true; //if true, only searches for reference blocks on
                                                  //the same grid as the program block (should help with docking small vessels)

        //PID Constants
        const double proportionalConstant = 2;
        const double derivativeConstant = .5;

        const double yawSpeedModifier = 1;

        // === Don't touch after this ===

        const double updatesPerSecond = 10;
        const double timeLimit = 1 / updatesPerSecond;
        double angleRoll = 0;
        double anglePitch = 0;
        double timeElapsed = 0;
        bool canTolerate = true;
        string stableStatus = ">> Disabled <<";
        string gravityMagnitudeString;
        string overrideStatus;

        List<IMyGyro> gyros = new List<IMyGyro>();
        List<IMyShipController> shipControllers = new List<IMyShipController>();

        PID pitchPID;
        PID rollPID;

        Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1;

            pitchPID = new PID(proportionalConstant, 0, derivativeConstant, -10, 10, timeLimit);
            rollPID = new PID(proportionalConstant, 0, derivativeConstant, -10, 10, timeLimit);
        }

        void Main(string arg, UpdateType updateSource)
        {
            

            timeElapsed += Runtime.TimeSinceLastRun.TotalSeconds;

            switch (arg.ToLower())
            {
                case "toggle":
                    if (!shouldAlign)
                    {
                        shouldAlign = true;
                        stableStatus = "<< Active >>";
                    }
                    else
                    {
                        shouldAlign = false;
                        stableStatus = ">> Disabled <<";
                    }
                    break;

                case "on":
                    shouldAlign = true;
                    stableStatus = "<< Active >>";
                    break;

                case "off":
                    shouldAlign = false;
                    stableStatus = ">> Disabled <<";
                    break;

                default:
                    break;
            }

            if (timeElapsed >= timeLimit)
            {
                AlignWithGravity();
                StatusScreens();
                timeElapsed = 0;
                Echo("Stabilizers on?: " + shouldAlign.ToString());
            }
        }

        bool ShouldFetch(IMyTerminalBlock block)
        {
            if (block is IMyShipController)
            {
                if (referenceOnSameGridAsProgram)
                {
                    return block.CubeGrid == Me.CubeGrid;
                }
                else
                {
                    return true;
                }
            }
            else
            {
                return false;
            }
        }

        IMyShipController GetControlledShipController(List<IMyShipController> controllers)
        {
            if (controllers.Count == 0)
                return null;

            foreach (IMyShipController thisController in controllers)
            {
                if (thisController.IsUnderControl && thisController.CanControlShip)
                    return thisController;
            }

            return controllers[0];
        }

        void AlignWithGravity()
        {
            //Find our refrence and comparision blocks    
            GridTerminalSystem.GetBlocksOfType(shipControllers, ShouldFetch);

            //Check for any cases that would lead to code failure
            if (shipControllers.Count == 0)
            {
                Echo($"ERROR: No ship controller was found");
                return;
            }

            //Assign our reference block
            IMyShipController referenceBlock = GetControlledShipController(shipControllers);

            //Populate gyro list
            gyros.Clear();
            GridTerminalSystem.GetBlocksOfType(gyros, block => block.CubeGrid == referenceBlock.CubeGrid && !block.CustomName.Contains(gyroExcludeName));

            if (gyros.Count == 0)
            {
                Echo("ERROR: No gyros found on ship");
                return;
            }

            //Get gravity vector    
            var referenceOrigin = referenceBlock.GetPosition();
            var gravityVec = referenceBlock.GetNaturalGravity();
            var gravityVecLength = gravityVec.Length();
            gravityMagnitudeString = Math.Round(gravityVecLength, 2).ToString() + " m/s²";
            if (gravityVec.LengthSquared() == 0)
            {
                gravityMagnitudeString = "No Gravity";

                foreach (IMyGyro thisGyro in gyros)
                {
                    thisGyro.SetValue("Override", false);
                }
                overrideStatus = "";
                stableStatus = ">> Disabled <<";

                shouldAlign = false;

                angleRoll = 0; angleRoll = 0;
                return;
            }

            //Direction vectors of the reference block     
            var referenceForward = referenceBlock.WorldMatrix.Forward;
            var referenceLeft = referenceBlock.WorldMatrix.Left;
            var referenceUp = referenceBlock.WorldMatrix.Up;

            //Get Roll and Pitch Angles 
            anglePitch = Math.Acos(MathHelper.Clamp(gravityVec.Dot(referenceForward) / gravityVecLength, -1, 1)) - Math.PI / 2;

            Vector3D planetRelativeLeftVec = referenceForward.Cross(gravityVec);
            angleRoll = VectorAngleBetween(referenceLeft, planetRelativeLeftVec);
            angleRoll *= VectorCompareDirection(VectorProjection(referenceLeft, gravityVec), gravityVec); //ccw is positive 

            anglePitch *= -1; angleRoll *= -1;

            Echo("pitch angle:" + Math.Round((anglePitch / Math.PI * 180), 2).ToString() + " deg");
            Echo("roll angle:" + Math.Round((angleRoll / Math.PI * 180), 2).ToString() + " deg");


            //Get Raw Deviation angle    
            double rawDevAngle = Math.Acos(MathHelper.Clamp(gravityVec.Dot(referenceForward) / gravityVec.Length() * 180 / Math.PI, -1, 1));

            //Angle controller    
            double rollSpeed = rollPID.Control(angleRoll);
            double pitchSpeed = pitchPID.Control(anglePitch);                                                                                                                                                            //w.H]i\p

            var inputVec = referenceBlock.MoveIndicator;


            //Check if we are inside our tolerances  
            canTolerate = true;

            if (Math.Abs(anglePitch * 180 / Math.PI) > angleTolerance)
            {
                canTolerate = false;
            }

            if (Math.Abs(angleRoll * 180 / Math.PI) > angleTolerance)
            {
                canTolerate = false;
            }

            //Set appropriate gyro override  
            if (shouldAlign && !canTolerate)
            {
                //do gyros
                ApplyGyroOverride(pitchSpeed, inputVec.X * yawSpeedModifier, -rollSpeed, gyros, referenceBlock);

                overrideStatus = "\n\nSAFETY OVERRIDE ACTIVE";
            }
            else
            {
                foreach (IMyGyro thisGyro in gyros)
                {
                    thisGyro.SetValue("Override", false);
                }
                overrideStatus = "";
            }
        }

        void StatusScreens()
        {
            //Get the parts of our string  
            double roll_deg = angleRoll / Math.PI * 180;
            double pitch_deg = -anglePitch / Math.PI * 180;
            string rollStatusString = AngleStatus(roll_deg);
            string pitchStatusString = AngleStatus(pitch_deg);

            //Construct our final string  
            string statusScreenMessage = 
                "Natural Gravity: " + gravityMagnitudeString
                + "\nStabilizer: " + stableStatus
                + "\n\nRoll Angle: " + Math.Round(roll_deg, 2).ToString() + " degrees\n" + rollStatusString
                + "\n\nPitch Angle: " + Math.Round(pitch_deg, 2).ToString() + " degrees\n" + pitchStatusString
                + overrideStatus;


            //Write to screens  
            var screens = new List<IMyTerminalBlock>();
            GridTerminalSystem.SearchBlocksOfName(statusScreenName, screens, block => block is IMyTextPanel);

            if (screens.Count == 0)
                return;

            foreach (IMyTextPanel thisScreen in screens)
            {
                thisScreen.ContentType = ContentType.TEXT_AND_IMAGE;
                thisScreen.WriteText(statusScreenMessage);
            }
        }

        const string align_15 = " [-15](-)-------0----------[+15]";
        const string align_14 = " [-15]-(-)------0----------[+15]";
        const string align_12 = " [-15]--(-)-----0----------[+15]";
        const string align_10 = " [-15]---(-)----0----------[+15]";
        const string align_8 = " [-15]----(-)---0----------[+15]";
        const string align_6 = " [-15]-----(-)--0----------[+15]";
        const string align_4 = " [-15]------(-)-0----------[+15]";
        const string align_2 = " [-15]-------(-)0----------[+15]";
        const string align0 = " [-15]---------(0)---------[+15]";
        const string align2 = " [-15]----------0(-)-------[+15]";
        const string align4 = " [-15]----------0-(-)------[+15]";
        const string align6 = " [-15]----------0--(-)-----[+15]";
        const string align8 = " [-15]----------0---(-)----[+15]";
        const string align10 = " [-15]----------0----(-)---[+15]";
        const string align12 = " [-15]----------0-----(-)--[+15]";
        const string align14 = " [-15]----------0------(-)-[+15]";
        const string align15 = " [-15]----------0-------(-)[+15]";

        string AngleStatus(double angle)
        {

            //Switch is not used because of an error with relational templates

            if (angle > 15)
                return align15;
            else if (angle > 14)
                return align14;
            else if (angle > 12)
                return align12;
            else if (angle > 10)
                return align10;
            else if (angle > 8)
                return align8;
            else if (angle > 6)
                return align6;
            else if (angle > 4)
                return align4;
            else if (angle > 2)
                return align2;
            else if (angle > -2)
                return align0;
            else if (angle > -4)
                return align_2;
            else if (angle > -6)
                return align_4;
            else if (angle > -8)
                return align_6;
            else if (angle > -10)
                return align_8;
            else if (angle > -12)
                return align_10;
            else if (angle > -14)
                return align_12;
            else if (angle > -15)
                return align_14;
            else
                return align_15;


        }

        Vector3D VectorProjection(Vector3D a, Vector3D b) //proj a on b    
        {
            Vector3D projection = a.Dot(b) / b.LengthSquared() * b;
            return projection;
        }

        int VectorCompareDirection(Vector3D a, Vector3D b) //returns -1 if vectors return negative dot product 
        {
            double check = a.Dot(b);
            if (check < 0)
                return -1;
            else
                return 1;
        }

        double VectorAngleBetween(Vector3D a, Vector3D b) //returns radians 
        {
            if (a.LengthSquared() == 0 || b.LengthSquared() == 0)
                return 0;
            else
                return Math.Acos(MathHelper.Clamp(a.Dot(b) / a.Length() / b.Length(), -1, 1));
        }

        //Applying override for each gyro depending on it's position related to reference
        void ApplyGyroOverride(double pitch_speed, double yaw_speed, double roll_speed, List<IMyGyro> gyro_list, IMyTerminalBlock reference)
        {
            var rotationVec = new Vector3D(-pitch_speed, yaw_speed, roll_speed); //because keen does some weird stuff with signs 
            var shipMatrix = reference.WorldMatrix;
            var relativeRotationVec = Vector3D.TransformNormal(rotationVec, shipMatrix);

            foreach (var thisGyro in gyro_list)
            {
                var gyroMatrix = thisGyro.WorldMatrix;
                var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(gyroMatrix));

                thisGyro.Pitch = (float)transformedRotationVec.X;
                thisGyro.Yaw = (float)transformedRotationVec.Y;
                thisGyro.Roll = (float)transformedRotationVec.Z;
                thisGyro.GyroOverride = true;
            }
        }

        //PID-regulator
        public class PID
        {
            double _kP = 0;
            double _kI = 0;
            double _kD = 0;
            double _integralDecayRatio = 0;
            double _lowerBound = 0;
            double _upperBound = 0;
            double _timeStep = 0;
            double _inverseTimeStep = 0;
            double _errorSum = 0;
            double _lastError = 0;
            bool _firstRun = true;
            bool _integralDecay = false;
            public double Value { get; private set; }

            public PID(double kP, double kI, double kD, double lowerBound, double upperBound, double timeStep)
            {
                _kP = kP;
                _kI = kI;
                _kD = kD;
                _lowerBound = lowerBound;
                _upperBound = upperBound;
                _timeStep = timeStep;
                _inverseTimeStep = 1 / _timeStep;
                _integralDecay = false;
            }

            public PID(double kP, double kI, double kD, double integralDecayRatio, double timeStep)
            {
                _kP = kP;
                _kI = kI;
                _kD = kD;
                _timeStep = timeStep;
                _inverseTimeStep = 1 / _timeStep;
                _integralDecayRatio = integralDecayRatio;
                _integralDecay = true;
            }

            public double Control(double error)
            {
                //Compute derivative term
                var errorDerivative = (error - _lastError) * _inverseTimeStep;

                if (_firstRun)
                {
                    errorDerivative = 0;
                    _firstRun = false;
                }

                //Compute integral term
                if (!_integralDecay)
                {
                    _errorSum += error * _timeStep;
                    //Clamp integral term
                    _errorSum = MathHelper.Clamp(_errorSum, _lowerBound, _upperBound);
                }
                else
                {
                    _errorSum = _errorSum * (1.0 - _integralDecayRatio) + error * _timeStep;
                }

                //Store this error as last error
                _lastError = error;

                //Construct output
                this.Value = _kP * error + _kI * _errorSum + _kD * errorDerivative;
                return this.Value;
            }

            public double Control(double error, double timeStep)
            {
                _timeStep = timeStep;
                _inverseTimeStep = 1 / _timeStep;
                return Control(error);
            }

            public void Reset()
            {
                _errorSum = 0;
                _lastError = 0;
                _firstRun = true;
            }
        }

        //============================================================
        //End copying the code into the program block before this line.
        //============================================================
    }
}