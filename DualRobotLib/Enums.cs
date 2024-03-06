using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DualRobotLib
{
    public enum Model
    {
        Null,
        CR7,
        CR15,
        Motor,
        LiftTable,
        LED
    }

    public enum MovementType
    {
        QuickCheck,
        StepRun
    }

    // either use RobotGuide Simulator or Real Fanuc Robot
    public enum Env
    {
        Real,
        Simulation
    }

    public enum SceneName
    {
        None,
        Scene1A,
        Scene1A_Sim,
        Scene1B,
        Scene1C,
        Scene2,
        Scene2_Sim,
        Scene3,
        Scene3_Sim,
        Scene4,
        Scene4_Sim
    }

    public enum Position
    {
        Home,
        InitPos_Scene1A,
        InitPos_Scene1B,
        InitPos_Scene1C,
        InitPos_Scene2,
        Home2,
        Ready_Scene1B,
        Ready_Scene2,
        Ready_Scene1A,
        Home3,
        InitPos_Scene3,
        Ready_Scene3
    }

    public struct ViaPoint
    {
        private double x, y, z, rx, ry, rz;
        private double vdi_angle;
        private double line_no;
        private double point_no;
        
    }

    public enum MovementStage
    {
        Null,
        One,
        Two,
        Three,
        Four
    }
}
