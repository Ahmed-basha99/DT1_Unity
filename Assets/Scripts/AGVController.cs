using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine.XR.Interaction.Toolkit;
using System;
using System.Collections.Generic;
namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS, XR_Control };

    public class AGVController : MonoBehaviour
    {
        public GameObject wheel1;
        public GameObject wheel2;
        public ControlMode mode = ControlMode.ROS;

        private ArticulationBody wA1;
        private ArticulationBody wA2;

        public float maxLinearSpeed = 2; //  m/s
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.033f; //meters
        public float trackWidth = 0.288f; // meters Distance between tyres
        public float forceLimit = 10;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;
        
        public XRController rightController; // Reference to the right XR controller
        public XRController leftController; // Reference to the left XR controller

        void Start()
        {
            var controllers = FindObjectsOfType<XRController>();
            rightController = controllers[0];
            leftController = controllers[1];
            Debug.Log("Number of XRControllers: " + controllers[1]);
            
            
//             Debug.Log( "hiiiiii");
//             var inputDevices = new List<UnityEngine.XR.InputDevice>();
// UnityEngine.XR.InputDevices.GetDevices(inputDevices);

// foreach (var device in inputDevices)
// {
//     Debug.Log(string.Format("Device found with name '{0}' and role '{1}'", device.name, device.role.ToString()));
// }
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            SetParameters(wA1);
            SetParameters(wA2);
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
        }

       void ReceiveROSCmd(TwistMsg cmdVel)
        {
            // Extract linear and angular values from TwistMsg
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;

            // // Add debug prints
            // Debug.Log("Received ROS Command:");
            // Debug.Log("Linear Velocity: " + rosLinear);
            // Debug.Log("Angular Velocity: " + rosAngular);
            // Debug.Log("Last Command Received Time: " + lastCmdReceived);
        }


        void FixedUpdate()
        {
            if (mode == ControlMode.Keyboard)
            {
                KeyBoardUpdate();
            }
            else if (mode == ControlMode.ROS)
            {
                ROSUpdate();
            }
            else if (mode == ControlMode.XR_Control) {
                // XRUpdate ();
            }     
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;

    
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

        // private void XRUpdate(){
        //       // Use the XR controller input for movement
        //     // float moveDirection = rightController.inputDevice;
        //     float inputSpeed;
        //     if (moveDirection > 0)
        //     {
        //         inputSpeed = maxLinearSpeed;
        //     }
        //     else if (moveDirection < 0)
        //     {
        //         inputSpeed = maxLinearSpeed * -1;
        //     }
        //     else
        //     {
        //         inputSpeed = 0;
        //     }

        //     // Use the XR controller input for rotation
        //     // float turnDirction = rightController.inputDevice.GetAxis("Primary2DAxis").y;
        //     float inputRotationSpeed;
        //     if (turnDirction > 0)
        //     {
        //         inputRotationSpeed = maxRotationalSpeed;
        //     }
        //     else if (turnDirction < 0)
        //     {
        //         inputRotationSpeed = maxRotationalSpeed * -1;
        //     }
        //     else
        //     {
        //         inputRotationSpeed = 0;
        //     }

        //     // Debug.Log("keyboard speed : " + inputSpeed + "\n");
        //     RobotInput(inputSpeed, inputRotationSpeed);
        // }
        private void KeyBoardUpdate()
        {
            
            float moveDirection = Input.GetAxis("Left_Right");
            float inputSpeed;
            float inputRotationSpeed;
            if (moveDirection > 0)
            {
                inputSpeed = maxLinearSpeed;
            }
            else if (moveDirection < 0)
            {
                inputSpeed = maxLinearSpeed * -1;
            }
            else
            {
                inputSpeed = 0;
            }
        
            float turnDirction = Input.GetAxis("Up_Down");
            if (turnDirction > 0)
            {
                inputRotationSpeed = maxRotationalSpeed;
            }
            else if (turnDirction < 0)
            {
                inputRotationSpeed = maxRotationalSpeed * -1;
            }
            else
            {
                inputRotationSpeed = 0;
            }
            //  Debug.Log("keyboard speed : " + inputSpeed + "\n");
            RobotInput(inputSpeed, inputRotationSpeed);
        }


        private void ROSUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                // ROS command timeout, reset values
                rosLinear = 0f;
                rosAngular = 0f;

                // Add debug prints
                Debug.Log("ROS Timeout: Resetting values to zero.");
                Debug.Log("lastCmdReceived: " + lastCmdReceived );
                Debug.Log("Time.time: " + Time.time );
                
            }

            // Call RobotInput with updated values
            // Debug.Log("Linear Velocity: " + rosLinear);
            // Debug.Log("Angular Velocity: " + rosAngular);
            // Debug.Log("ros speed : " + rosLinear + "\n");
            // float temp = (rosLinear > 0.2f) ?0.8f : ((rosLinear < 0.2f) ? -0.8f : 0f);
            // if (rosLinear>0.2) rosLinear = 0.8f ;
            // if (rosLinear<0.2) rosLinear = -0.8f ;
            // Debug.Log("TE speed : " + temp + "\n");


            RobotInput(3.8461f*rosLinear, -rosAngular);
        }


        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }
            float wheel1Rotation = (speed / wheelRadius);
            float wheel2Rotation = wheel1Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            }
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
            }
            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
        }
    }
}
