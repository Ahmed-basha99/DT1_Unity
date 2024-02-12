using System;
using UnityEngine;

/// <summary>
///     A simple free camera to be added to a Unity game object.
///     Keys:
///     wasd / arrows	- movement
///     q/e 			- down/up (local space)
///     r/f 			- up/down (world space)
///     pageup/pagedown	- up/down (world space)
///     hold shift		- enable fast movement mode
///     right mouse  	- enable free look
///     mouse			- free look / rotation
/// </summary>
public class FreeCam : MonoBehaviour
{


    public Transform target;
    public Vector3 offset = new Vector3(0, 0.4260001f, -1);



    void Update()
    {
         if (target != null)
        {
            // Update the position of the camera to follow the target with the specified offset
            transform.position = target.position + offset;
        }
    }

    
}
