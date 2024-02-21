using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;


public class cont : MonoBehaviour
{
    // Start is called before the first frame update
    private InputDevice targetDevice;
    void Start()
    {
        
        List<InputDevice> devices = new List<InputDevice>();
        InputDeviceCharacteristics rightControllerCharacteristics = InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller;

        InputDevices.GetDevicesWithCharacteristics(rightControllerCharacteristics, devices);

        foreach (var item in devices){
            Debug.Log(item.name  );
        }
        Debug.Log(devices.Count);
         if (devices.Count >0 ){

            targetDevice = devices[0];
            Debug.Log(targetDevice.name + " is target");
         }
    }

    // Update is called once per frame
    void Update()
    {
        targetDevice.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryButtonValue);

        if (primaryButtonValue){

            Debug.Log("Pressing Primary Button");
        }

        targetDevice.TryGetFeatureValue(CommonUsages.trigger, out float triggerValue);
        
        if (triggerValue> 0.1f){

            Debug.Log("Pressing trigger");
        }
            
    }
}
