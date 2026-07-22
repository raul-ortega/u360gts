## Adjust the OFFSET

While this step could be done after adjusting the PIDs, it is advisable to do it now to avoid accumulating errors during the configuration process.

The function of the OFFSET parameter is to shift the north by the necessary degrees to ensure that our antenna points correctly to the north. For example, this is necessary when the controller is mounted in a way that the magnetometer is not aligned with the north. When entering CLI mode, the tracker always attempts to point to 0 degrees (North). If it doesn't do so correctly, and you've already adjusted the pan0 and PIDs and calibrated the magnetometer, it's possible that the OFFSET parameter needs to be adjusted.

If you mounted the controller with the magnetometer aligned with the North, you may not need to touch this parameter unless you want to fine-tune it further. But if you mounted the controller in a different position, perhaps to have the micro USB connector on the side of the tracker box, then you need to adjust this parameter. For example, if you rotated the controller by 90 degrees:

1. Enter CLI mode
2. Execute the command `set offset=90`
3. Save the settings with `save`

We still need to adjust the PIDs.

[<< Go back](README.md)
