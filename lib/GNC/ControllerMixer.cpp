/**********************************************************************
 CONTROL MIXING CONCEPT

 Roll control  → differential brake
 Pitch control → symmetric brake

 Mixing equations:

    Left  = Pitch_cmd + Roll_cmd
    Right = Pitch_cmd - Roll_cmd

 Why this works:

    If Roll_cmd = 0:
        Left  = Pitch
        Right = Pitch
        → Both pull equally → symmetric brake → pitch control

    If Pitch_cmd = 0:
        Left  = +Roll
        Right = -Roll
        → One pulls more than the other → turning

    If both active:
        Pitch sets baseline brake
        Roll adds steering offset

**********************************************************************/
