# Robot Control and Motor Assignments

## **Drive Controller (Pilot)**

| PS5 Button                     | Assigned Function  | Motor Type | Motor ID   |
| ------------------------------ | ------------------ | ---------- | ---------- |
| X (Cross)                      |                    |            |            |
| O (Circle)                     |                    |            |            |
| △ (Triangle)                   |                    |            |            |
| □ (Square)                     |                    |            |            |
| L1                             |                    |            |            |
| L2                             |  for Slow/FastMode |            |            |
| R1                             |                    |            |            |
| R2                             |                    |            |            |
| L3 (Left Stick Click)          | Move Swerve        | Kraken     | 2,4,6,8    |
| R3 (Right Stick Click)         | Turn Swerve        | Kraken     | 1, 3, 5, 7 |
| D-Pad Up                       |                    |            |            |
| D-Pad Down                     |                    |            |            |
| D-Pad Left                     |                    |            |            |
| D-Pad Right                    |                    |            |            |
| Options Button + Create Button |                    |            |            |

---

## **Mech Controller (Operator)**

| PS5 Button                     | Assigned Mech Function | Motor Type | Motor ID |
| ------------------------------ | ---------------------- | ---------- | -------- |
| X (Cross)                      | Pivot In , pivot out   | Neo        | 14       |
| O (Circle)                     | Intake Coral           | Neo        | 16       |
| △ (Triangle)                   | Outtake Coral          | Neo        | 16       |
| □ (Square)                     |                        |            |          |
| L1                             | Rollers Outtake        | Neo        | 16       |
| R1                             | Rollers Intake         | Neo        | 16       |
| L2                             | Elevator Up            | Kraken     | 13       |
| R2                             | Elevator Down          | Kraken     | 13       |
| L3 (Left Stick Click)          |                        |            |          |
| R3 (Right Stick Click)         |                        |            |          |
| D-Pad Up                       | Stage 1 Elevator       | Kraken     | 13       |
| D-Pad Down                     | Stage 4 Elevator       | Kraken     | 13       |
| D-Pad Left                     | Stage 3 Elevator       | Kraken     | 13       |
| D-Pad Right                    | Stage 2 Elevator       | Kraken     | 13       |
| Options Button + Create Button | Deepclimb              | Neo        | 9, 10    |
|                                |                        |            |          |

---

## **Motor Assignments**

| Component    | Motor Type     | Position | Port | Status  |
| ------------ | -------------- | -------- | ---- | ------- |
| **Swerve**   | Neo (Steer)    | FL       | 1    | FLASHED |
|              |                | FR       | 3    | FLASHED |
|              |                | BR       | 5    | FLASHED |
|              |                | BL       | 7    | FLASHED |
|              | Kraken (Drive) | FL       | 2    | FLASHED |
|              |                | FR       | 4    | FLASHED |
|              |                | BL       | 6    | FLASHED |
|              |                | BR       | 8    | FLASHED |
| **Climb**    | Neo            | TOP      | 9    | FLASHED |
|              |                | BOTTOM   | 10   | FLASHED |
| **Elevator** | Kraken         | -        | 13   | FLASHED |
| **Pivot**    | Neo            | -        | 14   | FLASHED |
| **Intake**   | Neo            | -        | 16   | FLASHED |
|              |                |          |      |         |
