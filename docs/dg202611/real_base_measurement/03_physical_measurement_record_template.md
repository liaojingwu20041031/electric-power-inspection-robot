# 03 — Physical Measurement Record Template

Fill this in by hand. Record **raw readings**, not pre-averaged values — the
arithmetic and the confidence rating are derived afterwards from what you wrote.

If a field does not apply or is unknown, write `UNKNOWN`. Do not leave it blank and
do not guess.

---

## Session

```
MEASUREMENT_DATE          
OPERATOR                  
VEHICLE_CONFIGURATION     
GROUND_SURFACE            
PAYLOAD_STATE             
TIRE_STATE                
MEASUREMENT_TOOL          
TOOL_RESOLUTION           
DRIVE_POWER_DISCONNECTED  YES / NO
```

Notes on what these mean:

- `VEHICLE_CONFIGURATION` — what was mounted. Sensors, mast, battery, anything
  that changes load distribution.
- `PAYLOAD_STATE` — empty, or what was carried. Load changes loaded radius.
- `TIRE_STATE` — solid / pneumatic, and if pneumatic, the pressure. Also note
  visible wear or flat spots.
- `TOOL_RESOLUTION` — the smallest division you can actually read, e.g. `1 mm`.
  This bounds the precision of everything below.

---

## wheel_track

Method: measure outer-face-to-outer-face `O` and inner-face-to-inner-face `I`.
Reposition the vehicle between trials.

| trial | O (m) | I (m) | track = (O+I)/2 | tread = (O−I)/2 | notes |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
| 4 (optional) | | | | | |
| 5 (optional) | | | | | |

```
TRACK_MEAN              
TRACK_RANGE             (max − min)
TREAD_MEAN              
TREAD_CONSISTENT        YES / NO
REPOSITIONED_BETWEEN_TRIALS  YES / NO
PHOTO_REFERENCE         
NOTES                   
```

`TREAD_CONSISTENT` is a cross-check, not a deliverable: the derived tread width
should be roughly the same in every trial. If it varies a lot, one of the two face
readings is unreliable and the track values inherit that error.

---

## wheel_radius — Level A, geometric loaded radius

Record left and right **separately**. State the method used.

```
METHOD   AXLE_CENTRE_TO_GROUND  /  OVERALL_DIAMETER_HALVED
```

### Left wheel

| trial | reading (m) | notes |
|---|---|---|
| 1 | | |
| 2 | | |
| 3 | | |

```
LEFT_MEAN     
LEFT_RANGE    
```

### Right wheel

| trial | reading (m) | notes |
|---|---|---|
| 1 | | |
| 2 | | |
| 3 | | |

```
RIGHT_MEAN            
RIGHT_RANGE           
LEFT_RIGHT_DIFFERENCE 
PHOTO_REFERENCE       
NOTES                 
```

---

## wheel_radius — Level B, effective rolling radius

Leave entirely blank unless Level B has been separately authorised.

```
LEVEL_B_AUTHORISED   YES / NO
METHOD               B1_MANUAL_PUSH  /  B2_ENCODER
```

| run | distance D (m) | revolutions N | side | r = D/(2πN) | notes |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |

```
ROLLING_RADIUS_MEAN       
ROLLING_RADIUS_RANGE      
STRAIGHT_LINE_CONFIRMED   YES / NO
WHOLE_REVOLUTIONS_ONLY    YES / NO
PHOTO_REFERENCE           
NOTES                     
```

---

## Anything unexpected

Record anything that did not match expectation, even if it seems minor. A wheel
that will not settle straight, a visible camber, a wobble, an uneven contact patch,
or a left/right difference are all more informative than the averages.

```
OBSERVATIONS   
```

---

## Do not fill these in

The confidence rating and the freeze decision are derived from the readings above
by the criteria in `05_parameter_freeze_and_verification_plan.md`. Leave them to
that step rather than judging on the spot.

```
PHYSICAL_MEASUREMENT_CONFIDENCE   (derived later)
FREEZE_RECOMMENDED                (derived later)
```
