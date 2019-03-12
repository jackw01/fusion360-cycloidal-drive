import adsk.core
import adsk.fusion
import traceback
import math

# All units cm

# Rotor extrude thickness
rotorThickness = 0.4 
# Rotor nominal diameter
rotorDiameter = 3.4
# Rotor bearing hole diameter
rotorBearingHoleDiameter = 1.5875

# Camshaft cam diameter
camDiameter = 0.635
# Camshaft face diameter
camshaftDiameter = 1
# Camshaft reinforcement screw diameter
camshaftScrewHoleDiameter = 0.52

# Ring gear extrude thickness
ringGearThickness = rotorThickness 
# Ring gear outer diameter
ringGearOuterDiameter = rotorDiameter + 0.5
# Ring gear margin around nominal rotor diameter
ringGearMargin = 0.02
# Number of pins in ring gear
ringGearPins = 25
# Number of lobes on cycloidal rotor
rotorLobes = ringGearPins - 1
# Ring gear pin radius (ideally circumference / n_pins / 4)
ringGearPinRadius = rotorDiameter * math.pi / ringGearPins / 4
# Eccentric offset
eccentricOffset = 0.5 * ringGearPinRadius

# Rotor output hole diameter
outputHoleDiameter = 0.4
# Rotor output hole count
outputHoleCount = 6
# Rotor output hole circle diameter
outputCircleDiameter = (rotorDiameter + rotorBearingHoleDiameter) / 2 - ringGearPinRadius

# Output pin diameter
outputPinDiameter = outputHoleDiameter - ringGearPinRadius;
# Output body thickness
outputPlateThickness = 0.3

def getPoint(theta, rMajor, rMinor, e, n):
  psi = math.atan2(math.sin((1 - n) * theta), ((rMajor / (e * n)) - math.cos((1 - n) * theta)))
  x = (rMajor * math.cos(theta)) - (rMinor * math.cos(theta + psi)) - (e * math.cos(n * theta))
  y = (-rMajor * math.sin(theta)) + (rMinor * math.sin(theta + psi)) + (e * math.sin(n * theta))
  return (x, y)

def distance(xa, ya, xb, yb):
  return math.hypot((xa - xb), (ya - yb))

def run(context):
  ui = None

  try:
    rotorRadius = rotorDiameter / 2
    maxDist = 0.25 * ringGearPinRadius  # Maximum allowed distance between points
    minDist = 0.5 * maxDist # Minimum allowed distance between points

    app = adsk.core.Application.get()
    ui = app.userInterface
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent

    # Section: rotor
    rotorOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    rotor = rotorOcc.component
    rotor.name = 'Rotor'

    sk = rotor.sketches.add(root.xYConstructionPlane)
    points = adsk.core.ObjectCollection.create()

    ui.messageBox('Ratio: ' + str((ringGearPins - rotorLobes) / rotorLobes))

    (xs, ys) = getPoint(0, rotorRadius, ringGearPinRadius, eccentricOffset, ringGearPins)
    points.add(adsk.core.Point3D.create(xs, ys, 0))

    et = 2 * math.pi / rotorLobes
    (xe, ye) = getPoint(et, rotorRadius, ringGearPinRadius, eccentricOffset, ringGearPins)
    x = xs
    y = ys
    dist = 0
    ct = 0
    dt = math.pi / ringGearPins
    numPoints = 0

    while ((distance(x, y, xe, ye) > maxDist or ct < et / 2) and ct < et):
      (xt, yt) = getPoint(ct+dt, rotorRadius, ringGearPinRadius, eccentricOffset, ringGearPins)
      dist = distance(x, y, xt, yt)

      ddt = dt / 2
      lastTooBig = False
      lastTooSmall = False

      while (dist > maxDist or dist < minDist):
        if (dist > maxDist):
          if (lastTooSmall):
            ddt /= 2

          lastTooSmall = False
          lastTooBig = True

          if (ddt > dt / 2):
            ddt = dt / 2

          dt -= ddt

        elif (dist < minDist):
          if (lastTooBig):
            ddt /= 2

          lastTooSmall = True
          lastTooBig = False
          dt += ddt

        (xt, yt) = getPoint(ct + dt, rotorRadius, ringGearPinRadius, eccentricOffset, ringGearPins)
        dist = distance(x, y, xt, yt)

      x = xt
      y = yt
      points.add(adsk.core.Point3D.create(x, y, 0))
      numPoints += 1
      ct += dt

    points.add(adsk.core.Point3D.create(xe, ye, 0))
    curve = sk.sketchCurves.sketchFittedSplines.add(points)

    lines = sk.sketchCurves.sketchLines
    line1 = lines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), curve.startSketchPoint)
    line2 = lines.addByTwoPoints(line1.startSketchPoint, curve.endSketchPoint)

    # Extrude
    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(rotorThickness)
    extrudes = rotor.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    # Create component
    body1 = extrude.bodies.item(0)
    body1.name = "Rotor"
    inputEntities = adsk.core.ObjectCollection.create()
    inputEntities.add(body1)

    # Circular pattern
    zAxis = rotor.zConstructionAxis
    circularFeats = rotor.features.circularPatternFeatures
    circularFeatInput = circularFeats.createInput(inputEntities, zAxis)
    circularFeatInput.quantity = adsk.core.ValueInput.createByReal(rotorLobes)
    circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    circularFeatInput.isSymmetric = True
    circularFeat = circularFeats.add(circularFeatInput)

    # Combine pattern features
    ToolBodies = adsk.core.ObjectCollection.create()
    for b in circularFeat.bodies:
      ToolBodies.add(b)

    combineInput = rotor.features.combineFeatures.createInput(body1, ToolBodies)
    combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    combineInput.isNewComponent = False
    rotor.features.combineFeatures.add(combineInput)

    # Center bearing hole
    sk = rotor.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), rotorBearingHoleDiameter / 2)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(rotorThickness)
    extrudes = rotor.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.CutFeatureOperation)

    # Output holes
    sk = rotor.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, outputCircleDiameter / 2, 0), outputHoleDiameter / 2)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(rotorThickness)
    extrudes = rotor.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.CutFeatureOperation)

    inputEntities = adsk.core.ObjectCollection.create()
    inputEntities.add(extrude)

    # Circular pattern
    circularFeats = rotor.features.circularPatternFeatures
    circularFeatInput = circularFeats.createInput(inputEntities, zAxis)
    circularFeatInput.quantity = adsk.core.ValueInput.createByReal(outputHoleCount)
    circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    circularFeatInput.isSymmetric = True
    circularFeat = circularFeats.add(circularFeatInput)

    # Offset the rotor to make the ring gear concentric with origin
    transform = rotorOcc.transform
    transform.translation = adsk.core.Vector3D.create(eccentricOffset, 0, 0)
    rotorOcc.transform = transform
    des.snapshots.add()

    # Section: camshaft
    camshaftOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    camshaft = camshaftOcc.component
    camshaft.name = 'Camshaft'

    # Cam
    sk = camshaft.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(eccentricOffset, 0, 0), camDiameter / 2)
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(eccentricOffset, 0, 0), camshaftScrewHoleDiameter / 2)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(rotorThickness)
    extrudes = camshaft.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    # Section: output assembly
    outputOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    output = outputOcc.component
    output.name = 'Output'

    # Output pins
    sk = output.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, outputCircleDiameter / 2, 0), outputPinDiameter / 2)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(rotorThickness)
    extrudes = output.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    inputEntities = adsk.core.ObjectCollection.create()
    inputEntities.add(extrude)

    # Circular pattern
    zAxis = output.zConstructionAxis
    circularFeats = output.features.circularPatternFeatures
    circularFeatInput = circularFeats.createInput(inputEntities, zAxis)
    circularFeatInput.quantity = adsk.core.ValueInput.createByReal(outputHoleCount)
    circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    circularFeatInput.isSymmetric = True
    circularFeat = circularFeats.add(circularFeatInput)

    # Output body
    sk = output.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), outputCircleDiameter / 2 + outputPinDiameter)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(-outputPlateThickness)
    extrudes = output.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.JoinFeatureOperation)

    # Section: ring gear
    ringGearOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    ringGear = ringGearOcc.component
    ringGear.name = 'Ring Gear'

    # Pins
    sk = ringGear.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    centerPoint = adsk.core.Point3D.create(rotorRadius, 0, 0)
    sketchCircles.addByCenterRadius(centerPoint, ringGearPinRadius)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(ringGearThickness)
    extrudes = ringGear.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    
    pin = extrude.bodies.item(0)
    pin.name = "Pin"
    inputEntities = adsk.core.ObjectCollection.create()
    inputEntities.add(pin)

    # Circular pattern
    zAxis = ringGear.zConstructionAxis
    circularFeats = ringGear.features.circularPatternFeatures
    circularFeatInput = circularFeats.createInput(inputEntities, zAxis)
    circularFeatInput.quantity = adsk.core.ValueInput.createByReal(ringGearPins)
    circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    circularFeatInput.isSymmetric = True
    circularFeat = circularFeats.add(circularFeatInput)

    # Housing
    sk = ringGear.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), rotorRadius + ringGearMargin)
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), ringGearOuterDiameter / 2)

    prof = sk.profiles.item(1)
    dist = adsk.core.ValueInput.createByReal(ringGearThickness)
    extrudes = ringGear.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    return

  except:
    if ui:
      ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
