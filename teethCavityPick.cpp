/////////////////////////////////////////////////////////////////////////////
// OpenHaptics QuickHaptics - TeethCavityPick example
// SensAble Technologies, Woburn, MA
// November 11, 2008
// Programmer: Venkat Gourishankar
//////////////////////////////////////////////////////////////////////////////
#include <QHHeadersGLUT.h>  //Include all necessary headers

#include <cstdio>
#include <vector>

///
#include <cmath>
#include <eigen3/Eigen/Geometry>

class DataTransportClass  // This class carried data into the ServoLoop thread
{
   public:
    TriMesh* c1;
    TriMesh* c2;
};
double chargeRadius = 2.0;  // This variable defines the radius around the charge when the inverse square law changes to a spring force law.
hduMatrix WorldToDevice;    // This matrix contains the World Space to DeviceSpace Transformation
hduVector3Dd forceVec;      // This variable contains the force vector.

// Callback functions
void button1DownCallback(unsigned int ShapeID);
void button1UpCallback(unsigned int ShapeID);
void touchCallback(unsigned int ShapeID);

void graphicsCallback(void);

void HLCALLBACK computeForceCB(HDdouble force[3], HLcache* cache, void* userdata);                    // Servo loop callback
void HLCALLBACK startEffectCB(HLcache* cache, void* userdata);                                        // Servo Loop callback
void HLCALLBACK stopEffectCB(HLcache* cache, void* userdata);                                         // Servo Loop callback
hduVector3Dd forceField(hduVector3Dd Pos1, hduVector3Dd Pos2, HDdouble Multiplier, HLdouble Radius);  // This function computer the force beween the Model and the particle based on the positions

// Global state for directing callback function behavior
bool button1Down;
bool cursorMoving;
bool draggingGumModel;
bool draggingTeethModel;
bool draggingCavityFillModel;
bool draggingCavityModel;

TriMesh* gDentureGum = NULL;
TriMesh* gDentureTeeth = NULL;
TriMesh* gDentureCavityFill = NULL;
TriMesh* gDentureCavity = NULL;

Box* gStartButton = NULL;
Box* gResetButton = NULL;

Text* gStartupMsg = NULL;
Text* gResetMsg = NULL;
Text* gInstructionMsg = NULL;
Text* gSuccessMsg = NULL;

int main(int argc, char* argv[]) {
    QHGLUT* DisplayObject = new QHGLUT(argc, argv);  // create a display window

    DeviceSpace* deviceSpace = new DeviceSpace;  // Find a Phantom device named "Default PHANToM"
    DisplayObject->tell(deviceSpace);            // tell Quickhaptics that Omni exists
    DisplayObject->setBackgroundColor(0.0, 0.0, 0.6);

    DisplayObject->setHapticWorkspace(hduVector3Dd(-40, -40.0, -17.0), hduVector3Dd(95, 45.0, 17.0));

    DataTransportClass dataObject;  // Initialize an Object to transport data into the servoloop callback

    // Load cube1 model
    dataObject.c1 = new TriMesh("models/TeethCavityPickModels/cube.obj");
    gDentureGum = dataObject.c1;
    dataObject.c1->setName("cube");
    dataObject.c1->setShapeColor(1.0, 0.5, 0.65);
    dataObject.c1->setRotation(hduVector3Dd(1.0, 0.0, 0.0), 45.0);
    dataObject.c1->setStiffness(0.6);
    dataObject.c1->setDamping(0.1);
    dataObject.c1->setFriction(0.5, 0.9);
    DisplayObject->tell(dataObject.c1);  // Tell quickhaptics that cube exists

    // Load cube2 model
    dataObject.c2 = new TriMesh("models/TeethCavityPickModels/cube.obj");
    gDentureGum = dataObject.c1;
    dataObject.c2->setName("cube");
    dataObject.c2->setShapeColor(0.1, 0.5, 0.65);
    dataObject.c2->setRotation(hduVector3Dd(1.0, 0.0, 0.0), 45.0);
    dataObject.c2->setTranslation(hduVector3Dd(25.0, 0.0, 0.0));
    dataObject.c2->setStiffness(0.6);
    dataObject.c2->setDamping(0.1);
    dataObject.c2->setFriction(0.5, 0.9);
    DisplayObject->tell(dataObject.c2);  // Tell quickhaptics that cube exists

    // Load gums model
    TriMesh* tm = new TriMesh("models/TeethCavityPickModels/dentures-gums.obj");
    gDentureGum = tm;

    tm->setName("dentureGum");
    tm->setShapeColor(1.0, 0.5, 0.65);
    tm->setRotation(hduVector3Dd(1.0, 0.0, 0.0), 45.0);
    tm->setStiffness(0.5);
    tm->setDamping(0.6);
    tm->setFriction(0.3, 0.0);
    DisplayObject->tell(tm);  // Tell quickhaptics that gums exists

    // Load teeth model
    tm = new TriMesh("models/TeethCavityPickModels/dentures-teeth.obj");
    gDentureTeeth = tm;

    tm->setName("dentureTeeth");
    tm->setRotation(hduVector3Dd(1.0, 0.0, 0.0), 45.0);
    tm->setStiffness(1.0);
    tm->setDamping(0.0);
    tm->setFriction(0.0, 0.2);
    DisplayObject->tell(tm);

    // Load cavity model
    tm = new TriMesh("models/TeethCavityPickModels/dentures-cavity fill.obj");
    gDentureCavityFill = tm;
    tm->setName("dentureCavityFill");
    tm->setRotation(hduVector3Dd(1.0, 0.0, 0.0), 45.0);
    tm->setPopthrough(0.5);
    tm->setStiffness(0.6);
    tm->setDamping(0.3);
    tm->setFriction(0.5, 0.4);
    DisplayObject->tell(tm);

    // Load cavity "target"
    tm = new TriMesh("models/TeethCavityPickModels/dentures-marker.obj");
    gDentureCavity = tm;

    tm->setName("dentureCavity");
    tm->setUnDraggable();
    tm->setRotation(hduVector3Dd(1.0, 0.0, 0.0), 45.0);
    tm->setStiffness(0.2);
    tm->setDamping(0.4);
    tm->setFriction(0.0, 0.0);
    DisplayObject->tell(tm);

    // SensAble logo
    Plane* logoBox = new Plane(15, 9);
    logoBox->setTranslation(53.0, -27.0, 30.0);
    logoBox->setHapticVisibility(false);
    logoBox->setTexture("models/TeethCavityPickModels/sensableLogo.jpg");
    DisplayObject->tell(logoBox);

    // START button
    Box* box = gStartButton = new Box(20, 10, 10);
    gStartButton = box;

    box->setName("startButton");
    box->setUnDraggable();
    box->setTranslation(60.0, 20.0, 0.0);
    box->setRotation(hduVector3Dd(0.0, 1.0, 0.0), -15.0);
    box->setTexture("models/TeethCavityPickModels/start.jpg");
    DisplayObject->tell(box);

    // RESET button
    box = new Box(20, 10, 10);
    gResetButton = box;

    box->setName("resetButton");
    box->setUnDraggable();
    box->setTranslation(60.0, -5.0, 0.0);
    box->setRotation(hduVector3Dd(0.0, 1.0, 0.0), -15.0);
    box->setTexture("models/TeethCavityPickModels/reset.jpg");
    DisplayObject->tell(box);

    // Startup Message
    Text* text = new Text(20.0, "Please touch START & press button 1 to begin", 0.25, 0.9);
    gStartupMsg = text;

    text->setName("startupMsg");
    text->setShapeColor(0.0, 0.5, 0.75);
    text->setHapticVisibility(false);
    text->setGraphicVisibility(true);
    DisplayObject->tell(text);

    // Reset Message
    text = new Text(20.0, "Please touch RESET and press button 1 to Reset the demo", 0.2, 0.85);
    gResetMsg = text;

    text->setName("resetMsg");
    text->setShapeColor(0.0, 0.5, 0.75);
    text->setHapticVisibility(false);
    text->setGraphicVisibility(false);
    DisplayObject->tell(text);

    // Instruction Message
    text = new Text(20.0, "Please locate the cavity by probing the teeth", 0.25, 0.9);
    gInstructionMsg = text;

    text->setName("instructionMsg");
    text->setShapeColor(0.0, 0.5, 0.75);
    text->setHapticVisibility(false);
    text->setGraphicVisibility(false);
    DisplayObject->tell(text);

    // Success Message
    text = new Text(20.0, "OUCH!!&*! You have successfully located the cavity", 0.25, 0.9);
    gSuccessMsg = text;

    text->setName("successMsg");
    text->setShapeColor(1.0, 0.35, 0.5);
    text->setHapticVisibility(false);
    text->setGraphicVisibility(false);
    DisplayObject->tell(text);

    Cursor* OmniCursor = new Cursor("models/TeethCavityPickModels/dentalPick.obj");  // Load a cursor that looks like a dental pick
    TriMesh* cursorModel = OmniCursor->getTriMeshPointer();
    OmniCursor->setName("devCursor");  // Give it a name

    cursorModel->setShapeColor(0.35, 0.35, 0.35);
    OmniCursor->scaleCursor(0.007);
    OmniCursor->setRelativeShapeOrientation(0.0, 0.0, 1.0, -90.0);

    //    OmniCursor->debugCursor(); //Use this function the view the location of the proxy inside the Cursor mesh
    DisplayObject->tell(OmniCursor);  // Tell QuickHaptics that the cursor exists

    DisplayObject->preDrawCallback(graphicsCallback);
    deviceSpace->startServoLoopCallback(startEffectCB, computeForceCB, stopEffectCB, &dataObject);  // Register the servoloop callback

    deviceSpace->button1DownCallback(button1DownCallback, gResetButton);
    deviceSpace->button1DownCallback(button1DownCallback, gDentureGum);
    deviceSpace->button1DownCallback(button1DownCallback, gDentureTeeth);
    deviceSpace->button1DownCallback(button1DownCallback, gDentureCavityFill);
    deviceSpace->button1DownCallback(button1DownCallback, gStartButton);
    deviceSpace->touchCallback(touchCallback, gDentureCavity);

    deviceSpace->button1UpCallback(button1UpCallback);

    qhStart();  // Set everything in motion
    return 0;
}

void button1DownCallback(unsigned int ShapeID) {
    TriMesh* modelTouched = TriMesh::searchTriMesh(ShapeID);
    Box* buttonTouched = Box::searchBox(ShapeID);

    draggingGumModel = false;
    draggingTeethModel = false;
    draggingCavityFillModel = false;
    draggingCavityModel = false;

    if (modelTouched == gDentureGum) {
        draggingGumModel = true;

        gDentureTeeth->setHapticVisibility(false);
        gDentureCavityFill->setHapticVisibility(false);
        gDentureCavity->setHapticVisibility(false);
    } else if (modelTouched == gDentureTeeth) {
        draggingTeethModel = true;

        gDentureGum->setHapticVisibility(false);
        gDentureCavityFill->setHapticVisibility(false);
        gDentureCavity->setHapticVisibility(false);
    } else if (modelTouched == gDentureCavityFill) {
        draggingCavityFillModel = true;

        gDentureTeeth->setHapticVisibility(false);
        gDentureGum->setHapticVisibility(false);
        gDentureCavity->setHapticVisibility(false);
    }

    if (buttonTouched == gStartButton) {
        gStartupMsg->setGraphicVisibility(false);
        gInstructionMsg->setGraphicVisibility(true);
        gSuccessMsg->setGraphicVisibility(false);
        gResetMsg->setGraphicVisibility(false);
    } else if (buttonTouched == gResetButton) {
        gInstructionMsg->setGraphicVisibility(false);
        gSuccessMsg->setGraphicVisibility(false);
        gStartupMsg->setGraphicVisibility(true);
        gResetMsg->setGraphicVisibility(false);
    }
}

void button1UpCallback(unsigned int ShapeID) {
    draggingGumModel = false;
    draggingTeethModel = false;
    draggingCavityFillModel = false;

    gDentureGum->setHapticVisibility(true);
    gDentureTeeth->setHapticVisibility(true);
    gDentureCavityFill->setHapticVisibility(true);
    gDentureCavity->setHapticVisibility(true);
}

void touchCallback(unsigned int ShapeID) {
    TriMesh* modelTouched = TriMesh::searchTriMesh(ShapeID);

    if (modelTouched == gDentureCavity) {
        gSuccessMsg->setGraphicVisibility(true);
        gStartupMsg->setGraphicVisibility(false);
        gInstructionMsg->setGraphicVisibility(false);
        gResetMsg->setGraphicVisibility(true);

        gDentureCavityFill->setHapticVisibility(false);
    } else {
        gDentureCavityFill->setHapticVisibility(true);
        gDentureTeeth->setHapticVisibility(true);
        gDentureGum->setHapticVisibility(true);
    }
}

void graphicsCallback() {
    hduMatrix globalDragTransform;

    /////////////////////////////////////////////////////////////////////////////////////////////// getting cursor position
    Cursor* localDeviceCursor = Cursor::searchCursor("devCursor");  // Get a pointer to the cursor
    hduVector3Dd localCursorPosition;
    localCursorPosition = localDeviceCursor->getPosition();  // Get the local cursor position in World Space
    printf("--------------------------------------------------------- %f, %f, %f\n", localCursorPosition[0], localCursorPosition[1], localCursorPosition[2]);
    ///////////////////////////////////////////////////////////////////////////////////////////////

    if (draggingGumModel) {
        globalDragTransform = gDentureGum->getTransform();

        gDentureCavity->setTransform(globalDragTransform);
        gDentureTeeth->setTransform(globalDragTransform);
        gDentureCavityFill->setTransform(globalDragTransform);

    } else if (draggingTeethModel) {
        globalDragTransform = gDentureTeeth->getTransform();

        gDentureCavity->setTransform(globalDragTransform);
        gDentureGum->setTransform(globalDragTransform);
        gDentureCavityFill->setTransform(globalDragTransform);
    } else if (draggingCavityFillModel) {
        globalDragTransform = gDentureCavityFill->getTransform();

        gDentureCavity->setTransform(globalDragTransform);
        gDentureGum->setTransform(globalDragTransform);
        gDentureTeeth->setTransform(globalDragTransform);
    }
}

/***************************************************************************************
 Servo loop thread callback.  Computes a force effect. This callback defines the motion
 of the purple skull and calculates the force based on the "real-time" Proxy position
 in Device space.
****************************************************************************************/
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache* cache, void* userdata) {
    DataTransportClass* localdataObject = (DataTransportClass*)userdata;  // Typecast the pointer passed in appropriately
    // hduVector3Dd skullPositionDS;                                         // Position of the skull (Moving sphere) in Device Space.
    // hduVector3Dd proxyPosition;                                           // Position of the proxy in device space

    HDdouble instRate = 0.0;
    HDdouble deltaT = 0.0;
    static float counter = 0.0;
    float degInRad = 0.0;
    static int counter1 = 0;

    // Get the time delta since the last update.
    hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
    deltaT = 1.0 / instRate;
    counter += deltaT;
    degInRad = counter * 20 * 3.14159 / 180;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////// transforming object
    hduVector3Dd ModelPos = localdataObject->c1->getTranslation();
    localdataObject->c1->setTranslation(-ModelPos);

    // Eigen::Vector3f p = (opti->rigidObjects)[1]->position;

    // ocurr_temp[0] = (double)p[0];
    // ocurr_temp[1] = (double)p[1];
    // ocurr_temp[2] = (double)p[2];

    // // transform optitrack coordinate into haptic system
    // Eigen::Vector3d opti_to_hapt(0., 0., 0.);
    // Eigen::Vector3d opti_double;

    // opti_double[0] = (double)p[0];
    // opti_double[1] = (double)p[1];
    // opti_double[2] = (double)p[2];

    // if (matrix_flag) {
    //     t->transformPointFromSysAtoSysB(opti_double, opti_to_hapt);
    //     std::cout << "----------Transformation Matrix----------" << std::endl;
    //     std::cout << t->transformMat << std::endl;
    //     std::cout << "----------Error----------" << std::endl;
    //     double ee = t->errorCalculation();
    //     std::cout << ee << std::endl;
    // }

    // HDdouble x = (HDdouble)(opti_to_hapt[0]);
    // HDdouble y = (HDdouble)(opti_to_hapt[2]);
    // HDdouble z = (HDdouble)(opti_to_hapt[1]);

    // double rX = (opti->rigidObjects)[1]->rotation[0] * 3.14159 / 180.0;
    // double rY = (opti->rigidObjects)[1]->rotation[1] * 3.14159 / 180.0;
    // double rZ = (opti->rigidObjects)[1]->rotation[2] * 3.14159 / 180.0;

    // hduMatrix rZM = hduMatrix::createRotationAroundZ(rZ);
    // hduMatrix rYM = hduMatrix::createRotationAroundY(rY);
    // hduMatrix rXM = hduMatrix::createRotationAroundX(rX);

    // hduMatrix rxyz = rZM * rYM * rXM;

    // localdataObject->Model->setTransform(rxyz);  // rotate the skull with the optitrack trackerocurr_temp

    // localdataObject->Model->setTranslation(x, z, y);  // move the skull with the optitrack tracker

    // localdataObject->Model->setScaleInPlace(0.3);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // WorldToDevice.multVecMatrix(localdataObject->Model->getTranslation(), skullPositionDS);  // Convert the position of the sphere from world space to device space

    // hlCacheGetDoublev(cache, HL_PROXY_POSITION, proxyPosition);  // Get the position of the proxy in Device Coordinates (All HL commands in the servo loop callback fetch values in device coordinates)

    forceVec = forceField(localdataObject->c1->getTranslation(), localdataObject->c2->getTranslation(), 40.0, 5.0);  // Calculate the force

    counter1++;
    if (counter1 > 2000)  // Make the force start after 2 seconds of program start. This is because the servo loop thread executes before the graphics thread.
                          // Hence global variables set in the graphics thread will not be valid for sometime in the begining og the program
    {
        force[0] = forceVec[0];
        force[1] = forceVec[1];
        force[2] = forceVec[2];
        counter1 = 2001;
    } else {
        force[0] = 0.0;
        force[1] = 0.0;
        force[2] = 0.0;
    }
}

/******************************************************************************
 Servo loop thread callback called when the effect is started.
******************************************************************************/
void HLCALLBACK startEffectCB(HLcache* cache, void* userdata) {
    DataTransportClass* localdataObject = (DataTransportClass*)userdata;
    printf("Custom effect started\n");
}

/******************************************************************************
 Servo loop thread callback called when the effect is stopped.
******************************************************************************/
void HLCALLBACK stopEffectCB(HLcache* cache, void* userdata) {
    printf("Custom effect stopped\n");
}

/*******************************************************************************
 Given the position of the two charges in space,
 calculates the (modified) coulomb force.
*******************************************************************************/
hduVector3Dd forceField(hduVector3Dd Pos1, hduVector3Dd Pos2, HDdouble Multiplier, HLdouble Radius) {
    hduVector3Dd diffVec = Pos2 - Pos1;  // Find the difference in position
    double dist = 0.0;
    hduVector3Dd forceVec(0, 0, 0);

    HDdouble nominalMaxContinuousForce;
    hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &nominalMaxContinuousForce);  // Find the max continuous for that the device is capable of

    dist = diffVec.magnitude();

    if (dist < Radius * 2.0)  // Spring force (when the model and cursor are within a 'sphere of influence'
    {
        diffVec.normalize();
        forceVec = (Multiplier)*diffVec * dist / (4.0 * Radius * Radius);
        static int i = 0;
    } else  // Inverse square attraction
    {
        forceVec = Multiplier * diffVec / (dist * dist);
    }

    for (int i = 0; i < 3; i++)  // Limit force calculated to Max continuouis. This is very important because force values exceeding this value can damage the device motors.
    {
        if (forceVec[i] > nominalMaxContinuousForce)
            forceVec[i] = nominalMaxContinuousForce;

        if (forceVec[i] < -nominalMaxContinuousForce)
            forceVec[i] = -nominalMaxContinuousForce;
    }

    return forceVec;
}
